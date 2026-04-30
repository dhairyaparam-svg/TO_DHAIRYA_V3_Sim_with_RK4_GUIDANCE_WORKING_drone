"""
Atmosphere & Environment Models for Quadrotor SMC-IO Guidance
=============================================================

Provides:
  1. ISA standard-atmosphere air density  ρ(h)
  2. Aerodynamic drag acceleration
  3. Wind model (constant base + sinusoidal gust)
  4. Obstacle field with repulsive penalty

Author : Auto-generated from SMC-IO asteroid project adaptation
"""

import numpy as np

# =============================================================================
# ISA Standard Atmosphere  (troposphere 0-11 km, stratosphere 11-25 km)
# =============================================================================

# Sea-level reference values
RHO_0   = 1.225        # kg/m³   (sea-level density)
T_0     = 288.15       # K       (sea-level temperature)
P_0     = 101325.0     # Pa      (sea-level pressure)
LAPSE   = 0.0065       # K/m     (temperature lapse rate in troposphere)
G_CONST = 9.80665      # m/s²
R_AIR   = 287.058      # J/(kg·K)  specific gas constant for dry air


def isa_temperature(alt):
    """
    ISA temperature at altitude (m).
    Troposphere: T = T0 - L*h   (0 ≤ h ≤ 11000 m)
    Stratosphere: T = 216.65 K   (11000 < h ≤ 25000 m)
    """
    if alt <= 11000.0:
        return T_0 - LAPSE * alt
    else:
        return 216.65


def isa_density(alt):
    """
    ISA air density at geometric altitude *alt* (m).

    Returns ρ in kg/m³.  Valid for 0 ≤ alt ≤ 25 000 m.
    Uses the hydrostatic / perfect-gas approximation.

    >>> round(isa_density(0), 3)
    1.225
    >>> round(isa_density(1000), 3)
    1.112
    """
    alt = max(alt, 0.0)

    if alt <= 11000.0:
        # Troposphere (temperature lapse)
        T = T_0 - LAPSE * alt
        return RHO_0 * (T / T_0) ** (G_CONST / (LAPSE * R_AIR) - 1.0)
    else:
        # Stratosphere (isothermal layer at 216.65 K)
        # First get density at tropopause (11 km)
        T_trop = 216.65
        rho_trop = RHO_0 * (T_trop / T_0) ** (G_CONST / (LAPSE * R_AIR) - 1.0)
        return rho_trop * np.exp(-G_CONST * (alt - 11000.0) / (R_AIR * T_trop))


# =============================================================================
# Aerodynamic Drag
# =============================================================================

def drag_acceleration(vel_air, alt, Cd, A, m):
    """
    Aerodynamic drag acceleration vector (opposes airspeed).

    Parameters
    ----------
    vel_air : ndarray (3,)
        Airspeed vector  v_air = v_inertial − v_wind   (m/s)
    alt : float
        Altitude above sea level (m)
    Cd : float
        Drag coefficient (dimensionless)
    A : float
        Reference frontal area (m²)
    m : float
        Vehicle mass (kg)

    Returns
    -------
    a_drag : ndarray (3,)
        Drag acceleration  (m/s²), opposing the airspeed direction
    """
    rho = isa_density(alt)
    v_mag = np.linalg.norm(vel_air)
    if v_mag < 1e-12:
        return np.zeros(3)
    # F_drag = 0.5 * ρ * Cd * A * |v|²  in the −v direction
    a_drag = -0.5 * rho * Cd * A * v_mag * vel_air / m
    return a_drag


# =============================================================================
# Wind Model
# =============================================================================

def wind_model(t, pos, wind_params):
    """
    Wind velocity vector at position *pos* and time *t*.

    The model combines:
      • Constant base wind  (e.g. prevailing crosswind)
      • Sinusoidal gust with altitude-dependent amplitude

    Parameters
    ----------
    t : float
        Simulation time (s)
    pos : ndarray (3,)
        Position [x, y, z] (m) — z is altitude
    wind_params : dict
        Keys:
          "base"      : ndarray (3,)  constant wind [wx, wy, wz] (m/s)
          "gust_amp"  : float         gust amplitude at sea level (m/s)
          "gust_freq" : float         gust angular frequency (rad/s)
          "gust_dir"  : ndarray (3,)  unit direction of gust oscillation
          "alt_scale"  : float        altitude scale for amplitude growth (m)

    Returns
    -------
    v_wind : ndarray (3,)
        Wind velocity vector (m/s)
    """
    base = wind_params.get("base", np.zeros(3))
    gust_amp  = wind_params.get("gust_amp", 0.0)
    gust_freq = wind_params.get("gust_freq", 0.5)
    gust_dir  = wind_params.get("gust_dir", np.array([1.0, 0.0, 0.0]))
    alt_scale = wind_params.get("alt_scale", 100.0)

    alt = max(pos[2], 0.0)
    # Gust amplitude increases with sqrt(1 + h/h_scale)
    amp = gust_amp * np.sqrt(1.0 + alt / alt_scale)
    gust = amp * np.sin(gust_freq * t) * gust_dir

    return base + gust


# =============================================================================
# Obstacle Field
# =============================================================================

def make_obstacle(centre, radius):
    """Create an obstacle dict with centre (3,) and radius (float)."""
    return {"centre": np.array(centre, dtype=float), "radius": float(radius)}


def obstacle_vortex_guidance(pos, vel, target_pos, obstacles,
                             D_start=8.0, D_keep=2.0, k_rep=50.0,
                             max_force=None):
    """
    3-D vortex obstacle-guidance model  (Wang & Xiang, CAC 2022 — adapted).

    Core idea
    ---------
    Instead of a purely *radial* repulsive force (classic APF), each obstacle
    generates a *tangential* guidance acceleration that steers the vehicle
    *around* the obstacle toward the target.  The direction is computed by
    projecting the global-path unit vector (drone → target) onto the tangent
    plane of the obstacle sphere — this is the direction that simultaneously
    avoids the sphere *and* makes maximum progress toward the goal.

    This eliminates the two main failures of APF:
      • Local minima  (obstacle directly in path → APF stalls, vortex steers around)
      • Large abrupt turns  (APF repulsion is purely radial; vortex blends smoothly
                             between "go forward" and "slide around the sphere")

    Zones  (D = clearance from obstacle surface)
    --------------------------------------------
    D > D_start       : no influence
    D_keep ≤ D ≤ D_start : soft-avoidance zone — guidance direction blends from
                           global direction (at D_start) to tangential (at D_keep);
                           quadratic APF magnitude grows as drone approaches
    D < D_keep        : hard-avoidance zone — strong radial repulsion plus
                           tangential slide to move drone along the sphere surface

    Multi-obstacle handling
    -----------------------
    Forces are *summed* so that two simultaneous threats both act.
    Each obstacle is weighted by 1/D² (closer = more urgent, eq. 13 in paper).
    An obstacle whose surface the drone is already moving *away from* receives
    zero weight (drone is escaping — no counterproductive push).

    Parameters
    ----------
    pos        : (3,)  current position          [m]
    vel        : (3,)  current velocity          [m/s]
    target_pos : (3,)  target / destination      [m]
    obstacles  : list of {"centre": (3,), "radius": float}
    D_start    : float  distance from surface where influence begins  [m]
    D_keep     : float  desired minimum clearance from surface         [m]
    k_rep      : float  overall force gain

    Returns
    -------
    a_total : (3,)  avoidance acceleration  [m/s²]
    """
    if not obstacles:
        return np.zeros(3)

    R      = target_pos - pos
    R_norm = np.linalg.norm(R)
    if R_norm < 1e-6:
        return np.zeros(3)
    beta_vec = R / R_norm          # unit vector: drone → target  (global direction)

    total_force = np.zeros(3)

    for obs in obstacles:
        centre = np.asarray(obs["centre"], dtype=float)
        radius = float(obs.get("radius", 1.0))

        diff = pos - centre            # vector: obstacle centre → drone
        dist = np.linalg.norm(diff)
        if dist < 1e-6:
            continue

        n_hat = diff / dist            # outward unit normal (away from obstacle)
        D     = dist - radius          # clearance from obstacle surface

        if D > D_start:
            continue                   # outside influence zone — no effect

        # ------------------------------------------------------------------
        # Tangential guidance direction  (core of vortex model)
        #
        # Project global direction beta_vec onto the sphere tangent plane.
        # Result: the direction tangent to the sphere that points most toward
        # the target — steers around the obstacle without losing progress.
        # ------------------------------------------------------------------
        beta_n   = np.dot(beta_vec, n_hat)
        beta_tan = beta_vec - beta_n * n_hat   # tangential projection (un-normalised)
        t_norm   = np.linalg.norm(beta_tan)

        if t_norm < 1e-6:
            # beta_vec is radial: obstacle lies exactly on the drone-target line.
            # Pick the most upward perpendicular direction as tangential guide,
            # mirroring the paper's "left/right" vortex direction choice.
            up   = np.array([0., 0., 1.])
            perp = np.cross(n_hat, up)
            p_norm = np.linalg.norm(perp)
            if p_norm < 1e-6:          # n_hat is vertical — fall back to y-axis
                perp   = np.cross(n_hat, np.array([0., 1., 0.]))
                p_norm = np.linalg.norm(perp)
            beta_tan = perp / max(p_norm, 1e-6)
        else:
            beta_tan = beta_tan / t_norm

        # ------------------------------------------------------------------
        # Blending weight k  (Eq. 16 from Wang & Xiang 2022, adapted to 3-D)
        # k = 0  at D = D_start  →  follow global direction
        # k = 1  at D = D_keep   →  pure tangential guidance
        # ------------------------------------------------------------------
        D_clamped    = float(np.clip(D, D_keep, D_start))
        k            = (D_start - D_clamped) / (D_start - D_keep)

        g_raw        = (1.0 - k) * beta_vec + k * beta_tan
        g_norm       = np.linalg.norm(g_raw)
        guidance_dir = g_raw / g_norm if g_norm > 1e-6 else beta_tan

        # ------------------------------------------------------------------
        # Force magnitude
        # ------------------------------------------------------------------
        if D >= D_keep:
            # Soft zone: quadratic APF-style penalty from D_start inward
            pen   = 1.0 / max(D, 0.05) - 1.0 / D_start
            a_obs = k_rep * (pen ** 2) * guidance_dir
        else:
            # Hard zone: strong radial repulsion + tangential slide
            pen   = 1.0 / max(D, 0.05) - 1.0 / D_keep
            a_obs = k_rep * (pen ** 2) * (n_hat + beta_tan)

        # ------------------------------------------------------------------
        # Distance-squared weighting; zero if drone is already moving away
        # (eq. 13 paper spirit — don't fight the drone when it is escaping)
        # ------------------------------------------------------------------
        W          = 1.0 / max(D ** 2, 0.01)
        vel_toward = np.dot(vel, -n_hat)       # positive = approaching obstacle
        if vel_toward < 0.0 and D > D_keep:
            W = 0.0                            # already escaping — skip

        total_force += W * a_obs

    # Hard cap: obstacle force must never exceed max_force so that it cannot
    # consume the entire thrust budget and starve gravity compensation.
    if max_force is not None:
        f_norm = np.linalg.norm(total_force)
        if f_norm > max_force:
            total_force = total_force * (max_force / f_norm)

    return total_force

def obstacle_penalty(pos, obstacles, margin=2.0, k_rep=50.0):
    """Legacy alias — redirects to the vortex model with APF-compatible signature."""
    return obstacle_vortex_guidance(
        pos, np.zeros(3), pos + np.array([1., 0., 0.]),
        obstacles, D_start=margin, D_keep=margin * 0.25, k_rep=k_rep,
    )


def check_obstacle_violation(pos, obstacles):
    """
    Check if *pos* is inside any obstacle.

    Returns
    -------
    violated : bool
    min_clearance : float   (negative = inside obstacle)
    """
    min_clearance = np.inf
    for obs in obstacles:
        d = np.linalg.norm(pos - obs["centre"]) - obs["radius"]
        min_clearance = min(min_clearance, d)
    return min_clearance < 0, min_clearance

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


def obstacle_penalty(pos, obstacles, margin=0.5, k_rep=5):
    """
    Repulsive acceleration that pushes the vehicle away from obstacles.

    Uses an artificial-potential-field approach: when the vehicle enters
    the obstacle's influence zone (margin distance from the surface), a radial 
    repulsive force grows quadratically and approaches infinity at the surface.
    """
    a_rep = np.zeros(3)
    
    # Spherical obstacle repulsion
    for obs in obstacles:
        diff = pos - obs["centre"]
        dist = np.linalg.norm(diff)
        dist_to_surface = dist - obs["radius"]
        
        if dist_to_surface < margin:
            # If inside the obstacle, apply max repulsion to push it out
            if dist_to_surface <= 0.01:
                dist_to_surface = 0.01
                
            # Repulsion blows up as dist_to_surface -> 0
            penetration = 1.0 / dist_to_surface - 1.0 / margin
            direction = diff / dist if dist > 1e-6 else np.array([0., 0., 1.])
            a_rep += k_rep * (penetration**2) * direction
            
    return a_rep


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

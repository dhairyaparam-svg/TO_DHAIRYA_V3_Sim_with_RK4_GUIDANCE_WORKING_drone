"""
SMC-IO (Sliding-Mode-Control-based Instantaneously Optimal) Guidance
for Quadrotor Drone Trajectory Generation in Earth's Atmosphere

Adapted from the asteroid soft-landing guidance (Shincy & Ghosh, 2023).

New features vs. asteroid version:
  • Earth gravity  (constant g = 9.81 m/s² downward)
  • Aerodynamic drag  (ISA density × Cd × A)
  • Wind disturbances  (constant + sinusoidal gust)
  • Obstacle avoidance  (artificial-potential-field repulsive constraints)
  • Density variation with altitude  (ISA standard atmosphere)

Usage:
    python quadrotor_smc_guidance.py --scenario 1
    python quadrotor_smc_guidance.py --scenario 3 --mode standard
    python quadrotor_smc_guidance.py --mode compare
"""

import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Project-local imports
from atmosphere_model import (
    isa_density, drag_acceleration, wind_model,
    obstacle_penalty, check_obstacle_violation,
)
from quadrotor_scenarios import (
    build_params as scenarios_build_params,
    get_all_scenarios, get_scenario,
)

try:
    import plotly.graph_objects as go
    import plotly.subplots as sp
    PLOTLY_AVAILABLE = True
except ImportError:
    PLOTLY_AVAILABLE = False


# =============================================================================
# Environment Acceleration  (replaces gravity_model from asteroid code)
# =============================================================================

def environment_acceleration(pos, vel, t, params):
    """
    Total non-thrust environmental acceleration (gravity + drag only).
    Obstacle repulsion is handled separately to avoid destabilising SLSQP.
    """
    g = params["g_vec"].copy()

    # --- Wind → airspeed ---
    v_wind = wind_model(t, pos, params["wind"])
    v_air  = vel - v_wind

    # --- Drag ---
    alt = max(pos[2], 0.0)
    a_drag = drag_acceleration(v_air, alt, params["Cd"], params["A"], params["m0"])

    return g + a_drag


def obstacle_acceleration(pos, params):
    """Repulsive acceleration from obstacles (kept separate from SLSQP env model)."""
    if not params.get("obstacles"):
        return np.zeros(3)
    return obstacle_penalty(
        pos, params["obstacles"],
        margin=params.get("obstacle_margin", 0.5),
        k_rep=params.get("obstacle_k_rep", 1),
    )


# =============================================================================
# Velocity-profile helpers  (same structure as asteroid code Eqs. 26-32)
# =============================================================================

def velocity_profile_max(pos_next, params):
    """Maximum allowable velocity magnitude in each axis."""
    pos0 = params["pos0"]
    vel0 = params["vel0"]
    rfd  = params["rfd"]
    bx, by, bz = params["bx"], params["by"], params["bz"]
    cx, cy, cz = params["cx"], params["cy"], params["cz"]

    def _zeta(p_next_i, rfd_i, p0_i):
        denom = p0_i - rfd_i
        return (p_next_i - rfd_i) / denom if abs(denom) > 1e-12 else 0.0

    zx = _zeta(pos_next[0], rfd[0], pos0[0])
    zy = _zeta(pos_next[1], rfd[1], pos0[1])
    zz = _zeta(pos_next[2], rfd[2], pos0[2])

    vfd = params.get("vfd", np.zeros(3))
    
    # Calculate log10 shape envelope (tapers from 1 to 0 as distance z_i -> 0)
    shape_x = np.log10(max(1 + bx * zx + cx * zx**2, 1e-30))
    shape_y = np.log10(max(1 + by * zy + cy * zy**2, 1e-30))
    shape_z = np.log10(max(1 + bz * zz + cz * zz**2, 1e-30))

    # Envelope bounds taper down to abs(v_fd) rather than 0
    VxM = abs(vfd[0]) + abs(vel0[0] - vfd[0]) * shape_x
    VyM = abs(vfd[1]) + abs(vel0[1] - vfd[1]) * shape_y
    VzM = abs(vfd[2]) + abs(vel0[2] - vfd[2]) * shape_z

    # Clamp to a sensible minimum to avoid zero envelopes when vel0 ≈ 0
    v0_norm = np.linalg.norm(vel0)
    v_min = max(0.5, 0.1 * v0_norm)
    VxM = max(VxM, v_min)
    VyM = max(VyM, v_min)
    VzM = max(VzM, v_min)

    return VxM, VyM, VzM


def velocity_bounds(VxM, VyM, VzM, params):
    """Lower and upper velocity bounds per axis."""
    pos0 = params["pos0"]
    vel0 = params["vel0"]
    rfd  = params["rfd"]

    def _bound_1d(vi0, rifd, ri0, ViM):
        s_vel  = np.sign(vi0)
        s_disp = np.sign(rifd - ri0)
        if s_vel == s_disp:
            if s_vel >= 0:
                return (0.8 * ViM, 1.0 * ViM)
            else:
                return (-1.0 * ViM, -0.8 * ViM)
        else:
            return (-1.0 * ViM, 1.0 * ViM)

    bx_lo, bx_hi = _bound_1d(vel0[0], rfd[0], pos0[0], VxM)
    by_lo, by_hi = _bound_1d(vel0[1], rfd[1], pos0[1], VyM)
    bz_lo, bz_hi = _bound_1d(vel0[2], rfd[2], pos0[2], VzM)
    return (bx_lo, bx_hi), (by_lo, by_hi), (bz_lo, bz_hi)


# =============================================================================
# Heading-error sliding surface  (identical maths to asteroid code)
# =============================================================================

def heading_error_coefficients(r, v, rfd, g):
    """
    Compute alpha_0 … alpha_4 and sliding variable Se.
    See Eqs. (12-13) and (25) of Shincy & Ghosh 2023.
    """
    R     = rfd - r
    Rdot  = -v

    R_norm = np.linalg.norm(R)
    v_norm = np.linalg.norm(v)

    if R_norm < 1e-10 or v_norm < 1e-10:
        return 0.0, 0.0, 0.0, 0.0, 0.0

    Rv     = np.dot(R, v)
    cos_e  = np.clip(Rv / (R_norm * v_norm), -1.0, 1.0)
    Se     = np.arccos(cos_e)

    sin_e  = np.sqrt(max(1 - cos_e**2, 1e-30))
    if sin_e < 1e-12:
        sin_e = 1e-12
    K = -1.0 / ((R_norm * v_norm)**2 * sin_e)

    common = Rv * R_norm / v_norm

    alpha1 = K * (R_norm * v_norm * (rfd[0] - r[0]) - common * v[0])
    alpha2 = K * (R_norm * v_norm * (rfd[1] - r[1]) - common * v[1])
    alpha3 = K * (R_norm * v_norm * (rfd[2] - r[2]) - common * v[2])
    alpha4 = K * (
        R_norm * v_norm * np.dot(R, g)
        - (Rv * R_norm * np.dot(v, g)) / v_norm
        + R_norm * v_norm * np.dot(Rdot, v)
        - (Rv * v_norm * np.dot(R, Rdot)) / R_norm
    )

    return alpha1, alpha2, alpha3, alpha4, Se


def compute_distance_based_ee(r_norm, pos0_norm, Ee0, ee_function="quadratic_cubic",
                              rf_threshold=2.0, invert_ratio=False):
    """
    Distance-dependent sliding-mode reaching-law coefficient Ee.
    Same function set as the asteroid code, with a smaller rf_threshold
    appropriate for quadrotor scales.

    invert_ratio=True  : use r_norm / pos0_norm  (obstacle-avoidance scenarios —
                         Ee grows as the drone moves away from start, encouraging
                         stronger steering when far from home and near obstacles)
    invert_ratio=False : use pos0_norm / r_norm  (standard point-to-point —
                         Ee shrinks as the drone closes on the target)
    """
    if r_norm < rf_threshold:
        return 0.0

    #if invert_ratio:
        #r_ratio = r_norm / pos0_norm if pos0_norm > 1e-12 else 0.0
    #else:
        
    r_ratio = pos0_norm / r_norm if r_norm > 1e-12 else 0.0

    if ee_function == "linear":
        return Ee0 * r_ratio
    elif ee_function == "quadratic":
        return Ee0 * r_ratio**2
    elif ee_function == "cubic":
        return Ee0 * r_ratio**3
    elif ee_function == "exponential":
        return Ee0 * (np.exp(r_ratio) - 1.0)
    elif ee_function == "saturation":
        return Ee0
    else:  # "quadratic_cubic"
        if pos0_norm < 1e-12:
            return Ee0
        return (0.5 * (Ee0 / pos0_norm**2) * r_norm**2
                + 0.5 * (Ee0 / pos0_norm**3) * r_norm**3)


# =============================================================================
# Constraint builder and objective  (mirrors asteroid build_constraints)
# =============================================================================

def build_constraints(t, states, params):
    """
    Construct scipy constraints for the SLSQP guidance optimisation at one
    time step.  Constraints are:
      • Sliding-surface equality   (heading error reaching law)
      • Velocity upper/lower bounds per axis
      • Thrust magnitude limit
    """
    h  = params["dt"]
    r  = states[0:3]
    v  = states[3:6]
    m  = states[6]

    g = environment_acceleration(r, v, t, params)

    # Sliding-mode parameters
    Ee0 = params["Ee0"]
    ke  = params["ke"]
    ee_function = params.get("ee_function", "quadratic_cubic")

    r_norm    = np.linalg.norm(r - params["rfd"])
    pos0_norm = np.linalg.norm(params["pos0"] - params["rfd"])
    has_obstacles = bool(params.get("obstacles"))
    Ee = compute_distance_based_ee(r_norm, pos0_norm, Ee0, ee_function,
                                   rf_threshold=2.0, invert_ratio=has_obstacles)

    alpha1, alpha2, alpha3, alpha4, Se = heading_error_coefficients(
        r, v, params["rfd"], g
    )

    # If Se is essentially zero the vehicle is already heading straight at target
    if abs(Se) < 1e-10:
        alpha0 = 0.0
    else:
        alpha0 = -alpha4 - Ee * np.sign(Se) - ke * Se

    # Position at next time step (Euler approx for velocity envelope)
    pos_next = r + v * h

    # Velocity envelope
    VxM, VyM, VzM = velocity_profile_max(pos_next, params)
    (vx_lo, vx_hi), (vy_lo, vy_hi), (vz_lo, vz_hi) = velocity_bounds(
        VxM, VyM, VzM, params
    )

    Tmax = params["Tmax"]

    gx_h = g[0] * h
    gy_h = g[1] * h
    gz_h = g[2] * h

    # --- Pack constraints ---
    def eq_sliding(cmd):
        return alpha1 * cmd[0] + alpha2 * cmd[1] + alpha3 * cmd[2] - alpha0

    def ineq_vel_xL(cmd):
        return cmd[0] - (1 / h) * (vx_lo - v[0] - gx_h)
    def ineq_vel_xU(cmd):
        return (1 / h) * (vx_hi - v[0] - gx_h) - cmd[0]
    def ineq_vel_yL(cmd):
        return cmd[1] - (1 / h) * (vy_lo - v[1] - gy_h)
    def ineq_vel_yU(cmd):
        return (1 / h) * (vy_hi - v[1] - gy_h) - cmd[1]
    def ineq_vel_zL(cmd):
        return cmd[2] - (1 / h) * (vz_lo - v[2] - gz_h)
    def ineq_vel_zU(cmd):
        return (1 / h) * (vz_hi - v[2] - gz_h) - cmd[2]
    def ineq_thrust(cmd):
        return (Tmax / m)**2 - (cmd[0]**2 + cmd[1]**2 + cmd[2]**2)

    def ineq_ground_avoidance(cmd):
        # Prevents free fall.  Activates whenever v[2] <= 0 (level OR descending),
        # requiring at least full gravity compensation in the z-command.
        # Using < 0 would skip this at v[2]=0 (level flight), allowing SLSQP
        # to return cmd[2]=0 and letting gravity cause a free-fall step.
        if v[2] <= 0:
            z_safe = max(r[2] - 0.5, 0.01)  # 0.5 m margin
            a_req = (v[2]**2) / (2.0 * z_safe)
            return (cmd[2] + g[2]) - a_req
        return cmd[2] + g[2] + 100.0  # safe if going up

    constraints = [
        {"type": "eq",   "fun": eq_sliding},
        {"type": "ineq", "fun": ineq_vel_xL},
        {"type": "ineq", "fun": ineq_vel_xU},
        {"type": "ineq", "fun": ineq_vel_yL},
        {"type": "ineq", "fun": ineq_vel_yU},
        {"type": "ineq", "fun": ineq_vel_zL},
        {"type": "ineq", "fun": ineq_vel_zU},
        {"type": "ineq", "fun": ineq_thrust},
        #{"type": "ineq", "fun": ineq_ground_avoidance},
    ]
    return constraints


def objective(cmd, *_args):
    """Performance index: square of control acceleration magnitude."""
    return cmd[0]**2 + cmd[1]**2 + cmd[2]**2


# =============================================================================
# Guidance Law — dual mode: SMC-IO (far) + PD terminal (near)
# =============================================================================

def terminal_guidance(states, params):
    """
    Proportional-derivative terminal controller.
    Drives position to rfd and velocity to zero.
    Returns acceleration command [ax, ay, az].
    """
    r = states[0:3]
    v = states[3:6]
    m = states[6]

    vfd = params.get("vfd", np.zeros(3))
    pos_err = params["rfd"] - r
    vel_err = vfd - v   # target velocity = vfd

    # PD gains
    Kp = 1.5   # position proportional gain
    Kd = 3.0   # velocity damping gain

    # Desired acceleration = Kp * (rfd - r) + Kd * (0 - v) - g_compensation
    g_vec = params["g_vec"]
    a_cmd = Kp * pos_err + Kd * vel_err - g_vec   # compensate gravity

    # Add obstacle avoidance
    a_cmd += obstacle_acceleration(r, params)

    # Clamp to thrust limit
    a_max = params["Tmax"] / m
    a_norm = np.linalg.norm(a_cmd)
    if a_norm > a_max:
        a_cmd = a_cmd * (a_max / a_norm)

    return a_cmd


def guidance(t, states, params):
    """
    Dual-mode guidance:
      - Far from target (dist > terminal_radius):  SMC-IO via SLSQP
      - Near target (dist ≤ terminal_radius):      PD terminal controller

    This prevents oscillation at the target that occurs when the heading-
    error sliding surface degenerates as R → 0.
    """
    r = states[0:3]
    dist = np.linalg.norm(r - params["rfd"])
    terminal_radius = params.get("terminal_radius", 15.0)

    if dist <= terminal_radius:
        return terminal_guidance(states, params)

    # SMC-IO phase
    cons = build_constraints(t, states, params)
    x0   = np.zeros(3)
    res  = minimize(
        objective, x0,
        method="SLSQP",
        constraints=cons,
        options={"maxiter": 200, "ftol": 1e-12, "disp": False},
    )
    cmd = res.x

    # Add obstacle avoidance on top of SLSQP result
    a_obs = obstacle_acceleration(r, params)
    cmd = cmd + a_obs

    # Clamp to thrust limit
    m = states[6]
    a_max = params["Tmax"] / m
    a_norm = np.linalg.norm(cmd)
    if a_norm > a_max:
        cmd = cmd * (a_max / a_norm)

    return cmd


# =============================================================================
# State equations and RK4 integrator
# =============================================================================

def state_equations_rk4(states, command, t, params):
    """
    RHS of the ODE system.  State = [x, y, z, vx, vy, vz, m].
    """
    r = states[0:3]
    v = states[3:6]
    m = states[6]

    g = environment_acceleration(r, v, t, params)
    a_obs = obstacle_acceleration(r, params)

    a_total = np.linalg.norm(command)
    thrust  = m * a_total
    mdot    = thrust / params["c"]

    dstate = np.zeros(7)
    dstate[0:3] = v
    dstate[3:6] = g + a_obs + command
    dstate[6]   = -mdot
    return dstate


def rk4_step(states, command, t, params):
    """Classical 4th-order Runge-Kutta step."""
    h  = params["dt"]
    k1 = state_equations_rk4(states,                  command, t,           params)
    k2 = state_equations_rk4(states + 0.5 * h * k1,  command, t + 0.5 * h, params)
    k3 = state_equations_rk4(states + 0.5 * h * k2,  command, t + 0.5 * h, params)
    k4 = state_equations_rk4(states + h * k3,         command, t + h,       params)
    return states + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


# =============================================================================
# Wrapper
# =============================================================================

def build_params(scenario_id=1):
    """Wrapper that imports parameters from quadrotor_scenarios.py."""
    return scenarios_build_params(scenario_id)


# =============================================================================
# Main simulation loop
# =============================================================================

def simulate(scenario_id=1, verbose=True, params_override=None):
    """
    Run the SMC-IO guided quadrotor trajectory simulation.

    Returns
    -------
    t_arr       : ndarray (N,)
    states_arr  : ndarray (N, 7)  — [x,y,z, vx,vy,vz, m]
    command_arr : ndarray (N, 5)  — [t, ax, ay, az, |T|]
    J_arr       : ndarray (N, 2)  — [t, cumulative_control_effort]
    params      : dict
    """
    params = params_override if params_override else build_params(scenario_id)

    t0 = params["t0"]
    tf = params["tf"]
    dt = params["dt"]

    states = np.concatenate([params["pos0"], params["vel0"], [params["m0"]]])

    t_list       = []
    states_list  = []
    command_list = []
    J_cumulative = 0.0
    J_list       = []

    t    = t0
    step = 0
    n_steps = int((tf - t0) / dt)

    if verbose:
        print(f"\n{'='*60}")
        print(f"  Quadrotor SMC-IO Guidance  —  {params.get('scenario_name', 'Custom')}")
        print(f"{'='*60}")
        print(f"  Start pos  : {params['pos0']}")
        print(f"  Start vel  : {params['vel0']}")
        print(f"  Target pos : {params['rfd']}")
        print(f"  Max thrust : {params['Tmax']} N   |  Mass: {params['m0']} kg")
        print(f"  Wind base  : {params['wind']['base']}")
        print(f"  Obstacles  : {len(params.get('obstacles', []))}")
        print(f"  Ee function: {params.get('ee_function', 'quadratic_cubic')}")
        print(f"  Simulating {tf} s  with dt={dt} s ...\n")

    for i in range(n_steps):
        dist = np.linalg.norm(states[0:3] - params["rfd"])

        # Termination: close enough to target and slow enough
        if dist < 0.5 and np.linalg.norm(states[3:6]) < 0.5:
            if verbose:
                print(f"  ✓ Reached target at t = {t:.2f} s  (dist={dist:.4f} m)")
            break

        # Compute guidance command
        try:
            cmd = guidance(t, states, params)
        except Exception as e:
            if verbose:
                print(f"  ✗ Guidance failed at t={t:.2f}s: {e}")
            break

        # Propagate state
        states = rk4_step(states, cmd, t, params)

        # Ensure altitude doesn't go negative (ground)
        if states[2] < 0.0:
            states[2] = 0.0
            if states[5] < 0.0:
                states[5] = 0.0

        thrust_mag = np.linalg.norm(cmd) * states[6]

        # Record
        t_list.append(t)
        states_list.append(states.copy())
        command_list.append([t, cmd[0], cmd[1], cmd[2], thrust_mag])

        J_cumulative += np.linalg.norm(cmd) * dt
        J_list.append([t, J_cumulative])

        # Progress
        if verbose and (step % max(1, n_steps // 10) == 0):
            speed = np.linalg.norm(states[3:6])
            rho   = isa_density(max(states[2], 0.0))
            print(f"  t={t:7.2f}s  dist={dist:8.3f}m  |v|={speed:7.4f}m/s  "
                  f"alt={states[2]:8.2f}m  ρ={rho:.4f}kg/m³  m={states[6]:.4f}kg")

        t    += dt
        step += 1

    # Convert to arrays
    t_arr       = np.array(t_list)
    states_arr  = np.array(states_list)  if states_list  else np.empty((0, 7))
    command_arr = np.array(command_list) if command_list else np.empty((0, 5))
    J_arr       = np.array(J_list)      if J_list       else np.empty((0, 2))

    if verbose and len(states_list) > 0:
        rf = states_arr[-1, 0:3]
        vf = states_arr[-1, 3:6]
        mf = states_arr[-1, 6]
        print(f"\n  === Results ===")
        print(f"  Final position  : ({rf[0]:.4f}, {rf[1]:.4f}, {rf[2]:.4f}) m")
        print(f"  Position error  : {np.linalg.norm(rf - params['rfd']):.6f} m")
        print(f"  Final velocity  : ({vf[0]:.6f}, {vf[1]:.6f}, {vf[2]:.6f}) m/s")
        print(f"  Landing speed   : {np.linalg.norm(vf):.6f} m/s")
        print(f"  Final mass      : {mf:.4f} kg")
        print(f"  Mass consumed   : {params['m0'] - mf:.6f} kg")
        print(f"  Control effort  : {J_cumulative:.6f}")

        # Obstacle clearance report
        if params.get("obstacles"):
            violated, clearance = check_obstacle_violation(rf, params["obstacles"])
            print(f"  Min clearance   : {clearance:.2f} m {'⚠ VIOLATION' if violated else '✓ Safe'}")

    return t_arr, states_arr, command_arr, J_arr, params


# =============================================================================
# Plotting
# =============================================================================

def _draw_obstacles(ax, obstacles, alpha=0.15):
    """Draw translucent spheres for each obstacle on a 3D axis."""
    for obs in obstacles:
        c = obs["centre"]
        r = obs["radius"]
        u = np.linspace(0, 2 * np.pi, 30)
        v = np.linspace(0, np.pi, 20)
        xs = c[0] + r * np.outer(np.cos(u), np.sin(v))
        ys = c[1] + r * np.outer(np.sin(u), np.sin(v))
        zs = c[2] + r * np.outer(np.ones_like(u), np.cos(v))
        ax.plot_surface(xs, ys, zs, color='red', alpha=alpha)


def plot_results(t_arr, states_arr, command_arr, J_arr, params, save_prefix=None):
    """Generate plots: 3D trajectory, state history, thrust, control effort."""
    if len(t_arr) == 0:
        print("No data to plot.")
        return

    obstacles = params.get("obstacles", [])

    # --- Figure 1: 3D Trajectory ---
    fig1 = plt.figure(figsize=(10, 8))
    ax3d = fig1.add_subplot(111, projection="3d")
    ax3d.plot(states_arr[:, 0], states_arr[:, 1], states_arr[:, 2],
              'b-', linewidth=2, label="Trajectory")
    ax3d.scatter(*params["pos0"], color="green", s=100, marker="^", label="Start")
    ax3d.scatter(*params["rfd"],  color="red",   s=100, marker="*", label="Target")

    # Draw obstacles
    _draw_obstacles(ax3d, obstacles)

    # Wind annotation
    w = params["wind"]["base"]
    if np.linalg.norm(w) > 0:
        mid = states_arr[len(states_arr)//2, 0:3]
        ax3d.quiver(mid[0], mid[1], mid[2], w[0]*3, w[1]*3, w[2]*3,
                    color='orange', arrow_length_ratio=0.3, linewidth=2, label="Wind")

    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.set_title(f"Quadrotor Trajectory — {params.get('scenario_name', '')}")
    ax3d.legend()

    # --- Figure 2: State history 3×3 ---
    fig2, axes2 = plt.subplots(3, 3, figsize=(15, 10))
    labels_pos = ["x (m)", "y (m)", "z (m)"]
    labels_vel = ["vx (m/s)", "vy (m/s)", "vz (m/s)"]
    labels_acc = ["ax (m/s²)", "ay (m/s²)", "az (m/s²)"]

    for i in range(3):
        axes2[i, 0].plot(t_arr, states_arr[:, i], 'b-')
        axes2[i, 0].set_ylabel(labels_pos[i])
        axes2[i, 0].set_xlabel("Time (s)")
        axes2[i, 0].grid(True, alpha=0.3)

        axes2[i, 1].plot(t_arr, states_arr[:, i+3], 'r-')
        axes2[i, 1].set_ylabel(labels_vel[i])
        axes2[i, 1].set_xlabel("Time (s)")
        axes2[i, 1].grid(True, alpha=0.3)

        axes2[i, 2].plot(command_arr[:, 0], command_arr[:, i+1], 'g-')
        axes2[i, 2].set_ylabel(labels_acc[i])
        axes2[i, 2].set_xlabel("Time (s)")
        axes2[i, 2].grid(True, alpha=0.3)

    axes2[0, 0].set_title("Position")
    axes2[0, 1].set_title("Velocity")
    axes2[0, 2].set_title("Acceleration Command")
    fig2.suptitle(f"State History — {params.get('scenario_name', '')}", fontsize=14)
    fig2.tight_layout()

    # --- Figure 3: Mass ---
    fig3, ax3 = plt.subplots(figsize=(8, 5))
    ax3.plot(t_arr, states_arr[:, 6], 'k-', linewidth=2)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Mass (kg)")
    ax3.set_title("Quadrotor Mass")
    ax3.grid(True, alpha=0.3)

    # --- Figure 4: Thrust ---
    fig4, ax4 = plt.subplots(figsize=(8, 5))
    ax4.plot(command_arr[:, 0], command_arr[:, 4], 'purple', linewidth=2)
    ax4.axhline(params["Tmax"], color='r', linestyle='--', alpha=0.5, label=f"Tmax={params['Tmax']}N")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Thrust (N)")
    ax4.set_title("Commanded Thrust Magnitude")
    ax4.grid(True, alpha=0.3)
    ax4.legend()

    # --- Figure 5: Control effort ---
    fig5, ax5 = plt.subplots(figsize=(8, 5))
    ax5.plot(J_arr[:, 0], J_arr[:, 1], 'darkcyan', linewidth=2)
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Cumulative Control Effort")
    ax5.set_title("Control Effort")
    ax5.grid(True, alpha=0.3)

    # --- Figure 6: Air density along trajectory ---
    fig6, ax6 = plt.subplots(figsize=(8, 5))
    altitudes = states_arr[:, 2]
    densities = np.array([isa_density(max(a, 0)) for a in altitudes])
    ax6.plot(t_arr, densities, 'brown', linewidth=2)
    ax6.set_xlabel("Time (s)")
    ax6.set_ylabel("Air Density ρ (kg/m³)")
    ax6.set_title("Air Density Along Trajectory (ISA)")
    ax6.grid(True, alpha=0.3)

    if save_prefix:
        fig1.savefig(f"{save_prefix}_trajectory_3d.png", dpi=150)
        fig2.savefig(f"{save_prefix}_states.png", dpi=150)
        fig3.savefig(f"{save_prefix}_mass.png", dpi=150)
        fig4.savefig(f"{save_prefix}_thrust.png", dpi=150)
        fig5.savefig(f"{save_prefix}_control_effort.png", dpi=150)
        fig6.savefig(f"{save_prefix}_density.png", dpi=150)
        print(f"\n  Figures saved with prefix '{save_prefix}'")

    plt.show()


# =============================================================================
# Interactive 3D (Plotly)
# =============================================================================

def plot_interactive_3d(t_arr, states_arr, command_arr, params):
    """Interactive 3D trajectory with Plotly."""
    if not PLOTLY_AVAILABLE:
        print("Plotly not installed. Run:  pip install plotly")
        return

    fig = go.Figure()

    # Trajectory
    fig.add_trace(go.Scatter3d(
        x=states_arr[:, 0], y=states_arr[:, 1], z=states_arr[:, 2],
        mode='lines+markers',
        marker=dict(size=3, color=t_arr, colorscale='Viridis',
                    showscale=True, colorbar=dict(title="Time (s)")),
        line=dict(color='lightblue', width=2),
        text=[f"t={t:.2f}s<br>v={np.linalg.norm(states_arr[i,3:6]):.3f}m/s<br>"
              f"dist={np.linalg.norm(states_arr[i,0:3] - params['rfd']):.2f}m<br>"
              f"alt={states_arr[i,2]:.1f}m"
              for i, t in enumerate(t_arr)],
        hoverinfo="text",
        name="Trajectory"
    ))

    # Start / Target markers
    fig.add_trace(go.Scatter3d(
        x=[params['pos0'][0]], y=[params['pos0'][1]], z=[params['pos0'][2]],
        mode='markers+text', marker=dict(size=10, color='green'),
        text=['Start'], textposition='top center', name='Start'
    ))
    fig.add_trace(go.Scatter3d(
        x=[params['rfd'][0]], y=[params['rfd'][1]], z=[params['rfd'][2]],
        mode='markers+text', marker=dict(size=12, color='red'),
        text=['Target'], textposition='top center', name='Target'
    ))

    # Obstacles as mesh spheres
    for obs in params.get("obstacles", []):
        c, rad = obs["centre"], obs["radius"]
        u = np.linspace(0, 2*np.pi, 25)
        v = np.linspace(0, np.pi, 15)
        xs = c[0] + rad * np.outer(np.cos(u), np.sin(v))
        ys = c[1] + rad * np.outer(np.sin(u), np.sin(v))
        zs = c[2] + rad * np.outer(np.ones_like(u), np.cos(v))
        fig.add_trace(go.Surface(
            x=xs, y=ys, z=zs, opacity=0.25,
            colorscale=[[0, 'red'], [1, 'red']], showscale=False,
            name=f'Obstacle r={rad}m'
        ))

    fig.update_layout(
        title=f"Quadrotor Trajectory — {params.get('scenario_name', '')}",
        scene=dict(xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
                   aspectmode='data'),
        width=1200, height=800
    )
    fig.show()


# =============================================================================
# Scenario comparison
# =============================================================================

def compare_all_scenarios(verbose=False):
    """Run all flight scenarios and compare."""
    results = {}
    metrics = {}

    scenarios = get_all_scenarios()
    ids = sorted(scenarios.keys())

    print("\n" + "=" * 70)
    print("Running all quadrotor flight scenarios ...")
    print("=" * 70)

    for sid in ids:
        print(f"\n  Running FS{sid} ({scenarios[sid]['name']}) ...", end=" ", flush=True)
        t, s, c, j, p = simulate(scenario_id=sid, verbose=False)
        results[sid] = {'t': t, 's': s, 'c': c, 'j': j, 'p': p}

        if len(s) > 0:
            rf = s[-1, 0:3]
            vf = s[-1, 3:6]
            mf = s[-1, 6]
            metrics[sid] = {
                'final_time':     t[-1],
                'distance_error': np.linalg.norm(rf - p['rfd']),
                'landing_speed':  np.linalg.norm(vf),
                'fuel_used':      p['m0'] - mf,
                'control_effort': j[-1, 1],
            }
            print(f"✓  t={t[-1]:.1f}s  err={metrics[sid]['distance_error']:.3f}m  "
                  f"v={metrics[sid]['landing_speed']:.4f}m/s")
        else:
            print("✗ Failed")

    # Comparison plots
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    fig.suptitle("Quadrotor Flight Scenario Comparison", fontsize=16, fontweight='bold')

    ax = axes[0, 0]
    for sid in ids:
        if sid in results and len(results[sid]['s']) > 0:
            r = results[sid]
            dists = np.linalg.norm(r['s'][:, 0:3] - r['p']['rfd'], axis=1)
            ax.semilogy(r['t'], dists, linewidth=2, label=f'FS{sid}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (m)')
    ax.set_title('Distance to Target')
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[0, 1]
    for sid in ids:
        if sid in results and len(results[sid]['s']) > 0:
            r = results[sid]
            speeds = np.linalg.norm(r['s'][:, 3:6], axis=1)
            ax.plot(r['t'], speeds, linewidth=2, label=f'FS{sid}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (m/s)')
    ax.set_title('Speed Over Time')
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[0, 2]
    for sid in ids:
        if sid in results and len(results[sid]['j']) > 0:
            r = results[sid]
            ax.plot(r['j'][:, 0], r['j'][:, 1], linewidth=2, label=f'FS{sid}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Cumulative Effort')
    ax.set_title('Control Effort')
    ax.grid(True, alpha=0.3)
    ax.legend()

    m_ids = sorted(metrics.keys())
    ax = axes[1, 0]
    ax.bar(m_ids, [metrics[s]['final_time'] for s in m_ids], color='steelblue')
    ax.set_xlabel('Scenario')
    ax.set_ylabel('Time (s)')
    ax.set_title('Flight Duration')

    ax = axes[1, 1]
    ax.bar(m_ids, [metrics[s]['distance_error'] for s in m_ids], color='coral')
    ax.set_xlabel('Scenario')
    ax.set_ylabel('Error (m)')
    ax.set_title('Position Error')

    ax = axes[1, 2]
    ax.bar(m_ids, [metrics[s]['control_effort'] for s in m_ids], color='mediumseagreen')
    ax.set_xlabel('Scenario')
    ax.set_ylabel('Effort')
    ax.set_title('Total Control Effort')

    plt.tight_layout()
    plt.show()

    # Print summary table
    print("\n" + "=" * 80)
    print("Summary")
    print("=" * 80)
    print(f"{'FS':<4} {'Time(s)':<10} {'Error(m)':<12} {'Speed(m/s)':<14} {'Effort':<12}")
    print("-" * 80)
    for sid in m_ids:
        m = metrics[sid]
        print(f"{sid:<4} {m['final_time']:<10.2f} {m['distance_error']:<12.4f} "
              f"{m['landing_speed']:<14.6f} {m['control_effort']:<12.4f}")
    print("=" * 80)

    return results, metrics


# =============================================================================
# Entry point
# =============================================================================

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Quadrotor SMC-IO Guidance — trajectory with wind, drag, obstacles"
    )
    parser.add_argument(
        "--scenario", type=int, default=1,
        help="Flight scenario number (default: 1)"
    )
    parser.add_argument(
        "--mode", type=str, default="standard",
        choices=["standard", "interactive", "compare"],
        help="Visualisation mode (default: standard)"
    )
    parser.add_argument(
        "--no-plot", action="store_true",
        help="Run simulation without plotting"
    )
    parser.add_argument(
        "--save", type=str, default=None,
        help="Save plots with this filename prefix"
    )
    args = parser.parse_args()

    if args.mode == "compare":
        compare_all_scenarios()
    else:
        t_arr, states_arr, command_arr, J_arr, params = simulate(
            scenario_id=args.scenario, verbose=True
        )

        if not args.no_plot:
            if args.mode == "standard":
                plot_results(t_arr, states_arr, command_arr, J_arr, params,
                             save_prefix=args.save)
            elif args.mode == "interactive":
                plot_interactive_3d(t_arr, states_arr, command_arr, params)

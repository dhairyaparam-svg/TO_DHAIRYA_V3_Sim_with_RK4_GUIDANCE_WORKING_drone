"""
SMC-IO (Sliding-Mode-Control-based Instantaneously Optimal) Guidance
for Precision Soft Landing on an Asteroid

Python translation of the MATLAB simulation code.
Based on: Shincy & Ghosh, J. Spacecraft and Rockets, Vol. 60, No. 1, 2023.

Usage:
    python smc_io_guidance.py
"""

import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Import scenarios from external file
from scenarios import build_params as scenarios_build_params, get_all_scenarios, get_scenario

try:
    import plotly.graph_objects as go
    import plotly.subplots as sp
    PLOTLY_AVAILABLE = True
except ImportError:
    PLOTLY_AVAILABLE = False

try:
    import dash
    from dash import dcc, html, Input, Output
    DASH_AVAILABLE = True
except ImportError:
    DASH_AVAILABLE = False

# =============================================================================
# Gravity Model
# =============================================================================

def gravity_potential(pos, GM, la, lb, lc):
    """
    Second-order spherical harmonic gravitational potential of a triaxial
    ellipsoidal asteroid.  Eq. (5) in the paper.
    """
    x, y, z = pos
    C20 = (2 * lc**2 - (la**2 + lb**2)) / (10 * la**2)
    C22 = (la**2 - lb**2) / (20 * la**2)

    r = np.sqrt(x**2 + y**2 + z**2)
    phi = np.arcsin(z / r)          # latitude
    lamda = np.arctan2(y, x)        # longitude

    T1 = 0.5 * C20 * (3 * np.sin(phi)**2 - 1)
    T2 = 3 * C22 * np.cos(phi)**2 * np.cos(2 * lamda)

    U = (GM / r) * (1 + (la / r)**2 * (T1 + T2))
    return U


def numerical_gradient(func, pos, dx=1e-6):
    """Numerical gradient of a scalar function via forward differences."""
    f0 = func(pos)
    grad = np.zeros(3)
    for i in range(3):
        pos_pert = pos.copy()
        pos_pert[i] += dx
        grad[i] = (func(pos_pert) - f0) / dx
    return grad


def gravity_model(pos, vel, GM, la, lb, lc, omg, omg_dot):
    """
    Generalized acceleration  g(r,t) that combines gravitational gradient,
    Coriolis, centrifugal and Euler terms in the asteroid-center-fixed frame.
    See text after Eq. (2).
    """
    delU = numerical_gradient(
        lambda p: gravity_potential(p, GM, la, lb, lc), pos
    )
    g = (
        -2 * np.cross(omg, vel)
        - np.cross(omg_dot, pos)
        - np.cross(omg, np.cross(omg, pos))
        + delU
    )
    return g


# =============================================================================
# Velocity profile helpers  (Eqs. 26-32 in the paper)
# =============================================================================

def velocity_profile_max(pos_next, params):
    """
    Maximum allowable velocity magnitude in each axis at position pos_next.
    Returns VxM, VyM, VzM.
    """
    pos0 = params["pos0"]
    vel0 = params["vel0"]
    rfd  = params["rfd"]
    bx, by, bz = params["bx"], params["by"], params["bz"]
    cx, cy, cz = params["cx"], params["cy"], params["cz"]

    # zeta  (Eq. 27)
    denom_x = pos0[0] - rfd[0]
    denom_y = pos0[1] - rfd[1]
    denom_z = pos0[2] - rfd[2]

    zeta_x = (pos_next[0] - rfd[0]) / denom_x if abs(denom_x) > 1e-12 else 0.0
    zeta_y = (pos_next[1] - rfd[1]) / denom_y if abs(denom_y) > 1e-12 else 0.0
    zeta_z = (pos_next[2] - rfd[2]) / denom_z if abs(denom_z) > 1e-12 else 0.0

    # dx, dy, dz terms (set to 0 in the MATLAB code — original Eq. 26 form)
    # The MATLAB code uses dx=dy=dz=0, px=py=pz=0 so the extra term vanishes.
    VxM = abs(vel0[0]) * np.log10(max(1 + bx * zeta_x + cx * zeta_x**2, 1e-30))
    VyM = abs(vel0[1]) * np.log10(max(1 + by * zeta_y + cy * zeta_y**2, 1e-30))
    VzM = abs(vel0[2]) * np.log10(max(1 + bz * zeta_z + cz * zeta_z**2, 1e-30))

    return VxM, VyM, VzM


def velocity_bounds(VxM, VyM, VzM, params):
    """
    Lower and upper velocity bounds per Eq. (32).
    Returns (vx_lo, vx_hi), (vy_lo, vy_hi), (vz_lo, vz_hi).
    """
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
# Constraint and objective for the optimisation  (Eqs. 14-21 / 36)
# =============================================================================

def heading_error_coefficients(r, v, rfd, g):
    """
    Compute alpha_0 .. alpha_4 and the sliding variable Se.
    See Eqs. (12-13) and (25).
    """
    R     = rfd - r
    Rdot  = -v

    R_norm = np.linalg.norm(R)
    v_norm = np.linalg.norm(v)
    Rv     = np.dot(R, v)

    cos_e = np.clip(Rv / (R_norm * v_norm), -1.0, 1.0)
    Se    = np.arccos(cos_e)

    sin_e = np.sqrt(max(1 - cos_e**2, 1e-30))
    K = -1.0 / ((R_norm * v_norm)**2 * sin_e)

    common_Rv_R_over_v = Rv * R_norm / v_norm

    alpha1 = K * (R_norm * v_norm * (rfd[0] - r[0]) - common_Rv_R_over_v * v[0])
    alpha2 = K * (R_norm * v_norm * (rfd[1] - r[1]) - common_Rv_R_over_v * v[1])
    alpha3 = K * (R_norm * v_norm * (rfd[2] - r[2]) - common_Rv_R_over_v * v[2])
    alpha4 = K * (
        R_norm * v_norm * np.dot(R, g)
        - (Rv * R_norm * np.dot(v, g)) / v_norm
        + R_norm * v_norm * np.dot(Rdot, v)
        - (Rv * v_norm * np.dot(R, Rdot)) / R_norm
    )

    return alpha1, alpha2, alpha3, alpha4, Se


def compute_distance_based_ee(r_norm, pos0_norm, Ee0, ee_function="quadratic_cubic", rf_threshold=50.0):
    """
    Compute distance-dependent sliding mode coefficient Ee using selected function.
    
    Parameters
    ----------
    r_norm : float
        Norm of position vector (distance from origin)
    pos0_norm : float
        Norm of initial position vector (initial distance)
    Ee0 : float
        Sliding mode constant (tuning parameter)
    ee_function : str
        Name of distance-based function to use:
        - "quadratic_cubic": 0.5*(Ee0/r0²)*r² + 0.5*(Ee0/r0³)*r³ (default)
        - "linear": (Ee0/r0)*r
        - "quadratic": (Ee0/r0²)*r²
        - "cubic": (Ee0/r0³)*r³
        - "exponential": Ee0*(exp(r/r0) - 1)
        - "saturation": Ee0 (constant)
    rf_threshold : float
        Distance threshold below which Ee is set to 0
        
    Returns
    -------
    float
        Distance-dependent reaching law coefficient Ee
    """
    if r_norm < rf_threshold:
        return 0
    
    # Normalized distance ratio
    r_ratio =  pos0_norm / r_norm 
    
    if ee_function == "linear":
        return Ee0 * r_ratio
    
    elif ee_function == "quadratic":
        return Ee0 * r_ratio**2
    
    elif ee_function == "cubic":
        return Ee0 * r_ratio**3
    
    elif ee_function == "exponential":
        # Exponential: Ee0 * (exp(r/r0) - 1)
        return Ee0 * (np.exp(r_ratio) - 1.0)
    
    elif ee_function == "saturation":
        # Constant (no distance dependence)
        return Ee0
    
    else:  # "quadratic_cubic" (default)
        # Paper formula: balanced quadratic + cubic
        return (0.5 * (Ee0 / pos0_norm**2) * r_norm**2
                + 0.5 * (Ee0 / pos0_norm**3) * r_norm**3)


def build_constraints(states, params):
    """
    Return a list of scipy constraint dicts for the optimisation at one
    time step.
    """
    h  = params["dt"]
    r  = states[0:3]
    v  = states[3:6]
    m  = states[6]

    g = gravity_model(
        r, v,
        params["GM"], params["la"], params["lb"], params["lc"],
        params["omg"], params["omg_dot"],
    )

    # Sliding-mode parameters with distance-dependent function
    Ee0 = params["Ee0"]
    ke  = params["ke"]
    ee_function = params.get("ee_function", "quadratic_cubic")  # Default to paper formula
    rf_threshold = 1050.0
    r_norm = np.linalg.norm(r)
    pos0_norm = np.linalg.norm(params["pos0"])
    
    Ee = compute_distance_based_ee(r_norm, pos0_norm, Ee0, ee_function, rf_threshold)

    alpha1, alpha2, alpha3, alpha4, Se = heading_error_coefficients(
        r, v, params["rfd"], g
    )
    alpha0 = -alpha4 - Ee * np.sign(Se) - ke * Se

    # ---- Position at next time step (Euler approx) ----
    pos_next = r + v * h

    # ---- Velocity envelope ----
    VxM, VyM, VzM = velocity_profile_max(pos_next, params)
    (vx_lo, vx_hi), (vy_lo, vy_hi), (vz_lo, vz_hi) = velocity_bounds(
        VxM, VyM, VzM, params
    )

    Tmax = params["Tmax"]

    # Pre-compute gravity-assisted velocity terms
    gx_h = g[0] * h
    gy_h = g[1] * h
    gz_h = g[2] * h

    # ---- Pack constraints ----
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

    constraints = [
        {"type": "eq",   "fun": eq_sliding},
        {"type": "ineq", "fun": ineq_vel_xL},
        {"type": "ineq", "fun": ineq_vel_xU},
        {"type": "ineq", "fun": ineq_vel_yL},
        {"type": "ineq", "fun": ineq_vel_yU},
        {"type": "ineq", "fun": ineq_vel_zL},
        {"type": "ineq", "fun": ineq_vel_zU},
        {"type": "ineq", "fun": ineq_thrust},
    ]
    return constraints


def objective(cmd, *_args):
    """Performance index: square of instantaneous control acceleration."""
    return cmd[0]**2 + cmd[1]**2 + cmd[2]**2


# =============================================================================
# Guidance law  (calls the optimiser at every time step)
# =============================================================================

def guidance(t, states, params):
    """
    Solve the static optimization problem (Eq. 36) to get the SMC-IO
    guidance command [ax, ay, az].
    """
    cons = build_constraints(states, params)
    x0 = np.zeros(3)
    res = minimize(
        objective, x0,
        method="SLSQP",
        constraints=cons,
        options={"maxiter": 200, "ftol": 1e-12, "disp": False},
    )
    return res.x


# =============================================================================
# State equations and RK4 integrator
# =============================================================================

def state_equations_rk4(states, command, params):
    """
    Right-hand side of the ODE system (7 states: x,y,z, vx,vy,vz, m).
    """
    r = states[0:3]
    v = states[3:6]
    m = states[6]

    g = gravity_model(
        r, v,
        params["GM"], params["la"], params["lb"], params["lc"],
        params["omg"], params["omg_dot"],
    )

    a_total = np.linalg.norm(command)
    thrust  = m * a_total
    mdot    = thrust / params["c"]

    dstate = np.zeros(7)
    dstate[0:3] = v
    dstate[3:6] = g + command
    dstate[6]   = -mdot
    return dstate


def rk4_step(states, command, params):
    """Classical 4th-order Runge-Kutta step."""
    h = params["dt"]
    k1 = state_equations_rk4(states, command, params)
    k2 = state_equations_rk4(states + 0.5 * h * k1, command, params)
    k3 = state_equations_rk4(states + 0.5 * h * k2, command, params)
    k4 = state_equations_rk4(states + h * k3, command, params)
    return states + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


# =============================================================================
# Landing Scenarios - Now in scenarios.py
# =============================================================================
# Modify scenarios.py to add or edit landing scenarios!


def build_params(ls_number=1):
    """Wrapper that imports parameters from scenarios.py"""
    return scenarios_build_params(ls_number)


# =============================================================================
# Main simulation loop
# =============================================================================

def simulate_with_params(params, verbose=True):
    """
    Run the SMC-IO guided asteroid landing simulation with custom parameters.
    
    Parameters
    ----------
    params : dict
        Parameter dictionary with all simulation settings
    verbose : bool
        Print progress information
        
    Returns
    -------
    t_arr : ndarray (N,)
        Time array
    states_arr : ndarray (N, 7)
        States: [x,y,z, vx,vy,vz, m]
    command_arr : ndarray (N, 5)
        Commands: [t, ax, ay, az, Thrust]
    J_arr : ndarray (N, 2)
        Control effort: [t, cumulative_control_effort]
    """
    t0 = params["t0"]
    tf = params["tf"]
    dt = params["dt"]

    states = np.concatenate([params["pos0"], params["vel0"], [params["m0"]]])

    t_list       = []
    states_list  = []
    command_list = []
    J_cumulative = 0.0
    J_list       = []

    t = t0
    step = 0
    n_steps = int((tf - t0) / dt)

    if verbose:
        print(f"=== Custom Landing Scenario ===")
        print(f"  Initial pos : {params['pos0']}")
        print(f"  Initial vel : {params['vel0']}")
        print(f"  Target  pos : {params['rfd']}")
        print(f"  Max thrust  : {params['Tmax']} N")
        print(f"  Ee function : {params.get('ee_function', 'quadratic_cubic')}")
        print(f"  Simulating up to {tf} s with dt={dt} s ...\n")

    for i in range(n_steps):
        # Termination: close enough to landing site
        if np.linalg.norm(states[0:3] - params["rfd"]) < 1e-3:
            if verbose:
                print(f"  Reached landing site at t = {t:.2f} s")
            break

        # Compute guidance command
        try:
            cmd = guidance(t, states, params)
        except:
            if verbose:
                print(f"  Guidance computation failed at t={t:.2f}s")
            break

        # Execute RK4 step
        states = rk4_step(states, cmd, params)

        # Record
        t_list.append(t)
        states_list.append(states.copy())
        command_list.append([t, cmd[0], cmd[1], cmd[2], np.linalg.norm(cmd)])

        # Control effort (thrust integral)
        J_cumulative += np.linalg.norm(cmd) * dt
        J_list.append([t, J_cumulative])

        t += dt
        step += 1

        if verbose and step % 100 == 0:
            print(f"  Step {step}: t={t:.1f}s, dist={np.linalg.norm(states[0:3] - params['rfd']):.3f}m, "
                  f"speed={np.linalg.norm(states[3:6]):.6f}m/s, m={states[6]:.2f}kg")

    if verbose:
        print(f"\n  Total steps: {step}")

    # Convert to arrays
    t_arr       = np.array(t_list)
    states_arr  = np.array(states_list)
    command_arr = np.array(command_list)
    J_arr       = np.array(J_list)

    return t_arr, states_arr, command_arr, J_arr


def simulate(ls_number=1, verbose=True):
    """
    Run the SMC-IO guided asteroid landing simulation for a given
    landing scenario.

    Returns
    -------
    t_arr : ndarray (N,)
    states_arr : ndarray (N, 7)  — [x,y,z, vx,vy,vz, m]
    command_arr : ndarray (N, 5) — [t, ax, ay, az, Thrust]
    J_arr : ndarray (N, 2)      — [t, cumulative_control_effort]
    """
    params = build_params(ls_number)

    t0 = params["t0"]
    tf = params["tf"]
    dt = params["dt"]

    states = np.concatenate([params["pos0"], params["vel0"], [params["m0"]]])

    t_list       = []
    states_list  = []
    command_list = []
    J_cumulative = 0.0
    J_list       = []

    t = t0
    step = 0
    n_steps = int((tf - t0) / dt)

    if verbose:
        print(f"=== Landing Scenario {ls_number} ===")
        print(f"  Initial pos : {params['pos0']}")
        print(f"  Initial vel : {params['vel0']}")
        print(f"  Target  pos : {params['rfd']}")
        print(f"  Max thrust  : {params['Tmax']} N")
        print(f"  Simulating up to {tf} s with dt={dt} s ...\n")

    for i in range(n_steps):
        # Termination: close enough to landing site
        if np.linalg.norm(states[0:3] - params["rfd"]) < 1e-3:
            if verbose:
                print(f"  Reached landing site at t = {t:.2f} s")
            break

        # Compute guidance command
        cmd = guidance(t, states, params)

        # Propagate state with RK4
        states = rk4_step(states, cmd, params)

        thrust = np.linalg.norm(cmd) * states[6]

        # Accumulate control effort
        J_cumulative += np.linalg.norm(cmd) * dt

        # Store history
        t_list.append(t)
        states_list.append(states.copy())
        command_list.append([t, cmd[0], cmd[1], cmd[2], thrust])
        J_list.append([t, J_cumulative])

        # Progress display
        if verbose and (i % 100 == 0 or i == n_steps - 1):
            dist = np.linalg.norm(states[0:3] - params["rfd"])
            speed = np.linalg.norm(states[3:6])
            print(f"  t={t:7.1f}s  dist={dist:10.4f}m  |v|={speed:.6f} m/s  m={states[6]:.2f}kg")

        t += dt
        step += 1

    t_arr       = np.array(t_list)
    states_arr  = np.array(states_list)
    command_arr = np.array(command_list)
    J_arr       = np.array(J_list)

    if verbose and len(states_list) > 0:
        rf = states_arr[-1, 0:3]
        vf = states_arr[-1, 3:6]
        mf = states_arr[-1, 6]
        print(f"\n  === Results ===")
        print(f"  Final position    : ({rf[0]:.4f}, {rf[1]:.4f}, {rf[2]:.4f}) m")
        print(f"  Position error    : {np.linalg.norm(rf - params['rfd']):.6f} m")
        print(f"  Final velocity    : ({vf[0]:.6f}, {vf[1]:.6f}, {vf[2]:.6f}) m/s")
        print(f"  Landing speed     : {np.linalg.norm(vf):.6f} m/s")
        print(f"  Final mass        : {mf:.4f} kg")
        print(f"  Fuel used         : {params['m0'] - mf:.4f} kg")
        print(f"  Total ctrl effort : {J_cumulative:.6f}")

    return t_arr, states_arr, command_arr, J_arr, params


# =============================================================================
# Plotting (replicates MATLAB plotit.m)
# =============================================================================

def plot_results(t_arr, states_arr, command_arr, J_arr, params, save_prefix=None):
    """Generate the same plots as the MATLAB code."""
    if len(t_arr) == 0:
        print("No data to plot.")
        return

    # --- Figure 1: 3-D trajectory ---
    fig1 = plt.figure(figsize=(8, 6))
    ax3d = fig1.add_subplot(111, projection="3d")
    ax3d.plot(states_arr[:, 0], states_arr[:, 1], states_arr[:, 2])
    ax3d.set_xlabel("x (m)")
    ax3d.set_ylabel("y (m)")
    ax3d.set_zlabel("z (m)")
    ax3d.set_title("Spacecraft Trajectory")

    # --- Figure 2: 3x3 grid (position, velocity, acceleration) ---
    fig2, axes2 = plt.subplots(3, 3, figsize=(14, 10))
    labels_pos = ["x (m)", "y (m)", "z (m)"]
    labels_vel = ["vx (m/s)", "vy (m/s)", "vz (m/s)"]
    labels_acc = ["ax (m/s²)", "ay (m/s²)", "az (m/s²)"]

    for i in range(3):
        axes2[i, 0].plot(t_arr, states_arr[:, i])
        axes2[i, 0].set_ylabel(labels_pos[i])
        axes2[i, 0].set_xlabel("Time (s)")
        axes2[i, 0].grid(True)

        axes2[i, 1].plot(t_arr, states_arr[:, i + 3])
        axes2[i, 1].set_ylabel(labels_vel[i])
        axes2[i, 1].set_xlabel("Time (s)")
        axes2[i, 1].grid(True)

        axes2[i, 2].plot(command_arr[:, 0], command_arr[:, i + 1])
        axes2[i, 2].set_ylabel(labels_acc[i])
        axes2[i, 2].set_xlabel("Time (s)")
        axes2[i, 2].grid(True)

    axes2[0, 0].set_title("Position")
    axes2[0, 1].set_title("Velocity")
    axes2[0, 2].set_title("Acceleration")
    fig2.tight_layout()

    # --- Figure 3: Mass ---
    fig3, ax3 = plt.subplots(figsize=(8, 5))
    ax3.plot(t_arr, states_arr[:, 6])
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Mass (kg)")
    ax3.set_title("Spacecraft Mass")
    ax3.grid(True)

    # --- Figure 4: Thrust ---
    fig4, ax4 = plt.subplots(figsize=(8, 5))
    ax4.plot(command_arr[:, 0], command_arr[:, 4])
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Thrust (N)")
    ax4.set_title("Commanded Thrust")
    ax4.grid(True)

    # --- Figure 5: Control Effort ---
    fig5, ax5 = plt.subplots(figsize=(8, 5))
    ax5.plot(J_arr[:, 0], J_arr[:, 1])
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Control Effort")
    ax5.set_title("Cumulative Control Effort")
    ax5.grid(True)

    if save_prefix:
        fig1.savefig(f"{save_prefix}_trajectory_3d.png", dpi=150)
        fig2.savefig(f"{save_prefix}_states.png", dpi=150)
        fig3.savefig(f"{save_prefix}_mass.png", dpi=150)
        fig4.savefig(f"{save_prefix}_thrust.png", dpi=150)
        fig5.savefig(f"{save_prefix}_control_effort.png", dpi=150)

    plt.show()


# =============================================================================
# Interactive Visualizations
# =============================================================================

def plot_interactive_3d(t_arr, states_arr, command_arr, params):
    """Interactive 3D trajectory with Plotly - hover for details."""
    if not PLOTLY_AVAILABLE:
        print("Plotly not installed. Run: pip install plotly")
        return

    fig = go.Figure()

    # Trajectory with color by time
    fig.add_trace(go.Scatter3d(
        x=states_arr[:, 0], y=states_arr[:, 1], z=states_arr[:, 2],
        mode='lines+markers',
        marker=dict(
            size=3,
            color=t_arr,
            colorscale='Viridis',
            showscale=True,
            colorbar=dict(title="Time (s)")
        ),
        line=dict(color='lightblue', width=2),
        text=[f"t={t:.1f}s<br>v={np.linalg.norm(states_arr[i,3:6]):.4f}m/s<br>"
              f"dist={np.linalg.norm(states_arr[i,0:3] - params['rfd']):.3f}m"
              for i, t in enumerate(t_arr)],
        hoverinfo="text",
        name="Trajectory"
    ))

    # Landing site
    fig.add_trace(go.Scatter3d(
        x=[params['rfd'][0]], y=[params['rfd'][1]], z=[params['rfd'][2]],
        mode='markers+text',
        marker=dict(size=12, color='red'),
        text=['Landing Site'],
        textposition='top center',
        name='Landing Site'
    ))

    # Initial position
    fig.add_trace(go.Scatter3d(
        x=[params['pos0'][0]], y=[params['pos0'][1]], z=[params['pos0'][2]],
        mode='markers+text',
        marker=dict(size=10, color='green'),
        text=['Start'],
        textposition='top center',
        name='Start'
    ))

    fig.update_layout(
        title="Interactive Asteroid Landing Trajectory (SMC-IO)",
        scene=dict(
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            zaxis_title="Z (m)",
            aspectmode='data'
        ),
        hovermode='closest',
        width=1200, height=800
    )
    fig.show()


def create_dash_app(t_arr, states_arr, command_arr, J_arr, params):
    """Full interactive Dash web dashboard with timeline slider."""
    if not DASH_AVAILABLE:
        print("Dash not installed. Run: pip install dash")
        return None

    app = dash.Dash(__name__)

    app.layout = html.Div([
        html.H1("SMC-IO Asteroid Landing Simulator - Interactive Dashboard"),
        html.Hr(),

        html.Div([
            html.Label("Timeline (drag to explore):"),
            dcc.Slider(
                id='time-slider',
                min=0, max=len(t_arr)-1, step=1,
                value=len(t_arr)-1,
                marks={i: f"{int(t_arr[i])}s" for i in range(0, len(t_arr), max(1, len(t_arr)//5))}
            ),
            html.H3(id='time-display', style={'color': 'darkblue'})
        ], style={'marginBottom': 20}),

        html.Div([
            dcc.Graph(id='trajectory-3d', style={'width': '49%', 'display': 'inline-block'}),
            dcc.Graph(id='state-plot', style={'width': '49%', 'display': 'inline-block', 'marginLeft': '2%'})
        ]),

        html.Div([
            dcc.Graph(id='control-effort', style={'width': '49%', 'display': 'inline-block'}),
            dcc.Graph(id='velocity-envelope', style={'width': '49%', 'display': 'inline-block', 'marginLeft': '2%'})
        ]),
    ], style={'fontFamily': 'Arial', 'marginLeft': 20, 'marginRight': 20})

    @app.callback(
        [Output('trajectory-3d', 'figure'),
         Output('state-plot', 'figure'),
         Output('control-effort', 'figure'),
         Output('velocity-envelope', 'figure'),
         Output('time-display', 'children')],
        Input('time-slider', 'value')
    )
    def update_all(idx):
        idx = min(idx, len(t_arr) - 1)
        current_time = t_arr[idx]
        dist = np.linalg.norm(states_arr[idx, 0:3] - params['rfd'])
        speed = np.linalg.norm(states_arr[idx, 3:6])
        mass = states_arr[idx, 6]

        # Trajectory 3D
        fig_traj = go.Figure()
        fig_traj.add_trace(go.Scatter3d(
            x=states_arr[:idx+1, 0], y=states_arr[:idx+1, 1], z=states_arr[:idx+1, 2],
            mode='lines+markers',
            line=dict(color='blue', width=3),
            marker=dict(size=4, color='blue')
        ))
        fig_traj.add_trace(go.Scatter3d(
            x=[states_arr[idx, 0]], y=[states_arr[idx, 1]], z=[states_arr[idx, 2]],
            mode='markers', marker=dict(size=10, color='orange')
        ))
        fig_traj.add_trace(go.Scatter3d(
            x=[params['rfd'][0]], y=[params['rfd'][1]], z=[params['rfd'][2]],
            mode='markers', marker=dict(size=8, color='red')
        ))
        fig_traj.update_layout(title="Trajectory", scene_aspectmode='data')

        # States over time
        fig_state = sp.make_subplots(rows=3, cols=1, subplot_titles=("Position", "Velocity", "Mass"))
        fig_state.add_trace(go.Scatter(x=t_arr[:idx+1], y=states_arr[:idx+1, 0], name='x'), row=1, col=1)
        fig_state.add_trace(go.Scatter(x=t_arr[:idx+1], y=states_arr[:idx+1, 1], name='y'), row=1, col=1)
        fig_state.add_trace(go.Scatter(x=t_arr[:idx+1], y=states_arr[:idx+1, 2], name='z'), row=1, col=1)
        fig_state.add_trace(go.Scatter(x=t_arr[:idx+1], y=states_arr[:idx+1, 3], name='vx'), row=2, col=1)
        fig_state.add_trace(go.Scatter(x=t_arr[:idx+1], y=states_arr[:idx+1, 4], name='vy'), row=2, col=1)
        fig_state.add_trace(go.Scatter(x=t_arr[:idx+1], y=states_arr[:idx+1, 5], name='vz'), row=2, col=1)
        fig_state.add_trace(go.Scatter(x=t_arr[:idx+1], y=states_arr[:idx+1, 6], name='mass', line=dict(color='black')), row=3, col=1)
        fig_state.update_yaxes(title_text="Position (m)", row=1, col=1)
        fig_state.update_yaxes(title_text="Velocity (m/s)", row=2, col=1)
        fig_state.update_yaxes(title_text="Mass (kg)", row=3, col=1)
        fig_state.update_xaxes(title_text="Time (s)", row=3, col=1)

        # Control effort
        fig_effort = go.Figure()
        fig_effort.add_trace(go.Scatter(x=command_arr[:idx+1, 0], y=command_arr[:idx+1, 4],
                                        mode='lines', name='Thrust', fill='tozeroy'))
        fig_effort.update_layout(title="Commanded Thrust", xaxis_title="Time (s)", yaxis_title="Thrust (N)")

        # Velocity envelope
        distances = np.linalg.norm(states_arr[:idx+1, 0:3] - params['rfd'], axis=1)
        velocities = np.linalg.norm(states_arr[:idx+1, 3:6], axis=1)
        fig_phase = go.Figure()
        fig_phase.add_trace(go.Scatter(x=distances, y=velocities, mode='lines+markers',
                                       marker=dict(size=4, color=t_arr[:idx+1], colorscale='Plasma', showscale=True),
                                       line=dict(color='darkblue')))
        fig_phase.update_layout(title="Phase Portrait", xaxis_title="Distance to Target (m)",
                               yaxis_title="Speed (m/s)")

        time_str = f"Time: {current_time:.1f} s  |  Distance: {dist:.3f} m  |  Speed: {speed:.6f} m/s  |  Mass: {mass:.2f} kg"

        return fig_traj, fig_state, fig_effort, fig_phase, time_str

    return app


def create_animation(t_arr, states_arr, command_arr, params, save_path=None):
    """Animated trajectory with distance and acceleration plots."""
    fig = plt.figure(figsize=(15, 5))

    ax1 = fig.add_subplot(131, projection='3d')
    ax2 = fig.add_subplot(132)
    ax3 = fig.add_subplot(133)

    line, = ax1.plot([], [], [], 'b-', linewidth=2)
    point, = ax1.plot([], [], [], 'ro', markersize=8)
    target, = ax1.plot([params['rfd'][0]], [params['rfd'][1]], [params['rfd'][2]], 'g*', markersize=15)

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Trajectory')

    def init():
        line.set_data([], [])
        line.set_3d_properties([])
        point.set_data([], [])
        point.set_3d_properties([])
        return line, point

    def animate(frame):
        idx = min(frame, len(states_arr) - 1)

        # Update 3D trajectory
        line.set_data(states_arr[:idx, 0], states_arr[:idx, 1])
        line.set_3d_properties(states_arr[:idx, 2])
        point.set_data([states_arr[idx, 0]], [states_arr[idx, 1]])
        point.set_3d_properties([states_arr[idx, 2]])

        # Update distance plot
        ax2.clear()
        distances = np.linalg.norm(states_arr[:idx, 0:3] - params['rfd'], axis=1)
        ax2.plot(t_arr[:idx], distances, 'b-', linewidth=2)
        ax2.axhline(y=0, color='r', linestyle='--', alpha=0.5)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Distance to Target (m)')
        ax2.set_title('Distance to Target')
        ax2.grid(True, alpha=0.3)
        ax2.set_yscale('log')

        # Update acceleration plot
        ax3.clear()
        acc_mag = np.sqrt(command_arr[:idx, 1]**2 + command_arr[:idx, 2]**2 + command_arr[:idx, 3]**2)
        ax3.plot(command_arr[:idx, 0], acc_mag, 'g-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('|a| (m/s²)')
        ax3.set_title('Control Acceleration')
        ax3.grid(True, alpha=0.3)

        return line, point

    anim = FuncAnimation(fig, animate, init_func=init, frames=len(states_arr),
                        interval=50, blit=False, repeat=True)

    if save_path:
        anim.save(save_path, writer='ffmpeg', fps=20)
        print(f"Animation saved to {save_path}")

    plt.tight_layout()
    plt.show()


def plot_control_heatmap(t_arr, states_arr, command_arr, params):
    """Heatmaps and phase portraits - control authority visualization."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Control effort in each direction
    for i, axis in enumerate(axes.flat[:3]):
        distances = np.linalg.norm(states_arr[:, 0:3] - params['rfd'], axis=1)
        control_mag = np.abs(command_arr[:, 1+i])

        scatter = axis.scatter(distances, control_mag, c=t_arr, cmap='plasma', s=20, alpha=0.6)
        axis.set_xlabel('Distance to Target (m)')
        axis.set_ylabel(f'|a{chr(120+i)}| (m/s²)')
        axis.set_title(f'Control Authority - Axis {chr(120+i)}')
        plt.colorbar(scatter, ax=axis, label='Time (s)')
        axis.grid(True, alpha=0.3)
        axis.set_xscale('log')

    # Phase portrait: distance vs velocity
    ax = axes.flat[3]
    distances = np.linalg.norm(states_arr[:, 0:3] - params['rfd'], axis=1)
    velocities = np.linalg.norm(states_arr[:, 3:6], axis=1)
    scatter = ax.scatter(distances, velocities, c=t_arr, cmap='plasma', s=20, alpha=0.6)
    ax.set_xlabel('Distance to Target (m)')
    ax.set_ylabel('Speed (m/s)')
    ax.set_title('Phase Portrait: Distance vs Velocity')
    plt.colorbar(scatter, ax=ax, label='Time (s)')
    ax.grid(True, alpha=0.3)
    ax.set_xscale('log')
    ax.set_yscale('log')

    plt.tight_layout()
    plt.show()


def compare_all_scenarios(verbose=False):
    """Run all 7 landing scenarios and compare performance."""
    results = {}
    metrics = {}

    print("\n" + "="*70)
    print("Running all 7 landing scenarios...")
    print("="*70)

    for ls in range(1, 8):
        print(f"\nRunning LS{ls}...", end=" ", flush=True)
        t, s, c, j, p = simulate(ls_number=ls, verbose=False)
        results[ls] = {'t': t, 's': s, 'c': c, 'j': j, 'p': p}

        if len(s) > 0:
            rf = s[-1, 0:3]
            vf = s[-1, 3:6]
            mf = s[-1, 6]
            dist_error = np.linalg.norm(rf - p['rfd'])
            speed = np.linalg.norm(vf)
            fuel = p['m0'] - mf
            total_effort = j[-1, 1]

            metrics[ls] = {
                'final_time': t[-1],
                'distance_error': dist_error,
                'landing_speed': speed,
                'fuel_used': fuel,
                'control_effort': total_effort
            }
            print(f"✓ (t={t[-1]:.0f}s, err={dist_error:.3f}m, v={speed:.6f}m/s)")
        else:
            print("✗ Failed")

    # Comparison plots
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    fig.suptitle('Comparison of All 7 Landing Scenarios', fontsize=16, fontweight='bold')

    # Distance profiles
    ax = axes[0, 0]
    for ls in range(1, 8):
        if ls in results:
            r = results[ls]
            distances = np.linalg.norm(r['s'][:, 0:3] - r['p']['rfd'], axis=1)
            ax.semilogy(r['t'], distances, linewidth=2, label=f'LS{ls}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (m) [log scale]')
    ax.set_title('Distance to Target Over Time')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')

    # Velocity profiles
    ax = axes[0, 1]
    for ls in range(1, 8):
        if ls in results:
            r = results[ls]
            velocities = np.linalg.norm(r['s'][:, 3:6], axis=1)
            ax.semilogy(r['t'], velocities, linewidth=2, label=f'LS{ls}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (m/s) [log scale]')
    ax.set_title('Spacecraft Speed Over Time')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')

    # Control effort
    ax = axes[0, 2]
    for ls in range(1, 8):
        if ls in results:
            r = results[ls]
            ax.plot(r['j'][:, 0], r['j'][:, 1], linewidth=2, label=f'LS{ls}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Cumulative Control Effort')
    ax.set_title('Control Effort Accumulation')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')

    # Bar charts for metrics
    ls_list = list(metrics.keys())

    ax = axes[1, 0]
    final_times = [metrics[ls]['final_time'] for ls in ls_list]
    ax.bar(ls_list, final_times, color='steelblue')
    ax.set_xlabel('Landing Scenario')
    ax.set_ylabel('Final Time (s)')
    ax.set_title('Landing Time Comparison')
    ax.grid(True, alpha=0.3, axis='y')

    ax = axes[1, 1]
    errors = [metrics[ls]['distance_error'] for ls in ls_list]
    ax.bar(ls_list, errors, color='coral')
    ax.set_xlabel('Landing Scenario')
    ax.set_ylabel('Position Error (m)')
    ax.set_title('Landing Accuracy Comparison')
    ax.grid(True, alpha=0.3, axis='y')

    ax = axes[1, 2]
    fuel = [metrics[ls]['fuel_used'] for ls in ls_list]
    ax.bar(ls_list, fuel, color='lightgreen')
    ax.set_xlabel('Landing Scenario')
    ax.set_ylabel('Fuel Used (kg)')
    ax.set_title('Fuel Consumption Comparison')
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.show()

    # Print metrics table
    print("\n" + "="*70)
    print("Summary Metrics for All Scenarios")
    print("="*70)
    print(f"{'LS':<4} {'Time(s)':<10} {'Error(m)':<12} {'Speed(m/s)':<14} {'Fuel(kg)':<10} {'Effort':<10}")
    print("-"*70)
    for ls in sorted(metrics.keys()):
        m = metrics[ls]
        print(f"{ls:<4} {m['final_time']:<10.1f} {m['distance_error']:<12.6f} "
              f"{m['landing_speed']:<14.9f} {m['fuel_used']:<10.4f} {m['control_effort']:<10.4f}")
    print("="*70)

    return results, metrics


# =============================================================================
# Entry point
# =============================================================================

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="SMC-IO Guidance for asteroid soft landing"
    )
    parser.add_argument(
        "--scenario", type=int, default=1, choices=range(1, 8),
        help="Landing scenario number (1-7, default: 1)"
    )
    parser.add_argument(
        "--mode", type=str, default="standard",
        choices=["standard", "interactive", "dashboard", "animate", "heatmap", "compare"],
        help="Visualization mode (default: standard)"
    )
    parser.add_argument(
        "--no-plot", action="store_true",
        help="Disable plotting (just run simulation)"
    )
    parser.add_argument(
        "--save", type=str, default=None,
        help="Save plots/animation with this filename prefix"
    )
    args = parser.parse_args()

    if args.mode == "compare":
        results, metrics = compare_all_scenarios(verbose=False)
    else:
        t_arr, states_arr, command_arr, J_arr, params = simulate(
            ls_number=args.scenario, verbose=True
        )

        if not args.no_plot:
            if args.mode == "standard":
                plot_results(t_arr, states_arr, command_arr, J_arr, params,
                           save_prefix=args.save)
            elif args.mode == "interactive":
                plot_interactive_3d(t_arr, states_arr, command_arr, params)
            elif args.mode == "dashboard":
                app = create_dash_app(t_arr, states_arr, command_arr, J_arr, params)
                if app:
                    app.run(debug=True, port=8050)
            elif args.mode == "animate":
                save_as = f"{args.save}_animation.mp4" if args.save else None
                create_animation(t_arr, states_arr, command_arr, params, save_path=save_as)
            elif args.mode == "heatmap":
                plot_control_heatmap(t_arr, states_arr, command_arr, params)


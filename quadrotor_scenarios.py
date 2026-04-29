"""
Scenarios Configuration for Quadrotor SMC-IO Guidance
======================================================

Mirrors the structure of scenarios.py but with quadrotor-specific defaults:
  • Earth gravity (g = 9.81 m/s²)
  • Aerodynamic drag (Cd, A)
  • Wind parameters
  • Obstacle definitions

Four pre-defined scenarios:
  1. Point-to-Point Calm
  2. Windy Delivery
  3. Urban Obstacle Course
  4. High-Altitude Density Change

Usage:
    from quadrotor_scenarios import build_params, list_scenarios
    params = build_params(1)
"""

import numpy as np
from atmosphere_model import make_obstacle

# =============================================================================
# Default Quadrotor & Environment Parameters
# =============================================================================

DEFAULT_PARAMS = {
    # Simulation time
    "t0": 0.0,
    "tf": 200.0,
    "dt": 0.02,

    # Earth gravity  (NED-like but z-up: gravity points down)
    "g_vec": np.array([0.0, 0.0, -9.81]),

    # Quadrotor physical properties
    "Tmax": 60.0,          # Max total thrust (N) — about 2.0 g for 2.5 kg
    "m0":   3.5,           # Initial mass (kg)
    "c":    30000000000000000000000.0,        # Effective exhaust velocity (electric → large c, tiny mass loss)

    # Aerodynamics
    "Cd": 1.0,             # Drag coefficient
    "A":  0.04,            # Frontal area (m²)

    # Wind parameters (default: calm)
    "wind": {
        "base":      np.array([0.0, 0.0, 0.0]),
        "gust_amp":  0.0,
        "gust_freq": 0.5,
        "gust_dir":  np.array([1.0, 0.0, 0.0]),
        "alt_scale": 100.0,
    },

    # Obstacles (default: none)
    "obstacles": [],
    "obstacle_margin": 5.0,
    "obstacle_k_rep": 50.0,

    # SMC-IO guidance parameters  (tuned for ~100 m scale manoeuvres)
    "Ee0": 2.0,
    "ke":  3.0,
    "ee_function": "quadratic_cubic",

    # Velocity profile shape parameters (same form as asteroid code Eq. 26)
    "bx": 15, "by": 15, "bz": 15,
    "cx": 0, "cy": 0, "cz": 0,

    # Final velocity target (hover at destination)
    "vfd": np.array([0.0, 0.0, 0.0]),
}

# =============================================================================
# Flight Scenarios
# =============================================================================

FLIGHT_SCENARIOS = {
    # FS1: Simple point-to-point, no wind, no obstacles
    1: {
        "name": "FS1 — Point-to-Point Calm",
        "pos0": np.array([0.0, 0.0, 10.0]),
        "vel0": np.array([5.0, 1.5, 2.0]),
        "rfd":  np.array([100.0, 20.0, 60.0]),
        "tf":   60.0,
    },

    # FS2: Flight through a steady crosswind + gusts
    2: {
        "name": "FS2 — Windy Delivery",
        "pos0": np.array([0.0, 0.0, 20.0]),
        "vel0": np.array([6.0, 1.0, 1.0]),
        "rfd":  np.array([200.0, 0.0, 20.0]),
        "tf":   80.0,
        "wind": {
            "base":      np.array([5.0, 2.0, 0.0]),   # 5 m/s headwind, 2 m/s crosswind
            "gust_amp":  3.0,                          # 3 m/s gust amplitude
            "gust_freq": 0.8,                          # rad/s
            "gust_dir":  np.array([0.0, 1.0, 0.0]),   # gust in y-direction
            "alt_scale": 50.0,
        },
    },

    # FS3: Slalom through three obstacles placed on ALTERNATE SIDES of the path.
    #
    # Design rules (key to avoid the local-minimum / infinite-repulsion trap):
    #   • Flight path:  pos0=[0,0,50] → rfd=[150,0,50]  (straight along x-axis at y=0,z=50)
    #   • Obstacle centre offset in y: ±7 m  (obstacles stagger left/right)
    #   • Radius: 4 m  →  surface at |y|=3 m  →  3 m clear corridor either side of path
    #   • margin=6: influence zone extends 6 m from surface, reaching y=|3−6|=−3 m
    #                (i.e. the influence zone just crosses the path at y=0)
    #   • At closest approach (drone at y=0 passing x=xobs):
    #       dist_to_surface = 3 m < margin=6  →  in zone
    #       penetration = 1/3 − 1/6 = 0.167
    #       force = k_rep × 0.167² ≈ 30 × 0.028 ≈ 0.83 m/s²  (pure lateral, y-axis)
    #   • This lateral push creates a gentle slalom without blocking x-progress.
    3: {
        "name": "FS3 — Urban Obstacle Course",
        "pos0": np.array([0.0,   0.0, 50.0]),
        # vel0 MUST NOT be perfectly aligned with (rfd - pos0) = [150,0,0].
        # A perfectly aligned vel0=[5,0,0] makes heading error Se=0 → all
        # SMC alpha-coefficients=0 → SLSQP equality degenerates to 0=0 →
        # cmd=[0,0,0] → free fall.  The y-component breaks this degeneracy.
        "vel0": np.array([5.0,   0.5,  0.0]),
        "rfd":  np.array([150.0, 0.0, 50.0]),
        "tf":   90.0,
        "obstacles": [
            make_obstacle([ 20.0,  7.0, 50.0], 15.0),   # right side  → pushes drone left
            make_obstacle([ 80.0, -7.0, 50.0], 5.0),   # left  side  → pushes drone right
            make_obstacle([120.0,  7.0, 50.0], 8.0),   # right side  → pushes drone left
        ],
        "obstacle_margin": 0.5,
        "obstacle_k_rep":  30.0,
    },

    # FS4: Climb to 200 m — ISA density drops ~2.5 % — tractable for SMC
    4: {
        "name": "FS4 — High-Altitude Density Change",
        "pos0": np.array([0.0, 0.0, 10.0]),
        "vel0": np.array([2.0, 1.0, 3.0]),
        "rfd":  np.array([80.0, 0.0, 200.0]),
        "tf":   120.0,
        "wind": {
            "base":      np.array([2.0, 0.0, 0.0]),
            "gust_amp":  1.5,
            "gust_freq": 0.3,
            "gust_dir":  np.array([1.0, 0.0, 0.0]),
            "alt_scale": 200.0,
        },
    },
}

FLIGHT_SCENARIOS_ap = {
    
    

    # FS3: Slalom through three obstacles placed on ALTERNATE SIDES of the path.
    #
    # Design rules (key to avoid the local-minimum / infinite-repulsion trap):
    #   • Flight path:  pos0=[0,0,50] → rfd=[150,0,50]  (straight along x-axis at y=0,z=50)
    #   • Obstacle centre offset in y: ±7 m  (obstacles stagger left/right)
    #   • Radius: 4 m  →  surface at |y|=3 m  →  3 m clear corridor either side of path
    #   • margin=6: influence zone extends 6 m from surface, reaching y=|3−6|=−3 m
    #                (i.e. the influence zone just crosses the path at y=0)
    #   • At closest approach (drone at y=0 passing x=xobs):
    #       dist_to_surface = 3 m < margin=6  →  in zone
    #       penetration = 1/3 − 1/6 = 0.167
    #       force = k_rep × 0.167² ≈ 30 × 0.028 ≈ 0.83 m/s²  (pure lateral, y-axis)
    #   • This lateral push creates a gentle slalom without blocking x-progress.
    3: {
        "name": "FS3 — Urban Obstacle Course",
        "pos0": np.array([0.0,   0.0, 50.0]),
        # vel0 MUST NOT be perfectly aligned with (rfd - pos0) = [150,0,0].
        # A perfectly aligned vel0=[5,0,0] makes heading error Se=0 → all
        # SMC alpha-coefficients=0 → SLSQP equality degenerates to 0=0 →
        # cmd=[0,0,0] → free fall.  The y-component breaks this degeneracy.
        "vel0": np.array([5.0,   0.5,  0.0]),
        "rfd":  np.array([150.0, 0.0, 50.0]),
        "tf":   90.0,
        "obstacles": [
            make_obstacle([ 40.0,  7.0, 50.0], 4.0),   # right side  → pushes drone left
            make_obstacle([ 80.0, -7.0, 50.0], 4.0),   # left  side  → pushes drone right
            make_obstacle([120.0,  7.0, 50.0], 4.0),   # right side  → pushes drone left
        ],
        "obstacle_margin": 0.5,
        "obstacle_k_rep":  30.0,
    },

}


# =============================================================================
# Public API
# =============================================================================

def get_scenario(scenario_id):
    """Return scenario dict by ID.  Raises KeyError if not found."""
    if scenario_id not in FLIGHT_SCENARIOS:
        available = sorted(FLIGHT_SCENARIOS.keys())
        raise KeyError(f"Scenario {scenario_id} not found. Available: {available}")
    return {k: (v.copy() if isinstance(v, np.ndarray) else v)
            for k, v in FLIGHT_SCENARIOS[scenario_id].items()}


def get_all_scenarios():
    return {k: get_scenario(k) for k in FLIGHT_SCENARIOS}


def list_scenarios():
    """Print all flight scenarios."""
    print("\n" + "=" * 70)
    print("Available Quadrotor Flight Scenarios")
    print("=" * 70)
    for sid, sc in sorted(FLIGHT_SCENARIOS.items()):
        print(f"\n  FS{sid}: {sc['name']}")
        print(f"    Start    : {sc['pos0']}")
        print(f"    Velocity : {sc['vel0']}")
        print(f"    Target   : {sc['rfd']}")
        n_obs = len(sc.get("obstacles", []))
        has_wind = np.linalg.norm(sc.get("wind", {}).get("base", np.zeros(3))) > 0
        print(f"    Wind     : {'Yes' if has_wind else 'No'}")
        print(f"    Obstacles: {n_obs}")
    print("\n" + "=" * 70)


def build_params(scenario_id=1):
    """
    Build complete parameter dictionary for a quadrotor scenario.
    Merges DEFAULT_PARAMS with scenario-specific overrides.
    """
    scenario = get_scenario(scenario_id)

    params = {}
    # Deep-copy defaults (handle nested dicts / arrays)
    for k, v in DEFAULT_PARAMS.items():
        if isinstance(v, np.ndarray):
            params[k] = v.copy()
        elif isinstance(v, dict):
            params[k] = {kk: (vv.copy() if isinstance(vv, np.ndarray) else vv)
                         for kk, vv in v.items()}
        elif isinstance(v, list):
            params[k] = list(v)
        else:
            params[k] = v

    # Override with scenario values
    for k, v in scenario.items():
        if k == "wind" and isinstance(v, dict):
            params["wind"] = {kk: (vv.copy() if isinstance(vv, np.ndarray) else vv)
                              for kk, vv in v.items()}
        elif isinstance(v, np.ndarray):
            params[k] = v.copy()
        elif isinstance(v, list):
            params[k] = list(v)
        else:
            params[k] = v

    params["scenario_id"]   = scenario_id
    params["scenario_name"] = scenario["name"]
    return params


def modify_params(scenario_id=1, **kwargs):
    """Build params then override with keyword arguments."""
    params = build_params(scenario_id)
    params.update(kwargs)
    return params


def add_custom_scenario(scenario_id, name, pos0, vel0, rfd, **extra):
    """Add a custom flight scenario at runtime."""
    if scenario_id in FLIGHT_SCENARIOS:
        print(f"Warning: Scenario {scenario_id} already exists. Overwriting.")
    FLIGHT_SCENARIOS[scenario_id] = {
        "name": name,
        "pos0": np.array(pos0, dtype=float),
        "vel0": np.array(vel0, dtype=float),
        "rfd":  np.array(rfd,  dtype=float),
        **extra,
    }
    print(f"✓ Added custom scenario {scenario_id}: '{name}'")


# =============================================================================
# Entry point — print available scenarios
# =============================================================================

if __name__ == "__main__":
    list_scenarios()

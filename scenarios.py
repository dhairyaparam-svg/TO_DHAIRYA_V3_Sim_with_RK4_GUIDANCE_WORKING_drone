"""
Scenarios Configuration for SMC-IO Asteroid Landing Guidance
============================================================

This file contains all landing scenarios and parameters.
You can add your own scenarios here without modifying the main code.

Usage:
    from scenarios import get_scenario, add_custom_scenario
    params = get_scenario(1)  # Get scenario 1
    add_custom_scenario(8, {...})  # Add new scenario 8
"""

import numpy as np

# =============================================================================
# Default Asteroid & Spacecraft Parameters (Constant Across Scenarios)
# =============================================================================

DEFAULT_PARAMS = {
    # Simulation time
    "t0": 0.0,
    "tf": 1000.0,
    "dt": 1.0,
    
    # Asteroid gravity model (second-order spherical harmonics)
    "GM":      4.89256,      # Gravitational parameter
    "la":      1000.0,       # Semi-axis (x direction, m)
    "lb":      500.0,        # Semi-axis (y direction, m)
    "lc":      250.0,        # Semi-axis (z direction, m)
    "omg":     np.array([0.0, 0.0, 2.18e-4]),  # Angular velocity (rad/s)
    "omg_dot": np.array([0.0, 0.0, 0.0]),      # Angular acceleration
    
    # Spacecraft & engine
    "Tmax": 80.0,            # Max thrust (N)
    "m0":   1400.0,          # Initial mass (kg)
    "c":    2.206e3,         # Exhaust velocity (m/s)
    
    # SMC-IO guidance parameters
    "Ee0": 0.002,                       # Sliding mode constant (distance-dependent)
    "ke":  0.006,                       # Sliding mode parameter (proportional)
    "ee_function": "quadratic_cubic",   # Distance-based Ee computation function
    
    # Velocity profile shape parameters (Eq. 26)
    "bx": 9, "by": 8, "bz": 5,   # Linear terms
    "cx": 0, "cy": 1, "cz": 4,   # Quadratic terms
    
    # Final conditions (always zero velocity)
    "vfd": np.array([0.0, 0.0, 0.0]),
}

# =============================================================================
# Landing Scenarios - Table 2 from the Paper
# =============================================================================

LANDING_SCENARIOS = {
    # LS1: Equatorial landing
    1: {
        "name": "LS1 - Equatorial Landing",
        "pos0": np.array([1389.37, -60.78, -30.50]),
        "vel0": np.array([-2.00, -1.25, -0.50]),
        "rfd":  np.array([1000.00, 0.00, 0.00]),
    },
    
    # LS2: Mid-latitude 15°
    2: {
        "name": "LS2 - 15° Latitude",
        "pos0": np.array([1373.48, -60.08, 60.78]),
        "vel0": np.array([-1.75, -1.00, -0.50]),
        "rfd":  np.array([965.93, 0.00, 64.70]),
    },
    
    # LS3: 30° latitude
    3: {
        "name": "LS3 - 30° Latitude",
        "pos0": np.array([1264.00, -55.29, 147.92]),
        "vel0": np.array([-2.00, -1.50, 1.00]),
        "rfd":  np.array([866.03, 0.00, 125.00]),
    },
    
    # LS4: 45° latitude
    4: {
        "name": "LS4 - 45° Latitude",
        "pos0": np.array([1068.38, -46.74, 224.98]),
        "vel0": np.array([-2.00, -0.75, 0.75]),
        "rfd":  np.array([707.11, 0.00, 176.78]),
    },
    
    # LS5: 60° latitude
    5: {
        "name": "LS5 - 60° Latitude",
        "pos0": np.array([799.95, -34.99, 286.70]),
        "vel0": np.array([-2.00, -0.50, 0.75]),
        "rfd":  np.array([500.00, 0.00, 216.51]),
    },
    
    # LS6: 75° latitude
    6: {
        "name": "LS6 - 75° Latitude",
        "pos0": np.array([477.00, -20.86, 328.89]),
        "vel0": np.array([-2.50, -0.60, 1.25]),
        "rfd":  np.array([258.82, 0.00, 241.48]),
    },
    
    # LS7: Polar landing (90° latitude)
    7: {
        "name": "LS7 - Polar Landing (90°)",
        "pos0": np.array([121.55, -5.32, 348.67]),
        "vel0": np.array([-1.00, -0.95, -1.35]),
        "rfd":  np.array([0.00, 0.00, 250.00]),
    },
}


# =============================================================================
# Public API Functions
# =============================================================================

def get_scenario(scenario_id):
    """
    Get a landing scenario by ID.
    
    Parameters
    ----------
    scenario_id : int
        Scenario number (1-7 or custom defined)
    
    Returns
    -------
    dict
        Landing scenario data (pos0, vel0, rfd, name)
    
    Raises
    ------
    KeyError
        If scenario ID not found
    
    Example
    -------
    >>> ls = get_scenario(1)
    >>> print(ls['name'])
    'LS1 - Equatorial Landing'
    """
    if scenario_id not in LANDING_SCENARIOS:
        available = sorted(LANDING_SCENARIOS.keys())
        raise KeyError(
            f"Scenario {scenario_id} not found. "
            f"Available scenarios: {available}"
        )
    return LANDING_SCENARIOS[scenario_id].copy()


def get_all_scenarios():
    """Return all landing scenarios as a dictionary."""
    return {k: v.copy() for k, v in LANDING_SCENARIOS.items()}


def list_scenarios():
    """Print all available scenarios."""
    print("\n" + "="*70)
    print("Available Landing Scenarios")
    print("="*70)
    for sid, scenario in sorted(LANDING_SCENARIOS.items()):
        print(f"\n  LS{sid}: {scenario['name']}")
        print(f"    Initial pos:  {scenario['pos0']}")
        print(f"    Initial vel:  {scenario['vel0']}")
        print(f"    Landing site: {scenario['rfd']}")
    print("\n" + "="*70)


def add_custom_scenario(scenario_id, name, pos0, vel0, rfd):
    """
    Add a custom landing scenario.
    
    Parameters
    ----------
    scenario_id : int
        New scenario ID (recommend 8+)
    name : str
        Descriptive name for the scenario
    pos0 : array-like
        Initial position [x, y, z] (m)
    vel0 : array-like
        Initial velocity [vx, vy, vz] (m/s)
    rfd : array-like
        Final desired position [x, y, z] (m)
    
    Example
    -------
    >>> add_custom_scenario(
    ...     8,
    ...     "My Custom Landing",
    ...     pos0=[1500, -100, -50],
    ...     vel0=[-2.5, -1.0, -0.5],
    ...     rfd=[1000, 0, 0]
    ... )
    >>> params = build_params(8)
    """
    if scenario_id in LANDING_SCENARIOS:
        print(f"Warning: Scenario {scenario_id} already exists. Overwriting...")
    
    LANDING_SCENARIOS[scenario_id] = {
        "name": name,
        "pos0": np.array(pos0, dtype=float),
        "vel0": np.array(vel0, dtype=float),
        "rfd":  np.array(rfd, dtype=float),
    }
    print(f"\n✓ Added custom scenario {scenario_id}: '{name}'")


def build_params(scenario_id=1):
    """
    Build complete parameter dictionary for a scenario.
    
    Combines DEFAULT_PARAMS with scenario initial/final conditions.
    
    Parameters
    ----------
    scenario_id : int
        Landing scenario ID
    
    Returns
    -------
    dict
        Complete parameter dictionary for simulation
    
    Example
    -------
    >>> params = build_params(1)
    >>> print(params['pos0'])  # Initial position
    [1389.37  -60.78  -30.5 ]
    """
    scenario = get_scenario(scenario_id)
    
    params = DEFAULT_PARAMS.copy()
    params.update({
        "pos0": scenario["pos0"].copy(),
        "vel0": scenario["vel0"].copy(),
        "rfd":  scenario["rfd"].copy(),
        "scenario_id": scenario_id,
        "scenario_name": scenario["name"],
    })
    return params


def modify_params(scenario_id=1, **kwargs):
    """
    Build parameters and override specific values.
    
    Parameters
    ----------
    scenario_id : int
        Landing scenario ID
    **kwargs
        Any parameter to override (e.g., Tmax=100, ke=0.01)
    
    Returns
    -------
    dict
        Modified parameter dictionary
    
    Example
    -------
    >>> params = modify_params(1, Tmax=100, m0=1500)
    >>> params['Tmax']
    100
    """
    params = build_params(scenario_id)
    params.update(kwargs)
    return params


def get_presets():
    """
    Return common parameter presets for experimentation.
    
    Returns
    -------
    dict
        Dictionary of parameter presets
    """
    return {
        "low_thrust": {
            "Tmax": 30.0,
            "ke": 0.00225,  # Lower for reduced thrust
        },
        "high_thrust": {
            "Tmax": 150.0,
            "ke": 0.01,  # Higher for more agile control
        },
        "aggressive_control": {
            "Ee0": 0.005,
            "ke": 0.015,
        },
        "conservative_control": {
            "Ee0": 0.001,
            "ke": 0.003,
        },
        "long_gravity_field": {
            "GM": 4.89256 * 1.5,
            "la": 1200, "lb": 600, "lc": 300,
        },
        "fast_rotating": {
            "omg": np.array([0.0, 0.0, 5.0e-4]),
        },
    }


def get_smc_profiles():
    """
    Return sliding mode control (SMC) parameter presets for guidance tuning.
    
    Sliding mode control uses two key parameters:
    - Ee0: Distance-dependent reaching law coefficient (affects approach phase)
    - ke:  Proportional reaching law coefficient (affects convergence speed)
    
    Returns
    -------
    dict
        Dictionary of SMC parameter profiles with Ee0 and ke values
        
    Notes
    -----
    These profiles are designed to explore the guidance authority vs precision 
    tradeoff. Larger values lead to faster convergence but higher acceleration 
    demands and thrust requirements.
    
    Profile Characteristics:
    - conservative_reaching: Smooth, low-authority guidance (fuel-efficient, slower)
    - moderate_reaching:     Balanced approach (default, designed for LS scenarios)
    - aggressive_reaching:   High-authority guidance (fuel-intensive, faster)
    - high_reaching:         Maximum control authority (experimental, extreme demands)
    
    Example
    -------
    >>> profiles = get_smc_profiles()
    >>> params = modify_params(1, **profiles["aggressive_reaching"])
    >>> # Now simulate with Ee0=0.005, ke=0.015 on scenario LS1
    """
    return {
        "conservative_reaching": {
            "Ee0": 0.0008,
            "ke": 0.0024,
            "description": "Smooth, low-authority guidance for fuel efficiency",
        },
        "moderate_reaching": {
            "Ee0": 0.002,
            "ke": 0.006,
            "description": "Balanced approach (paper default for LS scenarios)",
        },
        "aggressive_reaching": {
            "Ee0": 0.005,
            "ke": 0.015,
            "description": "High-authority guidance for rapid convergence",
        },
        "high_reaching": {
            "Ee0": 0.0125,
            "ke": 0.035,
            "description": "Maximum control authority (extreme acceleration demands)",
        },
    }


def get_ee_functions():
    """
    Return different distance-based Ee computation function names for testing.
    
    Each function defines how the distance-dependent sliding mode reaching law 
    coefficient (Ee) varies with distance from target. Testing different formulas
    helps identify which optimizes trajectory efficiency best.
    
    Returns
    -------
    dict
        Dictionary of Ee function identifiers with descriptions
        
    Notes
    -----
    The Ee coefficient controls distance-dependent reaching dynamics in the 
    sliding mode control law: s_e_dot = -ke*s_e - Ee*sign(s_e)
    
    Available functions:
    - "quadratic_cubic": Current formula (balanced quadratic + cubic terms)
    - "linear": Simple linear distance scaling
    - "quadratic": Pure quadratic distance dependence
    - "cubic": Pure cubic distance dependence  
    - "exponential": Exponential distance scaling (steep near target)
    - "saturation": Saturated (constant beyond threshold)
    
    Usage
    -----
    >>> ee_funcs = get_ee_functions()
    >>> params = build_params(1, ee_function="quadratic")
    >>> # Simulator will use quadratic Ee function instead of default
    
    Example
    -------
    >>> # Test all Ee functions on LS1
    >>> from scenarios import build_params
    >>> ee_funcs = get_ee_functions()
    >>> 
    >>> for func_name in ee_funcs.keys():
    ...     params = build_params(1, ee_function=func_name)
    ...     # Use params in simulation to compare fuel, precision, convergence
    """
    return {
        "quadratic_cubic": {
            "name": "Quadratic + Cubic (Default)",
            "description": "Paper formula: Ee = 0.5*(Ee0/r0²)*r² + 0.5*(Ee0/r0³)*r³",
            "formula": "balanced polynomial (smooth)",
        },
        "linear": {
            "name": "Linear Distance Scaling",
            "description": "Ee scales linearly with distance: Ee = (Ee0/r0)*r",
            "formula": "linear (aggressive near target)",
        },
        "quadratic": {
            "name": "Quadratic Distance Scaling",
            "description": "Ee = (Ee0/r0²)*r² (pure quadratic)",
            "formula": "polynomial degree 2",
        },
        "cubic": {
            "name": "Cubic Distance Scaling",
            "description": "Ee = (Ee0/r0³)*r³ (pure cubic, aggressive scaling)",
            "formula": "polynomial degree 3",
        },
        "exponential": {
            "name": "Exponential Distance Scaling",
            "description": "Ee = Ee0*(exp(r/r0) - 1) (steep near target)",
            "formula": "exponential (high authority far, smooth near)",
        },
        "saturation": {
            "name": "Saturation/Constant",
            "description": "Ee = Ee0 for all r (constant reaching gain, no distance dependence)",
            "formula": "constant (robust, equal authority)",
        },
    }


# =============================================================================
# Example: How to Create New Scenarios
# =============================================================================

"""
EXAMPLE 1: Add a custom scenario manually
-------------------------------------------

from scenarios import add_custom_scenario, build_params

# Define your custom landing scenario
add_custom_scenario(
    scenario_id=8,
    name="Custom High-Altitude Landing",
    pos0=[1600, -100, -50],
    vel0=[-3.0, -1.5, -0.75],
    rfd=[1000, 0, 0]
)

# Use it
params = build_params(8)


EXAMPLE 2: Modify existing scenario
------------------------------------

from scenarios import modify_params

# Take LS1 but with stronger thrusters
params = modify_params(1, Tmax=120, ke=0.008)


EXAMPLE 3: Use presets
-----------------------

from scenarios import build_params, modify_params, get_presets

params = build_params(1)
presets = get_presets()

# Apply aggressive control to LS1
params.update(presets["aggressive_control"])


EXAMPLE 4: Vary sliding mode control parameters
------------------------------------------------

from scenarios import modify_params, get_smc_profiles

# Get available SMC tuning profiles
smc_profiles = get_smc_profiles()

# Apply conservative reaching law to LS2 (fuel-efficient)
params_conservative = modify_params(2, **smc_profiles["conservative_reaching"])

# Apply aggressive reaching law to LS2 (faster convergence, higher fuel cost)
params_aggressive = modify_params(2, **smc_profiles["aggressive_reaching"])

# Combine SMC tuning with other modifications
params_combined = modify_params(1, 
    Tmax=100,  # Reduce thrust
    **smc_profiles["moderate_reaching"]  # Use moderate SMC control
)


EXAMPLE 5: Vary distance-based Ee computation function
------------------------------------------------------

from scenarios import modify_params, get_ee_functions

# Get all available distance-based Ee functions
ee_funcs = get_ee_functions()

# Test linear Ee function on LS1
params_linear = modify_params(1, ee_function="linear")

# Test exponential Ee function on LS3 (steep authority near target)
params_exponential = modify_params(3, ee_function="exponential")

# Test saturation (constant) Ee function on LS2 (equal authority everywhere)
params_saturation = modify_params(2, ee_function="saturation")

# Combine Ee function change with SMC profile adjustment
from scenarios import get_smc_profiles
params_combo = modify_params(1, 
    ee_function="exponential",
    **get_smc_profiles()["aggressive_reaching"]
)


EXAMPLE 6: Compare all Ee functions on same scenario
-----------------------------------------------------

from scenarios import modify_params, get_ee_functions

ee_funcs = get_ee_functions()

results = {}
for func_name, func_info in ee_funcs.items():
    params = modify_params(1, ee_function=func_name)
    results[func_name] = params
    print(f"{func_name}: {func_info['description']}")
    # Use params in simulation to compare trajectories


EXAMPLE 7: Systematically test Ee functions vs SMC profiles
----------------------------------------------------------

from scenarios import modify_params, get_ee_functions, get_smc_profiles

ee_funcs = get_ee_functions()
smc_profiles = get_smc_profiles()

# Test all combinations on scenario LS1
experiment_matrix = {}
for func_name in ee_funcs.keys():
    experiment_matrix[func_name] = {}
    for profile_name, profile_params in smc_profiles.items():
        params = modify_params(1, 
            ee_function=func_name, 
            **profile_params
        )
        experiment_matrix[func_name][profile_name] = params
        # Use params in simulation

# This creates a matrix:
# - 6 Ee functions × 4 SMC profiles = 24 scenarios to test




EXAMPLE 4: Create scenario for experimentation
-----------------------------------------------

from scenarios import add_custom_scenario, build_params

# Low initial velocity scenario
add_custom_scenario(
    9,
    "Low Initial Velocity Test",
    pos0=[1200, -50, -25],
    vel0=[-0.5, -0.25, -0.1],  # Very low initial velocity
    rfd=[1000, 0, 0]
)

params = build_params(9)

"""

if __name__ == "__main__":
    # Print all available scenarios
    list_scenarios()
    
    # Example: Add custom scenario and display it
    print("\n" + "="*70)
    print("Example: Adding Custom Scenario")
    print("="*70)
    
    add_custom_scenario(
        8,
        "My Custom Landing",
        pos0=[1500, -100, -50],
        vel0=[-2.5, -1.0, -0.5],
        rfd=[1000, 0, 0]
    )
    
    # Display it
    ls8 = get_scenario(8)
    print(f"\nScenario 8 Details:")
    print(f"  Name: {ls8['name']}")
    print(f"  pos0: {ls8['pos0']}")
    print(f"  vel0: {ls8['vel0']}")
    print(f"  rfd:  {ls8['rfd']}")
    
    # Show presets
    print("\n" + "="*70)
    print("Available Parameter Presets")
    print("="*70)
    presets = get_presets()
    for preset_name, preset_values in presets.items():
        print(f"\n  {preset_name}:")
        for key, val in preset_values.items():
            print(f"    {key}: {val}")

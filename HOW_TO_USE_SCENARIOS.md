# How To Use Scenarios.py

Custom scenarios are now managed in a separate `scenarios.py` file. You don't need to modify the main code anymore!

## Quick Start

### 1. View All Available Scenarios

```bash
python scenarios.py
```

This will display:
- All 7 default scenarios (LS1-LS7) **
- Their initial positions, velocities, and landing sites

### 2. Run a Simulation with Default Scenario

```bash
python smc_io_guidance.py --scenario 1 --mode interactive
```

### 3. Add Your Own Scenario

Open **scenarios.py** and edit the `LANDING_SCENARIOS` dictionary:

```python
# Add after LS7:
8: {
    "name": "My Custom Landing",
    "pos0": np.array([1500, -100, -50]),
    "vel0": np.array([-2.5, -1.0, -0.5]),
    "rfd":  np.array([1000, 0, 0]),
},
```

Then run:
```bash
python smc_io_guidance.py --scenario 8 --mode interactive
```

---

## Scenarios.py API

### Adding Scenarios Programmatically

```python
from scenarios import add_custom_scenario, build_params

# Add a new scenario
add_custom_scenario(
    scenario_id=8,
    name="Low Gravity Landing",
    pos0=[1200, -50, -25],
    vel0=[-1.5, -0.75, -0.25],
    rfd=[1000, 0, 0]
)

# Use it
params = build_params(8)
```

### Getting Specific Scenarios

```python
from scenarios import get_scenario

ls1 = get_scenario(1)
print(ls1['name'])        # "LS1 - Equatorial Landing"
print(ls1['pos0'])        # Initial position
print(ls1['vel0'])        # Initial velocity
print(ls1['rfd'])         # Landing target
```

### Modifying Parameters

```python
from scenarios import modify_params

# Take LS1 but with stronger thrusters
params = modify_params(1, Tmax=120, ke=0.008)

# Take LS1 but with different mass
params = modify_params(1, m0=1800)
```

### Using Parameter Presets

```python
from scenarios import build_params, get_presets

params = build_params(1)
presets = get_presets()

# Apply aggressive control
params.update(presets["aggressive_control"])

# Available presets:
# - low_thrust
# - high_thrust
# - aggressive_control
# - conservative_control
# - long_gravity_field
# - fast_rotating
```

---

## File Structure

```
smc_io_guidance.py   ← Main simulation (unchanged)
scenarios.py         ← All landing scenarios (EDIT THIS!)
```

### Did I need to modify main code?
**No!** All your scenarios go in `scenarios.py`.

---

## Examples

### Example 1: Create Low-Velocity Scenario

```python
# In Python or in scenarios.py

from scenarios import add_custom_scenario, build_params

add_custom_scenario(
    9,
    "Low Initial Velocity Test",
    pos0=[1200, -50, -25],
    vel0=[-0.5, -0.25, -0.1],    # Very low
    rfd=[1000, 0, 0]
)

# Use it
python smc_io_guidance.py --scenario 9 --mode interactive
```

### Example 2: Create High-Altitude Landing

```python
add_custom_scenario(
    10,
    "High Altitude",
    pos0=[1800, -150, -75],
    vel0=[-3.0, -1.5, -0.75],
    rfd=[1000, 0, 0]
)
```

### Example 3: Create Polar Region Landing

```python
add_custom_scenario(
    11,
    "Near Polar (80°)",
    pos0=[200, -20, 400],
    vel0=[-0.5, -0.5, -1.5],
    rfd=[0, 0, 250]
)
```

### Example 4: Create Scenario with Different Asteroid

```python
# Scenario with different asteroid parameters
params = modify_params(
    1,
    GM=5.5,              # Stronger gravity
    la=1200, lb=600, lc=300,  # Larger asteroid
)
```

---

## Default Parameters (Always Applied)

Every scenario uses these default spacecraft/asteroid parameters:

```python
{
    "t0": 0.0,
    "tf": 1000.0,
    "dt": 1.0,
    
    # Asteroid
    "GM":      4.89256,
    "la":      1000.0,
    "lb":      500.0,
    "lc":      250.0,
    "omg":     np.array([0.0, 0.0, 2.18e-4]),
    "omg_dot": np.array([0.0, 0.0, 0.0]),
    
    # Spacecraft
    "Tmax": 80.0,
    "m0":   1400.0,
    "c":    2.206e3,
    
    # SMC-IO
    "Ee0": 0.002,
    "ke":  0.006,
    
    # Velocity profile
    "bx": 9, "by": 8, "bz": 5,
    "cx": 0, "cy": 1, "cz": 4,
}
```

You can override any of these with `modify_params()`.

---

## Workflow Summary

| Task | Command |
|------|---------|
| View all scenarios | `python scenarios.py` |
| List available functions | `python -c "from scenarios import *; help(scenarios)"` |
| Run LS1 | `python smc_io_guidance.py --scenario 1` |
| Run custom LS8 | `python smc_io_guidance.py --scenario 8` |
| Add scenario #8 | Edit `scenarios.py` → add `8: {...}` dict entry |
| Add scenario in code | `from scenarios import add_custom_scenario; add_custom_scenario(8, ...)` |
| Compare all scenarios | `python smc_io_guidance.py --mode compare` |

---

## Tuning Sliding Mode Control (SMC) Parameters

The guidance uses **sliding mode control** with two tunable parameters that affect guidance authority and fuel consumption:

- **Ee0**: Distance-dependent coefficient (affects approach phase)
- **ke**: Proportional coefficient (affects convergence speed)

The `get_smc_profiles()` function provides preset tunings:

### 1. Use a Preset SMC Profile

```python
from scenarios import modify_params, get_smc_profiles

# Get all available profiles
profiles = get_smc_profiles()

# Apply conservative reaching (fuel-efficient, slower)
params = modify_params(1, **profiles["conservative_reaching"])

# Apply aggressive reaching (faster, higher fuel cost)
params = modify_params(1, **profiles["aggressive_reaching"])
```

### 2. Available SMC Profiles

| Profile | Ee0 | ke | Use Case |
|---------|-----|-----|----------|
| `conservative_reaching` | 0.0008 | 0.0024 | Fuel-efficient, smooth guidance |
| `moderate_reaching` | 0.002 | 0.006 | Balanced (paper default) |
| `aggressive_reaching` | 0.005 | 0.015 | Rapid convergence, higher fuel cost |
| `high_reaching` | 0.0125 | 0.035 | Maximum control authority (experimental) |

### 3. Combine SMC Tuning with Other Parameters

```python
from scenarios import modify_params, get_smc_profiles

profiles = get_smc_profiles()

# Test LS2 with conservative SMC and reduced thrust
params = modify_params(2, Tmax=50, **profiles["conservative_reaching"])

# Test LS3 with aggressive SMC and high thrust
params = modify_params(3, Tmax=150, **profiles["aggressive_reaching"])
```

### 4. Explore SMC Effects Programmatically

```python
from scenarios import modify_params, get_smc_profiles

profiles = get_smc_profiles()

for profile_name, smc_params in profiles.items():
    params = modify_params(1, **smc_params)
    print(f"{profile_name}: Ee0={params['Ee0']}, ke={params['ke']}")
    # Use params in your simulation
```

### 4. Create Custom SMC Tuning

```python
from scenarios import modify_params

# Manually set SMC parameters
params = modify_params(1, Ee0=0.003, ke=0.008)
```

---

## Testing Distance-Based Ee Functions

Beyond tuning `Ee0` and `ke` magnitudes, you can test different **distance-based Ee computation functions** to find which formula optimizes your trajectory best.

### 1. Available Ee Functions

| Function | Formula | Characteristics |
|----------|---------|-----------------|
| `quadratic_cubic` | 0.5*(Ee0/r0²)*r² + 0.5*(Ee0/r0³)*r³ | Balanced polynomial (default, paper formula) |
| `linear` | (Ee0/r0)*r | Linear scaling, aggressive near target |
| `quadratic` | (Ee0/r0²)*r² | Pure quadratic |
| `cubic` | (Ee0/r0³)*r³ | Pure cubic, steeper scaling |
| `exponential` | Ee0*(exp(r/r0) - 1) | High authority far, smooth near target |
| `saturation` | Ee0 | Constant (equal authority everywhere) |

### 2. Test Single Ee Function

```python
from scenarios import modify_params

# Test linear Ee function on LS1
params = modify_params(1, ee_function="linear")

# Test exponential on LS3
params = modify_params(3, ee_function="exponential")

# Test constant reaching gain (saturation)
params = modify_params(1, ee_function="saturation")
```

### 3. Combine Ee Function with SMC Tuning

```python
from scenarios import modify_params, get_smc_profiles

profiles = get_smc_profiles()

# Exponential Ee + aggressive SMC profile
params = modify_params(1,
    ee_function="exponential",
    **profiles["aggressive_reaching"]
)

# Linear Ee + conservative SMC profile
params = modify_params(1,
    ee_function="linear",
    **profiles["conservative_reaching"]
)
```

### 4. Systematic Testing Matrix

Test all Ee functions × all SMC profiles to find optimal combination:

```python
from scenarios import modify_params, get_ee_functions, get_smc_profiles

ee_funcs = get_ee_functions()
profiles = get_smc_profiles()

# Create experiment matrix (6 × 4 = 24 combinations)
for func_name in ee_funcs.keys():
    for profile_name, profile_params in profiles.items():
        params = modify_params(1,
            ee_function=func_name,
            **profile_params
        )
        # Simulate with params and record fuel, precision, convergence time
        print(f"Testing: {func_name} + {profile_name}")
```

### 5. View All Ee Functions

```python
from scenarios import get_ee_functions

ee_funcs = get_ee_functions()
for name, info in ee_funcs.items():
    print(f"{name}:")
    print(f"  {info['description']}")
```

---

## Troubleshooting

**Q: "Scenario 5 not found"**  
A: Make sure scenario ID exists in `scenarios.py`. Check `python scenarios.py` to list all.

**Q: How do I edit scenarios permanently?**  
A: Edit `scenarios.py` directly - add/modify the `LANDING_SCENARIOS` dictionary.

**Q: Can I mix multiple scenarios?**  
A: Use `modify_params()` to create hybrid scenarios from multiple sources.

**Q: Where do asteroid parameters go?**  
A: Leave unchanged in `scenarios.py` DEFAULT_PARAMS, or override with `modify_params()`.

---

## Key Files to Know

1. **scenarios.py** ← Your scenarios go here (EDIT THIS)
2. **smc_io_guidance.py** ← Main simulation (don't edit)
3. **smc_io_guidance.py** imports from **scenarios.py**

---

Need help? Run:
```bash
python scenarios.py --help
```

or in Python:
```python
from scenarios import *
print(__doc__)  # Full documentation
```

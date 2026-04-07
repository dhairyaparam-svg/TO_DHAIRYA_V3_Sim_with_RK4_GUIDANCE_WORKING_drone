# Ee Function Comparison for SMC-IO Guidance

To support your research paper submission, I have created a Python script named `generate_ee_comparisons.py`. This script performs a comprehensive comparative analysis of the Quadrotor SMC-IO guidance law under all distance-based $E_e$ reaching functions and across all available flight scenarios.

## What The Script Automates

The script loops through all predefined flight scenarios (FS1, FS2, FS3, FS4) and dynamically varies the `ee_function` parameter among the 6 combinations:
- `linear`
- `quadratic`
- `cubic`
- `exponential`
- `saturation`
- `quadratic_cubic`

For each scenario, it independently executes the full simulation in silent mode (`verbose=False`) for each of the $E_e$ variations and accumulates the response data.

## Output Plots

For each flight scenario, the script automatically generates a $2 \times 3$ grid of comparison plots (styled for research paper figures) named `FS<id>_Ee_Comparison.png`, which contain:
1. **Distance to Target (m)** vs Time
2. **Speed (m/s)** vs Time
3. **Control Effort** vs Time
4. **Commanded Thrust (N)** vs Time
5. **Estimated Energy Usage (Joules)** vs Time
6. **Altitude Profile (Z in m)** vs Time

The visualizations map each $E_e$ function to a separate line series within the subplot, making it straightforward to contrast the reaching behavior, termination precision, and effort optimality among the sliding mode methods.

## Example Usage

The script is currently executing automatically in your terminal background, testing all ~24 high-fidelity environment runs and systematically saving the high-resolution `.png` files to your workspace directory. 

You can rerun this whenever you update quadrotor parameters or constraints by running:
```bash
python generate_ee_comparisons.py
```

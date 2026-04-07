# GUI Launcher - Interactive Control Panel

## Quick Start

Launch the interactive control panel:

```bash
python gui_launcher.py
```

This opens a tabbed window where you can:
1. **Select Scenario** - Choose from LS1-LS7 or create custom scenarios
2. **Tune Parameters** - Adjust SMC control parameters with presets or manual sliders
3. **Select Ee Function** - Test different distance-based reaching functions
4. **Visualize & Run** - Execute simulation and view results
5. **View Results** - Display simulation output

---

## Working Flow

### Tab 1: Select Scenario

**Option A: Use Default Scenario**
- Dropdown menu shows LS1-LS7 scenarios from the research paper
- Click to select and see scenario details (initial position, velocity, landing site)
- All asteroid and spacecraft parameters automatically loaded

**Option B: Create Custom Scenario**
- Enter custom scenario ID (8 and above)
- Input initial position, velocity, and landing site
- Click "Create & Use Custom Scenario"

### Tab 2: Tune Parameters

**Option A: Quick Presets**
- Select from 4 SMC profiles:
  - `conservative_reaching` - Fuel efficient, smooth
  - `moderate_reaching` - Balanced (paper default)
  - `aggressive_reaching` - Rapid convergence
  - `high_reaching` - Maximum control authority
- Adjusts Ee0 and ke automatically

**Option B: Manual Tuning**
- `Ee0` slider: Distance-dependent coefficient (0.0001 - 0.02)
- `ke` slider: Proportional coefficient (0.001 - 0.05)
- `Max Thrust`: Engine power in Newtons
- `Initial Mass`: Spacecraft mass in kg

### Tab 3: Select Ee Function

Choose how distance affects the reaching law:

| Function | Formula | Use When |
|----------|---------|----------|
| **Quadratic+Cubic** (default) | 0.5*(Ee0/r0²)*r² + 0.5*(Ee0/r0³)*r³ | Following paper exactly |
| **Linear** | (Ee0/r0)*r | Need linear distance scaling |
| **Quadratic** | (Ee0/r0²)*r² | Need gentler authority increase |
| **Cubic** | (Ee0/r0³)*r³ | Need aggressive cubic scaling |
| **Exponential** | Ee0*(exp(r/r0) - 1) | Want steep authority near target |
| **Saturation** | Ee0 | Want constant authority everywhere |

### Tab 4: Visualize & Run

**Select Visualization Mode:**
- **Standard Plot** - Matplotlib trajectory plot
- **Interactive 3D** - Hover-enabled Plotly 3D visualization
- **Web Dashboard** - Interactive timeline slider (Dash web app)
- **Animation** - MP4 video of trajectory
- **Control Heatmap** - Authority vs distance heatmap
- **Compare All** - All 7 scenarios side-by-side

**Click "Run Simulation":**
- Simulation starts with your selected parameters
- Status updates show progress
- Once complete, visualization launches automatically

### Tab 5: Results

Displays:
- Landing precision (distance error)
- Final velocity (should be ~0)
- Fuel consumed
- Total time steps
- Other metrics

---

## Example Workflows

### Example 1: Test Conservative vs Aggressive SMC

1. **Tab 1:** Select LS1
2. **Tab 2:** Choose "conservative_reaching" profile
3. **Tab 3:** Select "quadratic_cubic" (default)
4. **Tab 4:** Choose "Interactive 3D" visualization
5. **Run** → See conservative trajectory
6. Repeat with "aggressive_reaching" to compare

### Example 2: Test All Ee Functions

1. **Tab 1:** Select LS3
2. **Tab 2:** Keep moderate profile
3. **Tab 3:** Select "exponential"
4. **Tab 4:** Choose "Control Heatmap"
5. **Run** → View control authority pattern
6. Repeat with different Ee functions (linear, cubic, saturation, etc.)

### Example 3: Custom Scenario with Fine Tuning

1. **Tab 1:** Create custom scenario with different initial state
2. **Tab 2:** Manually adjust Ee0=0.003, ke=0.008
3. **Tab 3:** Try "saturation" (constant control)
4. **Tab 4:** Run multiple times with different visualizations

---

## Tips & Tricks

- **Preset Combinations:** Use SMC profiles + Ee functions together for systematic testing
- **Compare Modes:** Use "Compare All Scenarios" to see how all 7 landing cases respond
- **Dashboard Mode:** Best for interactive exploration during tests
- **Animation Mode:** Good for presentations and publications
- **Heatmap Mode:** Best for understanding control authority distribution

---

## Troubleshooting

**Q: Window freezes when running simulation?**
- Simulation runs in background thread. Be patient, especially for animation mode.

**Q: Visualization doesn't appear?**
- Check that required packages are installed: `pip install plotly dash`
- Close any previous visualization windows first

**Q: Can't create custom scenario?**
- Make sure format is: `1500, -100, -50` (numbers separated by commas)
- Use scenario IDs ≥ 8

**Q: Dashboard takes forever to load?**
- Dashboard mode requires starting a web server. Give it 10-15 seconds.
- Check browser console (F12) for errors

---

## Parameters Reference

### Default Aerospace Parameters (Constant)

```
Asteroid:
  GM = 4.89256 (gravitational parameter)
  la = 1000 m, lb = 500 m, lc = 250 m (semi-axes)
  Angular velocity = [0, 0, 2.18e-4] rad/s

Spacecraft:
  Tmax = 80 N (default max thrust)
  m0 = 1400 kg (default initial mass)
  c = 2206 m/s (exhaust velocity)
```

### Tunable Parameters

```
SMC Control:
  Ee0: 0.0008 - 0.0125 (distance-dependent coefficient)
  ke: 0.0024 - 0.035 (proportional coefficient)
  
Engine:
  Tmax: 30 - 150 N
  m0: 1000 - 1500 kg
```

---

## Files Used

- `gui_launcher.py` - Main GUI application (this file)
- `smc_io_guidance.py` - Core simulation engine
- `scenarios.py` - Scenario database and parameter management

---

## Keyboard Shortcuts

- Tab between fields
- Enter to activate buttons
- Escape to close dialogs

---

## Notes

- All simulations use RK4 integration with 1-second timesteps
- SMC-IO algorithm per Shincy & Ghosh (2023) research paper
- Variable gravity model uses 2nd-order spherical harmonics
- Coriolis and centrifugal effects included for rotating asteroid frame

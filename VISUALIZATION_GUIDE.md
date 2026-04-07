# SMC-IO Asteroid Landing Guidance - Visualization Guide

## Quick Start

All visualization modes are now integrated into `smc_io_guidance.py`. Choose your preferred visualization:

### 1. **Standard Mode** (Default - Static Matplotlib Plots)
```bash
python smc_io_guidance.py --scenario 1
python smc_io_guidance.py --scenario 3 --save my_plots
```
**Features:**
- 3D trajectory
- 3×3 grid: position, velocity, acceleration
- Mass and thrust profiles
- Cumulative control effort

---

### 2. **Interactive 3D** (Plotly - Best for Quick Analysis)
```bash
python smc_io_guidance.py --scenario 1 --mode interactive
```
**Features:**
- Click and drag to rotate in 3D
- Hover to see: time, velocity, distance to target
- Color-coded by time progression
- Red marker = landing site, Green = start position

---

### 3. **Full Dashboard** (Dash Web App - Most Comprehensive)
```bash
python smc_io_guidance.py --scenario 1 --mode dashboard
```
Then open: **http://localhost:8050** in your browser

**Features:**
- Interactive timeline slider to replay mission
- 4 synchronized plots updating in real-time
- Position, velocity, mass, and thrust curves
- Phase portrait (distance vs speed)
- Control effort accumulated over time

**Dashboard Components:**
- **Top-left:** 3D Trajectory (live update with slider)
- **Top-right:** State variables (x,y,z,vx,vy,vz,m)
- **Bottom-left:** Thrust profile
- **Bottom-right:** Phase portrait with log-log scale

---

### 4. **Animated Trajectory** (Video - Great for Presentations)
```bash
python smc_io_guidance.py --scenario 1 --mode animate
python smc_io_guidance.py --scenario 1 --mode animate --save my_landing
```
**Features:**
- Side-by-side animation showing:
  - Left: 3D trajectory with spacecraft position
  - Middle: Distance to target (log scale)
  - Right: Control acceleration magnitude
- Saves as MP4 video (requires ffmpeg)

**Install ffmpeg:**
```bash
# Ubuntu/Debian
sudo apt install ffmpeg

# macOS
brew install ffmpeg

# Windows (chocolatey)
choco install ffmpeg
```

---

### 5. **Control Authority Heatmaps** (Advanced Analysis)
```bash
python smc_io_guidance.py --scenario 1 --mode heatmap
```
**Features:**
- **Top-left:** Control acceleration x vs distance
- **Top-middle:** Control acceleration y vs distance
- **Top-right:** Control acceleration z vs distance
- **Bottom:** Phase portrait with log-log axes
- Color = time progression
- Understand when/where guidance is active

---

### 6. **Compare All Scenarios** (Benchmark 7 Landing Scenarios)
```bash
python smc_io_guidance.py --mode compare
```
**Features:**
- Runs all 7 landing scenarios (LS1-LS7)
- Generates 6 comparison plots:
  - Distance profiles over time
  - Velocity profiles over time
  - Control effort accumulation
  - Bar charts: landing time, accuracy, fuel consumption
- Prints summary metrics table

**Output Table:**
```
LS  Time(s)     Error(m)      Speed(m/s)     Fuel(kg)   Effort
1   868.0       0.001         0.000016       2.1483     3.388
2   771.5       0.000         0.000005       1.8515     2.225
...
```

---

## Combining Options

```bash
# Run scenario 3 and save all plots
python smc_io_guidance.py --scenario 3 --mode standard --save scenario_3

# Skip visualization, just run simulation
python smc_io_guidance.py --scenario 5 --no-plot

# Compare all scenarios and save plots
python smc_io_guidance.py --mode compare  # Saves comparison figs

# Animate LS2 and output video
python smc_io_guidance.py --scenario 2 --mode animate --save LS2
```

---

## Installation Requirements

**Core:**
```bash
pip install numpy scipy matplotlib
```

**Interactive Visualizations:**
```bash
pip install plotly dash
```

**Animations (Optional):**
```bash
pip install matplotlib
# Also need ffmpeg system dependency
```

---

## Visualization Comparison Table

| Mode | Best For | Speed | Interactivity | Dependencies |
|------|----------|-------|---------------|--------------|
| **standard** | Saving static images | Fast | None | matplotlib |
| **interactive** | Exploring 3D trajectory | Very fast | High | plotly |
| **dashboard** | Mission replay & analysis | Medium | Very high | dash, plotly |
| **animate** | Presentations | Slow | None | matplotlib, ffmpeg |
| **heatmap** | Control authority analysis | Very fast | None | matplotlib |
| **compare** | Benchmarking all scenarios | Slow | None | matplotlib |

---

## Example Workflows

### Workflow 1: Quick Analysis of One Scenario
```bash
python smc_io_guidance.py --scenario 1 --mode interactive
# Explore 3D trajectory interactively
```

### Workflow 2: Detailed Mission Review
```bash
python smc_io_guidance.py --scenario 1 --mode dashboard
# Open http://localhost:8050
# Use slider to replay entire mission in detail
```

### Workflow 3: Full Benchmark Report
```bash
python smc_io_guidance.py --mode compare
# Get comparison metrics for all 7 scenarios
```

### Workflow 4: Generate Publication Figures
```bash
python smc_io_guidance.py --scenario 1 --mode standard --save paper_LS1
python smc_io_guidance.py --scenario 1 --mode heatmap  # Screenshot manually
```

### Workflow 5: Create Conference Video
```bash
python smc_io_guidance.py --scenario 1 --mode animate --save conference_demo
# Use conference_demo_animation.mp4 in presentation
```

---

## Customization

All visualization functions are in `smc_io_guidance.py`:

- `plot_results()` - Standard matplotlib plots
- `plot_interactive_3d()` - Plotly 3D visualization
- `create_dash_app()` - Dash dashboard
- `create_animation()` - Animated trajectory
- `plot_control_heatmap()` - Control authority heatmaps
- `compare_all_scenarios()` - Benchmark all 7 scenarios

Edit these functions directly to customize colors, sizes, scales, etc.

---

## Troubleshooting

**Q: Dashboard won't start?**
A: Make sure port 8050 isn't in use: `python smc_io_guidance.py --scenario 1 --mode dashboard`

**Q: Animation not saving?**
A: Install ffmpeg and ensure it's on PATH: `ffmpeg -version`

**Q: "Plotly not installed"?**
A: Run: `pip install plotly dash`

**Q: Too slow for large scenarios?**
A: Use `--mode interactive` or `--mode heatmap` - they're the fastest

---

## Paper Reference

This code implements **SMC-IO (Sliding-Mode-Control-based Instantaneously Optimal) Guidance** from:

> V. S. Shincy & Satadal Ghosh. "Sliding-Mode-Control–Based Instantaneously Optimal Guidance for Precision Soft Landing on Asteroid." *Journal of Spacecraft and Rockets*, Vol. 60, No. 1, pp. 146-159, 2023.

---

**Enjoy exploring asteroid landing guidance! 🚀**

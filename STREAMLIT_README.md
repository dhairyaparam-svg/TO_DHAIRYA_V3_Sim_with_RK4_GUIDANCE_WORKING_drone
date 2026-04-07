# Modern Web-Based GUI - Installation & Usage

## Quick Start

### 1. Install Streamlit

```bash
pip install streamlit
```

### 2. Run the Web Launcher

```bash
streamlit run web_launcher.py
```

This will:
- Open your browser automatically to `http://localhost:8501`
- Display a modern, professional interface
- Let you interact with the simulator in real-time

## Features

### 🎨 Modern Design
- **Clean, professional UI** - Sleek tab-based interface
- **Responsive layout** - Works on desktop, tablet, and mobile
- **Color-coded metrics** - Visual feedback with gradients and colors
- **Interactive charts** - Hover tooltips, zoom, pan capabilities

### 📍 Scenario Tab
- **Default scenarios** - Browse all 7 landing scenarios (LS1-LS7)
- **Custom scenarios** - Create your own with full parameter control
- **Scenario preview** - See asteroid & spacecraft parameters before running

### ⚙️ Parameters Tab
- **Quick presets** - Apply SMC tuning profiles one-click
- **Manual sliders** - Fine-tune Ee0, ke, Tmax, m0 in real-time
- **Live values** - See all parameter values updating

### 📐 Ee Function Tab
- **6 function types** - Compare all distance-based Ee functions
- **Visual comparison** - Chart showing how each function affects control authority
- **Descriptions** - Learn what each function does

### 🎮 Simulation Tab
- **Run button** - Execute simulation with current parameters
- **Visualization modes** - Choose how to display results
- **Progress indicator** - See simulation status while running
- **Landing metrics** - Immediate display of key results

### 📊 Results Tab
- **Summary metrics** - Landing error, fuel, speed, time
- **Interactive plots** - Distance, velocity, thrust, mass over time
- **3D trajectory** - Full 3D visualization with color-coded time
- **Detailed analysis** - Multiple plot types for comprehensive analysis

## Usage Examples

### Example 1: Quick Test
1. Go to **📍 Scenario** → Select LS1
2. Go to **⚙️ Parameters** → Click preset "moderate_reaching"
3. Go to **🎮 Run** → Click "🚀 RUN SIMULATION"
4. See results in **📊 Results** tab

### Example 2: Compare Ee Functions
1. **📍 Scenario** → Select LS3
2. **📐 Ee Function** → See comparison chart
3. **⚙️ Parameters** → Go back and select different function
4. **🎮 Run** → Run simulation
5. **📊 Results** → Compare with previous results

### Example 3: Custom Scenario
1. **📍 Scenario** → Scroll to "Create Custom Scenario"
2. Enter your own initial position, velocity, landing site
3. Click "Create Custom Scenario"
4. Continue with parameters and simulation

## Navigation Tips

- **Tabs persist** - Parameters you set stay when switching tabs
- **Quick apply** - Use preset buttons for instant parameter changes
- **Live preview** - Charts update as you adjust parameters
- **Keyboard shortcuts** - Use Tab to navigate, Enter to submit forms

## System Requirements

- Python 3.8+
- ~50 MB disk space
- Any modern browser (Chrome, Firefox, Safari, Edge)

## Installed Extensions

Streamlit includes:
- `plotly` - Interactive charts
- `matplotlib` - Standard plots
- `numpy` - Numerical computation
- `scipy` - Scientific computing

## Browser Compatibility

✓ Chrome (recommended)
✓ Firefox
✓ Safari
✓ Edge
✓ Opera

## Troubleshooting

### "Streamlit not found"
```bash
pip install streamlit
```

### "Port 8501 already in use"
```bash
streamlit run web_launcher.py --server.port=8502
```

### "Simulation takes too long"
- This is normal! The first run optimizes the guidance law for ~15-30 seconds
- Subsequent runs on the same scenario are slightly faster

### "Charts not displaying"
- Refresh the browser (F5)
- Check browser console for errors (F12)

## Deployment

To share this with others, you can deploy to:

**Streamlit Cloud** (Free, easiest)
```bash
streamlit cloud deploy
```

**Heroku** (Free tier available)
```bash
git push heroku main
```

**Docker**
```dockerfile
FROM python:3.11
RUN pip install -r requirements.txt
CMD ["streamlit", "run", "web_launcher.py"]
```

## Performance Notes

- **First load**: ~3-5 seconds (Streamlit initialization)
- **Simulation**: ~15-30 seconds (physics computation with SLSQP optimizer)
- **Visualization**: ~2-5 seconds (chart rendering)
- **Total time**: ~30-40 seconds per simulation

**Why it's slow:**
- SLSQP optimizer recalculates globally optimal guidance at every timestep
- 868 timesteps × 0.2s per step = ~170 seconds of computation
- Streamlit recomputes on every interaction (by design)

**Optimization:**
- Use "Dashboard" mode for faster interactive exploration
- Reduce simulation time (try shorter scenarios)
- Pre-compute results and cache them

## Features Comparison

| Feature | Tkinter GUI | Web GUI (Streamlit) |
|---------|------------|-------------------|
| Look & Feel | Dated (1980s) | Modern, Professional |
| Responsive | No | Yes |
| Colors/Styling | Limited | Full CSS support |
| Mobile | No | Yes |
| Deployment | Local only | Cloud-ready |
| Charts | Matplotlib | Plotly (Interactive) |
| Ease of use | Complex | Very Simple |
| Learning curve | Steep | Shallow |

## File Structure

```
web_launcher.py          ← Start here!
├── Tab 1: Scenario Selection
├── Tab 2: Parameter Tuning
├── Tab 3: Ee Function Selection
├── Tab 4: Simulation Execution
└── Tab 5: Results Analysis

Supports:
├── scenarios.py
├── smc_io_guidance.py
└── Other simulation modules
```

## Advanced Usage

### Custom Page Configuration
Edit the page config in `web_launcher.py`:
```python
st.set_page_config(
    page_title="Your Title",
    page_icon="Your Emoji",
    layout="wide",  # or "centered"
)
```

### Add Dark Mode
Users can enable via browser: ☰ → Settings → Theme → Dark

### Export Results
Results are automatically computed. Export manually:
```python
import json
json.dump(results, open("results.json", "w"))
```

---

**Enjoy the modern interface! 🎉**

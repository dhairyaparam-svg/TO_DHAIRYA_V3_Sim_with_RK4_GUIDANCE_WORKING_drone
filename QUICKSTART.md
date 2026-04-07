# 🚀 Quick Start Guide - Modern Web GUI

## 30-Second Setup

### Step 1: Install Streamlit (One-time)
```bash
pip install streamlit
```

Or double-click: `setup.bat`

### Step 2: Launch
```bash
streamlit run web_launcher.py
```

### Step 3: Browser Opens Automatically
Your browser will open to: `http://localhost:8501`

**That's it!** 🎉

---

## What You'll See

A beautiful, modern interface with 5 tabs:

### 📍 **Tab 1: Scenario**
- Choose from 7 default scenarios (LS1-LS7)
- Or create your own custom scenario
- See all parameters before running

### ⚙️ **Tab 2: Parameters**
- Quick presets (1-click apply)
- Manual sliders for fine-tuning
- Real-time value display

### 📐 **Tab 3: Ee Function**
- Compare all 6 distance-based functions
- Visual chart showing differences
- See how each affects control authority

### 🎮 **Tab 4: Run Simulation**
- Choose visualization mode
- Click "🚀 RUN SIMULATION"
- Watch progress in real-time

### 📊 **Tab 5: Results**
- Landing precision, fuel, time
- Interactive 3D trajectory
- Distance, velocity, thrust plots

---

## Example Workflows

### 🎯 Workflow 1: Quick Test (2 minutes)

1. **Scenario** → Select LS1 → See details
2. **Parameters** → Click "aggressive_reaching" preset
3. **Ee Function** → Leave as "quadratic_cubic"
4. **Run** → Click 🚀 button → Wait 20 seconds
5. **Results** → See your landing metrics

### 🔬 Workflow 2: Compare Functions (5 minutes)

1. **Scenario** → Select LS3
2. **Ee Function** → Study the comparison chart
3. **Run** → Try "exponential" → Run simulation → See results
4. **Run** → Try "linear" → Run simulation → Compare
5. **Run** → Try "saturation" → Run simulation → Compare

### 🧪 Workflow 3: Custom Scenario (3 minutes)

1. **Scenario** → Scroll to "Create Custom Scenario"
2. Enter your own position, velocity, landing site
3. **Click** "Create Custom Scenario"
4. **Parameters** → Adjust tuning
5. **Run** → Execute simulation
6. **Results** → Analyze your custom scenario

---

## Common Questions

### Q: What's the 🚀 button under "Run Simulation"?
**A:** That's the button to start the simulation. Click it and wait for results.

### Q: Why is it slow?
**A:** The physics solver optimizes guidance at every 1-second timestep. ~20 seconds is normal for 868 steps.

### Q: Can I use this on my phone?
**A:** Yes! Streamlit is fully responsive. Just open the browser link on your phone. 📱

### Q: How do I stop it?
**A:** Press `Ctrl+C` in the terminal, or close the browser tab.

### Q: Where are my results?
**A:** They appear automatically in the **📊 Results** tab after simulation completes.

### Q: Can I zoom into the 3D plot?
**A:** Yes! Use mouse wheel to zoom, drag to rotate, double-click to reset.

### Q: How do I save the charts?
**A:** Hover over any chart → Click 📷 icon in top-right corner.

### Q: Can I share this with others?
**A:** Yes! Deploy to Streamlit Cloud for free sharing via URL.

---

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Tab` | Move between fields |
| `Enter` | Submit form |
| `Ctrl+C` | Stop running |
| `F5` / `Cmd+R` | Refresh page |
| `F12` | Open browser console |

---

## Tips & Tricks

### 💡 Tip 1: Use Presets
Don't manually adjust every parameter. Click a preset button to apply pre-tuned combinations instantly.

### 💡 Tip 2: Compare Side-by-Side
- Run Scenario A
- Note the results in Tab 5
- Go back to Tab 2, change parameters
- Run Scenario B
- Compare the two results

### 💡 Tip 3: Export Charts
Every chart has a download button (camera icon). Export for reports/presentations.

### 💡 Tip 4: Dark Mode
Click ☰ → Settings → Theme → Dark for night mode.

### 💡 Tip 5: Full Screen
Hover over any chart and click the expand icon for full-screen view.

---

## What Each Parameter Does

### **Ee0** (Sliding Mode Constant)
- Higher = More aggressive reaching
- Range: 0.0001 - 0.02
- ✅ Start with 0.002 (default)

### **ke** (Proportional Coefficient)
- Higher = Faster convergence
- Range: 0.001 - 0.05
- ✅ Start with 0.006 (default)

### **Tmax** (Max Thrust)
- Higher = Stronger engine
- Range: 30 - 200 N
- ✅ Start with 80 N (default)

### **m0** (Initial Mass)
- Affects fuel consumption
- Range: 1000 - 1600 kg
- ✅ Start with 1400 kg (default)

### **Ee Function**
- How distance affects control
- 6 different types available
- ✅ Start with "quadratic_cubic" (paper default)

---

## Understanding the Results

### Landing Error
- **< 0.001 m**: Perfect! 🎯
- **< 0.01 m**: Excellent ✅
- **< 0.1 m**: Good 👍
- **> 1 m**: Needs tuning ⚠️

### Final Velocity
- **< 1e-6 m/s**: Perfect stop 🎯
- **< 0.001 m/s**: Barely moving ✅
- **< 0.1 m/s**: Slow descent 👍
- **> 1 m/s**: Impact risk ⚠️

### Fuel Consumed
- **< 2 kg**: Very efficient 🌟
- **2-3 kg**: Efficient ✅
- **3-4 kg**: Normal 👍
- **> 4 kg**: High consumption ⚠️

### Time Steps
- **< 500**: Fast convergence 🚀
- **500-900**: Normal 👍
- **> 1000**: Slow convergence ⚠️

---

## Troubleshooting

### ❌ "StreamlitAPIException: Invalid default"
**Fix:** Restart the app with `Ctrl+C` then re-run

### ❌ "Port 8501 already in use"
**Fix:** Run on different port:
```bash
streamlit run web_launcher.py --server.port=8502
```

### ❌ "Charts not showing"
**Fix:** Refresh browser (F5) or clear cache:
```bash
streamlit cache clear
```

### ❌ "Simulation takes too long"
**Fix:** This is normal! First run is slower. Subsequent runs are faster.

### ❌ "Nothing happens when I click Run"
**Fix:** Check browser console (F12). Look for errors.

---

## Next Steps

### 📚 Learn More
- See `STREAMLIT_README.md` for detailed docs
- See `UI_COMPARISON.md` for before/after
- Check `scenarios.py` for scenario definitions

### 🚀 Go Deeper
- Modify `web_launcher.py` to customize colors
- Add your own scenarios in `scenarios.py`
- Create custom visualizations

### 📤 Share Your Work
- Deploy to Streamlit Cloud (free!)
- Share results with colleagues
- Present in meetings with live dashboard

### 💻 Deploy to Cloud
```bash
streamlit cloud deploy
```

---

## System Requirements

✓ Python 3.8+
✓ Any modern browser
✓ Internet connection (for Streamlit Cloud features)
✓ ~200MB disk space
✓ Any OS (Windows, Mac, Linux)

---

## File Structure

```
Your Project/
├── web_launcher.py          ← NEW: Modern web GUI
├── gui_launcher.py          ← OLD: Tkinter GUI (deprecated)
├── setup.bat                ← Automated setup
├── smc_io_guidance.py       ← Core simulation
├── scenarios.py             ← Landing scenarios
└── STREAMLIT_README.md      ← Full documentation
```

---

## Performance Expectations

| Task | Time |
|------|------|
| App startup | 5-8 seconds |
| Tab switching | <100ms |
| Parameter adjustment | <100ms |
| Simulation run | 20-30 seconds |
| Chart loading | 2-5 seconds |
| **Total per test** | ~30-40 seconds |

---

## One More Thing

### 🎓 This GUI is now:
- ✅ Modern and professional
- ✅ Mobile-friendly
- ✅ Cloud-deployable
- ✅ Interactive and responsive
- ✅ Easy to share
- ✅ Zero 1980s vibes

### 🚀 Ready to get started?

```bash
streamlit run web_launcher.py
```

Your browser will open automatically. Enjoy! 🎉

---

**Questions?** Check `STREAMLIT_README.md` or `UI_COMPARISON.md`

**Need help?** See the troubleshooting section above

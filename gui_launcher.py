"""
Interactive GUI Launcher for SMC-IO Asteroid Landing Guidance Simulation
=========================================================================

Provides a user-friendly interface to:
1. Select landing scenario or input custom parameters
2. Tune sliding mode control parameters
3. Choose distance-based Ee function
4. Select visualization mode
5. Run simulation with one click
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import numpy as np
from scenarios import (
    build_params, modify_params, get_smc_profiles, get_ee_functions,
    add_custom_scenario, LANDING_SCENARIOS, DEFAULT_PARAMS
)
from smc_io_guidance import simulate_with_params, plot_results, plot_interactive_3d, create_animation, plot_control_heatmap, compare_all_scenarios, create_dash_app
import threading


class GUIDancerLauncher:
    """Interactive GUI for asteroid landing guidance simulation."""
    
    def __init__(self, root):
        """Initialize the GUI application."""
        self.root = root
        self.root.title("SMC-IO Asteroid Landing Guidance - Control Panel")
        self.root.geometry("900x900")
        self.root.resizable(True, True)
        
        # Styling
        style = ttk.Style()
        style.theme_use('clam')
        
        # Current parameters
        self.current_params = None
        self.simulation_results = None
        
        # Build UI
        self._build_ui()
        
    def _build_ui(self):
        """Build the user interface."""
        # Main notebook (tabbed interface)
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Tab 1: Scenario Selection
        tab1 = ttk.Frame(notebook)
        notebook.add(tab1, text="1. Select Scenario")
        self._build_scenario_tab(tab1)
        
        # Tab 2: Parameter Tuning
        tab2 = ttk.Frame(notebook)
        notebook.add(tab2, text="2. Tune Parameters")
        self._build_tuning_tab(tab2)
        
        # Tab 3: Ee Function Selection
        tab3 = ttk.Frame(notebook)
        notebook.add(tab3, text="3. Select Ee Function")
        self._build_ee_tab(tab3)
        
        # Tab 4: Visualization
        tab4 = ttk.Frame(notebook)
        notebook.add(tab4, text="4. Visualize & Run")
        self._build_visualization_tab(tab4)
        
        # Tab 5: Results
        tab5 = ttk.Frame(notebook)
        notebook.add(tab5, text="5. Results")
        self._build_results_tab(tab5)
        
    def _build_scenario_tab(self, parent):
        """Build scenario selection tab."""
        # Title
        title = ttk.Label(parent, text="Landing Scenario Selection", font=("Arial", 14, "bold"))
        title.pack(padx=10, pady=10)
        
        # Scenario selection
        frame = ttk.LabelFrame(parent, text="Choose Scenario", padding=10)
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        ttk.Label(frame, text="Select from default scenarios:").pack(anchor=tk.W, pady=5)
        
        self.scenario_var = tk.StringVar(value="1")
        scenarios_list = list(LANDING_SCENARIOS.keys())
        scenario_dropdown = ttk.Combobox(
            frame, textvariable=self.scenario_var, 
            values=scenarios_list, state="readonly", width=40
        )
        scenario_dropdown.pack(anchor=tk.W, pady=5)
        scenario_dropdown.bind("<<ComboboxSelected>>", self._on_scenario_selected)
        
        # Scenario info display
        info_frame = ttk.LabelFrame(frame, text="Scenario Details", padding=10)
        info_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        self.scenario_info = scrolledtext.ScrolledText(info_frame, height=8, width=60, state=tk.DISABLED)
        self.scenario_info.pack(fill=tk.BOTH, expand=True)
        
        # Custom scenario option
        custom_frame = ttk.LabelFrame(parent, text="Or Create Custom Scenario", padding=10)
        custom_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        ttk.Label(custom_frame, text="Scenario ID:", font=("Arial", 10, "bold")).pack(anchor=tk.W, pady=5)
        self.custom_id = ttk.Entry(custom_frame, width=10)
        self.custom_id.pack(anchor=tk.W, pady=5)
        self.custom_id.insert(0, "8")
        
        ttk.Label(custom_frame, text="Initial Position [x, y, z] (m):").pack(anchor=tk.W, pady=5)
        self.custom_pos = ttk.Entry(custom_frame, width=40)
        self.custom_pos.pack(anchor=tk.W, pady=5)
        self.custom_pos.insert(0, "1500, -100, -50")
        
        ttk.Label(custom_frame, text="Initial Velocity [vx, vy, vz] (m/s):").pack(anchor=tk.W, pady=5)
        self.custom_vel = ttk.Entry(custom_frame, width=40)
        self.custom_vel.pack(anchor=tk.W, pady=5)
        self.custom_vel.insert(0, "-2.5, -1.0, -0.5")
        
        ttk.Label(custom_frame, text="Landing Site [rx, ry, rz] (m):").pack(anchor=tk.W, pady=5)
        self.custom_rfd = ttk.Entry(custom_frame, width=40)
        self.custom_rfd.pack(anchor=tk.W, pady=5)
        self.custom_rfd.insert(0, "1000, 0, 0")
        
        ttk.Button(custom_frame, text="Create & Use Custom Scenario", command=self._create_custom).pack(pady=10)
        
        # Load initial scenario info
        self._on_scenario_selected()
        
    def _on_scenario_selected(self, event=None):
        """Display selected scenario information."""
        try:
            scenario_id = int(self.scenario_var.get())
            params = build_params(scenario_id)
            
            info_text = f"Scenario LS{scenario_id}:\n"
            info_text += f"{'='*50}\n\n"
            info_text += f"Initial Position: {params['pos0']}\n"
            info_text += f"Initial Velocity: {params['vel0']}\n"
            info_text += f"Landing Site: {params['rfd']}\n\n"
            info_text += f"Asteroid Parameters:\n"
            info_text += f"  GM = {params['GM']}\n"
            info_text += f"  Dimensions: {params['la']} × {params['lb']} × {params['lc']}\n"
            info_text += f"  Rotation: {params['omg']}\n\n"
            info_text += f"Spacecraft Parameters:\n"
            info_text += f"  Tmax = {params['Tmax']} N\n"
            info_text += f"  m0 = {params['m0']} kg\n"
            info_text += f"  c = {params['c']} m/s\n"
            
            self.scenario_info.config(state=tk.NORMAL)
            self.scenario_info.delete(1.0, tk.END)
            self.scenario_info.insert(tk.END, info_text)
            self.scenario_info.config(state=tk.DISABLED)
            
            # Store params
            self.current_params = params
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load scenario: {e}")
            
    def _create_custom(self):
        """Create custom scenario."""
        try:
            sid = int(self.custom_id.get())
            pos0 = np.array([float(x.strip()) for x in self.custom_pos.get().split(',')])
            vel0 = np.array([float(x.strip()) for x in self.custom_vel.get().split(',')])
            rfd = np.array([float(x.strip()) for x in self.custom_rfd.get().split(',')])
            
            add_custom_scenario(sid, f"Custom Scenario {sid}", pos0, vel0, rfd)
            self.current_params = build_params(sid)
            
            messagebox.showinfo("Success", f"Custom scenario {sid} created and loaded!")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to create custom scenario: {e}")
            
    def _build_tuning_tab(self, parent):
        """Build parameter tuning tab."""
        title = ttk.Label(parent, text="SMC Control Parameter Tuning", font=("Arial", 14, "bold"))
        title.pack(padx=10, pady=10)
        
        # SMC Profiles quick selection
        frame = ttk.LabelFrame(parent, text="Quick SMC Profiles", padding=10)
        frame.pack(fill=tk.BOTH, padx=10, pady=10)
        
        ttk.Label(frame, text="Select preset SMC tuning:").pack(anchor=tk.W, pady=5)
        
        profiles = get_smc_profiles()
        self.smc_profile_var = tk.StringVar(value="moderate_reaching")
        profile_dropdown = ttk.Combobox(
            frame, textvariable=self.smc_profile_var,
            values=list(profiles.keys()), state="readonly", width=40
        )
        profile_dropdown.pack(anchor=tk.W, pady=5)
        profile_dropdown.bind("<<ComboboxSelected>>", self._on_profile_selected)
        
        self.profile_info = ttk.Label(frame, text="", wraplength=500, justify=tk.LEFT)
        self.profile_info.pack(anchor=tk.W, pady=10)
        
        # Manual tuning
        manual_frame = ttk.LabelFrame(parent, text="Or Manually Tune", padding=10)
        manual_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Ee0 slider
        ttk.Label(manual_frame, text="Ee0 (Distance coefficient):").pack(anchor=tk.W, pady=5)
        self.ee0_var = tk.DoubleVar(value=0.002)
        ee0_scale = ttk.Scale(
            manual_frame, from_=0.0001, to=0.02, orient=tk.HORIZONTAL,
            variable=self.ee0_var, command=self._update_ee0_label
        )
        ee0_scale.pack(fill=tk.X, pady=5)
        self.ee0_label = ttk.Label(manual_frame, text=f"Ee0 = 0.002")
        self.ee0_label.pack(anchor=tk.W)
        
        # ke slider
        ttk.Label(manual_frame, text="ke (Proportional coefficient):").pack(anchor=tk.W, pady=10)
        self.ke_var = tk.DoubleVar(value=0.006)
        ke_scale = ttk.Scale(
            manual_frame, from_=0.001, to=0.05, orient=tk.HORIZONTAL,
            variable=self.ke_var, command=self._update_ke_label
        )
        ke_scale.pack(fill=tk.X, pady=5)
        self.ke_label = ttk.Label(manual_frame, text=f"ke = 0.006")
        self.ke_label.pack(anchor=tk.W)
        
        # Other parameters
        other_frame = ttk.LabelFrame(parent, text="Other Parameters", padding=10)
        other_frame.pack(fill=tk.BOTH, padx=10, pady=10)
        
        ttk.Label(other_frame, text="Max Thrust (N):").pack(anchor=tk.W, pady=5)
        self.tmax_var = tk.DoubleVar(value=80.0)
        ttk.Entry(other_frame, textvariable=self.tmax_var, width=20).pack(anchor=tk.W, pady=5)
        
        ttk.Label(other_frame, text="Initial Mass (kg):").pack(anchor=tk.W, pady=5)
        self.m0_var = tk.DoubleVar(value=1400.0)
        ttk.Entry(other_frame, textvariable=self.m0_var, width=20).pack(anchor=tk.W, pady=5)
        
        self._on_profile_selected()
        
    def _on_profile_selected(self, event=None):
        """Update info when SMC profile is selected."""
        try:
            profile_name = self.smc_profile_var.get()
            profiles = get_smc_profiles()
            profile = profiles[profile_name]
            
            info = f"{profile.get('description', 'No description')}\n"
            info += f"Ee0 = {profile['Ee0']}, ke = {profile['ke']}"
            
            self.profile_info.config(text=info)
            self.ee0_var.set(profile['Ee0'])
            self.ke_var.set(profile['ke'])
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load profile: {e}")
            
    def _update_ee0_label(self, value):
        """Update Ee0 display label."""
        self.ee0_label.config(text=f"Ee0 = {float(value):.6f}")
        
    def _update_ke_label(self, value):
        """Update ke display label."""
        self.ke_label.config(text=f"ke = {float(value):.6f}")
        
    def _build_ee_tab(self, parent):
        """Build distance-based Ee function selection tab."""
        title = ttk.Label(parent, text="Distance-Based Ee Function Selection", font=("Arial", 14, "bold"))
        title.pack(padx=10, pady=10)
        
        frame = ttk.LabelFrame(parent, text="Available Ee Functions", padding=10)
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        ttk.Label(frame, text="Select how distance affects sliding mode reaching:").pack(anchor=tk.W, pady=5)
        
        ee_functions = get_ee_functions()
        self.ee_func_var = tk.StringVar(value="quadratic_cubic")
        
        # Radio buttons for each function
        for func_name, func_info in ee_functions.items():
            rb = ttk.Radiobutton(
                frame, text=func_name.replace("_", " ").title(),
                variable=self.ee_func_var, value=func_name,
                command=self._on_ee_func_selected
            )
            rb.pack(anchor=tk.W, pady=5)
            
            label = ttk.Label(frame, text=f"  → {func_info['description']}", 
                            wraplength=700, justify=tk.LEFT, foreground="blue")
            label.pack(anchor=tk.W, padx=30, pady=2)
        
        self._on_ee_func_selected()
        
    def _on_ee_func_selected(self):
        """Show selected Ee function details."""
        ee_functions = get_ee_functions()
        selected = self.ee_func_var.get()
        func_info = ee_functions.get(selected, {})
        # Details already shown in radiobuttons
        
    def _build_visualization_tab(self, parent):
        """Build visualization selection and run tab."""
        title = ttk.Label(parent, text="Visualization & Execution", font=("Arial", 14, "bold"))
        title.pack(padx=10, pady=10)
        
        # Visualization mode selection
        frame = ttk.LabelFrame(parent, text="Select Visualization Mode", padding=10)
        frame.pack(fill=tk.BOTH, padx=10, pady=10)
        
        ttk.Label(frame, text="How do you want to visualize results?").pack(anchor=tk.W, pady=5)
        
        self.viz_var = tk.StringVar(value="interactive")
        modes = [
            ("Standard Plot (matplotlib)", "standard"),
            ("Interactive 3D (Plotly)", "interactive"),
            ("Web Dashboard (Dash)", "dashboard"),
            ("Animation (MP4 video)", "animate"),
            ("Control Heatmap", "heatmap"),
            ("Compare All Scenarios", "compare"),
        ]
        
        for label, value in modes:
            rb = ttk.Radiobutton(frame, text=label, variable=self.viz_var, value=value)
            rb.pack(anchor=tk.W, pady=5)
        
        # Run button
        run_frame = ttk.LabelFrame(parent, text="Execute Simulation", padding=10)
        run_frame.pack(fill=tk.BOTH, padx=10, pady=10)
        
        ttk.Button(
            run_frame, text="Run Simulation with Selected Parameters",
            command=self._run_simulation
        ).pack(fill=tk.X, pady=10)
        
        # Status
        ttk.Label(run_frame, text="Status:").pack(anchor=tk.W, pady=5)
        self.status_text = scrolledtext.ScrolledText(run_frame, height=6, width=80, state=tk.DISABLED)
        self.status_text.pack(fill=tk.BOTH, expand=True, pady=5)
        
    def _build_results_tab(self, parent):
        """Build results display tab."""
        title = ttk.Label(parent, text="Simulation Results", font=("Arial", 14, "bold"))
        title.pack(padx=10, pady=10)
        
        self.results_text = scrolledtext.ScrolledText(parent, height=30, state=tk.DISABLED)
        self.results_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
    def _run_simulation(self):
        """Run simulation with current parameters."""
        try:
            # Gather all parameters
            if self.current_params is None:
                messagebox.showerror("Error", "Please select a scenario first!")
                return
            
            params = self.current_params.copy()
            
            # Override with manual tuning values
            params["Ee0"] = self.ee0_var.get()
            params["ke"] = self.ke_var.get()
            params["Tmax"] = self.tmax_var.get()
            params["m0"] = self.m0_var.get()
            params["ee_function"] = self.ee_func_var.get()
            
            # Update status
            self._update_status("Starting simulation...")
            self.root.update()
            
            # Run simulation in thread to avoid freezing UI
            thread = threading.Thread(target=self._simulate_thread, args=(params,))
            thread.start()
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to run simulation: {e}")
            
    def _simulate_thread(self, params):
        """Run simulation in background thread."""
        try:
            self._update_status(f"Running simulation with parameters:\n"
                              f"  Ee0 = {params['Ee0']}\n"
                              f"  ke = {params['ke']}\n"
                              f"  Ee function = {params['ee_function']}\n"
                              f"  Tmax = {params['Tmax']} N\n"
                              f"  m0 = {params['m0']} kg\n"
                              f"\nSimulating...")
            
            # Run simulation with custom parameters
            t_arr, states_arr, command_arr, J_arr = simulate_with_params(params, verbose=True)
            
            # Store results
            self.simulation_results = {
                't_arr': t_arr,
                'states_arr': states_arr,
                'command_arr': command_arr,
                'J_arr': J_arr,
                'params': params
            }
            
            # Prepare results display
            results_str = f"SIMULATION COMPLETED\n{'='*60}\n\n"
            results_str += f"Scenario: {params.get('scenario_name', 'Custom')}\n"
            results_str += f"Distance-based Ee function: {params['ee_function']}\n\n"
            results_str += f"Results:\n"
            
            if len(states_arr) > 0:
                final_pos = states_arr[-1, 0:3]
                final_vel = states_arr[-1, 3:6]
                final_mass = states_arr[-1, 6]
                landing_error = np.linalg.norm(final_pos - params['rfd'])
                landing_speed = np.linalg.norm(final_vel)
                fuel_used = params['m0'] - final_mass
                
                results_str += f"  Landing error: {landing_error:.6f} m\n"
                results_str += f"  Landing speed: {landing_speed:.9f} m/s\n"
                results_str += f"  Fuel consumed: {fuel_used:.4f} kg\n"
                results_str += f"  Final mass: {final_mass:.2f} kg\n"
                results_str += f"  Time steps: {len(t_arr)}\n"
                results_str += f"  Total time: {t_arr[-1]:.2f} s\n"
            
            self._update_results(results_str)
            
            # Run visualization
            viz_mode = self.viz_var.get()
            self._update_status(f"Simulation complete. Running {viz_mode} visualization...")
            
            if viz_mode == "interactive":
                plot_interactive_3d(t_arr, states_arr, command_arr, params)
            elif viz_mode == "dashboard":
                app = create_dash_app(t_arr, states_arr, command_arr, J_arr, params)
                if app:
                    app.run(debug=False)
            elif viz_mode == "animate":
                create_animation(t_arr, states_arr, command_arr, params)
            elif viz_mode == "heatmap":
                plot_control_heatmap(t_arr, states_arr, command_arr)
            elif viz_mode == "compare":
                compare_all_scenarios()
            else:
                # standard
                plot_results(t_arr, states_arr, command_arr, J_arr, params)
            
            self._update_status("Simulation complete! Results displayed.")
            
        except Exception as e:
            self._update_status(f"ERROR: {str(e)}")
            messagebox.showerror("Simulation Error", f"Simulation failed:\n{str(e)}")
            
    def _update_status(self, message):
        """Update status display (thread-safe)."""
        def update():
            self.status_text.config(state=tk.NORMAL)
            self.status_text.insert(tk.END, message + "\n")
            self.status_text.see(tk.END)
            self.status_text.config(state=tk.DISABLED)
        self.root.after(0, update)
        
    def _update_results(self, message):
        """Update results display (thread-safe)."""
        def update():
            self.results_text.config(state=tk.NORMAL)
            self.results_text.delete(1.0, tk.END)
            self.results_text.insert(tk.END, message)
            self.results_text.config(state=tk.DISABLED)
        self.root.after(0, update)


if __name__ == "__main__":
    root = tk.Tk()
    app = GUIDancerLauncher(root)
    root.mainloop()

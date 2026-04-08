"""
Modern Web-Based GUI for Quadrotor SMC-IO Trajectory Guidance Simulator
====================================================================

Built with Streamlit for a sleek, professional interface.

Run with: streamlit run quadrotor_web_launcher.py
"""

import streamlit as st
import numpy as np
import plotly.graph_objects as go
from quadrotor_scenarios import (
    build_params, modify_params, add_custom_scenario, FLIGHT_SCENARIOS, DEFAULT_PARAMS
)
from quadrotor_smc_guidance import (
    simulate, plot_interactive_3d
)
import matplotlib.pyplot as plt
from io import BytesIO
import sys
import tempfile
import os
from atmosphere_model import isa_density

# ============================================================================
# Functions manually added from scenarios.py since quadrotor_scenarios.py doesn't have them
# ============================================================================

def get_smc_profiles():
    return {
        "conservative_reaching": {
            "Ee0": 1.0,
            "ke": 1.5,
            "description": "Smooth, low-authority guidance for fuel efficiency",
        },
        "moderate_reaching": {
            "Ee0": 2.0,
            "ke": 3.0,
            "description": "Balanced approach (default for Quadrotor scenarios)",
        },
        "aggressive_reaching": {
            "Ee0": 4.0,
            "ke": 5.0,
            "description": "High-authority guidance for rapid convergence",
        },
        "high_reaching": {
            "Ee0": 6.0,
            "ke": 8.0,
            "description": "Maximum control authority (extreme acceleration demands)",
        },
    }

def get_ee_functions():
    return {
        "quadratic_cubic": {
            "name": "Quadratic + Cubic (Default)",
            "description": "Balanced polynomial (smooth)",
            "formula": "Ee = 0.5*(Ee0/r0²)*r² + 0.5*(Ee0/r0³)*r³",
        },
        "linear": {
            "name": "Linear Distance Scaling",
            "description": "Ee scales linearly with distance",
            "formula": "Ee = (Ee0/r0)*r",
        },
        "quadratic": {
            "name": "Quadratic Distance Scaling",
            "description": "Pure quadratic distance dependence",
            "formula": "Ee = (Ee0/r0²)*r²",
        },
        "cubic": {
            "name": "Cubic Distance Scaling",
            "description": "Pure cubic distance dependence",
            "formula": "Ee = (Ee0/r0³)*r³",
        },
        "exponential": {
            "name": "Exponential Distance Scaling",
            "description": "Steep near target",
            "formula": "Ee = Ee0*(exp(r/r0) - 1)",
        },
        "saturation": {
            "name": "Saturation/Constant",
            "description": "Constant reaching gain, no distance dependence",
            "formula": "Ee = Ee0",
        },
    }

# ============================================================================
# Page Configuration
# ============================================================================

st.set_page_config(
    page_title="Quadrotor SMC-IO Flight Guidance",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom CSS for better styling
st.markdown("""
    <style>
        .main {
            padding: 2rem;
        }
        .stTabs [data-baseweb="tab-list"] button {
            font-size: 18px;
            font-weight: bold;
        }
    </style>
""", unsafe_allow_html=True)

# ============================================================================
# Sidebar - Global Controls
# ============================================================================

st.sidebar.markdown("# 🚁 Quadrotor SMC-IO Guidance")
st.sidebar.markdown("**Sliding Mode Control-based**")
st.sidebar.markdown("**Flight Trajectory Planning**")
st.sidebar.divider()

# Initialize session state
if 'current_params' not in st.session_state:
    st.session_state.current_params = build_params(1)
if 'simulation_results' not in st.session_state:
    st.session_state.simulation_results = None

st.sidebar.markdown("""
<div style="text-align: center; margin-top: 3rem;">
    <p style="font-size: 0.85rem; color: #888;">
        <br>
        <b style="color: #555;">This simulation is performed on a 10 inch quad with 4S 5200MAh Powerpack</b><br>
        For Custom set of parameters kindly refer to the Repository.
    </p>
</div>

<div style="text-align: center; margin-top: 3rem;">
    <img src="https://iitgn.ac.in/assets/img/logo.png" width="100" style="margin-bottom: 10px;">
    <p style="font-size: 0.85rem; color: #888;">
        Developed in<br>
        <b style="color: #555;">Center of Research Commercialization</b><br>
        Indian Institute of Technology Gandhinagar
    </p>
</div>
""", unsafe_allow_html=True)

# ============================================================================
# Main Content
# ============================================================================

st.title("🚁 Quadrotor Flight Guidance Simulator")
st.markdown("**Advanced SMC-IO Control for Precision Flight & Obstacle Avoidance**")
st.divider()

# Create tabs
tab1, tab2, tab3, tab4, tab5 = st.tabs([
    "📍 Scenario", 
    "⚙️ Parameters", 
    "📐 Sliding Mode Variable Function",
    "🎮 Run Simulation",
    "📊 Results"
])

# ============================================================================
# TAB 1: Scenario Selection
# ============================================================================

with tab1:
    st.header("Flight Scenario Selection")
    
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("📚 Default Scenarios")
        st.write("Choose from 4 pre-defined scenarios:")
        
        scenario_nums = list(FLIGHT_SCENARIOS.keys())
        selected_scenario = st.selectbox(
            "Flight Scenario (FS):",
            scenario_nums,
            format_func=lambda x: f"FS{x}: {FLIGHT_SCENARIOS[x]['name']}"
        )
        
        # Load and display scenario info
        params = build_params(selected_scenario)
        st.session_state.current_params = params
        
        with st.container(border=True):
            st.markdown(f"### FS{selected_scenario}: {FLIGHT_SCENARIOS[selected_scenario]['name']}")
            
            col_a, col_b = st.columns(2)
            with col_a:
                st.metric("Initial Position", 
                         f"{np.linalg.norm(params['pos0']):.1f} m",
                         f"({params['pos0'][0]:.0f}, {params['pos0'][1]:.0f}, {params['pos0'][2]:.0f}) m")
                st.metric("Initial Velocity",
                         f"{np.linalg.norm(params['vel0']):.2f} m/s",
                         f"({params['vel0'][0]:.2f}, {params['vel0'][1]:.2f}, {params['vel0'][2]:.2f}) m/s")
            
            with col_b:
                st.metric("Target Site",
                         f"{np.linalg.norm(params['rfd']):.1f} m",
                         f"({params['rfd'][0]:.0f}, {params['rfd'][1]:.0f}, {params['rfd'][2]:.0f}) m")
                st.metric("Distance to Target",
                         f"{np.linalg.norm(params['pos0'] - params['rfd']):.1f} m")
            
            # Quadrotor parameters
            quad_col, env_col = st.columns(2)
            with quad_col:
                st.markdown("**Quadrotor Parameters:**")
                st.write(f"- Max Thrust = {params['Tmax']} N")
                st.write(f"- Mass = {params['m0']} kg")
                st.write(f"- Cd = {params['Cd']}, A = {params['A']} m²")
            
            with env_col:
                st.markdown("**Environment:**")
                st.write(f"- Wind Base: {params['wind']['base']} m/s")
                st.write(f"- Wind Gusts: {params['wind']['gust_amp']} m/s amp")
                st.write(f"- Obstacles: {len(params.get('obstacles', []))}")
    
    with col2:
        st.subheader("⚡ Create Custom Scenario")
        st.write("Define your own flight scenario:")
        
        with st.form("custom_scenario_form"):
            custom_id = st.number_input("Scenario ID", value=5, min_value=5, max_value=20)
            
            st.write("**Initial Position (x, y, z) in meters:**")
            col_x, col_y, col_z = st.columns(3)
            with col_x:
                pos_x = st.number_input("x", value=0.0, step=10.0)
            with col_y:
                pos_y = st.number_input("y", value=0.0, step=10.0)
            with col_z:
                pos_z = st.number_input("z", value=10.0, step=5.0)
            
            st.write("**Initial Velocity (vx, vy, vz) in m/s:**")
            col_vx, col_vy, col_vz = st.columns(3)
            with col_vx:
                vel_x = st.number_input("vx", value=2.0, step=0.5)
            with col_vy:
                vel_y = st.number_input("vy", value=0.5, step=0.5)
            with col_vz:
                vel_z = st.number_input("vz", value=0.0, step=0.5)
            
            st.write("**Target Site (rx, ry, rz) in meters:**")
            col_rx, col_ry, col_rz = st.columns(3)
            with col_rx:
                rfd_x = st.number_input("rx", value=100.0, step=10.0)
            with col_ry:
                rfd_y = st.number_input("ry", value=50.0, step=10.0)
            with col_rz:
                rfd_z = st.number_input("rz", value=20.0, step=5.0)
            
            submit_custom = st.form_submit_button("Create Custom Scenario", use_container_width=True)
            
            if submit_custom:
                try:
                    pos0 = np.array([pos_x, pos_y, pos_z])
                    vel0 = np.array([vel_x, vel_y, vel_z])
                    rfd = np.array([rfd_x, rfd_y, rfd_z])
                    
                    add_custom_scenario(int(custom_id), f"Custom Scenario {int(custom_id)}", pos0, vel0, rfd)
                    st.session_state.current_params = build_params(int(custom_id))
                    
                    st.success(f"✓ Custom scenario {int(custom_id)} created!")
                except Exception as e:
                    st.error(f"Failed to create scenario: {e}")

# ============================================================================
# TAB 2: Parameter Tuning
# ============================================================================

with tab2:
    st.header("⚙️ Control Parameter Tuning")
    
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("🎯 Quick SMC Profiles")
        st.write("Select a preset or tune manually:")
        
        profiles = get_smc_profiles()
        profile_names = list(profiles.keys())
        selected_profile = st.selectbox(
            "SMC Profile:",
            profile_names,
            key="profile_select"
        )
        
        profile_info = profiles[selected_profile]
        st.info(f"📋 {profile_info['description']}")
        
        if st.button("Apply Profile", use_container_width=True):
            params = st.session_state.current_params.copy()
            params.update(profile_info)
            st.session_state.current_params = params
            st.success("✓ Profile applied!")
    
    with col2:
        st.subheader("🎚️ Manual Tuning")
    
    # Manual parameter sliders
    st.divider()
    
    tuning_col1, tuning_col2, tuning_col3 = st.columns(3)
    
    with tuning_col1:
        st.subheader("Sliding Mode Constants")
        
        ee0 = st.slider(
            "Ee0 (Distance Coefficient)",
            min_value=0.1,
            max_value=10.0,
            value=st.session_state.current_params.get("Ee0", 2.0),
            step=0.1,
            format="%.2f"
        )
        
        ke = st.slider(
            "ke (Proportional Coefficient)",
            min_value=0.5,
            max_value=10.0,
            value=st.session_state.current_params.get("ke", 3.0),
            step=0.1,
            format="%.2f"
        )
        
        st.metric("SMC Formula", "s_e_dot = -ke*s_e - Ee*sign(s_e)")
    
    with tuning_col2:
        st.subheader("Drone Parameters")
        
        tmax = st.slider(
            "Max Thrust (N)",
            min_value=10.0,
            max_value=100.0,
            value=st.session_state.current_params.get("Tmax", 50.0),
            step=5.0
        )
        
        m0 = st.slider(
            "Initial Mass (kg)",
            min_value=1.0,
            max_value=10.0,
            value=st.session_state.current_params.get("m0", 2.5),
            step=0.1
        )
    
    with tuning_col3:
        st.subheader("📊 Current Values")
        current_params = st.session_state.current_params
        st.metric("Ee0", f"{ee0:.2f}")
        st.metric("ke", f"{ke:.2f}")
        st.metric("Tmax", f"{tmax:.1f} N")
        st.metric("m0", f"{m0:.1f} kg")
    
    # Update parameters
    st.session_state.current_params["Ee0"] = float(ee0)
    st.session_state.current_params["ke"] = float(ke)
    st.session_state.current_params["Tmax"] = float(tmax)
    st.session_state.current_params["m0"] = float(m0)

# ============================================================================
# TAB 3: Ee Function Selection
# ============================================================================

with tab3:
    st.header("📐 Distance-Based Sliding Mode Variable Function")
    st.write("Select how distance affects the sliding mode reaching law:")
    st.divider()
    
    ee_functions = get_ee_functions()
    
    # Display as radio buttons with descriptions
    selected_ee_func = st.radio(
        "Ee Function Type:",
        list(ee_functions.keys()),
        format_func=lambda x: ee_functions[x]['name']
    )
    
    # Show details for selected function
    func_info = ee_functions[selected_ee_func]
    
    col1, col2 = st.columns([2, 1])
    
    with col1:
        st.markdown(f"""
        ### {func_info['name']}
        **Formula:** `{func_info['formula']}`
        
        **Description:** {func_info['description']}
        """)
    
    with col2:
        st.markdown("**Formula Type:**")
        st.info(func_info['formula'])
    
    # Update session state
    st.session_state.current_params["ee_function"] = selected_ee_func

# ============================================================================
# TAB 4: Run Simulation
# ============================================================================

with tab4:
    st.header("🎮 Run Simulation")
    
    # Visualization mode mapping
    viz_map = {
        "Standard Plot": "standard",
        "Interactive 3D": "interactive"
    }
    
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("📊 Visualization Mode")
        viz_mode = st.radio(
            "Select visualization:",
            list(viz_map.keys()),
            horizontal=False
        )
    
    with col2:
        st.subheader("📋 Simulation Summary")
        
        params = st.session_state.current_params
        
        st.write("**Parameters to be used:**")
        st.write(f"- Ee0: {params['Ee0']:.2f}")
        st.write(f"- ke: {params['ke']:.2f}")
        st.write(f"- Ee function: {params.get('ee_function', 'quadratic_cubic')}")
        st.write(f"- Tmax: {params['Tmax']:.1f} N")
        st.write(f"- m0: {params['m0']:.1f} kg")
    
    st.divider()
    
    # Run button
    if st.button("🚀 RUN SIMULATION", use_container_width=True, type="primary"):
        with st.spinner("🛰️ Running simulation... (this may take up to a minute)"):
            try:
                # Run simulation
                t_arr, states_arr, command_arr, J_arr, p = simulate(
                    scenario_id=params["scenario_id"] if params.get("scenario_id") else 1, 
                    verbose=False,
                    params_override=params
                )
                
                # Store results
                st.session_state.simulation_results = {
                    't_arr': t_arr,
                    'states_arr': states_arr,
                    'command_arr': command_arr,
                    'J_arr': J_arr,
                    'params': params
                }
                
                # Calculate metrics
                if len(states_arr) > 0:
                    final_pos = states_arr[-1, 0:3]
                    final_vel = states_arr[-1, 3:6]
                    final_mass = states_arr[-1, 6]
                    
                    landing_error = np.linalg.norm(final_pos - params['rfd'])
                    landing_speed = np.linalg.norm(final_vel)
                    
                    # Calculate electric energy consumed
                    # Approximate efficiency: 12 Watts per Newton of thrust
                    power_W = 12.0 * command_arr[:, 4]  
                    energy_J = np.trapezoid(power_W, t_arr)
                    energy_Wh = energy_J / 3600.0
                    capacity_mAh = (energy_Wh / 14.8) * 1000.0  # Assumes typical 4S 14.8V battery
                    
                    st.session_state.landing_error = landing_error
                    st.session_state.landing_speed = landing_speed
                    st.session_state.energy_Wh = energy_Wh
                    st.session_state.capacity_mAh = capacity_mAh
                    st.session_state.steps = len(t_arr)
                    st.session_state.final_time = t_arr[-1]
                
                st.success("✓ Simulation completed successfully!")
                
                # Show results
                st.divider()
                st.subheader("🎯 Landing Results")
                
                metric_col1, metric_col2, metric_col3, metric_col4 = st.columns(4)
                
                with metric_col1:
                    st.metric("Landing Error", f"{landing_error:.6f} m", 
                             delta="✓ Excellent" if landing_error < 0.5 else "✓ Good" if landing_error < 5.0 else "⚠ Fair")
                
                with metric_col2:
                    st.metric("Final Velocity", f"{landing_speed:.6f} m/s",
                             delta="✓ Stopped" if landing_speed < 0.5 else "✓ Slow" if landing_speed < 2.0 else "⚠ Fast")
                
                with metric_col3:
                    st.metric("Battery Consumed", f"{energy_Wh:.2f} Wh",
                              delta=f"{capacity_mAh:.0f} mAh")
                
                with metric_col4:
                    st.metric("Time Steps", f"{len(t_arr)}")
                
                # Visualization
                st.divider()
                st.subheader("📈 Trajectory Visualization")
                
                if viz_map[viz_mode] == "interactive":
                    # Generate the interactive 3D Plotly figure directly in Streamlit
                    fig = go.Figure()

                    # Trajectory
                    fig.add_trace(go.Scatter3d(
                        x=states_arr[:, 0], y=states_arr[:, 1], z=states_arr[:, 2],
                        mode='lines+markers',
                        marker=dict(size=3, color=t_arr, colorscale='Viridis',
                                    showscale=True, colorbar=dict(title="Time (s)", x=0.85)),
                        line=dict(color='lightblue', width=3),
                        text=[f"t={t:.2f}s<br>v={np.linalg.norm(states_arr[i,3:6]):.2f}m/s<br>"
                              f"dist={np.linalg.norm(states_arr[i,0:3] - params['rfd']):.1f}m<br>"
                              f"alt={states_arr[i,2]:.1f}m"
                              for i, t in enumerate(t_arr)],
                        hoverinfo="text",
                        name="Trajectory"
                    ))

                    # Start / Target markers
                    fig.add_trace(go.Scatter3d(
                        x=[params['pos0'][0]], y=[params['pos0'][1]], z=[params['pos0'][2]],
                        mode='markers+text', marker=dict(size=8, color='green'),
                        text=['Start'], textposition='top center', name='Start'
                    ))
                    fig.add_trace(go.Scatter3d(
                        x=[params['rfd'][0]], y=[params['rfd'][1]], z=[params['rfd'][2]],
                        mode='markers+text', marker=dict(size=10, color='red'),
                        text=['Target'], textposition='top center', name='Target'
                    ))

                    # Obstacles as red transparent spheres
                    for obs in params.get("obstacles", []):
                        c, rad = obs["centre"], obs["radius"]
                        u = np.linspace(0, 2*np.pi, 25)
                        v = np.linspace(0, np.pi, 15)
                        xs = c[0] + rad * np.outer(np.cos(u), np.sin(v))
                        ys = c[1] + rad * np.outer(np.sin(u), np.sin(v))
                        zs = c[2] + rad * np.outer(np.ones_like(u), np.cos(v))
                        fig.add_trace(go.Surface(
                            x=xs, y=ys, z=zs, opacity=0.3,
                            colorscale=[[0, 'red'], [1, 'red']], showscale=False,
                            name=f'Obstacle r={rad}m', hoverinfo='skip'
                        ))

                    fig.update_layout(
                        title="Quadrotor Interactive 3D Trajectory",
                        scene=dict(xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Altitude Z (m)",
                                   aspectmode='data'),
                        height=700,
                        margin=dict(r=10, l=10, b=10, t=40)
                    )
                    
                    st.plotly_chart(fig, use_container_width=True)
                
                elif viz_map[viz_mode] == "standard":
                    # Create standard matplotlib plots
                    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
                    
                    # 3D trajectory
                    ax3d = fig.add_subplot(221, projection='3d')
                    ax3d.plot(states_arr[:, 0], states_arr[:, 1], states_arr[:, 2], 'b-', linewidth=2)
                    ax3d.scatter(*params['pos0'], color='green', s=100, label='Start')
                    ax3d.scatter(*params['rfd'], color='red', s=100, label='Target')
                    
                    # Draw obstacles
                    for obs in params.get("obstacles", []):
                        c = obs["centre"]
                        r = obs["radius"]
                        u = np.linspace(0, 2 * np.pi, 20)
                        v = np.linspace(0, np.pi, 10)
                        xs = c[0] + r * np.outer(np.cos(u), np.sin(v))
                        ys = c[1] + r * np.outer(np.sin(u), np.sin(v))
                        zs = c[2] + r * np.outer(np.ones_like(u), np.cos(v))
                        ax3d.plot_surface(xs, ys, zs, color='red', alpha=0.15)
                    
                    ax3d.set_xlabel('X (m)')
                    ax3d.set_ylabel('Y (m)')
                    ax3d.set_zlabel('Z (m)')
                    ax3d.set_title('3D Trajectory')
                    ax3d.legend()
                    
                    # Distance to target
                    distances = np.linalg.norm(states_arr[:, 0:3] - params['rfd'], axis=1)
                    axes[0, 1].plot(t_arr, distances, 'b-', linewidth=2)
                    axes[0, 1].set_xlabel('Time (s)')
                    axes[0, 1].set_ylabel('Distance (m)')
                    axes[0, 1].set_title('Distance to Landing Site')
                    axes[0, 1].grid(True, alpha=0.3)
                    
                    # Velocity magnitude
                    velocities = np.linalg.norm(states_arr[:, 3:6], axis=1)
                    axes[1, 0].plot(t_arr, velocities, 'g-', linewidth=2)
                    axes[1, 0].set_xlabel('Time (s)')
                    axes[1, 0].set_ylabel('Speed (m/s)')
                    axes[1, 0].set_title('Velocity Magnitude')
                    axes[1, 0].grid(True, alpha=0.3)
                    
                    # Thrust
                    thrust = command_arr[:, 4]
                    axes[1, 1].plot(command_arr[:, 0], thrust, 'r-', linewidth=2)
                    axes[1, 1].axhline(params["Tmax"], color='r', linestyle='--', alpha=0.5, label=f"Tmax={params['Tmax']}N")
                    axes[1, 1].set_xlabel('Time (s)')
                    axes[1, 1].set_ylabel('Thrust (N)')
                    axes[1, 1].set_title('Commanded Thrust')
                    axes[1, 1].grid(True, alpha=0.3)
                    axes[1, 1].legend()
                    
                    plt.tight_layout()
                    st.pyplot(fig)

            except Exception as e:
                st.error(f"❌ Simulation failed: {str(e)}")
                import traceback
                st.code(traceback.format_exc())

            st.caption("The Vehicle Arrives 2 Meters above Z Coordinate of the target to ensure Safe Landing")

# ============================================================================
# TAB 5: Results
# ============================================================================

with tab5:
    st.header("📊 Results & Analysis")
    
    if st.session_state.simulation_results is None:
        st.info("💡 Run a simulation first to see results!")
    else:
        results = st.session_state.simulation_results
        t_arr = results['t_arr']
        states_arr = results['states_arr']
        command_arr = results['command_arr']
        J_arr = results['J_arr']
        params = results['params']
        
        st.subheader("✅ Simulation Summary")
        
        summary_col1, summary_col2 = st.columns(2)
        
        with summary_col1:
            st.markdown("""
            **Scenario Details:**
            """)
            st.write(f"- Initial Position: {np.linalg.norm(params['pos0']):.1f} m")
            st.write(f"- Initial Velocity: {np.linalg.norm(params['vel0']):.3f} m/s")
            st.write(f"- Target Distance: {np.linalg.norm(params['pos0'] - params['rfd']):.1f} m")
            st.write(f"- Ee Function: {params.get('ee_function', 'quadratic_cubic')}")
        
        with summary_col2:
            st.markdown("""
            **Control Parameters:**
            """)
            st.write(f"- Ee0: {params['Ee0']:.2f}")
            st.write(f"- ke: {params['ke']:.2f}")
            st.write(f"- Tmax: {params['Tmax']:.1f} N")
            st.write(f"- m0: {params['m0']:.1f} kg")
        
        st.divider()
        
        # Key metrics
        st.subheader("🎯 Landing Metrics")
        
        landing_error = np.linalg.norm(states_arr[-1, 0:3] - params['rfd'])
        landing_speed = np.linalg.norm(states_arr[-1, 3:6])
        
        power_W = 12.0 * command_arr[:, 4]
        energy_Wh = np.trapezoid(power_W, t_arr) / 3600.0
        
        metric_row1, metric_row2 = st.columns(2)
        
        with metric_row1:
            st.metric("❎ Landing Error", f"{landing_error:.6f} m")
            st.metric("🔋 Energy Consumed", f"{energy_Wh:.2f} Wh")
        
        with metric_row2:
            st.metric("🎯 Landing Speed", f"{landing_speed:.6f} m/s")
            st.metric("⏱️ Total Time", f"{t_arr[-1]:.1f} s ({len(t_arr)} steps)")
        
        st.divider()
        
        # Detailed plots
        st.subheader("📈 Trajectory Analysis")
        
        plot_col1, plot_col2 = st.columns(2)
        
        with plot_col1:
            # Distance plot
            distances = np.linalg.norm(states_arr[:, 0:3] - params['rfd'], axis=1)
            
            fig_dist = go.Figure()
            fig_dist.add_trace(go.Scatter(
                x=t_arr, y=distances,
                mode='lines',
                fill='tozeroy',
                name='Distance to Target'
            ))
            fig_dist.update_layout(
                title="Distance to Landing Site",
                xaxis_title="Time (s)",
                yaxis_title="Distance (m)",
                hovermode='x unified',
                height=400,
                template="plotly_white"
            )
            st.plotly_chart(fig_dist, use_container_width=True)
        
        with plot_col2:
            # Velocity plot
            velocities = np.linalg.norm(states_arr[:, 3:6], axis=1)
            
            fig_vel = go.Figure()
            fig_vel.add_trace(go.Scatter(
                x=t_arr, y=velocities,
                mode='lines',
                fill='tozeroy',
                name='Speed',
                line=dict(color='green')
            ))
            fig_vel.update_layout(
                title="Velocity Magnitude",
                xaxis_title="Time (s)",
                yaxis_title="Speed (m/s)",
                hovermode='x unified',
                height=400,
                template="plotly_white"
            )
            st.plotly_chart(fig_vel, use_container_width=True)
        
        plot_col3, plot_col4 = st.columns(2)
        
        with plot_col3:
            # Thrust plot
            thrust = command_arr[:, 4]
            
            fig_thrust = go.Figure()
            fig_thrust.add_trace(go.Scatter(
                x=t_arr, y=thrust,
                mode='lines',
                fill='tozeroy',
                name='Thrust Magnitude',
                line=dict(color='purple')
            ))
            
            # Add Tmax line
            fig_thrust.add_trace(go.Scatter(
                x=[0, t_arr[-1]], y=[params['Tmax'], params['Tmax']],
                mode='lines',
                name='Max Thrust',
                line=dict(color='red', dash='dash')
            ))
            
            fig_thrust.update_layout(
                title="Commanded Thrust Magnitude",
                xaxis_title="Time (s)",
                yaxis_title="Thrust (N)",
                hovermode='x unified',
                height=400,
                template="plotly_white"
            )
            st.plotly_chart(fig_thrust, use_container_width=True)
            
        with plot_col4:
            # Density plot
            altitudes = states_arr[:, 2]
            densities = np.array([isa_density(max(a, 0)) for a in altitudes])
            
            fig_dens = go.Figure()
            fig_dens.add_trace(go.Scatter(
                x=t_arr, y=densities,
                mode='lines',
                fill='tozeroy',
                name='Air Density',
                line=dict(color='brown')
            ))
            
            fig_dens.update_layout(
                title="Air Density (ISA model)",
                xaxis_title="Time (s)",
                yaxis_title="Air Density (kg/m³)",
                hovermode='x unified',
                height=400,
                template="plotly_white"
            )
            st.plotly_chart(fig_dens, use_container_width=True)
# ============================================================================
# Footer
# ============================================================================

st.divider()
st.markdown("""
---
**SMC-IO Asteroid Landing Guidance System**
*Based on research by V.S. Shincy & Dr. Satadal Ghosh (2023)*

Built with: Passion | Creativity | Love | 

[🚀 SMC-IO Asteroid Landing Guidance](https://slidingmodeguidence.streamlit.app/) | [📋 Documentation](https://arc.aiaa.org/doi/10.2514/1.A35412) | [🐛 Report Issues](dhairya.param@iitgn.ac.in)
""")

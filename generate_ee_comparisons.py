import numpy as np
import matplotlib.pyplot as plt
from quadrotor_smc_guidance import simulate
from quadrotor_scenarios import get_all_scenarios, build_params

def main():
    scenarios = get_all_scenarios()
    ee_functions = ["linear", "quadratic", "cubic", "exponential", "saturation", "quadratic_cubic"]
    
    for sid, sc in scenarios.items():
        print(f"\nProcessing Scenario {sid}: {sc['name']}")
        
        results = {}
        for ee_func in ee_functions:
            print(f"  Running Ee function: {ee_func}...")
            params = build_params(sid)
            params["ee_function"] = ee_func
            try:
                t, s, c, j, p = simulate(scenario_id=sid, verbose=False, params_override=params)
                results[ee_func] = {'t': t, 's': s, 'c': c, 'j': j, 'p': p}
            except Exception as e:
                print(f"    Failed: {e}")
                
        # Now plot for this scenario
        fig, axes = plt.subplots(2, 3, figsize=(16, 10))
        fig.suptitle(f"{sc['name']} - Ee Function Comparison", fontsize=16, fontweight='bold')
        
        # 1. Distance
        ax = axes[0, 0]
        for ee_func, r in results.items():
            if len(r['s']) > 0:
                dists = np.linalg.norm(r['s'][:, 0:3] - r['p']['rfd'], axis=1)
                ax.plot(r['t'], dists, linewidth=2, label=ee_func)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance (m)')
        ax.set_title('Distance to Target')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # 2. Speed
        ax = axes[0, 1]
        for ee_func, r in results.items():
            if len(r['s']) > 0:
                speeds = np.linalg.norm(r['s'][:, 3:6], axis=1)
                ax.plot(r['t'], speeds, linewidth=2, label=ee_func)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed (m/s)')
        ax.set_title('Speed Over Time')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # 3. Control Effort
        ax = axes[0, 2]
        for ee_func, r in results.items():
            if len(r['j']) > 0:
                ax.plot(r['j'][:, 0], r['j'][:, 1], linewidth=2, label=ee_func)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Cumulative Effort')
        ax.set_title('Control Effort')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # 4. Thrust
        ax = axes[1, 0]
        for ee_func, r in results.items():
            if len(r['c']) > 0:
                ax.plot(r['c'][:, 0], r['c'][:, 4], linewidth=2, label=ee_func)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Thrust (N)')
        ax.set_title('Commanded Thrust')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # 5. Energy Usage (Total Work/Induced Power)
        ax = axes[1, 1]
        try:
            from atmosphere_model import isa_density
        except ImportError:
            isa_density = lambda z: 1.225 # fallback
            
        for ee_func, r in results.items():
            if len(r['s']) > 0 and len(r['c']) > 0:
                dt = r['p'].get('dt', 0.02)
                A = r['p'].get('A', 0.04)
                
                thrust_mag = r['c'][:, 4]
                z_vals = r['s'][:, 2]
                rho = np.array([isa_density(max(z, 0)) for z in z_vals])
                
                # Approximate induced thrust power: P = T^1.5 / sqrt(2 * rho * A)
                power = thrust_mag**1.5 / np.sqrt(2 * rho * A + 1e-6)
                
                energy = np.cumsum(power) * dt
                ax.plot(r['t'], energy, linewidth=2, label=ee_func)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Energy (Joules)')
        ax.set_title('Estimated Energy Usage')
        ax.grid(True, alpha=0.3)
        ax.legend()

        # 6. Altitude Profile
        ax = axes[1, 2]
        for ee_func, r in results.items():
            if len(r['s']) > 0:
                ax.plot(r['t'], r['s'][:, 2], linewidth=2, label=ee_func)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Altitude Z (m)')
        ax.set_title('Altitude Profile')
        ax.grid(True, alpha=0.3)
        ax.legend()

        plt.tight_layout()
        filename = f"FS{sid}_Ee_Comparison.png"
        fig.savefig(filename, dpi=150)
        print(f"Saved {filename}")

if __name__ == '__main__':
    main()

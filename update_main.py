#!/usr/bin/env python
"""Update the main section in smc_io_guidance.py"""

with open('smc_io_guidance.py', 'r') as f:
    lines = f.readlines()

# Find the line with "if __name__" and replace from there
start_idx = None
for i, line in enumerate(lines):
    if 'if __name__' in line:
        start_idx = i
        break

if start_idx is None:
    print("Could not find main section")
    exit(1)

# New main section
new_main = '''if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="SMC-IO Guidance for asteroid soft landing"
    )
    parser.add_argument(
        "--scenario", type=int, default=1, choices=range(1, 8),
        help="Landing scenario number (1-7, default: 1)"
    )
    parser.add_argument(
        "--mode", type=str, default="standard",
        choices=["standard", "interactive", "dashboard", "animate", "heatmap", "compare"],
        help="Visualization mode (default: standard)"
    )
    parser.add_argument(
        "--no-plot", action="store_true",
        help="Disable plotting (just run simulation)"
    )
    parser.add_argument(
        "--save", type=str, default=None,
        help="Save plots/animation with this filename prefix"
    )
    args = parser.parse_args()

    if args.mode == "compare":
        results, metrics = compare_all_scenarios(verbose=False)
    else:
        t_arr, states_arr, command_arr, J_arr, params = simulate(
            ls_number=args.scenario, verbose=True
        )

        if not args.no_plot:
            if args.mode == "standard":
                plot_results(t_arr, states_arr, command_arr, J_arr, params,
                           save_prefix=args.save)
            elif args.mode == "interactive":
                plot_interactive_3d(t_arr, states_arr, command_arr, params)
            elif args.mode == "dashboard":
                app = create_dash_app(t_arr, states_arr, command_arr, J_arr, params)
                if app:
                    app.run_server(debug=True, port=8050)
            elif args.mode == "animate":
                save_as = f"{args.save}_animation.mp4" if args.save else None
                create_animation(t_arr, states_arr, command_arr, params, save_path=save_as)
            elif args.mode == "heatmap":
                plot_control_heatmap(t_arr, states_arr, command_arr, params)
'''

new_lines = lines[:start_idx] + [new_main + '\n']

with open('smc_io_guidance.py', 'w') as f:
    f.writelines(new_lines)

print(f"Updated main section (removed {len(lines) - len(new_lines)} old lines)")
print(f"Total lines now: {len(new_lines)}")

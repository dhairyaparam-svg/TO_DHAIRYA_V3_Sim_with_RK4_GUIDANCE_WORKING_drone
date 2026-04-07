# Quadrotor SMC-IO Trajectory Guidance with Atmospheric Effects

Adapt the existing SMC-IO (Sliding-Mode-Control-based Instantaneously Optimal) guidance law — currently targeting asteroid soft-landing — to **quadrotor drone trajectory generation** in Earth's atmosphere. The new simulator adds:

1. **Wind disturbances** (constant + Dryden gust model)
2. **Density variation with altitude** (ISA standard atmosphere)
3. **Obstacle avoidance** (spherical keep-out zones via penalty constraints)

The core SMC sliding-surface (heading-error ξ), the SLSQP-at-every-step optimisation, and the RK4 integrator are preserved from the original project.

> [!IMPORTANT]
> This creates **three brand-new files** in the project directory. No existing files are modified.

## Proposed Changes

### Atmosphere & Environment

#### [NEW] [atmosphere_model.py](file:///c:/Users/IITGN/Downloads/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/atmosphere_model.py)

| Component | Detail |
|---|---|
| `isa_density(alt)` | ISA standard-atmosphere air density ρ(h) for 0–86 km |
| `drag_acceleration(vel, alt, Cd, A, m)` | Aerodynamic drag: a_drag = −½ρ·Cd·A·‖v_air‖·v_air / m |
| `wind_model(t, pos, wind_params)` | Wind = constant base + sinusoidal gust with altitude-dependent magnitude |
| `obstacle_field` | List of [(centre, radius)](file:///c:/Users/IITGN/Downloads/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/smc_io_guidance.py#856-862) spheres; helper `obstacle_penalty(pos, obstacles, margin)` returns repulsive acceleration vector |

---

### Scenarios

#### [NEW] [quadrotor_scenarios.py](file:///c:/Users/IITGN/Downloads/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/quadrotor_scenarios.py)

Mirrors [scenarios.py](file:///c:/Users/IITGN/Downloads/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/scenarios.py) structure with quadrotor-specific defaults:

| Parameter | Value |
|---|---|
| Mass | 2.5 kg |
| Max thrust | 40 N (≈ 1.6g) |
| Cd / A | 1.0 / 0.04 m² |
| g | 9.81 m/s² (downward) |
| SMC gains Ee0, ke | 0.5, 1.0 |
| dt | 0.02 s |

**4 pre-defined scenarios:**

| # | Name | Description |
|---|---|---|
| 1 | Point-to-Point Calm | 100 m lateral, 50 m climb, no wind, no obstacles |
| 2 | Windy Delivery | 200 m flight through 5 m/s crosswind |
| 3 | Urban Obstacle Course | Navigate around 3 spherical obstacles |
| 4 | High-Altitude Density Change | Climb from 0 m to 500 m — density drops ~6% |

---

### Core Guidance

#### [NEW] [quadrotor_smc_guidance.py](file:///c:/Users/IITGN/Downloads/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/quadrotor_smc_guidance.py)

Adapted from [smc_io_guidance.py](file:///c:/Users/IITGN/Downloads/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/smc_io_guidance.py), key differences:

| Aspect | Asteroid (original) | Quadrotor (new) |
|---|---|---|
| Gravity | Triaxial ellipsoidal harmonics | Constant g = [0, 0, −9.81] |
| External forces | Coriolis, centrifugal, Euler | Aerodynamic drag + wind |
| State vector | 7 (x,y,z, vx,vy,vz, m) | 7 (x,y,z, vx,vy,vz, m) |
| Sliding surface | Heading-error ξ (same) | Heading-error ξ (same) |
| Constraints | Velocity bounds + thrust limit | Velocity bounds + thrust limit + obstacle penalty |
| Integrator | RK4 (same) | RK4 (same) |

**Functions:**

- `environment_acceleration(pos, vel, t, params)` — gravity + drag + wind (replaces [gravity_model](file:///c:/Users/IITGN/Downloads/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/smc_io_guidance.py#70-86))
- [heading_error_coefficients(r, v, rfd, g)](file:///c:/Users/IITGN/Downloads/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/smc_io_guidance.py#151-182) — **identical** to original
- [compute_distance_based_ee(...)](file:///c:/Users/IITGN/Downloads/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/smc_io_guidance.py#184-239) — **identical** to original
- [build_constraints(states, params)](file:///c:/Users/IITGN/Downloads/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/smc_io_guidance.py#241-324) — same structure, adds obstacle avoidance inequality
- [guidance(t, states, params)](file:///c:/Users/IITGN/Downloads/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/smc_io_guidance.py#335-349) → SLSQP solver (same)
- [state_equations_rk4(states, command, params)](file:///c:/Users/IITGN/Downloads/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/smc_io_guidance.py#355-378) — uses `environment_acceleration`
- [rk4_step(...)](file:///c:/Users/IITGN/Downloads/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/smc_io_guidance.py#380-388) — **identical**
- [simulate(scenario_id, verbose)](file:///c:/Users/IITGN/Downloads/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/smc_io_guidance.py#498-588) — same loop
- [plot_results(...)](file:///c:/Users/IITGN/Downloads/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING/smc_io_guidance.py#594-668) — 3D trajectory + position/velocity/acceleration grids + obstacle spheres overlay
- CLI entry point with `--scenario`, `--mode`, `--save`

---

## Verification Plan

### Automated Tests

Run the following script to verify all 4 scenarios converge to the target position:

```bash
cd "c:\Users\IITGN\Downloads\TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING-20260401T112830Z-1-001\TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING\TO_DHAIRYA_V3_Sim_with_RK4_GUIDANCE_WORKING"
python quadrotor_smc_guidance.py --scenario 1 --no-plot
python quadrotor_smc_guidance.py --scenario 2 --no-plot
python quadrotor_smc_guidance.py --scenario 3 --no-plot
python quadrotor_smc_guidance.py --scenario 4 --no-plot
```

**Pass criteria:** Each scenario prints final position error < 5 m and landing speed < 1 m/s.

### Visual Verification

```bash
python quadrotor_smc_guidance.py --scenario 3 --mode standard
```

Verify obstacle spheres are rendered and the trajectory curves around them.

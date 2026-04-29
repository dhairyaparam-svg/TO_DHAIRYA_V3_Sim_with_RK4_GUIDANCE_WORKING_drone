"""
Comprehensive Ee-Function Comparison Study
===========================================
Mirrors the publication-style results of Shincy & Ghosh (2023):
  - Full simulation loop with per-step timing and sliding-variable recording
  - Table 1: Summary results (mf, Δm, ∫|a|dt, tf, pos-error, |vf|, Tr, Ts, status)
  - Table 2: Computational-time statistics per case
  - Table 3: Convergence analysis (what converged, what didn't, why)
  - Plots: 6-panel comparison figure per scenario (saved as PNG)
  - All tables saved as .txt and .csv files
"""

import time
import csv
import sys
import traceback

import numpy as np
import matplotlib
matplotlib.use("Agg")           # non-interactive backend – safe for batch runs
import matplotlib.pyplot as plt

from atmosphere_model import isa_density
from quadrotor_scenarios import build_params, get_all_scenarios
from quadrotor_smc_guidance import (
    guidance, rk4_step, heading_error_coefficients,
    environment_acceleration,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
EE_FUNCTIONS   = ["linear", "quadratic", "cubic", "exponential",
                  "saturation", "quadratic_cubic"]

SE_REACHED_THR = 0.05   # rad – sliding variable "reached" threshold
CONV_DIST      = 0.5    # m   – position convergence radius
CONV_SPEED     = 0.5    # m/s – velocity convergence threshold


# ---------------------------------------------------------------------------
# Extended simulation loop
# ---------------------------------------------------------------------------
def simulate_full(sid, ee_func):
    """
    Run one scenario/Ee-function combination and return a rich result dict.

    Tracked per step (in addition to standard states / commands):
      Se_arr      – sliding variable (heading error) history
      comp_times  – wall-clock seconds per guidance call
      Tr          – time when Se first falls below SE_REACHED_THR  (None if never)
      converged   – bool
      fail_reason – string if not converged
    """
    params = build_params(sid)
    params["ee_function"] = ee_func

    t0  = params["t0"]
    dt  = params["dt"]
    m0  = params["m0"]

    # Run until convergence.  Use a flat 600 s safety ceiling — do NOT cap
    # at params["tf"]*1.05, which is only a rough mission-time hint and would
    # cause spurious timeouts for slow Ee functions.
    SAFETY_CEILING = 600.0
    tf_max  = SAFETY_CEILING
    n_steps = int((tf_max - t0) / dt)

    states = np.concatenate([params["pos0"], params["vel0"], [m0]])

    t_list, states_list, cmd_list = [], [], []
    J_list, Se_list, ct_list      = [], [], []
    J_cum  = 0.0

    Tr          = None      # reaching time
    Ts          = None      # sliding time (from Tr to end)
    converged   = False
    fail_reason = f"Safety ceiling reached ({SAFETY_CEILING:.0f}s) — never converged"
    ground_hit  = False

    # vfd is the desired final velocity (hover = zero by default)
    vfd = params.get("vfd", np.zeros(3))

    for _i in range(n_steps):
        r    = states[0:3]
        v    = states[3:6]
        dist = np.linalg.norm(r - params["rfd"])
        spd  = np.linalg.norm(v - vfd)   # relative to target velocity

        # --- Convergence check ---
        if dist < CONV_DIST and spd < CONV_SPEED:
            converged   = True
            fail_reason = "—"
            break

        # --- Guidance command (timed) ---
        t_wall_start = time.perf_counter()
        try:
            cmd = guidance(t0, states, params)
        except Exception as exc:
            fail_reason = f"guidance_error @ t={t0:.2f}s: {exc}"
            break
        ct = time.perf_counter() - t_wall_start

        # --- Sliding variable (computed at current state, before propagation) ---
        g_env = environment_acceleration(r, v, t0, params)
        _, _, _, _, Se = heading_error_coefficients(r, v, params["rfd"], g_env)

        # --- Reaching time Tr ---
        if Tr is None and Se < SE_REACHED_THR:
            Tr = t0

        # --- Propagate ---
        states = rk4_step(states, cmd, t0, params)

        # Ground-stop
        if states[2] <= 0.0:
            states[2] = 0.0
            if states[5] < 0.0:
                states[5] = 0.0
            if np.linalg.norm(states[0:3] - params["rfd"]) > CONV_DIST:
                ground_hit  = True
                fail_reason = "ground_collision (hit z=0 before target)"

        thrust_mag = np.linalg.norm(cmd) * states[6]
        J_cum     += np.linalg.norm(cmd) * dt

        # All appends together so every array stays the same length
        t_list.append(t0)
        states_list.append(states.copy())
        cmd_list.append([t0, cmd[0], cmd[1], cmd[2], thrust_mag])
        J_list.append([t0, J_cum])
        Se_list.append(Se)
        ct_list.append(ct)

        if ground_hit:
            break

        t0 += dt

    # If converged and Tr was found, Ts = tf - Tr
    if converged and Tr is not None:
        Ts = t0 - Tr
    elif converged:
        Ts = 0.0   # already on sliding surface from the start

    # Pack arrays
    t_arr  = np.array(t_list)
    s_arr  = np.array(states_list)  if states_list  else np.empty((0, 7))
    c_arr  = np.array(cmd_list)     if cmd_list      else np.empty((0, 5))
    j_arr  = np.array(J_list)       if J_list        else np.empty((0, 2))
    Se_arr = np.array(Se_list)      if Se_list        else np.empty(0)
    ct_arr = np.array(ct_list)      if ct_list        else np.empty(0)

    # Final-state metrics
    if len(s_arr):
        rf   = s_arr[-1, 0:3]
        vf   = s_arr[-1, 3:6]
        mf   = s_arr[-1,  6]
        tf_  = t_arr[-1]
    else:
        rf = params["pos0"].copy()
        vf = params["vel0"].copy()
        mf = m0
        tf_= 0.0

    pos_err = np.linalg.norm(rf - params["rfd"])
    vf_mag  = np.linalg.norm(vf)
    delta_m = m0 - mf
    CE      = J_cum

    return {
        "sid":         sid,
        "ee_func":     ee_func,
        "name":        params.get("name", f"FS{sid}"),
        "converged":   converged,
        "fail_reason": fail_reason,
        "tf":          tf_,
        "mf":          mf,
        "delta_m":     delta_m,
        "CE":          CE,
        "rf":          rf,
        "vf":          vf,
        "pos_err":     pos_err,
        "vf_mag":      vf_mag,
        "Tr":          Tr,
        "Ts":          Ts,
        "t_arr":       t_arr,
        "s_arr":       s_arr,
        "c_arr":       c_arr,
        "j_arr":       j_arr,
        "Se_arr":      Se_arr,
        "ct_arr":      ct_arr,
        "params":      params,
    }


# ---------------------------------------------------------------------------
# Table helpers
# ---------------------------------------------------------------------------
def _hdr(text, width=100):
    return "\n" + "=" * width + f"\n  {text}\n" + "=" * width


def _sep(widths, ch="─"):
    return "┼".join(ch * w for w in widths)


def format_table_1(all_results):
    """
    Table 1 – Summary Results
    Columns: FS | Ee_Func | Conv | tf(s) | Pos_Err(m) | |vf|(m/s)
             | ∫|a|dt | Δm(kg) | mf(kg) | Tr(s) | Ts(s)
    """
    cols = [
        ("FS",       4),  ("Ee Function",  18), ("Conv",  5),
        ("tf (s)",   8),  ("Pos_Err (m)",  12), ("|vf| (m/s)", 11),
        ("∫|a|dt",   9),  ("Δm (kg)",       9), ("mf (kg)",     9),
        ("Tr (s)",   9),  ("Ts (s)",         9),
    ]
    hdrs  = [c[0] for c in cols]
    ws    = [c[1] for c in cols]

    def row(r):
        Tr_str = f"{r['Tr']:.2f}" if r["Tr"] is not None else "—"
        Ts_str = f"{r['Ts']:.2f}" if r["Ts"] is not None else "—"
        return [
            f"FS{r['sid']}",
            r["ee_func"],
            "YES" if r["converged"] else "NO",
            f"{r['tf']:.2f}",
            f"{r['pos_err']:.4f}",
            f"{r['vf_mag']:.6f}",
            f"{r['CE']:.5f}",
            f"{r['delta_m']:.5f}",
            f"{r['mf']:.4f}",
            Tr_str,
            Ts_str,
        ]

    lines = [_hdr("TABLE 1 — Summary Results (all scenarios × Ee functions)")]
    fmt   = "  " + "  ".join(f"{{:<{w}}}" for w in ws)
    lines.append(fmt.format(*hdrs))
    lines.append("  " + _sep(ws))

    last_sid = None
    for r in all_results:
        if r["sid"] != last_sid:
            if last_sid is not None:
                lines.append("  " + _sep(ws, ch="·"))
            last_sid = r["sid"]
        lines.append(fmt.format(*row(r)))

    return "\n".join(lines)


def format_table_2(all_results):
    """
    Table 2 – Computational-Time Statistics per Case
    Columns: FS | Ee_Func | N_steps | Mean(ms) | Med(ms) | Max(ms) | Std(ms) | % of dt
    """
    cols = [
        ("FS",       4),  ("Ee Function",  18), ("N_steps", 9),
        ("Mean(ms)", 10), ("Med(ms)",      10), ("Max(ms)",   10),
        ("Std(ms)",  10), ("% of dt",       9),
    ]
    hdrs = [c[0] for c in cols]
    ws   = [c[1] for c in cols]

    def row(r):
        ct = r["ct_arr"]
        if len(ct) == 0:
            return [f"FS{r['sid']}", r["ee_func"], "0",
                    "—", "—", "—", "—", "—"]
        dt_ms  = r["params"]["dt"] * 1000.0
        mean_  = ct.mean()  * 1000
        med_   = np.median(ct) * 1000
        max_   = ct.max()   * 1000
        std_   = ct.std()   * 1000
        pct_   = mean_ / dt_ms * 100
        return [
            f"FS{r['sid']}",
            r["ee_func"],
            str(len(ct)),
            f"{mean_:.3f}",
            f"{med_:.3f}",
            f"{max_:.3f}",
            f"{std_:.3f}",
            f"{pct_:.1f}%",
        ]

    lines = [_hdr("TABLE 2 — Computational-Time Statistics per Case")]
    fmt   = "  " + "  ".join(f"{{:<{w}}}" for w in ws)
    lines.append(fmt.format(*hdrs))
    lines.append("  " + _sep(ws))

    last_sid = None
    for r in all_results:
        if r["sid"] != last_sid:
            if last_sid is not None:
                lines.append("  " + _sep(ws, ch="·"))
            last_sid = r["sid"]
        lines.append(fmt.format(*row(r)))

    return "\n".join(lines)


def format_table_3(all_results):
    """
    Table 3 – Convergence Analysis
    For converged cases: brief summary.
    For failed cases:    detailed diagnosis.
    """
    lines = [_hdr("TABLE 3 — Convergence Analysis")]

    converged_cases = [r for r in all_results if r["converged"]]
    failed_cases    = [r for r in all_results if not r["converged"]]

    lines.append(f"\n  CONVERGED  ({len(converged_cases)} / {len(all_results)} cases)\n")
    lines.append(f"  {'Case':<22}  {'tf(s)':>8}  {'Pos_Err(m)':>12}  {'|vf|(m/s)':>11}  "
                 f"{'Tr(s)':>8}  {'Ts(s)':>8}  {'Best_CE?':>9}")
    lines.append("  " + "─" * 90)

    # Mark the best (min) CE per scenario among converged cases
    best_ce = {}
    for r in converged_cases:
        sid = r["sid"]
        if sid not in best_ce or r["CE"] < best_ce[sid][1]:
            best_ce[sid] = (r["ee_func"], r["CE"])

    for r in converged_cases:
        is_best = "★ BEST" if best_ce.get(r["sid"], (None,))[0] == r["ee_func"] else ""
        Tr_s = f"{r['Tr']:.2f}" if r["Tr"] is not None else "—"
        Ts_s = f"{r['Ts']:.2f}" if r["Ts"] is not None else "—"
        lines.append(
            f"  FS{r['sid']}/{r['ee_func']:<18}  {r['tf']:>8.2f}  "
            f"{r['pos_err']:>12.4f}  {r['vf_mag']:>11.6f}  "
            f"{Tr_s:>8}  {Ts_s:>8}  {is_best:>9}"
        )

    if failed_cases:
        lines.append(f"\n  FAILED / DID NOT CONVERGE  ({len(failed_cases)} cases)\n")
        lines.append(f"  {'Case':<22}  {'Reason & Diagnosis'}")
        lines.append("  " + "─" * 90)
        for r in failed_cases:
            reason = r["fail_reason"]

            # Automatic diagnosis
            diag = []
            if "Timeout" in reason:
                if r["pos_err"] < 5.0:
                    diag.append("Nearly reached target but tf too short — increase tf")
                elif len(r["Se_arr"]) and r["Se_arr"][-1] > 0.3:
                    diag.append("Sliding variable never converged (Se_final>"
                                f"{r['Se_arr'][-1]:.2f} rad) — increase ke/Ee0 or tf")
                else:
                    diag.append("Trajectory diverged or speed remained high — "
                                "check velocity-profile parameters")
            elif "ground_collision" in reason:
                diag.append("Z-velocity not arrested before ground — "
                             "increase Tmax or reduce bz/cz to allow more braking")
            elif "guidance_error" in reason:
                diag.append("SLSQP infeasible: constraint set may be empty — "
                             "reduce ke so thrust limit is not overwhelmed")

            lines.append(f"  FS{r['sid']}/{r['ee_func']:<18}  Reason: {reason}")
            for d in diag:
                lines.append(f"  {'':22}  Diagnosis: {d}")
    else:
        lines.append("\n  All cases converged successfully.")

    return "\n".join(lines)


def format_table_4(all_results):
    """
    Table 4 – Per-Scenario Best/Worst Ee-Function Rankings
    Ranked by total control effort (lower is better).
    """
    lines = [_hdr("TABLE 4 — Per-Scenario Ee-Function Rankings (by Control Effort)")]

    scenarios = sorted(set(r["sid"] for r in all_results))
    for sid in scenarios:
        grp = [r for r in all_results if r["sid"] == sid]
        name = grp[0]["name"]
        lines.append(f"\n  {name}")
        lines.append(f"  {'Rank':<6} {'Ee Function':<20} {'Conv':>5}  "
                     f"{'∫|a|dt':>10}  {'Δm(kg)':>10}  {'|vf|(m/s)':>12}  {'Pos_Err(m)':>12}")
        lines.append("  " + "─" * 80)

        # Sort converged first, then by CE
        def sort_key(r):
            return (0 if r["converged"] else 1, r["CE"])

        for rank, r in enumerate(sorted(grp, key=sort_key), 1):
            tag = "★" if rank == 1 and r["converged"] else (" " if r["converged"] else "✗")
            lines.append(
                f"  {rank:<5}{tag} {r['ee_func']:<20} {'YES' if r['converged'] else 'NO':>5}  "
                f"{r['CE']:>10.5f}  {r['delta_m']:>10.5f}  "
                f"{r['vf_mag']:>12.6f}  {r['pos_err']:>12.4f}"
            )

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Save to CSV
# ---------------------------------------------------------------------------
def save_csv(all_results, filename="ee_comparison_results.csv"):
    fieldnames = ["FS", "ee_func", "converged", "fail_reason",
                  "tf_s", "pos_err_m", "vf_mag_ms", "CE", "delta_m_kg",
                  "mf_kg", "Tr_s", "Ts_s",
                  "ct_mean_ms", "ct_median_ms", "ct_max_ms", "ct_std_ms", "ct_pct_dt"]
    with open(filename, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fieldnames)
        w.writeheader()
        for r in all_results:
            ct = r["ct_arr"]
            dt_ms = r["params"]["dt"] * 1000.0
            if len(ct):
                ct_mean  = ct.mean()  * 1000
                ct_med   = np.median(ct) * 1000
                ct_max   = ct.max()   * 1000
                ct_std   = ct.std()   * 1000
                ct_pct   = ct_mean / dt_ms * 100
            else:
                ct_mean = ct_med = ct_max = ct_std = ct_pct = float("nan")
            w.writerow({
                "FS":           f"FS{r['sid']}",
                "ee_func":      r["ee_func"],
                "converged":    r["converged"],
                "fail_reason":  r["fail_reason"],
                "tf_s":         round(r["tf"],      4),
                "pos_err_m":    round(r["pos_err"],  6),
                "vf_mag_ms":    round(r["vf_mag"],   6),
                "CE":           round(r["CE"],        6),
                "delta_m_kg":   round(r["delta_m"],  6),
                "mf_kg":        round(r["mf"],        4),
                "Tr_s":         round(r["Tr"], 2) if r["Tr"] is not None else "",
                "Ts_s":         round(r["Ts"], 2) if r["Ts"] is not None else "",
                "ct_mean_ms":   round(ct_mean, 4),
                "ct_median_ms": round(ct_med,  4),
                "ct_max_ms":    round(ct_max,  4),
                "ct_std_ms":    round(ct_std,  4),
                "ct_pct_dt":    round(ct_pct,  2),
            })
    print(f"  → Saved {filename}")


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------
COLORS = plt.rcParams["axes.prop_cycle"].by_key()["color"]

# Smoothing window (number of samples).  dt=0.02 s → window=25 = 0.5 s of data.
# Adjust upward if chatter frequency is lower than the scenario's dt.
_SMOOTH_WIN = 25


def _smooth(y, win=_SMOOTH_WIN):
    """
    Causal moving-average smoother.
    Returns the smoothed array at the same length as y.
    Uses 'valid' convolution then pads the front so length is preserved.
    """
    if len(y) < win:
        return y.copy()
    kernel = np.ones(win) / win
    smoothed = np.convolve(y, kernel, mode="valid")
    # Pad the beginning with the raw signal so lengths match
    pad = np.full(len(y) - len(smoothed), y[0])
    return np.concatenate([pad, smoothed])


def _chatter_intensity(y, win=_SMOOTH_WIN):
    """
    Returns a scalar in [0, 1] indicating how much high-frequency content
    (chattering) is present.  0 = perfectly smooth; 1 = extremely chattery.
    """
    if len(y) < win + 1:
        return 0.0
    residual = y - _smooth(y, win)
    sig_range = np.ptp(y)
    if sig_range < 1e-12:
        return 0.0
    return float(np.std(residual) / (sig_range + 1e-12))


def _clip_n(r):
    """
    Return the index at which to clip plot arrays for result `r`.
    For converged cases: clip at the first step where dist < CONV_DIST
    (so the plot ends exactly when the drone enters the target zone).
    For failed cases: show all available data.
    """
    s = r["s_arr"]
    if not len(s):
        return 0
    rfd  = r["params"]["rfd"]
    dists = np.linalg.norm(s[:, 0:3] - rfd, axis=1)
    hits  = np.where(dists < CONV_DIST)[0]
    if len(hits):
        return int(hits[0]) + 1   # include the first step inside the zone
    return len(r["t_arr"])


def _plot_with_chatter(ax, t, y, label, color, linestyle, linewidth=2.0,
                       smooth_win=_SMOOTH_WIN):
    """
    Plot strategy:
      • Raw signal: faint, thin, same colour  (shows chattering honestly)
      • Smoothed signal: solid, full-weight line  (readable trend)
    The legend entry is attached only to the smoothed line.
    A chatter index χ = std(residual)/range is shown in the legend
    when χ > 2 %.
    """
    t  = np.asarray(t)
    y  = np.asarray(y)
    ys = _smooth(y, smooth_win)
    ci = _chatter_intensity(y, smooth_win)

    # Raw signal (faint)
    ax.plot(t, y, color=color, linestyle=linestyle, linewidth=0.4,
            alpha=0.20, zorder=2)

    # Smoothed signal (clean, labelled)
    lbl = f"{label}  [χ={ci:.2f}]" if ci > 0.02 else label
    ax.plot(t, ys, color=color, linestyle=linestyle, linewidth=linewidth,
            label=lbl, zorder=3, alpha=0.95)


def plot_scenario(sid, sc_name, scenario_results):
    """
    6-panel comparison figure.
    All series are CLIPPED at the first step where the drone enters CONV_DIST
    of the target — so plots end when the mission is actually accomplished.
    For failed cases the full trajectory is shown.

    Panels:
      [0,0] Distance to target     [0,1] Speed
      [0,2] Cumulative ctrl effort  [1,0] Commanded thrust
      [1,1] Battery / energy usage  [1,2] Altitude profile

    Each chattery signal: faint raw + bold smoothed line.
    χ = chatter index (std of residual / signal range) shown in legend.
    """
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(
        f"{sc_name} — Ee Function Comparison\n"
        "Bold = smoothed trend  |  Faint = raw signal  |  "
        "[χ=…] = chatter index  |  Series clipped at target arrival",
        fontsize=11, fontweight="bold")

    col = {r["ee_func"]: COLORS[i % len(COLORS)]
           for i, r in enumerate(scenario_results)}
    ls_map = {"linear": "-", "quadratic": "--", "cubic": "-.",
              "exponential": ":", "saturation": (0, (3, 1, 1, 1)),
              "quadratic_cubic": (0, (5, 2))}

    # ── 1. Distance to target ─────────────────────────────────────────────
    ax = axes[0, 0]
    for r in scenario_results:
        if not len(r["s_arr"]): continue
        n = _clip_n(r)
        dists = np.linalg.norm(r["s_arr"][:n, 0:3] - r["params"]["rfd"], axis=1)
        _plot_with_chatter(ax, r["t_arr"][:n], dists,
                           r["ee_func"], col[r["ee_func"]], ls_map.get(r["ee_func"], "-"))
    ax.axhline(CONV_DIST, color="k", linestyle=":", linewidth=1,
               label=f"Target radius {CONV_DIST} m")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Distance (m)")
    ax.set_title("Distance to Target"); ax.grid(True, alpha=0.3); ax.legend(fontsize=7)

    # ── 2. Speed ──────────────────────────────────────────────────────────
    ax = axes[0, 1]
    for r in scenario_results:
        if not len(r["s_arr"]): continue
        n = _clip_n(r)
        speeds = np.linalg.norm(r["s_arr"][:n, 3:6], axis=1)
        _plot_with_chatter(ax, r["t_arr"][:n], speeds,
                           r["ee_func"], col[r["ee_func"]], ls_map.get(r["ee_func"], "-"))
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Speed (m/s)")
    ax.set_title("Speed Over Time"); ax.grid(True, alpha=0.3); ax.legend(fontsize=7)

    # ── 3. Cumulative control effort (monotone → no smoothing needed) ─────
    ax = axes[0, 2]
    for r in scenario_results:
        if not len(r["j_arr"]): continue
        n = _clip_n(r)
        c = col[r["ee_func"]]; ls = ls_map.get(r["ee_func"], "-")
        ax.plot(r["j_arr"][:n, 0], r["j_arr"][:n, 1],
                color=c, linestyle=ls, linewidth=2.0, label=r["ee_func"])
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Cumulative ∫|a|dt")
    ax.set_title("Control Effort"); ax.grid(True, alpha=0.3); ax.legend(fontsize=7)

    # ── 4. Thrust force ───────────────────────────────────────────────────
    ax = axes[1, 0]
    for r in scenario_results:
        if not len(r["c_arr"]): continue
        n = _clip_n(r)
        _plot_with_chatter(ax, r["c_arr"][:n, 0], r["c_arr"][:n, 4],
                           r["ee_func"], col[r["ee_func"]], ls_map.get(r["ee_func"], "-"))
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Thrust (N)")
    ax.set_title("Commanded Thrust"); ax.grid(True, alpha=0.3); ax.legend(fontsize=7)

    # ── 5. Battery / Induced-power energy usage ───────────────────────────
    # E(t) = ∫ P dt  where P = T^1.5 / sqrt(2·ρ·A)  (actuator-disc model)
    ax = axes[1, 1]
    for r in scenario_results:
        if not len(r["s_arr"]) or not len(r["c_arr"]): continue
        n   = _clip_n(r)
        dt  = r["params"]["dt"]
        A   = r["params"]["A"]
        T   = r["c_arr"][:n, 4]                              # thrust magnitude [N]
        z   = r["s_arr"][:n, 2]
        rho = np.array([isa_density(max(float(zi), 0.0)) for zi in z])
        P   = T ** 1.5 / np.sqrt(2.0 * rho * A + 1e-9)      # induced power [W]
        E   = np.cumsum(P) * dt                              # cumulative energy [J]
        # Cumulative energy is monotone — plot direct (no smoothing needed)
        # Show instantaneous power faintly for context
        ax.plot(r["t_arr"][:n], P, color=col[r["ee_func"]],
                linestyle=ls_map.get(r["ee_func"], "-"),
                linewidth=0.4, alpha=0.18, zorder=2)
        ci = _chatter_intensity(P)
        lbl = f"{r['ee_func']}  [χ={ci:.2f}]" if ci > 0.02 else r["ee_func"]
        ax.plot(r["t_arr"][:n], E, color=col[r["ee_func"]],
                linestyle=ls_map.get(r["ee_func"], "-"),
                linewidth=2.0, label=lbl, zorder=3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Energy (J)  [bold=cumulative, faint=inst. power]")
    ax.set_title("Battery / Induced-Power Energy Usage")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=7)

    # ── 6. Altitude profile ───────────────────────────────────────────────
    ax = axes[1, 2]
    for r in scenario_results:
        if not len(r["s_arr"]): continue
        n = _clip_n(r)
        _plot_with_chatter(ax, r["t_arr"][:n], r["s_arr"][:n, 2],
                           r["ee_func"], col[r["ee_func"]], ls_map.get(r["ee_func"], "-"))
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Altitude Z (m)")
    ax.set_title("Altitude Profile"); ax.grid(True, alpha=0.3); ax.legend(fontsize=7)

    plt.tight_layout()
    fname = f"FS{sid}_Ee_Comparison.png"
    fig.savefig(fname, dpi=150)
    plt.close(fig)
    print(f"  → Saved {fname}")


# ---------------------------------------------------------------------------
# Table 5 — Initial & Final States per Case
# ---------------------------------------------------------------------------
def format_table_5(all_results):
    """
    Table 5 – Initial & Final States.
    Groups by scenario (initial pos/vel/target shown once per group).
    Per Ee-function: final position, final velocity, position error, speed.
    """
    lines = [_hdr("TABLE 5 — Initial & Final States per Case")]

    scenarios = sorted(set(r["sid"] for r in all_results))
    for sid in scenarios:
        grp   = [r for r in all_results if r["sid"] == sid]
        p     = grp[0]["params"]
        pos0  = p["pos0"]
        vel0  = p["vel0"]
        rfd   = p["rfd"]
        name  = grp[0]["name"]

        lines.append(f"\n  {name}")
        lines.append(
            f"  Initial position : ({pos0[0]:8.3f}, {pos0[1]:8.3f}, {pos0[2]:8.3f}) m")
        lines.append(
            f"  Initial velocity : ({vel0[0]:8.3f}, {vel0[1]:8.3f}, {vel0[2]:8.3f}) m/s")
        lines.append(
            f"  Target position  : ({rfd[0]:8.3f}, {rfd[1]:8.3f}, {rfd[2]:8.3f}) m")
        lines.append("")

        hdr = (f"  {'Ee Function':<18}  {'Conv':>5}  "
               f"{'xf (m)':>9} {'yf (m)':>9} {'zf (m)':>9}  "
               f"{'vxf(m/s)':>9} {'vyf(m/s)':>9} {'vzf(m/s)':>9}  "
               f"{'|Δpos|(m)':>10}  {'|vf|(m/s)':>10}")
        lines.append(hdr)
        lines.append("  " + "─" * (len(hdr) - 2))

        for r in grp:
            rf = r["rf"]
            vf = r["vf"]
            lines.append(
                f"  {r['ee_func']:<18}  {'YES' if r['converged'] else 'NO':>5}  "
                f"{rf[0]:>9.4f} {rf[1]:>9.4f} {rf[2]:>9.4f}  "
                f"{vf[0]:>9.4f} {vf[1]:>9.4f} {vf[2]:>9.4f}  "
                f"{r['pos_err']:>10.4f}  {r['vf_mag']:>10.6f}"
            )

    return "\n".join(lines)


def plot_comp_time(all_results):
    """Bar chart of mean computation time per case."""
    labels  = [f"FS{r['sid']}/{r['ee_func']}" for r in all_results]
    means   = [r["ct_arr"].mean() * 1000 if len(r["ct_arr"]) else 0
               for r in all_results]
    dt_ms   = all_results[0]["params"]["dt"] * 1000.0

    fig, ax = plt.subplots(figsize=(max(12, len(labels) * 0.7), 5))
    bars = ax.bar(range(len(labels)), means, color="steelblue", edgecolor="black", linewidth=0.5)
    ax.axhline(dt_ms, color="red", linestyle="--", linewidth=1.5,
               label=f"Step dt={dt_ms:.0f} ms")
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=7)
    ax.set_ylabel("Mean guidance computation time (ms)")
    ax.set_title("Computational Time per Guidance Call — All Cases")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    # Annotate % of dt
    for bar, m in zip(bars, means):
        pct = m / dt_ms * 100
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.05,
                f"{pct:.0f}%", ha="center", va="bottom", fontsize=6)

    plt.tight_layout()
    fig.savefig("Comp_Time_Summary.png", dpi=150)
    plt.close(fig)
    print("  → Saved Comp_Time_Summary.png")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    scenarios = get_all_scenarios()

    all_results = []    # flat list ordered by sid, then ee_func

    # ── Run all combinations ─────────────────────────────────────────────────
    total = len(scenarios) * len(EE_FUNCTIONS)
    done  = 0
    for sid in sorted(scenarios.keys()):
        sc = scenarios[sid]
        print(f"\n{'─'*70}")
        print(f"  Scenario FS{sid}: {sc['name']}")
        print(f"{'─'*70}")
        scenario_results = []
        for ee_func in EE_FUNCTIONS:
            done += 1
            print(f"  [{done:2d}/{total}]  Running Ee={ee_func} ...", end="", flush=True)
            try:
                res = simulate_full(sid, ee_func)
            except Exception as exc:
                print(f"  EXCEPTION: {exc}")
                traceback.print_exc()
                res = {
                    "sid": sid, "ee_func": ee_func,
                    "name": sc.get("name", f"FS{sid}"),
                    "converged": False,
                    "fail_reason": f"unhandled_exception: {exc}",
                    "tf": 0.0, "mf": 0.0, "delta_m": 0.0, "CE": 0.0,
                    "rf": np.zeros(3), "vf": np.zeros(3),
                    "pos_err": float("inf"), "vf_mag": float("inf"),
                    "Tr": None, "Ts": None,
                    "t_arr": np.empty(0), "s_arr": np.empty((0, 7)),
                    "c_arr": np.empty((0, 5)), "j_arr": np.empty((0, 2)),
                    "Se_arr": np.empty(0), "ct_arr": np.empty(0),
                    "params": build_params(sid),
                }
            status = "CONVERGED" if res["converged"] else f"FAILED ({res['fail_reason'][:40]})"
            ct_ms  = res["ct_arr"].mean() * 1000 if len(res["ct_arr"]) else 0
            print(f"  {status}  |  CE={res['CE']:.5f}  |  mean_ct={ct_ms:.2f}ms")
            all_results.append(res)
            scenario_results.append(res)

        # ── Per-scenario plots ────────────────────────────────────────────────
        plot_scenario(sid, sc["name"], scenario_results)

    # ── Computational-time summary plot ──────────────────────────────────────
    plot_comp_time(all_results)

    # ── Build table strings ──────────────────────────────────────────────────
    t1 = format_table_1(all_results)
    t2 = format_table_2(all_results)
    t3 = format_table_3(all_results)
    t4 = format_table_4(all_results)
    t5 = format_table_5(all_results)

    full_report = "\n".join([t1, "", t2, "", t3, "", t4, "", t5])

    # ── Print to console ─────────────────────────────────────────────────────
    print("\n" + full_report)

    # ── Save text report ─────────────────────────────────────────────────────
    report_file = "ee_comparison_report.txt"
    with open(report_file, "w", encoding="utf-8") as fh:
        fh.write(full_report)
    print(f"\n  → Full report saved to {report_file}")

    # ── Save CSV ─────────────────────────────────────────────────────────────
    save_csv(all_results)

    # ── Final summary ────────────────────────────────────────────────────────
    n_conv = sum(r["converged"] for r in all_results)
    print(f"\n{'='*70}")
    print(f"  DONE.  {n_conv}/{total} cases converged.")
    print(f"  Output files: ee_comparison_report.txt, ee_comparison_results.csv,")
    print(f"                FS*_Ee_Comparison.png, Comp_Time_Summary.png")
    print(f"{'='*70}")


if __name__ == "__main__":
    main()

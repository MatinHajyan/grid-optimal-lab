#!/usr/bin/env python3
"""
STEP 3 — Discretization study (compare different discretization scales)
=======================================================================

Supervisor's note 4: "solve the discrete problem and compare different
discretization scales together."

The grid stays fixed at 100x100. We vary the CAR's discretization knobs
ONE AT A TIME (so we can see the effect of each in isolation), then a final
combined refinement. Everything else (map, cost = sum of heights, Dijkstra)
is reused from Step 2.

Knobs studied:
  Phase 1 : heading resolution  (N_HEADINGS)  -> 8, 16, 24, 32
  Phase 2 : step length         (STEP_LEN)    -> 5, 3, 2
  Phase 3 : steering choices    (N_STEER)     -> 3, 5, 7
  Phase 4 : all together        coarse -> fine

For each run we record COST (sum of heights) and RUNTIME. A convergence plot
shows: as resolution gets finer, cost approaches the "true" optimum but time
grows. Where the cost stops changing = "fine enough".

Run:
  python step03_discretization.py            # full study (slow: minutes)
  python step03_discretization.py --quick    # fast smoke test (coarse values)
  python step03_discretization.py --open
"""

from __future__ import annotations

import argparse
import subprocess
import time
from pathlib import Path

import step02_ackermann as s2
from step01_base import make_spiral_map, RESULTS

# Baseline (the Step 2 default) — every sweep changes ONE knob away from this.
BASE = {
    "n_headings": 16,
    "step_len": 3.0,
    "n_steer": 5,
    "steer_max": 40.0,
}

# Fair-comparison "ruler": measure every finished path at this fixed spacing
# (in cells), no matter what driving step the search used. This makes the cost
# comparable across discretizations. The SEARCH still optimizes sum-of-heights;
# this is only the evaluation of the final path.
EVAL_SPACING = 1.0


def evaluate_path_cost(H, poses, spacing: float = EVAL_SPACING) -> float:
    """Sum of cell heights sampled at a FIXED spacing along the final path.

    Independent of the search's step length, so different discretizations are
    measured with the same ruler (approximates the integral of height over the
    path length).
    """
    import math

    n_rows, n_cols = H.shape
    pts = [(p[0], p[1]) for p in poses]

    def h_at(x: float, y: float) -> float:
        ci = min(max(int(round(x)), 0), n_cols - 1)
        ri = min(max(int(round(y)), 0), n_rows - 1)
        return float(H[ri, ci])

    if len(pts) < 2:
        return h_at(*pts[0])

    seg_len = [math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
               for i in range(1, len(pts))]
    total_len = sum(seg_len)

    total = 0.0
    n_samples = int(total_len / spacing)
    for s in range(n_samples + 1):
        target = s * spacing
        remaining = target
        seg_i = 0
        while seg_i < len(seg_len) and remaining > seg_len[seg_i]:
            remaining -= seg_len[seg_i]
            seg_i += 1
        if seg_i >= len(seg_len):
            x, y = pts[-1]
        else:
            t = remaining / seg_len[seg_i] if seg_len[seg_i] > 0 else 0.0
            x = pts[seg_i][0] + t * (pts[seg_i + 1][0] - pts[seg_i][0])
            y = pts[seg_i][1] + t * (pts[seg_i + 1][1] - pts[seg_i][1])
        total += h_at(x, y)
    return total


def run_once(H, *, n_headings, step_len, n_steer, steer_max) -> dict:
    """Set the Step 2 globals, solve once, and record cost + runtime."""
    s2.N_HEADINGS = n_headings
    s2.STEP_LEN = step_len
    s2.N_STEER = n_steer
    s2.STEER_MAX_DEG = steer_max

    t0 = time.perf_counter()
    try:
        poses, cost = s2.plan_ackermann(H, s2.START_POSE, s2.GOAL_CELL, goal_tol=s2.GOAL_TOL)
        elapsed = time.perf_counter() - t0
        eval_cost = evaluate_path_cost(H, poses)
        return {"ok": True, "cost": cost, "eval_cost": eval_cost,
                "time": elapsed, "moves": len(poses)}
    except RuntimeError:
        elapsed = time.perf_counter() - t0
        return {"ok": False, "cost": None, "eval_cost": None,
                "time": elapsed, "moves": None}


def run_sweep(H, knob: str, values: list, base: dict) -> list[dict]:
    """Vary one knob across `values`, keep the others at their base setting."""
    rows: list[dict] = []
    for v in values:
        params = dict(base)
        params[knob] = v
        res = run_once(H, **params)
        res["param"] = v
        rows.append(res)
        status = (f"fair={res['eval_cost']:.1f}  raw={res['cost']:.1f}  "
                  f"time={res['time']:.1f}s  moves={res['moves']}"
                  if res["ok"] else f"UNREACHABLE  (time={res['time']:.1f}s)")
        print(f"    {knob}={v!s:<5}  ->  {status}")
    return rows


def print_table(title: str, xlabel: str, rows: list[dict]) -> list[str]:
    """Console + report table for one phase."""
    lines = [f"\n{title}", "-" * 68,
             f"  {xlabel:<12}{'fair cost':>11}{'raw cost':>10}{'time (s)':>10}"
             f"{'moves':>7}{'status':>10}"]
    finest = next((r["eval_cost"] for r in reversed(rows) if r["ok"]), None)
    for r in rows:
        if r["ok"]:
            delta = "" if finest is None else f"  (Δ {r['eval_cost'] - finest:+.1f})"
            lines.append(f"  {r['param']!s:<12}{r['eval_cost']:>11.1f}{r['cost']:>10.1f}"
                         f"{r['time']:>10.1f}{r['moves']:>7}{'OK':>10}{delta}")
        else:
            lines.append(f"  {r['param']!s:<12}{'—':>11}{'—':>10}{r['time']:>10.1f}"
                         f"{'—':>7}{'UNREACH':>10}")
    text = "\n".join(lines)
    print(text)
    return lines


def plot_phases(phases: list[dict], out: Path) -> None:
    """One figure: per phase, cost (left axis) and time (right axis) vs resolution."""
    import matplotlib.pyplot as plt

    n = len(phases)
    fig, axes = plt.subplots(1, n, figsize=(6 * n, 5))
    if n == 1:
        axes = [axes]
    fig.patch.set_facecolor("white")

    for ax, ph in zip(axes, phases):
        ok = [r for r in ph["rows"] if r["ok"]]
        xs = [str(r["param"]) for r in ok]
        costs = [r["eval_cost"] for r in ok]
        times = [r["time"] for r in ok]

        ax.plot(xs, costs, "o-", color="#1565c0", lw=2, ms=8, label="cost")
        ax.set_xlabel(ph["xlabel"])
        ax.set_ylabel("fair cost (same 1-cell ruler)", color="#1565c0")
        ax.tick_params(axis="y", labelcolor="#1565c0")
        ax.set_title(ph["title"], fontsize=11)
        ax.grid(True, alpha=0.3)

        ax2 = ax.twinx()
        ax2.plot(xs, times, "s--", color="#e65100", lw=2, ms=7, label="time")
        ax2.set_ylabel("runtime (s)", color="#e65100")
        ax2.tick_params(axis="y", labelcolor="#e65100")

    fig.suptitle("Step 3 — discretization study  (blue = cost, orange = runtime)  "
                 "|  grid fixed 100x100", fontsize=13, y=1.02)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150, facecolor="white", bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Step 3: discretization comparison study.")
    parser.add_argument("--quick", action="store_true",
                        help="fewer/coarser values for a fast smoke test")
    parser.add_argument("--open", action="store_true", help="open the plot after saving")
    args = parser.parse_args()

    if args.quick:
        heading_vals = [8, 16]
        step_vals = [5, 3]
        steer_vals = [3, 5]
        combo = [("coarse", 8, 5.0, 3), ("fine", 16, 3.0, 5)]
    else:
        heading_vals = [8, 16, 24, 32]
        step_vals = [5.0, 3.0, 2.0]
        steer_vals = [3, 5, 7]
        combo = [("coarse", 8, 5.0, 3), ("medium", 16, 3.0, 5), ("fine", 32, 2.0, 7)]

    H = make_spiral_map()

    print("=" * 60)
    print("STEP 3 — DISCRETIZATION STUDY  (grid fixed 100x100)")
    print("=" * 60)
    print(f"  Baseline: headings={BASE['n_headings']}, step={BASE['step_len']}, "
          f"steering={BASE['n_steer']}, steer_max={BASE['steer_max']}")
    print("=" * 60)

    print("\n[Phase 1] Vary HEADING resolution (step + steering fixed)")
    p1 = run_sweep(H, "n_headings", heading_vals, BASE)

    print("\n[Phase 2] Vary STEP length (headings + steering fixed)")
    p2 = run_sweep(H, "step_len", step_vals, BASE)

    print("\n[Phase 3] Vary STEERING choices (headings + step fixed)")
    p3 = run_sweep(H, "n_steer", steer_vals, BASE)

    print("\n[Phase 4] Combined refinement (all knobs coarse -> fine)")
    p4: list[dict] = []
    for label, nh, sl, ns in combo:
        res = run_once(H, n_headings=nh, step_len=sl, n_steer=ns, steer_max=BASE["steer_max"])
        res["param"] = f"{label}\n({nh}h,{sl}s,{ns}st)"
        p4.append(res)
        status = (f"fair={res['eval_cost']:.1f}  raw={res['cost']:.1f}  time={res['time']:.1f}s"
                  if res["ok"] else f"UNREACHABLE (time={res['time']:.1f}s)")
        print(f"    {label:<7} headings={nh}, step={sl}, steering={ns}  ->  {status}")

    # Tables (console + report)
    report_lines = ["STEP 3 — DISCRETIZATION STUDY", "=" * 60,
                    "Grid fixed at 100x100. One knob varied per phase.",
                    f"Baseline: headings={BASE['n_headings']}, step={BASE['step_len']}, "
                    f"steering={BASE['n_steer']}.",
                    "",
                    "fair cost = final path measured at a FIXED 1-cell ruler (comparable",
                    "            across all settings).",
                    "raw cost  = what the search summed (depends on step length; shown for",
                    "            reference only, NOT directly comparable across step lengths)."]
    report_lines += print_table("PHASE 1 — heading resolution", "N_HEADINGS", p1)
    report_lines += print_table("PHASE 2 — step length", "STEP_LEN", p2)
    report_lines += print_table("PHASE 3 — steering choices", "N_STEER", p3)
    report_lines += print_table("PHASE 4 — combined refinement", "setting", p4)
    report_lines += ["", "=" * 60,
                     "READING IT: use the FAIR cost. It should drop then flatten as",
                     "resolution gets finer. Where it flattens = 'fine enough'. Runtime",
                     "grows with resolution; pick the coarsest setting whose fair cost is",
                     "close to the finest."]

    report = RESULTS / "step03_discretization.txt"
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text("\n".join(report_lines) + "\n", encoding="utf-8")

    phases = [
        {"title": "Phase 1 — headings", "xlabel": "N_HEADINGS", "rows": p1},
        {"title": "Phase 2 — step length", "xlabel": "STEP_LEN", "rows": p2},
        {"title": "Phase 3 — steering", "xlabel": "N_STEER", "rows": p3},
    ]
    png = RESULTS / "step03_discretization.png"
    plot_phases(phases, png)

    print("\n" + "=" * 60)
    print(f"  Report saved : {report}")
    print(f"  Plot saved   : {png}")
    print("=" * 60)

    if args.open:
        subprocess.Popen(["xdg-open", str(png)])


if __name__ == "__main__":
    main()

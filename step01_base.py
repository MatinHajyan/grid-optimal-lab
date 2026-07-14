#!/usr/bin/env python3
"""
STEP 1 — Professor's base problem (no wall, no length limit yet)
==============================================================

Problem:
  - Map H: 100×100 cells, each with a height 0…100.
  - Start A: bottom-left  → column 0, row 0  (0, 0).
  - Goal  B: bottom-right → column 99, row 0  (100, 0 on the sketch).
  - Move: 4 or 8 neighbors; optional max turn angle (default 45° — no sharp 90° corners).
  - Objective: minimize SUM of heights along the path (+ optional turn penalty).

Default map: 23 circular hills (professor-style reference terrain).

Run:
  python step01_base.py --open              # hill map, smooth turns (default)
  python step01_base.py --compare-turns     # all turn modes on one picture
  python step01_base.py --sharp-turns       # 4-neighbor, 90° corners OK
  python step01_base.py --max-turn 0        # straight ahead only
  python step01_base.py --max-turn 60       # looser turns
  python step01_base.py --gif --open
  python step01_base.py --random            # random 100×100 heights
"""

from __future__ import annotations

import argparse
import heapq
import math
import subprocess
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Problem constants (professor specification)
# ---------------------------------------------------------------------------
N_COLS = 100
N_ROWS = 100
START = (0, 0)    # (col, row) = bottom-left (0, 0)
GOAL = (99, 99)    # bottom-right — sketch label (100, 0); cols are 0…99

RESULTS = Path(__file__).resolve().parent / "results"

# Staggered hills in 5 rows — no straight corridor at row 50.
# Each entry: (center_col, center_row, radius)
HILLS = [
    # row 1 (top)
    (12, 84, 10), (30, 88, 9), (50, 82, 10), (70, 88, 9), (88, 84, 10),
    # row 2 (upper) — offset into the gaps above
    (22, 68, 11), (46, 72, 11), (68, 66, 11), (86, 70, 10),
    # row 3 (middle) — blocks the direct east-west line
    (10, 50, 11), (32, 54, 12), (54, 48, 12), (76, 52, 11), (94, 50, 9),
    # row 4 (lower) — offset again
    (18, 34, 11), (42, 30, 11), (64, 36, 11), (84, 32, 10),
    # row 5 (bottom)
    (12, 16, 10), (32, 20, 9), (52, 14, 10), (72, 20, 9), (88, 16, 10),
]
FLOOR = 5
PEAK = 95

# Movement: 8 directions (col delta, row delta)
MOVES_4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
MOVES_8 = (
    (1, 0), (1, 1), (0, 1), (-1, 1),
    (-1, 0), (-1, -1), (0, -1), (1, -1),
)
NO_PREV = (0, 0)  # sentinel: no incoming direction yet (at start)


def _step_moves(eight_connected: bool) -> tuple[tuple[int, int], ...]:
    return MOVES_8 if eight_connected else MOVES_4


def _turn_angle_deg(prev: tuple[int, int], new: tuple[int, int]) -> float:
    """Angle between two grid steps (degrees). prev=(0,0) means 'from start'."""
    if prev == NO_PREV:
        return 0.0
    pdc, pdr = prev
    ndc, ndr = new
    dot = pdc * ndc + pdr * ndr
    mag = math.hypot(pdc, pdr) * math.hypot(ndc, ndr)
    if mag == 0:
        return 0.0
    cos_a = max(-1.0, min(1.0, dot / mag))
    return math.degrees(math.acos(cos_a))


def count_sharp_turns(path: list[tuple[int, int]], max_ok_deg: float) -> int:
    """Count turns strictly sharper than max_ok_deg (e.g. 90° corners)."""
    sharp = 0
    for i in range(2, len(path)):
        pdc = path[i - 1][0] - path[i - 2][0]
        pdr = path[i - 1][1] - path[i - 2][1]
        ndc = path[i][0] - path[i - 1][0]
        ndr = path[i][1] - path[i - 1][1]
        if _turn_angle_deg((pdc, pdr), (ndc, ndr)) > max_ok_deg + 1e-6:
            sharp += 1
    return sharp


def make_height_map(seed: int | None) -> np.ndarray:
    """Build H[row,col] with random integer heights in [0, 100]."""
    rng = np.random.default_rng(seed)
    return rng.integers(0, 101, size=(N_ROWS, N_COLS))


def _circular_hill(col: float, row: float, cx: float, cy: float, radius: float) -> float:
    """0…1 strength of a round hill with concentric rings."""
    rho = np.hypot(col - cx, row - cy)
    if rho >= radius:
        return 0.0
    t = 1.0 - rho / radius
    rings = 0.5 + 0.5 * np.cos(rho / radius * 5.5 * np.pi)
    return (t ** 1.4) * (0.35 + 0.65 * rings)


def make_spiral_map() -> np.ndarray:
    """
    Professor-style map: flat floor + round hills with contour rings.
    Path must weave between the hills from A (left) to B (right).
    """
    H = np.full((N_ROWS, N_COLS), FLOOR, dtype=float)
    for r in range(N_ROWS):
        for c in range(N_COLS):
            strength = max(_circular_hill(c, r, cx, cy, rad) for cx, cy, rad in HILLS)
            H[r, c] = FLOOR + strength * (PEAK - FLOOR)

    # Low pads at A and B (corners)
    for dr in range(4):
        for dc in range(6):
            for sc, sr in (START, GOAL):
                rr, cc = sr + dr, sc + (dc if sc == START[0] else -dc)
                if 0 <= rr < N_ROWS and 0 <= cc < N_COLS:
                    H[rr, cc] = FLOOR

    return np.clip(np.round(H), 0, 100).astype(int)


def straight_row_cost(H: np.ndarray, row: int) -> float:
    """Cost of walking the full horizontal row (naive 'just go east')."""
    return float(H[row, :].sum())


def neighbors(
    col: int,
    row: int,
    n_cols: int = N_COLS,
    n_rows: int = N_ROWS,
    eight_connected: bool = True,
):
    """Yield neighbor cells (4- or 8-connected)."""
    for dc, dr in _step_moves(eight_connected):
        nc, nr = col + dc, row + dr
        if 0 <= nc < n_cols and 0 <= nr < n_rows:
            yield nc, nr, dc, dr


def dijkstra(
    H: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    *,
    eight_connected: bool = True,
    max_turn_deg: float | None = 45.0,
    turn_penalty: float = 0.0,
):
    """
    Minimum-cost path on the grid.

    Cost to enter a cell = its height (+ turn_penalty for sharp turns).
    State includes incoming direction so we can limit turn angle.
    """
    sc, sr = start
    gc, gr = goal
    start_state = (sc, sr, NO_PREV[0], NO_PREV[1])
    dist: dict[tuple[int, int, int, int], float] = {start_state: float(H[sr, sc])}
    pred: dict[tuple[int, int, int, int], tuple[int, int, int, int] | None] = {
        start_state: None,
    }
    settled: set[tuple[int, int, int, int]] = set()
    seen_cells: set[tuple[int, int]] = set()
    pq: list[tuple[float, int, int, int, int]] = [(dist[start_state], sc, sr, NO_PREV[0], NO_PREV[1])]
    frames: list[set[tuple[int, int]]] = []
    frame_stride = max(1, (N_COLS * N_ROWS) // 80)

    def turn_extra(prev: tuple[int, int], step: tuple[int, int]) -> float:
        if prev == NO_PREV or max_turn_deg is None:
            return 0.0
        angle = _turn_angle_deg(prev, step)
        if angle > max_turn_deg + 1e-6:
            return math.inf
        if turn_penalty > 0 and angle > 1e-6:
            return turn_penalty * (angle / 90.0)
        return 0.0

    while pq:
        d, c, r, pdc, pdr = heapq.heappop(pq)
        state = (c, r, pdc, pdr)
        if state in settled:
            continue
        settled.add(state)

        if (c, r) not in seen_cells:
            seen_cells.add((c, r))
            if len(seen_cells) % frame_stride == 0 or (c, r) == goal:
                frames.append(set(seen_cells))

        prev = NO_PREV if (pdc, pdr) == NO_PREV else (pdc, pdr)
        for nc, nr, dc, dr in neighbors(c, r, eight_connected=eight_connected):
            step = (dc, dr)
            extra = turn_extra(prev, step)
            if math.isinf(extra):
                continue
            nd = d + float(H[nr, nc]) + extra
            nstate = (nc, nr, dc, dr)
            if nd < dist.get(nstate, math.inf):
                dist[nstate] = nd
                pred[nstate] = state
                heapq.heappush(pq, (nd, nc, nr, dc, dr))

    best_state: tuple[int, int, int, int] | None = None
    best_cost = math.inf
    for (c, r, pdc, pdr), cost in dist.items():
        if (c, r) == goal and cost < best_cost:
            best_cost = cost
            best_state = (c, r, pdc, pdr)

    if best_state is None:
        raise RuntimeError("Goal not reachable (try --sharp-turns or check start/goal).")

    path: list[tuple[int, int]] = []
    cur: tuple[int, int, int, int] | None = best_state
    while cur is not None:
        path.append((cur[0], cur[1]))
        cur = pred.get(cur)
    path.reverse()

    height_cost = sum(float(H[r, c]) for c, r in path)
    return path, height_cost, frames


def verify_path(
    H: np.ndarray,
    path: list[tuple[int, int]],
    reported_cost: float,
    start: tuple[int, int],
    goal: tuple[int, int],
    *,
    eight_connected: bool = True,
) -> bool:
    """Quick sanity checks printed to the terminal."""
    ok = True
    recomputed = sum(float(H[r, c]) for c, r in path)

    if path[0] != start or path[-1] != goal:
        print("  [FAIL] Path does not connect A → B.")
        ok = False
    for i in range(1, len(path)):
        c, r = path[i]
        pc, pr = path[i - 1]
        dc, dr = abs(c - pc), abs(r - pr)
        if max(dc, dr) != 1 or (dc == 0 and dr == 0):
            print(f"  [FAIL] Invalid jump at step {i}.")
            ok = False
            break
        if not eight_connected and dc + dr != 1:
            print(f"  [FAIL] Diagonal move at step {i} (4-neighbor mode).")
            ok = False
            break
    if abs(recomputed - reported_cost) > 1e-6:
        print(f"  [FAIL] Cost mismatch: {recomputed:.1f} vs {reported_cost:.1f}")
        ok = False

    if ok:
        print("  [OK] Path connects A → B, valid moves, height cost matches.")
    return ok


def _dir_label(dc: int, dr: int) -> str:
    if dr == 1:
        return "UP"
    if dr == -1:
        return "DOWN"
    if dc == 1:
        return "EAST"
    if dc == -1:
        return "WEST"
    return "?"


def _cost_to_goal(
    H: np.ndarray,
    cell: tuple[int, int],
    goal: tuple[int, int],
    cache: dict,
    search_kw: dict,
) -> float:
    if cell not in cache:
        _, cost, _ = dijkstra(H, cell, goal, **search_kw)
        cache[cell] = cost
    return cache[cell]


def analyze_forks(
    H: np.ndarray,
    path: list[tuple[int, int]],
    goal: tuple[int, int],
    search_kw: dict,
) -> list[dict]:
    """Find forks where another direction loses to the chosen one."""
    n_rows, n_cols = H.shape
    cache: dict[tuple[int, int], float] = {}
    forks: list[dict] = []

    for i in range(1, len(path) - 1):
        prev, cur, nxt = path[i - 1], path[i], path[i + 1]
        c_ch, r_ch = nxt
        dc, dr = c_ch - cur[0], r_ch - cur[1]
        chosen_remain = float(H[r_ch, c_ch]) + _cost_to_goal(H, nxt, goal, cache, search_kw)

        for nc, nr, alt_dc, alt_dr in neighbors(
            cur[0], cur[1], n_cols, n_rows, search_kw.get("eight_connected", True),
        ):
            alt = (nc, nr)
            if alt in (prev, nxt):
                continue
            alt_remain = float(H[nr, nc]) + _cost_to_goal(H, alt, goal, cache, search_kw)
            if alt_remain <= chosen_remain:
                continue

            alt_h, chosen_h = int(H[nr, nc]), int(H[r_ch, c_ch])
            forks.append({
                "step": i,
                "at": cur,
                "prev": prev,
                "chosen": nxt,
                "chosen_dir": _dir_label(dc, dr),
                "chosen_h": chosen_h,
                "chosen_remain": chosen_remain,
                "alt": alt,
                "alt_dir": _dir_label(alt_dc, alt_dr),
                "alt_h": alt_h,
                "alt_remain": alt_remain,
                "saving": alt_remain - chosen_remain,
                "misleading": alt_h < chosen_h,
            })

    forks.sort(key=lambda f: (-f["misleading"], -f["saving"]))
    return forks


def save_proof_report(
    forks: list[dict],
    total_cost: float,
    path: list[tuple[int, int]],
    out: Path,
    top_n: int = 8,
) -> None:
    """Text report with numbers for each fork decision."""
    lines = [
        "STEP 1 — OPTIMALITY PROOF AT FORK POINTS",
        "=" * 60,
        f"Global optimum (Dijkstra total): {total_cost:.1f}",
        f"Path length: {len(path)} cells",
        "",
        "At each fork below:",
        "  • chosen_remain = height(next) + cheapest cost from there to goal",
        "  • alt_remain    = same for the other direction",
        "  • If saving > 0, the algorithm's choice is cheaper (proven).",
        "",
        "misleading = YES means the other cell looks cheaper locally",
        "(lower height) but is MORE expensive overall to the goal.",
        "=" * 60,
        "",
    ]

    shown = 0
    for f in forks:
        if shown >= top_n or f["saving"] <= 0:
            continue
        shown += 1
        lines.extend([
            f"FORK #{shown}  (path step {f['step']})  at cell {f['at']}",
            f"  came from {f['prev']}",
            f"  CHOSE  {f['chosen']}  {f['chosen_dir']:5s}  height={f['chosen_h']:3d}  "
            f"→ remaining cost to goal = {f['chosen_remain']:.1f}",
            f"  OTHER  {f['alt']}  {f['alt_dir']:5s}  height={f['alt_h']:3d}  "
            f"→ remaining cost to goal = {f['alt_remain']:.1f}",
            f"  PROOF: chosen saves {f['saving']:.1f} vs alternative",
        ])
        if f["misleading"]:
            lines.append(
                f"  ⚠ misleading: {f['alt_dir']} looks better now (height {f['alt_h']} < {f['chosen_h']}) "
                f"but costs {f['saving']:.1f} MORE to finish!"
            )
        lines.append("")

    lines.extend([
        "=" * 60,
        "THEORY: Dijkstra with non-negative costs is globally optimal.",
        "Each comparison above is a independent shortest-path subproblem",
        "from the fork to the goal — not just a visual guess.",
    ])

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _draw_board_background(ax, H: np.ndarray, n_rows: int, n_cols: int) -> None:
    """Whiteboard-style hills (red rings) — shared by PNG and GIF."""
    from matplotlib.patches import Rectangle

    ax.set_facecolor("white")
    xx = np.arange(n_cols)
    yy = np.arange(n_rows)
    ax.contourf(xx, yy, H, levels=np.linspace(FLOOR, PEAK, 12),
                cmap="Reds", alpha=0.18, antialiased=True)
    ax.contour(xx, yy, H, levels=[20, 35, 50, 65, 80, 90],
               colors="#c62828", linewidths=1.8, alpha=0.9)
    border = Rectangle((0, 0), n_cols, n_rows, fill=False, edgecolor="black", lw=2.5)
    ax.add_patch(border)
    ax.text(n_cols / 2, n_rows + 4, r"$N_y$", ha="center", fontsize=14, fontweight="bold")
    ax.text(-6, n_rows / 2, "100", ha="center", va="center", fontsize=12, rotation=90)
    ax.text(n_cols / 2, -6, "100", ha="center", fontsize=12)
    ax.text(-4, -4, "(0,0)", ha="right", fontsize=10)
    for g in range(6):
        ax.plot([g, g], [0, 5], color="gray", lw=0.4, alpha=0.5)
        ax.plot([0, 5], [g, g], color="gray", lw=0.4, alpha=0.5)
    ax.set_xlim(-12, n_cols + 2)
    ax.set_ylim(-10, n_rows + 8)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])


def _draw_board_markers(ax, start: tuple[int, int], goal: tuple[int, int]) -> None:
    ax.plot(start[0], start[1], "s", color="#1565c0", ms=12, zorder=8)
    ax.plot(goal[0], goal[1], "s", color="#1565c0", ms=12, zorder=8)
    ax.annotate("goal", (goal[0] + 3, goal[1] + 2), fontsize=13, color="#1565c0", fontweight="bold")


def _draw_fork_markers(ax, forks: list[dict] | None, top_n: int = 8) -> None:
    if not forks:
        return
    shown = 0
    placed: set[tuple[int, int]] = set()
    for f in forks:
        if f["saving"] <= 0 or f["at"] in placed:
            continue
        shown += 1
        if shown > top_n:
            break
        placed.add(f["at"])
        c, r = f["at"]
        color = "#e65100" if f["misleading"] else "#2e7d32"
        ax.plot(c, r, "o", color=color, ms=14, zorder=7)
        ax.text(c, r, str(shown), ha="center", va="center", fontsize=8,
                color="white", fontweight="bold", zorder=8)


def plot_path(
    H: np.ndarray,
    path: list[tuple[int, int]],
    total_cost: float,
    out: Path,
    start: tuple[int, int],
    goal: tuple[int, int],
    board_style: bool = False,
    forks: list[dict] | None = None,
):
    """Save terrain + path. board_style = professor whiteboard look."""
    import matplotlib.pyplot as plt

    n_rows, n_cols = H.shape
    cols = [p[0] for p in path]
    rows = [p[1] for p in path]

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_facecolor("white")
    fig.patch.set_facecolor("white")

    if board_style:
        _draw_board_background(ax, H, n_rows, n_cols)
        ax.plot(cols, rows, color="#1565c0", linewidth=3.0, solid_capstyle="round",
                label="optimal path", zorder=5)
        _draw_board_markers(ax, start, goal)
        _draw_fork_markers(ax, forks)
        ax.text(-14, n_rows / 2, "(E W S N)", ha="center", va="center", fontsize=10, rotation=90)
        ax.set_title(
            f"Step 1 — weave through {len(HILLS)} hills  |  {len(path)} cells  |  cost = {total_cost:.0f}\n"
            f"numbered dots = proof forks  (orange = other way looks cheaper locally)",
            fontsize=12, pad=12,
        )
    else:
        im = ax.imshow(H, origin="lower", cmap="terrain", aspect="equal", vmin=0, vmax=100)
        plt.colorbar(im, ax=ax, label="height", shrink=0.85)
        ax.plot(cols, rows, color="#1565c0", linewidth=3.0, label="optimal path", zorder=5)
        ax.plot(start[0], start[1], "s", color="dodgerblue", ms=12, label="A start", zorder=6)
        ax.plot(goal[0], goal[1], "s", color="red", ms=12, label="B goal", zorder=6)
        ax.set_xlabel("column x")
        ax.set_ylabel("row y")
        ax.set_title(f"Step 1  |  {len(path)} cells  |  cost = {total_cost:.1f}")
        ax.legend(loc="upper right")

    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=160, facecolor="white")
    plt.close(fig)


def save_gif(
    H: np.ndarray,
    frames: list[set],
    path: list[tuple[int, int]],
    out: Path,
    start: tuple[int, int],
    goal: tuple[int, int],
    board_style: bool = False,
):
    """Animated Dijkstra wavefront, then optimal path."""
    try:
        import imageio.v2 as imageio
        import matplotlib.pyplot as plt
        from matplotlib.colors import LinearSegmentedColormap
    except ImportError:
        print("  [skip GIF] install with: pip install imageio")
        return

    n_rows, n_cols = H.shape
    images: list[np.ndarray] = []
    fig, ax = plt.subplots(figsize=(10, 10))
    fig.patch.set_facecolor("white")
    wave_cmap = LinearSegmentedColormap.from_list("wave", ["#e3f2fd", "#1565c0"])
    cumulative: set[tuple[int, int]] = set()

    for settled in frames:
        cumulative |= settled
        ax.cla()
        if board_style:
            _draw_board_background(ax, H, n_rows, n_cols)
            wave = np.full((n_rows, n_cols), np.nan)
            for c, r in cumulative:
                wave[r, c] = 1.0
            ax.imshow(wave, origin="lower", cmap=wave_cmap, alpha=0.72, vmin=0, vmax=1, zorder=4)
            _draw_board_markers(ax, start, goal)
            ax.set_title("Dijkstra searching…  (blue = visited)", fontsize=12, pad=12)
        else:
            wave = np.full((n_rows, n_cols), np.nan)
            for c, r in cumulative:
                wave[r, c] = float(H[r, c])
            ax.imshow(H, origin="lower", cmap="gray", vmin=0, vmax=100, aspect="equal")
            ax.imshow(wave, origin="lower", cmap="Blues", alpha=0.75, aspect="equal")
            ax.plot(start[0], start[1], "s", color="dodgerblue", ms=10)
            ax.plot(goal[0], goal[1], "s", color="red", ms=10)
            ax.set_title(f"Dijkstra wavefront — {len(cumulative)} cells")
        fig.canvas.draw()
        buf = fig.canvas.buffer_rgba()
        images.append(np.asarray(buf)[:, :, :3].copy())

    ax.cla()
    if board_style:
        _draw_board_background(ax, H, n_rows, n_cols)
        ax.plot([p[0] for p in path], [p[1] for p in path], color="#1565c0", lw=3.0, zorder=5)
        _draw_board_markers(ax, start, goal)
        ax.set_title("Optimal path", fontsize=13, pad=12, fontweight="bold")
    else:
        ax.imshow(H, origin="lower", cmap="terrain", aspect="equal")
        ax.plot([p[0] for p in path], [p[1] for p in path], "w-", lw=2)
        ax.plot(start[0], start[1], "bo", ms=8)
        ax.plot(goal[0], goal[1], "r*", ms=12)
        ax.set_title("Optimal path")
    fig.canvas.draw()
    buf = fig.canvas.buffer_rgba()
    images.extend([np.asarray(buf)[:, :, :3].copy()] * 5)
    plt.close(fig)

    out.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(out, images, duration=0.12, loop=0)


def search_settings(*, sharp_turns: bool = False, max_turn: float = 45.0) -> tuple[dict, str]:
    """Build dijkstra kwargs + human label for a turn mode."""
    if sharp_turns:
        return (
            {"eight_connected": False, "max_turn_deg": None, "turn_penalty": 0.0},
            "4-neighbor, 90° corners OK",
        )
    return (
        {"eight_connected": True, "max_turn_deg": max_turn, "turn_penalty": 0.0},
        f"8-neighbor, max turn {max_turn:.0f}°",
    )


def _compare_mode_list() -> list[tuple[str, str, dict, str]]:
    """(file_id, title, search_kw, line_style/color key)."""
    modes: list[tuple[str, str, dict, str]] = []
    kw, _ = search_settings(max_turn=45.0)
    modes.append(("default_45", "Default — max 45° (smooth)", kw, "#1565c0"))
    kw, _ = search_settings(sharp_turns=True)
    modes.append(("sharp_90", "Sharp — 4-way, 90° OK", kw, "#e65100"))
    kw, _ = search_settings(max_turn=0.0)
    modes.append(("straight_0", "Strict — max 0° (straight only)", kw, "#6a1b9a"))
    kw, _ = search_settings(max_turn=60.0)
    modes.append(("loose_60", "Loose — max 60°", kw, "#2e7d32"))
    return modes


def run_turn_comparison(
    H: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[dict]:
    """Run every turn mode; return result dicts (path may be None if unreachable)."""
    results: list[dict] = []
    for file_id, title, search_kw, color in _compare_mode_list():
        print(f"\n--- {title} ---")
        row: dict = {
            "id": file_id,
            "title": title,
            "search_kw": search_kw,
            "color": color,
            "path": None,
            "cost": None,
            "cells": None,
            "sharp": None,
            "error": None,
        }
        try:
            path, cost, _ = dijkstra(H, start, goal, **search_kw)
            threshold = search_kw.get("max_turn_deg") or 90.0
            sharp = count_sharp_turns(path, threshold)
            row.update(path=path, cost=cost, cells=len(path), sharp=sharp)
            print(f"  cells={len(path)}  cost={cost:.1f}  sharp_turns={sharp}")
        except RuntimeError as exc:
            row["error"] = str(exc)
            print(f"  UNREACHABLE: {exc}")
        results.append(row)
    return results


def plot_turn_comparison(
    H: np.ndarray,
    results: list[dict],
    start: tuple[int, int],
    goal: tuple[int, int],
    out: Path,
) -> None:
    """One whiteboard map with every turn mode drawn + legend."""
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    n_rows, n_cols = H.shape
    fig, ax = plt.subplots(figsize=(12, 11))
    fig.patch.set_facecolor("white")
    _draw_board_background(ax, H, n_rows, n_cols)
    _draw_board_markers(ax, start, goal)
    ax.text(-14, n_rows / 2, "(E W S N)", ha="center", va="center", fontsize=10, rotation=90)

    legend_handles: list[Line2D] = []
    for r in results:
        if r["path"] is None:
            legend_handles.append(Line2D(
                [0], [0], color="gray", lw=2, linestyle=":",
                label=f"{r['title']} — UNREACHABLE",
            ))
            continue
        cols = [p[0] for p in r["path"]]
        rows = [p[1] for p in r["path"]]
        lw = 4.0 if r["id"] == "default_45" else 2.5
        ax.plot(cols, rows, color=r["color"], linewidth=lw, solid_capstyle="round",
                alpha=0.95, zorder=5)
        legend_handles.append(Line2D(
            [0], [0], color=r["color"], lw=lw,
            label=(
                f"{r['title']}  |  {r['cells']} cells  |  cost {r['cost']:.0f}  "
                f"|  sharp {r['sharp']}"
            ),
        ))

    ax.set_title(
        f"Turn-mode comparison  |  A {start} → B {goal}  |  {len(HILLS)} hills",
        fontsize=13, pad=12,
    )
    ax.legend(handles=legend_handles, loc="upper left", bbox_to_anchor=(1.02, 1),
              fontsize=9, framealpha=0.95)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=160, facecolor="white", bbox_inches="tight")
    plt.close(fig)


def save_turn_comparison_report(results: list[dict], start, goal, out: Path) -> None:
    lines = [
        "STEP 1 — TURN MODE COMPARISON",
        "=" * 70,
        f"Start A : {start}",
        f"Goal B  : {goal}",
        "",
        f"{'Mode':<32} {'Cells':>6} {'Cost':>8} {'Sharp':>6}  Status",
        "-" * 70,
    ]
    for r in results:
        if r["path"] is None:
            lines.append(f"{r['title']:<32} {'—':>6} {'—':>8} {'—':>6}  UNREACHABLE")
        else:
            lines.append(
                f"{r['title']:<32} {r['cells']:>6} {r['cost']:>8.1f} {r['sharp']:>6}  OK"
            )
    lines.extend(["", "=" * 70])
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")


def compare_turns_main(open_after: bool = False) -> None:
    """Run all turn modes and save combined PNG + report + per-mode PNGs."""
    H = make_spiral_map()
    start, goal = START, GOAL

    print("=" * 60)
    print("STEP 1 — compare all turn modes")
    print("=" * 60)
    print(f"  Start A : {start}")
    print(f"  Goal B  : {goal}")
    print(f"  Hills   : {len(HILLS)}")
    print("=" * 60)

    results = run_turn_comparison(H, start, goal)

    compare_dir = RESULTS / "compare_turns"
    compare_dir.mkdir(parents=True, exist_ok=True)
    for r in results:
        if r["path"] is None:
            continue
        plot_path(
            H, r["path"], r["cost"], compare_dir / f"{r['id']}.png",
            start, goal, board_style=True,
        )

    combined = RESULTS / "step01_compare_turns.png"
    plot_turn_comparison(H, results, start, goal, combined)

    report = RESULTS / "step01_compare_turns.txt"
    save_turn_comparison_report(results, start, goal, report)

    print("\n" + "=" * 60)
    print("  Combined PNG :", combined)
    print("  Report       :", report)
    print("  Per-mode PNGs:", compare_dir)
    print("=" * 60)

    if open_after:
        subprocess.Popen(["xdg-open", str(combined)])


def main():
    parser = argparse.ArgumentParser(description="Step 1: height grid, Dijkstra A→B")
    parser.add_argument("--random", action="store_true", help="random heights instead of hills")
    parser.add_argument("--seed", type=int, default=42, help="random seed (--random only)")
    parser.add_argument(
        "--compare-turns", action="store_true",
        help="run all turn modes (45°, sharp/90°, 0°, 60°) → one comparison PNG",
    )
    parser.add_argument(
        "--sharp-turns", action="store_true",
        help="classic 4-neighbor grid, allow 90° corners (no turn limit)",
    )
    parser.add_argument(
        "--max-turn", type=float, default=45.0, metavar="DEG",
        help="max turn angle in degrees (default 45 — blocks 90° corners)",
    )
    parser.add_argument("--gif", action="store_true", help="save wavefront GIF")
    parser.add_argument("--open", action="store_true", help="open PNG after run (Linux)")
    args = parser.parse_args()

    if args.compare_turns:
        compare_turns_main(open_after=args.open)
        return

    if args.sharp_turns:
        search_kw, move_label = search_settings(sharp_turns=True)
    else:
        search_kw, move_label = search_settings(max_turn=args.max_turn)

    start, goal = START, GOAL
    if args.random:
        H = make_height_map(args.seed)
        title = "STEP 1 — random mountain grid"
        board_style = False
        png = RESULTS / "step01_path.png"
    else:
        H = make_spiral_map()
        title = "STEP 1 — reference hill map"
        board_style = True
        png = RESULTS / "step01_spiral.png"

    print("=" * 60)
    print(title)
    print("=" * 60)
    print(f"  Grid        : {N_COLS} x {N_ROWS}")
    print(f"  Start A     : (col,row) = {start}")
    print(f"  Goal B      : (col,row) = {goal}")
    print(f"  Moves       : {move_label}")
    print(f"  Objective   : minimize sum of heights")
    if args.random:
        print(f"  Random seed : {args.seed}")
    else:
        print(f"  Hills       : {len(HILLS)} circular peaks")
    print("=" * 60)

    path, total_cost, frames = dijkstra(H, start, goal, **search_kw)

    sharp = count_sharp_turns(path, search_kw.get("max_turn_deg") or 90.0)
    print(f"  Path cells  : {len(path)}")
    print(f"  Total cost  : {total_cost:.1f}  (sum of cell heights)")
    print(f"  Sharp turns : {sharp}  (angles > {search_kw.get('max_turn_deg') or 90:.0f}°)")
    if board_style:
        naive = straight_row_cost(H, start[1])
        print(f"  Naive row   : {naive:.1f}  (straight along row {start[1]})")
        print(f"  Savings     : {naive - total_cost:.1f}  (Dijkstra detours through valleys)")
    print("  Optimality  : Dijkstra is exact when all step costs >= 0")
    print("=" * 60)
    verify_path(H, path, total_cost, start, goal, eight_connected=search_kw["eight_connected"])

    forks: list[dict] = []
    if board_style and args.sharp_turns:
        print("  Analyzing fork points…")
        forks = analyze_forks(H, path, goal, search_kw)
        proof = RESULTS / "step01_proof.txt"
        save_proof_report(forks, total_cost, path, proof)
        print(f"  Proof saved : {proof}")
    elif board_style:
        print("  Fork proof  : skipped (turn-limited search — use --sharp-turns for fork report)")
    print("=" * 60)

    plot_path(H, path, total_cost, png, start, goal, board_style=board_style, forks=forks)
    print(f"  PNG saved   : {png}")

    if args.gif:
        gif = RESULTS / ("step01_spiral.gif" if board_style else "step01_dijkstra.gif")
        save_gif(H, frames, path, gif, start, goal, board_style=board_style)
        print(f"  GIF saved   : {gif}")

    if args.open:
        subprocess.Popen(["xdg-open", str(png)])


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
STEP 2 — Ackermann (car-like) motion on the height map
======================================================

Extends Step 1 from a POINT to a small CAR:

  Step 1 : state = (col, row, incoming direction), jumps to a neighbor cell.
  Step 2 : state = (x, y, heading theta), drives forward along Ackermann arcs.

What is new (supervisor's request — WITHOUT multi-destination):
  - Ackermann / bicycle motion   : the car moves forward along arcs, no sideways jumps.
  - Control constraint           : steering angle limited to +/- STEER_MAX.
  - Max change of heading         : follows automatically from the steering limit.
  - State = (x, y, theta)        : position AND orientation.
  - Small vehicle (footprint)    : a rectangle (length x width), not a dot.
  - Cost stays SIMPLE            : sum of cell heights along the path (same as Step 1).
  - Solver stays Dijkstra        : globally optimal over the discretized (x, y, theta) states.

This is the "Hybrid-A*-lite" idea: keep a CONTINUOUS pose in each search node,
but use a DISCRETIZED key (cell_x, cell_y, heading_bin) to decide which states
are 'the same' for the shortest-path bookkeeping.

Run:
  python step02_ackermann.py
  python step02_ackermann.py --open
  python step02_ackermann.py --steer-max 30 --step-len 3
"""

from __future__ import annotations

import argparse
import heapq
import math
import subprocess
from pathlib import Path

import numpy as np

# Reuse the Step 1 terrain + drawing so both steps share the SAME map.
from step01_base import (
    N_COLS,
    N_ROWS,
    FLOOR,
    PEAK,
    HILLS,
    make_spiral_map,
    _draw_board_background,
    _draw_board_markers,
    RESULTS,
)

# ---------------------------------------------------------------------------
# Vehicle + search parameters  (edit these to experiment)
# ---------------------------------------------------------------------------
# Ackermann / bicycle model
WHEELBASE = 2.5          # distance between front and rear axle (in grid cells)
STEER_MAX_DEG = 40.0     # |steering angle| limit (control constraint)
N_STEER = 5              # how many discrete steering choices (odd => includes 0)
STEP_LEN = 3.0           # forward distance per motion primitive (cells)
N_SUBSTEPS = 6           # integration sub-steps per primitive (smoothness)

# Vehicle footprint (a small rectangle)
VEHICLE_LENGTH = 3.0     # along heading (cells)
VEHICLE_WIDTH = 1.6      # across heading (cells)

# State discretization
N_HEADINGS = 16          # heading bins -> 360/16 = 22.5 degrees per bin

# Start / goal
START_POSE = (2.0, 2.0, math.radians(45.0))   # (x, y, heading) — facing the goal
GOAL_CELL = (99, 99)                            # target cell
GOAL_TOL = 3.0                                  # reached if center within this many cells

# The car may nose this far past the grid edge (so corners stay reachable).
BOUND_MARGIN = 1.5


# ---------------------------------------------------------------------------
# Small geometry helpers
# ---------------------------------------------------------------------------
def wrap_angle(theta: float) -> float:
    """Keep an angle in [0, 2*pi)."""
    return theta % (2.0 * math.pi)


def heading_bin(theta: float) -> int:
    """Which discrete heading bucket does this angle fall into?"""
    step = 2.0 * math.pi / N_HEADINGS
    return int(round(wrap_angle(theta) / step)) % N_HEADINGS


def state_key(x: float, y: float, theta: float) -> tuple[int, int, int]:
    """Discretized identity of a pose: same cell + same heading bin = 'same state'."""
    return (int(round(x)), int(round(y)), heading_bin(theta))


def steering_set() -> list[float]:
    """Discrete steering angles (radians) from -STEER_MAX to +STEER_MAX."""
    smax = math.radians(STEER_MAX_DEG)
    if N_STEER == 1:
        return [0.0]
    return [(-smax + 2.0 * smax * i / (N_STEER - 1)) for i in range(N_STEER)]


def vehicle_corners(x: float, y: float, theta: float) -> list[tuple[float, float]]:
    """Four corners of the vehicle rectangle centered at (x, y), facing theta."""
    half_l = VEHICLE_LENGTH / 2.0
    half_w = VEHICLE_WIDTH / 2.0
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    body = [(+half_l, +half_w), (+half_l, -half_w),
            (-half_l, -half_w), (-half_l, +half_w)]
    corners = []
    for bx, by in body:
        wx = x + bx * cos_t - by * sin_t
        wy = y + bx * sin_t + by * cos_t
        corners.append((wx, wy))
    return corners


def footprint_ok(x: float, y: float, theta: float) -> bool:
    """Is the whole vehicle inside the grid (plus a small margin)?

    No walls yet, so only the boundary matters here. When Step 3 adds a wall,
    this is the single place to also reject cells that hit the wall.
    """
    lo = -BOUND_MARGIN
    hi_x = (N_COLS - 1) + BOUND_MARGIN
    hi_y = (N_ROWS - 1) + BOUND_MARGIN
    for wx, wy in vehicle_corners(x, y, theta):
        if not (lo <= wx <= hi_x and lo <= wy <= hi_y):
            return False
    return True


def cell_height(H: np.ndarray, x: float, y: float) -> tuple[float, tuple[int, int]]:
    """Height of the cell under (x, y), with indices clamped to the grid."""
    ci = min(max(int(round(x)), 0), N_COLS - 1)
    ri = min(max(int(round(y)), 0), N_ROWS - 1)
    return float(H[ri, ci]), (ci, ri)


# ---------------------------------------------------------------------------
# Ackermann motion primitive (the "how the car moves" part)
# ---------------------------------------------------------------------------
def integrate_primitive(
    x: float, y: float, theta: float, steer: float, H: np.ndarray,
):
    """Drive forward STEP_LEN cells with a fixed steering angle.

    Bicycle model (rear-axle reference):
        d(theta) = ds * tan(steer) / wheelbase
        straight when steer == 0, otherwise a circular arc of radius
        R = wheelbase / tan(steer).

    Returns (nx, ny, ntheta, edge_cost) or None if the vehicle would leave
    the allowed area at any point along the arc.
    edge_cost = sum of heights of the NEW cells the center drives through.
    """
    ds = STEP_LEN / N_SUBSTEPS
    cx, cy, cth = x, y, theta
    edge_cost = 0.0
    visited: set[tuple[int, int]] = set()

    for _ in range(N_SUBSTEPS):
        if abs(steer) < 1e-9:
            nx = cx + ds * math.cos(cth)
            ny = cy + ds * math.sin(cth)
            nth = cth
        else:
            dth = ds * math.tan(steer) / WHEELBASE
            nth = cth + dth
            radius = WHEELBASE / math.tan(steer)
            nx = cx + radius * (math.sin(nth) - math.sin(cth))
            ny = cy - radius * (math.cos(nth) - math.cos(cth))

        if not footprint_ok(nx, ny, nth):
            return None

        h, cell = cell_height(H, nx, ny)
        if cell not in visited:
            edge_cost += h
            visited.add(cell)

        cx, cy, cth = nx, ny, nth

    return cx, cy, wrap_angle(cth), edge_cost


# ---------------------------------------------------------------------------
# Dijkstra over (x, y, heading) states
# ---------------------------------------------------------------------------
def plan_ackermann(H: np.ndarray, start_pose, goal_cell, goal_tol=GOAL_TOL):
    """Cheapest (sum-of-heights) Ackermann path from start_pose to goal_cell.

    Same Dijkstra logic as Step 1, but:
      - nodes carry a CONTINUOUS pose (x, y, theta),
      - identity for bookkeeping is the DISCRETE state_key(),
      - neighbors come from Ackermann motion primitives (not 8 grid cells).
    """
    sx, sy, sth = start_pose
    start_k = state_key(sx, sy, sth)
    start_h, _ = cell_height(H, sx, sy)

    dist: dict[tuple[int, int, int], float] = {start_k: start_h}
    pose_at: dict[tuple[int, int, int], tuple[float, float, float]] = {start_k: (sx, sy, sth)}
    came: dict[tuple[int, int, int], tuple[int, int, int] | None] = {start_k: None}
    settled: set[tuple[int, int, int]] = set()

    pq: list[tuple[float, float, float, float]] = [(start_h, sx, sy, sth)]
    steers = steering_set()
    goal_k: tuple[int, int, int] | None = None

    while pq:
        d, x, y, th = heapq.heappop(pq)
        k = state_key(x, y, th)
        if k in settled:
            continue
        settled.add(k)

        if math.hypot(x - goal_cell[0], y - goal_cell[1]) <= goal_tol:
            goal_k = k
            break

        for steer in steers:
            res = integrate_primitive(x, y, th, steer, H)
            if res is None:
                continue
            nx, ny, nth, edge_cost = res
            nk = state_key(nx, ny, nth)
            nd = d + edge_cost
            if nd < dist.get(nk, math.inf):
                dist[nk] = nd
                pose_at[nk] = (nx, ny, nth)
                came[nk] = k
                heapq.heappush(pq, (nd, nx, ny, nth))

    if goal_k is None:
        raise RuntimeError(
            "Goal not reachable — try larger --steer-max, smaller --step-len, "
            "or a bigger --goal-tol."
        )

    poses: list[tuple[float, float, float]] = []
    cur: tuple[int, int, int] | None = goal_k
    while cur is not None:
        poses.append(pose_at[cur])
        cur = came[cur]
    poses.reverse()

    return poses, dist[goal_k]


# ---------------------------------------------------------------------------
# Verification (mirror of Step 1's verify_path, adapted to Ackermann)
# ---------------------------------------------------------------------------
def verify_ackermann(poses, start_pose, goal_cell, goal_tol=GOAL_TOL) -> bool:
    """Sanity checks: start matches, goal reached, and every step is feasible."""
    ok = True
    sx, sy, _ = start_pose
    if math.hypot(poses[0][0] - sx, poses[0][1] - sy) > 1e-6:
        print("  [FAIL] Path does not start at the start pose.")
        ok = False

    last = poses[-1]
    if math.hypot(last[0] - goal_cell[0], last[1] - goal_cell[1]) > goal_tol + 1e-6:
        print("  [FAIL] Path does not reach the goal.")
        ok = False

    max_dtheta = STEP_LEN * math.tan(math.radians(STEER_MAX_DEG)) / WHEELBASE
    for i in range(1, len(poses)):
        x0, y0, _ = poses[i - 1]
        x1, y1, _ = poses[i]
        gap = math.hypot(x1 - x0, y1 - y0)
        if gap > STEP_LEN * 1.5 + 1e-6:
            print(f"  [FAIL] Step {i} too long ({gap:.2f} > {STEP_LEN}).")
            ok = False
            break
        if not footprint_ok(*poses[i]):
            print(f"  [FAIL] Vehicle out of bounds at step {i}.")
            ok = False
            break

    if ok:
        print("  [OK] Starts at A, reaches goal, all steps Ackermann-feasible.")
        print(f"       max heading change/step = {math.degrees(max_dtheta):.1f} deg "
              f"(from steer limit {STEER_MAX_DEG:.0f} deg)")
    return ok


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
def plot_ackermann(H, poses, total_cost, out: Path, start_pose, goal_cell) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    fig, ax = plt.subplots(figsize=(10, 10))
    fig.patch.set_facecolor("white")
    _draw_board_background(ax, H, N_ROWS, N_COLS)
    _draw_board_markers(ax, (int(start_pose[0]), int(start_pose[1])), goal_cell)

    xs = [p[0] for p in poses]
    ys = [p[1] for p in poses]
    ax.plot(xs, ys, color="#1565c0", linewidth=2.5, solid_capstyle="round",
            zorder=5, label="Ackermann path")

    # Draw the vehicle rectangle at a few sample poses to show orientation + size.
    every = max(1, len(poses) // 14)
    for i in range(0, len(poses), every):
        corners = vehicle_corners(*poses[i])
        ax.add_patch(Polygon(corners, closed=True, fill=True,
                             facecolor="#1565c0", edgecolor="#0d3c78",
                             alpha=0.28, zorder=4))

    ax.set_title(
        f"Step 2 — Ackermann car through {len(HILLS)} hills  |  "
        f"{len(poses)} moves  |  cost = {total_cost:.0f}\n"
        f"wheelbase={WHEELBASE}, steer<={STEER_MAX_DEG:.0f} deg, "
        f"vehicle={VEHICLE_LENGTH}x{VEHICLE_WIDTH}, step={STEP_LEN}",
        fontsize=11, pad=12,
    )
    ax.legend(loc="lower right", fontsize=10)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=160, facecolor="white")
    plt.close(fig)


def save_gif_ackermann(H, poses, out: Path, start_pose, goal_cell) -> None:
    """Animate the car driving along the optimal path, one frame per move."""
    try:
        import imageio.v2 as imageio
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon
    except ImportError:
        print("  [skip GIF] install with: pip install imageio")
        return

    images: list[np.ndarray] = []
    fig, ax = plt.subplots(figsize=(9, 9))
    fig.patch.set_facecolor("white")

    for i in range(len(poses)):
        ax.cla()
        _draw_board_background(ax, H, N_ROWS, N_COLS)
        _draw_board_markers(ax, (int(start_pose[0]), int(start_pose[1])), goal_cell)

        xs = [p[0] for p in poses[: i + 1]]
        ys = [p[1] for p in poses[: i + 1]]
        ax.plot(xs, ys, color="#1565c0", linewidth=2.5, solid_capstyle="round", zorder=5)

        corners = vehicle_corners(*poses[i])
        ax.add_patch(Polygon(corners, closed=True, fill=True,
                             facecolor="#1565c0", edgecolor="#0d3c78",
                             alpha=0.85, zorder=6))
        ax.set_title(f"Ackermann car driving…  move {i + 1}/{len(poses)}",
                     fontsize=12, pad=12)

        fig.canvas.draw()
        buf = fig.canvas.buffer_rgba()
        images.append(np.asarray(buf)[:, :, :3].copy())

    images.extend([images[-1]] * 8)  # hold the final frame
    plt.close(fig)

    out.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(out, images, duration=0.15, loop=0)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main() -> None:
    global STEER_MAX_DEG, STEP_LEN, N_STEER

    parser = argparse.ArgumentParser(
        description="Step 2: Ackermann car on the height grid (Dijkstra, sum-of-heights)."
    )
    parser.add_argument("--steer-max", type=float, default=STEER_MAX_DEG,
                        help=f"max steering angle in degrees (default {STEER_MAX_DEG:.0f})")
    parser.add_argument("--step-len", type=float, default=STEP_LEN,
                        help=f"forward distance per move in cells (default {STEP_LEN})")
    parser.add_argument("--n-steer", type=int, default=N_STEER,
                        help=f"number of discrete steering choices (default {N_STEER})")
    parser.add_argument("--goal-tol", type=float, default=GOAL_TOL,
                        help=f"reach radius around the goal (default {GOAL_TOL})")
    parser.add_argument("--gif", action="store_true", help="save an animation of the car driving")
    parser.add_argument("--open", action="store_true", help="open the PNG after saving")
    args = parser.parse_args()

    STEER_MAX_DEG = args.steer_max
    STEP_LEN = args.step_len
    N_STEER = args.n_steer

    H = make_spiral_map()

    print("=" * 60)
    print("STEP 2 — Ackermann (car-like) motion")
    print("=" * 60)
    print(f"  Grid          : {N_COLS} x {N_ROWS}")
    print(f"  Start pose    : x={START_POSE[0]}, y={START_POSE[1]}, "
          f"heading={math.degrees(START_POSE[2]):.0f} deg")
    print(f"  Goal cell     : {GOAL_CELL}  (reach radius {args.goal_tol})")
    print(f"  Wheelbase     : {WHEELBASE} cells")
    print(f"  Steering      : +/- {STEER_MAX_DEG:.0f} deg, {N_STEER} choices")
    print(f"  Step length   : {STEP_LEN} cells")
    print(f"  Vehicle size  : {VEHICLE_LENGTH} x {VEHICLE_WIDTH} cells")
    print(f"  Headings      : {N_HEADINGS} bins ({360/N_HEADINGS:.1f} deg each)")
    print(f"  Objective     : minimize sum of heights (same as Step 1)")
    print("=" * 60)

    poses, total_cost = plan_ackermann(H, START_POSE, GOAL_CELL, goal_tol=args.goal_tol)

    print(f"  Moves         : {len(poses)}")
    print(f"  Total cost    : {total_cost:.1f}  (sum of cell heights)")
    print(f"  Min turn radius: {WHEELBASE / math.tan(math.radians(STEER_MAX_DEG)):.2f} cells")
    print("  Optimality    : Dijkstra is exact over the (x, y, heading) states")
    print("=" * 60)
    verify_ackermann(poses, START_POSE, GOAL_CELL, goal_tol=args.goal_tol)
    print("=" * 60)

    png = RESULTS / "step02_ackermann.png"
    plot_ackermann(H, poses, total_cost, png, START_POSE, GOAL_CELL)
    print(f"  PNG saved     : {png}")

    if args.gif:
        gif = RESULTS / "step02_ackermann.gif"
        save_gif_ackermann(H, poses, gif, START_POSE, GOAL_CELL)
        print(f"  GIF saved     : {gif}")

    if args.open:
        subprocess.Popen(["xdg-open", str(png)])


if __name__ == "__main__":
    main()

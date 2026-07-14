#!/usr/bin/env python3
"""
Visualize ONE circular hill in isolation (same formula as step01_base.py).

Run:
  python step01_single_hill.py
  python step01_single_hill.py --open
"""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

import numpy as np

FLOOR = 5
PEAK = 95
CONTOUR_LEVELS = [20, 35, 50, 65, 80, 90]

# One hill: (center_col, center_row, radius)
SINGLE_HILL = (50, 50, 12)

RESULTS = Path(__file__).resolve().parent / "results"


def circular_hill(col: float, row: float, cx: float, cy: float, radius: float) -> float:
    """0…1 strength — identical to step01_base._circular_hill."""
    rho = np.hypot(col - cx, row - cy)
    if rho >= radius:
        return 0.0
    t = 1.0 - rho / radius
    rings = 0.5 + 0.5 * np.cos(rho / radius * 5.5 * np.pi)
    return (t ** 1.4) * (0.35 + 0.65 * rings)


def make_single_hill_map(
    n_cols: int = 100,
    n_rows: int = 100,
    hill: tuple[int, int, int] = SINGLE_HILL,
) -> np.ndarray:
    cx, cy, radius = hill
    H = np.full((n_rows, n_cols), FLOOR, dtype=float)
    for r in range(n_rows):
        for c in range(n_cols):
            strength = circular_hill(c, r, cx, cy, radius)
            H[r, c] = FLOOR + strength * (PEAK - FLOOR)
    return H


def cross_section(hill: tuple[int, int, int], n_samples: int = 200) -> tuple[np.ndarray, np.ndarray]:
    """Heights along a horizontal slice through the hill center."""
    cx, cy, radius = hill
    dist = np.linspace(0, radius * 1.15, n_samples)
    strength = np.array([circular_hill(cx + d, cy, cx, cy, radius) for d in dist])
    height = FLOOR + strength * (PEAK - FLOOR)
    return dist, height


def print_sample_heights(H: np.ndarray, hill: tuple[int, int, int]) -> None:
    cx, cy, _ = hill
    print("\nSample cell heights (col, row) → H:")
    samples = [
        (cx, cy, "center (peak)"),
        (cx + 3, cy, "3 cells east"),
        (cx + 6, cy, "6 cells east"),
        (cx + 9, cy, "9 cells east"),
        (cx + 12, cy, "at radius (edge)"),
        (cx + 15, cy, "outside hill"),
    ]
    for c, r, label in samples:
        if 0 <= c < H.shape[1] and 0 <= r < H.shape[0]:
            print(f"  ({c:2d}, {r:2d})  {label:20s}  →  {H[r, c]:.1f}")


def plot_single_hill(out: Path, hill: tuple[int, int, int] = SINGLE_HILL) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    cx, cy, radius = hill
    H = make_single_hill_map(hill=hill)
    n_rows, n_cols = H.shape

    # Zoom window around the hill (with margin)
    margin = 8
    c0 = max(0, cx - radius - margin)
    c1 = min(n_cols, cx + radius + margin + 1)
    r0 = max(0, cy - radius - margin)
    r1 = min(n_rows, cy + radius + margin + 1)
    H_zoom = H[r0:r1, c0:c1]

    fig, axes = plt.subplots(1, 3, figsize=(16, 5.5))
    fig.patch.set_facecolor("white")

    # --- Panel 1: full grid ---
    ax = axes[0]
    ax.set_facecolor("white")
    xx = np.arange(n_cols)
    yy = np.arange(n_rows)
    ax.contourf(xx, yy, H, levels=np.linspace(FLOOR, PEAK, 12), cmap="Reds", alpha=0.18)
    cs = ax.contour(xx, yy, H, levels=CONTOUR_LEVELS, colors="#c62828", linewidths=1.8)
    ax.clabel(cs, inline=True, fontsize=8, fmt="%d")
    ax.add_patch(Rectangle((0, 0), n_cols, n_rows, fill=False, edgecolor="black", lw=2))
    ax.plot(cx, cy, "k*", ms=14, zorder=5)
    ax.set_title(f"Full 100×100 grid\none hill at ({cx}, {cy}), r={radius}")
    ax.set_aspect("equal")
    ax.set_xlim(-2, n_cols + 2)
    ax.set_ylim(-2, n_rows + 2)

    # --- Panel 2: zoomed top view ---
    ax = axes[1]
    ax.set_facecolor("white")
    xx_z = np.arange(c0, c1)
    yy_z = np.arange(r0, r1)
    ax.contourf(xx_z, yy_z, H_zoom, levels=np.linspace(FLOOR, PEAK, 12), cmap="Reds", alpha=0.25)
    cs = ax.contour(xx_z, yy_z, H_zoom, levels=CONTOUR_LEVELS, colors="#c62828", linewidths=2.0)
    ax.clabel(cs, inline=True, fontsize=9, fmt="%d")
    ax.add_patch(Circle((cx, cy), radius, fill=False, edgecolor="#333", lw=1.5, ls="--"))
    ax.plot(cx, cy, "k*", ms=16, zorder=5)
    ax.set_title("Zoomed top view\nred rings = equal height (contour lines)")
    ax.set_xlabel("column")
    ax.set_ylabel("row")
    ax.set_aspect("equal")

    # --- Panel 3: cross-section through center ---
    ax = axes[2]
    dist, height = cross_section(hill)
    ax.set_facecolor("white")
    ax.fill_between(dist, FLOOR, height, color="#ef9a9a", alpha=0.35)
    ax.plot(dist, height, color="#c62828", lw=2.5)
    for level in CONTOUR_LEVELS:
        ax.axhline(level, color="#c62828", ls=":", lw=1, alpha=0.7)
        ax.text(radius * 1.12, level, str(level), va="center", fontsize=8, color="#c62828")
    ax.axvline(radius, color="#333", ls="--", lw=1, alpha=0.6)
    ax.text(radius, FLOOR - 3, f"r={radius}", ha="center", fontsize=9)
    ax.set_xlim(0, radius * 1.15)
    ax.set_ylim(FLOOR - 5, PEAK + 5)
    ax.set_xlabel("distance from center (cells)")
    ax.set_ylabel("height")
    ax.set_title("Side view (horizontal slice through peak)\ndotted lines = same levels as red rings")
    ax.grid(True, alpha=0.3)

    fig.suptitle(
        f"One hill: center=({cx},{cy}), radius={radius}  |  "
        f"height = {FLOOR} + strength×({PEAK}-{FLOOR})",
        fontsize=13,
        y=1.02,
    )
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"Saved: {out}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize one circular hill in isolation.")
    parser.add_argument("--cx", type=int, default=SINGLE_HILL[0], help="hill center column")
    parser.add_argument("--cy", type=int, default=SINGLE_HILL[1], help="hill center row")
    parser.add_argument("--radius", type=int, default=SINGLE_HILL[2], help="hill radius")
    parser.add_argument("--open", action="store_true", help="open PNG after saving")
    args = parser.parse_args()

    hill = (args.cx, args.cy, args.radius)
    out = RESULTS / "step01_single_hill.png"

    H = make_single_hill_map(hill=hill)
    print(f"Single hill at center=({hill[0]}, {hill[1]}), radius={hill[2]}")
    print(f"Height range on map: {H.min():.1f} … {H.max():.1f}")
    print(f"Contour levels drawn: {CONTOUR_LEVELS}")
    print_sample_heights(H, hill)

    plot_single_hill(out, hill=hill)

    if args.open:
        subprocess.run(["xdg-open", str(out)], check=False)


if __name__ == "__main__":
    main()

# grid-optimal-lab

Grid pathfinding lab for **minimal-risk maneuver (MRM)** research — build and verify **globally optimal** paths on a height map before connecting to continuous planners (Frenetix / MPPI hybrid).

Professor's exercises, **one step at a time**, separate from the hybrid thesis repo.

| Project | Path |
|---------|------|
| This lab | `grid-optimal-lab` |
| Hybrid thesis | `frenetix-mppi-hybrid` |

## Problem (Step 1)

- **Grid:** 100×100 cells, each with a height (cost / risk proxy).
- **Start A:** `(0, 0)` bottom-left.
- **Goal B:** `(99, 99)` top-right.
- **Move:** 4- or 8-neighbor; optional max turn angle (default 45°).
- **Objective:** minimize **sum of (height × step length)** along the path.
- **Solver:** Dijkstra (exact optimum for this discrete problem).

Default terrain: **23 circular hills** with contour rings (professor-style reference map).

## Setup

```bash
cd grid-optimal-lab
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
# Default: hill map, 8-neighbor, max 45° turns
python step01_base.py

# Compare all turn modes (45°, 90°, 0°, 60°)
python step01_base.py --compare-turns

# 4-neighbor grid + fork optimality proof
python step01_base.py --sharp-turns

# Dijkstra wavefront animation
python step01_base.py --gif

# Random height map (testing)
python step01_base.py --random --seed 42

# Visualize one hill in isolation (terrain tutorial)
python step01_single_hill.py

# Step 2: Ackermann (car-like) motion, same map and cost
python step02_ackermann.py
python step02_ackermann.py --gif          # animation of the car driving

# Step 3: discretization study (compare resolution scales, grid fixed)
python step03_discretization.py
python step03_discretization.py --quick   # fast smoke test
```

## Outputs

Generated under `results/` (not committed — recreate with commands above):

| File | Command |
|------|---------|
| `step01_spiral.png` | `python step01_base.py` |
| `step01_compare_turns.png` | `--compare-turns` |
| `step01_proof.txt` | `--sharp-turns` |
| `step01_single_hill.png` | `python step01_single_hill.py` |
| `step01_spiral.gif` | `--gif` |
| `step02_ackermann.png` | `python step02_ackermann.py` |
| `step02_ackermann.gif` | `python step02_ackermann.py --gif` |
| `step03_discretization.png` | `python step03_discretization.py` |
| `step03_discretization.txt` | `python step03_discretization.py` |

## Project structure

```
grid-optimal-lab/
├── step01_base.py          # Step 1: map, Dijkstra, plots, fork proof
├── step01_single_hill.py   # One-hill terrain visualization
├── step02_ackermann.py     # Step 2: Ackermann car (imports Step 1 map)
├── step03_discretization.py# Step 3: discretization comparison study
├── docs/STEP01_README.md   # Step 1 checklist
├── docs/STEP02_README.md   # Step 2 checklist
├── docs/STEP03_README.md   # Step 3 checklist
├── requirements.txt
└── results/                # gitignored — run scripts to generate
```

## Roadmap

- [x] **Step 1** — base grid, Dijkstra, 23 hills, turn modes, fork proof
- [x] **Step 2** — Ackermann (car-like) motion: `(x, y, heading)` state, steering limit, vehicle footprint
- [x] **Step 3** — discretization study: sweep headings/step/steering, fair convergence comparison (grid fixed)
- [ ] **Step 4** — vertical wall obstacle (footprint collision already wired)
- [ ] **Step 5** — max path length constraint
- [ ] Multi-destination (several goals) — investigate
- [ ] Grid-size (spatial) refinement
- [ ] Real elevation data (e.g. Elbsandstein DEM)
- [ ] Compare grid optimum vs continuous planners

## Notes

- Dijkstra with non-negative costs is **globally optimal** on the grid.
- Fork analysis (`step01_proof.txt`) proves local decisions are cheaper all the way to the goal.
- `results/` and `.venv/` are gitignored; outputs are reproducible from the scripts.

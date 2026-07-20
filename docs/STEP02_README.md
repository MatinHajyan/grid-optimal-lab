# Step 2 — read this before running the car code

## What changed from Step 1

Step 1 moved a **point** across the hill map. Step 2 moves a **small car**.

| | Step 1 (`step01_base.py`) | Step 2 (`step02_ackermann.py`) |
|---|---------------------------|--------------------------------|
| Agent | a point (dot) | a small rectangle (car) |
| State | `(col, row, incoming direction)` | `(x, y, heading θ)` |
| Move | jump to a neighbor cell | drive a steering-limited arc |
| Turn rule | max turn angle between steps | max **steering** angle (Ackermann) |
| Cost | sum of cell heights | **sum of cell heights (same!)** |
| Solver | Dijkstra | Dijkstra (over poses) |

The **map** and the **cost** are identical — only the mover changed from a dot to a car.

## The car (Ackermann / bicycle model)

The car steers its front wheels and rolls forward — it **cannot slide sideways or spin in place**. Three formulas describe it:

```
heading change:  Δθ = (ds · tan δ) / L      (δ = steering, L = wheelbase, ds = step)
turning radius:  R  = L / tan δ
tightest turn:   R_min = L / tan(δ_max)
```

The steering limit `δ_max` is the **control constraint**. It automatically caps how fast the heading can change (the "max change of heading").

## What you are solving

From the start pose (position **and** heading) reach the goal cell with the **smallest sum of cell heights**, using only car-feasible arcs, with the whole rectangle staying inside the grid.

## Files

| File | Role |
|------|------|
| `step02_ackermann.py` | Builds the car, runs Dijkstra over poses, saves plot |
| `results/step02_ackermann.png` | Output picture (path + car footprints) |
| `results/step02_ackermann.gif` | Optional animation of the car driving |

It imports the map and drawing style from `step01_base.py`, so both steps share the same terrain.

## Commands

```bash
cd ~/Matin/MRM/grid-optimal-lab
source .venv/bin/activate

python step02_ackermann.py                 # solve + save PNG
python step02_ackermann.py --gif           # also save the driving animation
python step02_ackermann.py --open          # open the PNG afterwards
python step02_ackermann.py --steer-max 30  # tighter steering limit
python step02_ackermann.py --step-len 2    # shorter forward step per move
```

## Key parameters (top of `step02_ackermann.py`)

```python
WHEELBASE = 2.5          # front-to-rear axle distance (cells)
STEER_MAX_DEG = 40.0     # steering limit (control constraint)
STEP_LEN = 3.0           # forward distance per move (cells)
VEHICLE_LENGTH = 3.0     # car size
VEHICLE_WIDTH = 1.6
N_HEADINGS = 16          # heading resolution (22.5 deg each)
```

## How the search works (one paragraph)

A "state" is a car pose `(x, y, heading)`. Each pose is kept in **two versions**: the
**exact** decimals (used to drive the arcs accurately) and a **rounded bucket key**
`(cell_x, cell_y, heading_bin)` (used to decide "have I already explored this
situation?"). Dijkstra pops the cheapest pose, tries all steering choices (each drives
one arc via the bicycle model), and keeps the cheapest way to each bucket. This is the
"Hybrid-A*-lite" idea — smooth motion from exact poses, finite search from rounded keys.

## Checklist (before Step 3)

- [ ] I ran the script and got a "Total cost" number
- [ ] I opened the PNG and see a smooth curved path with car rectangles along it
- [ ] I can explain: state = (x, y, heading), move = steering-limited arc, cost = sum of heights
- [ ] I understand why the search is bigger than Step 1 (16x more states from the heading)

## Not yet done (leave for later)

- Wall obstacle (footprint collision is already wired in `footprint_ok`)
- Max path length
- Multi-destination (several goals) — investigate later

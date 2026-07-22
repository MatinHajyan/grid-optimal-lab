# Step 3 — read this before running the discretization study

## What this step answers

Supervisor's note: *"solve the discrete problem and compare different
discretization scales together."*

The real world is continuous; the computer must chop it into finite pieces
(**discretization**). This step measures: **how fine must we chop before the
answer stops changing?** That tells us the resolution is "good enough" and that
our reference solution can be trusted.

The **grid stays fixed at 100x100**. We vary the CAR's discretization knobs.

## The knobs (discretization scales)

| Knob | Parameter | Meaning | Values swept |
|------|-----------|---------|--------------|
| Heading resolution | `N_HEADINGS` | how many facing directions | 8, 16, 24, 32 |
| Step length | `STEP_LEN` | how far the car drives per move | 5, 3, 2 |
| Steering choices | `N_STEER` | how many turn options per move | 3, 5, 7 |

We vary **one knob at a time** (holding the others at the baseline
16 / 3 / 5) so the effect of each is isolated. Phase 4 refines all together.

## The "fair cost" (important)

The search cost is the **sum of cell heights** along the path (kept simple, as
the supervisor asked). But that raw number depends on **how many cells you
sample**, which depends on the step length — so raw costs are NOT directly
comparable across step lengths (a coarse step counts fewer cells and looks
artificially cheaper).

Fix: after solving, we measure every finished path with the **same ruler** —
sampling the height every `1.0` cell regardless of the search's step. This
**fair cost** is what the tables and plots compare.

```
raw cost  = what the search summed        (step-dependent, reference only)
fair cost = final path measured at a fixed 1-cell ruler (comparable)  <-- use this
```

## Files

| File | Role |
|------|------|
| `step03_discretization.py` | Runs the sweeps, records cost + runtime |
| `results/step03_discretization.txt` | Table of every run |
| `results/step03_discretization.png` | Convergence plots (cost + runtime vs resolution) |

It imports the map from `step01_base.py` and the planner from `step02_ackermann.py`.

## Commands

```bash
cd ~/Matin/MRM/grid-optimal-lab
source .venv/bin/activate

python step03_discretization.py            # full study (several minutes)
python step03_discretization.py --quick    # fast smoke test (coarse values)
python step03_discretization.py --open     # open the plot afterwards
```

## How to read the results

- **Fair cost vs resolution** should **drop then flatten**. Where it flattens =
  "fine enough" — finer settings barely help.
- **Runtime vs resolution** grows (finer = slower).
- Pick the **coarsest** setting whose fair cost is already close to the finest:
  that is the best speed/accuracy trade-off.

```
 fair cost                          runtime
   |*                                  |          *
   | *                                 |       *
   |   *___*___*  (converged)          |   *
   +---------------- finer -->         +-------------- finer -->
```

## What this gives the supervisor

- A defensible statement like: *"beyond N headings the optimum changes by less
  than X, so N is sufficient."*
- Evidence that the discrete reference solution is **converged** (trustworthy).
- Awareness that the naive sum-of-heights cost is sampling-sensitive, handled by
  the fixed-ruler evaluation.

## Not done yet

- Grid-size (spatial) refinement — intentionally left fixed at 100x100 for now.
- Wall obstacle and max path length.
- Multi-destination.

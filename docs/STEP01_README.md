# Step 1 — read this before running code

## What you are solving

Imagine a 100×100 map of hills. Each cell has a height 0…100.

- **Start A** = left side, middle: (x=0, y=50)
- **Goal B**  = right side, middle: (x=99, y=50)

You walk cell to cell (up/down/left/right). You pay the **height** of each cell you enter.

**Question:** which path from A to B has the **smallest total height**?

That total is your **J_global** for this toy problem.

## Files

| File | Role |
|------|------|
| `step01_base.py` | Builds H, runs Dijkstra, saves plot |
| `results/step01_path.png` | Output picture |

## Commands

```bash
cd ~/Matin/MRM/grid-optimal-lab
source .venv/bin/activate
python step01_base.py
```

Optional GIF (install imageio first: `pip install imageio`):

```bash
python step01_base.py --gif --open
```

## What `if __name__ == "__main__":` does

At the bottom of `step01_base.py`:

```python
if __name__ == "__main__":
    main()
```

- When you run `python step01_base.py` → Python sets `__name__` to `"__main__"` → `main()` runs.
- When another file `import step01_base` → `main()` does **not** run automatically.

## Checklist (do not go to Step 2 until yes)

- [ ] I ran the script and got a number for "Total cost"
- [ ] I opened the PNG and see white path from blue dot to red star
- [ ] I can explain: cell = grid square, cost = height, Dijkstra = best path on grid

## Next step (later)

Step 2: add a vertical **wall** at x=50 — block those cells, run Dijkstra again.

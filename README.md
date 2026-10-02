# Intelligent Maze Solver — A* Search

An interactive maze-solving application that visualises the **A\* heuristic search algorithm** in real time using a Tkinter GUI.

---

## Features

| Feature | Details |
|---|---|
| 🧱 Interactive drawing | Place walls, start, and goal with your mouse |
| ▶ Animated search | Watch A\* explore cells step by step |
| ⏸ Pause / Resume | Stop and inspect the search at any point |
| ⚡ Speed slider | Control animation speed from slow to near-instant |
| 📊 Statistics panel | Nodes explored, path length, execution time |
| 📐 Grid resizer | Adjust grid dimensions (rows × columns) on the fly |

---

## Project Structure

```
AstarMaze/
├── main.py     # Tkinter GUI — drawing, animation, controls
├── astar.py    # A* search generator
├── grid.py     # 2-D grid, cell states, neighbor enumeration
└── README.md
```

---

## Requirements

- **Python 3.10+** (uses `tuple[int,int] | None` type hints)
- **Tkinter** — bundled with most standard Python distributions on Windows and macOS

No third-party packages are required.

---

## Running the Application

```bash
python main.py
```

---

## How to Use

### 1 — Draw your maze

| Mode | Action |
|---|---|
| 🧱 Draw Wall | Click and drag to place walls |
| 🟢 Set Start | Click any empty cell |
| 🔴 Set Goal | Click any empty cell |
| 🗑️ Erase | Click and drag to remove walls |

### 2 — Run

Click **▶ Run Search**. Watch the colours update live:

| Colour | Meaning |
|---|---|
| 🔵 Blue | Frontier cells — discovered but not yet expanded (open set) |
| 🟦 Dark blue | Explored cells — already expanded (closed set) |
| 🟡 Gold | Final shortest path found by A\* |
| 🟢 Green | Start cell |
| 🔴 Red | Goal cell |

Use **⏸ Pause** at any moment to freeze the animation and inspect the current state, then **▶ Resume** to continue.

---

## How A\* Works

### Core Idea

A\* is an **informed search algorithm** — unlike blind search strategies, it uses a heuristic estimate of the remaining distance to the goal to decide which cell to explore next. This lets it focus its search toward the goal and avoid wasting time on obviously bad directions.

### The Evaluation Function

At every step, A\* scores each candidate cell `n` with:

```
f(n) = g(n) + h(n)
```

| Term | Meaning |
|---|---|
| `g(n)` | Exact cost of the cheapest path found so far from **Start → n** |
| `h(n)` | Heuristic estimate of the remaining cost from **n → Goal** |
| `f(n)` | Estimated total cost of a path through `n` |

The algorithm always expands the cell with the **lowest f(n)** first.

### Heuristic: Manhattan Distance

Because movement is restricted to 4 directions (up, down, left, right) with unit cost, this project uses:

```
h(n) = |row_n - row_goal| + |col_n - col_goal|
```

This heuristic is **admissible** — it never over-estimates the true remaining distance — which guarantees that A\* finds the **optimal (shortest) path**.

### Step-by-Step Execution

1. **Initialise** — Place the start cell in the *open set* with `g = 0`, `f = h(start)`.
2. **Loop** — While the open set is not empty:
   - Pop the cell `current` with the lowest `f(n)`.
   - If `current` is the **goal** → reconstruct and return the path.
   - Move `current` to the *closed set* (mark dark blue).
   - For each **neighbour** of `current` (not a wall, not in closed set):
     - Compute `tentative_g = g(current) + 1`.
     - If this is cheaper than any previously known path to neighbour:
       - Record `came_from[neighbour] = current`.
       - Update `g(neighbour)` and push to the open set (mark blue).
3. **No path** — If the open set empties without reaching the goal, no path exists.

### Path Reconstruction

Once the goal is reached, A\* traces back through the `came_from` map from goal → start, then reverses the list to produce the **start → goal path** (highlighted gold).

### Why A\* is Efficient

- Cells with a low heuristic (close to the goal) get explored first.
- Cells already expanded are never re-expanded (closed set check).
- The priority queue (min-heap) ensures the best candidate is always processed in O(log n) time.

---

## Test Cases

| Scenario | Expected result |
|---|---|
| No obstacles | Finds the direct shortest path |
| Obstacles present | Finds the shortest detour |
| Narrow corridor | Navigates through the passage |
| Goal surrounded by walls | Reports "No path exists" |
| Start equals goal | Finds path of length 0 |

---

## AI Syllabus Connections

| Concept | Implementation |
|---|---|
| Intelligent agent | The solver navigates the grid autonomously |
| State-space search | Each traversable cell is a state |
| Heuristic search | A\* uses Manhattan distance to estimate remaining cost |
| Evaluation function | `f(n) = g(n) + h(n)` computed at every expansion |
| Admissibility | Heuristic never over-estimates → optimal path guaranteed |
| Priority queue | Min-heap ensures lowest-f cell is always expanded next |

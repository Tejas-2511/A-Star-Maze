# Intelligent Maze Solver — A* Search & Manhattan Calculation Lab

An interactive maze-solving application and educational calculation laboratory that visualises the **A\* heuristic search algorithm** in real time using a sleek modern Tkinter GUI.

This project focuses exclusively on the **Manhattan Distance heuristic ($L_1$ norm)** for $f(n) = g(n) + h(n)$ calculation, presenting the exact mathematical formulas, substitutions, neighbor evaluations, and priority queue operations live on screen.

---

## 🌟 Key Features

| Feature | Details |
|---|---|
| 📐 **Manhattan Heuristic Formula in GUI** | Dedicated cards and headers showing $f(n) = g(n) + h(n)$ and $h(n) = \|r_n - r_{goal}\| + \|c_n - c_{goal}\|$ |
| 🧮 **Live Calculation Lab** | Displays exact step-by-step arithmetic and numerical substitutions for each active expanding node |
| 🧭 **4-Direction Neighbor Evaluation** | Evaluates North, South, East, West with tentative $g$, Manhattan $h$, $f$, and heap decisions |
| 📋 **Priority Queue Inspector** | Real-time Open Set min-heap leaderboard showing why the lowest-$f$ (with lower-$h$ tie-breaker) node is expanded next |
| 🔢 **Grid Cell Value Overlays** | Switch between viewing $f$-score, $g$-cost (distance from start), $h$-cost, or full $f, g, h$ on cells |
| 🔍 **Live Hover Cell Inspector** | Hover over any cell to inspect $(r, c)$, status, $g$, $h$, $f$, parent node, and live Manhattan calculation |
| ⏭ **Step-by-Step Execution** | Step forward one iteration at a time to examine calculations at your own pace |
| ⌨️ **Ergonomic Shortcuts** | `[Space]` Run/Pause/Resume, `[S]` or `[→]` Step Forward, `[R]` Reset Search, `[C]` Clear All |
| 🎲 **1-Click Maze Presets** | Instant test patterns: *Simple Maze*, *Corridors*, *Spiral Labyrinth*, *Random (28%)*, and *Clear Walls* |
| 🧱 **Interactive Drawing** | Click and drag to draw walls, set start (🟢) and goal (🔴), or erase |
| ⚡ **Speed Control & Resizer** | Smooth animation speed slider with live millisecond display (1 ms to 200 ms) and dynamic grid resizer |

---

## 📁 Project Structure

```
A-Star-Maze/
├── main.py     # Interactive Tkinter GUI & Live Manhattan Calculation Lab
├── astar.py    # A* Search generator with Manhattan heuristic & priority queue engine
├── grid.py     # 2-D grid representation, cell states, neighbor enumeration & presets
└── README.md   # Documentation, mathematical formulas, and algorithm reference
```

---

## 💻 Requirements

- **Python 3.10+** (uses modern type hints)
- **Tkinter** (bundled with standard Python on Windows, macOS, and Linux)

*No third-party packages or pip installs required.*

---

## 🚀 Running the Application

```bash
python main.py
```

---

## 🎮 How to Use

### 1 — Draw Your Maze or Load a Preset

* **Draw Tools**:
  * 🧱 **Wall**: Click and drag on empty cells to build walls.
  * 🟢 **Start**: Click any cell to place the starting position.
  * 🔴 **Goal**: Click any cell to place the target goal.
  * 🗑️ **Erase**: Click and drag to remove walls.
* **Presets**: Use **Simple Maze**, **Corridors**, **Spiral Lab**, or **Random 28%** to instantly generate complex mazes.

### 2 — Manhattan Heuristic Display & Cell Numbers

* **Manhattan Heuristic**:
  * Defined as $h(n) = |r_n - r_{goal}| + |c_n - c_{goal}| = |\Delta \text{row}| + |\Delta \text{col}|$.
  * Admissible ($h(n) \le h^*(n)$) and consistent for 4-directional orthogonal movement.
  * Prominently displayed in the Left Panel and Right Calculation Lab.
* **Cell Values on Grid**:
  * `f-score (total: g + h)`: Shows total estimated cost.
  * `g-cost (from start)`: Shows step count propagating outward from Start.
  * `h-cost (heuristic: |Δr| + |Δc|)`: Shows remaining distance counting down to $0$ at Goal.
  * `f, g, h breakdown`: Shows $f$ in the center with $g$ and $h$ in the corners.
  * `Hide numbers (clean)`: Minimalist view.

### 3 — Run, Step, and Inspect

* **▶ Run Search** (or `[Space]`): Starts animated continuous search.
* **⏭ Step** (or `[S]` / `[→]`): Advances exactly one node expansion to inspect arithmetic substitutions and neighbor decisions.
* **⏸ Pause / Resume** (or `[Space]`): Freeze search at any point to inspect candidate nodes.
* **↺ Reset Search** (or `[R]`): Keeps walls and start/goal intact while clearing paths and calculations.
* **⬛ Clear All** (or `[C]`): Resets the entire grid and walls.

---

## 🎨 Color Legend

| Indicator | Color | Meaning |
|---|---|---|
| 🟢 Green | `#10b981` | Start Node |
| 🔴 Red | `#ef4444` | Goal Node |
| 🟣 Pink / Magenta | `#ec4899` | Active Expanding Node (currently being evaluated) |
| 🟡 Gold | `#f59e0b` | Final optimal shortest path found by A\* |
| 🔵 Royal Blue | `#1d4ed8` | Frontier / Open Set (discovered candidate cells) |
| 🟦 Slate Blue | `#1e293b` | Explored / Closed Set (already expanded cells) |
| 🧱 Slate | `#2a3348` | Obstacle Wall |
| ⬛ Dark Navy | `#151c2c` | Unvisited Empty Cell |

---

## 🧮 How A\* Works & The Manhattan Mathematics

### 1. The Evaluation Function

At every step, A\* scores candidate cells with:

$$f(n) = g(n) + h(n)$$

where $h(n)$ is the **Manhattan Distance**:

$$h(n) = |r_n - r_{goal}| + |c_n - c_{goal}| = |\Delta \text{row}| + |\Delta \text{col}|$$

| Component | Definition | Meaning |
|---|---|---|
| $g(n)$ | Exact Path Cost | Number of unit steps taken along the cheapest path from **Start → $n$** |
| $h(n)$ | Manhattan Heuristic | Exact minimum grid steps from **$n$ → Goal** with no walls |
| $f(n)$ | Total Estimated Cost | Estimated total cost of an entire path passing through node $n$ |

The algorithm always extracts and expands the cell with the **lowest $f(n)$** from the priority queue (breaking ties in favor of lower $h(n)$ to move directly towards the goal).

---

### 2. Why Direct Paths Share the Same $f$-Score

If Start is at $(2, 2)$ and Goal is at $(15, 23)$, the initial Manhattan distance is:

$$h(\text{Start}) = |15 - 2| + |23 - 2| = 13 + 21 = \mathbf{34}$$

Whenever A\* steps closer to the goal (Down $\downarrow$ or Right $\rightarrow$):
1. $g(n)$ increases by $1$ ($g \to g + 1$).
2. $h(n)$ decreases by $1$ ($h \to h - 1$).
3. The sum is **conserved**:

$$f(n) = (g + 1) + (h - 1) = g + h = \mathbf{34}$$

| Position | $g(n)$ | $h(n)$ | $f(n) = g + h$ |
|---|:---:|:---:|:---:|
| **Start (S)** | $0$ | $34$ | **$34$** |
| Step 5 | $5$ | $29$ | **$34$** |
| Step 15 | $15$ | $19$ | **$34$** |
| Step 30 | $30$ | $4$ | **$34$** |

### 3. Why Detours Increase $f$ to 36, 38, or 40

When a wall blocks the direct path, the algorithm must step sideways or backward:
* $g(n)$ increases by $1$ ($g \to g + 1$).
* $h(n)$ does not decrease (or increases by $1$).
* The total $f$-score jumps: **$34 \to 36 \to 38 \to 40$**.
* A\* automatically prioritizes nodes with $f=34$ first, only exploring $f=36$ or $f=38$ paths when direct routes are blocked.

---

## 📚 AI Theoretical Concepts

| Concept | Project Implementation |
|---|---|
| **Intelligent Agent** | Solver autonomously explores states and discovers optimal path |
| **State-Space Search** | Each traversable grid coordinate $(r, c)$ represents a discrete state |
| **Heuristic Search** | Manhattan distance guides search toward the goal, pruning bad branches |
| **Evaluation Function** | $f(n) = g(n) + h(n)$ calculated and displayed live for every candidate |
| **Admissibility** | Manhattan distance never overestimates true grid cost ($h(n) \le h^*(n)$), guaranteeing optimality |
| **Consistency / Monotonicity** | $h(n) \le c(n, a, n') + h(n')$, ensuring nodes are expanded in monotonic order of $f$ |
| **Priority Queue** | Min-heap maintains the Open Set, extracting the lowest-$(f, h)$ node in $O(\log n)$ time |

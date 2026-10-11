# Intelligent Maze Solver — A* Search & Live Calculation Lab

An interactive maze-solving application and educational calculation laboratory that visualises the **A\* heuristic search algorithm** in real time using a sleek modern Tkinter GUI.

---

## 🌟 Key Features

| Feature | Details |
|---|---|
| 🧮 **Live Calculation Lab** | Displays exact $f(n) = g(n) + h(n)$ formulas and values substituted in real time |
| 🧭 **4-Direction Neighbor Evaluation** | Evaluates North, South, East, West with tentative $g$, heuristic $h$, $f$, and heap decisions |
| 📋 **Priority Queue Inspector** | Real-time Open Set min-heap leaderboard showing why the lowest $f$ node is expanded next |
| 🔢 **Grid Cell Value Overlays** | Switch between viewing $f$-score, $g$-cost (distance from start), $h$-cost, or full $f, g, h$ on cells |
| 🔍 **Live Hover Cell Inspector** | Hover over any cell to inspect $(r, c)$, status, $g$, $h$, $f$, and parent node |
| ⏭ **Step-by-Step Execution** | Step forward one iteration at a time to examine calculations at your own pace |
| 📐 **Heuristic Selector** | Choose between **Manhattan** ($|dx| + |dy|$), **Euclidean** ($\sqrt{dx^2+dy^2}$), and **Chebyshev** |
| 🎲 **1-Click Maze Presets** | Instant test patterns: *Simple Maze*, *Random (28%)*, *Spiral Labyrinth*, and *Clear Walls* |
| 🧱 **Interactive Drawing** | Click and drag to draw walls, set start (🟢) and goal (🔴), or erase |
| ⚡ **Speed Control & Resizer** | Smooth animation speed slider (1 ms to 200 ms) and dynamic grid resizer |

---

## 📁 Project Structure

```
AstarMaze/
├── main.py     # Interactive Tkinter GUI & Live Calculation Lab
├── astar.py    # Pure A* Search generator with calculation metrics & heuristics
├── grid.py     # 2-D grid representation, cell states, neighbor enumeration & presets
└── README.md   # Documentation and algorithm reference
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
* **Presets**: Use **Simple Maze**, **Random 28%**, or **Spiral Lab** to instantly generate complex mazes.

### 2 — Select Heuristic & Cell Number Display

* **Heuristics**:
  * **Manhattan** ($|dx| + |dy|$): Standard, admissible heuristic for 4-directional grid movement.
  * **Euclidean** ($\sqrt{dx^2 + dy^2}$): Straight-line Euclidean distance.
  * **Chebyshev** ($\max(|dx|, |dy|)$): Diagonal metric.
* **Cell Values on Grid**:
  * `f-score (total: g + h)`: Shows total estimated cost.
  * `g-cost (from start: 0, 1, 2…)`: Shows step count propagating outward from Start.
  * `h-cost (heuristic to goal)`: Shows remaining distance counting down to $0$ at Goal.
  * `f, g, h breakdown`: Shows $f$ in the center with $g$ and $h$ in the corners.
  * `Hide numbers (clean)`: Minimalist view.

### 3 — Run, Step, and Inspect

* **▶ Run Search**: Starts animated continuous search.
* **⏭ Step**: Advances exactly one node expansion so you can inspect the arithmetic and neighbor decisions.
* **⏸ Pause / Resume**: Freeze search at any point to inspect candidate nodes.
* **↺ Reset Search**: Keeps walls and start/goal intact while clearing paths and calculations.

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

## 🧮 How A\* Works & The Mathematics

### 1. The Evaluation Function

At every step, A\* scores candidate cells with:

$$f(n) = g(n) + h(n)$$

| Component | Definition | Meaning |
|---|---|---|
| $g(n)$ | Exact Path Cost | Number of steps taken along the cheapest known path from **Start → $n$** |
| $h(n)$ | Heuristic Estimate | Estimated distance remaining from **$n$ → Goal** |
| $f(n)$ | Total Estimated Cost | Estimated total cost of an entire path passing through node $n$ |

The algorithm always extracts and expands the cell with the **lowest $f(n)$** from the priority queue.

---

### 2. Why Direct Paths Share the Same $f$-Score

If Start is at $(2, 2)$ and Goal is at $(15, 23)$, the initial Manhattan distance is:

$$\text{Distance} = |15 - 2| + |23 - 2| = 13 + 21 = \mathbf{34}$$

Whenever A\* moves closer to the goal (Down $\downarrow$ or Right $\rightarrow$):
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

When a wall blocks the path, the algorithm must step sideways or backward:
* $g(n)$ increases by $1$ ($g \to g + 1$).
* $h(n)$ does not decrease (or increases by $1$).
* The total $f$-score jumps: **$34 \to 36 \to 38 \to 40$**.
* A\* automatically prioritizes nodes with $f=34$ first, only exploring $f=36$ or $f=38$ paths when direct routes are blocked.

---

## 📚 AI Syllabus & Theoretical Concepts

| Concept | Project Implementation |
|---|---|
| **Intelligent Agent** | Solver autonomously explores states and discovers optimal path |
| **State-Space Search** | Each traversable grid coordinate $(r, c)$ represents a discrete state |
| **Heuristic Search** | Manhattan distance guides search toward the goal, pruning bad branches |
| **Evaluation Function** | $f(n) = g(n) + h(n)$ calculated and displayed for every candidate |
| **Admissibility** | Manhattan distance never overestimates true grid cost ($h(n) \le h^*(n)$), guaranteeing optimality |
| **Priority Queue** | Min-heap maintains the Open Set, extracting the lowest-$f$ node in $O(\log n)$ time |

# A* Maze Solver & Pathfinding Visualizer

An interactive pathfinding visualizer and algorithm laboratory built with **HTML5, CSS3, and JavaScript (ES6+)**.

This project provides a clean, step-by-step demonstration of the **A\* Search Algorithm**, featuring live mathematical evaluations ($f(n) = g(n) + h(n)$), real-time priority queue inspection, and an intuitive grid for drawing custom mazes and obstacles.

---

## How to Run the Project

The visualizer is **100% client-side** with zero external dependencies or build steps.

### 1-Click Launch:
Simply **double-click** [index.html](file:///c:/Users/tejas/Documents/tejas/codin/A-Star-Maze/index.html) to open it in any web browser.

Or run from PowerShell:
```powershell
start index.html
```

---

## The Core Mathematics of A*

Every candidate node $n$ is evaluated using the formula:

$$f(n) = g(n) + h(n)$$

| Component | Meaning | How it is Calculated |
|---|---|---|
| **$g(n)$** | **Past Cost** | Exact number of steps taken from the Start node to node $n$. |
| **$h(n)$** | **Future Estimate (Heuristic)** | Estimated distance from node $n$ to Goal using **Manhattan Distance**: $\|r_n - r_{goal}\| + \|c_n - c_{goal}\|$. |
| **$f(n)$** | **Total Estimated Cost** | Sum of $g(n) + h(n)$. The node with the **lowest $f(n)$** is always expanded next. |

### Why Manhattan Distance?
For a 4-directional grid (moving Up, Down, Left, Right), Manhattan distance is:
1. **Admissible**: It never overestimates the true remaining distance ($h(n) \le h^*(n)$).
2. **Consistent (Monotonic)**: It satisfies the triangle inequality, guaranteeing that when the goal is reached, the path found is **mathematically optimal**.

---

## Step-by-Step Algorithm Flow

```mermaid
flowchart TD
    A[Start: Push Start Node to Open Set Heap] --> B{Is Open Set Empty?}
    B -- Yes --> C[No Path Possible / Unreachable]
    B -- No --> D[Pop Node with Lowest f from Heap]
    D --> E{Is Node the Goal?}
    E -- Yes --> F[Backtrack parent pointers to construct Shortest Path]
    E -- No --> G[Add Current Node to Closed Set]
    G --> H[Check 4 Orthogonal Neighbors: Up, Down, Left, Right]
    H --> I[If neighbor is Wall or in Closed Set, Skip]
    I --> J[Calculate tentative g = current.g + 1]
    J --> K{Is tentative g < neighbor.g?}
    K -- Yes --> L[Update neighbor parent, g, h, f and push to Open Set Heap]
    K -- No --> M[Skip neighbor]
    L --> B
    M --> B
```

---

## Code Architecture (How to Walk Through the Code)

The codebase is organized into clean, modular files designed for easy explanation:

```
A-Star-Maze/
├── index.html     # Semantic layout: header metrics, control toolbar, grid canvas, and live math card
├── style.css      # Dark-mode styling, glassmorphism, responsive grid layout
├── script.js     # Clean, well-commented JavaScript engine (~350 lines)
└── README.md      # Algorithm documentation and Viva guide
```

### Breakdown of [script.js](file:///c:/Users/tejas/Documents/tejas/codin/A-Star-Maze/script.js):
1. **Cell Constants**: Defines `EMPTY`, `WALL`, `START`, `GOAL`, `OPEN`, `CLOSED`, `PATH`.
2. **`MinHeap` Class**: Implements a binary min-heap for the Open Set so that popping the node with the lowest $f(n)$ takes $O(\log N)$ time.
3. **`manhattanDistance()`**: Computes $|r_1 - r_2| + |c_1 - c_2|$.
4. **`*aStarGenerator()`**: An ES6 generator function running the core A* algorithm. Yielding at each step enables both automatic smooth animation and single-step manual stepping.
5. **`_render()`**: Draws the grid, active expanding node, cell numbers ($f$-score), and the glowing golden shortest path on the HTML5 Canvas.

---

## Grid Color Legend

| Color | State | Meaning |
|---|---|---|
| Green | Start | Starting position (S) |
| Red | Goal | Target destination (G) |
| Slate | Wall | Impassable obstacle |
| Pink | Current Node | The active node currently being expanded |
| Blue | Open Set (Frontier) | Discovered candidate nodes waiting in the priority queue |
| Dark Slate | Closed Set (Visited) | Explored nodes already evaluated |
| Radiant Gold | Shortest Path | Optimal path reconstructed from Goal to Start |

---

## Keyboard Shortcuts

| Shortcut | Action |
|---|---|
| <kbd>Space</kbd> | Run / Pause Visualizer |
| <kbd>S</kbd> or <kbd>→</kbd> | Step forward one node expansion |
| <kbd>R</kbd> | Reset Search (preserves drawn walls) |
| <kbd>W</kbd> | Select Draw Wall tool |
| <kbd>E</kbd> | Select Erase Wall tool |

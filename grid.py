"""
grid.py — Grid and Cell Management
===================================
Manages the 2D maze grid, cell states, and coordinate utilities.
"""

# ── Cell State Constants ──────────────────────────────────────────────────────
EMPTY   = 0
WALL    = 1
START   = 2
GOAL    = 3
OPEN    = 4   # cells in the open set (frontier)
CLOSED  = 5   # cells already explored
PATH    = 6   # final reconstructed path


class Grid:
    """
    Represents the maze as a 2D list of cell states.

    Attributes
    ----------
    rows : int
        Number of rows in the grid.
    cols : int
        Number of columns in the grid.
    cells : list[list[int]]
        2-D array storing the state of each cell.
    start : tuple[int, int] | None
        (row, col) of the start cell.
    goal  : tuple[int, int] | None
        (row, col) of the goal cell.
    """

    def __init__(self, rows: int, cols: int) -> None:
        self.rows = rows
        self.cols = cols
        self.cells: list[list[int]] = [[EMPTY] * cols for _ in range(rows)]
        self.start: tuple[int, int] | None = None
        self.goal:  tuple[int, int] | None = None

    # ── State helpers ─────────────────────────────────────────────────────────

    def in_bounds(self, row: int, col: int) -> bool:
        """Return True if (row, col) is inside the grid."""
        return 0 <= row < self.rows and 0 <= col < self.cols

    def is_traversable(self, row: int, col: int) -> bool:
        """Return True if the cell is not a wall."""
        return self.in_bounds(row, col) and self.cells[row][col] != WALL

    def set_start(self, row: int, col: int) -> None:
        """Mark a cell as the start, clearing any previous start."""
        if self.start is not None:
            r, c = self.start
            self.cells[r][c] = EMPTY
        self.start = (row, col)
        self.cells[row][col] = START

    def set_goal(self, row: int, col: int) -> None:
        """Mark a cell as the goal, clearing any previous goal."""
        if self.goal is not None:
            r, c = self.goal
            self.cells[r][c] = EMPTY
        self.goal = (row, col)
        self.cells[row][col] = GOAL

    def toggle_wall(self, row: int, col: int) -> None:
        """Toggle a cell between EMPTY and WALL (cannot wall start/goal)."""
        if self.cells[row][col] in (START, GOAL):
            return
        if self.cells[row][col] == WALL:
            self.cells[row][col] = EMPTY
        else:
            self.cells[row][col] = WALL

    def set_wall(self, row: int, col: int) -> None:
        """Force a cell to WALL state (cannot wall start/goal)."""
        if self.cells[row][col] in (START, GOAL):
            return
        self.cells[row][col] = WALL

    def set_state(self, row: int, col: int, state: int) -> None:
        """Set a cell to any state, preserving start and goal markers."""
        if (row, col) == self.start and state not in (START, EMPTY, WALL):
            return   # don't overwrite start with algo colours
        if (row, col) == self.goal and state not in (GOAL, EMPTY, WALL):
            return   # don't overwrite goal with algo colours
        self.cells[row][col] = state

    # ── Neighbor enumeration ──────────────────────────────────────────────────

    def neighbors(self, row: int, col: int) -> list[tuple[int, int]]:
        """
        Return the 4-directional traversable neighbors of (row, col).
        Movement cost between adjacent cells is always 1.
        """
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        result = []
        for dr, dc in directions:
            nr, nc = row + dr, col + dc
            if self.is_traversable(nr, nc):
                result.append((nr, nc))
        return result

    # ── Reset helpers ─────────────────────────────────────────────────────────

    def clear_search(self) -> None:
        """Remove exploration/path colouring, keep walls, start, and goal."""
        for r in range(self.rows):
            for c in range(self.cols):
                if self.cells[r][c] in (OPEN, CLOSED, PATH):
                    self.cells[r][c] = EMPTY

    def clear_all(self) -> None:
        """Reset the entire grid to EMPTY."""
        self.cells = [[EMPTY] * self.cols for _ in range(self.rows)]
        self.start = None
        self.goal  = None

    def resize(self, rows: int, cols: int) -> None:
        """Resize the grid, discarding all existing state."""
        self.rows = rows
        self.cols = cols
        self.clear_all()

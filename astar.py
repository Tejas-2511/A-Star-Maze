"""
astar.py — A* Search Algorithm
===============================
Pure-Python implementation of A* search.
Implemented as a *generator function* that yields step-by-step progress,
enabling smooth GUI animation without threads.
"""

import heapq
from grid import Grid, CLOSED, OPEN, PATH, START, GOAL


# ── Heuristic ─────────────────────────────────────────────────────────────────

def manhattan(a: tuple[int, int], b: tuple[int, int]) -> int:
    """Manhattan distance — admissible heuristic for 4-directional movement."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# ── Path reconstruction helper ────────────────────────────────────────────────

def _reconstruct(came_from: dict, current: tuple) -> list[tuple]:
    """
    Walk the came_from map back from *current* to the start and return the
    list of cells in start→goal order.
    """
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.append(current)   # include start
    path.reverse()
    return path


# ── A* Search ─────────────────────────────────────────────────────────────────

def astar(grid: Grid):
    """
    A* search generator.

    Yields
    ------
    dict with keys:
        'explored'  : list of (row, col) cells added to the closed set this step
        'frontier'  : list of (row, col) cells currently on the open set
        'path'      : list of (row, col) path cells (only on final yield)
        'done'      : bool — True on the last yield
        'found'     : bool — True if a path was found
        'stats'     : dict with 'nodes_explored', 'path_length', 'elapsed_ms'

    Usage
    -----
        gen = astar(grid)
        for update in gen:
            render(update)
    """
    import time

    start = grid.start
    goal  = grid.goal

    if start is None or goal is None:
        yield {'done': True, 'found': False,
               'path': [], 'explored': [], 'frontier': [],
               'stats': {'nodes_explored': 0, 'path_length': 0, 'elapsed_ms': 0}}
        return

    t0 = time.perf_counter()

    # g_score[n] = cost of cheapest known path from start to n
    g_score: dict[tuple, float] = {start: 0}

    # f_score[n] = g_score[n] + h(n)
    f_start = manhattan(start, goal)

    # Priority queue entries: (f_score, tie-break counter, node)
    counter = 0
    open_heap: list = [(f_start, counter, start)]

    came_from: dict[tuple, tuple] = {}
    in_open:   set  = {start}
    closed:    set  = set()

    nodes_explored = 0

    while open_heap:
        _, _, current = heapq.heappop(open_heap)

        if current in closed:
            continue          # stale entry

        in_open.discard(current)
        closed.add(current)
        nodes_explored += 1

        # ── Yield progress ──────────────────────────────────────────────────
        if current == goal:
            path = _reconstruct(came_from, goal)
            elapsed = round((time.perf_counter() - t0) * 1000, 3)
            yield {
                'done': True, 'found': True,
                'path': path,
                'explored': [current],
                'frontier': list(in_open),
                'stats': {
                    'nodes_explored': nodes_explored,
                    'path_length': len(path) - 1,   # edges
                    'elapsed_ms': elapsed,
                },
            }
            return

        yield {
            'done': False, 'found': False,
            'path': [],
            'explored': [current],
            'frontier': list(in_open),
            'stats': None,
        }

        # ── Expand neighbors ─────────────────────────────────────────────────
        for neighbor in grid.neighbors(*current):
            if neighbor in closed:
                continue

            tentative_g = g_score[current] + 1   # unit cost

            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + manhattan(neighbor, goal)
                counter += 1
                heapq.heappush(open_heap, (f, counter, neighbor))
                in_open.add(neighbor)

    # No path found
    elapsed = round((time.perf_counter() - t0) * 1000, 3)
    yield {
        'done': True, 'found': False,
        'path': [],
        'explored': [],
        'frontier': [],
        'stats': {
            'nodes_explored': nodes_explored,
            'path_length': 0,
            'elapsed_ms': elapsed,
        },
    }


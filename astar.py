"""
astar.py — A* Search Algorithm & Heuristic Calculation Engine
=============================================================
Pure-Python implementation of A* search.
Implemented as a generator function that yields step-by-step progress,
complete calculation breakdowns (g, h, f values, formulas, neighbor
evaluations, and priority queue states) for deep GUI visualization.
"""

import heapq
import math
from grid import Grid, CLOSED, OPEN, PATH, START, GOAL


# ── Heuristic Functions ───────────────────────────────────────────────────────

def manhattan(a: tuple[int, int], b: tuple[int, int]) -> int:
    """Manhattan distance (|dx| + |dy|) — admissible for 4-directional grids."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a: tuple[int, int], b: tuple[int, int]) -> float:
    """Euclidean straight-line distance sqrt(dx^2 + dy^2)."""
    return round(math.hypot(a[0] - b[0], a[1] - b[1]), 2)


def chebyshev(a: tuple[int, int], b: tuple[int, int]) -> int:
    """Chebyshev distance max(|dx|, |dy|)."""
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


HEURISTIC_MAP = {
    "manhattan": manhattan,
    "euclidean": euclidean,
    "chebyshev": chebyshev,
}


def heuristic_formula_text(heuristic_name: str, a: tuple[int, int], b: tuple[int, int], val) -> str:
    """Generates human-readable formula string showing the actual numbers substituted."""
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    if heuristic_name == "manhattan":
        return f"|{a[0]}-{b[0]}| + |{a[1]}-{b[1]}| = {dr} + {dc} = {val}"
    elif heuristic_name == "euclidean":
        return f"√(({a[0]}-{b[0]})² + ({a[1]}-{b[1]})²) = √({dr*dr} + {dc*dc}) = {val}"
    elif heuristic_name == "chebyshev":
        return f"max(|{a[0]}-{b[0]}|, |{a[1]}-{b[1]}|) = max({dr}, {dc}) = {val}"
    return f"h = {val}"


# ── Path reconstruction helper ────────────────────────────────────────────────

def _reconstruct(came_from: dict, current: tuple) -> list[tuple]:
    """
    Walk the came_from map back from current to the start and return the
    list of cells in start→goal order.
    """
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.append(current)   # include start
    path.reverse()
    return path


# ── A* Search Generator ───────────────────────────────────────────────────────

def astar(grid: Grid, heuristic_name: str = "manhattan"):
    """
    A* search generator yielding rich calculation metrics at every step.

    Parameters
    ----------
    grid : Grid
        The maze grid instance.
    heuristic_name : str
        'manhattan', 'euclidean', or 'chebyshev'.

    Yields
    ------
    dict with full calculation state and metrics for f(n) = g(n) + h(n).
    """
    import time

    start = grid.start
    goal = grid.goal

    if start is None or goal is None:
        yield {
            'done': True, 'found': False,
            'current': None, 'current_calc': None,
            'neighbors': [], 'open_queue': [],
            'g_scores': {}, 'h_scores': {}, 'f_scores': {}, 'came_from': {},
            'path': [], 'explored': [], 'frontier': [],
            'stats': {'nodes_explored': 0, 'path_length': 0, 'elapsed_ms': 0, 'open_size': 0, 'step': 0}
        }
        return

    h_func = HEURISTIC_MAP.get(heuristic_name, manhattan)
    t0 = time.perf_counter()

    # Cost tracking: g_score[n] = exact cost from start to n
    g_score: dict[tuple, float] = {start: 0}
    h_start = h_func(start, goal)
    h_score: dict[tuple, float] = {start: h_start}
    f_score: dict[tuple, float] = {start: h_start}

    # Priority queue: min-heap of (f_score, tie-break counter, node)
    counter = 0
    open_heap: list = [(h_start, counter, start)]
    came_from: dict[tuple, tuple] = {}
    in_open: set = {start}
    closed: set = set()

    nodes_explored = 0
    step_num = 0

    directions = [
        (-1, 0, "North (↑)"),
        (1, 0, "South (↓)"),
        (0, -1, "West (←)"),
        (0, 1, "East (→)"),
    ]

    while open_heap:
        f_curr, _, current = heapq.heappop(open_heap)

        if current in closed:
            continue  # Stale heap entry

        in_open.discard(current)
        closed.add(current)
        nodes_explored += 1
        step_num += 1

        curr_g = g_score.get(current, 0)
        curr_h = h_score.get(current, h_func(current, goal))
        curr_f = f_score.get(current, curr_g + curr_h)

        # Snapshot of top open set entries for queue inspector
        open_queue_snapshot = []
        for entry in sorted(open_heap):
            e_f, _, e_node = entry
            if e_node not in closed and e_node not in [x['pos'] for x in open_queue_snapshot]:
                open_queue_snapshot.append({
                    'pos': e_node,
                    'f': e_f,
                    'g': g_score.get(e_node, 0),
                    'h': h_score.get(e_node, 0),
                })
                if len(open_queue_snapshot) >= 6:
                    break

        # Formula text for current node: f(n) = g(n) + h(n)
        h_formula = heuristic_formula_text(heuristic_name, current, goal, curr_h)
        f_formula = f"f(n) = g({curr_g}) + h({curr_h}) = {curr_f}"

        current_calc = {
            'pos': current,
            'g': curr_g,
            'h': curr_h,
            'f': curr_f,
            'h_formula': h_formula,
            'f_formula': f_formula,
            'heuristic_name': heuristic_name,
        }

        # ── Goal Reached ─────────────────────────────────────────────────────
        if current == goal:
            path = _reconstruct(came_from, goal)
            elapsed = round((time.perf_counter() - t0) * 1000, 2)
            yield {
                'done': True, 'found': True,
                'current': current,
                'current_calc': current_calc,
                'neighbors': [],
                'open_queue': open_queue_snapshot,
                'g_scores': g_score,
                'h_scores': h_score,
                'f_scores': f_score,
                'came_from': came_from,
                'path': path,
                'explored': [current],
                'frontier': list(in_open),
                'stats': {
                    'nodes_explored': nodes_explored,
                    'path_length': len(path) - 1,
                    'elapsed_ms': elapsed,
                    'open_size': len(in_open),
                    'step': step_num,
                },
            }
            return

        # ── Evaluate 4 Neighbors with Mathematical Details ───────────────────
        neighbors_eval = []
        for dr, dc, dname in directions:
            nr, nc = current[0] + dr, current[1] + dc
            neighbor = (nr, nc)

            if not grid.in_bounds(nr, nc):
                continue

            if not grid.is_traversable(nr, nc):
                neighbors_eval.append({
                    'dir': dname,
                    'pos': neighbor,
                    'status': 'Wall',
                    'detail': 'Obstacle wall',
                    'g': None, 'h': None, 'f': None,
                })
                continue

            if neighbor in closed:
                neighbors_eval.append({
                    'dir': dname,
                    'pos': neighbor,
                    'status': 'Closed',
                    'detail': 'Already expanded in closed set',
                    'g': g_score.get(neighbor),
                    'h': h_score.get(neighbor),
                    'f': f_score.get(neighbor),
                })
                continue

            tentative_g = curr_g + 1  # Unit step cost
            existing_g = g_score.get(neighbor, float('inf'))
            n_h = h_func(neighbor, goal)
            n_f = round(tentative_g + n_h, 2)

            if tentative_g < existing_g:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                h_score[neighbor] = n_h
                f_score[neighbor] = n_f
                counter += 1
                heapq.heappush(open_heap, (n_f, counter, neighbor))
                is_new = neighbor not in in_open
                in_open.add(neighbor)

                status_label = "Added to Open" if is_new else "Updated Cost"
                detail_msg = f"g={tentative_g}, h={n_h} ⇒ f={n_f}"
                neighbors_eval.append({
                    'dir': dname,
                    'pos': neighbor,
                    'status': status_label,
                    'detail': detail_msg,
                    'g': tentative_g,
                    'h': n_h,
                    'f': n_f,
                })
            else:
                neighbors_eval.append({
                    'dir': dname,
                    'pos': neighbor,
                    'status': 'Skipped',
                    'detail': f"Existing g={existing_g} ≤ new g={tentative_g}",
                    'g': tentative_g,
                    'h': n_h,
                    'f': n_f,
                })

        # Yield step state for rendering & inspection
        yield {
            'done': False, 'found': False,
            'current': current,
            'current_calc': current_calc,
            'neighbors': neighbors_eval,
            'open_queue': open_queue_snapshot,
            'g_scores': g_score,
            'h_scores': h_score,
            'f_scores': f_score,
            'came_from': came_from,
            'path': [],
            'explored': [current],
            'frontier': list(in_open),
            'stats': {
                'nodes_explored': nodes_explored,
                'path_length': 0,
                'elapsed_ms': round((time.perf_counter() - t0) * 1000, 2),
                'open_size': len(in_open),
                'step': step_num,
            },
        }

    # ── No Path Found ────────────────────────────────────────────────────────
    elapsed = round((time.perf_counter() - t0) * 1000, 2)
    yield {
        'done': True, 'found': False,
        'current': None, 'current_calc': None,
        'neighbors': [], 'open_queue': [],
        'g_scores': g_score,
        'h_scores': h_score,
        'f_scores': f_score,
        'came_from': came_from,
        'path': [],
        'explored': [],
        'frontier': [],
        'stats': {
            'nodes_explored': nodes_explored,
            'path_length': 0,
            'elapsed_ms': elapsed,
            'open_size': 0,
            'step': step_num,
        },
    }

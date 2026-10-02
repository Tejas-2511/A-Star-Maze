"""
main.py — Intelligent Maze Solver (A* Search)
=============================================
Tkinter GUI with:
  • Interactive grid — place walls, start, and goal with mouse
  • Animated A* search step-by-step
  • Speed slider and Step / Pause controls
  • Statistics panel (nodes explored, path length, time)
  • Dark-mode, colour-coded cells, smooth animations
"""

import tkinter as tk
from tkinter import ttk, messagebox, font
import time

from grid import (
    Grid, EMPTY, WALL, START, GOAL, OPEN, CLOSED, PATH
)
from astar import astar

# ── Palette ───────────────────────────────────────────────────────────────────
PALETTE = {
    "bg":           "#0f1117",
    "panel":        "#1a1d27",
    "panel_border": "#2a2d3e",
    "accent":       "#7c6af7",       # purple
    "accent2":      "#4fc3f7",       # cyan
    "accent3":      "#f7c948",       # gold
    "text":         "#e2e8f0",
    "text_dim":     "#64748b",
    "btn_hover":    "#9580ff",

    # cell colours
    EMPTY:   "#1e2132",
    WALL:    "#2d3250",              # dark blue-grey walls
    START:   "#22c55e",             # vivid green
    GOAL:    "#ef4444",             # vivid red
    OPEN:    "#1e40af",             # frontier (dark blue)
    CLOSED:  "#1d4ed8",             # explored (mid blue)
    PATH:    "#f59e0b",             # golden path
    "grid_line": "#262840",
}

CELL_BORDER  = 1
MIN_CELL_PX  = 12
MAX_CELL_PX  = 48
DEFAULT_ROWS = 25
DEFAULT_COLS = 35
DEFAULT_SPEED = 20   # ms between steps


class MazeSolverApp:
    """Main application window."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Intelligent Maze Solver — A* Search")
        self.root.configure(bg=PALETTE["bg"])
        self.root.resizable(True, True)

        # ── State ─────────────────────────────────────────────────────────────
        self.grid = Grid(DEFAULT_ROWS, DEFAULT_COLS)
        self.mode          = tk.StringVar(value="wall")   # wall | start | goal
        self.speed_ms      = tk.IntVar(value=DEFAULT_SPEED)
        self.is_running    = False
        self.is_paused     = False
        self._after_id     = None
        self._search_gen   = None
        self._drag_erase   = False   # True while drag-erasing walls
        self._last_drag    = (-1, -1)

        # stats
        self.stat_nodes    = tk.StringVar(value="—")
        self.stat_length   = tk.StringVar(value="—")
        self.stat_time     = tk.StringVar(value="—")
        self.stat_result   = tk.StringVar(value="Ready")
        self.stat_result_color = tk.StringVar(value=PALETTE["text_dim"])

        self._build_ui()
        self._draw_grid()

    # ═══════════════════════════════════════════════════════════════════════════
    # UI CONSTRUCTION
    # ═══════════════════════════════════════════════════════════════════════════

    def _build_ui(self) -> None:
        """Assemble left panel + canvas area."""
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        self._build_left_panel()
        self._build_canvas_area()

    # ── Left control panel ────────────────────────────────────────────────────

    def _build_left_panel(self) -> None:
        panel = tk.Frame(self.root, bg=PALETTE["panel"], width=220,
                         highlightbackground=PALETTE["panel_border"],
                         highlightthickness=1)
        panel.grid(row=0, column=0, sticky="ns", padx=(8, 0), pady=8)
        panel.grid_propagate(False)
        panel.columnconfigure(0, weight=1)

        row = 0

        # Title
        tk.Label(panel, text="🧭  Maze Solver",
                 bg=PALETTE["panel"], fg=PALETTE["accent"],
                 font=("Segoe UI", 14, "bold")).grid(
            row=row, column=0, pady=(18, 4)); row += 1

        tk.Label(panel, text="A* Heuristic Search",
                 bg=PALETTE["panel"], fg=PALETTE["text_dim"],
                 font=("Segoe UI", 9)).grid(
            row=row, column=0, pady=(0, 16)); row += 1

        self._section(panel, row, "DRAW MODE"); row += 1
        modes = [("🧱  Draw Wall",  "wall"),
                 ("🟢  Set Start",   "start"),
                 ("🔴  Set Goal",    "goal"),
                 ("🗑️  Erase",       "erase")]
        self._mode_btns: dict[str, tk.Button] = {}
        for label, val in modes:
            b = self._toggle_btn(panel, row, label, val)
            self._mode_btns[val] = b
            row += 1

        self._section(panel, row, "SPEED"); row += 1
        speed_frame = tk.Frame(panel, bg=PALETTE["panel"])
        speed_frame.grid(row=row, column=0, sticky="ew", padx=14, pady=(0, 8))
        row += 1
        tk.Label(speed_frame, text="Slow", bg=PALETTE["panel"],
                 fg=PALETTE["text_dim"], font=("Segoe UI", 8)).pack(side="left")
        ttk.Scale(speed_frame, from_=200, to=5, variable=self.speed_ms,
                  orient="horizontal").pack(side="left", fill="x", expand=True, padx=4)
        tk.Label(speed_frame, text="Fast", bg=PALETTE["panel"],
                 fg=PALETTE["text_dim"], font=("Segoe UI", 8)).pack(side="left")

        self._section(panel, row, "CONTROLS"); row += 1
        self.run_btn = self._action_btn(
            panel, row, "▶  Run Search", self._run_search,
            bg=PALETTE["accent"]); row += 1
        self.pause_btn = self._action_btn(
            panel, row, "⏸  Pause", self._toggle_pause,
            bg="#334155", state="disabled"); row += 1
        self._action_btn(panel, row, "↺  Clear Search", self._clear_search,
                         bg="#1e293b"); row += 1
        self._action_btn(panel, row, "⬛  Clear All",   self._clear_all,
                         bg="#1e293b"); row += 1

        self._section(panel, row, "GRID SIZE"); row += 1
        size_frame = tk.Frame(panel, bg=PALETTE["panel"])
        size_frame.grid(row=row, column=0, sticky="ew", padx=14, pady=(0, 8))
        row += 1
        tk.Label(size_frame, text="Rows:", bg=PALETTE["panel"],
                 fg=PALETTE["text_dim"], font=("Segoe UI", 9)).grid(
            row=0, column=0, sticky="w")
        self.rows_var = tk.IntVar(value=DEFAULT_ROWS)
        tk.Spinbox(size_frame, from_=5, to=50, textvariable=self.rows_var,
                   width=5, bg=PALETTE["panel_border"], fg=PALETTE["text"],
                   insertbackground=PALETTE["text"],
                   buttonbackground=PALETTE["panel_border"],
                   relief="flat", font=("Segoe UI", 9)).grid(
            row=0, column=1, padx=(6, 0))
        tk.Label(size_frame, text="Cols:", bg=PALETTE["panel"],
                 fg=PALETTE["text_dim"], font=("Segoe UI", 9)).grid(
            row=1, column=0, sticky="w", pady=(4, 0))
        self.cols_var = tk.IntVar(value=DEFAULT_COLS)
        tk.Spinbox(size_frame, from_=5, to=70, textvariable=self.cols_var,
                   width=5, bg=PALETTE["panel_border"], fg=PALETTE["text"],
                   insertbackground=PALETTE["text"],
                   buttonbackground=PALETTE["panel_border"],
                   relief="flat", font=("Segoe UI", 9)).grid(
            row=1, column=1, padx=(6, 0), pady=(4, 0))
        self._action_btn(size_frame, 2, "Apply", self._apply_grid_size,
                         bg=PALETTE["accent2"], colspan=2, pady=6)

        # ── Stats panel ───────────────────────────────────────────────────────
        self._section(panel, row, "STATISTICS"); row += 1
        stats_frame = tk.Frame(panel, bg=PALETTE["panel_border"],
                                highlightbackground=PALETTE["panel_border"],
                                highlightthickness=1)
        stats_frame.grid(row=row, column=0, sticky="ew",
                         padx=14, pady=(0, 8)); row += 1
        stats_frame.columnconfigure(1, weight=1)

        for i, (lbl, var) in enumerate([
            ("Nodes:", self.stat_nodes),
            ("Path:",  self.stat_length),
            ("Time:",  self.stat_time),
        ]):
            tk.Label(stats_frame, text=lbl, bg=PALETTE["panel_border"],
                     fg=PALETTE["text_dim"], font=("Segoe UI", 9)).grid(
                row=i, column=0, sticky="w", padx=8, pady=3)
            tk.Label(stats_frame, textvariable=var, bg=PALETTE["panel_border"],
                     fg=PALETTE["accent3"], font=("Segoe UI", 9, "bold")).grid(
                row=i, column=1, sticky="e", padx=8)

        # result label
        self.result_lbl = tk.Label(panel, textvariable=self.stat_result,
                                   bg=PALETTE["panel"],
                                   fg=PALETTE["text_dim"],
                                   font=("Segoe UI", 10, "bold"),
                                   wraplength=195)
        self.result_lbl.grid(row=row, column=0, pady=(4, 12)); row += 1

        # ── Legend ────────────────────────────────────────────────────────────
        self._section(panel, row, "LEGEND"); row += 1
        legend = [
            (PALETTE[START],  "Start"),
            (PALETTE[GOAL],   "Goal"),
            (PALETTE[WALL],   "Wall"),
            (PALETTE[CLOSED], "Explored"),
            (PALETTE[OPEN],   "Frontier"),
            (PALETTE[PATH],   "Path"),
        ]
        leg_frame = tk.Frame(panel, bg=PALETTE["panel"])
        leg_frame.grid(row=row, column=0, sticky="ew", padx=14, pady=(0, 16))
        for i, (color, label) in enumerate(legend):
            swatch = tk.Frame(leg_frame, bg=color, width=14, height=14)
            swatch.grid(row=i, column=0, padx=(0, 8), pady=2)
            tk.Label(leg_frame, text=label, bg=PALETTE["panel"],
                     fg=PALETTE["text_dim"], font=("Segoe UI", 9)).grid(
                row=i, column=1, sticky="w")

    # ── Canvas area ───────────────────────────────────────────────────────────

    def _build_canvas_area(self) -> None:
        frame = tk.Frame(self.root, bg=PALETTE["bg"])
        frame.grid(row=0, column=1, sticky="nsew", padx=8, pady=8)
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(frame, bg=PALETTE[EMPTY],
                                highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")

        # mouse bindings
        self.canvas.bind("<Button-1>",        self._on_click)
        self.canvas.bind("<B1-Motion>",       self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Configure>",       self._on_resize)

    # ── Widget helpers ────────────────────────────────────────────────────────

    def _section(self, parent: tk.Frame, row: int, text: str) -> None:
        f = tk.Frame(parent, bg=PALETTE["panel_border"], height=1)
        f.grid(row=row, column=0, sticky="ew", padx=14, pady=(10, 2))
        tk.Label(parent, text=text, bg=PALETTE["panel"],
                 fg=PALETTE["text_dim"], font=("Segoe UI", 7, "bold")).grid(
            row=row + 1, column=0, sticky="w", padx=16, pady=(0, 4))
        # We offset by occupying the "section+label" as two rows but only pass
        # one row index — caller increments by 1.  We embed label inside same row.

    def _toggle_btn(self, parent: tk.Frame, row: int,
                    label: str, mode: str) -> tk.Button:
        def cmd():
            self.mode.set(mode)
            self._refresh_mode_btns()

        b = tk.Button(parent, text=label, command=cmd,
                      bg=PALETTE["panel_border"], fg=PALETTE["text"],
                      activebackground=PALETTE["accent"],
                      activeforeground="white",
                      relief="flat", bd=0, cursor="hand2",
                      font=("Segoe UI", 10), anchor="w",
                      padx=12, pady=6)
        b.grid(row=row, column=0, sticky="ew", padx=14, pady=2)
        return b

    def _action_btn(self, parent: tk.Frame, row: int, label: str,
                    cmd, bg: str, state: str = "normal",
                    colspan: int = 1, pady: int = 2) -> tk.Button:
        b = tk.Button(parent, text=label, command=cmd,
                      bg=bg, fg="white",
                      activebackground=PALETTE["btn_hover"],
                      activeforeground="white",
                      relief="flat", bd=0, cursor="hand2",
                      font=("Segoe UI", 10, "bold"),
                      pady=7, state=state)
        b.grid(row=row, column=0, columnspan=colspan,
               sticky="ew", padx=14, pady=pady)
        return b

    def _refresh_mode_btns(self) -> None:
        current = self.mode.get()
        for mode, btn in self._mode_btns.items():
            if mode == current:
                btn.configure(bg=PALETTE["accent"], fg="white")
            else:
                btn.configure(bg=PALETTE["panel_border"], fg=PALETTE["text"])


    # ═══════════════════════════════════════════════════════════════════════════
    # GRID DRAWING
    # ═══════════════════════════════════════════════════════════════════════════

    def _cell_size(self) -> int:
        cw = self.canvas.winfo_width()  or 600
        ch = self.canvas.winfo_height() or 500
        sz = min(cw // self.grid.cols, ch // self.grid.rows)
        return max(MIN_CELL_PX, min(MAX_CELL_PX, sz))

    def _cell_rect(self, row: int, col: int,
                   sz: int) -> tuple[int, int, int, int]:
        x0 = col * sz + CELL_BORDER
        y0 = row * sz + CELL_BORDER
        x1 = x0 + sz - CELL_BORDER
        y1 = y0 + sz - CELL_BORDER
        return x0, y0, x1, y1

    def _draw_grid(self) -> None:
        """Full redraw of the grid (called on resize or clear)."""
        self.canvas.delete("all")
        sz = self._cell_size()
        self._rect_ids: dict[tuple[int, int], int] = {}

        for r in range(self.grid.rows):
            for c in range(self.grid.cols):
                x0, y0, x1, y1 = self._cell_rect(r, c, sz)
                color = PALETTE[self.grid.cells[r][c]]
                rid = self.canvas.create_rectangle(
                    x0, y0, x1, y1,
                    fill=color, outline=PALETTE["grid_line"],
                    width=CELL_BORDER, tags=f"cell_{r}_{c}"
                )
                self._rect_ids[(r, c)] = rid

    def _update_cell(self, row: int, col: int) -> None:
        """Repaint a single cell without full redraw."""
        rid = self._rect_ids.get((row, col))
        if rid is None:
            return
        color = PALETTE[self.grid.cells[row][col]]
        self.canvas.itemconfigure(rid, fill=color)

    def _on_resize(self, _event=None) -> None:
        self._draw_grid()

    # ═══════════════════════════════════════════════════════════════════════════
    # MOUSE INTERACTION
    # ═══════════════════════════════════════════════════════════════════════════

    def _canvas_to_cell(self, x: int, y: int) -> tuple[int, int] | None:
        sz = self._cell_size()
        col = x // sz
        row = y // sz
        if self.grid.in_bounds(row, col):
            return row, col
        return None

    def _on_click(self, event: tk.Event) -> None:
        if self.is_running:
            return
        cell = self._canvas_to_cell(event.x, event.y)
        if cell is None:
            return
        row, col = cell
        mode = self.mode.get()

        if mode == "start":
            self.grid.set_start(row, col)
            # refresh neighbours that may have been start before
            self._draw_grid()
        elif mode == "goal":
            self.grid.set_goal(row, col)
            self._draw_grid()
        elif mode == "wall":
            self.grid.set_wall(row, col)
            self._update_cell(row, col)
            self._drag_erase = False
        elif mode == "erase":
            if self.grid.cells[row][col] not in (START, GOAL):
                self.grid.cells[row][col] = EMPTY
                self._update_cell(row, col)
        self._last_drag = (row, col)

    def _on_drag(self, event: tk.Event) -> None:
        if self.is_running:
            return
        cell = self._canvas_to_cell(event.x, event.y)
        if cell is None or cell == self._last_drag:
            return
        row, col = cell
        mode = self.mode.get()
        if mode == "wall":
            self.grid.set_wall(row, col)
            self._update_cell(row, col)
        elif mode == "erase":
            if self.grid.cells[row][col] not in (START, GOAL):
                self.grid.cells[row][col] = EMPTY
                self._update_cell(row, col)
        self._last_drag = (row, col)

    def _on_release(self, _event: tk.Event) -> None:
        self._drag_erase = False
        self._last_drag  = (-1, -1)

    # ═══════════════════════════════════════════════════════════════════════════
    # SEARCH ANIMATION
    # ═══════════════════════════════════════════════════════════════════════════

    def _run_search(self) -> None:
        if self.grid.start is None or self.grid.goal is None:
            messagebox.showwarning(
                "Missing points",
                "Please set both a Start (🟢) and a Goal (🔴) before running."
            )
            return

        self._clear_search()
        self._search_gen = astar(self.grid)

        self.is_running = True
        self.is_paused  = False
        self.stat_result.set("Searching…")
        self.result_lbl.configure(fg=PALETTE["accent2"])
        self.run_btn.configure(state="disabled")
        self.pause_btn.configure(state="normal", text="⏸  Pause")
        self._step()

    def _step(self) -> None:
        if not self.is_running or self.is_paused:
            return

        try:
            update = next(self._search_gen)
        except StopIteration:
            self._finish(found=False, stats=None)
            return

        # Paint explored cells
        for cell in update.get("explored", []):
            r, c = cell
            if (r, c) != self.grid.start and (r, c) != self.grid.goal:
                self.grid.cells[r][c] = CLOSED
                self._update_cell(r, c)

        # Paint frontier (open set) — lighter colour
        for cell in update.get("frontier", []):
            r, c = cell
            if self.grid.cells[r][c] not in (START, GOAL, CLOSED):
                self.grid.cells[r][c] = OPEN
                self._update_cell(r, c)

        if update.get("done"):
            if update["found"]:
                self._paint_path(update["path"])
            self._finish(found=update["found"], stats=update.get("stats"))
            return

        self._after_id = self.root.after(self.speed_ms.get(), self._step)

    def _paint_path(self, path: list) -> None:
        for r, c in path:
            if (r, c) != self.grid.start and (r, c) != self.grid.goal:
                self.grid.cells[r][c] = PATH
                self._update_cell(r, c)

    def _finish(self, found: bool, stats: dict | None) -> None:
        self.is_running = False
        self.run_btn.configure(state="normal")
        self.pause_btn.configure(state="disabled")

        if found and stats:
            self.stat_nodes.set(str(stats["nodes_explored"]))
            self.stat_length.set(f"{stats['path_length']} cells")
            self.stat_time.set(f"{stats['elapsed_ms']} ms")
            self.stat_result.set("✅  Path found!")
            self.result_lbl.configure(fg="#22c55e")
        elif stats:
            self.stat_nodes.set(str(stats.get("nodes_explored", "—")))
            self.stat_length.set("—")
            self.stat_time.set(f"{stats.get('elapsed_ms', '—')} ms")
            self.stat_result.set("❌  No path exists")
            self.result_lbl.configure(fg="#ef4444")
        else:
            self.stat_result.set("❌  No path exists")
            self.result_lbl.configure(fg="#ef4444")

    def _toggle_pause(self) -> None:
        if not self.is_running:
            return
        self.is_paused = not self.is_paused
        if self.is_paused:
            self.pause_btn.configure(text="▶  Resume")
            if self._after_id:
                self.root.after_cancel(self._after_id)
        else:
            self.pause_btn.configure(text="⏸  Pause")
            self._step()

    # ── Clear helpers ─────────────────────────────────────────────────────────

    def _clear_search(self) -> None:
        if self._after_id:
            self.root.after_cancel(self._after_id)
        self.is_running = False
        self.is_paused  = False
        self._search_gen = None
        self.run_btn.configure(state="normal")
        self.pause_btn.configure(state="disabled", text="⏸  Pause")
        self.grid.clear_search()
        self.stat_nodes.set("—")
        self.stat_length.set("—")
        self.stat_time.set("—")
        self.stat_result.set("Ready")
        self.result_lbl.configure(fg=PALETTE["text_dim"])
        self._draw_grid()

    def _clear_all(self) -> None:
        self._clear_search()
        self.grid.clear_all()
        self._draw_grid()

    # ── Grid resize ───────────────────────────────────────────────────────────

    def _apply_grid_size(self) -> None:
        rows = max(5, min(50, self.rows_var.get()))
        cols = max(5, min(70, self.cols_var.get()))
        self.grid.resize(rows, cols)
        self._draw_grid()


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    root = tk.Tk()
    root.geometry("1100x680")
    root.minsize(800, 520)

    # Try to use a nice font if available
    try:
        default_font = font.nametofont("TkDefaultFont")
        default_font.configure(family="Segoe UI", size=10)
    except Exception:
        pass

    app = MazeSolverApp(root)
    app._refresh_mode_btns()     # highlight the default draw mode
    root.mainloop()


if __name__ == "__main__":
    main()

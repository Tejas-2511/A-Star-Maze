"""
main.py — Intelligent Maze Solver (A* Search & Calculation Lab)
==============================================================
Interactive Tkinter GUI dedicated purely to A* Search (f = g + h):
  • Interactive grid: place walls, start, and goal with mouse or presets
  • Live step-by-step mathematical calculations:
      - Formula breakdown: f(n) = g(n) + h(n) with actual numbers substituted
      - Heuristic calculation breakdown (Manhattan: |dx| + |dy|, Euclidean, Chebyshev)
      - 4-direction neighbor evaluations (tentative g, heuristic h, f, and heap actions)
      - Open Set Priority Queue (min-heap top candidates showing why lowest f is chosen)
      - Cell numbers rendered directly on grid cells (f, g, h)
      - Live Hover Cell Inspector (inspect any cell on mouseover without lag)
  • Search execution controls:
      - Execution: Run, Step-Forward (⏭), Pause/Resume (⏸), Reset Search, Clear All
      - Maze Presets: Simple Maze, Random Walls, Spiral Labyrinth, Clear Walls
      - Speed control: slider from 1ms to 200ms
  • Modern dark aesthetic with sleek card panels and color-coded status badges
"""

import tkinter as tk
from tkinter import ttk, messagebox, font
import time

from grid import (
    Grid, EMPTY, WALL, START, GOAL, OPEN, CLOSED, PATH
)
from astar import astar, manhattan, HEURISTIC_MAP, heuristic_formula_text

# ── Color Palette ─────────────────────────────────────────────────────────────
PALETTE = {
    "bg":           "#0b0f19",       # Deep space slate
    "panel":        "#131824",       # Control card background
    "panel_border": "#1f293d",       # Subtle card borders
    "card_bg":      "#1a2234",       # Inner sub-card background
    "card_border":  "#27354f",       # Inner border
    "header_bg":    "#161d2d",

    # Accents
    "primary":      "#6366f1",       # Electric Indigo
    "primary_hover":"#797cf5",
    "accent_cyan":  "#38bdf8",       # Sky cyan
    "accent_gold":  "#fbbf24",       # Amber gold
    "accent_green": "#10b981",       # Emerald green
    "accent_rose":  "#f43f5e",       # Rose / active highlight
    "accent_purple":"#a855f7",       # Violet

    # Typography
    "text":         "#f8fafc",       # Primary light
    "text_sub":     "#94a3b8",       # Secondary slate
    "text_dim":     "#64748b",       # Muted slate

    # Cell Colors
    EMPTY:          "#151c2c",       # Dark empty cell
    WALL:           "#2a3348",       # Slate obstacle wall
    START:          "#10b981",       # Vivid emerald start
    GOAL:           "#ef4444",       # Vivid ruby red goal
    OPEN:           "#1d4ed8",       # Frontier open set (blue)
    CLOSED:         "#1e293b",       # Explored closed set (slate blue)
    PATH:           "#f59e0b",       # Optimal solution path (gold)
    "ACTIVE":       "#ec4899",       # Current expanding cell (neon pink)
    "HOVER":        "#38bdf8",       # Hover crosshair color
    "grid_line":    "#1e2638",
    "cell_text":    "#ffffff",
}

DEFAULT_ROWS = 18
DEFAULT_COLS = 26
DEFAULT_SPEED = 25  # ms per step
MIN_CELL_PX = 10
MAX_CELL_PX = 56


class MazeSolverApp:
    """Main Application Window & A* Visual Calculation Lab."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Intelligent Maze Solver — A* Search & Live Calculation Lab")
        self.root.configure(bg=PALETTE["bg"])
        self.root.geometry("1340x820")
        self.root.minsize(1080, 660)

        # ── State ─────────────────────────────────────────────────────────────
        self.grid = Grid(DEFAULT_ROWS, DEFAULT_COLS)
        self.grid.set_start(2, 2)
        self.grid.set_goal(DEFAULT_ROWS - 3, DEFAULT_COLS - 3)

        self.mode = tk.StringVar(value="wall")            # wall | start | goal | erase
        self.heuristic = tk.StringVar(value="manhattan")  # manhattan | euclidean | chebyshev
        self.cell_display = tk.StringVar(value="f")       # f | fgh | none
        self.speed_ms = tk.IntVar(value=DEFAULT_SPEED)

        self.is_running = False
        self.is_paused = False
        self._after_id = None
        self._search_gen = None
        self._last_drag = (-1, -1)
        self._active_node = None
        self._hover_node = None

        # Live calculation caches
        self._g_scores = {}
        self._h_scores = {}
        self._f_scores = {}
        self._came_from = {}
        self._step_count = 0
        self._nodes_explored = 0

        # UI dynamic variables
        self.stat_status = tk.StringVar(value="Ready")
        self.stat_nodes = tk.StringVar(value="0")
        self.stat_path_len = tk.StringVar(value="—")
        self.stat_time = tk.StringVar(value="—")
        self.stat_open_size = tk.StringVar(value="0")
        self.stat_step_num = tk.StringVar(value="0")

        # Calculation card variables
        self.calc_node = tk.StringVar(value="None")
        self.calc_formula = tk.StringVar(value="f(n) = g(n) + h(n)")
        self.calc_g = tk.StringVar(value="—")
        self.calc_h = tk.StringVar(value="—")
        self.calc_f = tk.StringVar(value="—")
        self.calc_h_detail = tk.StringVar(value="Run or step the search to inspect live formulas")

        # Hover inspector variables
        self.hov_coords = tk.StringVar(value="Hover over any cell")
        self.hov_type = tk.StringVar(value="—")
        self.hov_g = tk.StringVar(value="—")
        self.hov_h = tk.StringVar(value="—")
        self.hov_f = tk.StringVar(value="—")
        self.hov_parent = tk.StringVar(value="—")

        # Canvas items cache
        self._rect_ids: dict[tuple[int, int], int] = {}
        self._text_ids: dict[tuple[int, int], int] = {}
        self._hover_rect_id = None

        self._build_ui()
        self._draw_grid()

    # ═══════════════════════════════════════════════════════════════════════════
    # UI CONSTRUCTION
    # ═══════════════════════════════════════════════════════════════════════════

    def _build_ui(self) -> None:
        """Construct the top header and 3-column dashboard layout."""
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(1, weight=1)

        # ── 1. Top Header Bar ────────────────────────────────────────────────
        header = tk.Frame(self.root, bg=PALETTE["header_bg"], height=52,
                          highlightbackground=PALETTE["panel_border"], highlightthickness=1)
        header.grid(row=0, column=0, columnspan=3, sticky="ew")
        header.grid_propagate(False)

        title_box = tk.Frame(header, bg=PALETTE["header_bg"])
        title_box.pack(side="left", padx=16, pady=8)

        tk.Label(title_box, text="🧭 A* Maze Solver & Calculation Lab",
                 bg=PALETTE["header_bg"], fg=PALETTE["text"],
                 font=("Segoe UI", 13, "bold")).pack(side="left")
        tk.Label(title_box, text="  |  f(n) = g(n) + h(n) Heuristic Search Visualization",
                 bg=PALETTE["header_bg"], fg=PALETTE["text_dim"],
                 font=("Segoe UI", 9)).pack(side="left")

        # Status badge pill
        self.status_badge = tk.Label(header, textvariable=self.stat_status,
                                     bg=PALETTE["card_bg"], fg=PALETTE["accent_cyan"],
                                     font=("Segoe UI", 10, "bold"),
                                     padx=14, pady=3, relief="flat",
                                     highlightbackground=PALETTE["card_border"],
                                     highlightthickness=1)
        self.status_badge.pack(side="right", padx=16, pady=10)

        # ── 2. Left Control Panel ─────────────────────────────────────────────
        self._build_left_panel()

        # ── 3. Center Canvas Area ─────────────────────────────────────────────
        self._build_canvas_area()

        # ── 4. Right Calculations & Inspector Panel ───────────────────────────
        self._build_right_panel()

    # ──────────────────────────────────────────────────────────────────────────
    # LEFT PANEL: Controls, Presets, Settings
    # ──────────────────────────────────────────────────────────────────────────

    def _build_left_panel(self) -> None:
        left_container = tk.Frame(self.root, bg=PALETTE["panel"], width=270,
                                  highlightbackground=PALETTE["panel_border"], highlightthickness=1)
        left_container.grid(row=1, column=0, sticky="nsew", padx=(10, 5), pady=8)
        left_container.grid_propagate(False)

        canvas = tk.Canvas(left_container, bg=PALETTE["panel"], highlightthickness=0, width=254)
        vscroll = ttk.Scrollbar(left_container, orient="vertical", command=canvas.yview)
        panel = tk.Frame(canvas, bg=PALETTE["panel"])

        panel.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=panel, anchor="nw", width=250)
        canvas.configure(yscrollcommand=vscroll.set)

        canvas.pack(side="left", fill="both", expand=True)
        vscroll.pack(side="right", fill="y")

        r = 0

        # Section: Draw Tools
        self._section_title(panel, r, "DRAW TOOLS")
        r += 1
        tools_frame = tk.Frame(panel, bg=PALETTE["panel"])
        tools_frame.grid(row=r, column=0, sticky="ew", padx=10, pady=(0, 6))
        r += 1

        self._mode_btns = {}
        modes = [
            ("🧱 Wall", "wall", 0, 0),
            ("🟢 Start", "start", 0, 1),
            ("🔴 Goal", "goal", 1, 0),
            ("🗑️ Erase", "erase", 1, 1),
        ]
        for text, val, gr, gc in modes:
            b = tk.Button(tools_frame, text=text,
                          command=lambda m=val: self._set_mode(m),
                          bg=PALETTE["card_bg"], fg=PALETTE["text"],
                          activebackground=PALETTE["primary"], activeforeground="white",
                          relief="flat", bd=0, font=("Segoe UI", 9, "bold"),
                          padx=8, pady=5, cursor="hand2")
            b.grid(row=gr, column=gc, sticky="ew", padx=3, pady=3)
            tools_frame.columnconfigure(gc, weight=1)
            self._mode_btns[val] = b

        # Section: A* Search Settings
        self._section_title(panel, r, "A* SEARCH SETTINGS")
        r += 1
        algo_frame = tk.Frame(panel, bg=PALETTE["panel"])
        algo_frame.grid(row=r, column=0, sticky="ew", padx=10, pady=(0, 6))
        r += 1

        # A* formula banner badge
        badge = tk.Frame(algo_frame, bg=PALETTE["card_bg"], highlightbackground=PALETTE["card_border"],
                         highlightthickness=1)
        badge.pack(fill="x", pady=(0, 6))
        tk.Label(badge, text="★ A* Search: f(n) = g(n) + h(n)", bg=PALETTE["card_bg"],
                 fg=PALETTE["accent_cyan"], font=("Segoe UI", 8, "bold"), pady=4).pack()

        heur_box = tk.Frame(algo_frame, bg=PALETTE["panel"])
        heur_box.pack(fill="x", pady=2)
        tk.Label(heur_box, text="Heuristic:", bg=PALETTE["panel"], fg=PALETTE["text_sub"],
                 font=("Segoe UI", 8)).pack(side="left")
        self.heur_combo = ttk.Combobox(heur_box, textvariable=self.heuristic,
                                       values=["manhattan", "euclidean", "chebyshev"],
                                       state="readonly", font=("Segoe UI", 9), width=12)
        self.heur_combo.pack(side="right")
        self.heur_combo.bind("<<ComboboxSelected>>", lambda e: self._on_heuristic_change())

        # Section: Maze Presets
        self._section_title(panel, r, "MAZE PRESETS")
        r += 1
        preset_frame = tk.Frame(panel, bg=PALETTE["panel"])
        preset_frame.grid(row=r, column=0, sticky="ew", padx=10, pady=(0, 6))
        r += 1

        presets = [
            ("Simple Maze", self._preset_simple),
            ("Random 28%", self._preset_random),
            ("Spiral Lab", self._preset_spiral),
            ("Clear Walls", self._preset_clear_walls),
        ]
        for i, (name, cmd) in enumerate(presets):
            b = tk.Button(preset_frame, text=name, command=cmd,
                          bg=PALETTE["card_bg"], fg=PALETTE["accent_cyan"],
                          activebackground=PALETTE["primary"], activeforeground="white",
                          relief="flat", bd=0, font=("Segoe UI", 8, "bold"),
                          padx=6, pady=4, cursor="hand2")
            b.grid(row=i // 2, column=i % 2, sticky="ew", padx=3, pady=2)
            preset_frame.columnconfigure(i % 2, weight=1)

        # Section: Grid Cell Numbers
        self._section_title(panel, r, "CELL VALUES ON GRID")
        r += 1
        disp_frame = tk.Frame(panel, bg=PALETTE["panel"])
        disp_frame.grid(row=r, column=0, sticky="ew", padx=10, pady=(0, 6))
        r += 1

        for val, label in [
            ("f", "f-score (total: g + h)"),
            ("g", "g-cost (from start: 0, 1, 2…)"),
            ("h", "h-cost (heuristic to goal)"),
            ("fgh", "f, g, h breakdown"),
            ("none", "Hide numbers (clean)"),
        ]:
            rb = tk.Radiobutton(disp_frame, text=label, value=val, variable=self.cell_display,
                                command=self._on_display_mode_change,
                                bg=PALETTE["panel"], fg=PALETTE["text"],
                                selectcolor=PALETTE["card_bg"],
                                activebackground=PALETTE["panel"], activeforeground=PALETTE["text"],
                                font=("Segoe UI", 8))
            rb.pack(anchor="w", pady=1)

        # Section: Animation Speed
        self._section_title(panel, r, "ANIMATION SPEED")
        r += 1
        speed_frame = tk.Frame(panel, bg=PALETTE["panel"])
        speed_frame.grid(row=r, column=0, sticky="ew", padx=10, pady=(0, 6))
        r += 1

        tk.Label(speed_frame, text="Fast", bg=PALETTE["panel"], fg=PALETTE["text_dim"],
                 font=("Segoe UI", 8)).pack(side="left")
        ttk.Scale(speed_frame, from_=200, to=1, variable=self.speed_ms,
                  orient="horizontal").pack(side="left", fill="x", expand=True, padx=6)
        tk.Label(speed_frame, text="Slow", bg=PALETTE["panel"], fg=PALETTE["text_dim"],
                 font=("Segoe UI", 8)).pack(side="left")

        # Section: Execution Controls
        self._section_title(panel, r, "SEARCH CONTROLS")
        r += 1

        self.run_btn = tk.Button(panel, text="▶  Run Search", command=self._run_search,
                                 bg=PALETTE["primary"], fg="white",
                                 activebackground=PALETTE["primary_hover"], activeforeground="white",
                                 relief="flat", bd=0, font=("Segoe UI", 10, "bold"),
                                 pady=7, cursor="hand2")
        self.run_btn.grid(row=r, column=0, sticky="ew", padx=10, pady=3)
        r += 1

        step_pause_box = tk.Frame(panel, bg=PALETTE["panel"])
        step_pause_box.grid(row=r, column=0, sticky="ew", padx=10, pady=2)
        r += 1

        self.step_btn = tk.Button(step_pause_box, text="⏭ Step", command=self._step_once,
                                  bg=PALETTE["card_bg"], fg=PALETTE["accent_cyan"],
                                  activebackground=PALETTE["primary"], activeforeground="white",
                                  relief="flat", bd=0, font=("Segoe UI", 9, "bold"),
                                  pady=6, cursor="hand2")
        self.step_btn.pack(side="left", fill="x", expand=True, padx=(0, 3))

        self.pause_btn = tk.Button(step_pause_box, text="⏸ Pause", command=self._toggle_pause,
                                   bg=PALETTE["card_bg"], fg=PALETTE["text"],
                                   activebackground=PALETTE["primary"], activeforeground="white",
                                   relief="flat", bd=0, font=("Segoe UI", 9, "bold"),
                                   pady=6, state="disabled", cursor="hand2")
        self.pause_btn.pack(side="right", fill="x", expand=True, padx=(3, 0))

        self.reset_search_btn = tk.Button(panel, text="↺  Reset Search", command=self._clear_search,
                                          bg=PALETTE["card_bg"], fg=PALETTE["text_sub"],
                                          activebackground=PALETTE["panel_border"], activeforeground="white",
                                          relief="flat", bd=0, font=("Segoe UI", 9),
                                          pady=5, cursor="hand2")
        self.reset_search_btn.grid(row=r, column=0, sticky="ew", padx=10, pady=2)
        r += 1

        self.clear_all_btn = tk.Button(panel, text="⬛  Clear All", command=self._clear_all,
                                       bg=PALETTE["card_bg"], fg=PALETTE["text_dim"],
                                       activebackground=PALETTE["panel_border"], activeforeground="white",
                                       relief="flat", bd=0, font=("Segoe UI", 9),
                                       pady=5, cursor="hand2")
        self.clear_all_btn.grid(row=r, column=0, sticky="ew", padx=10, pady=2)
        r += 1

        # Section: Grid Size
        self._section_title(panel, r, "GRID DIMENSIONS")
        r += 1
        dim_frame = tk.Frame(panel, bg=PALETTE["panel"])
        dim_frame.grid(row=r, column=0, sticky="ew", padx=10, pady=(0, 10))
        r += 1

        tk.Label(dim_frame, text="Rows:", bg=PALETTE["panel"], fg=PALETTE["text_sub"],
                 font=("Segoe UI", 8)).grid(row=0, column=0, sticky="w")
        self.rows_var = tk.IntVar(value=DEFAULT_ROWS)
        tk.Spinbox(dim_frame, from_=6, to=40, textvariable=self.rows_var, width=4,
                   bg=PALETTE["card_bg"], fg=PALETTE["text"], relief="flat",
                   buttonbackground=PALETTE["card_bg"]).grid(row=0, column=1, padx=4)

        tk.Label(dim_frame, text="Cols:", bg=PALETTE["panel"], fg=PALETTE["text_sub"],
                 font=("Segoe UI", 8)).grid(row=0, column=2, sticky="w", padx=(6, 0))
        self.cols_var = tk.IntVar(value=DEFAULT_COLS)
        tk.Spinbox(dim_frame, from_=8, to=60, textvariable=self.cols_var, width=4,
                   bg=PALETTE["card_bg"], fg=PALETTE["text"], relief="flat",
                   buttonbackground=PALETTE["card_bg"]).grid(row=0, column=3, padx=4)

        tk.Button(dim_frame, text="Resize", command=self._apply_grid_size,
                  bg=PALETTE["card_border"], fg=PALETTE["text"],
                  relief="flat", bd=0, font=("Segoe UI", 8, "bold"),
                  padx=6, pady=2, cursor="hand2").grid(row=0, column=4, padx=(6, 0))

        self._refresh_mode_btns()

    # ──────────────────────────────────────────────────────────────────────────
    # CENTER CANVAS AREA: Interactive Maze Grid
    # ──────────────────────────────────────────────────────────────────────────

    def _build_canvas_area(self) -> None:
        frame = tk.Frame(self.root, bg=PALETTE["bg"])
        frame.grid(row=1, column=1, sticky="nsew", padx=5, pady=8)
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(frame, bg=PALETTE[EMPTY], highlightthickness=1,
                                highlightbackground=PALETTE["panel_border"])
        self.canvas.grid(row=0, column=0, sticky="nsew")

        # Mouse Event Bindings
        self.canvas.bind("<Button-1>",        self._on_click)
        self.canvas.bind("<B1-Motion>",       self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Motion>",          self._on_mouse_move)
        self.canvas.bind("<Leave>",           self._on_mouse_leave)
        self.canvas.bind("<Configure>",       self._on_resize)

    # ──────────────────────────────────────────────────────────────────────────
    # RIGHT PANEL: Live Calculations Lab & Detailed Inspector
    # ──────────────────────────────────────────────────────────────────────────

    def _build_right_panel(self) -> None:
        right_container = tk.Frame(self.root, bg=PALETTE["panel"], width=370,
                                   highlightbackground=PALETTE["panel_border"], highlightthickness=1)
        right_container.grid(row=1, column=2, sticky="nsew", padx=(5, 10), pady=8)
        right_container.grid_propagate(False)

        canvas = tk.Canvas(right_container, bg=PALETTE["panel"], highlightthickness=0, width=354)
        vscroll = ttk.Scrollbar(right_container, orient="vertical", command=canvas.yview)
        panel = tk.Frame(canvas, bg=PALETTE["panel"])

        panel.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=panel, anchor="nw", width=350)
        canvas.configure(yscrollcommand=vscroll.set)

        canvas.pack(side="left", fill="both", expand=True)
        vscroll.pack(side="right", fill="y")

        r = 0

        # Title
        tk.Label(panel, text="🧮  LIVE CALCULATION LAB", bg=PALETTE["panel"],
                 fg=PALETTE["accent_gold"], font=("Segoe UI", 11, "bold")).grid(
            row=r, column=0, sticky="w", padx=12, pady=(12, 4)); r += 1

        # ── Card 1: Active Expanding Node Calculations ──
        calc_card = self._create_card(panel, r, "ACTIVE NODE EVALUATION: f(n) = g(n) + h(n)")
        r += 1

        node_header = tk.Frame(calc_card, bg=PALETTE["card_bg"])
        node_header.pack(fill="x", padx=10, pady=(6, 2))
        tk.Label(node_header, text="Current Node:", bg=PALETTE["card_bg"],
                 fg=PALETTE["text_dim"], font=("Segoe UI", 9)).pack(side="left")
        tk.Label(node_header, textvariable=self.calc_node, bg=PALETTE["card_bg"],
                 fg=PALETTE["accent_rose"], font=("Segoe UI", 10, "bold")).pack(side="left", padx=6)

        # Big formula readout banner
        formula_box = tk.Frame(calc_card, bg="#111827", highlightbackground=PALETTE["card_border"],
                               highlightthickness=1)
        formula_box.pack(fill="x", padx=10, pady=4)
        tk.Label(formula_box, textvariable=self.calc_formula, bg="#111827",
                 fg=PALETTE["accent_cyan"], font=("Consolas", 10, "bold"),
                 pady=5).pack()

        # Detailed g, h, f breakdown table
        grid_vals = tk.Frame(calc_card, bg=PALETTE["card_bg"])
        grid_vals.pack(fill="x", padx=10, pady=4)
        for i in range(3):
            grid_vals.columnconfigure(i, weight=1)

        # g box
        g_box = tk.Frame(grid_vals, bg="#141c2c", highlightbackground=PALETTE["card_border"], highlightthickness=1)
        g_box.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)
        tk.Label(g_box, text="g(n) Cost", bg="#141c2c", fg=PALETTE["text_dim"], font=("Segoe UI", 8)).pack(pady=(2, 0))
        tk.Label(g_box, textvariable=self.calc_g, bg="#141c2c", fg=PALETTE["accent_green"],
                 font=("Consolas", 11, "bold")).pack(pady=(0, 3))

        # h box
        h_box = tk.Frame(grid_vals, bg="#141c2c", highlightbackground=PALETTE["card_border"], highlightthickness=1)
        h_box.grid(row=0, column=1, sticky="nsew", padx=2, pady=2)
        tk.Label(h_box, text="h(n) Heuristic", bg="#141c2c", fg=PALETTE["text_dim"], font=("Segoe UI", 8)).pack(pady=(2, 0))
        tk.Label(h_box, textvariable=self.calc_h, bg="#141c2c", fg=PALETTE["accent_gold"],
                 font=("Consolas", 11, "bold")).pack(pady=(0, 3))

        # f box
        f_box = tk.Frame(grid_vals, bg="#141c2c", highlightbackground=PALETTE["card_border"], highlightthickness=1)
        f_box.grid(row=0, column=2, sticky="nsew", padx=2, pady=2)
        tk.Label(f_box, text="f(n) Total", bg="#141c2c", fg=PALETTE["text_dim"], font=("Segoe UI", 8)).pack(pady=(2, 0))
        tk.Label(f_box, textvariable=self.calc_f, bg="#141c2c", fg=PALETTE["accent_rose"],
                 font=("Consolas", 11, "bold")).pack(pady=(0, 3))

        # Step by step heuristic equation
        tk.Label(calc_card, textvariable=self.calc_h_detail, bg=PALETTE["card_bg"],
                 fg=PALETTE["text_sub"], font=("Segoe UI", 8), wraplength=320,
                 justify="left").pack(anchor="w", padx=10, pady=(2, 8))

        # ── Card 2: 4-Direction Neighbor Evaluation ──
        nbr_card = self._create_card(panel, r, "NEIGHBOR EXPANSION (NORTH, SOUTH, EAST, WEST)")
        r += 1

        self.nbr_rows_container = tk.Frame(nbr_card, bg=PALETTE["card_bg"])
        self.nbr_rows_container.pack(fill="x", padx=8, pady=6)
        self.nbr_labels = []
        for i in range(4):
            nf = tk.Frame(self.nbr_rows_container, bg="#131926", padx=6, pady=3,
                          highlightbackground=PALETTE["card_border"], highlightthickness=1)
            nf.pack(fill="x", pady=2)
            lbl_dir = tk.Label(nf, text=f"Dir {i+1}", bg="#131926", fg=PALETTE["text"],
                               font=("Segoe UI", 8, "bold"), width=12, anchor="w")
            lbl_dir.pack(side="left")
            lbl_stat = tk.Label(nf, text="—", bg="#131926", fg=PALETTE["text_dim"],
                                font=("Segoe UI", 8), width=12, anchor="w")
            lbl_stat.pack(side="left")
            lbl_math = tk.Label(nf, text="—", bg="#131926", fg=PALETTE["accent_cyan"],
                                font=("Consolas", 8), anchor="e")
            lbl_math.pack(side="right", fill="x", expand=True)
            self.nbr_labels.append((lbl_dir, lbl_stat, lbl_math))

        # ── Card 3: Priority Queue (Open Set Min-Heap) ──
        queue_card = self._create_card(panel, r, "OPEN SET PRIORITY QUEUE (TOP CANDIDATES)")
        r += 1

        self.queue_container = tk.Frame(queue_card, bg=PALETTE["card_bg"])
        self.queue_container.pack(fill="x", padx=8, pady=6)
        self.queue_rows = []
        for i in range(5):
            qf = tk.Frame(self.queue_container, bg="#131926", padx=6, pady=2)
            qf.pack(fill="x", pady=1)
            ql_rank = tk.Label(qf, text=f"#{i+1}", bg="#131926", fg=PALETTE["accent_gold"],
                               font=("Segoe UI", 8, "bold"), width=3)
            ql_rank.pack(side="left")
            ql_pos = tk.Label(qf, text="—", bg="#131926", fg=PALETTE["text"],
                              font=("Segoe UI", 8), width=10, anchor="w")
            ql_pos.pack(side="left")
            ql_scores = tk.Label(qf, text="—", bg="#131926", fg=PALETTE["accent_cyan"],
                                 font=("Consolas", 8), anchor="e")
            ql_scores.pack(side="right", fill="x", expand=True)
            self.queue_rows.append((ql_rank, ql_pos, ql_scores))

        # ── Card 4: Hover Cell Inspector ──
        hov_card = self._create_card(panel, r, "LIVE CELL INSPECTOR (MOUSEOVER)")
        r += 1

        h_info = tk.Frame(hov_card, bg=PALETTE["card_bg"])
        h_info.pack(fill="x", padx=8, pady=6)
        h_info.columnconfigure(1, weight=1)
        h_info.columnconfigure(3, weight=1)

        # Row 1: Coords and State
        tk.Label(h_info, text="Pos:", bg=PALETTE["card_bg"], fg=PALETTE["text_dim"],
                 font=("Segoe UI", 8)).grid(row=0, column=0, sticky="w", padx=2, pady=2)
        tk.Label(h_info, textvariable=self.hov_coords, bg=PALETTE["card_bg"], fg=PALETTE["text"],
                 font=("Segoe UI", 8, "bold")).grid(row=0, column=1, sticky="w", padx=2, pady=2)

        tk.Label(h_info, text="State:", bg=PALETTE["card_bg"], fg=PALETTE["text_dim"],
                 font=("Segoe UI", 8)).grid(row=0, column=2, sticky="w", padx=2, pady=2)
        tk.Label(h_info, textvariable=self.hov_type, bg=PALETTE["card_bg"], fg=PALETTE["accent_cyan"],
                 font=("Segoe UI", 8, "bold")).grid(row=0, column=3, sticky="w", padx=2, pady=2)

        # Row 2: g-cost and h-cost
        tk.Label(h_info, text="g(n):", bg=PALETTE["card_bg"], fg=PALETTE["text_dim"],
                 font=("Segoe UI", 8)).grid(row=1, column=0, sticky="w", padx=2, pady=2)
        tk.Label(h_info, textvariable=self.hov_g, bg=PALETTE["card_bg"], fg=PALETTE["accent_green"],
                 font=("Consolas", 8, "bold")).grid(row=1, column=1, sticky="w", padx=2, pady=2)

        tk.Label(h_info, text="h(n):", bg=PALETTE["card_bg"], fg=PALETTE["text_dim"],
                 font=("Segoe UI", 8)).grid(row=1, column=2, sticky="w", padx=2, pady=2)
        tk.Label(h_info, textvariable=self.hov_h, bg=PALETTE["card_bg"], fg=PALETTE["accent_gold"],
                 font=("Consolas", 8, "bold")).grid(row=1, column=3, sticky="w", padx=2, pady=2)

        # Row 3: f-score and Parent
        tk.Label(h_info, text="f(n):", bg=PALETTE["card_bg"], fg=PALETTE["text_dim"],
                 font=("Segoe UI", 8)).grid(row=2, column=0, sticky="w", padx=2, pady=2)
        tk.Label(h_info, textvariable=self.hov_f, bg=PALETTE["card_bg"], fg=PALETTE["accent_rose"],
                 font=("Consolas", 8, "bold")).grid(row=2, column=1, sticky="w", padx=2, pady=2)

        tk.Label(h_info, text="Parent:", bg=PALETTE["card_bg"], fg=PALETTE["text_dim"],
                 font=("Segoe UI", 8)).grid(row=2, column=2, sticky="w", padx=2, pady=2)
        tk.Label(h_info, textvariable=self.hov_parent, bg=PALETTE["card_bg"], fg=PALETTE["text_sub"],
                 font=("Segoe UI", 8)).grid(row=2, column=3, sticky="w", padx=2, pady=2)

        # ── Card 5: Metrics & Statistics ──
        stats_card = self._create_card(panel, r, "SEARCH METRICS")
        r += 1

        s_grid = tk.Frame(stats_card, bg=PALETTE["card_bg"])
        s_grid.pack(fill="x", padx=10, pady=6)
        s_grid.columnconfigure(1, weight=1)
        s_grid.columnconfigure(3, weight=1)

        metrics = [
            ("Explored:", self.stat_nodes, 0, 0),
            ("Frontier:", self.stat_open_size, 0, 2),
            ("Path Len:", self.stat_path_len, 1, 0),
            ("Time:", self.stat_time, 1, 2),
            ("Step Count:", self.stat_step_num, 2, 0),
        ]
        for lbl, var, gr, gc in metrics:
            tk.Label(s_grid, text=lbl, bg=PALETTE["card_bg"], fg=PALETTE["text_dim"],
                     font=("Segoe UI", 8)).grid(row=gr, column=gc, sticky="w", pady=2)
            tk.Label(s_grid, textvariable=var, bg=PALETTE["card_bg"], fg=PALETTE["accent_gold"],
                     font=("Segoe UI", 8, "bold")).grid(row=gr, column=gc + 1, sticky="e", pady=2, padx=(0, 8))

        # ── Card 6: Color Legend ──
        leg_card = self._create_card(panel, r, "COLOR LEGEND")
        r += 1

        leg_grid = tk.Frame(leg_card, bg=PALETTE["card_bg"])
        leg_grid.pack(fill="x", padx=10, pady=6)
        legend_items = [
            (PALETTE[START], "Start Node"),
            (PALETTE[GOAL], "Goal Node"),
            (PALETTE["ACTIVE"], "Current Expanding"),
            (PALETTE[PATH], "Optimal Path"),
            (PALETTE[OPEN], "Open / Frontier"),
            (PALETTE[CLOSED], "Closed / Explored"),
            (PALETTE[WALL], "Obstacle Wall"),
            (PALETTE[EMPTY], "Unvisited Cell"),
        ]
        for i, (col, text) in enumerate(legend_items):
            gr, gc = i // 2, (i % 2) * 2
            swatch = tk.Frame(leg_grid, bg=col, width=12, height=12)
            swatch.grid(row=gr, column=gc, padx=(0, 4), pady=2)
            tk.Label(leg_grid, text=text, bg=PALETTE["card_bg"], fg=PALETTE["text_sub"],
                     font=("Segoe UI", 8)).grid(row=gr, column=gc + 1, sticky="w", pady=2, padx=(0, 8))

    # ── UI Construction Helpers ───────────────────────────────────────────────

    def _section_title(self, parent: tk.Frame, row: int, text: str) -> None:
        sep = tk.Frame(parent, bg=PALETTE["panel_border"], height=1)
        sep.grid(row=row, column=0, sticky="ew", padx=10, pady=(10, 3))
        tk.Label(parent, text=text, bg=PALETTE["panel"], fg=PALETTE["text_dim"],
                 font=("Segoe UI", 7, "bold")).grid(row=row + 1, column=0, sticky="w", padx=12, pady=(0, 4))

    def _create_card(self, parent: tk.Frame, row: int, title: str) -> tk.Frame:
        card = tk.Frame(parent, bg=PALETTE["card_bg"],
                        highlightbackground=PALETTE["card_border"], highlightthickness=1)
        card.grid(row=row, column=0, sticky="ew", padx=10, pady=5)
        title_bar = tk.Frame(card, bg="#131926", height=24)
        title_bar.pack(fill="x")
        title_bar.pack_propagate(False)
        tk.Label(title_bar, text=title, bg="#131926", fg=PALETTE["accent_cyan"],
                 font=("Segoe UI", 8, "bold")).pack(side="left", padx=8, pady=2)
        return card

    def _set_mode(self, mode: str) -> None:
        self.mode.set(mode)
        self._refresh_mode_btns()

    def _refresh_mode_btns(self) -> None:
        current = self.mode.get()
        for mode, btn in self._mode_btns.items():
            if mode == current:
                btn.configure(bg=PALETTE["primary"], fg="white")
            else:
                btn.configure(bg=PALETTE["card_bg"], fg=PALETTE["text"])

    def _on_heuristic_change(self) -> None:
        if self.is_running:
            self._toggle_pause()
        self._clear_search()
        heur = self.heuristic.get()
        self.calc_formula.set(f"f(n) = g(n) + h(n)  [{heur.capitalize()}]")

    def _on_display_mode_change(self) -> None:
        self._draw_grid()

    # ═══════════════════════════════════════════════════════════════════════════
    # CANVAS & GRID RENDERING
    # ═══════════════════════════════════════════════════════════════════════════

    def _cell_size(self) -> int:
        cw = self.canvas.winfo_width() or 600
        ch = self.canvas.winfo_height() or 500
        sz = min(cw // self.grid.cols, ch // self.grid.rows)
        return max(MIN_CELL_PX, min(MAX_CELL_PX, sz))

    def _cell_rect(self, row: int, col: int, sz: int) -> tuple[int, int, int, int]:
        x0 = col * sz
        y0 = row * sz
        return x0, y0, x0 + sz, y0 + sz

    def _draw_grid(self) -> None:
        """Complete redraw of all grid cells and numbers."""
        self.canvas.delete("all")
        sz = self._cell_size()
        self._rect_ids.clear()
        self._text_ids.clear()
        self._hover_rect_id = None

        disp_mode = self.cell_display.get()

        for r in range(self.grid.rows):
            for c in range(self.grid.cols):
                x0, y0, x1, y1 = self._cell_rect(r, c, sz)
                st = self.grid.cells[r][c]
                col = PALETTE.get(st, PALETTE[EMPTY])

                # Current active expanding node highlight
                if self._active_node == (r, c):
                    col = PALETTE["ACTIVE"]

                rid = self.canvas.create_rectangle(
                    x0, y0, x1, y1,
                    fill=col, outline=PALETTE["grid_line"], width=1,
                    tags=(f"c_{r}_{c}", "cell")
                )
                self._rect_ids[(r, c)] = rid

                # Cell Icons for Start & Goal
                if (r, c) == self.grid.start and sz >= 14:
                    self.canvas.create_text(
                        (x0 + x1) / 2, (y0 + y1) / 2,
                        text="S", fill="white",
                        font=("Segoe UI", max(8, sz // 3), "bold"),
                        tags=f"icon_{r}_{c}"
                    )
                elif (r, c) == self.grid.goal and sz >= 14:
                    self.canvas.create_text(
                        (x0 + x1) / 2, (y0 + y1) / 2,
                        text="G", fill="white",
                        font=("Segoe UI", max(8, sz // 3), "bold"),
                        tags=f"icon_{r}_{c}"
                    )
                # Cell Numbers (f, g, h) if enabled and state has score
                elif disp_mode != "none" and sz >= 16:
                    self._draw_cell_numbers(r, c, x0, y0, x1, y1, sz, disp_mode)

    def _draw_cell_numbers(self, r: int, c: int, x0: int, y0: int, x1: int, y1: int,
                           sz: int, disp_mode: str) -> None:
        """Draws f, g, h numbers inside a cell."""
        if (r, c) not in self._f_scores:
            return
        f_val = self._f_scores[(r, c)]
        g_val = self._g_scores.get((r, c), 0)
        h_val = self._h_scores.get((r, c), 0)

        f_str = f"{int(f_val)}" if isinstance(f_val, (int, float)) and f_val == int(f_val) else f"{f_val}"
        g_str = f"{int(g_val)}" if isinstance(g_val, (int, float)) and g_val == int(g_val) else f"{g_val}"
        h_str = f"{int(h_val)}" if isinstance(h_val, (int, float)) and h_val == int(h_val) else f"{h_val}"

        if disp_mode == "g":
            font_main = ("Segoe UI", max(7, sz // 3), "bold")
            tid = self.canvas.create_text((x0 + x1) / 2, (y0 + y1) / 2,
                                          text=g_str, fill=PALETTE["accent_cyan"], font=font_main, tags=f"txt_{r}_{c}")
            self._text_ids[(r, c)] = tid
        elif disp_mode == "h":
            font_main = ("Segoe UI", max(7, sz // 3), "bold")
            tid = self.canvas.create_text((x0 + x1) / 2, (y0 + y1) / 2,
                                          text=h_str, fill=PALETTE["accent_gold"], font=font_main, tags=f"txt_{r}_{c}")
            self._text_ids[(r, c)] = tid
        elif disp_mode == "fgh" and sz >= 26:
            font_main = ("Segoe UI", max(7, sz // 4), "bold")
            font_sub = ("Segoe UI", max(6, sz // 5))
            tid_f = self.canvas.create_text((x0 + x1) / 2, y0 + sz * 0.35,
                                            text=f_str, fill="white", font=font_main, tags=f"txt_{r}_{c}")
            tid_g = self.canvas.create_text(x0 + sz * 0.28, y0 + sz * 0.75,
                                            text=f"g{g_str}", fill=PALETTE["accent_cyan"], font=font_sub, tags=f"txt_{r}_{c}")
            tid_h = self.canvas.create_text(x1 - sz * 0.28, y0 + sz * 0.75,
                                            text=f"h{h_str}", fill=PALETTE["accent_gold"], font=font_sub, tags=f"txt_{r}_{c}")
            self._text_ids[(r, c)] = tid_f
        elif disp_mode in ("f", "fgh"):
            font_main = ("Segoe UI", max(7, sz // 3), "bold")
            tid = self.canvas.create_text((x0 + x1) / 2, (y0 + y1) / 2,
                                          text=f_str, fill="white", font=font_main, tags=f"txt_{r}_{c}")
            self._text_ids[(r, c)] = tid

    def _update_cell(self, row: int, col: int) -> None:
        """Update color and text for a single cell without redrawing everything."""
        rid = self._rect_ids.get((row, col))
        if rid is None:
            return

        st = self.grid.cells[row][col]
        col_color = PALETTE.get(st, PALETTE[EMPTY])
        if self._active_node == (row, col):
            col_color = PALETTE["ACTIVE"]

        self.canvas.itemconfigure(rid, fill=col_color)

        # Update cell number text if enabled
        disp_mode = self.cell_display.get()
        sz = self._cell_size()
        if disp_mode != "none" and sz >= 16 and (row, col) not in (self.grid.start, self.grid.goal):
            self.canvas.delete(f"txt_{row}_{col}")
            x0, y0, x1, y1 = self._cell_rect(row, col, sz)
            self._draw_cell_numbers(row, col, x0, y0, x1, y1, sz, disp_mode)

    def _on_resize(self, _event=None) -> None:
        self._draw_grid()

    # ═══════════════════════════════════════════════════════════════════════════
    # MOUSE INTERACTION & HOVER INSPECTION
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
        r, c = cell
        m = self.mode.get()

        if m == "start":
            self.grid.set_start(r, c)
            self._clear_search()
        elif m == "goal":
            self.grid.set_goal(r, c)
            self._clear_search()
        elif m == "wall":
            self.grid.set_wall(r, c)
            self._update_cell(r, c)
        elif m == "erase":
            if (r, c) not in (self.grid.start, self.grid.goal):
                self.grid.cells[r][c] = EMPTY
                self._update_cell(r, c)
        self._last_drag = (r, c)
        self._inspect_cell(r, c)

    def _on_drag(self, event: tk.Event) -> None:
        if self.is_running:
            return
        cell = self._canvas_to_cell(event.x, event.y)
        if cell is None or cell == self._last_drag:
            return
        r, c = cell
        m = self.mode.get()
        if m == "wall":
            self.grid.set_wall(r, c)
            self._update_cell(r, c)
        elif m == "erase":
            if (r, c) not in (self.grid.start, self.grid.goal):
                self.grid.cells[r][c] = EMPTY
                self._update_cell(r, c)
        self._last_drag = (r, c)
        self._inspect_cell(r, c)

    def _on_release(self, _event: tk.Event) -> None:
        self._last_drag = (-1, -1)

    def _on_mouse_move(self, event: tk.Event) -> None:
        """Handles live hover crosshair and updates live cell inspector."""
        cell = self._canvas_to_cell(event.x, event.y)
        if cell is None:
            self._on_mouse_leave()
            return

        r, c = cell
        if self._hover_node == (r, c):
            return
        self._hover_node = (r, c)

        sz = self._cell_size()
        x0, y0, x1, y1 = self._cell_rect(r, c, sz)
        if self._hover_rect_id is None:
            self._hover_rect_id = self.canvas.create_rectangle(
                x0, y0, x1, y1,
                outline=PALETTE["HOVER"], width=2, tags="hover_box"
            )
        else:
            self.canvas.coords(self._hover_rect_id, x0, y0, x1, y1)
            self.canvas.tag_raise(self._hover_rect_id)

        self._inspect_cell(r, c)

    def _on_mouse_leave(self, _event=None) -> None:
        self._hover_node = None
        if self._hover_rect_id is not None:
            self.canvas.delete(self._hover_rect_id)
            self._hover_rect_id = None

    def _inspect_cell(self, r: int, c: int) -> None:
        """Populates the Live Cell Inspector Card with detailed data for (r, c)."""
        self.hov_coords.set(f"Row {r}, Col {c}")
        st = self.grid.cells[r][c]
        type_names = {
            EMPTY: "Empty / Unvisited",
            WALL: "Obstacle Wall",
            START: "Start Node (🟢)",
            GOAL: "Goal Node (🔴)",
            OPEN: "Frontier (Open)",
            CLOSED: "Explored (Closed)",
            PATH: "Optimal Path (★)",
        }
        self.hov_type.set(type_names.get(st, "Unknown"))

        if (r, c) in self._g_scores:
            g_val = self._g_scores[(r, c)]
            self.hov_g.set(str(int(g_val) if g_val == int(g_val) else g_val))
        else:
            self.hov_g.set("—")

        if (r, c) in self._h_scores:
            h_val = self._h_scores[(r, c)]
            self.hov_h.set(str(int(h_val) if h_val == int(h_val) else h_val))
        elif self.grid.goal is not None:
            h_func = HEURISTIC_MAP.get(self.heuristic.get(), manhattan)
            h_val = h_func((r, c), self.grid.goal)
            self.hov_h.set(f"~{h_val}")
        else:
            self.hov_h.set("—")

        if (r, c) in self._f_scores:
            f_val = self._f_scores[(r, c)]
            self.hov_f.set(str(int(f_val) if f_val == int(f_val) else f_val))
        else:
            self.hov_f.set("—")

        if (r, c) in self._came_from:
            pr, pc = self._came_from[(r, c)]
            self.hov_parent.set(f"({pr}, {pc})")
        else:
            self.hov_parent.set("None (Start/Root)")

    # ═══════════════════════════════════════════════════════════════════════════
    # SEARCH EXECUTION & LIVE CALCULATION UPDATES
    # ═══════════════════════════════════════════════════════════════════════════

    def _run_search(self) -> None:
        """Starts continuous automated search animation."""
        if not self._check_endpoints():
            return

        if not self.is_running:
            self._start_search_engine()

        self.is_running = True
        self.is_paused = False
        self.stat_status.set("Searching…")
        self.status_badge.configure(fg=PALETTE["accent_cyan"])
        self.run_btn.configure(state="disabled")
        self.pause_btn.configure(state="normal", text="⏸  Pause")
        self._step()

    def _step_once(self) -> None:
        """Executes exactly one step of the search and pauses for calculation inspection."""
        if not self._check_endpoints():
            return

        if not self.is_running and self._search_gen is None:
            self._start_search_engine()
            self.is_running = True
            self.is_paused = True
            self.stat_status.set("Paused / Stepping")
            self.status_badge.configure(fg=PALETTE["accent_gold"])
            self.run_btn.configure(state="normal", text="▶  Resume")
            self.pause_btn.configure(state="normal", text="▶  Resume")

        self._execute_single_step()

    def _check_endpoints(self) -> bool:
        if self.grid.start is None or self.grid.goal is None:
            messagebox.showwarning(
                "Missing Endpoints",
                "Please define both a Start (🟢) and a Goal (🔴) cell before running."
            )
            return False
        return True

    def _start_search_engine(self) -> None:
        """Initializes A* generator and clear previous paths."""
        self._clear_search()
        heur = self.heuristic.get()
        self._search_gen = astar(self.grid, heuristic_name=heur)

    def _step(self) -> None:
        """Timer callback for continuous search."""
        if not self.is_running or self.is_paused:
            return

        has_more = self._execute_single_step()
        if has_more:
            self._after_id = self.root.after(self.speed_ms.get(), self._step)

    def _execute_single_step(self) -> bool:
        """Pulls next update from generator and renders all calculation breakdowns."""
        if self._search_gen is None:
            return False

        try:
            update = next(self._search_gen)
        except StopIteration:
            self._finish(found=False, stats=None)
            return False

        # Cache calculation dicts for hover and grid numbers
        self._g_scores = update.get("g_scores", self._g_scores)
        self._h_scores = update.get("h_scores", self._h_scores)
        self._f_scores = update.get("f_scores", self._f_scores)
        self._came_from = update.get("came_from", self._came_from)

        prev_active = self._active_node
        self._active_node = update.get("current")

        # Repaint previous active cell and new active cell
        if prev_active and prev_active not in (self.grid.start, self.grid.goal):
            self._update_cell(*prev_active)
        if self._active_node and self._active_node not in (self.grid.start, self.grid.goal):
            self._update_cell(*self._active_node)

        # Repaint frontier and explored cells
        for r, c in update.get("explored", []):
            if (r, c) not in (self.grid.start, self.grid.goal):
                self.grid.cells[r][c] = CLOSED
                self._update_cell(r, c)

        for r, c in update.get("frontier", []):
            if (r, c) not in (self.grid.start, self.grid.goal) and self.grid.cells[r][c] != CLOSED:
                self.grid.cells[r][c] = OPEN
                self._update_cell(r, c)

        # ── Update Calculations Dashboard Cards ──────────────────────────────
        calc = update.get("current_calc")
        if calc:
            cp = calc["pos"]
            self.calc_node.set(f"({cp[0]}, {cp[1]})")
            self.calc_formula.set(calc["f_formula"])
            self.calc_g.set(str(calc["g"]))
            self.calc_h.set(str(calc["h"]))
            self.calc_f.set(str(calc["f"]))
            self.calc_h_detail.set(f"Heuristic breakdown: {calc['h_formula']}")

        # Update Neighbor Expansion Card
        nbrs = update.get("neighbors", [])
        for i in range(4):
            lbl_dir, lbl_stat, lbl_math = self.nbr_labels[i]
            if i < len(nbrs):
                n_info = nbrs[i]
                lbl_dir.configure(text=n_info["dir"])
                st_color = {
                    "Added to Open": PALETTE["accent_green"],
                    "Updated Cost": PALETTE["accent_cyan"],
                    "Closed": PALETTE["text_dim"],
                    "Wall": PALETTE["text_sub"],
                    "Skipped": PALETTE["accent_gold"],
                }.get(n_info["status"], PALETTE["text"])
                lbl_stat.configure(text=n_info["status"], fg=st_color)
                lbl_math.configure(text=n_info["detail"])
            else:
                lbl_dir.configure(text="—")
                lbl_stat.configure(text="—", fg=PALETTE["text_dim"])
                lbl_math.configure(text="—")

        # Update Priority Queue Card
        queue = update.get("open_queue", [])
        for i in range(5):
            ql_rank, ql_pos, ql_scores = self.queue_rows[i]
            if i < len(queue):
                item = queue[i]
                pos = item["pos"]
                ql_pos.configure(text=f"({pos[0]},{pos[1]})")
                ql_scores.configure(text=f"f={item['f']} [g={item['g']}, h={item['h']}]")
            else:
                ql_pos.configure(text="—")
                ql_scores.configure(text="—")

        # Update Search Metrics
        stats = update.get("stats")
        if stats:
            self.stat_nodes.set(str(stats.get("nodes_explored", 0)))
            self.stat_open_size.set(str(stats.get("open_size", 0)))
            self.stat_step_num.set(str(stats.get("step", 0)))
            self.stat_time.set(f"{stats.get('elapsed_ms', 0)} ms")

        # On Completion
        if update.get("done"):
            if update["found"]:
                self._paint_path(update["path"])
            self._finish(found=update["found"], stats=update.get("stats"))
            return False

        return True

    def _paint_path(self, path: list) -> None:
        """Paints the golden solution path cells."""
        for r, c in path:
            if (r, c) not in (self.grid.start, self.grid.goal):
                self.grid.cells[r][c] = PATH
                self._update_cell(r, c)

    def _finish(self, found: bool, stats: dict | None) -> None:
        """Search completion handler."""
        self.is_running = False
        self.run_btn.configure(state="normal", text="▶  Run Search")
        self.pause_btn.configure(state="disabled", text="⏸  Pause")
        self._active_node = None

        if found and stats:
            self.stat_nodes.set(str(stats["nodes_explored"]))
            self.stat_path_len.set(f"{stats['path_length']} steps")
            self.stat_time.set(f"{stats['elapsed_ms']} ms")
            self.stat_status.set("✅ Optimal Path Found!")
            self.status_badge.configure(fg=PALETTE["accent_green"])
        else:
            self.stat_path_len.set("None")
            self.stat_status.set("❌ No Path Exists")
            self.status_badge.configure(fg=PALETTE["accent_rose"])

    def _toggle_pause(self) -> None:
        if not self.is_running and self._search_gen is None:
            return
        self.is_paused = not self.is_paused
        if self.is_paused:
            self.pause_btn.configure(text="▶  Resume")
            self.run_btn.configure(state="normal", text="▶  Resume")
            self.stat_status.set("⏸ Search Paused")
            self.status_badge.configure(fg=PALETTE["accent_gold"])
            if self._after_id:
                self.root.after_cancel(self._after_id)
        else:
            self.pause_btn.configure(text="⏸  Pause")
            self.run_btn.configure(state="disabled", text="▶  Run Search")
            self.stat_status.set("Searching…")
            self.status_badge.configure(fg=PALETTE["accent_cyan"])
            self._step()

    # ── Clear & Reset ─────────────────────────────────────────────────────────

    def _clear_search(self) -> None:
        """Clears search paths and calculations, keeping maze walls intact."""
        if self._after_id:
            self.root.after_cancel(self._after_id)
        self.is_running = False
        self.is_paused = False
        self._search_gen = None
        self._active_node = None

        self._g_scores.clear()
        self._h_scores.clear()
        self._f_scores.clear()
        self._came_from.clear()

        self.run_btn.configure(state="normal", text="▶  Run Search")
        self.pause_btn.configure(state="disabled", text="⏸  Pause")
        self.grid.clear_search()

        self.stat_nodes.set("0")
        self.stat_open_size.set("0")
        self.stat_path_len.set("—")
        self.stat_time.set("—")
        self.stat_step_num.set("0")
        self.stat_status.set("Ready")
        self.status_badge.configure(fg=PALETTE["accent_cyan"])

        self.calc_node.set("None")
        self.calc_g.set("—")
        self.calc_h.set("—")
        self.calc_f.set("—")
        self.calc_h_detail.set("Run or step the search to inspect live formulas")

        self._draw_grid()

    def _clear_all(self) -> None:
        """Completely resets grid and clears all walls."""
        self._clear_search()
        self.grid.clear_all()
        self.grid.set_start(2, 2)
        self.grid.set_goal(self.grid.rows - 3, self.grid.cols - 3)
        self._draw_grid()

    # ── Maze Presets ──────────────────────────────────────────────────────────

    def _preset_simple(self) -> None:
        self._clear_search()
        self.grid.generate_simple_maze()
        self._draw_grid()

    def _preset_random(self) -> None:
        self._clear_search()
        self.grid.generate_random_walls(density=0.28)
        self._draw_grid()

    def _preset_spiral(self) -> None:
        self._clear_search()
        self.grid.generate_spiral()
        self._draw_grid()

    def _preset_clear_walls(self) -> None:
        self._clear_search()
        self.grid.clear_walls()
        self._draw_grid()

    # ── Grid Resizing ─────────────────────────────────────────────────────────

    def _apply_grid_size(self) -> None:
        self._clear_search()
        rows = max(6, min(40, self.rows_var.get()))
        cols = max(8, min(60, self.cols_var.get()))
        self.grid.resize(rows, cols)
        self.grid.set_start(2, 2)
        self.grid.set_goal(rows - 3, cols - 3)
        self._draw_grid()


# ── Entry Point ───────────────────────────────────────────────────────────────

def main() -> None:
    root = tk.Tk()

    try:
        default_font = font.nametofont("TkDefaultFont")
        default_font.configure(family="Segoe UI", size=9)
    except Exception:
        pass

    app = MazeSolverApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()

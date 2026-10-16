/**
 * ============================================================================
 * A* Pathfinding Visualizer — Core Algorithm & Application Logic
 * Min-Heap Priority Queue, Animated Open/Closed Sets, Live Math & Explanations
 * ============================================================================
 */

// Cell State Constants
const EMPTY  = 0;
const WALL   = 1;
const START  = 2;
const GOAL   = 3;
const OPEN   = 4; // Frontier / Priority Queue
const CLOSED = 5; // Visited / Explored
const PATH   = 6; // Optimal shortest path

// ── Min-Heap Priority Queue ──────────────────────────────────────────────────
class MinHeap {
  constructor() {
    this.heap = [];
  }

  size() {
    return this.heap.length;
  }

  isEmpty() {
    return this.heap.length === 0;
  }

  push(node) {
    this.heap.push(node);
    this._bubbleUp(this.heap.length - 1);
  }

  pop() {
    if (this.isEmpty()) return null;
    const min = this.heap[0];
    const last = this.heap.pop();
    if (this.heap.length > 0) {
      this.heap[0] = last;
      this._sinkDown(0);
    }
    return min;
  }

  toArray() {
    return [...this.heap].sort((a, b) => this._compare(a, b));
  }

  // Primary: Lowest f(n). Secondary tie-breaker: Lowest h(n) (closer to goal)
  _compare(a, b) {
    if (a.f !== b.f) return a.f - b.f;
    return a.h - b.h;
  }

  _bubbleUp(index) {
    while (index > 0) {
      const parentIndex = Math.floor((index - 1) / 2);
      if (this._compare(this.heap[index], this.heap[parentIndex]) >= 0) break;
      [this.heap[index], this.heap[parentIndex]] = [this.heap[parentIndex], this.heap[index]];
      index = parentIndex;
    }
  }

  _sinkDown(index) {
    const length = this.heap.length;
    while (true) {
      const left = 2 * index + 1;
      const right = 2 * index + 2;
      let smallest = index;

      if (left < length && this._compare(this.heap[left], this.heap[smallest]) < 0) {
        smallest = left;
      }
      if (right < length && this._compare(this.heap[right], this.heap[smallest]) < 0) {
        smallest = right;
      }
      if (smallest === index) break;

      [this.heap[index], this.heap[smallest]] = [this.heap[smallest], this.heap[index]];
      index = smallest;
    }
  }
}

// Manhattan Distance Heuristic (|r1 - r2| + |c1 - c2|)
function manhattanDistance(r1, c1, r2, c2) {
  return Math.abs(r1 - r2) + Math.abs(c1 - c2);
}

// ── Application Controller ───────────────────────────────────────────────────
class AStarVisualizer {
  constructor() {
    this.rows = 18;
    this.cols = 26;
    this.grid = [];
    this.start = [2, 2];
    this.goal = [this.rows - 3, this.cols - 3];

    // Search state
    this.gScores = new Map();
    this.hScores = new Map();
    this.fScores = new Map();
    this.cameFrom = new Map();
    this.finalPath = [];
    this.activeNode = null;
    this.iterations = 0;
    this.stepIndex = 0;

    // Execution state
    this.generator = null;
    this.isRunning = false;
    this.timerId = null;
    this.delayMs = 25;

    // Interaction state
    this.activeTool = 'wall';
    this.overlayType = 'f';
    this.isMouseDown = false;

    // Canvas setup
    this.canvas = document.getElementById('grid-canvas');
    this.ctx = this.canvas.getContext('2d');
    this.cellSize = 32;

    this._bindElements();
    this._initGrid();
    this._setupEventListeners();
    this._resizeCanvas();
    this._render();
  }

  _bindElements() {
    this.el = {
      btnPlay: document.getElementById('btn-play'),
      playBtnText: document.getElementById('play-btn-text'),
      btnStep: document.getElementById('btn-step'),
      btnResetSearch: document.getElementById('btn-reset-search'),
      btnClearWalls: document.getElementById('btn-clear-walls'),
      presetSelect: document.getElementById('preset-select'),
      overlaySelect: document.getElementById('overlay-select'),
      speedSlider: document.getElementById('speed-slider'),
      speedDisplay: document.getElementById('speed-display'),

      // Statistics
      statStatus: document.getElementById('stat-status'),
      statPathLen: document.getElementById('stat-path-len'),
      statExplored: document.getElementById('stat-explored'),
      statFrontier: document.getElementById('stat-frontier'),
      statIterations: document.getElementById('stat-iterations'),

      // Live Math
      currentCoord: document.getElementById('current-coord'),
      currentEval: document.getElementById('current-eval'),
      subG: document.getElementById('sub-g'),
      subH: document.getElementById('sub-h'),
      subF: document.getElementById('sub-f'),
      hCalcDetail: document.getElementById('h-calc-detail'),

      // Neighbors & PQ
      neighborEvalBody: document.getElementById('neighbor-eval-body'),
      pqLeaderboard: document.getElementById('pq-leaderboard'),
      pqCountBadge: document.getElementById('pq-count-badge'),

      // Explanation Log
      explanationLog: document.getElementById('explanation-log'),
      btnClearLog: document.getElementById('btn-clear-log'),

      // Tools
      tools: document.querySelectorAll('.tool-btn')
    };
  }

  _initGrid() {
    this.grid = Array.from({ length: this.rows }, () => new Array(this.cols).fill(EMPTY));
    this.grid[this.start[0]][this.start[1]] = START;
    this.grid[this.goal[0]][this.goal[1]] = GOAL;
    this._resetCaches();
  }

  _resetCaches() {
    this.gScores.clear();
    this.hScores.clear();
    this.fScores.clear();
    this.cameFrom.clear();
    this.finalPath = [];
    this.activeNode = null;
    this.iterations = 0;
    this.stepIndex = 0;
  }

  // ── Core A* Generator ──────────────────────────────────────────────────────
  *aStarGenerator() {
    const startKey = `${this.start[0]},${this.start[1]}`;
    const goalKey = `${this.goal[0]},${this.goal[1]}`;

    const heap = new MinHeap();
    const closedSet = new Set();
    const openSet = new Set([startKey]);

    const gScore = new Map();
    const hScore = new Map();
    const fScore = new Map();
    const cameFrom = new Map();

    const startH = manhattanDistance(this.start[0], this.start[1], this.goal[0], this.goal[1]);
    gScore.set(startKey, 0);
    hScore.set(startKey, startH);
    fScore.set(startKey, startH);

    heap.push({
      r: this.start[0],
      c: this.start[1],
      g: 0,
      h: startH,
      f: startH
    });

    this.gScores = gScore;
    this.hScores = hScore;
    this.fScores = fScore;
    this.cameFrom = cameFrom;

    const directions = [
      { name: 'Up',    dr: -1, dc: 0 },
      { name: 'Down',  dr: 1,  dc: 0 },
      { name: 'Left',  dr: 0,  dc: -1 },
      { name: 'Right', dr: 0,  dc: 1 }
    ];

    while (!heap.isEmpty()) {
      const current = heap.pop();
      const currentKey = `${current.r},${current.c}`;

      // Lazy deletion check: skip closed nodes or stale higher-cost paths
      if (closedSet.has(currentKey)) continue;
      if (current.g > gScore.get(currentKey)) continue;

      this.iterations++;
      this.stepIndex++;
      openSet.delete(currentKey);
      closedSet.add(currentKey);
      this.activeNode = [current.r, current.c];

      // Mark cell as closed on visual grid (except start/goal)
      if (currentKey !== startKey && currentKey !== goalKey) {
        this.grid[current.r][current.c] = CLOSED;
      }

      // Check if goal reached upon popping from priority queue
      if (current.r === this.goal[0] && current.c === this.goal[1]) {
        const path = [];
        let curr = goalKey;
        while (cameFrom.has(curr)) {
          const [pr, pc] = curr.split(',').map(Number);
          path.push([pr, pc]);
          curr = cameFrom.get(curr);
        }
        path.push([this.start[0], this.start[1]]);
        path.reverse();

        for (let i = 1; i < path.length - 1; i++) {
          this.grid[path[i][0]][path[i][1]] = PATH;
        }
        this.finalPath = path;

        yield {
          type: 'found',
          step: this.stepIndex,
          current,
          path,
          explored: closedSet.size,
          frontier: heap.size(),
          iterations: this.iterations,
          heapSnapshot: heap.toArray(),
          neighbors: []
        };
        return;
      }

      // Expand 4 orthogonal neighbors
      const neighborEvaluations = [];
      for (const dir of directions) {
        const nr = current.r + dir.dr;
        const nc = current.c + dir.dc;
        const neighborKey = `${nr},${nc}`;

        if (nr < 0 || nr >= this.rows || nc < 0 || nc >= this.cols) continue;

        if (this.grid[nr][nc] === WALL) {
          neighborEvaluations.push({
            dir: dir.name,
            r: nr, c: nc,
            g: '—', h: '—', f: '—',
            status: 'tag-wall',
            label: 'Wall'
          });
          continue;
        }

        if (closedSet.has(neighborKey)) {
          const prevF = fScore.get(neighborKey);
          neighborEvaluations.push({
            dir: dir.name,
            r: nr, c: nc,
            g: gScore.get(neighborKey),
            h: hScore.get(neighborKey),
            f: prevF,
            status: 'tag-closed',
            label: 'Closed'
          });
          continue;
        }

        const tentativeG = current.g + 1;
        const existingG = gScore.has(neighborKey) ? gScore.get(neighborKey) : Infinity;
        const isGoal = (nr === this.goal[0] && nc === this.goal[1]);

        if (tentativeG < existingG) {
          const h = manhattanDistance(nr, nc, this.goal[0], this.goal[1]);
          const f = tentativeG + h;

          cameFrom.set(neighborKey, currentKey);
          gScore.set(neighborKey, tentativeG);
          hScore.set(neighborKey, h);
          fScore.set(neighborKey, f);

          openSet.add(neighborKey);
          heap.push({ r: nr, c: nc, g: tentativeG, h, f });

          if (!isGoal && (nr !== this.start[0] || nc !== this.start[1])) {
            this.grid[nr][nc] = OPEN;
          }

          neighborEvaluations.push({
            dir: dir.name,
            r: nr, c: nc,
            g: tentativeG, h, f,
            status: isGoal ? 'tag-goal' : 'tag-added',
            label: isGoal ? 'Goal Node' : 'Added to Open'
          });
        } else {
          neighborEvaluations.push({
            dir: dir.name,
            r: nr, c: nc,
            g: tentativeG,
            h: manhattanDistance(nr, nc, this.goal[0], this.goal[1]),
            f: tentativeG + manhattanDistance(nr, nc, this.goal[0], this.goal[1]),
            status: 'tag-worse',
            label: 'Worse Cost'
          });
        }
      }

      yield {
        type: 'step',
        step: this.stepIndex,
        current,
        explored: closedSet.size,
        frontier: openSet.size,
        iterations: this.iterations,
        heapSnapshot: heap.toArray(),
        neighbors: neighborEvaluations
      };
    }

    // Open set exhausted without finding Goal
    yield {
      type: 'not_found',
      step: this.stepIndex,
      explored: closedSet.size,
      frontier: 0,
      iterations: this.iterations,
      heapSnapshot: [],
      neighbors: []
    };
  }

  // ── Playback Controls ──────────────────────────────────────────────────────
  toggleRun() {
    if (this.isRunning) {
      this.pause();
    } else {
      this.run();
    }
  }

  run() {
    if (this.isRunning) return;

    if (!this.generator) {
      this.generator = this.aStarGenerator();
    }

    this.isRunning = true;
    this.el.btnPlay.classList.add('paused');
    this.el.playBtnText.textContent = 'Pause';
    this.el.statStatus.textContent = 'Searching...';
    this.el.statStatus.className = 'status-badge status-running';

    const loop = () => {
      if (!this.isRunning) return;
      const res = this.generator.next();

      if (res.done || !res.value) {
        this.pause();
        return;
      }

      this._handleStepResult(res.value);

      if (res.value.type === 'found' || res.value.type === 'not_found') {
        this.pause();
        return;
      }

      this.timerId = setTimeout(loop, this.delayMs);
    };

    loop();
  }

  pause() {
    this.isRunning = false;
    if (this.timerId) {
      clearTimeout(this.timerId);
      this.timerId = null;
    }
    this.el.btnPlay.classList.remove('paused');
    this.el.playBtnText.textContent = 'Visualize A*';

    if (this.finalPath.length > 0) {
      this.el.statStatus.textContent = 'Path Found!';
      this.el.statStatus.className = 'status-badge status-found';
    } else if (this.activeNode) {
      this.el.statStatus.textContent = 'Paused';
      this.el.statStatus.className = 'status-badge status-ready';
    }
  }

  step() {
    this.pause();
    if (!this.generator) {
      this.generator = this.aStarGenerator();
      this.el.statStatus.textContent = 'Stepping...';
      this.el.statStatus.className = 'status-badge status-running';
    }

    const res = this.generator.next();
    if (res.done || !res.value) {
      this.generator = null;
      return;
    }

    this._handleStepResult(res.value);
  }

  _handleStepResult(data) {
    this.el.statExplored.textContent = data.explored;
    this.el.statFrontier.textContent = data.frontier;
    this.el.statIterations.textContent = data.iterations;

    if (data.type === 'step') {
      this._updateMathLab(data.current, data.neighbors, data.heapSnapshot, false, data.step);
    } else if (data.type === 'found') {
      this.el.statStatus.textContent = 'Goal Reached!';
      this.el.statStatus.className = 'status-badge status-found';
      this.el.statPathLen.textContent = `${data.path.length - 1} steps (Cost: ${data.path.length - 1})`;
      this._updateMathLab(data.current, [], data.heapSnapshot, true, data.step);
      this._appendLog(`[Step ${data.step}] GOAL REACHED! Optimal path found: ${data.path.length - 1} steps. Total explored: ${data.explored} nodes.`, 'log-found');
    } else if (data.type === 'not_found') {
      this.el.statStatus.textContent = 'No Path Possible';
      this.el.statStatus.className = 'status-badge status-failed';
      this.el.statPathLen.textContent = 'Unreachable';
      this._appendLog(`[Search Terminated] Open set exhausted. No valid path exists from Start to Goal.`, 'log-failed');
      this.el.neighborEvalBody.innerHTML = `<tr><td colspan="4" style="color: #ef4444; text-align: center;">Frontier exhausted. No path found.</td></tr>`;
    }

    this._render();
  }

  _updateMathLab(node, neighbors = [], heapSnapshot = [], isGoal = false, step = 0) {
    if (!node) return;

    this.el.currentCoord.textContent = `(${node.r}, ${node.c})`;
    this.el.currentEval.textContent = `f = ${node.g} + ${node.h} = ${node.f}`;
    this.el.subG.textContent = node.g;
    this.el.subH.textContent = node.h;
    this.el.subF.textContent = node.f;

    const dr = Math.abs(node.r - this.goal[0]);
    const dc = Math.abs(node.c - this.goal[1]);
    this.el.hCalcDetail.textContent = `|${node.r} - ${this.goal[0]}| + |${node.c} - ${this.goal[1]}| = ${dr} + ${dc} = ${node.h}`;

    // Update Neighbors Table
    if (neighbors.length > 0) {
      let tbodyHtml = '';
      for (const n of neighbors) {
        const costStr = (n.g !== '—') ? `${n.g} + ${n.h} = ${n.f}` : '—';
        tbodyHtml += `
          <tr>
            <td><strong>${n.dir}</strong></td>
            <td>(${n.r}, ${n.c})</td>
            <td>${costStr}</td>
            <td><span class="${n.status}">${n.label}</span></td>
          </tr>
        `;
      }
      this.el.neighborEvalBody.innerHTML = tbodyHtml;

      // Add to explanation log
      const addedCount = neighbors.filter(n => n.status.includes('added') || n.status.includes('goal')).length;
      this._appendLog(`[Step ${step}] Popped (${node.r}, ${node.c}) with f=${node.f} (g=${node.g}, h=${node.h}). Added ${addedCount} neighbor(s) to Open Set.`);
    }

    // Update Priority Queue peek
    if (heapSnapshot) {
      this.el.pqCountBadge.textContent = `${heapSnapshot.length} nodes`;
      if (heapSnapshot.length === 0) {
        this.el.pqLeaderboard.innerHTML = `<div style="color: #6b7280; padding: 4px;">Open set is empty.</div>`;
      } else {
        let pqHtml = '';
        const top = heapSnapshot.slice(0, 5);
        top.forEach((item, idx) => {
          const isTop = idx === 0;
          pqHtml += `
            <div class="pq-row ${isTop ? 'pq-top' : ''}">
              <span>#${idx + 1} (${item.r}, ${item.c})</span>
              <span>g:${item.g} h:${item.h} <strong>f:${item.f}</strong> ${isTop ? '← next' : ''}</span>
            </div>
          `;
        });
        this.el.pqLeaderboard.innerHTML = pqHtml;
      }
    }
  }

  _appendLog(message, className = '') {
    const entry = document.createElement('div');
    entry.className = `log-entry ${className}`;
    entry.textContent = message;
    this.el.explanationLog.appendChild(entry);
    this.el.explanationLog.scrollTop = this.el.explanationLog.scrollHeight;
  }

  // ── Reset & Clearing ───────────────────────────────────────────────────────
  resetSearch() {
    this.pause();
    this.generator = null;
    this._resetCaches();

    for (let r = 0; r < this.rows; r++) {
      for (let c = 0; c < this.cols; c++) {
        if (this.grid[r][c] === OPEN || this.grid[r][c] === CLOSED || this.grid[r][c] === PATH) {
          this.grid[r][c] = EMPTY;
        }
      }
    }
    this.grid[this.start[0]][this.start[1]] = START;
    this.grid[this.goal[0]][this.goal[1]] = GOAL;

    this.el.statStatus.textContent = 'Ready';
    this.el.statStatus.className = 'status-badge status-ready';
    this.el.statPathLen.textContent = '—';
    this.el.statExplored.textContent = '0';
    this.el.statFrontier.textContent = '0';
    this.el.statIterations.textContent = '0';

    this.el.currentCoord.textContent = '—';
    this.el.currentEval.textContent = 'f(n) = g(n) + h(n)';
    this.el.subG.textContent = '—';
    this.el.subH.textContent = '—';
    this.el.subF.textContent = '—';
    this.el.hCalcDetail.textContent = 'h = |r - r_goal| + |c - c_goal|';

    this.el.neighborEvalBody.innerHTML = `<tr><td colspan="4" style="color: #6b7280; text-align: center;">Standing by for search...</td></tr>`;
    this.el.pqLeaderboard.innerHTML = `<div style="color: #6b7280; padding: 4px;">Open set is empty.</div>`;
    this.el.pqCountBadge.textContent = '0 items';

    this._appendLog('--- Search Reset (Walls Kept) ---');
    this._render();
  }

  clearWalls() {
    this.resetSearch();
    for (let r = 0; r < this.rows; r++) {
      for (let c = 0; c < this.cols; c++) {
        if (this.grid[r][c] === WALL) {
          this.grid[r][c] = EMPTY;
        }
      }
    }
    this._appendLog('--- Walls Cleared ---');
    this._render();
  }

  loadPreset(type) {
    this.clearWalls();
    if (type === 'simple') {
      for (let c = 4; c < this.cols - 2; c += 4) {
        const gapRow = (c % 8 === 4) ? this.rows - 3 : 2;
        for (let r = 0; r < this.rows; r++) {
          if ((r === this.start[0] && c === this.start[1]) || (r === this.goal[0] && c === this.goal[1])) continue;
          if (Math.abs(r - gapRow) > 1) {
            this.grid[r][c] = WALL;
          }
        }
      }
      this._appendLog('Loaded Simple Maze Preset.');
    } else if (type === 'random') {
      for (let r = 0; r < this.rows; r++) {
        for (let c = 0; c < this.cols; c++) {
          if ((r === this.start[0] && c === this.start[1]) || (r === this.goal[0] && c === this.goal[1])) continue;
          if (Math.random() < 0.3) {
            this.grid[r][c] = WALL;
          }
        }
      }
      this._appendLog('Generated 30% Random Obstacles.');
    }
    this._render();
  }

  // ── Canvas Rendering Engine ────────────────────────────────────────────────
  _render() {
    const ctx = this.ctx;
    const cs = this.cellSize;
    const width = this.cols * cs;
    const height = this.rows * cs;

    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, width, height);

    const colorMap = {
      [EMPTY]:  { fill: '#ffffff', border: '#e5e7eb' },
      [WALL]:   { fill: '#334155', border: '#1e293b' },
      [START]:  { fill: '#22c55e', border: '#16a34a' },
      [GOAL]:   { fill: '#ef4444', border: '#dc2626' },
      [OPEN]:   { fill: '#38bdf8', border: '#0284c7' },
      [CLOSED]: { fill: '#cbd5e1', border: '#94a3b8' },
      [PATH]:   { fill: '#fef08a', border: '#eab308' }
    };

    // Render cells
    for (let r = 0; r < this.rows; r++) {
      for (let c = 0; c < this.cols; c++) {
        const state = this.grid[r][c];
        const x = c * cs;
        const y = r * cs;

        const styling = colorMap[state] || colorMap[EMPTY];
        ctx.fillStyle = styling.fill;
        ctx.fillRect(x + 1, y + 1, cs - 2, cs - 2);

        ctx.strokeStyle = styling.border;
        ctx.lineWidth = 1;
        ctx.strokeRect(x + 0.5, y + 0.5, cs - 1, cs - 1);

        // Active node highlight
        if (this.activeNode && this.activeNode[0] === r && this.activeNode[1] === c && state !== START && state !== GOAL) {
          ctx.fillStyle = '#f97316';
          ctx.fillRect(x + 1, y + 1, cs - 2, cs - 2);
          ctx.strokeStyle = '#c2410c';
          ctx.lineWidth = 2;
          ctx.strokeRect(x + 1, y + 1, cs - 2, cs - 2);
        }

        // Render Numbers (f-score or all values)
        if (this.overlayType !== 'none' && state !== WALL) {
          this._renderCellOverlay(ctx, r, c, x, y, cs);
        }
      }
    }

    // Render Polyline on shortest path
    if (this.finalPath.length > 1) {
      ctx.save();
      ctx.beginPath();
      ctx.lineWidth = Math.max(3, Math.floor(cs * 0.22));
      ctx.strokeStyle = '#eab308';
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';

      for (let i = 0; i < this.finalPath.length; i++) {
        const [pr, pc] = this.finalPath[i];
        const px = pc * cs + cs / 2;
        const py = pr * cs + cs / 2;
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
      ctx.restore();
    }

    // Render Start (S) and Goal (G) Letters
    this._renderNodeBadge(this.start[0], this.start[1], 'S', '#ffffff');
    this._renderNodeBadge(this.goal[0], this.goal[1], 'G', '#ffffff');
  }

  _renderNodeBadge(r, c, letter, color) {
    const cs = this.cellSize;
    const x = c * cs + cs / 2;
    const y = r * cs + cs / 2;
    const ctx = this.ctx;

    ctx.save();
    ctx.font = `bold ${Math.max(12, Math.floor(cs * 0.5))}px sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = color;
    ctx.fillText(letter, x, y);
    ctx.restore();
  }

  _renderCellOverlay(ctx, r, c, x, y, cs) {
    const key = `${r},${c}`;
    if (!this.fScores.has(key)) return;

    const g = this.gScores.get(key);
    const h = this.hScores.get(key);
    const f = this.fScores.get(key);

    ctx.save();
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    if (this.overlayType === 'f') {
      ctx.font = `600 ${Math.max(9, Math.floor(cs * 0.36))}px monospace`;
      ctx.fillStyle = '#111827';
      ctx.fillText(f, x + cs / 2, y + cs / 2);
    } else if (this.overlayType === 'all') {
      const smallFont = `500 ${Math.max(7, Math.floor(cs * 0.24))}px monospace`;
      // f in center
      ctx.font = `bold ${Math.max(9, Math.floor(cs * 0.32))}px monospace`;
      ctx.fillStyle = '#111827';
      ctx.fillText(f, x + cs / 2, y + cs / 2 + 1);

      // g in top-left
      ctx.font = smallFont;
      ctx.fillStyle = '#0369a1';
      ctx.textAlign = 'left';
      ctx.fillText(g, x + 3, y + Math.floor(cs * 0.26));

      // h in top-right
      ctx.fillStyle = '#15803d';
      ctx.textAlign = 'right';
      ctx.fillText(h, x + cs - 3, y + Math.floor(cs * 0.26));
    }
    ctx.restore();
  }

  // ── Sizing & Events ────────────────────────────────────────────────────────
  _resizeCanvas() {
    const viewport = document.getElementById('canvas-viewport');
    const containerW = viewport && viewport.clientWidth > 0 ? viewport.clientWidth : 832;
    const maxCell = Math.floor(containerW / this.cols);
    this.cellSize = Math.max(16, Math.min(32, maxCell));

    const targetWidth = this.cols * this.cellSize;
    const targetHeight = this.rows * this.cellSize;

    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = targetWidth * dpr;
    this.canvas.height = targetHeight * dpr;
    this.canvas.style.width = `${targetWidth}px`;
    this.canvas.style.height = `${targetHeight}px`;

    this.ctx.resetTransform?.();
    this.ctx.scale(dpr, dpr);
    this._render();
  }

  _getCellFromEvent(e) {
    const rect = this.canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const col = Math.floor(x / this.cellSize);
    const row = Math.floor(y / this.cellSize);

    if (row >= 0 && row < this.rows && col >= 0 && col < this.cols) {
      return [row, col];
    }
    return null;
  }

  _applyToolToCell(r, c) {
    if (this.activeTool === 'wall') {
      if ((r === this.start[0] && c === this.start[1]) || (r === this.goal[0] && c === this.goal[1])) return;
      this.grid[r][c] = WALL;
      this._render();
    } else if (this.activeTool === 'erase') {
      if ((r === this.start[0] && c === this.start[1]) || (r === this.goal[0] && c === this.goal[1])) return;
      this.grid[r][c] = EMPTY;
      this._render();
    } else if (this.activeTool === 'start') {
      if (r === this.goal[0] && c === this.goal[1]) return;
      this.grid[this.start[0]][this.start[1]] = EMPTY;
      this.start = [r, c];
      this.grid[r][c] = START;
      this.resetSearch();
    } else if (this.activeTool === 'goal') {
      if (r === this.start[0] && c === this.start[1]) return;
      this.grid[this.goal[0]][this.goal[1]] = EMPTY;
      this.goal = [r, c];
      this.grid[r][c] = GOAL;
      this.resetSearch();
    }
  }

  _setupEventListeners() {
    this.el.btnPlay.addEventListener('click', () => this.toggleRun());
    this.el.btnStep.addEventListener('click', () => this.step());
    this.el.btnResetSearch.addEventListener('click', () => this.resetSearch());
    this.el.btnClearWalls.addEventListener('click', () => this.clearWalls());

    this.el.btnClearLog.addEventListener('click', () => {
      this.el.explanationLog.innerHTML = '<div class="log-entry">Log cleared. Ready.</div>';
    });

    this.el.tools.forEach(btn => {
      btn.addEventListener('click', () => {
        this.el.tools.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        this.activeTool = btn.dataset.tool;
      });
    });

    this.el.presetSelect.addEventListener('change', (e) => {
      if (e.target.value) {
        this.loadPreset(e.target.value);
        e.target.value = '';
      }
    });

    this.el.overlaySelect.addEventListener('change', (e) => {
      this.overlayType = e.target.value;
      this._render();
    });

    this.el.speedSlider.addEventListener('input', (e) => {
      this.delayMs = parseInt(e.target.value, 10);
      this.el.speedDisplay.textContent = `${this.delayMs} ms`;
    });

    this.canvas.addEventListener('mousedown', (e) => {
      if (this.isRunning) return;
      this.isMouseDown = true;
      const cell = this._getCellFromEvent(e);
      if (cell) this._applyToolToCell(cell[0], cell[1]);
    });

    this.canvas.addEventListener('mousemove', (e) => {
      if (this.isMouseDown && !this.isRunning) {
        const cell = this._getCellFromEvent(e);
        if (cell) this._applyToolToCell(cell[0], cell[1]);
      }
    });

    window.addEventListener('mouseup', () => {
      this.isMouseDown = false;
    });

    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;

      if (e.code === 'Space') {
        e.preventDefault();
        this.toggleRun();
      } else if (e.code === 'KeyS' || e.code === 'ArrowRight') {
        e.preventDefault();
        this.step();
      } else if (e.code === 'KeyR') {
        e.preventDefault();
        this.resetSearch();
      } else if (e.code === 'KeyW') {
        this._selectTool('wall');
      } else if (e.code === 'KeyE') {
        this._selectTool('erase');
      }
    });

    window.addEventListener('resize', () => {
      this._resizeCanvas();
    });
  }

  _selectTool(toolName) {
    const target = Array.from(this.el.tools).find(b => b.dataset.tool === toolName);
    if (target) target.click();
  }
}

// Instantiate on load
document.addEventListener('DOMContentLoaded', () => {
  window.app = new AStarVisualizer();
});

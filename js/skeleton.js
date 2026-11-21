/* The supervisor's waypoint idea, ported from Start-Kit-sipp-cpp/src/MAPFPlanner.cpp (initialize, reduceMapStart, reduceMap,
   reduceMapUpdate, BFStoReducedMap, reducedMapAStar). Cells are indexed row * cols + col like in the planner.
   1. reduceMapStart: every cell gets its distance to the nearest obstacle (four sweeps over the map). In big open areas, cells on a
      grid of `step` cells become artificial obstacles, so open space does not stay empty.
   2. thinning: while cells can be removed, remove the layer of cells next to an obstacle that are not needed for the connection
      (reduceMap(true)), then the dead-end spurs (reduceMap(false)); the distances are recomputed after each pass.
      What stays (value > 0) is a thin skeleton of corridors and crossroads.
   3. route: for a start and a goal, take the nearest skeleton cell of each (BFS), run A* on the skeleton (Manhattan heuristic)
      and keep every `step`-th cell of the path, counted from the goal: these are the route waypoints. */
(function () {
  "use strict";

  function Skeleton(map, step) {
    this.rows = map.rows; this.cols = map.cols; this.grid = map.grid; this.n = map.rows * map.cols;
    this.step = step;
    this.rm = new Int32Array(this.n);
    this.rounds = 0;
  }

  // reduceMapStart
  Skeleton.prototype.start = function () {
    var rows = this.rows, cols = this.cols, grid = this.grid, rm = this.rm, S = this.step, x, y, loc, d;
    for (x = 0; x < rows; x++) {
      d = 0;
      for (y = 0; y < cols; y++) {
        loc = x * cols + y;
        d = grid[loc] === 1 ? 0 : d + 1;
        if (d > S && x % S === 0 && y % S === 0) d = 0;
        rm[loc] = d;
      }
    }
    for (x = rows - 1; x >= 0; x--) {
      d = 0;
      for (y = cols - 1; y >= 0; y--) {
        loc = x * cols + y;
        d = grid[loc] === 1 ? 0 : Math.min(d + 1, rm[loc]);
        if (d > S && x % S === 0 && y % S === 0) d = 0;
        rm[loc] = d;
      }
    }
    for (y = 0; y < cols; y++) {
      d = 0;
      for (x = 0; x < rows; x++) {
        loc = x * cols + y;
        d = grid[loc] === 1 ? 0 : Math.min(d + 1, rm[loc]);
        if (d > S && x % S === 0 && y % S === 0) d = 0;
        rm[loc] = d;
      }
    }
    for (y = cols - 1; y >= 0; y--) {
      d = 0;
      for (x = rows - 1; x >= 0; x--) {
        loc = x * cols + y;
        d = grid[loc] === 1 ? 0 : Math.min(d + 1, rm[loc]);
        if (d > S && x % S === 0 && y % S === 0) d = 0;
        rm[loc] = d;
      }
    }
  };

  // reduceMapUpdate: distances again, now every zero cell (obstacle or removed cell) is a source
  Skeleton.prototype.update = function () {
    var rows = this.rows, cols = this.cols, rm = this.rm, x, y, loc, d;
    for (x = 0; x < rows; x++) { d = 0; for (y = 0; y < cols; y++) { loc = x * cols + y; d = rm[loc] === 0 ? 0 : Math.min(d + 1, rm[loc]); rm[loc] = d; } }
    for (x = rows - 1; x >= 0; x--) { d = 0; for (y = cols - 1; y >= 0; y--) { loc = x * cols + y; d = rm[loc] === 0 ? 0 : Math.min(d + 1, rm[loc]); rm[loc] = d; } }
    for (y = 0; y < cols; y++) { d = 0; for (x = 0; x < rows; x++) { loc = x * cols + y; d = rm[loc] === 0 ? 0 : Math.min(d + 1, rm[loc]); rm[loc] = d; } }
    for (y = cols - 1; y >= 0; y--) { d = 0; for (x = rows - 1; x >= 0; x--) { loc = x * cols + y; d = rm[loc] === 0 ? 0 : Math.min(d + 1, rm[loc]); rm[loc] = d; } }
  };

  Skeleton.prototype.countV = function (loc) {
    var rm = this.rm, c = this.cols, k = 0;
    if (loc >= c && rm[loc - c] > 0) k++;
    if (loc + c < this.n && rm[loc + c] > 0) k++;
    return k;
  };
  Skeleton.prototype.countH = function (loc) {
    var rm = this.rm, c = this.cols, y = loc % c, k = 0;
    if (y > 0 && rm[loc - 1] > 0) k++;
    if (y + 1 < c && rm[loc + 1] > 0) k++;
    return k;
  };

  // areNeighboursTraversable: false for a bridge node and for the corner cases of the 2x2 blocks
  Skeleton.prototype.traversable = function (loc) {
    var rm = this.rm, c = this.cols, rows = this.rows, n = this.n, x = (loc / c) | 0, y = loc % c;
    var h = this.countH(loc), v = this.countV(loc);
    if ((v === 0 && h === 2) || (v === 2 && h === 0)) return false;
    var N = loc >= c && rm[loc - c] > 0, S = loc + c < n && rm[loc + c] > 0, E = y + 1 < c && rm[loc + 1] > 0, W = y > 0 && rm[loc - 1] > 0;
    if (N && E && !(x > 0 && y + 1 < c && rm[loc - c + 1] > 0)) return false;
    if (S && E && !(x + 1 < rows && y + 1 < c && rm[loc + c + 1] > 0)) return false;
    if (S && W && !(x + 1 < rows && y + 1 > 0 && rm[loc + c - 1] > 0)) return false;
    if (N && W && !(x > 0 && y > 0 && rm[loc - c - 1] > 0)) return false;
    return true;
  };

  // reduceMap(keepSalient)
  Skeleton.prototype.reduce = function (keepSalient) {
    var rm = this.rm, n = this.n, changed = false, i;
    if (keepSalient) {
      for (i = 0; i < n; i++) {
        if (rm[i] === 1 && this.traversable(i) && (this.countV(i) + this.countH(i)) > 1) { changed = true; rm[i] = 0; }
      }
    } else {
      var tmp = new Int32Array(rm);
      for (i = 0; i < n; i++) {
        if (rm[i] === 1 && (this.countV(i) + this.countH(i)) === 1) { changed = true; tmp[i] = 0; }
      }
      this.rm.set(tmp);
    }
    this.update();
    return changed;
  };

  Skeleton.prototype.build = function () {
    this.start();
    var rounds = 0;
    while (this.reduce(true) && rounds < 2000) { this.reduce(false); rounds++; }
    this.rounds = rounds;
    var cnt = 0;
    for (var i = 0; i < this.n; i++) if (this.rm[i] > 0) cnt++;
    this.cells = cnt;
  };

  // BFStoReducedMap: the nearest skeleton cell (ties: the smallest cell index), by steps over free cells
  Skeleton.prototype.nearest = function (start) {
    var cols = this.cols, rows = this.rows, grid = this.grid, rm = this.rm;
    var seen = new Uint8Array(this.n), layer = [start], dist = 0;
    seen[start] = 1;
    while (layer.length) {
      var best = -1;
      for (var i = 0; i < layer.length; i++) if (rm[layer[i]] > 0 && (best < 0 || layer[i] < best)) best = layer[i];
      if (best >= 0) return { cell: best, steps: dist };
      var next = [];
      for (var k = 0; k < layer.length; k++) {
        var u = layer[k], r = (u / cols) | 0, c = u - r * cols, v;
        if (c + 1 < cols) { v = u + 1; if (!grid[v] && !seen[v]) { seen[v] = 1; next.push(v); } }
        if (r + 1 < rows) { v = u + cols; if (!grid[v] && !seen[v]) { seen[v] = 1; next.push(v); } }
        if (c > 0) { v = u - 1; if (!grid[v] && !seen[v]) { seen[v] = 1; next.push(v); } }
        if (r > 0) { v = u - cols; if (!grid[v] && !seen[v]) { seen[v] = 1; next.push(v); } }
      }
      layer = next; dist++;
    }
    return null;
  };

  // reducedMapAStar: A* on the skeleton; returns the path (start to end) and the waypoints (every `step`-th cell counted from the end)
  Skeleton.prototype.route = function (start, end) {
    var cols = this.cols, n = this.n, rm = this.rm, step = this.step;
    var heap = [];
    function push(key) { heap.push(key); var i = heap.length - 1; while (i > 0) { var p = (i - 1) >> 1; if (heap[p] <= heap[i]) break; var t = heap[p]; heap[p] = heap[i]; heap[i] = t; i = p; } }
    function pop() {
      var top = heap[0], last = heap.pop();
      if (heap.length) {
        heap[0] = last; var i = 0;
        for (;;) { var l = 2 * i + 1, r = l + 1, m = i; if (l < heap.length && heap[l] < heap[m]) m = l; if (r < heap.length && heap[r] < heap[m]) m = r; if (m === i) break; var t = heap[m]; heap[m] = heap[i]; heap[i] = t; i = m; }
      }
      return top;
    }
    var M = 4194304;                                                    // key = f * 2^22 + cell: smaller f first, then smaller cell (as the planner's priority queue)
    function manh(a, b) { return Math.abs(((a / cols) | 0) - ((b / cols) | 0)) + Math.abs(a % cols - b % cols); }
    var g = new Int32Array(n).fill(-1), parent = new Int32Array(n).fill(-2);
    g[start] = manh(start, end); parent[start] = -1;                   // the planner starts g at the heuristic value (the path is the same)
    g[start] = 0;
    push(manh(start, end) * M + start);
    var found = false;
    while (heap.length) {
      var cur = pop() % M;
      if (cur === end) { found = true; break; }
      var newG = g[cur] + 1, x = (cur / cols) | 0, y = cur - x * cols;
      var cand = [cur + 1, cur + cols, cur - 1, cur - cols];
      for (var i = 0; i < 4; i++) {
        var nb = cand[i];
        if (nb < 0 || nb >= n || rm[nb] === 0) continue;
        var nx = (nb / cols) | 0, ny = nb - nx * cols;
        if (Math.abs(nx - x) + Math.abs(ny - y) > 1) continue;
        if (g[nb] < 0 || newG < g[nb]) { g[nb] = newG; parent[nb] = cur; push((newG + manh(nb, end)) * M + nb); }
      }
    }
    if (!found) return null;
    var path = [], wps = [], distance = 0, c = end;
    while (c !== -1) {
      path.push(c);
      if (distance === 0) { wps.push(c); distance = step; }
      distance--;
      c = parent[c];
    }
    path.reverse(); wps.reverse();
    return { path: path, waypoints: wps };
  };

  window.Skeleton = Skeleton;
})();

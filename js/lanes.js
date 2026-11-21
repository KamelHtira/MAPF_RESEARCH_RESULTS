/* One-way lanes tool: the street grid of the PIBT planner (src/PIBT/Guidance.cpp, applyStreetLanes) with the dead-end check. */
(function () {
  "use strict";
  var $ = function (id) { return document.getElementById(id); };
  var view = new MapTools.MapView($("canvas"));
  var map = null, res = null, laneLayer = null;
  var DIRS = ["east", "south", "west", "north"];
  var COLORS = [[25, 113, 194], [47, 158, 68], [240, 140, 0], [174, 62, 201]];   // preferred direction
  var timer = null;

  function compute(m, minCells, prune) {
    var rows = m.rows, cols = m.cols, grid = m.grid, n = rows * cols;
    var free = function (r, c) { return r >= 0 && r < rows && c >= 0 && c < cols && !grid[r * cols + c]; };
    // neighbour of cell u in direction d (0 east, 1 south, 2 west, 3 north), -1 if there is none or it is an obstacle
    function nbr(u, d) {
      var c = u % cols, v;
      if (d === 0) { if (c + 1 >= cols) return -1; v = u + 1; }
      else if (d === 1) { v = u + cols; if (v >= n) return -1; }
      else if (d === 2) { if (c === 0) return -1; v = u - 1; }
      else { v = u - cols; if (v < 0) return -1; }
      return grid[v] ? -1 : v;
    }
    var vert = new Uint8Array(n), horz = new Uint8Array(n), vcount = new Int32Array(cols), hcount = new Int32Array(rows), corridor = 0;
    for (var u = 0; u < n; u++) {
      if (grid[u]) continue;
      var r = (u / cols) | 0, c = u - r * cols;
      if (free(r - 1, c) && free(r + 1, c) && !free(r, c - 1) && !free(r, c + 1)) { vert[u] = 1; vcount[c]++; corridor++; }
      if (free(r, c - 1) && free(r, c + 1) && !free(r - 1, c) && !free(r + 1, c)) { horz[u] = 1; hcount[r]++; corridor++; }
    }
    var vrank = new Int32Array(cols).fill(-1), hrank = new Int32Array(rows).fill(-1), nv = 0, nh = 0;
    for (var c2 = 0; c2 < cols; c2++) if (vcount[c2] >= minCells) vrank[c2] = nv++;
    for (var r2 = 0; r2 < rows; r2++) if (hcount[r2] >= minCells) hrank[r2] = nh++;

    // lane segments: straight runs of lane cells; laneDir = the direction that is against the lane
    var laneDir = new Int8Array(n).fill(-1), segOf = new Int32Array(n).fill(-1), segs = [];
    for (var a = 0; a < n; a++) {
      var ra = (a / cols) | 0, ca = a - ra * cols;
      if (vert[a] && vrank[ca] >= 0) {
        laneDir[a] = (vrank[ca] % 2 === 0) ? 3 : 1;
        segOf[a] = (ra > 0 && segOf[a - cols] >= 0 && vert[a - cols]) ? segOf[a - cols] : segs.length;
      } else if (horz[a] && hrank[ra] >= 0) {
        laneDir[a] = (hrank[ra] % 2 === 0) ? 2 : 0;
        segOf[a] = (ca > 0 && segOf[a - 1] >= 0 && horz[a - 1]) ? segOf[a - 1] : segs.length;
      } else continue;
      if (segOf[a] === segs.length) segs.push([]);
      segs[segOf[a]].push(a);
    }
    var active = new Uint8Array(segs.length).fill(1);
    var against = new Uint8Array(n * 4);
    function rebuild() {
      against.fill(0);
      for (var k = 0; k < segs.length; k++) {
        if (!active[k]) continue;
        for (var i = 0; i < segs[k].length; i++) { var cell = segs[k][i], d = laneDir[cell]; if (nbr(cell, d) >= 0) against[cell * 4 + d] = 1; }
      }
    }
    // strongly connected components when the moves against the lanes are removed (Kosaraju, iterative); returns the biggest one
    var label = new Int32Array(n);
    function scc() {
      label.fill(-1);
      var vis = new Uint8Array(n), order = new Int32Array(n), ol = 0, sn = new Int32Array(n), sd = new Uint8Array(n);
      for (var s = 0; s < n; s++) {
        if (grid[s] || vis[s]) continue;
        var sp = 0; sn[0] = s; sd[0] = 0; sp = 1; vis[s] = 1;
        while (sp > 0) {
          var x = sn[sp - 1], d = sd[sp - 1];
          if (d < 4) {
            sd[sp - 1]++;
            var v = nbr(x, d);
            if (v >= 0 && !against[x * 4 + d] && !vis[v]) { vis[v] = 1; sn[sp] = v; sd[sp] = 0; sp++; }
          } else { order[ol++] = x; sp--; }
        }
      }
      var sizes = [];
      for (var i = ol - 1; i >= 0; i--) {
        var s2 = order[i];
        if (label[s2] >= 0) continue;
        var id = sizes.length; sizes.push(0);
        var st = [s2]; label[s2] = id;
        while (st.length) {
          var y = st.pop(); sizes[id]++;
          for (var e = 0; e < 4; e++) {
            var p = nbr(y, e);                                   // p -> y is a move in direction (e + 2)
            if (p >= 0 && label[p] < 0 && !against[p * 4 + ((e + 2) & 3)]) { label[p] = id; st.push(p); }
          }
        }
      }
      var best = 0;
      for (var j = 1; j < sizes.length; j++) if (sizes[j] > sizes[best]) best = j;
      return { main: best, size: sizes[best] || 0 };
    }

    var dropped = 0, rounds = 0, main = { main: 0, size: 0 };
    for (; rounds < 30; rounds++) {
      rebuild();
      main = scc();
      if (!prune) break;
      var bad = new Uint8Array(n), any = false;
      for (var b = 0; b < n; b++) if (!grid[b] && label[b] !== main.main) { bad[b] = 1; any = true; }
      if (!any) break;
      var d2 = 0;
      for (var k = 0; k < segs.length; k++) {
        if (!active[k]) continue;
        var hit = false;
        for (var i = 0; i < segs[k].length && !hit; i++) {
          var cell = segs[k][i];
          if (bad[cell]) { hit = true; break; }
          for (var d = 0; d < 4; d++) { var v = nbr(cell, d); if (v >= 0 && bad[v]) { hit = true; break; } }
        }
        if (hit) { active[k] = 0; d2++; }
      }
      dropped += d2;
      if (d2 === 0) break;
    }
    rebuild();
    main = scc();
    var laneCells = 0;
    for (var k2 = 0; k2 < segs.length; k2++) if (active[k2]) for (var i2 = 0; i2 < segs[k2].length; i2++) if (nbr(segs[k2][i2], laneDir[segs[k2][i2]]) >= 0) laneCells++;
    return { nv: nv, nh: nh, corridor: corridor, segs: segs, active: active, segOf: segOf, laneDir: laneDir, vrank: vrank, hrank: hrank, dropped: dropped,
             rounds: rounds, laneCells: laneCells, connected: main.size, nbr: nbr };
  }

  function buildLayer() {
    laneLayer = view.cellLayer(function (u) {
      var k = res.segOf[u];
      if (k < 0) return null;
      if (!res.active[k]) return $("show-dropped").checked ? [224, 49, 49, 255] : null;
      var c = COLORS[(res.laneDir[u] + 2) & 3];
      return [c[0], c[1], c[2], 255];
    });
  }

  function stats() {
    var rows = [
      ["Map", map.rows + " x " + map.cols],
      ["Free cells", map.free.toLocaleString()],
      ["1-wide corridor cells", res.corridor.toLocaleString()],
      ["Vertical streets", res.nv],
      ["Horizontal streets", res.nh],
      ["Lane segments", res.segs.length.toLocaleString()],
      ["Dropped (dead ends)", res.dropped.toLocaleString()],
      ["Cells with a lane", res.laneCells.toLocaleString()],
      ["Free cells that reach all others", res.connected.toLocaleString() + " of " + map.free.toLocaleString() + (res.connected === map.free ? " (all)" : "")]
    ];
    $("stats").innerHTML = rows.map(function (r) { return "<dt>" + r[0] + "</dt><dd>" + r[1] + "</dd>"; }).join("");
  }

  function recompute() {
    if (!map) return;
    $("mv").textContent = $("min").value;
    $("stats").innerHTML = "<dt>Working ...</dt><dd></dd>";
    setTimeout(function () {
      res = compute(map, +$("min").value, $("prune").checked);
      buildLayer();
      stats();
      $("info").textContent = "Click a lane cell to see its street and segment.";
      view.draw();
    }, 20);
  }
  function later() { clearTimeout(timer); timer = setTimeout(recompute, 150); $("mv").textContent = $("min").value; }

  view.layers.push(function (ctx, v) {
    if (!res || !laneLayer) return;
    ctx.drawImage(laneLayer, 0, 0);
    if (v.scale < 7 || !$("show-arrows").checked) return;
    var vis = v.visible(), cols = map.cols;
    ctx.fillStyle = "rgba(255,255,255,0.95)";
    ctx.beginPath();
    for (var r = vis[0]; r <= vis[1]; r++) {
      for (var c = vis[2]; c <= vis[3]; c++) {
        var u = r * cols + c, k = res.segOf[u];
        if (k < 0 || !res.active[k]) continue;
        var d = (res.laneDir[u] + 2) & 3, x = c + 0.5, y = r + 0.5, s = 0.3;
        var dx = d === 0 ? 1 : d === 2 ? -1 : 0, dy = d === 1 ? 1 : d === 3 ? -1 : 0;       // direction of travel
        ctx.moveTo(x + dx * s, y + dy * s);
        ctx.lineTo(x - dx * s + dy * s, y - dy * s + dx * s);
        ctx.lineTo(x - dx * s - dy * s, y - dy * s - dx * s);
        ctx.closePath();
      }
    }
    ctx.fill();
  });

  view.onClick = function (cell) {
    if (!cell || !res) return;
    var k = res.segOf[cell.u];
    if (k < 0) { $("info").textContent = "Row " + cell.r + ", column " + cell.c + ": no lane here" + (map.grid[cell.u] ? " (obstacle)." : "."); return; }
    var vertical = res.laneDir[cell.u] === 1 || res.laneDir[cell.u] === 3;
    var rank = vertical ? res.vrank[cell.c] : res.hrank[cell.r];
    var dir = DIRS[(res.laneDir[cell.u] + 2) & 3];
    $("info").textContent = "Row " + cell.r + ", column " + cell.c + ": " + (vertical ? "vertical" : "horizontal") + " street number " + rank +
      " (of " + (vertical ? res.nv : res.nh) + "), going " + dir + ". Segment of " + res.segs[k].length + " cells" +
      (res.active[k] ? "." : ", dropped because it would create a dead end: robots may use it in both directions.");
  };
  view.onMove = function (cell) {
    var h = $("hover");
    if (!cell || !map) { h.hidden = true; return; }
    h.hidden = false;
    var k = res ? res.segOf[cell.u] : -1;
    h.textContent = "row " + cell.r + ", col " + cell.c + (map.grid[cell.u] ? " (obstacle)" : k >= 0 ? (res.active[k] ? " (lane " + DIRS[(res.laneDir[cell.u] + 2) & 3] + ")" : " (dropped lane)") : "");
  };

  $("min").addEventListener("input", later);
  $("prune").addEventListener("change", recompute);
  $("show-dropped").addEventListener("change", function () { if (res) { buildLayer(); view.draw(); } });
  $("show-arrows").addEventListener("change", function () { view.draw(); });
  $("zin").addEventListener("click", function () { view.zoomCenter(1.5); });
  $("zout").addEventListener("click", function () { view.zoomCenter(1 / 1.5); });
  $("fit").addEventListener("click", function () { view.fit(); });

  var loadSample = MapTools.setupLoader({
    file: $("file"), drop: $("drop"), sample: $("sample"), status: $("status"),
    onMap: function (m) { map = m; res = null; laneLayer = null; view.setMap(m); recompute(); }
  });
  $("mv").textContent = $("min").value;
  loadSample();
})();

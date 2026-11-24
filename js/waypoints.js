/* Waypoint tool. Four views of a map:
   - cover points: the waypoints of the PIBT planner (src/PIBT/Waypoints.cpp, plain steps) that its waypoint distance is built on;
   - the supervisor's skeleton and route waypoints (js/skeleton.js, ported from his SIPP planner);
   - clusters: the area method of Team Hipparcos (2024) with base points on the junctions of the skeleton (or on a uniform grid, as they do);
   - dynamic cover points: the cover points with a local radius (denser at junctions and thin walls, sparser in open areas and corridors),
     and a measurement of how far the distance estimate lies above the true distance. */
(function () {
  "use strict";
  var $ = function (id) { return document.getElementById(id); };
  var view = new MapTools.MapView($("canvas"));
  var map = null, n = 0, rows = 0, cols = 0, grid = null;
  var mode = "cover";
  var seen, dist, queue, stamp = 0;                 // scratch arrays of the bounded search
  var cells = [], wpOf = null, owner = null, maxCover = 0;
  var sel = null;                                   // selected cover point: { wp, window, dist, neigh }
  var covLayer = null, selLayer = null;
  var skel = null, skelLayer = null, route = null;  // skeleton view
  var timer = null, timer2 = null, timer3 = null;
  var gskel = null, gLayer = null, G = null, gSel = null;   // graph view: skeleton, its faint layer, the graph, the selected node
  var dyn = null, wpTag = null, extraEst = null, extraMeasure = null;   // dynamic view: per-map data, rule that set each waypoint, extra rows of numbers
  function isCover() { return mode === "cover" || mode === "dyn"; }

  // breadth-first search from src over free cells, up to `limit` steps; onCell(cell, steps) for every cell reached
  function bfs(src, limit, onCell) {
    var st = ++stamp, head = 0, tail = 1;
    queue[0] = src; seen[src] = st; dist[src] = 0;
    while (head < tail) {
      var u = queue[head++], du = dist[u];
      onCell(u, du);
      if (du >= limit) continue;
      var r = (u / cols) | 0, c = u - r * cols, v;
      if (c > 0)        { v = u - 1;    if (!grid[v] && seen[v] !== st) { seen[v] = st; dist[v] = du + 1; queue[tail++] = v; } }
      if (c < cols - 1) { v = u + 1;    if (!grid[v] && seen[v] !== st) { seen[v] = st; dist[v] = du + 1; queue[tail++] = v; } }
      if (r > 0)        { v = u - cols; if (!grid[v] && seen[v] !== st) { seen[v] = st; dist[v] = du + 1; queue[tail++] = v; } }
      if (r < rows - 1) { v = u + cols; if (!grid[v] && seen[v] !== st) { seen[v] = st; dist[v] = du + 1; queue[tail++] = v; } }
    }
  }

  // ---------- cover points (the planner's waypoints) ----------
  function place(C) {
    var covered = new Uint8Array(n);
    wpOf = new Int32Array(n).fill(-1);
    cells = []; wpTag = null;
    for (var u = 0; u < n; u++) {
      if (grid[u] || covered[u]) continue;
      wpOf[u] = cells.length;
      cells.push(u);
      bfs(u, C, function (v) { covered[v] = 1; });
    }
    computeOwner();
  }

  // nearest waypoint of every cell (all waypoints search at the same time)
  function computeOwner() {
    owner = new Int32Array(n).fill(-1);
    var ownerDist = new Int16Array(n);
    var q = new Int32Array(n), head = 0, tail = 0;
    for (var k = 0; k < cells.length; k++) { owner[cells[k]] = k; q[tail++] = cells[k]; }
    maxCover = 0;
    while (head < tail) {
      var a = q[head++], da = ownerDist[a];
      if (da > maxCover) maxCover = da;
      var r = (a / cols) | 0, c = a - r * cols, v;
      if (c > 0)        { v = a - 1;    if (!grid[v] && owner[v] < 0) { owner[v] = owner[a]; ownerDist[v] = da + 1; q[tail++] = v; } }
      if (c < cols - 1) { v = a + 1;    if (!grid[v] && owner[v] < 0) { owner[v] = owner[a]; ownerDist[v] = da + 1; q[tail++] = v; } }
      if (r > 0)        { v = a - cols; if (!grid[v] && owner[v] < 0) { owner[v] = owner[a]; ownerDist[v] = da + 1; q[tail++] = v; } }
      if (r < rows - 1) { v = a + cols; if (!grid[v] && owner[v] < 0) { owner[v] = owner[a]; ownerDist[v] = da + 1; q[tail++] = v; } }
    }
    covLayer = null;
  }

  // ---------- dynamic cover points ----------
  // The same greedy pass as above, but every cell has its own radius rad[u] (how far a point there has to reach):
  //   default C; around skeleton junctions and at thin walls a fraction of C (denser); in open areas and corridors a multiple of C (sparser).
  // A point at u covers the cells v within min(rad[u], rad[v]) steps, so a cell in a dense zone is never left to a far point of a sparse zone.
  // Points on skeleton junctions are placed first. The radius stays between 2 and max(C, R / 2): two neighbouring points must be within R steps.
  var DYN_NAME = ["default", "junction point", "dense near junction", "dense at thin wall", "sparse, open area", "sparse, corridor"];
  var DYN_COLOR = ["#d6336c", "#f08c00", "#e8590c", "#7048e8", "#1971c2", "#12b886"];

  function dynPrep() {
    if (dyn) return dyn;
    var d = { tw: null }, u, r, c, v, head, tail, q = new Int32Array(n);
    // clearance: steps to the nearest obstacle (the map border counts as an obstacle)
    var cl = new Uint8Array(n).fill(255);
    head = 0; tail = 0;
    for (u = 0; u < n; u++) {
      if (grid[u]) { cl[u] = 0; continue; }
      r = (u / cols) | 0; c = u - r * cols;
      if (r === 0 || c === 0 || r === rows - 1 || c === cols - 1 ||
          grid[u - 1] || grid[u + 1] || grid[u - cols] || grid[u + cols]) { cl[u] = 1; q[tail++] = u; }
    }
    while (head < tail) {
      u = q[head++]; r = (u / cols) | 0; c = u - r * cols;
      var nb = [c > 0 ? u - 1 : -1, c < cols - 1 ? u + 1 : -1, r > 0 ? u - cols : -1, r < rows - 1 ? u + cols : -1];
      for (var k = 0; k < 4; k++) { v = nb[k]; if (v >= 0 && !grid[v] && cl[v] === 255) { cl[v] = Math.min(254, cl[u] + 1); q[tail++] = v; } }
    }
    for (u = 0; u < n; u++) if (cl[u] === 255) cl[u] = 1;
    d.cl = cl;
    // largest clearance within 3 cells in every direction (a cell is in a corridor when this is at most 2)
    var tmp = new Uint8Array(n), mcl = new Uint8Array(n), x, best;
    for (r = 0; r < rows; r++) for (c = 0; c < cols; c++) {
      best = 0;
      for (x = Math.max(0, c - 3); x <= Math.min(cols - 1, c + 3); x++) if (cl[r * cols + x] > best) best = cl[r * cols + x];
      tmp[r * cols + c] = best;
    }
    for (r = 0; r < rows; r++) for (c = 0; c < cols; c++) {
      best = 0;
      for (x = Math.max(0, r - 3); x <= Math.min(rows - 1, r + 3); x++) if (tmp[x * cols + c] > best) best = tmp[x * cols + c];
      mcl[r * cols + c] = best;
    }
    d.mcl = mcl;
    // skeleton junctions (3 or 4 skeleton neighbours) and the steps from every cell to the nearest one
    var sk = skel || gskel;
    if (!sk) { sk = new Skeleton(map, 24); sk.build(); }
    var t4 = [0, 0, 0, 0], junc = [];
    for (u = 0; u < n; u++) { if (sk.rm[u] > 0 && skelNbrs(sk.rm, u, t4) >= 3) junc.push(u); }
    d.junc = junc;
    var jd = new Uint8Array(n).fill(255);
    head = 0; tail = 0;
    for (var j = 0; j < junc.length; j++) { jd[junc[j]] = 0; q[tail++] = junc[j]; }
    while (head < tail) {
      u = q[head++]; if (jd[u] >= 60) continue;
      r = (u / cols) | 0; c = u - r * cols;
      var nb2 = [c > 0 ? u - 1 : -1, c < cols - 1 ? u + 1 : -1, r > 0 ? u - cols : -1, r < rows - 1 ? u + cols : -1];
      for (var k2 = 0; k2 < 4; k2++) { v = nb2[k2]; if (v >= 0 && !grid[v] && jd[v] === 255) { jd[v] = jd[u] + 1; q[tail++] = v; } }
    }
    d.jd = jd;
    dyn = d;
    return d;
  }

  // thin walls: the share (percent) of the free cells within 3 steps (straight-line, in rows plus columns) that need a detour of 2 or more steps
  function dynTw() {
    var d = dynPrep();
    if (d.tw) return d.tw;
    var tw = new Uint8Array(n), K = 3;
    for (var u = 0; u < n; u++) {
      if (grid[u]) continue;
      var r0 = (u / cols) | 0, c0 = u - r0 * cols, f3 = 0, ok = 0, dr, dc;
      for (dr = -K; dr <= K; dr++) {
        var rr = r0 + dr; if (rr < 0 || rr >= rows) continue;
        var span = K - Math.abs(dr);
        for (dc = -span; dc <= span; dc++) { var cc = c0 + dc; if (cc >= 0 && cc < cols && !grid[rr * cols + cc]) f3++; }
      }
      bfs(u, K + 2, function (v, dd) {
        var vr = (v / cols) | 0, m = Math.abs(vr - r0) + Math.abs(v - vr * cols - c0);
        if (m <= K && dd <= m + 1) ok++;
      });
      tw[u] = f3 > 0 ? Math.round(100 * (1 - ok / f3)) : 0;
    }
    d.tw = tw;
    return tw;
  }

  function dynOptions() {
    return { seed: $("d-seed").checked, zone: +$("d-zone").value, juncP: +$("d-juncp").value / 100,
             wall: $("d-wall").checked, wallP: +$("d-wallp").value / 100, openCl: +$("d-opencl").value,
             openP: +$("d-openp").value / 100, corrP: +$("d-corrp").value / 100, twMin: 25 };
  }

  function dynRadius(C, R, o) {
    var d = dynPrep(), tw = o.wall ? dynTw() : null;
    var rad = new Uint8Array(n), tag = new Uint8Array(n), cmax = Math.max(C, R >> 1), lo = Math.min(2, C);
    for (var u = 0; u < n; u++) {
      if (grid[u]) continue;
      var f = 1, t = 0;
      if (d.cl[u] >= o.openCl) { f = o.openP; t = 4; }
      else if (d.mcl[u] <= 2) { f = o.corrP; t = 5; }
      var fz = (o.zone > 0 && d.jd[u] <= o.zone) ? o.juncP : 9, fw = (tw && tw[u] >= o.twMin) ? o.wallP : 9, fd = Math.min(fz, fw);
      if (fd < f) { f = fd; t = fz <= fw ? 2 : 3; }
      rad[u] = Math.max(lo, Math.min(cmax, Math.round(C * f)));
      tag[u] = t;
    }
    return { rad: rad, tag: tag };
  }

  function placeDyn(C, R, opt) {
    var o = opt || dynOptions(), d = dynPrep(), rt = dynRadius(C, R, o), rad = rt.rad, tag = rt.tag;
    var covered = new Uint8Array(n);
    wpOf = new Int32Array(n).fill(-1);
    cells = []; wpTag = [];
    function add(u, t) {
      wpOf[u] = cells.length; cells.push(u); wpTag.push(t);
      bfs(u, rad[u], function (v, dd) { if (dd <= rad[v]) covered[v] = 1; });
    }
    if (o.seed) { for (var j = 0; j < d.junc.length; j++) { if (!covered[d.junc[j]]) add(d.junc[j], 1); } }
    for (var u = 0; u < n; u++) { if (!grid[u] && !covered[u]) add(u, tag[u]); }
    computeOwner();
  }

  // the plain greedy pass (the planner's), only the list of waypoints
  function uniformList(C) {
    var covered = new Uint8Array(n), list = [];
    for (var u = 0; u < n; u++) {
      if (grid[u] || covered[u]) continue;
      list.push(u);
      bfs(u, C, function (v) { covered[v] = 1; });
    }
    return list;
  }

  function dynStats(extra) {
    var R = +$("r").value, C = +$("c").value, cnt = [0, 0, 0, 0, 0, 0], i;
    for (i = 0; i < wpTag.length; i++) cnt[wpTag[i]]++;
    var list = [
      ["Map", map.rows + " x " + map.cols],
      ["Free cells", map.free.toLocaleString()],
      ["Dynamic waypoints", cells.length.toLocaleString() + " (uniform with C = " + C + ": " + uniformCount(C).toLocaleString() + ")"],
      ["  junction points", cnt[1].toLocaleString()],
      ["  dense near junctions / at thin walls", cnt[2].toLocaleString() + " / " + cnt[3].toLocaleString()],
      ["  sparse in open areas / in corridors", cnt[4].toLocaleString() + " / " + cnt[5].toLocaleString()],
      ["  default", cnt[0].toLocaleString()],
      ["Free cells per waypoint", (map.free / Math.max(1, cells.length)).toFixed(1)],
      ["Farthest cell from a waypoint", maxCover + " steps"],
      ["Waypoint data per goal", ((cells.length * 2 + (2 * R + 1) * (2 * R + 1) * 2) / 1000).toFixed(0) + " KB (about)"]
    ];
    showStats(extra ? list.concat(extra) : list);
  }
  var ucCache = {};
  function uniformCount(C) {
    var key = C + "," + stamp0;
    if (ucCache[key] === undefined) ucCache[key] = uniformList(C).length;
    return ucCache[key];
  }
  var stamp0 = 0;      // changes with the map, so the cached counts of another map are not used

  function showCoverStats(extra) {
    var rows2 = [];
    if (extraEst) rows2 = rows2.concat(extraEst);
    if (extraMeasure) rows2 = rows2.concat(extraMeasure);
    if (extra) rows2 = rows2.concat(extra);
    if (mode === "dyn" && wpTag) dynStats(rows2.length ? rows2 : null); else coverStats(rows2.length ? rows2 : null);
  }

  // How far above the true distance is the waypoint estimate? Same rule as the planner: exact inside the window of R steps around the
  // goal, otherwise the smallest (exact steps from the cell to a waypoint whose window holds it) + (steps from that waypoint to the goal,
  // by a shortest-path run over the waypoint graph that starts from the waypoints inside the goal's window).
  // Also `walks` starts per goal are walked downhill on the estimate, the way PIBT moves a robot without other robots in the way:
  // does the walk reach the goal, and how much longer is it than the shortest path (the planner's own check, WP_CHECK).
  function evalSet(list, R, goals, starts, seed, walks) {
    var nW = list.length, i, k, wpLoc = new Int32Array(n).fill(-1);
    for (k = 0; k < nW; k++) wpLoc[list[k]] = k;
    var cs = new Int32Array(n + 1);
    for (k = 0; k < nW; k++) bfs(list[k], R, function (u) { cs[u + 1]++; });
    for (i = 0; i < n; i++) cs[i + 1] += cs[i];
    var ew = new Int32Array(cs[n]), ed = new Uint16Array(cs[n]), fp = cs.slice(0, n), aA = [], aB = [], aD = [];
    for (k = 0; k < nW; k++) {
      (function (kk) {
        bfs(list[kk], R, function (u, dd) {
          var p = fp[u]++; ew[p] = kk; ed[p] = dd;
          var o = wpLoc[u]; if (o >= 0 && o !== kk) { aA.push(kk); aB.push(o); aD.push(dd); }
        });
      })(k);
    }
    var as = new Int32Array(nW + 1);
    for (i = 0; i < aA.length; i++) as[aA[i] + 1]++;
    for (i = 0; i < nW; i++) as[i + 1] += as[i];
    var at = new Int32Array(aA.length), al = new Int32Array(aA.length), af = as.slice(0, nW);
    for (i = 0; i < aA.length; i++) { var pp = af[aA[i]]++; at[pp] = aB[i]; al[pp] = aD[i]; }
    var INF = 1e9, T = new Int32Array(n), D = new Float64Array(nW);
    var rng = seed || 12345, free = [], overs = [], wOk = 0, wAll = 0, wStretch = 0, wStretchMax = 0, gRow = 0, gCol = 0;
    for (i = 0; i < n; i++) if (!grid[i]) free.push(i);
    function rnd() { rng = (rng * 1103515245 + 12345) & 0x7fffffff; return rng / 2147483648; }
    var sum = 0, cntP = 0, mx = 0, bad = 0, missing = 0, hk = [], hv = [];
    function hpush(key, v) {
      hk.push(key); hv.push(v); var x = hk.length - 1;
      while (x > 0) { var pr = (x - 1) >> 1; if (hk[pr] <= hk[x]) break; var a = hk[pr]; hk[pr] = hk[x]; hk[x] = a; a = hv[pr]; hv[pr] = hv[x]; hv[x] = a; x = pr; }
    }
    function hpop() {
      var top = hv[0], lk = hk.pop(), lv = hv.pop();
      if (hk.length) {
        hk[0] = lk; hv[0] = lv; var x = 0, L = hk.length;
        for (;;) {
          var l = 2 * x + 1, r = l + 1, m = x;
          if (l < L && hk[l] < hk[m]) m = l;
          if (r < L && hk[r] < hk[m]) m = r;
          if (m === x) break;
          var a = hk[m]; hk[m] = hk[x]; hk[x] = a; a = hv[m]; hv[m] = hv[x]; hv[x] = a; x = m;
        }
      }
      return top;
    }
    function estAt(u) {
      var tt = T[u];
      if (tt >= 0 && tt <= R) return tt;
      var best = INF;
      for (var q2 = cs[u]; q2 < cs[u + 1]; q2++) { var dq = D[ew[q2]]; if (dq < INF && ed[q2] + dq < best) best = ed[q2] + dq; }
      if (best >= INF) { var ur = (u / cols) | 0; return 30000 + Math.abs(ur - gRow) + Math.abs(u - ur * cols - gCol); }
      return best;
    }
    for (var gi = 0; gi < goals; gi++) {
      var goal = free[Math.floor(rnd() * free.length)];
      gRow = (goal / cols) | 0; gCol = goal - gRow * cols;
      T.fill(-1);
      bfs(goal, 30000, function (u, dd) { T[u] = dd; });
      D.fill(INF); hk.length = 0; hv.length = 0;
      for (k = 0; k < nW; k++) { var tk = T[list[k]]; if (tk >= 0 && tk <= R) { D[k] = tk; hpush(tk, k); } }
      while (hk.length) {
        var dk = hk[0], w = hpop();
        if (dk > D[w]) continue;
        for (var e = as[w]; e < as[w + 1]; e++) { var w2 = at[e], nd = dk + al[e]; if (nd < D[w2]) { D[w2] = nd; hpush(nd, w2); } }
      }
      for (var si = 0; si < starts; si++) {
        var s = free[Math.floor(rnd() * free.length)], tr = T[s];
        if (tr <= 0) continue;
        var est;
        if (tr <= R) est = tr;
        else { est = INF; for (var p2 = cs[s]; p2 < cs[s + 1]; p2++) { var dw = D[ew[p2]]; if (dw < INF && ed[p2] + dw < est) est = ed[p2] + dw; } }
        if (est >= INF) { missing++; continue; }
        var over = 100 * (est - tr) / tr;
        sum += over; cntP++; if (over > mx) mx = over; if (over > 10) bad++; overs.push(over);
      }
      for (var wi = 0; wi < (walks || 0); wi++) {
        var cur = free[Math.floor(rnd() * free.length)], trw = T[cur];
        if (trw <= 0) continue;
        var stp = 0, lim = 4 * trw + 50;
        while (cur !== goal && stp < lim) {
          var cr = (cur / cols) | 0, cc2 = cur - cr * cols, bv = -1, bval = 1e18, nbs = [cc2 > 0 ? cur - 1 : -1, cc2 < cols - 1 ? cur + 1 : -1, cr > 0 ? cur - cols : -1, cr < rows - 1 ? cur + cols : -1];
          for (var nk = 0; nk < 4; nk++) { var nv = nbs[nk]; if (nv < 0 || grid[nv]) continue; var val = 1 + estAt(nv); if (val < bval) { bval = val; bv = nv; } }
          if (bv < 0) break;
          cur = bv; stp++;
        }
        wAll++;
        if (cur === goal) { wOk++; var sr = stp / trw; wStretch += sr; if (sr > wStretchMax) wStretchMax = sr; }
      }
    }
    overs.sort(function (a, b) { return a - b; });
    return { n: nW, mean: cntP ? sum / cntP : 0, p95: overs.length ? overs[Math.floor(overs.length * 0.95)] : 0, max: mx, bad: cntP ? 100 * bad / cntP : 0, missing: missing, pairs: cntP,
             entries: cs[n], edges: aA.length, walks: wAll, succ: wAll ? 100 * wOk / wAll : 0, stretch: wOk ? wStretch / wOk : 0, stretchMax: wStretchMax };
  }
  function fmtEval(label, ev) {
    return [label, ev.n.toLocaleString() + " points: mean +" + ev.mean.toFixed(2) + "%, worst +" + ev.max.toFixed(0) + "%, " + ev.bad.toFixed(1) +
      "% of pairs over +10%" + (ev.missing ? ", " + ev.missing + " without estimate" : "") + "; walk to goal: " + ev.succ.toFixed(1) + "% reach it, " +
      "length x" + ev.stretch.toFixed(3) + "; " + (ev.entries / 1e6).toFixed(2) + " M entries, " + (ev.edges / 1e3).toFixed(0) + " k edges"];
  }
  function measureNow() {
    var R = +$("r").value, C = +$("c").value, goals = 20, starts = 150, walks = 40, rowsOut = [["Distance error", goals + " goals x " + starts + " starts and " + walks + " downhill walks each, no robots"]];
    var uni = uniformList(C);
    if (mode === "dyn" && wpTag) {
      rowsOut.push(fmtEval("Dynamic", evalSet(cells, R, goals, starts, 0, walks)));
      rowsOut.push(fmtEval("Uniform, C = " + C, evalSet(uni, R, goals, starts, 0, walks)));
      var bestC = C, bestDiff = Infinity;
      for (var cc = 2; cc <= Math.max(C + 6, 12); cc++) { var diff = Math.abs(uniformCount(cc) - cells.length); if (diff < bestDiff) { bestDiff = diff; bestC = cc; } }
      if (bestC !== C) rowsOut.push(fmtEval("Uniform, C = " + bestC + " (nearest count)", evalSet(uniformList(bestC), R, goals, starts, 0, walks)));
    } else {
      rowsOut.push(fmtEval("Uniform, C = " + C, evalSet(uni, R, goals, starts, 0, walks)));
    }
    return rowsOut;
  }

  function hsl(h, s, l) {
    s /= 100; l /= 100;
    var a = s * Math.min(l, 1 - l);
    function f(nn) { var k = (nn + h / 30) % 12; return Math.round(255 * (l - a * Math.max(-1, Math.min(k - 3, 9 - k, 1)))); }
    return [f(0), f(8), f(4)];
  }

  function coverageLayer() {
    if (!covLayer) {
      covLayer = view.cellLayer(function (u) {
        if (grid[u] || owner[u] < 0) return null;
        var c = hsl((owner[u] * 137.508) % 360, 55, 72);
        return [c[0], c[1], c[2], 255];
      });
    }
    return covLayer;
  }

  function showStats(rowsHtml) {
    $("stats").innerHTML = rowsHtml.map(function (r) { return "<dt>" + r[0] + "</dt><dd>" + r[1] + "</dd>"; }).join("");
  }
  function coverStats(extra) {
    var R = +$("r").value, C = +$("c").value;
    var list = [
      ["Map", map.rows + " x " + map.cols],
      ["Free cells", map.free.toLocaleString()],
      ["Waypoints", cells.length.toLocaleString()],
      ["Free cells per waypoint", (map.free / Math.max(1, cells.length)).toFixed(1)],
      ["Farthest cell from a waypoint", maxCover + " steps (limit C = " + C + ")"],
      ["Full table per goal", (n * 2 / 1000).toFixed(0) + " KB (one entry per map cell)"],
      ["Waypoint data per goal", ((cells.length * 2 + (2 * R + 1) * (2 * R + 1) * 2) / 1000).toFixed(0) + " KB (about)"]
    ];
    showStats(extra ? list.concat(extra) : list);
  }

  var COVER_HELP = "Click a cell: the waypoint nearest to it is selected, with its window and the waypoints it connects to.";
  function selectCover(wp) {
    if (wp == null || wp < 0) { sel = null; selLayer = null; $("info").textContent = COVER_HELP; view.draw(); return; }
    var R = +$("r").value, win = [], dd = [], neigh = [];
    bfs(cells[wp], R, function (u, d) { win.push(u); dd.push(d); if (wpOf[u] >= 0 && wpOf[u] !== wp) neigh.push(wpOf[u]); });
    var dm = new Map();
    for (var i = 0; i < win.length; i++) dm.set(win[i], dd[i]);
    sel = { wp: wp, window: win, dist: dm, neigh: neigh };
    selLayer = view.cellLayer(function (u) {
      if (!dm.has(u)) return null;
      return [25, 113, 194, 40 + Math.round(90 * (1 - dm.get(u) / (R + 1)))];
    });
    var r = (cells[wp] / cols) | 0, c = cells[wp] % cols;
    $("info").textContent = "Waypoint " + wp + " at row " + r + ", column " + c + ". Its window of " + R + " steps holds " + win.length +
      " cells and " + neigh.length + " other waypoints (the graph edges at this waypoint).";
    view.draw();
  }

  // ---------- the supervisor's skeleton ----------
  var SKEL_HELP = "Click a start cell and then a goal cell: the route follows the skeleton, and the waypoints are every step-th cell of it, counted from the goal.";
  function buildSkeleton() {
    if (!map) return;
    var step = +$("step").value;
    $("sv").textContent = step;
    showStats([["Working ...", ""]]);
    setTimeout(function () {
      var t0 = performance.now();
      skel = new Skeleton(map, step);
      skel.build();
      skelLayer = view.cellLayer(function (u) { return skel.rm[u] > 0 ? [18, 184, 134, 255] : null; });
      var ms = performance.now() - t0;
      showStats([
        ["Map", map.rows + " x " + map.cols],
        ["Free cells", map.free.toLocaleString()],
        ["Skeleton cells", skel.cells.toLocaleString() + " (" + (100 * skel.cells / Math.max(1, map.free)).toFixed(1) + "% of the free cells)"],
        ["Thinning rounds", skel.rounds],
        ["Waypoint step", step + " cells"],
        ["Time in the browser", (ms / 1000).toFixed(2) + " s"]
      ]);
      if (route && route.start != null && route.goal != null) computeRoute();
      view.draw();
    }, 20);
  }

  function computeRoute() {
    var s = skel.nearest(route.start), g = skel.nearest(route.goal);
    route.sNear = s; route.gNear = g;
    var res = (s && g) ? skel.route(s.cell, g.cell) : null;
    route.path = res ? res.path : null;
    route.wps = res ? res.waypoints : null;
    if (!res) { $("info").textContent = "No route on the skeleton between these two cells."; return; }
    var rc = function (u) { return "row " + ((u / cols) | 0) + ", col " + (u % cols); };
    $("info").textContent = "Start joins the skeleton at " + rc(s.cell) + " (" + s.steps + " steps away), the goal at " + rc(g.cell) + " (" + g.steps +
      " steps away). Route on the skeleton: " + res.path.length + " cells, " + res.waypoints.length + " waypoints.";
  }

  // ---------- clusters from base points (the area method of Team Hipparcos, 2024) ----------
  // Base points: either the junctions of the skeleton (crossings first, then T junctions, optionally dead ends), a junction being skipped
  // when it is closer than the minimum distance to a base point already chosen; or, as Team Hipparcos does it, one random free cell near
  // every node of a uniform grid. Areas: all base points grow at the same time, round d lets every base point take the cells it reaches
  // with cost <= d (a step costs 1, a step that turns costs 2), a cell belongs to the base point that takes it first.
  var CL_HELP = "Click a cell: its cluster is selected, with the base point it grows from and the clusters next to it.";
  var areaLayer = null;

  function skelNbrs(rm, u, out) {
    var r = (u / cols) | 0, c = u - r * cols, k = 0;
    if (c > 0 && rm[u - 1] > 0) out[k++] = u - 1;
    if (c < cols - 1 && rm[u + 1] > 0) out[k++] = u + 1;
    if (r > 0 && rm[u - cols] > 0) out[k++] = u - cols;
    if (r < rows - 1 && rm[u + cols] > 0) out[k++] = u + cols;
    return k;
  }

  function lcg(seed) { var x = seed >>> 0; return function () { x = (Math.imul(x, 1664525) + 1013904223) >>> 0; return x / 4294967296; }; }

  function junctionBasePoints(rm, minD, deadEnds, info) {
    var tmp = [0, 0, 0, 0], lists = [[], [], []], u;
    for (u = 0; u < n; u++) {
      if (rm[u] <= 0) continue;
      var k = skelNbrs(rm, u, tmp);
      if (k === 4) lists[0].push(u); else if (k === 3) lists[1].push(u); else if (k <= 1) lists[2].push(u);
    }
    info.cross = lists[0].length; info.tee = lists[1].length; info.dead = lists[2].length;
    var blocked = new Uint8Array(n), bp = [];
    function tryAdd(c) {
      if (blocked[c]) return;
      bp.push(c);
      bfs(c, minD - 1, function (v) { blocked[v] = 1; });
    }
    lists[0].forEach(tryAdd); lists[1].forEach(tryAdd);
    if (deadEnds) lists[2].forEach(tryAdd);
    return bp;
  }

  function uniformBasePoints(distance) {
    var rnd = lcg(12345), r3 = Math.floor(distance / 3), bp = [];
    for (var i = distance >> 1; i < rows; i += distance) {
      for (var j = distance >> 1; j < cols; j += distance) {
        var cand = [];
        for (var ii = Math.max(i - r3, 0); ii < Math.min(i + r3, rows); ii++)
          for (var jj = Math.max(j - r3, 0); jj < Math.min(j + r3, cols); jj++)
            if (!grid[ii * cols + jj]) cand.push(ii * cols + jj);
        if (cand.length) bp.push(cand[(rnd() * cand.length) | 0]);
      }
    }
    return bp;
  }

  var HM = 4194304;                       // heap key = cost * HM + node index
  function hPush(h, key) {
    h.push(key); var i = h.length - 1;
    while (i > 0) { var p = (i - 1) >> 1; if (h[p] <= h[i]) break; var t = h[p]; h[p] = h[i]; h[i] = t; i = p; }
  }
  function hPop(h) {
    var top = h[0], last = h.pop();
    if (h.length) {
      h[0] = last; var i = 0;
      for (;;) { var l = 2 * i + 1, r = l + 1, m = i; if (l < h.length && h[l] < h[m]) m = l; if (r < h.length && h[r] < h[m]) m = r; if (m === i) break; var t = h[m]; h[m] = h[i]; h[i] = t; i = m; }
    }
    return top;
  }

  // the area growth of Team Hipparcos (MapUtils::createAreasAroundBasePoints): returns the cluster of every cell and the cost at which it was taken
  function growAreas(bp) {
    var B = bp.length, area = new Int32Array(n).fill(-1), taken = new Int16Array(n);
    var H = [], S = [], i;
    for (i = 0; i < B; i++) {
      var st = { loc: [bp[i]], par: [-1], g: [0], closed: [0], map: new Map() };
      st.map.set(bp[i], 0);
      H.push([0]); S.push(st);
    }
    var d = 0, running = true, cand = [0, 0, 0, 0];
    while (running) {
      running = false;
      for (i = 0; i < B; i++) {
        var h = H[i];
        if (!h.length) continue;
        running = true;
        var s = S[i];
        while (h.length) {
          var key = h[0], idx = key % HM, gk = (key - idx) / HM;
          if (s.closed[idx] || gk > s.g[idx]) { hPop(h); continue; }
          if (gk > d) break;
          hPop(h);
          s.closed[idx] = 1;
          var loc = s.loc[idx], par = s.par[idx], g = s.g[idx], row = (loc / cols) | 0, col = loc - row * cols;
          cand[0] = col + 1 < cols ? loc + 1 : -1; cand[1] = row + 1 < rows ? loc + cols : -1; cand[2] = col > 0 ? loc - 1 : -1; cand[3] = row > 0 ? loc - cols : -1;
          for (var q = 0; q < 4; q++) {
            var c = cand[q];
            if (c < 0 || grid[c] || c === par) continue;
            var cost = (par < 0 || c === loc + (loc - par)) ? 1 : 2;
            var j = s.map.get(c);
            if (j !== undefined) {
              if (s.closed[j]) continue;
              if (g + cost < s.g[j]) { s.g[j] = g + cost; s.par[j] = loc; hPush(h, (g + cost) * HM + j); }
            } else if (area[c] < 0) {
              j = s.loc.length; s.loc.push(c); s.par.push(loc); s.g.push(g + cost); s.closed.push(0); s.map.set(c, j);
              hPush(h, (g + cost) * HM + j);
            }
          }
          if (area[loc] < 0) { area[loc] = i; taken[loc] = g; }
        }
      }
      d++;
    }
    return { area: area, taken: taken };
  }

  function buildGraph() {
    if (!map) return;
    var uniform = $("g-base").value === "uniform", step = +$("gstep").value, minD = +$("gmin").value, space = +$("gspace").value, deadEnds = $("gdead").checked;
    $("gsv").textContent = step; $("gmv").textContent = minD; $("gpv").textContent = space;
    $("g-junc-opts").hidden = uniform; $("g-uni-opts").hidden = !uniform;
    showStats([["Working ...", ""]]);
    setTimeout(function () {
      var t0 = performance.now(), info = { cross: 0, tee: 0, dead: 0 }, bp, i, u;
      if (!uniform) {
        if (!gskel || gskel.step !== step) {
          if (skel && skel.step === step) gskel = skel; else { gskel = new Skeleton(map, step); gskel.build(); }
          gLayer = null;
        }
        bp = junctionBasePoints(gskel.rm, minD, deadEnds, info);
      } else bp = uniformBasePoints(space);
      var res = growAreas(bp), B = bp.length, area = res.area, size = new Int32Array(B), far = new Int16Array(B), none = 0;
      for (u = 0; u < n; u++) {
        if (grid[u]) continue;
        var a = area[u];
        if (a < 0) { none++; continue; }
        size[a]++; if (res.taken[u] > far[a]) far[a] = res.taken[u];
      }
      // clusters that touch: pairs of neighbouring cells with different clusters
      var pairSet = new Map(), nbList = [];
      for (i = 0; i < B; i++) nbList.push([]);
      function touch(a, b) {
        if (a < 0 || b < 0 || a === b) return;
        if (a > b) { var t = a; a = b; b = t; }
        var k = a * B + b;
        if (!pairSet.has(k)) { pairSet.set(k, 1); nbList[a].push(b); nbList[b].push(a); }
      }
      for (u = 0; u < n; u++) {
        if (grid[u]) continue;
        var rr = (u / cols) | 0, cc = u - rr * cols;
        if (cc < cols - 1 && !grid[u + 1]) touch(area[u], area[u + 1]);
        if (rr < rows - 1 && !grid[u + cols]) touch(area[u], area[u + cols]);
      }
      var sizes = Array.prototype.slice.call(size).sort(function (x, y) { return x - y; });
      var sizeSum = 0, farMax = 0, nbSum = 0, nbMax = 0;
      for (i = 0; i < B; i++) { sizeSum += size[i]; if (far[i] > farMax) farMax = far[i]; nbSum += nbList[i].length; if (nbList[i].length > nbMax) nbMax = nbList[i].length; }
      var pairBytes = B * B * 4, buildMs = performance.now() - t0;
      G = { bp: Int32Array.from(bp), area: area, size: size, far: far, nb: nbList, count: B, stats: null };
      areaLayer = null;
      var rows2 = [["Map", map.rows + " x " + map.cols], ["Free cells", map.free.toLocaleString()]];
      if (!uniform) rows2.push(["Skeleton cells", gskel.cells.toLocaleString() + " (" + (100 * gskel.cells / Math.max(1, map.free)).toFixed(1) + "% of the free cells)"],
                               ["Crossings / T junctions / dead ends", info.cross.toLocaleString() + " / " + info.tee.toLocaleString() + " / " + info.dead.toLocaleString()]);
      rows2.push(
        ["Base points (= clusters)", B.toLocaleString() + (uniform ? " (one per " + space + " x " + space + " cells of the grid)" : " (at least " + minD + " steps apart)")],
        ["Cluster size, cells: smallest / median / mean / largest", (B ? sizes[0] + " / " + sizes[B >> 1] + " / " + (sizeSum / B).toFixed(1) + " / " + sizes[B - 1] : "-")],
        ["Farthest cell from its base point", farMax + " (cost: a step 1, a turn 2)"],
        ["Neighbour clusters: mean / most", (B ? (nbSum / B).toFixed(1) : "0") + " / " + nbMax],
        ["Cells in no cluster", none.toLocaleString() + (none ? " (no base point in their part of the map)" : "")],
        ["Pair table of the base points", (pairBytes >= 1e9 ? (pairBytes / 1e9).toFixed(1) + " GB" : (pairBytes / 1e6).toFixed(1) + " MB") + " (B x B x 4 bytes, before the route lists)"],
        ["Build time in the browser", (buildMs / 1000).toFixed(2) + " s"]);
      G.stats = rows2;
      gSel = null;
      if (mode === "graph") { showStats(G.stats); $("info").textContent = CL_HELP; }
      view.draw();
    }, 20);
  }

  function gSkelLayer() {
    if (!gLayer) gLayer = view.cellLayer(function (u) { return gskel && gskel.rm[u] > 0 ? [18, 184, 134, 130] : null; });
    return gLayer;
  }
  function clusterLayer() {
    if (!areaLayer) {
      areaLayer = view.cellLayer(function (u) {
        if (grid[u]) return null;
        var a = G.area[u];
        if (a < 0) return [255, 201, 201, 255];
        var c = hsl((a * 137.508) % 360, 55, 72);
        return [c[0], c[1], c[2], 255];
      });
    }
    return areaLayer;
  }

  function selectCluster(a) {
    if (a < 0) { gSel = null; $("info").textContent = "This cell is in no cluster: there is no base point in this part of the map."; view.draw(); return; }
    var k = G.bp[a], r = (k / cols) | 0, c = k % cols;
    gSel = { a: a, nb: G.nb[a], layer: view.cellLayer(function (u) { return !grid[u] && G.area[u] === a ? [25, 113, 194, 150] : null; }) };
    $("info").textContent = "Cluster " + a + ": base point at row " + r + ", column " + c + ", " + G.size[a].toLocaleString() + " cells, farthest cell at cost " + G.far[a] +
      ", " + G.nb[a].length + " neighbour clusters.";
    view.draw();
  }

  // ---------- drawing ----------
  view.layers.push(function (ctx) {
    if (!map) return;
    if (isCover()) {
      if ($("show-cov").checked && owner) ctx.drawImage(coverageLayer(), 0, 0);
      if (selLayer) ctx.drawImage(selLayer, 0, 0);
    } else if (mode === "graph") {
      if (G && $("g-areas").checked) ctx.drawImage(clusterLayer(), 0, 0);
      if (G && gskel && $("g-skel").checked && $("g-base").value !== "uniform") ctx.drawImage(gSkelLayer(), 0, 0);
      if (G && gSel) ctx.drawImage(gSel.layer, 0, 0);
    } else if (skelLayer && $("show-skel").checked) {
      ctx.drawImage(skelLayer, 0, 0);
    }
  });
  view.layers.push(function (ctx, v) {
    if (!map) return;
    var rad = Math.max(0.35, 2.4 / v.scale);
    function px(u) { return (u % cols) + 0.5; }
    function py(u) { return ((u / cols) | 0) + 0.5; }
    function dot(u, r, color) { ctx.fillStyle = color; ctx.beginPath(); ctx.arc(px(u), py(u), r, 0, 6.2832); ctx.fill(); }
    if (isCover()) {
      if ($("show-wp").checked) {
        if (mode === "dyn" && wpTag) {
          for (var tg = 0; tg < 6; tg++) {
            ctx.fillStyle = DYN_COLOR[tg]; ctx.beginPath();
            var rr = tg === 1 ? rad * 1.5 : rad;
            for (var k2 = 0; k2 < cells.length; k2++) { if (wpTag[k2] !== tg) continue; var x2 = px(cells[k2]), y2 = py(cells[k2]); ctx.moveTo(x2 + rr, y2); ctx.arc(x2, y2, rr, 0, 6.2832); }
            ctx.fill();
          }
        } else {
          ctx.fillStyle = "#d6336c";
          ctx.beginPath();
          for (var k = 0; k < cells.length; k++) { var x = px(cells[k]), y = py(cells[k]); ctx.moveTo(x + rad, y); ctx.arc(x, y, rad, 0, 6.2832); }
          ctx.fill();
        }
      }
      if (sel) {
        var sx = px(cells[sel.wp]), sy = py(cells[sel.wp]);
        ctx.strokeStyle = "#f08c00"; ctx.lineWidth = Math.max(0.12, 1.2 / v.scale);
        ctx.beginPath();
        sel.neigh.forEach(function (w) { ctx.moveTo(sx, sy); ctx.lineTo(px(cells[w]), py(cells[w])); });
        ctx.stroke();
        ctx.fillStyle = "#f08c00";
        ctx.beginPath();
        sel.neigh.forEach(function (w) { var x = px(cells[w]), y = py(cells[w]); ctx.moveTo(x + rad * 1.5, y); ctx.arc(x, y, rad * 1.5, 0, 6.2832); });
        ctx.fill();
        dot(cells[sel.wp], rad * 2.2, "#1971c2");
      }
      return;
    }
    if (mode === "graph") {
      if (!G) return;
      if ($("g-nodes").checked) {
        ctx.fillStyle = "#c2255c"; ctx.beginPath();
        for (var g = 0; g < G.count; g++) { var x = px(G.bp[g]), y = py(G.bp[g]); ctx.moveTo(x + rad, y); ctx.arc(x, y, rad, 0, 6.2832); }
        ctx.fill();
      }
      if (gSel) {
        var sx = px(G.bp[gSel.a]), sy = py(G.bp[gSel.a]);
        ctx.strokeStyle = "#f08c00"; ctx.lineWidth = Math.max(0.2, 2 / v.scale);
        ctx.beginPath();
        gSel.nb.forEach(function (w) { ctx.moveTo(sx, sy); ctx.lineTo(px(G.bp[w]), py(G.bp[w])); });
        ctx.stroke();
        gSel.nb.forEach(function (w) { dot(G.bp[w], rad * 1.8, "#f08c00"); });
        dot(G.bp[gSel.a], rad * 2.4, "#ffffff"); dot(G.bp[gSel.a], rad * 1.8, "#1971c2");
      }
      return;
    }
    if (!route || !$("show-route").checked) return;
    if (route.path) {
      ctx.strokeStyle = "#f08c00"; ctx.lineWidth = Math.max(0.2, 2 / v.scale); ctx.lineJoin = "round";
      ctx.beginPath();
      route.path.forEach(function (u, i) { if (i) ctx.lineTo(px(u), py(u)); else ctx.moveTo(px(u), py(u)); });
      ctx.stroke();
      route.wps.forEach(function (u) { dot(u, rad * 1.8, "#ffffff"); dot(u, rad * 1.2, "#d6336c"); });
    }
    ctx.strokeStyle = "#555"; ctx.lineWidth = Math.max(0.1, 1 / v.scale); ctx.setLineDash([0.4, 0.4]);
    if (route.sNear && route.start != null) { ctx.beginPath(); ctx.moveTo(px(route.start), py(route.start)); ctx.lineTo(px(route.sNear.cell), py(route.sNear.cell)); ctx.stroke(); }
    if (route.gNear && route.goal != null) { ctx.beginPath(); ctx.moveTo(px(route.goal), py(route.goal)); ctx.lineTo(px(route.gNear.cell), py(route.gNear.cell)); ctx.stroke(); }
    ctx.setLineDash([]);
    if (route.start != null) dot(route.start, rad * 2, "#2f9e44");
    if (route.goal != null) dot(route.goal, rad * 2, "#e03131");
  });

  // ---------- controls ----------
  function recompute() {
    if (!map) return;
    var R = +$("r").value, C = +$("c").value;
    $("rv").textContent = R; $("cv").textContent = C;
    if (isCover()) showStats([["Working ...", ""]]);
    setTimeout(function () {
      if (mode === "dyn") placeDyn(C, R); else place(C);
      sel = null; selLayer = null; extraEst = null; extraMeasure = null;
      if (isCover()) showCoverStats();
      view.draw();
    }, 20);
  }
  function later() { clearTimeout(timer); timer = setTimeout(recompute, 150); $("rv").textContent = $("r").value; $("cv").textContent = $("c").value; }

  $("r").addEventListener("input", function () {
    if ($("auto").checked) { $("c").value = Math.max(1, Math.floor((+$("r").value - 1) / 2)); }
    $("c").max = $("r").value;
    if (+$("c").value > +$("r").value) $("c").value = $("r").value;
    later();
  });
  $("c").addEventListener("input", function () { $("auto").checked = false; if (+$("c").value > +$("r").value) $("c").value = $("r").value; later(); });
  $("auto").addEventListener("change", function () { if ($("auto").checked) { $("c").value = Math.max(1, Math.floor((+$("r").value - 1) / 2)); later(); } });
  ["show-cov", "show-wp", "show-skel", "show-route", "g-skel", "g-areas", "g-nodes"].forEach(function (id) { $(id).addEventListener("change", function () { view.draw(); }); });
  function laterGraph() { clearTimeout(timer3); timer3 = setTimeout(buildGraph, 200); }
  ["gstep", "gmin", "gspace"].forEach(function (id) { $(id).addEventListener("input", function () { $({ gstep: "gsv", gmin: "gmv", gspace: "gpv" }[id]).textContent = $(id).value; laterGraph(); }); });
  ["gdead", "g-base"].forEach(function (id) { $(id).addEventListener("change", buildGraph); });
  var dynLabels = { "d-zone": "dzv", "d-juncp": "djv", "d-wallp": "dwv", "d-opencl": "dcv", "d-openp": "dov", "d-corrp": "dkv" };
  function dynLabel(id) { var v = $(id).value; $(dynLabels[id]).textContent = (id === "d-zone" || id === "d-opencl") ? v : v + "%"; }
  Object.keys(dynLabels).forEach(function (id) { dynLabel(id); $(id).addEventListener("input", function () { dynLabel(id); if (mode === "dyn") later(); }); });
  ["d-seed", "d-wall"].forEach(function (id) { $(id).addEventListener("change", function () { if (mode === "dyn") later(); }); });
  $("step").addEventListener("input", function () { $("sv").textContent = $("step").value; clearTimeout(timer2); timer2 = setTimeout(buildSkeleton, 200); });
  $("clear-route").addEventListener("click", function () { route = null; $("info").textContent = SKEL_HELP; view.draw(); });
  $("zin").addEventListener("click", function () { view.zoomCenter(1.5); });
  $("zout").addEventListener("click", function () { view.zoomCenter(1 / 1.5); });
  $("fit").addEventListener("click", function () { view.fit(); });

  Array.prototype.forEach.call(document.getElementsByName("mode"), function (rb) {
    rb.addEventListener("change", function () {
      if (!rb.checked) return;
      var wasDyn = mode === "dyn";
      mode = rb.value;
      if (mode === "dyn" && $("auto").checked) {      // the default C = (R - 1) / 2 leaves no room for sparser areas: start from R / 3
        $("auto").checked = false; $("c").value = Math.max(2, Math.floor(+$("r").value / 3)); $("cv").textContent = $("c").value;
      }
      $("cover-panel").hidden = !isCover();
      $("dyn-panel").hidden = mode !== "dyn";
      $("dyn-legend").hidden = mode !== "dyn";
      $("measure").hidden = !isCover();
      $("sup-panel").hidden = mode !== "sup";
      $("graph-panel").hidden = mode !== "graph";
      $("cover-legend").hidden = mode !== "cover";
      $("sup-legend").hidden = mode !== "sup";
      $("graph-legend").hidden = mode !== "graph";
      $("estimate").hidden = !isCover();
      $("csv").textContent = isCover() ? "Download waypoints (.csv)" : mode === "graph" ? "Download base points (.csv)" : "Download skeleton cells (.csv)";
      $("info").textContent = isCover() ? COVER_HELP : mode === "graph" ? CL_HELP : SKEL_HELP;
      if (!map) return;
      if (isCover() && (mode === "dyn" || wasDyn)) { sel = null; selLayer = null; recompute(); return; }
      if (mode === "cover") showCoverStats();
      else if (mode === "graph") { if (!G) buildGraph(); else showStats(G.stats); }
      else if (!skel) buildSkeleton();
      else { showStats([["Map", map.rows + " x " + map.cols], ["Free cells", map.free.toLocaleString()],
        ["Skeleton cells", skel.cells.toLocaleString() + " (" + (100 * skel.cells / Math.max(1, map.free)).toFixed(1) + "% of the free cells)"],
        ["Thinning rounds", skel.rounds], ["Waypoint step", skel.step + " cells"]]); }
      view.draw();
    });
  });

  // how much data the waypoint distance needs: the windows of all waypoints (cell entries) and the graph between waypoints
  $("estimate").addEventListener("click", function () {
    if (!map) return;
    var R = +$("r").value, entries = 0, edges = 0;
    $("estimate").textContent = "Working ...";
    setTimeout(function () {
      for (var k = 0; k < cells.length; k++) {
        bfs(cells[k], R, function (u) { entries++; if (wpOf[u] >= 0 && wpOf[u] !== k) edges++; });
      }
      var mb = ((entries * 6 + edges * 6 + cells.length * 8 + n * 8) / 1e6);
      extraEst = [["Cell-to-waypoint entries", entries.toLocaleString()], ["Waypoint graph edges", edges.toLocaleString()], ["Shared data (once per map)", mb.toFixed(1) + " MB (about)"]];
      showCoverStats();
      $("estimate").textContent = "Estimate data size";
    }, 20);
  });
  $("measure").addEventListener("click", function () {
    if (!map || !cells.length) return;
    $("measure").textContent = "Working ...";
    setTimeout(function () {
      extraMeasure = measureNow();
      showCoverStats();
      $("measure").textContent = "Measure distance error";
    }, 20);
  });
  $("csv").addEventListener("click", function () {
    if (!map) return;
    var out = ["id,cell,row,col"], list = [];
    if (isCover()) { list = cells; if (mode === "dyn" && wpTag) out = ["id,cell,row,col,rule"]; }
    else if (mode === "graph") {
      if (!G) return;
      out = ["id,cell,row,col,cluster_cells"];
      for (var g = 0; g < G.count; g++) out.push(g + "," + G.bp[g] + "," + ((G.bp[g] / cols) | 0) + "," + (G.bp[g] % cols) + "," + G.size[g]);
    }
    else if (skel) { for (var u = 0; u < n; u++) if (skel.rm[u] > 0) list.push(u); }
    for (var k = 0; k < list.length; k++) out.push(k + "," + list[k] + "," + ((list[k] / cols) | 0) + "," + (list[k] % cols) + (mode === "dyn" && wpTag ? "," + DYN_NAME[wpTag[k]] : ""));
    var a = document.createElement("a");
    a.href = URL.createObjectURL(new Blob([out.join("\n") + "\n"], { type: "text/csv" }));
    a.download = mode === "dyn" ? "dynamic_waypoints.csv" : mode === "cover" ? "waypoints.csv" : mode === "graph" ? "base_points.csv" : "skeleton.csv";
    a.click();
  });

  view.onClick = function (cell) {
    if (!cell || !map || grid[cell.u]) return;
    if (isCover()) { selectCover(wpOf[cell.u] >= 0 ? wpOf[cell.u] : owner[cell.u]); return; }
    if (mode === "graph") {
      if (G) selectCluster(G.area[cell.u]);
      return;
    }
    if (!skel) return;
    if (!route || route.goal != null) { route = { start: cell.u, goal: null }; $("info").textContent = "Start set. Now click the goal."; view.draw(); return; }
    route.goal = cell.u;
    computeRoute();
    view.draw();
  };
  view.onMove = function (cell) {
    var h = $("hover");
    if (!cell || !map) { h.hidden = true; return; }
    h.hidden = false;
    var extra = grid[cell.u] ? " (obstacle)" : "";
    if (!extra && isCover() && wpOf[cell.u] >= 0) extra = " (waypoint " + wpOf[cell.u] + (mode === "dyn" && wpTag ? ", " + DYN_NAME[wpTag[wpOf[cell.u]]] : "") + ")";
    if (!extra && mode === "sup" && skel && skel.rm[cell.u] > 0) extra = " (skeleton)";
    if (!extra && mode === "graph" && G) {
      var ga = G.area[cell.u];
      extra = ga < 0 ? " (no cluster)" : (G.bp[ga] === cell.u ? " (base point " + ga + ")" : " (cluster " + ga + ")");
    }
    h.textContent = "row " + cell.r + ", col " + cell.c + extra;
  };

  var loadSample = MapTools.setupLoader({
    file: $("file"), drop: $("drop"), sample: $("sample"), status: $("status"),
    onMap: function (m) {
      map = m; rows = m.rows; cols = m.cols; n = rows * cols; grid = m.grid;
      seen = new Int32Array(n); dist = new Int16Array(n); queue = new Int32Array(n); stamp = 0;
      skel = null; skelLayer = null; route = null; sel = null; selLayer = null;
      gskel = null; gLayer = null; G = null; gSel = null; areaLayer = null;
      dyn = null; wpTag = null; extraEst = null; extraMeasure = null; ucCache = {}; stamp0++;
      view.setMap(m);
      recompute();
      if (mode === "sup") buildSkeleton();
      if (mode === "graph") buildGraph();
    }
  });
  if (/[?&]test/.test(location.search)) {        // batch hook for the experiments: window.WP_TEST.run(spec)
    window.WP_TEST = { run: function (s) {
      var t0 = performance.now(), list, tags = null, o = null;
      if (s.kind === "dyn") {
        o = { seed: s.seed !== false, zone: s.zone || 0, juncP: (s.juncP || 100) / 100, wall: !!s.wall, wallP: (s.wallP || 100) / 100,
              openCl: s.openCl || 4, openP: (s.openP || 100) / 100, corrP: (s.corrP || 100) / 100, twMin: 25 };
        placeDyn(s.C, s.R, o); list = cells.slice(); tags = [0, 0, 0, 0, 0, 0];
        for (var i = 0; i < wpTag.length; i++) tags[wpTag[i]]++;
      } else { list = uniformList(s.C); }
      var ev = evalSet(list, s.R, s.goals || 50, s.starts || 300, s.evalSeed || 777, s.walks === undefined ? 100 : s.walks);
      ev.tags = tags; ev.ms = Math.round(performance.now() - t0);
      return ev;
    } };
  }
  $("rv").textContent = $("r").value; $("cv").textContent = $("c").value; $("sv").textContent = $("step").value;
  $("gsv").textContent = $("gstep").value; $("gmv").textContent = $("gmin").value; $("gpv").textContent = $("gspace").value;
  loadSample();
})();

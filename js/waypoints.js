/* Waypoint tool. Four views of a map:
   - cover points: the waypoints of the PIBT planner (src/PIBT/Waypoints.cpp, plain steps) that its waypoint distance is built on;
   - the supervisor's skeleton and route waypoints (js/skeleton.js, ported from his SIPP planner);
   - the skeleton turned into a graph: nodes are its dead ends and junctions, edges are the corridors between them;
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

  // ---------- the skeleton as a graph ----------
  // Nodes: skeleton cells with 1 neighbour (dead end), 3 (T junction) or 4 (crossing) skeleton neighbours; optionally the bends (2 neighbours,
  // not in a line) too. Edges: the chains of 2-neighbour cells between two nodes, with their length as cost.
  // Merging: nodes joined by an edge of at most k steps become one node (placed at the member nearest to the group's centre).
  var GRAPH_HELP = "Click a cell: the node nearest to it along the skeleton is selected, with the nodes it connects to. The tool also runs a shortest-path search from it over the whole graph and reports how long that took.";
  var TYPE_NAME = ["", "dead end", "bend", "T junction", "crossing", "merged group"];
  var TYPE_COLOR = ["", "#1971c2", "#7048e8", "#f08c00", "#d6336c", "#862e9c"];

  function skelNbrs(rm, u, out) {
    var r = (u / cols) | 0, c = u - r * cols, k = 0;
    if (c > 0 && rm[u - 1] > 0) out[k++] = u - 1;
    if (c < cols - 1 && rm[u + 1] > 0) out[k++] = u + 1;
    if (r > 0 && rm[u - cols] > 0) out[k++] = u - cols;
    if (r < rows - 1 && rm[u + cols] > 0) out[k++] = u + cols;
    return k;
  }

  // shortest paths from node src over the graph (edge lengths in steps); returns how many nodes it reached and the farthest distance
  function gDijkstra(src) {
    var INF = 2147483647, M = 4194304, d = new Int32Array(G.count).fill(INF), heap = [];
    function push(key) { heap.push(key); var i = heap.length - 1; while (i > 0) { var p = (i - 1) >> 1; if (heap[p] <= heap[i]) break; var t = heap[p]; heap[p] = heap[i]; heap[i] = t; i = p; } }
    function pop() {
      var top = heap[0], last = heap.pop();
      if (heap.length) {
        heap[0] = last; var i = 0;
        for (;;) { var l = 2 * i + 1, r = l + 1, m = i; if (l < heap.length && heap[l] < heap[m]) m = l; if (r < heap.length && heap[r] < heap[m]) m = r; if (m === i) break; var t = heap[m]; heap[m] = heap[i]; heap[i] = t; i = m; }
      }
      return top;
    }
    d[src] = 0; push(src);
    var reached = 0, maxD = 0;
    while (heap.length) {
      var key = pop(), v = key % M, dv = (key - v) / M;
      if (dv > d[v]) continue;
      reached++; if (dv > maxD) maxD = dv;
      for (var k = G.adjStart[v]; k < G.adjStart[v + 1]; k++) {
        var to = G.adjTo[k], nd = dv + G.adjLen[k];
        if (nd < d[to]) { d[to] = nd; push(nd * M + to); }
      }
    }
    return { reached: reached, maxD: maxD };
  }

  function buildGraph() {
    if (!map) return;
    var step = +$("gstep").value, corners = $("gcorner").checked, mergeK = +$("gmerge").value;
    $("gsv").textContent = step; $("gmv").textContent = mergeK;
    showStats([["Working ...", ""]]);
    setTimeout(function () {
      var t0 = performance.now();
      if (!gskel || gskel.step !== step) {
        if (skel && skel.step === step) gskel = skel; else { gskel = new Skeleton(map, step); gskel.build(); }
        gLayer = null;
      }
      var rm = gskel.rm, tmp = [0, 0, 0, 0], tmp2 = [0, 0, 0, 0], u, i, j;
      // 1. nodes
      var typ = new Uint8Array(n), nodeOrig = new Int32Array(n).fill(-1), nodes = [], cnt = [0, 0, 0, 0, 0], bends = 0;
      for (u = 0; u < n; u++) {
        if (rm[u] <= 0) continue;
        var k = skelNbrs(rm, u, tmp), t = 0;
        if (k <= 1) t = 1;
        else if (k === 2) { if (tmp[0] + tmp[1] !== 2 * u) { bends++; if (corners) t = 2; } }
        else t = k;
        if (t > 0) { typ[u] = t; nodeOrig[u] = nodes.length; nodes.push(u); cnt[t]++; }
      }
      var m = nodes.length;
      // 2. edges: walk along the chain of plain cells from every node; keep each edge once (from the end with the smaller index)
      var edgeA = [], edgeB = [], edgeL = [], loops = 0;
      for (i = 0; i < m; i++) {
        var s = nodes[i], kk = skelNbrs(rm, s, tmp), nb = [tmp[0], tmp[1], tmp[2], tmp[3]];
        for (j = 0; j < kk; j++) {
          var prev = s, cur = nb[j], len = 1, ok = true;
          while (typ[cur] === 0) {
            var kc = skelNbrs(rm, cur, tmp2), nx = -1;
            for (var q = 0; q < kc; q++) { if (tmp2[q] !== prev) { nx = tmp2[q]; break; } }
            if (nx < 0 || ++len > n) { ok = false; break; }
            prev = cur; cur = nx;
          }
          if (!ok) continue;
          var e = nodeOrig[cur];
          if (e === i) { loops++; continue; }
          if (i < e) { edgeA.push(i); edgeB.push(e); edgeL.push(len); }
        }
      }
      // 3. merge: go through the nodes in order; a node that is not in a group yet becomes the centre of a new one and takes every node
      //    within `mergeK` steps along the edges (so a group never spreads further than that from its centre)
      var INF = 2147483647, oStart = new Int32Array(m + 1);
      for (i = 0; i < edgeA.length; i++) { oStart[edgeA[i] + 1]++; oStart[edgeB[i] + 1]++; }
      for (i = 0; i < m; i++) oStart[i + 1] += oStart[i];
      var oTo = new Int32Array(oStart[m]), oLen = new Int32Array(oStart[m]), oFill = oStart.slice(0, m);
      for (i = 0; i < edgeA.length; i++) {
        oTo[oFill[edgeA[i]]] = edgeB[i]; oLen[oFill[edgeA[i]]++] = edgeL[i];
        oTo[oFill[edgeB[i]]] = edgeA[i]; oLen[oFill[edgeB[i]]++] = edgeL[i];
      }
      var gid = new Int32Array(m).fill(-1), gc = 0, repList = [], memList = [], dd = new Int32Array(m).fill(INF), toC = new Int32Array(m);   // toC: steps from a node to the centre of its group
      for (i = 0; i < m; i++) {
        if (gid[i] >= 0) continue;
        var g0 = gc++, taken = 1;
        gid[i] = g0; repList.push(nodes[i]);
        if (mergeK > 0) {
          var touched = [i], qq = [i];
          dd[i] = 0;
          for (var qi = 0; qi < qq.length; qi++) {
            var a0 = qq[qi];
            for (var ke = oStart[a0]; ke < oStart[a0 + 1]; ke++) {
              var b0 = oTo[ke], nd0 = dd[a0] + oLen[ke];
              if (nd0 <= mergeK && nd0 < dd[b0]) {
                if (dd[b0] === INF) touched.push(b0);
                dd[b0] = nd0; qq.push(b0);
                if (gid[b0] < 0) { gid[b0] = g0; taken++; }
              }
            }
          }
          for (var ti = 0; ti < touched.length; ti++) { if (gid[touched[ti]] === g0) toC[touched[ti]] = dd[touched[ti]]; dd[touched[ti]] = INF; }
        }
        memList.push(taken);
      }
      var rep = Int32Array.from(repList), members = Int32Array.from(memList), repType = new Uint8Array(gc);
      for (i = 0; i < gc; i++) repType[i] = members[i] > 1 ? 5 : typ[rep[i]];
      // distinct edges between groups (shortest wins)
      var emap = new Map(), ea = [], eb = [], el = [];
      for (i = 0; i < edgeA.length; i++) {
        var a = gid[edgeA[i]], b = gid[edgeB[i]];
        if (a === b) continue;
        if (a > b) { var sw = a; a = b; b = sw; }
        var key = a * gc + b, idx = emap.get(key), cost = edgeL[i] + toC[edgeA[i]] + toC[edgeB[i]];   // centre to centre, through this edge
        if (idx === undefined) { emap.set(key, ea.length); ea.push(a); eb.push(b); el.push(cost); }
        else if (cost < el[idx]) el[idx] = cost;
      }
      var adjStart = new Int32Array(gc + 1);
      for (i = 0; i < ea.length; i++) { adjStart[ea[i] + 1]++; adjStart[eb[i] + 1]++; }
      for (i = 0; i < gc; i++) adjStart[i + 1] += adjStart[i];
      var adjTo = new Int32Array(adjStart[gc]), adjLen = new Int32Array(adjStart[gc]), fill = adjStart.slice(0, gc);
      for (i = 0; i < ea.length; i++) {
        adjTo[fill[ea[i]]] = eb[i]; adjLen[fill[ea[i]]++] = el[i];
        adjTo[fill[eb[i]]] = ea[i]; adjLen[fill[eb[i]]++] = el[i];
      }
      // 4. the node nearest to every skeleton cell, along the skeleton (multi-source search from all nodes)
      var ownO = new Int32Array(n).fill(-1), qu = new Int32Array(n), qh = 0, qt = 0;
      for (i = 0; i < m; i++) { ownO[nodes[i]] = i; qu[qt++] = nodes[i]; }
      while (qh < qt) {
        var cu = qu[qh++], kn = skelNbrs(rm, cu, tmp);
        for (j = 0; j < kn; j++) { if (ownO[tmp[j]] < 0) { ownO[tmp[j]] = ownO[cu]; qu[qt++] = tmp[j]; } }
      }
      var ownerGroup = new Int32Array(n).fill(-1);
      for (u = 0; u < n; u++) { if (ownO[u] >= 0) ownerGroup[u] = gid[ownO[u]]; }
      G = { count: gc, rep: rep, col: repType, members: members, ea: ea, eb: eb, el: el, adjStart: adjStart, adjTo: adjTo, adjLen: adjLen, ownerGroup: ownerGroup, stats: null };
      var buildMs = performance.now() - t0;
      // search time on the graph: three runs from different nodes
      var ts = performance.now(), reached = 0;
      for (i = 1; i <= 3; i++) { reached = gDijkstra(Math.min(gc - 1, Math.floor(gc * i / 4))).reached; }
      var searchMs = (performance.now() - ts) / 3;
      var edgeSum = 0, edgeMax = 0;
      for (i = 0; i < el.length; i++) { edgeSum += el[i]; if (el[i] > edgeMax) edgeMax = el[i]; }
      var pairBytes = gc * gc * 2;
      G.stats = [
        ["Map", map.rows + " x " + map.cols],
        ["Free cells", map.free.toLocaleString()],
        ["Skeleton cells", gskel.cells.toLocaleString() + " (" + (100 * gskel.cells / Math.max(1, map.free)).toFixed(1) + "% of the free cells)"],
        ["Dead ends / T junctions / crossings", cnt[1].toLocaleString() + " / " + cnt[3].toLocaleString() + " / " + cnt[4].toLocaleString()],
        ["Bends (2 neighbours, not in a line)", bends.toLocaleString() + (corners ? " (nodes)" : " (not nodes)")],
        ["Graph nodes", gc.toLocaleString() + (mergeK > 0 ? " after merging (from " + m.toLocaleString() + ")" : "")],
        ["Skeleton cells per node", (gskel.cells / Math.max(1, gc)).toFixed(1)],
        ["Edges", ea.length.toLocaleString() + (loops ? " (" + (loops / 2) + " loops skipped)" : "")],
        ["Edge length: mean / longest", (edgeSum / Math.max(1, el.length)).toFixed(1) + " / " + edgeMax + " steps"],
        ["Table per goal on the graph", (gc * 2 / 1000).toFixed(0) + " KB (full table: " + (n * 2 / 1000).toFixed(0) + " KB)"],
        ["All pairs on the graph", pairBytes >= 1e9 ? (pairBytes / 1e9).toFixed(1) + " GB" : (pairBytes / 1e6).toFixed(0) + " MB"],
        ["Search from one node (Dijkstra)", searchMs.toFixed(1) + " ms, reaches " + reached.toLocaleString() + " nodes"],
        ["Build time in the browser", (buildMs / 1000).toFixed(2) + " s"]
      ];
      gSel = null;
      if (mode === "graph") { showStats(G.stats); $("info").textContent = GRAPH_HELP; }
      view.draw();
    }, 20);
  }

  function gSkelLayer() {
    if (!gLayer) gLayer = view.cellLayer(function (u) { return gskel.rm[u] > 0 ? [18, 184, 134, 110] : null; });
    return gLayer;
  }

  function selectNode(g) {
    var t0 = performance.now(), res = gDijkstra(g), ms = performance.now() - t0, nb = [];
    for (var k = G.adjStart[g]; k < G.adjStart[g + 1]; k++) nb.push(G.adjTo[k]);
    gSel = { g: g, nb: nb };
    var r = (G.rep[g] / cols) | 0, c = G.rep[g] % cols;
    $("info").textContent = "Node " + g + " at row " + r + ", column " + c + ": " + (G.members[g] > 1 ? "a merged group of " + G.members[g] + " nodes" : TYPE_NAME[G.col[g]]) +
      " with " + nb.length + " edges. A shortest-path search from it over the whole graph reaches " + res.reached.toLocaleString() + " of " + G.count.toLocaleString() +
      " nodes (farthest " + res.maxD + " steps) in " + ms.toFixed(1) + " ms in this browser.";
    view.draw();
  }

  // ---------- drawing ----------
  view.layers.push(function (ctx) {
    if (!map) return;
    if (isCover()) {
      if ($("show-cov").checked && owner) ctx.drawImage(coverageLayer(), 0, 0);
      if (selLayer) ctx.drawImage(selLayer, 0, 0);
    } else if (mode === "graph") {
      if (G && $("g-skel").checked) ctx.drawImage(gSkelLayer(), 0, 0);
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
      if ($("g-edges").checked) {
        ctx.strokeStyle = "rgba(50,50,50,0.75)"; ctx.lineWidth = Math.max(0.1, 0.9 / v.scale);
        ctx.beginPath();
        for (var e = 0; e < G.ea.length; e++) { ctx.moveTo(px(G.rep[G.ea[e]]), py(G.rep[G.ea[e]])); ctx.lineTo(px(G.rep[G.eb[e]]), py(G.rep[G.eb[e]])); }
        ctx.stroke();
      }
      if ($("g-nodes").checked) {
        for (var t = 1; t <= 5; t++) {
          ctx.fillStyle = TYPE_COLOR[t]; ctx.beginPath();
          for (var g = 0; g < G.count; g++) { if (G.col[g] === t) { var x = px(G.rep[g]), y = py(G.rep[g]); ctx.moveTo(x + rad, y); ctx.arc(x, y, rad, 0, 6.2832); } }
          ctx.fill();
        }
      }
      if (gSel) {
        var sx = px(G.rep[gSel.g]), sy = py(G.rep[gSel.g]);
        ctx.strokeStyle = "#f08c00"; ctx.lineWidth = Math.max(0.2, 2 / v.scale);
        ctx.beginPath();
        gSel.nb.forEach(function (w) { ctx.moveTo(sx, sy); ctx.lineTo(px(G.rep[w]), py(G.rep[w])); });
        ctx.stroke();
        gSel.nb.forEach(function (w) { dot(G.rep[w], rad * 1.8, "#f08c00"); });
        dot(G.rep[gSel.g], rad * 2.4, "#ffffff"); dot(G.rep[gSel.g], rad * 1.8, "#1971c2");
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
  ["show-cov", "show-wp", "show-skel", "show-route", "g-skel", "g-edges", "g-nodes"].forEach(function (id) { $(id).addEventListener("change", function () { view.draw(); }); });
  $("gstep").addEventListener("input", function () { $("gsv").textContent = $("gstep").value; clearTimeout(timer3); timer3 = setTimeout(buildGraph, 200); });
  $("gmerge").addEventListener("input", function () { $("gmv").textContent = $("gmerge").value; clearTimeout(timer3); timer3 = setTimeout(buildGraph, 200); });
  $("gcorner").addEventListener("change", buildGraph);
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
      $("csv").textContent = isCover() ? "Download waypoints (.csv)" : mode === "graph" ? "Download nodes (.csv)" : "Download skeleton cells (.csv)";
      $("info").textContent = isCover() ? COVER_HELP : mode === "graph" ? GRAPH_HELP : SKEL_HELP;
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
      out = ["id,cell,row,col,type,members"];
      for (var g = 0; g < G.count; g++) out.push(g + "," + G.rep[g] + "," + ((G.rep[g] / cols) | 0) + "," + (G.rep[g] % cols) + "," + TYPE_NAME[G.col[g]] + "," + G.members[g]);
    }
    else if (skel) { for (var u = 0; u < n; u++) if (skel.rm[u] > 0) list.push(u); }
    for (var k = 0; k < list.length; k++) out.push(k + "," + list[k] + "," + ((list[k] / cols) | 0) + "," + (list[k] % cols) + (mode === "dyn" && wpTag ? "," + DYN_NAME[wpTag[k]] : ""));
    var a = document.createElement("a");
    a.href = URL.createObjectURL(new Blob([out.join("\n") + "\n"], { type: "text/csv" }));
    a.download = mode === "dyn" ? "dynamic_waypoints.csv" : mode === "cover" ? "waypoints.csv" : mode === "graph" ? "graph_nodes.csv" : "skeleton.csv";
    a.click();
  });

  view.onClick = function (cell) {
    if (!cell || !map || grid[cell.u]) return;
    if (isCover()) { selectCover(wpOf[cell.u] >= 0 ? wpOf[cell.u] : owner[cell.u]); return; }
    if (mode === "graph") {
      if (!G) return;
      var near = gskel.nearest(cell.u);
      if (near && G.ownerGroup[near.cell] >= 0) selectNode(G.ownerGroup[near.cell]);
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
    if (!extra && mode === "graph" && G && gskel.rm[cell.u] > 0) {
      var og = G.ownerGroup[cell.u];
      extra = (og >= 0 && G.rep[og] === cell.u) ? " (node " + og + ", " + (G.members[og] > 1 ? "merged group of " + G.members[og] : TYPE_NAME[G.col[og]]) + ")" : " (skeleton)";
    }
    h.textContent = "row " + cell.r + ", col " + cell.c + extra;
  };

  var loadSample = MapTools.setupLoader({
    file: $("file"), drop: $("drop"), sample: $("sample"), status: $("status"),
    onMap: function (m) {
      map = m; rows = m.rows; cols = m.cols; n = rows * cols; grid = m.grid;
      seen = new Int32Array(n); dist = new Int16Array(n); queue = new Int32Array(n); stamp = 0;
      skel = null; skelLayer = null; route = null; sel = null; selLayer = null;
      gskel = null; gLayer = null; G = null; gSel = null;
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
  $("gsv").textContent = $("gstep").value; $("gmv").textContent = $("gmerge").value;
  loadSample();
})();

/* Cover point (CP) demo: the distance method of the PIBT lab planner (src/PIBT/Waypoints.cpp) on a tiny grid.
   1. CPs: every free cell is within C steps of one (the same greedy pass as the planner)
   2. static lists: for every cell, the exact walking steps to each CP within R steps (walls count)
   3. per goal: the exact window of R steps around the goal, then one Dijkstra over the CPs gives D (CP -> goal)
   4. the score of a cell = its exact distance inside the window, else the smallest (steps to a CP + D of it)
   5. the robot steps to the neighbour with the lowest score (ties: east, south, west, north, like the planner). */
(function () {
  "use strict";
  var $ = function (id) { return document.getElementById(id); };
  var DIRS = [[0, 1, "E"], [1, 0, "S"], [0, -1, "W"], [-1, 0, "N"]];
  var S = { n: 7, R: 4, C: 2, mode: "select", wall: [], cp: [], goal: 6, robot: 0, start: 0, sel: -1,
            showScores: true, showVia: true, showWin: true, D: null, order: [], k: 0, path: [], note: "" };
  var M = {};            // derived data: cpOf, ent (static lists), edges, win, tdist
  var timer = null, drag = null, justDragged = false;

  // ---------- grid helpers ----------
  function nbrs(u) {
    var n = S.n, r = (u / n) | 0, c = u % n, out = [];
    for (var d = 0; d < 4; d++) {
      var rr = r + DIRS[d][0], cc = c + DIRS[d][1];
      if (rr >= 0 && rr < n && cc >= 0 && cc < n && !S.wall[rr * n + cc]) { out.push([rr * n + cc, DIRS[d][2]]); }
    }
    return out;
  }
  function bfs(src, lim) {                      // walking steps from src, -1 = not reached (within lim steps)
    var dist = new Array(S.n * S.n).fill(-1), q = [src];
    dist[src] = 0;
    for (var h = 0; h < q.length; h++) {
      var u = q[h];
      if (dist[u] >= lim) { continue; }
      var nb = nbrs(u);
      for (var i = 0; i < nb.length; i++) { if (dist[nb[i][0]] < 0) { dist[nb[i][0]] = dist[u] + 1; q.push(nb[i][0]); } }
    }
    return dist;
  }
  function name(u) { return "(" + ((u / S.n) | 0) + "," + (u % S.n) + ")"; }
  function effC() { return Math.min(S.C, S.R); }

  // ---------- the method ----------
  function placeCps() {                         // one pass over the cells: an uncovered free cell becomes a CP and covers its C-step neighbourhood
    var N = S.n * S.n, covered = new Array(N).fill(false);
    S.cp = [];
    for (var u = 0; u < N; u++) {
      if (S.wall[u] || covered[u]) { continue; }
      S.cp.push(u);
      var d = bfs(u, effC());
      for (var t = 0; t < N; t++) { if (d[t] >= 0) { covered[t] = true; } }
    }
  }
  function build() {
    var N = S.n * S.n;
    M.cpOf = new Array(N).fill(-1);
    S.cp.forEach(function (u, i) { M.cpOf[u] = i; });
    M.ent = [];
    for (var u = 0; u < N; u++) { M.ent.push([]); }
    M.edges = S.cp.map(function () { return []; });
    S.cp.forEach(function (cu, i) {
      var d = bfs(cu, S.R);
      for (var t = 0; t < N; t++) {
        if (d[t] < 0) { continue; }
        M.ent[t].push([i, d[t]]);
        var j = M.cpOf[t];
        if (j >= 0 && j !== i) { M.edges[i].push([j, d[t]]); }
      }
    });
    M.ent.forEach(function (l) { l.sort(function (a, b) { return a[1] - b[1] || a[0] - b[0]; }); });
    M.win = bfs(S.goal, S.R);
    M.tdist = bfs(S.goal, Infinity);
  }
  function runDijkstra() {
    var m = S.cp.length, D = new Array(m).fill(Infinity), done = new Array(m).fill(false), order = [];
    S.cp.forEach(function (u, i) { if (M.win[u] >= 0) { D[i] = M.win[u]; } });      // seeds: CPs inside the goal window
    for (;;) {
      var b = -1;
      for (var i = 0; i < m; i++) { if (!done[i] && D[i] < Infinity && (b < 0 || D[i] < D[b])) { b = i; } }
      if (b < 0) { break; }
      done[b] = true;
      order.push(b);
      M.edges[b].forEach(function (e) { if (D[b] + e[1] < D[e[0]]) { D[e[0]] = D[b] + e[1]; } });
    }
    S.D = D; S.order = order; S.k = order.length;
  }
  function dview() {                            // D as far as the Dijkstra slider has settled
    if (!S.D) { return null; }
    var v = new Array(S.cp.length).fill(Infinity);
    for (var i = 0; i < S.k; i++) { v[S.order[i]] = S.D[S.order[i]]; }
    return v;
  }
  function evalCell(u, Dv) {                    // {score, exact} or {score, id, steps, dv} or null
    if (M.win[u] >= 0) { return { score: M.win[u], exact: true }; }
    var best = null;
    M.ent[u].forEach(function (e) {
      var dv = Dv[e[0]];
      if (dv === Infinity) { return; }
      var s = e[1] + dv;
      if (!best || s < best.score) { best = { score: s, id: e[0], steps: e[1], dv: dv }; }
    });
    return best;
  }

  // ---------- state changes ----------
  function stopPlay() { if (timer) { clearInterval(timer); timer = null; $("play").textContent = "Play"; } }
  function invalidate() {                       // anything but the robot changed: the Dijkstra has to be run again
    stopPlay();
    S.D = null; S.order = []; S.k = 0; S.note = "";
    S.robot = S.start; S.path = [S.robot];
    placeCps();
    build();
  }
  function defaultMap(n) {
    S.n = n;
    S.wall = new Array(n * n).fill(0);
    var col = (n / 2) | 0;
    for (var r = 0; r < n - 2; r++) { S.wall[r * n + col] = 1; }
    S.start = S.robot = 0;
    S.goal = n - 1;
    S.sel = -1;
    invalidate();
  }
  function stepRobot() {
    if (!S.D) { runDijkstra(); }
    S.k = S.order.length;
    var Dv = dview();
    if (S.robot === S.goal) { S.note = "At the goal."; stopPlay(); render(); return; }
    var cur = evalCell(S.robot, Dv), best = null;
    nbrs(S.robot).forEach(function (x) {
      var e = evalCell(x[0], Dv);
      if (e && (!best || e.score < best.score)) { best = { u: x[0], dir: x[1], score: e.score }; }
    });
    if (!best) { S.note = "Goal unreachable from here."; stopPlay(); render(); return; }
    var visits = S.path.filter(function (u) { return u === best.u; }).length;
    if (visits >= 3) { S.note = "Stuck in a loop at " + name(best.u) + "."; stopPlay(); render(); return; }
    var bad = cur && best.score >= cur.score;
    S.robot = best.u;
    S.path.push(best.u);
    S.note = "Moved " + best.dir + " to " + name(best.u) + ", score " + best.score + "." + (bad ? " Not lower than before: scores are approximate here." : "");
    if (S.robot === S.goal) { S.note += " Goal reached in " + (S.path.length - 1) + " steps (shortest " + M.tdist[S.start] + ")."; stopPlay(); }
    render();
  }

  // ---------- drawing ----------
  function fmt(x) { return x === Infinity || x == null ? "&ndash;" : String(x); }
  function render() {
    var n = S.n, N = n * n, Dv = dview(), nbSet = {}, best = -1, bestScore = Infinity;
    if (Dv && S.robot !== S.goal) {
      nbrs(S.robot).forEach(function (x) {
        var e = evalCell(x[0], Dv);
        nbSet[x[0]] = e;
        if (e && e.score < bestScore) { bestScore = e.score; best = x[0]; }
      });
    }
    var inList = {};
    if (S.sel >= 0 && !S.wall[S.sel]) { M.ent[S.sel].forEach(function (e) { inList[S.cp[e[0]]] = true; }); }
    var trail = {};
    S.path.forEach(function (u) { trail[u] = true; });
    // two views of the same state: the simple grid (the score of every cell) and the detailed grid (every cell as a grid of its CPs)
    var grid = $("grid"), grid2 = $("grid2"), html = "", html2 = "";
    grid.style.gridTemplateColumns = "repeat(" + n + ", 1fr)";
    grid2.style.gridTemplateColumns = "repeat(" + n + ", 1fr)";
    var cs = (grid2.getBoundingClientRect().width || 640) / n - 2;               // size of a cell of the detailed grid in px
    for (var u = 0; u < N; u++) {
      var cls = "dc", inner = "", inner2 = "";
      if (S.wall[u]) { cls += " wall"; }
      else {
        var ev = Dv ? evalCell(u, Dv) : null, isNb = nbSet.hasOwnProperty(u), inWin = M.win[u] >= 0;
        if (S.showWin && inWin) { cls += " win"; }
        if (trail[u]) { cls += " trail"; }
        if (u === S.sel) { cls += " sel"; }
        if (inList[u] && M.cpOf[u] >= 0) { cls += " inlist"; }
        if (isNb) { cls += " nb"; if (u === best) { cls += " best"; } }
        var tag = M.cpOf[u] >= 0 ? '<span class="cptag' + (inList[u] ? " inlist" : "") + '">CP' + M.cpOf[u] + (Dv ? " &middot; " + fmt(Dv[M.cpOf[u]]) : "") + "</span>" : "";
        var tokG = u === S.goal ? '<span class="tok goal" data-tok="goal" title="drag to move the goal">G</span>' : "";
        var tokR = u === S.robot ? '<span class="tok robot" data-tok="robot" title="drag to move the robot">R</span>' : "";
        // simple grid: the score of the cell, and which CP gave it
        inner = tag;
        if (Dv && (S.showScores || isNb)) {
          inner += '<span class="sc">' + (ev ? ev.score : "&ndash;") + "</span>";
          if (S.showVia && ev && !ev.exact) { inner += '<span class="via">&larr; CP' + ev.id + "</span>"; }
        }
        inner += tokG + tokR;
        // detailed grid: a slot per CP of the cell's list, "steps + D" (D is not added up on purpose)
        var hd = tag;
        if (Dv && inWin) { hd += '<span class="tot" title="inside the goal window: the exact distance is used, the slots are ignored">exact ' + M.win[u] + "</span>"; }
        else if (Dv && (isNb || u === S.robot)) { hd += '<span class="tot" title="score of the cell">= ' + (ev ? ev.score : "&ndash;") + "</span>"; }
        hd += tokG.replace(' data-tok="goal"', "") + tokR.replace(' data-tok="robot"', "");
        var l = M.ent[u], k = l.length, cols = k <= 2 ? Math.max(1, k) : Math.ceil(Math.sqrt(k)), rows = Math.max(1, Math.ceil(k / cols));
        var fs = Math.max(5, Math.min(12, (cs - 6) / cols / 3.6, (cs - 20) / (rows * 2.5))), body = "";
        l.forEach(function (e) {
          var dv = Dv ? Dv[e[0]] : null, win = ev && !ev.exact && ev.id === e[0];
          var dtxt = !Dv ? "D" + e[0] : dv !== Infinity ? String(dv) : (S.D[e[0]] !== Infinity ? "&hellip;" : "&infin;");
          body += '<span class="sl' + (win ? " wn" : "") + '" title="CP' + e[0] + ": " + e[1] + " steps" + (Dv && dv !== Infinity ? " + D " + dv + " = " + (e[1] + dv) : "") + '"><i>CP' + e[0] + "</i><b>" + e[1] + "+" + dtxt + "</b></span>";
        });
        inner2 = '<div class="hd">' + hd + '</div><div class="mg' + (Dv && inWin ? " ex" : "") + '" style="grid-template-columns:repeat(' + cols + ',1fr);font-size:' + fs.toFixed(1) + 'px">' + body + "</div>";
      }
      html += '<div class="' + cls + '" data-i="' + u + '">' + inner + "</div>";
      html2 += '<div class="' + cls + '" data-i="' + u + '">' + inner2 + "</div>";
    }
    grid.innerHTML = html;
    grid2.innerHTML = html2;
    $("rv").textContent = S.R;
    $("cv").textContent = S.C > S.R ? S.C + " (uses " + S.R + ")" : S.C;
    $("nv").textContent = n;

    // CP scores for this goal
    var free = 0, winCells = 0;
    for (var i = 0; i < N; i++) { if (!S.wall[i]) { free++; if (M.win[i] >= 0) { winCells++; } } }
    var m = S.cp.length, chips = "";
    for (var c = 0; c < m; c++) {
      var settled = Dv && Dv[c] !== Infinity;
      chips += '<span class="chip' + (settled ? " on" : "") + '"><b>CP' + c + "</b> " + name(S.cp[c]) + " = " + (Dv ? (settled ? Dv[c] : (S.D[c] === Infinity ? "unreachable" : "&hellip;")) : "?") + "</span>";
    }
    $("darray").innerHTML = chips;
    $("dk").max = S.order.length; $("dk").value = S.k; $("dk").disabled = !S.D;
    $("dkv").textContent = S.D ? S.k + " of " + S.order.length : "not run";
    $("dnote").innerHTML = S.D ? "D = each CP's steps to the goal. CPs inside the exact window start with their exact value; the Dijkstra spreads the rest." : "Press <em>Run Dijkstra</em> to get D.";
    $("nums").innerHTML = "Stored per goal: <strong>" + (S.cp.length + winCells) + "</strong> numbers (" + S.cp.length + " D + " + winCells + " window cells). A full table: <strong>" + free + "</strong>.";
    $("rnote").textContent = S.note;
    $("steps").textContent = "steps: " + (S.path.length - 1) + (M.tdist[S.start] >= 0 ? " (shortest " + M.tdist[S.start] + ")" : "");
  }

  // ---------- events ----------
  function cellAt(e) {
    var el = document.elementFromPoint(e.clientX, e.clientY), c = el && el.closest ? el.closest(".dc") : null;
    return c ? +c.getAttribute("data-i") : -1;
  }
  var ghost = $("ghost");
  function moveGhost(e) { ghost.style.left = e.clientX + "px"; ghost.style.top = e.clientY + "px"; }
  function mark(sel, cls, on) { Array.prototype.forEach.call(document.querySelectorAll(sel), function (el) { el.classList[on ? "add" : "remove"](cls); }); }
  function clearDrop() { mark(".dc.drop", "drop", false); }
  $("grid").addEventListener("pointerdown", function (e) {
    var t = e.target.closest ? e.target.closest(".tok") : null;
    if (!t) { return; }
    e.preventDefault();
    drag = { tok: t.getAttribute("data-tok"), moved: false };
    ghost.textContent = t.textContent;
    ghost.className = "ghost " + drag.tok;
    ghost.hidden = false;
    moveGhost(e);
  });
  window.addEventListener("pointermove", function (e) {
    if (!drag) { return; }
    drag.moved = true;
    moveGhost(e);
    clearDrop();
    var c = cellAt(e);
    if (c >= 0) { mark('.dc[data-i="' + c + '"]', "drop", true); }
  });
  window.addEventListener("pointerup", function (e) {
    if (!drag) { return; }
    var c = cellAt(e), d = drag;
    ghost.hidden = true; clearDrop(); drag = null;
    justDragged = d.moved;
    setTimeout(function () { justDragged = false; }, 0);
    if (c >= 0 && d.moved) {
      S.wall[c] = 0;
      if (d.tok === "goal") { S.goal = c; } else { S.start = c; }
      invalidate();
    }
    render();
  });
  function onGridClick(e) {
    if (justDragged) { return; }
    var el = e.target.closest ? e.target.closest(".dc") : null;
    if (!el) { return; }
    var u = +el.getAttribute("data-i");
    if (S.mode === "select") { S.sel = (S.sel === u || S.wall[u]) ? -1 : u; }
    else if (S.mode === "wall") {
      if (u === S.goal || u === S.start) { return; }
      S.wall[u] = S.wall[u] ? 0 : 1;
      invalidate();
    }
    render();
  }
  $("grid").addEventListener("click", onGridClick);
  $("grid2").addEventListener("click", onGridClick);
  function bind(id, ev, fn) { $(id).addEventListener(ev, fn); }
  bind("size", "change", function () { defaultMap(+$("size").value); render(); });
  bind("rr", "input", function () { S.R = +$("rr").value; invalidate(); render(); });
  bind("cc", "input", function () { S.C = +$("cc").value; invalidate(); render(); });
  Array.prototype.forEach.call(document.querySelectorAll("input[name=mode]"), function (r) {
    r.addEventListener("change", function () { S.mode = r.value; });
  });
  bind("show-scores", "change", function () { S.showScores = $("show-scores").checked; render(); });
  bind("show-via", "change", function () { S.showVia = $("show-via").checked; render(); });
  bind("show-win", "change", function () { S.showWin = $("show-win").checked; render(); });
  var resizeTimer = null;
  window.addEventListener("resize", function () { clearTimeout(resizeTimer); resizeTimer = setTimeout(render, 120); });
  bind("clear", "click", function () { S.wall.fill(0); invalidate(); render(); });
  bind("reset", "click", function () { defaultMap(S.n); render(); });
  bind("run", "click", function () { runDijkstra(); render(); });
  bind("dk", "input", function () { S.k = +$("dk").value; render(); });
  bind("step", "click", function () { stepRobot(); });
  bind("home", "click", function () { stopPlay(); S.robot = S.start; S.path = [S.robot]; S.note = ""; render(); });
  bind("play", "click", function () {
    if (timer) { stopPlay(); return; }
    $("play").textContent = "Pause";
    timer = setInterval(stepRobot, 450);
  });

  defaultMap(7);
  render();
  window.coverDemo = { S: S, M: M, evalCell: evalCell, dview: dview };      // for checking in the console
})();

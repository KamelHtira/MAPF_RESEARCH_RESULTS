/* Shared code of the map tools (tools/waypoints.html, tools/lanes.html):
   reading a .map file into a matrix, loading it (file picker, drag and drop, sample maps) and a zoomable canvas. */
(function () {
  "use strict";

  // The planner reads a map like this (src/Grid.cpp): header "type octile / height H / width W / map" (or "rows,cols" on the
  // first line), then H lines of W characters. '@' and 'T' are obstacles, every other character is free.
  // The matrix is flat and indexed row * cols + col, like the planner indexes cells.
  function parseMap(text) {
    var lines = text.replace(/\r/g, "").split("\n");
    var rows, cols, start;
    if (/^\s*type/i.test(lines[0])) {
      rows = parseInt(lines[1].split(/\s+/)[1], 10);
      cols = parseInt(lines[2].split(/\s+/)[1], 10);
      start = 4;
    } else {
      var p = lines[0].split(",");
      rows = parseInt(p[0], 10);
      cols = parseInt(p[1], 10);
      start = 1;
    }
    if (!(rows > 0 && cols > 0) || rows * cols > 4e6) throw new Error("This does not look like a .map file (could not read the height and width).");
    var grid = new Uint8Array(rows * cols);
    var free = 0;
    for (var r = 0; r < rows; r++) {
      var line = lines[start + r] || "";
      for (var c = 0; c < cols; c++) {
        var ch = line.charAt(c);
        var obstacle = ch === "@" || ch === "T";
        grid[r * cols + c] = obstacle ? 1 : 0;
        if (!obstacle) free++;
      }
    }
    return { rows: rows, cols: cols, grid: grid, free: free };
  }

  var SAMPLES = [
    { label: "warehouse_small (33 x 57)", file: "../maps/warehouse_small.map" },
    { label: "warehouse_large (140 x 500)", file: "../maps/warehouse_large.map" },
    { label: "sortation_large (140 x 500)", file: "../maps/sortation_large.map" },
    { label: "Paris city (256 x 256)", file: "../maps/Paris_1_256.map" },
    { label: "brc202d (481 x 530)", file: "../maps/brc202d.map" }
  ];

  // file picker, drag and drop and the sample list; calls onMap(map, name)
  function setupLoader(o) {
    var status = o.status;
    function say(t, bad) { status.textContent = t; status.className = "small" + (bad ? " bad" : ""); }
    function use(text, name) {
      try { o.onMap(parseMap(text), name); say(name); } catch (e) { say(e.message, true); }
    }
    o.file.addEventListener("change", function () {
      var f = o.file.files[0];
      if (!f) return;
      var rd = new FileReader();
      rd.onload = function () { use(String(rd.result), f.name); };
      rd.readAsText(f);
    });
    ["dragenter", "dragover"].forEach(function (ev) {
      o.drop.addEventListener(ev, function (e) { e.preventDefault(); o.drop.classList.add("over"); });
    });
    ["dragleave", "drop"].forEach(function (ev) {
      o.drop.addEventListener(ev, function (e) { e.preventDefault(); o.drop.classList.remove("over"); });
    });
    o.drop.addEventListener("drop", function (e) {
      var f = e.dataTransfer.files[0];
      if (!f) return;
      var rd = new FileReader();
      rd.onload = function () { use(String(rd.result), f.name); };
      rd.readAsText(f);
    });
    SAMPLES.forEach(function (s, i) {
      var op = document.createElement("option");
      op.value = String(i); op.textContent = s.label;
      o.sample.appendChild(op);
    });
    function loadSample() {
      var s = SAMPLES[parseInt(o.sample.value, 10)];
      say("Loading " + s.label + " ...");
      fetch(s.file).then(function (r) {
        if (!r.ok) throw new Error("not found");
        return r.text();
      }).then(function (t) { use(t, s.label); }).catch(function () {
        say("Could not load the sample map (a page opened straight from disk cannot fetch files). Use the file picker, or open the page over http.", true);
      });
    }
    o.sample.addEventListener("change", loadSample);
    return loadSample;
  }

  // canvas with zoom (wheel, buttons) and pan (drag). The map is drawn as one pixel per cell on a hidden canvas and scaled up.
  function MapView(canvas) {
    var self = this;
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.map = null;
    this.base = null;
    this.layers = [];          // functions (ctx, view) drawn on top, in cell coordinates already transformed: 1 unit = 1 cell
    this.scale = 1; this.ox = 0; this.oy = 0;
    this.onClick = null;
    this.onMove = null;
    this.autoFit = true;       // keep the whole map in view until the user zooms or moves it
    var drag = null, moved = false;

    function resize() {
      var rect = canvas.getBoundingClientRect();
      var dpr = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.round(rect.width * dpr));
      canvas.height = Math.max(1, Math.round(rect.height * dpr));
      self.dpr = dpr;
      if (self.autoFit) self.fit(); else self.draw();
    }
    this.resize = resize;
    if (window.ResizeObserver) new ResizeObserver(resize).observe(canvas); else window.addEventListener("resize", resize);

    canvas.addEventListener("wheel", function (e) {
      e.preventDefault();
      var p = self.local(e);
      self.autoFit = false;
      self.zoomAt(p.x, p.y, e.deltaY < 0 ? 1.2 : 1 / 1.2);
    }, { passive: false });
    canvas.addEventListener("pointerdown", function (e) {
      canvas.setPointerCapture(e.pointerId);
      drag = { x: e.clientX, y: e.clientY, ox: self.ox, oy: self.oy };
      moved = false;
    });
    canvas.addEventListener("pointermove", function (e) {
      if (drag) {
        var dx = e.clientX - drag.x, dy = e.clientY - drag.y;
        if (Math.abs(dx) + Math.abs(dy) > 3) moved = true;
        if (moved) { self.autoFit = false; self.ox = drag.ox + dx; self.oy = drag.oy + dy; self.draw(); }
      }
      if (self.onMove) { var c = self.cellAt(e); self.onMove(c, e); }
    });
    canvas.addEventListener("pointerup", function (e) {
      var wasClick = drag && !moved;
      drag = null;
      if (wasClick && self.onClick) self.onClick(self.cellAt(e), e);
    });
    canvas.addEventListener("pointerleave", function () { if (self.onMove) self.onMove(null); });
  }

  MapView.prototype.local = function (e) {
    var rect = this.canvas.getBoundingClientRect();
    return { x: e.clientX - rect.left, y: e.clientY - rect.top };
  };
  // the cell under a mouse event, or null outside the map
  MapView.prototype.cellAt = function (e) {
    if (!this.map) return null;
    var p = this.local(e);
    var c = Math.floor((p.x - this.ox) / this.scale), r = Math.floor((p.y - this.oy) / this.scale);
    if (r < 0 || c < 0 || r >= this.map.rows || c >= this.map.cols) return null;
    return { r: r, c: c, u: r * this.map.cols + c };
  };
  MapView.prototype.zoomAt = function (x, y, f) {
    var s = Math.min(60, Math.max(0.2, this.scale * f));
    f = s / this.scale;
    this.ox = x - (x - this.ox) * f;
    this.oy = y - (y - this.oy) * f;
    this.scale = s;
    this.draw();
  };
  MapView.prototype.zoomCenter = function (f) {
    this.autoFit = false;
    var rect = this.canvas.getBoundingClientRect();
    this.zoomAt(rect.width / 2, rect.height / 2, f);
  };
  MapView.prototype.fit = function () {
    if (!this.map) return;
    this.autoFit = true;
    var rect = this.canvas.getBoundingClientRect();
    this.scale = Math.min(rect.width / this.map.cols, rect.height / this.map.rows);
    this.ox = (rect.width - this.map.cols * this.scale) / 2;
    this.oy = (rect.height - this.map.rows * this.scale) / 2;
    this.draw();
  };
  MapView.prototype.setMap = function (map) {
    this.map = map;
    var oc = document.createElement("canvas");
    oc.width = map.cols; oc.height = map.rows;
    var octx = oc.getContext("2d");
    var img = octx.createImageData(map.cols, map.rows);
    for (var i = 0; i < map.grid.length; i++) {
      var v = map.grid[i] ? 0x39 : 0xf4;
      img.data[i * 4] = v; img.data[i * 4 + 1] = map.grid[i] ? 0x42 : 0xf6; img.data[i * 4 + 2] = map.grid[i] ? 0x4b : 0xf8; img.data[i * 4 + 3] = 255;
    }
    octx.putImageData(img, 0, 0);
    this.base = oc;
    this.fit();
  };
  // an offscreen image with one pixel per cell, colours given by f(cell) -> [r, g, b, a] or null
  MapView.prototype.cellLayer = function (f) {
    var m = this.map, oc = document.createElement("canvas");
    oc.width = m.cols; oc.height = m.rows;
    var octx = oc.getContext("2d"), img = octx.createImageData(m.cols, m.rows);
    for (var u = 0; u < m.grid.length; u++) {
      var col = f(u);
      if (!col) continue;
      img.data[u * 4] = col[0]; img.data[u * 4 + 1] = col[1]; img.data[u * 4 + 2] = col[2]; img.data[u * 4 + 3] = col[3];
    }
    octx.putImageData(img, 0, 0);
    return oc;
  };
  MapView.prototype.draw = function () {
    var ctx = this.ctx, cv = this.canvas;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, cv.width, cv.height);
    if (!this.map) return;
    var d = this.dpr || 1;
    ctx.setTransform(this.scale * d, 0, 0, this.scale * d, this.ox * d, this.oy * d);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(this.base, 0, 0);
    for (var i = 0; i < this.layers.length; i++) { ctx.save(); this.layers[i](ctx, this); ctx.restore(); }
  };
  // cells currently visible: [r0, r1, c0, c1] (inclusive)
  MapView.prototype.visible = function () {
    var rect = this.canvas.getBoundingClientRect(), m = this.map;
    return [Math.max(0, Math.floor(-this.oy / this.scale)), Math.min(m.rows - 1, Math.floor((rect.height - this.oy) / this.scale)),
            Math.max(0, Math.floor(-this.ox / this.scale)), Math.min(m.cols - 1, Math.floor((rect.width - this.ox) / this.scale))];
  };

  window.MapTools = { parseMap: parseMap, setupLoader: setupLoader, MapView: MapView };
})();

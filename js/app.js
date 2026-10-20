(function () {
  "use strict";

  var D = window.RESULTS;
  if (!D) {
    document.getElementById("title").textContent = "No data found (data/results.js did not load).";
    return;
  }

  // ---------- helpers ----------
  function el(tag, props, kids) {
    var n = document.createElement(tag);
    Object.keys(props || {}).forEach(function (k) {
      var v = props[k];
      if (v === false || v == null) return;
      if (k === "class") n.className = v;
      else if (k === "text") n.textContent = v;
      else n.setAttribute(k, v === true ? "" : v);
    });
    (kids || []).forEach(function (c) {
      if (c == null) return;
      n.appendChild(typeof c === "string" ? document.createTextNode(c) : c);
    });
    return n;
  }
  function $(id) { return document.getElementById(id); }
  function mean(a) { return a.reduce(function (s, x) { return s + x; }, 0) / a.length; }
  function fmt(x) { return Number.isInteger(x) ? String(x) : x.toFixed(1); }
  function basename(p) { return p.split("/").pop(); }

  // each implementation (Python, C++) is compared with its own default planner: they break ties between equal paths differently
  function implOf(m) { return m.impl || "python"; }
  function baselineFor(m) {
    return D.methods.filter(function (x) { return x.baseline && implOf(x) === implOf(m); })[0] || D.methods[0];
  }

  // the runs of a method on a scenario, each as an object (a bare number means just the task count)
  function runsOf(method, sid) {
    var list = method.results && method.results[sid];
    if (!list || !list.length) return null;
    var timing = (window.TIMING || {})[method.id + "|" + sid];
    if (timing && timing.length !== list.length) timing = null;
    return list.map(function (r, i) {
      var run = typeof r === "number" ? { tasks: r } : r;
      if (timing) { run.plan = timing[i][0]; run.slow = timing[i][1]; run.over = timing[i][2]; }
      return run;
    });
  }
  // planning time of a set of runs: mean total over the whole run, slowest single step seen in any run
  function timingOf(runs) {
    if (!runs || runs[0].plan == null) return null;
    return { total: mean(runs.map(function (r) { return r.plan; })), slow: Math.max.apply(null, runs.map(function (r) { return r.slow; })) };
  }
  function fmtTime(sec) {
    if (sec < 1) return sec.toFixed(1) + " s";
    if (sec < 10) return sec.toFixed(1) + " s";
    if (sec < 60) return Math.round(sec) + " s";
    if (sec < 3600) return (sec < 600 ? (sec / 60).toFixed(1) : Math.round(sec / 60)) + " min";
    return (sec / 3600).toFixed(1) + " h";
  }
  function fmtTimes(x) { return x >= 10 ? String(Math.round(x)) : x.toFixed(1); }
  function tasksOf(runs) { return runs.map(function (r) { return r.tasks; }); }
  function meanTasks(runs) { return mean(tasksOf(runs)); }
  function range(runs) {
    var t = tasksOf(runs);
    return [Math.min.apply(null, t), Math.max.apply(null, t)];
  }

  // Welch's t-test at the 5% level (two-sided): is the difference bigger than run-to-run variation?
  var T_CRIT = [0, 12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179, 2.160, 2.145, 2.131,
                2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048, 2.045, 2.042];
  function variance(a) {
    var m = mean(a);
    return a.reduce(function (s, x) { return s + (x - m) * (x - m); }, 0) / (a.length - 1);
  }
  function significant(a, b) {
    if (a.length < 2 || b.length < 2) return false;
    var ma = mean(a), mb = mean(b);
    var qa = variance(a) / a.length, qb = variance(b) / b.length;
    var se2 = qa + qb;
    if (se2 === 0) return ma !== mb;                       // both sets of runs are identical numbers
    var t = Math.abs(ma - mb) / Math.sqrt(se2);
    var df = se2 * se2 / ((qa * qa) / (a.length - 1) + (qb * qb) / (b.length - 1));
    var i = Math.max(1, Math.floor(df));
    return t > (i <= 30 ? T_CRIT[i] : 2.0);
  }

  // change against the baseline: green / red when significant, grey otherwise
  function delta(method, sid) {
    var base = baselineFor(method);
    var r = runsOf(method, sid), b = runsOf(base, sid);
    if (!r || !b || method === base) return null;
    var d = (meanTasks(r) - meanTasks(b)) / meanTasks(b) * 100;
    var sig = significant(tasksOf(r), tasksOf(b));
    return el("span", {
      class: sig ? (d > 0 ? "delta up" : "delta down") : "delta",
      title: sig ? "significant (Welch t-test, 5% level)" : "not significant: may be run-to-run variation",
      text: (d > 0 ? "+" : "") + d.toFixed(1) + "%"
    });
  }

  function downloadLink(href, label) {
    return el("a", { href: href, download: basename(href) }, [label]);
  }

  // a clickable row plus a detail row below it that opens and closes with an animation
  function makeExpandable(row, button, detailId, colspan, content) {
    button.setAttribute("aria-expanded", "false");
    button.setAttribute("aria-controls", detailId);
    // the detail row stays in the table; its content collapses to zero height, which is what makes the animation possible
    var panel = el("div", { class: "expand", inert: true, "aria-hidden": "true" }, [el("div", { class: "expand-inner" }, [content])]);
    var detail = el("tr", { class: "detail", id: detailId }, [el("td", { colspan: String(colspan) }, [panel])]);
    row.addEventListener("click", function (e) {
      if (e.target.closest("a")) return;
      var open = button.getAttribute("aria-expanded") !== "true";
      button.setAttribute("aria-expanded", String(open));
      detail.classList.toggle("open", open);
      row.classList.toggle("open", open);
      panel.setAttribute("aria-hidden", String(!open));
      if (open) panel.removeAttribute("inert"); else panel.setAttribute("inert", "");
    });
    return detail;
  }

  // ---------- header, explanation, footer ----------
  document.title = D.meta.title + (D.meta.author ? " | " + D.meta.author : "");
  $("title").textContent = D.meta.title;
  $("description").textContent = D.meta.description || "";

  var mast = $("masthead");
  if (D.meta.logo) {
    var logo = el("img", { class: "logo", src: D.meta.logo, alt: (D.meta.university || "University") + " logo" });
    logo.addEventListener("error", function () { logo.remove(); });   // no file yet: leave the logo out
    mast.appendChild(logo);
  }
  mast.appendChild(el("div", { class: "who" }, [
    D.meta.university ? el("p", { class: "uni", text: D.meta.university }) : null,
    el("p", { class: "people" }, [
      D.meta.author ? el("span", { class: "author", text: D.meta.author }) : null,
      D.meta.supervisor ? el("span", { class: "supervisor" }, [el("span", { class: "label", text: "Supervisor" }), " " + D.meta.supervisor]) : null
    ])
  ]));
  $("setup").textContent = D.meta.setup || "";
  $("updated").textContent = D.meta.updated ? "Last updated " + D.meta.updated + "." : "";
  if (D.meta.repo) $("repo-link").href = D.meta.repo; else $("repo-link").hidden = true;

  // ---------- the table: one tab per map ----------
  var table = $("results");
  var maps = D.maps && D.maps.length ? D.maps : [{ id: "all", name: "All scenarios" }];
  var shown = [];   // the scenarios of the selected map (the columns, and the scenarios listed inside an expanded row)

  function scenariosOf(map) {
    return D.scenarios.filter(function (s) {
      if (s.hidden) return false;   // solved edge cases: the data stays in results.js, the column is not shown
      return maps.length === 1 && !s.mapId ? true : s.mapId === map.id;
    });
  }
  // a method is listed when it has results on this map; C++ rows are always listed, so a missing C++ result shows as n/a
  function listed(m, scen) {
    if (implOf(m) === "cpp") return true;
    return scen.some(function (s) { return runsOf(m, s.id); });
  }

  // one scenario inside an expanded algorithm: a summary row that opens to the list of its runs
  function scenarioRows(m, s, runs, out) {
    var button = el("button", { type: "button", text: s.name });
    var r = range(runs);
    var tm = timingOf(runs);
    var summary = el("tr", { class: "srow" }, [
      el("td", {}, [button]),
      el("td", { class: "num", text: String(runs.length) }),
      el("td", { class: "num" }, [el("strong", { text: fmt(meanTasks(runs)) }), " ", delta(m, s.id) || el("span", { class: "delta", text: "baseline" })]),
      el("td", { class: "num", text: r[0] === r[1] ? String(r[0]) : r[0] + " to " + r[1] }),
      el("td", { class: "num", text: tm ? fmtTime(tm.total) + ", slowest step " + fmtTime(tm.slow) : "n/a" })
    ]);

    var tbody = el("tbody");
    runs.forEach(function (run, i) {
      tbody.appendChild(el("tr", {}, [
        el("td", { class: "num", text: String(i + 1) }),
        el("td", { class: "num", text: String(run.tasks) }),
        el("td", { class: "num", text: String(run.steps || s.steps) }),
        el("td", { class: "num", text: run.plan != null ? fmtTime(run.plan) : "n/a" }),
        el("td", { class: "num", text: run.plan != null ? fmtTime(run.slow) : "n/a" }),
        el("td", {}, [run.file ? downloadLink(run.file, "download result") : "n/a"])
      ]));
    });
    var sub = el("table", { class: "sub" }, [
      el("thead", {}, [el("tr", {}, [
        el("th", { class: "num", text: "Run" }),
        el("th", { class: "num", text: "Tasks finished" }),
        el("th", { class: "num", text: "In time steps" }),
        el("th", { class: "num", text: "Planning time" }),
        el("th", { class: "num", text: "Slowest step" }),
        el("th", { text: "Result file" })
      ])]),
      tbody
    ]);
    var detail = makeExpandable(summary, button, "detail-" + m.id + "-" + s.id, 5,
      el("div", { class: "content sub" }, [el("div", { class: "inner-wrap" }, [sub])]));
    out.appendChild(summary);
    out.appendChild(detail);
  }

  // the expanded part of an algorithm row: conclusion, settings, and its scenarios
  function detailContent(m) {
    var thead = el("thead", {}, [el("tr", {}, [
      el("th", { text: "Scenario" }),
      el("th", { class: "num", text: "Runs" }),
      el("th", { class: "num", text: "Mean tasks, change vs " + baselineFor(m).name }),
      el("th", { class: "num", text: "Min to max" }),
      el("th", { class: "num", text: "Planning time" })
    ])]);
    var tbody = el("tbody");
    shown.forEach(function (s) {
      var runs = runsOf(m, s.id);
      if (runs) scenarioRows(m, s, runs, tbody);
    });

    var settingsLine = el("p", {}, ["Settings: "]);
    if ((m.settings || []).length) {
      m.settings.forEach(function (x, i) {
        if (i) settingsLine.appendChild(document.createTextNode(" "));
        settingsLine.appendChild(el("code", { text: x }));
      });
    } else {
      settingsLine.appendChild(document.createTextNode("none (the unchanged planner)"));
    }
    settingsLine.appendChild(document.createTextNode(" | Kit folder: "));
    settingsLine.appendChild(el("code", { text: m.kit || "n/a" }));

    return el("div", { class: "content" }, [
      m.conclusion ? el("p", {}, [el("strong", { text: "Current conclusion: " }), m.conclusion]) : null,
      el("p", { text: m.description || "" }),
      settingsLine,
      el("div", { class: "inner-wrap" }, [el("table", { class: "runs" }, [thead, tbody])]),
      m.file ? el("p", {}, [downloadLink(m.file, "Download algorithm (" + basename(m.file) + ")")]) : null
    ]);
  }

  var metric = "tasks";     // "tasks" or "time"
  var current = 0;

  // a cell of the planning time view: mean total time of the runs, how it compares with the baseline, slowest step
  function timeCell(m, s) {
    var runs = runsOf(m, s.id);
    var t = runs ? timingOf(runs) : null;
    if (!t) return el("td", { class: "num", text: "n/a" });
    var base = baselineFor(m), bt = m === base ? null : timingOf(runsOf(base, s.id));
    var cmp = null;
    if (bt) {
      var ratio = bt.total / t.total;
      if (ratio >= 1.5) cmp = el("span", { class: "delta up", text: fmtTimes(ratio) + "x faster" });
      else if (ratio <= 1 / 1.5) cmp = el("span", { class: "delta down", text: fmtTimes(1 / ratio) + "x slower" });
      else cmp = el("span", { class: "delta", text: "about the same" });
    }
    var fewRuns = runs.length < 10 ? " \u00B7 " + runs.length + (runs.length === 1 ? " run" : " runs") : "";
    return el("td", { class: "num" }, [
      el("span", { class: "mean", text: fmtTime(t.total) }), cmp,
      el("span", { class: "nruns", text: "slowest step " + fmtTime(t.slow) + fewRuns })
    ]);
  }

  function renderMap(map) {
    shown = scenariosOf(map);
    table.textContent = "";
    var headRow = el("tr", {}, [el("th", { text: "Algorithm" })]);
    shown.forEach(function (s) {
      headRow.appendChild(el("th", { class: "num" }, [
        s.robots + " robots",
        el("small", { text: s.steps + " steps" }),
        s.download ? downloadLink(s.download, "download scenario") : null
      ]));
    });
    table.appendChild(el("thead", {}, [headRow]));
    var cols = shown.length + 1;

    var body = el("tbody");
    D.methods.forEach(function (m) {
      if (!listed(m, shown)) return;
      var button = el("button", { type: "button", text: m.name });
      var row = el("tr", { class: "row", id: m.id }, [
        el("td", {}, [
          button,
          m.baseline ? " (baseline)" : null,
          el("span", { class: "tag", text: implOf(m) === "cpp" ? "C++" : "Python" }),
          m.tag ? el("span", { class: "tag", text: m.tag }) : null,
          el("span", { class: "tagline", text: m.tagline || "" })
        ])
      ]);
      shown.forEach(function (s) {
        if (metric === "time") { row.appendChild(timeCell(m, s)); return; }
        var runs = runsOf(m, s.id);
        if (!runs) { row.appendChild(el("td", { class: "num", text: "n/a" })); return; }
        // fewer runs than the usual 10: say so, so a few runs are not read as a proper average
        var few = runs.length < 10 ? el("span", { class: "nruns", text: runs.length + (runs.length === 1 ? " run" : " runs") }) : null;
        row.appendChild(el("td", { class: "num" }, [el("span", { class: "mean", text: fmt(meanTasks(runs)) }), delta(m, s.id), few]));
      });
      var detail = makeExpandable(row, button, "detail-" + m.id, cols, detailContent(m));
      body.appendChild(row);
      body.appendChild(detail);
    });
    table.appendChild(body);
    $("map-note").textContent = map.note || "";
    $("metric-note").textContent = metric === "time"
      ? "Total time the planner spent over the whole run (all steps added up), and the slowest single step. Taken from the same runs as the task counts; several runs shared one laptop, so read them as estimates. Only the C++ rows were timed."
      : "";
  }

  var tabs = $("tabs");
  function select(i) {
    current = i;
    Array.prototype.forEach.call(tabs.children, function (b, k) { b.setAttribute("aria-selected", String(k === i)); });
    renderMap(maps[i]);
  }
  // switch between task counts and planning time (only offered when planning times are loaded)
  var metricBox = $("metric");
  if (window.TIMING) {
    [["tasks", "Tasks finished"], ["time", "Planning time"]].forEach(function (o) {
      var b = el("button", { type: "button", class: "seg", text: o[1], "aria-pressed": String(o[0] === metric) });
      b.addEventListener("click", function () {
        metric = o[0];
        Array.prototype.forEach.call(metricBox.querySelectorAll(".seg"), function (x) { x.setAttribute("aria-pressed", String(x === b)); });
        select(current);
      });
      metricBox.appendChild(b);
    });
  } else {
    metricBox.hidden = true;
  }
  if (maps.length > 1) {
    maps.forEach(function (map, i) {
      var b = el("button", { type: "button", class: "tab", role: "tab", text: map.name });
      b.addEventListener("click", function () { select(i); });
      tabs.appendChild(b);
    });
  } else {
    tabs.hidden = true;
  }

  // open the row named in the address (index.html#wait-action), on the first map where it is listed
  var wanted = location.hash.slice(1);
  var start = 0;
  var wm = D.methods.filter(function (m) { return m.id === wanted; })[0];
  if (wm) {
    for (var q = 0; q < maps.length; q++) {
      if (listed(wm, scenariosOf(maps[q]))) { start = q; break; }
    }
  }
  select(start);
  if (wanted && document.getElementById(wanted) && document.getElementById(wanted).classList.contains("row")) {
    document.getElementById(wanted).click();
  }
})();

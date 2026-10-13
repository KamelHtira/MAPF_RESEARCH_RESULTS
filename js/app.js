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

  var baseline = D.methods.filter(function (m) { return m.baseline; })[0] || D.methods[0];

  // the runs of a method on a scenario, each as an object (a bare number means just the task count)
  function runsOf(method, sid) {
    var list = method.results && method.results[sid];
    if (!list || !list.length) return null;
    return list.map(function (r) { return typeof r === "number" ? { tasks: r } : r; });
  }
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
    var r = runsOf(method, sid), b = runsOf(baseline, sid);
    if (!r || !b || method === baseline) return null;
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
  document.title = D.meta.title;
  $("title").textContent = D.meta.title;
  $("subtitle").textContent = D.meta.subtitle || "";
  $("explain").textContent =
    "Each row is an algorithm that was tried. Each column is a test scenario: the same map with a different number of robots. " +
    "The number in a cell is the mean number of tasks finished over all runs (higher is better), and the line under it is the change against \"" +
    baseline.name + "\". The change is green or red only when it is statistically significant (Welch's t-test, 5% level), and grey otherwise, " +
    "because then it may just be run-to-run variation. The test is run separately for every cell, so one or two coloured cells can still be chance. " +
    "Click a row to see its conclusion and settings, then click a scenario to list its individual runs, each with a download of its result file. " +
    "The links in the column headers download the scenario files, so the same problem can be run again.";
  $("setup").textContent = D.meta.setup || "";
  $("updated").textContent = D.meta.updated ? "Last updated " + D.meta.updated + "." : "";
  if (D.meta.repo) $("repo-link").href = D.meta.repo; else $("repo-link").hidden = true;

  // ---------- the table ----------
  var table = $("results");
  var cols = D.scenarios.length + 1;

  var headRow = el("tr", {}, [el("th", { text: "Algorithm" })]);
  D.scenarios.forEach(function (s) {
    headRow.appendChild(el("th", { class: "num" }, [
      s.robots + " robots",
      el("small", { text: s.steps + " steps" }),
      s.download ? downloadLink(s.download, "download scenario") : null
    ]));
  });
  table.appendChild(el("thead", {}, [headRow]));

  // one scenario inside an expanded algorithm: a summary row that opens to the list of its runs
  function scenarioRows(m, s, runs, out) {
    var button = el("button", { type: "button", text: s.name });
    var r = range(runs);
    var summary = el("tr", { class: "srow" }, [
      el("td", {}, [button]),
      el("td", { class: "num", text: String(runs.length) }),
      el("td", { class: "num" }, [el("strong", { text: fmt(meanTasks(runs)) }), " ", delta(m, s.id) || el("span", { class: "delta", text: "baseline" })]),
      el("td", { class: "num", text: r[0] === r[1] ? String(r[0]) : r[0] + " to " + r[1] })
    ]);

    var tbody = el("tbody");
    runs.forEach(function (run, i) {
      tbody.appendChild(el("tr", {}, [
        el("td", { class: "num", text: String(i + 1) }),
        el("td", { class: "num", text: String(run.tasks) }),
        el("td", { class: "num", text: String(run.steps || s.steps) }),
        el("td", {}, [run.file ? downloadLink(run.file, "download result") : "n/a"])
      ]));
    });
    var sub = el("table", { class: "sub" }, [
      el("thead", {}, [el("tr", {}, [
        el("th", { class: "num", text: "Run" }),
        el("th", { class: "num", text: "Tasks finished" }),
        el("th", { class: "num", text: "In time steps" }),
        el("th", { text: "Result file" })
      ])]),
      tbody
    ]);
    var detail = makeExpandable(summary, button, "detail-" + m.id + "-" + s.id, 4,
      el("div", { class: "content sub" }, [el("div", { class: "inner-wrap" }, [sub])]));
    out.appendChild(summary);
    out.appendChild(detail);
  }

  // the expanded part of an algorithm row: conclusion, settings, and its scenarios
  function detailContent(m) {
    var thead = el("thead", {}, [el("tr", {}, [
      el("th", { text: "Scenario" }),
      el("th", { class: "num", text: "Runs" }),
      el("th", { class: "num", text: "Mean tasks, change vs " + baseline.name }),
      el("th", { class: "num", text: "Min to max" })
    ])]);
    var tbody = el("tbody");
    D.scenarios.forEach(function (s) {
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

  var body = el("tbody");
  D.methods.forEach(function (m) {
    var button = el("button", { type: "button", text: m.name });
    var row = el("tr", { class: "row", id: m.id }, [
      el("td", {}, [button, m.baseline ? " (baseline)" : null, el("span", { class: "tagline", text: m.tagline || "" })])
    ]);
    D.scenarios.forEach(function (s) {
      var runs = runsOf(m, s.id);
      if (!runs) { row.appendChild(el("td", { class: "num", text: "n/a" })); return; }
      row.appendChild(el("td", { class: "num" }, [el("span", { class: "mean", text: fmt(meanTasks(runs)) }), delta(m, s.id)]));
    });
    var detail = makeExpandable(row, button, "detail-" + m.id, cols, detailContent(m));
    body.appendChild(row);
    body.appendChild(detail);
  });
  table.appendChild(body);

  // open the row named in the address (index.html#wait-action)
  var wanted = location.hash.slice(1);
  if (wanted && document.getElementById(wanted) && document.getElementById(wanted).classList.contains("row")) {
    document.getElementById(wanted).click();
  }
})();

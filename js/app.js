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
  function meanTasks(runs) { return mean(runs.map(function (r) { return r.tasks; })); }

  function range(runs) {
    var t = runs.map(function (r) { return r.tasks; });
    return [Math.min.apply(null, t), Math.max.apply(null, t)];
  }

  // change against the baseline: green / red, but grey when the runs overlap the baseline's runs (it may be run-to-run variation)
  function delta(method, sid) {
    var r = runsOf(method, sid), b = runsOf(baseline, sid);
    if (!r || !b || method === baseline) return null;
    var d = (meanTasks(r) - meanTasks(b)) / meanTasks(b) * 100;
    var rr = range(r), br = range(b);
    var overlap = rr[0] <= br[1] && br[0] <= rr[1];
    var cls = overlap ? "delta" : (d > 0 ? "delta up" : "delta down");
    return el("span", { class: cls, text: (d > 0 ? "+" : "") + d.toFixed(1) + "%" });
  }

  function downloadLink(href, label) {
    return el("a", { href: href, download: basename(href) }, [label]);
  }

  // ---------- header, explanation, footer ----------
  document.title = D.meta.title;
  $("title").textContent = D.meta.title;
  $("subtitle").textContent = D.meta.subtitle || "";
  $("explain").textContent =
    "Each row is an algorithm that was tried. Each column is a test scenario: the same map with a different number of robots. " +
    "The number in a cell is the mean number of tasks finished over several runs (higher is better), and the line under it is the change against \"" +
    baseline.name + "\". A change is grey when the runs of that algorithm overlap the runs of \"" + baseline.name + "\", because then it may just be run-to-run variation. " +
    "Click a row to see its conclusion, its settings, and every individual run with a download of that run's result file. " +
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

  // the expanded part of a row: conclusion, settings, and one table row per run
  function detailContent(m) {
    var thead = el("thead", {}, [el("tr", {}, [
      el("th", { text: "Scenario" }),
      el("th", { class: "num", text: "Run" }),
      el("th", { class: "num", text: "Tasks finished" }),
      el("th", { class: "num", text: "In time steps" }),
      el("th", { text: "Result file" })
    ])]);

    var tbody = el("tbody");
    D.scenarios.forEach(function (s) {
      var runs = runsOf(m, s.id);
      if (!runs) return;
      runs.forEach(function (run, i) {
        var tr = el("tr");
        if (i === 0) {
          tr.appendChild(el("td", { rowspan: String(runs.length), class: "scn" }, [
            el("strong", { text: s.name }),
            el("span", { class: "meanline" }, ["mean " + fmt(meanTasks(runs)) + " ", delta(m, s.id) || "(baseline)"])
          ]));
        }
        tr.appendChild(el("td", { class: "num", text: String(i + 1) }));
        tr.appendChild(el("td", { class: "num", text: String(run.tasks) }));
        tr.appendChild(el("td", { class: "num", text: String(run.steps || s.steps) }));
        tr.appendChild(el("td", {}, [run.file ? downloadLink(run.file, "download result") : "n/a"]));
        if (i === 0) tr.classList.add("first-run");
        tbody.appendChild(tr);
      });
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
    var detailId = "detail-" + m.id;
    var button = el("button", { type: "button", "aria-expanded": "false", "aria-controls": detailId, text: m.name });
    var row = el("tr", { class: "row", id: m.id }, [
      el("td", {}, [button, m.baseline ? " (baseline)" : null, el("span", { class: "tagline", text: m.tagline || "" })])
    ]);
    D.scenarios.forEach(function (s) {
      var runs = runsOf(m, s.id);
      if (!runs) { row.appendChild(el("td", { class: "num", text: "n/a" })); return; }
      row.appendChild(el("td", { class: "num" }, [el("span", { class: "mean", text: fmt(meanTasks(runs)) }), delta(m, s.id)]));
    });

    // the detail row stays in the table; its content collapses to zero height, which is what makes the animation possible
    var panel = el("div", { class: "expand", inert: true, "aria-hidden": "true" }, [el("div", { class: "expand-inner" }, [detailContent(m)])]);
    var detail = el("tr", { class: "detail", id: detailId }, [el("td", { colspan: String(cols) }, [panel])]);

    row.addEventListener("click", function (e) {
      if (e.target.closest("a")) return;
      var open = button.getAttribute("aria-expanded") !== "true";
      button.setAttribute("aria-expanded", String(open));
      detail.classList.toggle("open", open);
      row.classList.toggle("open", open);
      panel.setAttribute("aria-hidden", String(!open));
      if (open) panel.removeAttribute("inert"); else panel.setAttribute("inert", "");
    });

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

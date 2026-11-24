/* The Tools menu shared by the results page and the tool pages. Put <span id="toolmenu"></span> where it should appear
   and load this script; add a tool by adding one line to TOOLS (hrefs are relative to the site root). */
(function () {
  "use strict";
  var TOOLS = [
    { href: "tools/waypoints.html", name: "Map views", desc: "cover points, route waypoints and junction clusters of a map" },
    { href: "tools/lanes.html", name: "Guidance algorithms", desc: "one-way lanes on a map" },
    { href: "tools/coverdemo.html", name: "PIBT-CP demo", desc: "a small grid, step by step: lists, Dijkstra, robot steps" }
  ];
  var host = document.getElementById("toolmenu");
  if (!host) { return; }
  var root = "", scripts = document.getElementsByTagName("script");
  for (var i = 0; i < scripts.length; i++) {
    var src = scripts[i].getAttribute("src") || "", k = src.indexOf("js/toolmenu.js");
    if (k >= 0) { root = src.slice(0, k); break; }
  }
  var details = document.createElement("details");
  details.className = "toolmenu";
  var summary = document.createElement("summary");
  summary.textContent = "Tools";
  var list = document.createElement("div");
  list.className = "toolmenu-list";
  TOOLS.forEach(function (t) {
    var a = document.createElement("a"), name = document.createElement("strong"), desc = document.createElement("span");
    a.href = root + t.href;
    name.textContent = t.name;
    desc.textContent = t.desc;
    a.appendChild(name);
    a.appendChild(desc);
    if (location.pathname.slice(-t.href.length) === t.href) { a.className = "here"; a.setAttribute("aria-current", "page"); }
    list.appendChild(a);
  });
  details.appendChild(summary);
  details.appendChild(list);
  host.appendChild(details);
  document.addEventListener("click", function (e) { if (details.open && !details.contains(e.target)) { details.open = false; } });
  document.addEventListener("keydown", function (e) { if (e.key === "Escape") { details.open = false; } });
})();

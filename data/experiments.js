/*
  The experiments section of the page (below the results table).
  Each item is a table: `columns` are the headers, each row is a list of cells.
  A cell is a plain string, or [text, change, kind] where change is a small note under the text
  and kind is "up" (green), "down" (red) or "" (grey, not significant).
  A row can be written { cells: [...], key: true } to be highlighted.
*/
window.EXPERIMENTS = {
  title: "Which parts of the planner are worth keeping? (November 2026)",
  intro: "Follow-up runs of the C++ planner to decide what to keep, what to remove and what to build next. " +
         "Changes are against Default. In experiment 1 (10 runs) a change is green or red only when it is bigger than run-to-run variation (Welch t-test, 5% level), otherwise grey. Experiments 2 to 4 are single runs, so colour shows direction only. Planning times are estimates: several runs shared one laptop.",
  decisions: [
    "Keep: the priority swap when the search finds no path (call it G). It gives most of the gain of the full version at the cost of Default.",
    "Remove: the swap triggered by a long detour (about +1% for twice the searches) and, for now, the wait move (no gain alone).",
    "Use the true-distance heuristic on maps with walls (Paris, brc202d); it makes no difference on the warehouse maps.",
    "Next: spread the planning of the first step over several steps, so a time limit per step can be met, and make the heuristic aware of turns."
  ],
  items: [
    {
      id: "ablation",
      title: "1. What does each part of v2 add?",
      setup: "warehouse_small, C++, 10 runs per cell (seeds 1 to 10). Mean tasks finished, change against Default.",
      columns: ["Variant", "50 robots, 300 steps", "100 robots, 300 steps", "100 robots, 2000 steps", "Stuck runs (of 10, 2000 steps)", "Planning time (2000 steps)"],
      rows: [
        ["Default", "380.8", "697.8", "4757", "4", "7 s"],
        ["Wait action only", ["387.7", "+1.8%", ""], ["697.9", "+0.0%", ""], ["4740", "-0.4%", ""], "4", "10 s"],
        { cells: ["Swap on empty path only (G)", ["395.9", "+4.0%", "up"], ["753.6", "+8.0%", "up"], ["5261", "+10.6%", "up"], "1", "9 s"], key: true },
        ["Swap on empty path and on long detour", ["397.1", "+4.3%", "up"], ["753.7", "+8.0%", "up"], ["5305", "+11.5%", "up"], "0", "17 s"],
        ["Wait + swap on empty path only", ["397.5", "+4.4%", "up"], ["760.7", "+9.0%", "up"], ["5302", "+11.5%", "up"], "0", "13 s"],
        ["Wait + full swap (v2)", ["398.4", "+4.6%", "up"], ["765.9", "+9.8%", "up"], ["5366", "+12.8%", "up"], "0", "18 s"],
        ["Wait + swap on long detour only (v1)", ["375.9", "-1.3%", ""], ["649.7", "-6.9%", "down"], ["4106", "-13.7%", "down"], "8", "25 s"],
        ["Default + true-distance heuristic", ["383.1", "+0.6%", ""], ["694.7", "-0.4%", ""], ["4743", "-0.3%", ""], "3", "6 s"],
        ["v2 + true-distance heuristic", ["400.7", "+5.2%", "up"], ["771.0", "+10.5%", "up"], ["5343", "+12.3%", "up"], "0", "16 s"]
      ],
      finding: "The swap when the search finds no path is the whole gain: +8% to +11%. The wait move alone does nothing and makes each search about 40% bigger. The long-detour swap adds about 1% and runs an extra search for every robot, so G keeps nearly all of v2 with Default's planning cost. v1 (no empty-path swap) is worse than Default."
    },
    {
      id: "limit",
      title: "2. Tasks finished with a 1 second limit per step",
      setup: "Competition style: a step that takes more than 1 s counts as a timeout. 300 steps, 1 run each, one run at a time on an idle machine.",
      columns: ["Algorithm", "warehouse_large, 1000 robots", "sortation_large, 1200 robots", "Steps lost to timeouts (warehouse / sortation)"],
      rows: [
        ["Default", "822", "990", "18 / 24"],
        ["Wait action + priority swap v2", ["705", "-14.2%", "down"], ["861", "-13.0%", "down"], "48 / 52"],
        { cells: ["Swap on empty path only (G)", ["828", "+0.7%", ""], ["992", "+0.2%", ""], "18 / 24"], key: true },
        ["G + true-distance heuristic", ["835", "+1.6%", ""], ["987", "-0.3%", ""], "18 / 26"],
        ["SIPP with waypoints", ["794", "-3.4%", ""], ["1027", "+3.7%", ""], "1 / 7"]
      ],
      finding: "With a limit, v2 is the worst: its first steps take too long and about 50 steps are lost. Default and G also lose 18 to 24 steps at the start, because the first step takes 20 to 40 s; SIPP loses almost none. G is as good as Default (a single run, so the small differences mean nothing). Without any limit G scores 921 and 1139 and v2 922.7 and 1140, the same as in the main table."
    },
    {
      id: "density",
      title: "3. More robots and more steps on warehouse_large",
      setup: "No time limit, 1 run each. Planning time is the whole run; first step in brackets.",
      columns: ["Algorithm", "1000 robots, 1000 steps", "3000 robots, 300 steps", "Planning time (1000 / 3000 robots)"],
      rows: [
        ["Default", "3752", "2599", "176 s (19 s) / 266 s (77 s)"],
        ["Wait action + priority swap v2", ["3871", "+3.2%", "up"], ["2673", "+2.8%", "up"], "324 s (56 s) / 593 s (290 s)"],
        ["SIPP with waypoints", ["3472", "-7.5%", "down"], ["1898", "-27.0%", "down"], "31 s (1.8 s) / 56 s (5.4 s)"]
      ],
      finding: "v2 keeps a small lead as robots get denser, at 3 to 5 times the first-step time. SIPP falls behind faster the more crowded the map is. Changes here come from single runs and are not tested for significance."
    },
    {
      id: "heuristic",
      title: "4. Is the heuristic the reason the search is slow?",
      setup: "Default planner, 1000 robots, 1 run. Manhattan distance against the true shortest-path distance (a BFS table per goal). A* expansions per search at step 0.",
      columns: ["Map", "Manhattan: expansions per search", "True distance: expansions per search", "First step: Manhattan / true distance", "Whole run (300 steps)", "Tasks"],
      rows: [
        ["warehouse_large", "9,941", "10,021", "22 s / 24 s", "about the same", "914 against 905 to 918"],
        { cells: ["Paris", "239,677", "28,864", "17.6 min / 1.5 min", "about 44 min / 3.2 min", "1139 against 1144 to 1145"], key: true },
        { cells: ["brc202d", "did not finish one step in 30 min", "65,702", "n/a / 4 min", "n/a / 7 min", "413 (SIPP: 370, 11 s)"], key: true }
      ],
      finding: "On the warehouse maps the shelves form open aisles, so Manhattan is already almost exact and the cost is turns and time, not the heuristic. On maps with walls the true distance cuts the search by 8 times and the run by 14 times, and makes brc202d possible at all. v2 with the true distance on Paris: 1153 tasks in 17 min; on brc202d it did not finish within 48 min."
    }
  ],
  caveats: "Big-map rows (experiments 2 to 4) are single runs, so read them as indications. Runs that shared the laptop have inflated times, which is why search counts are shown next to them."
};

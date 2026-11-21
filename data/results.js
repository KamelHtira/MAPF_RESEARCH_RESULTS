/*
  ALL THE CONTENT OF THE WEBSITE LIVES IN THIS FILE.

  To add a method:     copy one object in `methods`, give it a new id, point `file` at its planner file
                       (put the file in downloads/algorithms/) and fill in `results`.
  To add a scenario:   copy one object in `scenarios` (put its zip in downloads/scenarios/) and add an entry for it
                       under `results` of every method that was run on it.
  `results[scenarioId] = runs(methodId, scenarioId, [tasks, tasks, ...])`, one number per run:
      the tasks finished in that run (the page works out the mean and the change against the baseline).
  The result file of run N is expected at  downloads/runs/<methodId>_<scenarioId>_<N>.json
  (that is the file the planner wrote with  -o  ; PlanViz can open it again).
  A run can also be written out by hand, e.g.  { tasks: 380, steps: 500, file: "downloads/runs/x.json" }
  `steps` is optional and defaults to the scenario's steps.
  `conclusion` = one or two sentences shown when the row is expanded (update it as you learn more).
  Exactly one method must have `baseline: true`.
*/
function runs(method, scenario, taskCounts) {
  return taskCounts.map(function (tasks, i) {
    return { tasks: tasks, file: "downloads/runs/" + method + "_" + scenario + "_" + (i + 1) + ".json" };
  });
}

// big-map runs are 2 MB each: only run 1 has a result file, the others are listed as numbers
function runsSample(method, scenario, taskCounts) {
  return taskCounts.map(function (tasks, i) {
    return i === 0 ? { tasks: tasks, file: "downloads/runs/" + method + "_" + scenario + "_1.json" } : { tasks: tasks };
  });
}

window.RESULTS = {
  meta: {
    title: "MAPF Research Results",
    description: "This page is part of a PhD project on Multi-Agent Path Finding (MAPF): planning collision-free routes for many robots that share the same space. " +
                 "The work is built around the League of Robot Runners competition, a lifelong warehouse setting in which robots keep receiving new tasks, can move forward, turn or wait, " +
                 "and are scored by how many tasks they finish in a fixed number of steps. ",
    author: "Kamel Htira",
    supervisor: "Varga László Zsolt",
    university: "Eötvös Loránd University (ELTE)",
    logo: "assets/elte-logo.svg",   // the official logo file; the page just leaves it out if the file is missing
    updated: "2026-11-21",
    setup: "Kit: League of Robot Runners start-kit v1.1.5 (2023), built in WSL Ubuntu. Python rows ran in the kit's Python planner, C++ rows in a C++ port of it. " +
           "Python rows: 10 runs per setting. C++ rows: 10 runs on warehouse_small (seeds 1 to 10) and 3 runs on the big maps (PIBT and SIPP: 1 run). Runs of one setting differ because equal-cost paths are tie-broken by memory address (Python) or a seed (C++), so each implementation is compared with its own Default. " +
           "A robot counts as stuck when it makes no forward move in the last 60 steps of a run.",
    repo: "https://github.com/KamelHtira/MAPF_RESEARCH_RESULTS"
  },

  maps: [
    { id: "warehouse_small", name: "warehouse_small", note: "33 x 57 cells. The scenarios of the first part of the project." },
    { id: "warehouse_large", name: "warehouse_large", note: "140 x 500 cells, 1000 robots. C++ only, 3 runs per setting (SIPP: 1 run)." },
    { id: "sortation_large", name: "sortation_large", note: "140 x 500 cells, 1200 robots. C++ only, 3 runs per setting (SIPP: 1 run)." },
    { id: "paris", name: "Paris city", note: "256 x 256 cells, 1000 robots. C++ only. Default needs about 44 minutes for 300 steps; Wait action + priority swap v2 is not run yet. SIPP: 1 run." },
    { id: "brc202d", name: "brc202d", note: "481 x 530 game map, 1000 robots. Default and Wait action + priority swap v2 did not finish (more than an hour per run); SIPP finishes in about a minute." }
  ],

  scenarios: [
    {
      id: "s2",
      mapId: "warehouse_small",
      hidden: true,   // solved edge case, not shown on the page
      name: "2 robots: one agent pushed along",
      map: "warehouse_small (33 x 57)",
      robots: 2,
      steps: 400,
      description: "Two agents in a 1-wide aisle. One agent has no time to turn aside, so the other pushes it along the aisle for a long time.",
      files: ["wh_small_2.json", "maps/warehouse_small.map", "agents/wh_small_2.agents", "tasks/wh_small_2.tasks", "README.txt"],
      download: "downloads/scenarios/s2.zip"
    },
    {
      id: "s10",
      mapId: "warehouse_small",
      hidden: true,   // solved edge case, not shown on the page
      name: "10 robots",
      map: "warehouse_small (33 x 57)",
      robots: 10,
      steps: 300,
      description: "The kit's 10-robot example.",
      files: ["warehouse_small_10.json", "maps/warehouse_small.map", "agents/warehouse_small_10.agents", "tasks/warehouse_small.tasks", "README.txt"],
      download: "downloads/scenarios/s10.zip"
    },
    {
      id: "s50",
      mapId: "warehouse_small",
      name: "50 robots",
      map: "warehouse_small (33 x 57)",
      robots: 50,
      steps: 300,
      description: "The kit's 50-robot example.",
      files: ["warehouse_small_50.json", "maps/warehouse_small.map", "agents/warehouse_small_50.agents", "tasks/warehouse_small.tasks", "README.txt"],
      download: "downloads/scenarios/s50.zip"
    },
    {
      id: "s100",
      mapId: "warehouse_small",
      name: "100 robots",
      map: "warehouse_small (33 x 57)",
      robots: 100,
      steps: 300,
      description: "The kit's 100-robot example.",
      files: ["warehouse_small_100.json", "maps/warehouse_small.map", "agents/warehouse_small_100.agents", "tasks/warehouse_small.tasks", "README.txt"],
      download: "downloads/scenarios/s100.zip"
    },
    {
      id: "s10L",
      mapId: "warehouse_small",
      name: "10 robots, 2000 steps",
      map: "warehouse_small (33 x 57)",
      robots: 10,
      steps: 2000,
      description: "The 10-robot example run for 2000 steps instead of 300, to see whether the gain keeps growing. C++ only, no result files.",
      files: ["warehouse_small_10.json", "maps/warehouse_small.map", "agents/warehouse_small_10.agents", "tasks/warehouse_small.tasks", "README.txt"],
      download: "downloads/scenarios/s10.zip"
    },
    {
      id: "s100L",
      mapId: "warehouse_small",
      name: "100 robots, 2000 steps",
      map: "warehouse_small (33 x 57)",
      robots: 100,
      steps: 2000,
      description: "The 100-robot example run for 2000 steps instead of 300. C++ only, no result files.",
      files: ["warehouse_small_100.json", "maps/warehouse_small.map", "agents/warehouse_small_100.agents", "tasks/warehouse_small.tasks", "README.txt"],
      download: "downloads/scenarios/s100.zip"
    },
    {
      id: "wl1000",
      mapId: "warehouse_large",
      name: "1000 robots",
      map: "warehouse_large (140 x 500)",
      robots: 1000,
      steps: 300,
      description: "The kit's warehouse_large_1000 example. 3 runs per setting; only run 1 has a result file (2 MB each).",
      files: ["warehouse_large_1000.json", "maps/warehouse_large.map", "agents/warehouse_large_1000.agents", "tasks/warehouse_large.tasks"]
    },
    {
      id: "sl1200",
      mapId: "sortation_large",
      name: "1200 robots",
      map: "sortation_large (140 x 500)",
      robots: 1200,
      steps: 300,
      description: "The kit's sortation_large_1200 example. 3 runs per setting; only run 1 has a result file.",
      files: ["sortation_large_1200.json", "maps/sortation_large.map", "agents/sortation_large_1200.agents", "tasks/sortation_large.tasks"]
    },
    {
      id: "paris1000",
      mapId: "paris",
      name: "1000 robots",
      map: "Paris_1_256 (256 x 256)",
      robots: 1000,
      steps: 300,
      description: "The kit's paris_1000 example. 3 runs per setting; only run 1 has a result file.",
      files: ["paris_1000.json", "maps/Paris_1_256.map", "agents/paris_1000.agents", "tasks/Paris.tasks"]
    },
    {
      id: "brc1000",
      mapId: "brc202d",
      name: "1000 robots",
      map: "brc202d (481 x 530)",
      robots: 1000,
      steps: 300,
      description: "The kit's brc202d_1000 example. Not run yet.",
      files: ["brc202d_1000.json", "maps/brc202d.map", "agents/brc202d_1000.agents", "tasks/brc202d.tasks"]
    }
  ],

  methods: [
    {
      id: "default",
      name: "Default",
      baseline: true,
      tagline: "Prioritized planning with space-time A*.",
      description: "Robots plan one after another in index order. Each robot runs A* in space and time around the squares the earlier robots reserved, and a safety net makes two robots that would collide drop their paths and turn in place. A robot can only go forward, turn left or turn right, never wait.",
      settings: [],
      kit: "Start-Kit-default",
      file: "downloads/algorithms/default_planner.py",
      conclusion: "Reference point. At 10 robots 9 of 10 runs end with stuck robots, all from the same head-on jam at step 113.",
      results: {
        s2:   runs("default", "s2", [23, 23, 23, 23, 23, 23, 23, 23, 23, 23]),
        s10:  runs("default", "s10", [70, 73, 73, 70, 76, 73, 70, 70, 73, 70]),
        s50:  runs("default", "s50", [398, 390, 398, 398, 377, 398, 398, 398, 398, 398]),
        s100: runs("default", "s100", [709, 697, 661, 717, 726, 726, 717, 732, 717, 698])
      }
    },
    {
      id: "delay-fix",
      name: "1-tick delay of the blocker",
      tagline: "Make a nearby robot wait 1 or 2 ticks when a robot's route is much longer than ideal.",
      description: "After a robot plans, it compares its path with its ideal path (the same trip with all other robots removed). If it is 6 or more ticks longer, one of the 3 nearest robots is asked to wait 1 or 2 ticks and the robot replans. The change is kept only if it saves at least 3 ticks in total.",
      settings: ["DELAY_FIX=1", "DETOUR_LIMIT=6", "DETOUR_MARGIN=3"],
      kit: "Start-Kit-delay-fix",
      file: "downloads/algorithms/delay_fix_planner.py",
      conclusion: "Better at 10 robots (+6.3%), no clear change at 50 or 100. Almost never ends with stuck robots.",
      results: {
        s2:   runs("delay-fix", "s2", [23, 23, 23, 23, 23, 23, 23, 23, 23, 23]),
        s10:  runs("delay-fix", "s10", [79, 76, 73, 76, 76, 79, 76, 76, 76, 76]),
        s50:  runs("delay-fix", "s50", [399, 402, 399, 399, 367, 361, 399, 399, 402, 393]),
        s100: runs("delay-fix", "s100", [720, 723, 720, 718, 714, 711, 686, 741, 711, 686])
      }
    },
    {
      id: "wait-action",
      name: "Wait action in A*",
      tagline: "Let the search wait in place instead of only turning.",
      description: "A* gets a fourth move: wait in place and keep the facing. Without it, a robot can only wait by turning, and it needs two turns to face the same way again.",
      settings: ["WAIT_ACTION=1", "PRIORITY_SWAP=0"],
      kit: "Start-Kit-wait-priority",
      file: "downloads/algorithms/wait_action_planner.py",
      conclusion: "Best single change at 10 robots (+14.6%), no gain at 50 or 100. Does not remove the step-113 jam: robots 4 and 8 stay frozen for 131 steps in 2 of 10 runs.",
      results: {
        s2:   runs("wait-action", "s2", [23, 23, 23, 23, 23, 23, 23, 23, 23, 23]),
        s10:  runs("wait-action", "s10", [83, 83, 79, 84, 83, 83, 79, 83, 83, 83]),
        s50:  runs("wait-action", "s50", [401, 362, 393, 390, 373, 382, 392, 394, 392, 369]),
        s100: runs("wait-action", "s100", [701, 659, 704, 695, 703, 750, 673, 723, 673, 697])
      }
    },
    {
      id: "priority-swap",
      name: "Priority swap",
      tagline: "Let the stuck robot plan first and the blocker replan around it.",
      description: "When a robot's path is 6 or more ticks longer than its ideal path, find the robots whose reserved paths cross its ideal route. The stuck robot then plans first without them, and the blocker replans around the new path. Kept only if the two robots together save at least 3 ticks.",
      settings: ["WAIT_ACTION=0", "PRIORITY_SWAP=1", "DETOUR_LIMIT=6", "DETOUR_MARGIN=3"],
      kit: "Start-Kit-wait-priority",
      file: "downloads/algorithms/priority_swap_planner.py",
      conclusion: "Fixes the 2-robot case (24 against 23). Mixed at 10 robots, worse at 50. Skips robots with no path, so it never fires at the step-113 jam (fixed in v2).",
      results: {
        s2:   runs("priority-swap", "s2", [24, 24, 24, 24, 24, 24, 24, 24, 24, 24]),
        s10:  runs("priority-swap", "s10", [83, 70, 70, 83, 70, 70, 70, 83, 83, 83]),
        s50:  runs("priority-swap", "s50", [391, 371, 360, 360, 378, 391, 389, 372, 391, 378]),
        s100: runs("priority-swap", "s100", [737, 725, 714, 706, 715, 717, 712, 705, 705, 717])
      }
    },
    {
      id: "wait-and-swap",
      name: "Wait action + priority swap",
      tagline: "Both changes together.",
      description: "The wait action in A* and the priority swap enabled at the same time (the default of the Start-Kit-wait-priority folder).",
      settings: ["WAIT_ACTION=1", "PRIORITY_SWAP=1"],
      kit: "Start-Kit-wait-priority",
      file: "downloads/algorithms/wait_and_swap_planner.py",
      conclusion: "Same 2-robot gain, but worse than Default at 100 robots (-6.8%) and 7 of 10 runs end stuck. Not recommended.",
      results: {
        s2:   runs("wait-and-swap", "s2", [24, 24, 24, 24, 24, 24, 24, 24, 24, 24]),
        s10:  runs("wait-and-swap", "s10", [83, 70, 73, 73, 83, 83, 73, 73, 70, 70]),
        s50:  runs("wait-and-swap", "s50", [392, 370, 393, 391, 389, 399, 382, 392, 393, 390]),
        s100: runs("wait-and-swap", "s100", [720, 623, 596, 635, 667, 618, 718, 667, 720, 652])
      }
    },
    {
      id: "priority-swap-v2",
      name: "Priority swap",
      tag: "v2",
      tagline: "Priority swap that also fires when the search finds no path at all.",
      description: "Same as the priority swap, with one more case. When the A* finds no path at all for a robot (every route is blocked by reservations), the swap is tried too: the robots whose reserved paths cross its ideal route are found, the stuck robot plans first without them, the blocker replans around it, and the cheapest pair of paths is taken. There is no 'saves at least 3 ticks' check in this case, since there is nothing to compare against. The wait action stays off.",
      settings: ["WAIT_ACTION=0", "PRIORITY_SWAP=1", "DETOUR_LIMIT=6", "DETOUR_MARGIN=3"],
      kit: "Start-Kit-priority-swap-v2",
      file: "downloads/algorithms/priority_swap_v2_planner.py",
      conclusion: "Removes the step-113 jam: 83 or 84 tasks in all 10 runs at 10 robots (+16.2%). At 100 robots +4.1%, borderline, and 3 of 10 runs still end stuck.",
      results: {
        s2:  runs("priority-swap-v2", "s2", [24, 24, 24, 24, 24, 24, 24, 24, 24, 24]),
        s10: runs("priority-swap-v2", "s10", [83, 83, 84, 84, 84, 83, 83, 84, 83, 83]),
        s50: runs("priority-swap-v2", "s50", [400, 402, 399, 400, 400, 400, 394, 399, 400, 399]),
        s100:runs("priority-swap-v2", "s100", [760, 760, 757, 772, 772, 760, 760, 684, 684, 684])
      }
    },
    {
      id: "wait-and-swap-v2",
      name: "Wait action + priority swap",
      tag: "v2",
      tagline: "Both changes together, with the swap also firing when there is no path.",
      description: "The wait action in A* and the priority swap enabled together, with the same extra case as the other v2 row: the swap is also tried when the search finds no path at all, and the cheapest pair of paths is taken. With the wait move the blocker can simply hold its square while the stuck robot turns and leaves.",
      settings: ["WAIT_ACTION=1", "PRIORITY_SWAP=1"],
      kit: "Start-Kit-wait-priority-v2",
      file: "downloads/algorithms/wait_and_swap_v2_planner.py",
      conclusion: "Best Python row: +16.0% at 10 robots and +8.3% at 100, no clear change at 50. 1 of 10 runs at 100 robots ends stuck.",
      results: {
        s2:  runs("wait-and-swap-v2", "s2", [24, 24, 24, 24, 24, 24, 24, 24, 24, 24]),
        s10: runs("wait-and-swap-v2", "s10", [84, 83, 83, 84, 83, 83, 84, 83, 83, 83]),
        s50: runs("wait-and-swap-v2", "s50", [396, 401, 398, 401, 397, 399, 395, 401, 391, 400]),
        s100:runs("wait-and-swap-v2", "s100", [763, 778, 770, 769, 761, 764, 781, 763, 770, 773])
      }
    },
    {
      id: "cpp-default",
      impl: "cpp",
      baseline: true,
      name: "Default",
      tagline: "The default planner rewritten in C++.",
      description: "The Python Default rewritten in C++ with the same rules and the same search. In 24 tests (6 settings on 4 maps) it chose exactly the same actions as the Python version when ties between equal paths were broken the same way. Here ties are broken with a seeded random number, one seed per run.",
      settings: ["WAIT_ACTION=0", "PRIORITY_SWAP=0", "TIE_SEED=1 to 10"],
      kit: "Start-Kit-cpp",
      file: "downloads/algorithms/cpp_planner.zip",
      conclusion: "Baseline for the C++ rows. Same planner as the Python Default, but ties are broken differently, so its numbers differ (78.5 against 71.8 at 10 robots).",
      results: {
        s2:   runs("cpp-default", "s2", [23, 23, 23, 23, 23, 23, 23, 23, 23, 23]),
        s10:  runs("cpp-default", "s10", [73, 83, 83, 76, 83, 83, 70, 76, 79, 79]),
        s50:  runs("cpp-default", "s50", [362, 391, 393, 385, 393, 381, 387, 390, 394, 332]),
        s100: runs("cpp-default", "s100", [719, 679, 684, 721, 705, 640, 721, 737, 685, 687]),
        s10L:  [495, 583, 597, 589, 587, 572, 553, 570, 541, 552],
        s100L: [4846, 4881, 4730, 4732, 4861, 4703, 4476, 4881, 4724, 4739],
        wl1000:    runsSample("cpp-default", "wl1000", [907, 918, 905]),
        sl1200:    runsSample("cpp-default", "sl1200", [1133, 1128, 1132]),
        paris1000: runsSample("cpp-default", "paris1000", [1145, 1144, 1144])
      }
    },
    {
      id: "cpp-wait-and-swap-v2",
      impl: "cpp",
      tag: "v2",
      name: "Wait action + priority swap",
      tagline: "The best version so far, in C++.",
      description: "The Python v2 planner (wait move in the A*, priority swap, and the swap also tried when the search finds no path) rewritten in C++. It also raises the limit on A* expansions from 100000 to 1000000, because long paths on the big maps hit the old limit.",
      settings: ["WAIT_ACTION=1", "PRIORITY_SWAP=1", "EMPTY_PATH_SWAP=1", "TIE_SEED=1 to 10"],
      kit: "Start-Kit-cpp",
      file: "downloads/algorithms/cpp_planner.zip",
      conclusion: "+4% to +10% on warehouse_small and no run ends stuck at 300 steps; +12.8% at 100 robots over 2000 steps. Only about +1% on the big maps (3 runs), where Default ends stuck in most runs. Planning takes about twice as long as Default, and the first step 40 to 60 s on the big maps.",
      results: {
        s2:   runs("cpp-wait-and-swap-v2", "s2", [24, 24, 24, 24, 24, 24, 24, 24, 24, 24]),
        s10:  runs("cpp-wait-and-swap-v2", "s10", [84, 84, 84, 84, 84, 83, 84, 83, 84, 83]),
        s50:  runs("cpp-wait-and-swap-v2", "s50", [402, 395, 403, 392, 390, 402, 400, 400, 400, 400]),
        s100: runs("cpp-wait-and-swap-v2", "s100", [777, 767, 771, 775, 766, 765, 771, 750, 756, 761]),
        s10L:  [537, 597, 596, 598, 598, 598, 597, 596, 585, 591],
        s100L: [5403, 5314, 5365, 5385, 5368, 5371, 5378, 5340, 5358, 5381],
        wl1000: runsSample("cpp-wait-and-swap-v2", "wl1000", [919, 925, 924]),
        sl1200: runsSample("cpp-wait-and-swap-v2", "sl1200", [1141, 1141, 1139])
      }
    },
    {
      id: "cpp-pibt",
      impl: "cpp",
      name: "PIBT",
      tagline: "Priority inheritance with backtracking (Team Hipparcos).",
      description: "Every robot moves one cell at a time and a robot with higher priority makes the robots in its way move first (priority inheritance, with backtracking). There are no paths and no reservations, and ties between cells are broken by distance to the goal, then by turning cost. The code is the PIBT of Team Hipparcos from the 2024 competition archive, unchanged. On warehouse maps it uses Manhattan distance to the goal and never chooses to stay in place.",
      settings: ["none"],
      kit: "Start-Kit-pibt-cpp",
      file: "downloads/algorithms/pibt_cpp.zip",
      conclusion: "One run so far: far below Default (170 against 380.8 at 50 robots, 298 against 697.8 at 100). Robots rarely wait but wander, probably because Manhattan distance ignores the shelves. Cheapest to plan: 0.1 s per run.",
      results: {
        s50:  runs("cpp-pibt", "s50", [170]),
        s100: runs("cpp-pibt", "s100", [298])
      }
    },
    {
      id: "cpp-sipp",
      impl: "cpp",
      name: "SIPP with waypoints",
      tagline: "Safe interval planning toward waypoints of a thinned map.",
      description: "At start-up the map is thinned into a skeleton of corridors and crossroads, and the cells left are waypoints. For each goal a cheap A* on the skeleton picks a waypoint about every 24 cells. The real search is a safe interval path planner (SIPP): it plans only to the next waypoint, within 8 cells of it, and may wait before a move until the next cell is free. Robots are planned in index order, paths are reserved, and a robot about to collide drops its path and turns in place, as in the default planner. No priority swap.",
      settings: ["none"],
      kit: "Start-Kit-sipp-cpp",
      file: "downloads/algorithms/sipp_cpp.zip",
      conclusion: "One run per map. Much faster to plan: 10 s against 47 s for Default on warehouse_large, 1.1 min against 41 min on Paris, brc202d finishes, and no step takes more than 3 s (ours up to 1 min). The price is 6% to 14% fewer tasks than Default on the big maps.",
      results: {
        s50:  runs("cpp-sipp", "s50", [349]),
        s100: runs("cpp-sipp", "s100", [629]),
        wl1000:    runs("cpp-sipp", "wl1000", [797]),
        sl1200:    runs("cpp-sipp", "sl1200", [1064]),
        paris1000: runs("cpp-sipp", "paris1000", [980]),
        brc1000:   runs("cpp-sipp", "brc1000", [370])
      }
    }
  ]
};

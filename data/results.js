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
    updated: "2026-10-14",
    setup: "Kit: League of Robot Runners start-kit v1.1.5 (2023), Python planner, built in WSL Ubuntu. " +
           "Each setting was run 10 times on the same machine. Runs of the same setting can differ, because the planner breaks ties by memory address, so even a small edit to the file can change the results. " +
           "A robot counts as stuck when it makes no forward move in the last 60 steps of a run.",
    repo: "https://github.com/KamelHtira/MAPF_RESEARCH_RESULTS"
  },

  scenarios: [
    {
      id: "s2",
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
      name: "100 robots",
      map: "warehouse_small (33 x 57)",
      robots: 100,
      steps: 300,
      description: "The kit's 100-robot example.",
      files: ["warehouse_small_100.json", "maps/warehouse_small.map", "agents/warehouse_small_100.agents", "tasks/warehouse_small.tasks", "README.txt"],
      download: "downloads/scenarios/s100.zip"
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
      conclusion: "Reference point for everything below. In the 10-robot scenario 9 of 10 runs end with robots stuck at a junction, and at 100 robots 5 of 10 do. The 10-robot stall is the same every time: at step 113 robot 8 reaches its goal in the one-wide connector at column 43 and gets a new goal behind it, while robot 4 is walking through that same square. Robot 8's search finds no path, and because a robot without a path reserves nothing, robot 4 keeps planning straight through it. The safety net only makes robot 4 turn in place, so it turns away, replans the same path and turns back, every 2 steps, and nobody gives way.",
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
      conclusion: "Same 2-robot result as Default (23). Clearly better at 10 robots (76.3 tasks on average against 71.8) and the only algorithm whose runs almost never end with stuck robots (1 of 10 runs at 10 robots, 0 of 10 at 100 robots). No significant change in tasks at 50 and 100 robots. The planner runs slower than Default, roughly 1.5 to 2 times in my timings.",
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
      conclusion: "Biggest gain at 10 robots: 82.3 tasks on average against 71.8 for Default, and no run ends with stuck robots. Slightly lower than Default at 50 robots, and no difference at 100 robots, where 7 of 10 runs end with stuck robots. No effect on the 2-robot case. The wait move does not remove the step-113 deadlock itself: in 4 traced runs robots 4 and 8 stayed frozen for 133 and 187 steps in two of them and only paused 2 steps in the other two. Robot 8 still gets an empty path, because it needs two turns to leave and the reservation of robot 4 reaches its square one step earlier, so waiting cannot help it.",
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
      conclusion: "Fixes the 2-robot case: 24 tasks in all 10 runs against 23 for Default. At 10 robots the results split into two groups (70 to 83 tasks) and 5 of 10 runs end with stuck robots. Lower than Default at 50 robots and no different at 100 robots. It never fires at the step-113 deadlock, because it only starts when a path exists and is long; robot 8 has no path at all (fixed in the v2 row).",
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
      conclusion: "Same 2-robot gain as the priority swap alone (24 tasks). Worse than Default at 100 robots (596 to 720 tasks), and 7 of 10 runs end with stuck robots at both 10 and 100 robots. Not recommended as it is. The reason is the same gap as in the priority swap: it is not tried when the search finds no path at all, so the step-113 deadlock in the 10-robot scenario stays (see the v2 row).",
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
      conclusion: "The empty-path case fixes the 10-robot deadlock: robots 4 and 8 never stay frozen after step 113 in any of the 10 runs (Priority swap v1: 5 of 10 runs stay frozen for 187 steps), the results are 83 or 84 tasks in every run against 71.8 for Default, and no run ends with stuck robots. At 50 robots 399.3 tasks, no clear difference from Default (p = 0.09). At 100 robots 739.3 against 710.0 for Default, borderline (p = 0.05): seven runs reach 757 to 772 tasks but three identical runs stop at 684 with robots 15, 57 and 97 stuck, so 3 of 10 runs still end with stuck robots. Same 2-robot gain as the other swap rows (24).",
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
      conclusion: "Best row so far. The 10-robot deadlock is gone: robots 4 and 8 are never frozen after step 113 in any of the 10 runs (v1 of this method: 7 of 10 runs frozen for 187 steps), 83 or 84 tasks in every run against 71.8 for Default, and no run ends with stuck robots. At 50 robots 397.9 tasks, no significant difference from Default (395.1), but better than Wait action alone (384.8). At 100 robots 769.2 tasks against 710.0 for Default (+8.3%), better than every other row, with a narrow spread (761 to 781) and only 1 of 10 runs ending with stuck robots (v1 of this method: 661.6 tasks, 7 of 10 stuck). Same 2-robot gain as the other swap rows (24).",
      results: {
        s2:  runs("wait-and-swap-v2", "s2", [24, 24, 24, 24, 24, 24, 24, 24, 24, 24]),
        s10: runs("wait-and-swap-v2", "s10", [84, 83, 83, 84, 83, 83, 84, 83, 83, 83]),
        s50: runs("wait-and-swap-v2", "s50", [396, 401, 398, 401, 397, 399, 395, 401, 391, 400]),
        s100:runs("wait-and-swap-v2", "s100", [763, 778, 770, 769, 761, 764, 781, 763, 770, 773])
      }
    }
  ]
};

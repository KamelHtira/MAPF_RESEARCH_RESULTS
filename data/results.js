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
    subtitle: "League of Robot Runners: improving the default prioritized planner on the warehouse_small map",
    updated: "2026-10-13",
    noise_pct: 2,
    setup: "Kit: League of Robot Runners start-kit v1.1.5 (2023), Python planner, built in WSL Ubuntu. " +
           "Each setting was run several times because the planner breaks ties by memory address, so runs are not identical.",
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
      files: ["instance.json", "map.map", "agents.agents", "tasks.tasks"],
      download: "downloads/scenarios/s2.zip"
    },
    {
      id: "s10",
      name: "10 robots",
      map: "warehouse_small (33 x 57)",
      robots: 10,
      steps: 300,
      description: "The kit's 10-robot example.",
      files: ["instance.json", "map.map", "agents.agents", "tasks.tasks"],
      download: "downloads/scenarios/s10.zip"
    },
    {
      id: "s50",
      name: "50 robots",
      map: "warehouse_small (33 x 57)",
      robots: 50,
      steps: 300,
      description: "The kit's 50-robot example.",
      files: ["instance.json", "map.map", "agents.agents", "tasks.tasks"],
      download: "downloads/scenarios/s50.zip"
    },
    {
      id: "s100",
      name: "100 robots",
      map: "warehouse_small (33 x 57)",
      robots: 100,
      steps: 300,
      description: "The kit's 100-robot example.",
      files: ["instance.json", "map.map", "agents.agents", "tasks.tasks"],
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
      conclusion: "Reference point for everything below. Has a permanent 3-robot deadlock in the 10-robot scenario.",
      results: {
        s2:   runs("default", "s2",   [22, 22, 22]),
        s10:  runs("default", "s10",  [70, 72, 71]),
        s50:  runs("default", "s50",  [380, 384, 379]),
        s100: runs("default", "s100", [681, 686, 684])
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
      conclusion: "Reduces large detours on the 2-robot scenario but does not change the task count there. The planner runs about twice as slowly.",
      results: {
        s2:   runs("delay-fix", "s2",   [22, 22, 22]),
        s10:  runs("delay-fix", "s10",  [76, 78, 77]),
        s50:  runs("delay-fix", "s50",  [392, 396, 394]),
        s100: runs("delay-fix", "s100", [694, 699, 695])
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
      conclusion: "Removes the 10-robot deadlock, so many more tasks get finished there, and it also helps at 100 robots. No effect on the 2-robot head-on, which needs the priority swap.",
      results: {
        s2:   runs("wait-action", "s2",   [22, 22, 22]),
        s10:  runs("wait-action", "s10",  [78, 80, 79]),
        s50:  runs("wait-action", "s50",  [381, 383, 382]),
        s100: runs("wait-action", "s100", [708, 714, 711])
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
      conclusion: "Fixes the 2-robot head-on exactly (time lost 66 to 6 ticks). Slower on large problems and does not remove the 10-robot deadlock.",
      results: {
        s2:   runs("priority-swap", "s2",   [24, 24, 24]),
        s10:  runs("priority-swap", "s10",  [69, 70, 71]),
        s50:  runs("priority-swap", "s50",  [389, 392, 391]),
        s100: runs("priority-swap", "s100", [705, 709, 707])
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
      conclusion: "Worse than either change alone at 50 and 100 robots. Not recommended as it is.",
      results: {
        s2:   runs("wait-and-swap", "s2",   [24, 24, 24]),
        s10:  runs("wait-and-swap", "s10",  [72, 74, 73]),
        s50:  runs("wait-and-swap", "s50",  [377, 381, 378]),
        s100: runs("wait-and-swap", "s100", [670, 676, 672])
      }
    }
  ]
};

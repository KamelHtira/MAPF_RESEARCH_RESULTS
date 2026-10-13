# Method: Wait action + priority swap (WAIT_ACTION=1, PRIORITY_SWAP=1)
# Use it by saving this file as python/pyMAPFPlanner.py in a League of Robot Runners start-kit v1.1.5 built with Python enabled.
import os
import sys
import MAPF

from typing import Dict, List, Tuple,Set
from queue import PriorityQueue
import numpy as np

# 0=Action.FW, 1=Action.CR, 2=Action.CCR, 3=Action.W

# the harness ends with _exit(0), which drops buffered Python output, so flush at every newline
sys.stdout.reconfigure(line_buffering=True)

# Two independent changes to the professor's planner (both 0 = his original behaviour):
#  WAIT_ACTION=1    the A* may also WAIT in place (keeps its facing). He only had forward / turn left / turn right, so a
#                   robot could only wait by turning (and needs two turns to face the same way again).
#  PRIORITY_SWAP=1  when a robot's planned path is DETOUR_LIMIT or more ticks longer than its ideal path (same trip with
#                   every other robot removed), find the robots whose reserved paths cross its ideal path, let the stuck
#                   robot plan FIRST and the blocker replan around it. Keep it only if the two robots together save
#                   at least DETOUR_MARGIN ticks.
WAIT_ACTION = int(os.environ.get("WAIT_ACTION", "1"))
PRIORITY_SWAP = int(os.environ.get("PRIORITY_SWAP", "1"))
DETOUR_LIMIT = int(os.environ.get("DETOUR_LIMIT", "6"))
DETOUR_MARGIN = int(os.environ.get("DETOUR_MARGIN", "3"))
SWAP_MAX_CANDIDATES = 3     # how many blocking robots are tried
ASTAR_EXPANSION_CAP = 100000  # safety stop for one A* search when waiting is allowed
SWAP_DEBUG = os.environ.get("SWAP_DEBUG")

class pyMAPFPlanner:

    reservation = set()  # loc1, loc2, t
    agent_map = dict()
    conflicts = set()

    def __init__(self, pyenv=None) -> None:
        if pyenv is not None:
            self.env = pyenv.env

        self.time = -1
        self.paths = []
        self.agent_map = dict()
        self.vertex_conflict = []
        self.edge_conflict = []
        self.ideal_cache = {}  # (start cell, facing, goal cell) -> length of the ideal path
        self.astar_capped = 0  # how many searches hit ASTAR_EXPANSION_CAP

        print("pyMAPFPlanner created!  python debug")

    def initialize(self, preprocess_time_limit: int):
        """_summary_

        Args:
            preprocess_time_limit (_type_): _description_
        """
        # pass

        for a in range(self.env.num_of_agents):
            self.paths.append([])

        # testlib.test_torch()
        print("planner initialize done... python debug")
        return True
        # raise NotImplementedError()

    def init_agent_map(self):
        # print("clearing agent map ...")

        self.agent_map.clear()
        for i in range(self.env.num_of_agents):
            self.agent_map.update({self.env.curr_states[i].location:i})
        return

    # ---------------- priority swap ----------------

    def _res_tuples(self, path):
        """The reservation entries a path holds (same convention as the planner: step t of a path is at time self.time + t)."""
        out = []
        last_loc = -1
        for t, p in enumerate(path, 1):
            out.append((p[0], -1, self.time + t))
            if last_loc != -1:
                out.append((last_loc, p[0], self.time + t))
            last_loc = p[0]
        return out

    def _ideal_len(self, start, ori, goal) -> int:
        """Length of the best path from (start, ori) to goal with every other robot removed."""
        key = (start, ori, goal)
        if key not in self.ideal_cache:
            self.ideal_cache[key] = len(self.space_time_plan(start, ori, goal, set()))
        return self.ideal_cache[key]

    def _first_action(self, i):
        """The action robot i takes this tick if it follows its stored path (same rule as the planning loop)."""
        path = self.paths[i]
        cur = self.env.curr_states[i]
        if path:
            if path[0][0] != cur.location:
                return MAPF.Action.FW
            if path[0][1] != cur.orientation:
                incr = path[0][1] - cur.orientation
                if incr == 1 or incr == -3:
                    return MAPF.Action.CR
                if incr == -1 or incr == 3:
                    return MAPF.Action.CCR
        return MAPF.Action.W

    def _blockers(self, i, ideal_path):
        """Robots whose reserved paths cross robot i's ideal path (same square at the same time, or a head-on swap),
        the one that crosses it earliest first."""
        mine = self._res_tuples(ideal_path)
        found = []
        for j in range(self.env.num_of_agents):
            if j == i or not self.paths[j]:
                continue
            theirs = set(self._res_tuples(self.paths[j]))
            for e in mine:
                if e in theirs or (e[1] != -1 and (e[1], e[0], e[2]) in theirs):
                    found.append((e[2], j))
                    break
        found.sort()
        return [j for _, j in found[:SWAP_MAX_CANDIDATES]]

    def _try_priority_swap(self, i, goal, path, ideal, actions):
        """Robot i's planned path is much longer than its ideal one. For each robot j that blocks the ideal route: let i plan
        first without j's reservations, then let j replan around i's new path (j may wait, if WAIT_ACTION is on). Keep the best
        option if i and j together need at least DETOUR_MARGIN fewer ticks. Returns i's path."""
        cur = self.env.curr_states[i]
        ideal_path = self.space_time_plan(cur.location, cur.orientation, goal, set())
        best = None
        for j in self._blockers(i, ideal_path):
            if not self.env.goal_locations[j]:
                continue
            sj = self.env.curr_states[j]
            goal_j = self.env.goal_locations[j][0][0]
            res = set(self.reservation)
            res.difference_update(self._res_tuples(self.paths[j]))
            alt_i = self.space_time_plan(cur.location, cur.orientation, goal, res)
            if not alt_i:
                continue
            res.update(self._res_tuples(alt_i))
            alt_j = self.space_time_plan(sj.location, sj.orientation, goal_j, res)
            if not alt_j:
                continue
            res.update(self._res_tuples(alt_j))
            gain = (len(path) + len(self.paths[j])) - (len(alt_i) + len(alt_j))
            if best is None or gain > best[0]:
                best = (gain, j, alt_i, alt_j, res)
        if best is None or best[0] < DETOUR_MARGIN:
            return path
        gain, j, alt_i, alt_j, res = best
        print("[swap] t=%d robot %d at %s: planned %d ticks vs ideal %d -> robot %d replans (%d -> %d ticks), robot %d now needs %d"
              % (self.time, i, divmod(cur.location, self.env.cols), len(path), ideal, j, len(self.paths[j]), len(alt_j), i, len(alt_i)))
        self.reservation = res
        self.paths[j] = alt_j
        actions[j] = self._first_action(j)
        return alt_i

    def plan(self, time_limit):
        """_summary_

        Return:
            actions ([Action]): the next actions

        Args:
            time_limit (_type_): _description_
        """

        self.time = self.time + 1

        # example of only using single-agent search
        return self.sample_priority_planner(time_limit)
        # print("python binding debug")
        # print("env.rows=",self.env.rows,"env.cols=",self.env.cols,"env.map=",self.env.map)
        # raise NotImplementedError("YOU NEED TO IMPLEMENT THE PYMAPFPLANNER!")

    def naive_a_star(self,time_limit):
        print("I am planning")
        actions = [MAPF.Action.W for i in range(len(self.env.curr_states))]
        for i in range(0, self.env.num_of_agents):
            print("python start plan for agent ", i, end=" ")
            path = []
            if len(self.env.goal_locations[i]) == 0:
                print(i, " does not have any goal left", end=" ")
                path.append(
                    (self.env.curr_states[i].location, self.env.curr_states[i].orientation))
            else:
                print(" with start and goal: ", end=" ")
                path = self.single_agent_plan(
                    self.env.curr_states[i].location, self.env.curr_states[i].orientation, self.env.goal_locations[i][0][0])

            print("current location:", path[0][0],
                  "current direction: ", path[0][1])
            if path[0][0] != self.env.curr_states[i].location:
                actions[i] = MAPF.Action.FW
            elif path[0][1] != self.env.curr_states[i].orientation:
                incr = path[0][1]-self.env.curr_states[i].orientation
                if incr == 1 or incr == -3:
                    actions[i] = MAPF.Action.CR
                elif incr == -1 or incr == 3:
                    actions[i] = MAPF.Action.CCR
        # print(actions)
        actions = [int(a) for a in actions]
        # print(actions)
        return np.array(actions, dtype=int)

    def single_agent_plan(self, start: int, start_direct: int, end: int):
        print(start, start_direct, end)
        path = []
        # AStarNode (u,dir,t,f)
        open_list = PriorityQueue()
        s = (start, start_direct, 0, self.getManhattanDistance(start, end))
        open_list.put([0, s])
        all_nodes = dict()
        close_list = set()
        parent = {(start, start_direct): None}
        all_nodes[start*4+start_direct] = s
        while not open_list.empty():
            curr = (open_list.get())[1]
            close_list.add(curr[0]*4+curr[1])
            if curr[0] == end:
                curr = (curr[0], curr[1])
                while curr != None:
                    path.append(curr)
                    curr = parent[curr]
                path.pop()
                path.reverse()

                break
            neighbors = self.getNeighbors(curr[0], curr[1])
            # print("neighbors=",neighbors)
            for neighbor in neighbors:
                if (neighbor[0]*4+neighbor[1]) in close_list:
                    continue
                next_node = (neighbor[0], neighbor[1], curr[2]+1,
                             self.getManhattanDistance(neighbor[0], end))
                parent[(next_node[0], next_node[1])] = (curr[0], curr[1])
                open_list.put([next_node[3]+next_node[2], next_node])
        print(path)
        return path

    def getManhattanDistance(self, loc1: int, loc2: int) -> int:
        loc1_x = loc1//self.env.cols
        loc1_y = loc1 % self.env.cols
        loc2_x = loc2//self.env.cols
        loc2_y = loc2 % self.env.cols
        return abs(loc1_x-loc2_x)+abs(loc1_y-loc2_y)

    def validateMove(self, loc: int, loc2: int) -> bool:
        loc_x = loc//self.env.cols
        loc_y = loc % self.env.cols
        if(loc_x < 0 or loc_x >= self.env.rows or loc_y < 0 or loc_y >= self.env.cols or self.env.map[loc] == 1):
            return False
        loc2_x = loc2//self.env.cols
        loc2_y = loc2 % self.env.cols
        if(abs(loc_x-loc2_x)+abs(loc_y-loc2_y) > 1):
            return False
        return True

    def getNeighbors(self, location: int, direction: int):
        neighbors = []
        # forward
        candidates = [location+1, location+self.env.cols,
                      location-1, location-self.env.cols]
        forward = candidates[direction]
        new_direction = direction
        if (forward >= 0 and forward < len(self.env.map) and self.validateMove(forward, location)):
            neighbors.append((forward, new_direction))
        # turn left
        new_direction = direction-1
        if (new_direction == -1):
            new_direction = 3
        neighbors.append((location, new_direction))
        # turn right
        new_direction = direction+1
        if (new_direction == 4):
            new_direction = 0
        neighbors.append((location, new_direction))
        # print("debug!!!!!!!", neighbors)
        return neighbors

    def space_time_plan(self,start: int, start_direct: int, end: int, reservation: Set[Tuple[int, int, int]]) -> List[Tuple[int, int]]:
        # print(start, start_direct, end)
        path = []
        open_list = PriorityQueue()
        all_nodes = {}  # loc+dict, t
        parent={}
        s = (start, start_direct, 0, self.getManhattanDistance(start, end))
        open_list.put((s[3], id(s), s))
        # all_nodes[(start * 4 + start_direct, 0)] = s
        parent[(start * 4 + start_direct, 0)]=None

        while not open_list.empty():
            n=open_list.get()
            # if start==761: print("n=",n)
            _, _, curr = n

            curr_location, curr_direction, curr_g, _ = curr

            if (curr_location*4+curr_direction,curr_g) in all_nodes:
                continue
            all_nodes[(curr_location*4+curr_direction,curr_g)]=curr
            if WAIT_ACTION and len(all_nodes) > ASTAR_EXPANSION_CAP:
                self.astar_capped += 1
                print("[wait] A* stopped after %d expansions (start %s goal %s)" % (ASTAR_EXPANSION_CAP, divmod(start, self.env.cols), divmod(end, self.env.cols)))
                return []
            if curr_location == end:
                while True:
                    path.append((curr[0], curr[1]))
                    curr=parent[(curr[0]*4+curr[1],curr[2])]
                    if curr is None:
                        break
                    # curr = curr[5]
                path.pop()
                path.reverse()
                break
            
            neighbors = self.getNeighbors(curr_location, curr_direction)
            if WAIT_ACTION:
                neighbors.append((curr_location, curr_direction))  # wait in place, same facing

            for neighbor in neighbors:
                neighbor_location, neighbor_direction = neighbor

                if (neighbor_location, -1, self.time + curr[2] + 1) in reservation:
                    continue

                if (neighbor_location, curr_location, self.time + curr[2] + 1) in reservation:
                    continue

                neighbor_key = (neighbor_location * 4 +
                                neighbor_direction, curr[2] + 1)

                if neighbor_key in all_nodes:
                    old = all_nodes[neighbor_key]
                    if curr_g + 1 < old[2]:
                        old = (old[0], old[1], curr_g + 1, old[3], old[4])
                else:
                    next_node = (neighbor_location, neighbor_direction, curr_g + 1,
                                self.getManhattanDistance(neighbor_location, end))
        
                    open_list.put(
                        (next_node[3] + next_node[2], id(next_node), next_node))
                
                    parent[(neighbor_location * 4 +
                            neighbor_direction, next_node[2])]=curr

        # for v in path:
        #     print(f"({v[0]},{v[1]}), ", end="")
        # print()
        return path

    def sample_priority_planner(self,time_limit:int):
        actions = [MAPF.Action.W] * len(self.env.curr_states)
        # self.reservation = set()  # loc1, loc2, t

        # ### initial reservation makes no sense, because the agent blocks itself
        # if self.time == 0:
        #     for i in range(self.env.num_of_agents):
        #         # print("starting initital reservation for agent", i)
        #         self.reservation.add((self.env.curr_states[i].location, -1, self.time+1))
        #         self.reservation.add((self.env.curr_states[i].location, -1, self.time+2))
        # ### initial reservation makes no sense, because the agent blocks itself

        # for i in range(self.env.num_of_agents):
        #     # print("re-reserving the path for ", i)
        #     path = self.paths[i].copy()
        #     # if path == []:
        #     #     self.reservation.add((self.env.curr_states[i].location, -1, self.time+1))
        #     #     # self.reservation.add((self.env.curr_states[i].location, -1, self.time+2))
        #     last_loc = -1
        #     t = 1
        #     for p in path:
        #         self.reservation.add((p[0], -1, self.time + t))
        #         if last_loc != -1:
        #             self.reservation.add((last_loc, p[0], self.time + t))
        #         last_loc = p[0]
        #         t += 1

        self.init_agent_map()

        if WAIT_ACTION or PRIORITY_SWAP:  # entries for past timesteps are never looked at again
            self.reservation = {r for r in self.reservation if r[2] > self.time}

        for i in range(self.env.num_of_agents):
            # print("start plan for agent", i)
            path = []
            if not self.env.goal_locations[i]:
                print(", which does not have any goal left.")
                path.append((self.env.curr_states[i].location, self.env.curr_states[i].orientation))
                self.reservation.add((self.env.curr_states[i].location, -1, self.time + 1))

        for i in range(self.env.num_of_agents):
            if self.paths[i] == []:
                # print("start plan for agent", i)
                path = []
                if self.env.goal_locations[i]:
                    # print("with start and goal:")
                    path = self.space_time_plan(
                        self.env.curr_states[i].location,
                        self.env.curr_states[i].orientation,
                        self.env.goal_locations[i][0][0],
                        self.reservation
                    )
                    if PRIORITY_SWAP and path:
                        cur = self.env.curr_states[i]
                        goal = self.env.goal_locations[i][0][0]
                        ideal = self._ideal_len(cur.location, cur.orientation, goal)
                        if SWAP_DEBUG:
                            print("[detour] t=%d robot %d planned=%d ideal=%d excess=%d" % (self.time, i, len(path), ideal, len(path) - ideal))
                        if len(path) - ideal >= DETOUR_LIMIT:
                            path = self._try_priority_swap(i, goal, path, ideal, actions)
                self.paths[i] = path.copy()
                # reserve the newly created path
                last_loc = -1
                t = 1
                # info = "agent " + str(i) + " path: "
                for p in path:
                    self.reservation.add((p[0], -1, self.time + t))
                    # if i == 13 or i ==5: info = info + str(self.time+t) + ":" + str(divmod(p[0],self.env.cols)) + " "
                    if last_loc != -1:
                        self.reservation.add((last_loc, p[0], self.time + t))
                    last_loc = p[0]
                    t += 1
                # if i == 13 or i ==5: print(info)
            
            path = self.paths[i]
            if path:
                # print("current location:", path[0][0], "current direction:", path[0][1])
                if path[0][0] != self.env.curr_states[i].location:
                    actions[i] = MAPF.Action.FW
                elif path[0][1] != self.env.curr_states[i].orientation:
                    incr = path[0][1] - self.env.curr_states[i].orientation
                    if incr == 1 or incr == -3:
                        actions[i] = MAPF.Action.CR
                    elif incr == -1 or incr == 3:
                        actions[i] = MAPF.Action.CCR
                # if i == 13: print("agent 13 action: ", actions[i])

        self.vertex_conflict = [0] * len(self.env.map)
        self.edge_conflict = [-1] * len(self.env.map)
        found = self.find_conflicts()
        while found:
            for i in self.conflicts:
                path = self.paths[i].copy()
                # remove reservations
                last_loc = -1
                t = 1
                for p in path:
                    self.reservation.discard((p[0], -1, self.time + t))
                    if last_loc != -1:
                        self.reservation.discard((last_loc, p[0], self.time + t))
                    last_loc = p[0]
                    t += 1
                path = []
                next_orientation = self.env.curr_states[i].orientation + 1
                if next_orientation > 3: next_orientation = 0
                path.append((self.env.curr_states[i].location, next_orientation))
                self.paths[i] = []
                self.paths[i] = path.copy()
                self.reservation.add((self.env.curr_states[i].location, -1, self.time + 1))
                actions[i] = MAPF.Action.CR
                # if i == 13: print("agent 13 in conflict")
            found = self.find_conflicts()

        for i in range(self.env.num_of_agents):
            if len(self.paths[i]) > 0: 
                self.paths[i].pop(0)

        return actions

    def find_conflicts(self) -> bool:
        self.conflicts.clear()
        found = False
        surrounding_agents = list()
        # for i in range(self.num_of_agents) if self.time % 2 == 1 else reversed(range(self.num_of_agents)):
        for i in range(self.env.num_of_agents):
            if (self.paths[i] != [] and self.paths[i][0][0] != self.env.curr_states[i].location): # this robot moves
                location = self.env.curr_states[i].location
                candidates = [location+1, location+self.env.cols,
                              location-1, location-self.env.cols]
                forward = candidates[self.env.curr_states[i].orientation]
                surrounding_agents.clear()
                if self.agent_map.get(forward) != None: surrounding_agents.append(self.agent_map.get(forward)) # agent in the target cell
                if self.agent_map.get(forward+1) != None: surrounding_agents.append(self.agent_map.get(forward+1)) # agents in the surrounding cells
                if self.agent_map.get(forward+self.env.cols) != None: surrounding_agents.append(self.agent_map.get(forward+self.env.cols)) # agents in the surrounding cells
                if self.agent_map.get(forward-1) != None: surrounding_agents.append(self.agent_map.get(forward-1)) # agents in the surrounding cells
                if self.agent_map.get(forward-self.env.cols) != None: surrounding_agents.append(self.agent_map.get(forward-self.env.cols)) # agents in the surrounding cells
                for agent_j in surrounding_agents:
                    if agent_j == i:
                        continue
                    if ((self.env.curr_states[agent_j].location == forward and  # there is a robot and
                        ((self.paths[agent_j] == [] or self.paths[agent_j][0][0] == forward) or # that robot does not move
                         (self.paths[agent_j] != [] and self.paths[agent_j][0][0] == location)) ) or # or moves in the opposite direction
                        (self.paths[agent_j] != [] and self.paths[agent_j][0][0] == forward)  # or that robot moves to the same place
                    ):
                        if (self.paths[agent_j] != [] and self.paths[agent_j][0][0] == location):   # it is an edge conflict
                            self.edge_conflict[location] = forward
                        else:              # if not edge conflict then it is a vertex conflict
                            self.vertex_conflict[forward] = self.vertex_conflict[forward] +1
                        self.conflicts.add(i)
                        found = True
            # if found: break
        return found



if __name__ == "__main__":
    test_planner = pyMAPFPlanner()
    test_planner.initialize(100)

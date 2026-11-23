// PIBT with cover points, in one file. Replaces src/MAPFPlanner.cpp of the start-kit v1.1.5 (MAPFPlanner.h stays as in the kit).
//
// PIBT moves every robot one step toward its goal, in priority order; a robot that is in the way is asked to move first.
// Choosing "toward the goal" needs the distance of a cell to the goal. Instead of a table of the whole map for every goal,
// this file computes it from cover points:
//
//   once per map   1. Cover points: one pass over the cells. A free cell that no earlier cover point covers becomes a cover
//                     point and covers every cell within C steps of it (walls count).
//                  2. Lists: from every cover point a search of R steps. Every cell keeps the cover points within R steps and
//                     the exact steps to each. Cover points within R steps of each other are joined into a graph.
//   per goal       3. Window: a search of R steps from the goal gives the exact distance of the cells around it.
//                  4. One Dijkstra over the cover-point graph, started from the cover points inside the window, gives every
//                     cover point its distance to the goal.
//   score of cell  inside the window its exact distance; outside, the smallest (steps to a cover point + that cover point's
//                  distance to the goal). A robot steps to the neighbour with the lowest score.
//
// Settings (environment variables, read once): COVER_R (default 24) and COVER_C (default 11, at most (R - 1) / 2).

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <queue>
#include <vector>
#include "MAPFPlanner.h"

namespace {

using Clock = std::chrono::steady_clock;

constexpr int kDefaultR = 24;
constexpr int kDefaultC = 11;
constexpr int kDeadlineMs = 900;          // planning stops here; robots that were not planned wait
constexpr int kMarginMs = 20;             // kept free below the time limit of the harness
constexpr uint16_t kNone = 0xFFFF;        // "no value" in the distance tables
constexpr int kUnreachable = 60000;       // score of a cell that cannot reach the goal

int envInt(const char* name, int fallback) {
    const char* s = std::getenv(name);
    return s && *s ? std::atoi(s) : fallback;
}

// direction of a move between two neighbouring cells: 0 east, 1 south, 2 west, 3 north
int directionOf(int diff, int cols) {
    if (diff == 1) { return 0; }
    if (diff == cols) { return 1; }
    if (diff == -1) { return 2; }
    assert(diff == -cols);
    return 3;
}

class CoverPoints;

// What one goal holds: the exact distances inside a window of R steps around it, and for every cover point its distance to it.
class GoalTable {
public:
    int dist(int loc) const;                      // score of a cell: how far it is from the goal (smaller is closer)

private:
    friend class CoverPoints;
    const CoverPoints* index = nullptr;
    int cols = 0, row0 = 0, col0 = 0, height = 0, width = 0;
    std::vector<uint16_t> window;                 // exact steps to the goal for the box around it, kNone outside the window
    std::vector<uint16_t> coverDist;              // cover point -> goal, kNone if unknown

    int exact(int loc) const;                     // exact steps inside the window, kNone outside
};

// Cover points of a map, built once.
//   C: every free cell is within C steps of a cover point.
//   R: every cell knows its exact steps to each cover point within R steps; cover points within R steps of each other are joined.
// C is capped at (R - 1) / 2, which keeps the cover points connected whenever the map is.
class CoverPoints {
public:
    CoverPoints(const SharedEnvironment* env, int R, int C) : rows(env->rows), cols(env->cols), R(R), C(C) {
        const int n = rows * cols;
        std::vector<char> freeCell(n);
        for (int u = 0; u < n; ++u) { freeCell[u] = env->map[u] != 1; }

        nb.assign((size_t)n * 4, -1);
        for (int u = 0; u < n; ++u) {
            if (!freeCell[u]) { continue; }
            const int c = u % cols;
            const int next[4] = {c + 1 < cols ? u + 1 : -1, u + cols < n ? u + cols : -1, c > 0 ? u - 1 : -1, u - cols >= 0 ? u - cols : -1};
            for (int d = 0; d < 4; ++d) {
                if (next[d] >= 0 && freeCell[next[d]]) { nb[(size_t)u * 4 + d] = next[d]; }
            }
        }

        steps.assign(n, kNone);
        placeCoverPoints(freeCell);
        buildLists();
    }

    // free neighbour of a cell in a direction (0 east, 1 south, 2 west, 3 north), -1 if there is none
    int neighbor(int loc, int dir) const { return nb[(size_t)loc * 4 + dir]; }

    std::shared_ptr<GoalTable> makeTable(int goal);

private:
    friend class GoalTable;
    int rows, cols, R, C;
    std::vector<int> nb;                          // nb[loc * 4 + dir]
    std::vector<int> cell;                        // cover point -> its cell
    std::vector<int> listStart, listCp;           // per cell: the cover points within R steps ...
    std::vector<uint16_t> listSteps;              // ... and the exact steps to each
    std::vector<int> edgeStart, edgeTo;           // per cover point: the cover points within R steps ...
    std::vector<uint16_t> edgeCost;               // ... and the exact steps to each

    // breadth-first search workspace: after search(), steps holds the distances of the cells in reached
    std::vector<uint16_t> steps;
    std::vector<int> reached;

    void search(int source, int limit) {
        reached.clear();
        steps[source] = 0;
        reached.push_back(source);
        for (size_t head = 0; head < reached.size(); ++head) {
            const int u = reached[head];
            if (steps[u] >= limit) { continue; }
            for (int d = 0; d < 4; ++d) {
                const int v = nb[(size_t)u * 4 + d];
                if (v >= 0 && steps[v] == kNone) {
                    steps[v] = steps[u] + 1;
                    reached.push_back(v);
                }
            }
        }
    }

    void clear() {
        for (int u : reached) { steps[u] = kNone; }
    }

    void placeCoverPoints(const std::vector<char>& freeCell) {
        const int n = rows * cols;
        std::vector<char> covered(n, 0);
        for (int u = 0; u < n; ++u) {
            if (!freeCell[u] || covered[u]) { continue; }
            cell.push_back(u);
            search(u, C);
            for (int t : reached) { covered[t] = 1; }
            clear();
        }
    }

    // Two passes over the same searches: pass 0 counts the entries of every list, pass 1 stores them (compact arrays, no per-cell vectors).
    void buildLists() {
        const int n = rows * cols, m = (int)cell.size();
        std::vector<int> coverOf(n, -1);
        for (int i = 0; i < m; ++i) { coverOf[cell[i]] = i; }

        listStart.assign(n + 1, 0);
        edgeStart.assign(m + 1, 0);
        std::vector<int> listAt, edgeAt;
        for (int pass = 0; pass < 2; ++pass) {
            if (pass == 1) {
                for (int u = 0; u < n; ++u) { listStart[u + 1] += listStart[u]; }
                for (int i = 0; i < m; ++i) { edgeStart[i + 1] += edgeStart[i]; }
                listCp.resize(listStart[n]);
                listSteps.resize(listStart[n]);
                edgeTo.resize(edgeStart[m]);
                edgeCost.resize(edgeStart[m]);
                listAt.assign(listStart.begin(), listStart.end() - 1);
                edgeAt.assign(edgeStart.begin(), edgeStart.end() - 1);
            }
            for (int i = 0; i < m; ++i) {
                search(cell[i], R);
                for (int t : reached) {
                    const bool otherCover = coverOf[t] >= 0 && coverOf[t] != i;
                    if (pass == 0) {
                        ++listStart[t + 1];
                        if (otherCover) { ++edgeStart[i + 1]; }
                    } else {
                        listCp[listAt[t]] = i;
                        listSteps[listAt[t]++] = steps[t];
                        if (otherCover) {
                            edgeTo[edgeAt[i]] = coverOf[t];
                            edgeCost[edgeAt[i]++] = steps[t];
                        }
                    }
                }
                clear();
            }
        }
    }
};

std::shared_ptr<GoalTable> CoverPoints::makeTable(int goal) {
    auto table = std::make_shared<GoalTable>();
    table->index = this;
    table->cols = cols;
    const int goalRow = goal / cols, goalCol = goal % cols;
    table->row0 = std::max(0, goalRow - R);
    table->col0 = std::max(0, goalCol - R);
    table->height = std::min(rows - 1, goalRow + R) - table->row0 + 1;
    table->width = std::min(cols - 1, goalCol + R) - table->col0 + 1;
    table->window.assign((size_t)table->height * table->width, kNone);
    table->coverDist.assign(cell.size(), kNone);

    // window: exact distances around the goal
    search(goal, R);
    for (int t : reached) {
        table->window[(size_t)(t / cols - table->row0) * table->width + (t % cols - table->col0)] = steps[t];
    }
    clear();

    // Dijkstra over the cover points, started from those inside the window
    using Entry = std::pair<uint16_t, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    for (int i = 0; i < (int)cell.size(); ++i) {
        const int d = table->exact(cell[i]);
        if (d != kNone) {
            table->coverDist[i] = (uint16_t)d;
            queue.push({(uint16_t)d, i});
        }
    }
    while (!queue.empty()) {
        const auto [d, i] = queue.top();
        queue.pop();
        if (d > table->coverDist[i]) { continue; }
        for (int e = edgeStart[i]; e < edgeStart[i + 1]; ++e) {
            const int j = edgeTo[e], nd = d + edgeCost[e];
            if (nd < table->coverDist[j]) {
                table->coverDist[j] = (uint16_t)nd;
                queue.push({(uint16_t)nd, j});
            }
        }
    }
    return table;
}

int GoalTable::exact(int loc) const {
    const int r = loc / cols - row0, c = loc % cols - col0;
    if (r < 0 || r >= height || c < 0 || c >= width) { return kNone; }
    return window[(size_t)r * width + c];
}

int GoalTable::dist(int loc) const {
    const int d = exact(loc);
    if (d != kNone) { return d; }
    int best = kUnreachable;
    for (int e = index->listStart[loc]; e < index->listStart[loc + 1]; ++e) {
        const uint16_t toGoal = coverDist[index->listCp[e]];
        if (toGoal != kNone) { best = std::min(best, index->listSteps[e] + toGoal); }
    }
    return best;
}

struct Agent {
    int id;
    double p;                             // current priority (higher plans first)
    double baseP;                         // the priority a robot gets with a new goal
    int nextLoc = -1;                     // cell chosen for this step, -1 if none yet
    int goal = -1;
    std::shared_ptr<GoalTable> table;     // distances to the goal, built for every new goal
};

class PIBT {
public:
    explicit PIBT(SharedEnvironment* env) : env(env) {
        const int R = std::max(3, envInt("COVER_R", kDefaultR));
        const int C = std::clamp(envInt("COVER_C", kDefaultC), 1, (R - 1) / 2);
        cover = std::make_unique<CoverPoints>(env, R, C);

        prevReservations.assign(env->map.size(), -1);
        nextReservations.assign(env->map.size(), -1);

        // robot 0 has the highest base priority, the last robot the lowest
        const int n = env->num_of_agents;
        agents.reserve(n);
        for (int i = 0; i < n; ++i) {
            const double base = (n - i) / (n + 1.0);
            agents.push_back(Agent{i, base, base});
            order.push_back(i);
        }
    }

    void nextStep(int timeLimitMs, std::vector<Action>& actions) {
        const auto start = Clock::now();
        const int budgetMs = std::max(1, std::min(kDeadlineMs, timeLimitMs - kMarginMs));
        deadline = start + std::chrono::milliseconds(budgetMs);
        aborted = false;
        calls = 0;
        const int n = env->num_of_agents;

        for (const Agent& a : agents) { prevReservations[loc(a)] = a.id; }

        // a robot on its way gains priority every step; with a new goal it starts again from its base priority
        for (Agent& a : agents) {
            if (isNewGoal(a)) { setGoal(a); a.p = a.baseP; }
            else if (loc(a) != a.goal) { a.p += 1; }
        }
        std::sort(order.begin(), order.end(), [this](int x, int y) { return agents[x].p > agents[y].p; });

        for (Agent& a : agents) {
            if (!a.table) { a.table = cover->makeTable(a.goal); }
        }

        // plan in priority order; robots not reached before the deadline wait
        for (int id : order) {
            if (Clock::now() > deadline) { break; }
            if (agents[id].nextLoc == -1) { getNextLoc(agents[id], nullptr); }
        }

        actions.assign(n, Action::NA);
        std::vector<bool> visited(n, false);
        for (int i = 0; i < n; ++i) {
            if (actions[i] == Action::NA) { actions[i] = getNextAction(actions, visited, agents[i]); }
            agents[i].nextLoc = -1;
        }

        std::fill(prevReservations.begin(), prevReservations.end(), -1);
        std::fill(nextReservations.begin(), nextReservations.end(), -1);
    }

private:
    SharedEnvironment* env;
    std::unique_ptr<CoverPoints> cover;
    std::vector<Agent> agents;
    std::vector<int> order;                  // robot ids, highest priority first
    std::vector<int> prevReservations;       // cell -> robot standing on it now
    std::vector<int> nextReservations;       // cell -> robot that will stand on it after this step

    // the priority recursion gives up (the robots wait) when the step deadline has passed
    Clock::time_point deadline;
    bool aborted = false;
    unsigned calls = 0;

    int loc(const Agent& a) const { return env->curr_states[a.id].location; }
    int dir(const Agent& a) const { return env->curr_states[a.id].orientation; }

    bool isNewGoal(const Agent& a) const {
        if (env->goal_locations[a.id].empty()) { return a.goal == -1; }
        return a.goal != env->goal_locations[a.id].front().first;
    }

    void setGoal(Agent& a) {
        a.goal = env->goal_locations[a.id].empty() ? loc(a) : env->goal_locations[a.id].front().first;
        a.table.reset();
    }

    // Gives robot a the best free neighbour cell (lowest score). If a robot stands there, it is asked to move on first (priority
    // inheritance); b is the robot that asked a, whose cell a must not take. Returns false if a found no cell.
    bool getNextLoc(Agent& a, const Agent* b) {
        // the recursion can blow up in narrow places, so the deadline is checked in here too
        if (aborted) { return false; }
        if ((++calls & 255u) == 0 && Clock::now() > deadline) { aborted = true; return false; }

        auto neighbors = getNeighbors(a);
        std::sort(neighbors.begin(), neighbors.end(),
            [this, &a](const std::pair<int, int>& x, const std::pair<int, int>& y) { return neighborTieBreak(x, y, a); });

        for (const auto& neighbor : neighbors) {
            const int cellNext = neighbor.first;
            if (nextReservations[cellNext] != -1) { continue; }
            if (b != nullptr && prevReservations[cellNext] == b->id) { continue; }

            a.nextLoc = cellNext;
            nextReservations[cellNext] = a.id;

            const int other = prevReservations[cellNext];
            if (other != -1 && agents[other].nextLoc == -1 && !getNextLoc(agents[other], &a)) {
                a.nextLoc = -1;
                nextReservations[cellNext] = -1;
                continue;
            }
            return true;
        }
        return false;
    }

    // the cells a robot can move to, each with its score
    std::vector<std::pair<int, int>> getNeighbors(const Agent& a) const {
        std::vector<std::pair<int, int>> result;
        const int from = loc(a);
        for (int d = 0; d < 4; ++d) {
            const int v = cover->neighbor(from, d);
            if (v >= 0) { result.emplace_back(v, a.table->dist(v)); }
        }
        return result;
    }

    // Turns the chosen cell into an action (forward, turn or wait). A robot moves forward only if the robot in front of it does.
    Action getNextAction(std::vector<Action>& actions, std::vector<bool>& visited, const Agent& a) {
        if (a.nextLoc == -1 || a.nextLoc == loc(a)) { return Action::W; }
        const int want = directionOf(a.nextLoc - loc(a), env->cols);
        Action action;

        if (dir(a) == want) {
            const int other = prevReservations[a.nextLoc];
            assert(other != a.id);
            if (other != -1 && actions[other] == Action::NA && !visited[other]) {
                visited[a.id] = true;
                actions[other] = getNextAction(actions, visited, agents[other]);
                action = nextReservations[a.nextLoc] == a.id ? Action::FW : Action::W;
            } else if (other != -1 && actions[other] == Action::NA && visited[other]) {
                action = Action::FW;
            } else if (other != -1 && actions[other] != Action::FW) {
                action = Action::W;
            } else {
                action = Action::FW;
            }
        } else {
            const int incr = want - dir(a);
            action = (incr == 1 || incr == -3 || incr == 2 || incr == -2) ? Action::CR : Action::CCR;
        }

        // a robot that does not move keeps its own cell
        if (action != Action::FW) {
            if (nextReservations[a.nextLoc] == a.id) { nextReservations[a.nextLoc] = -1; }
            nextReservations[loc(a)] = a.id;
        }
        return action;
    }

    // lower score first; on equal scores a free cell first, then fewer turns
    bool neighborTieBreak(const std::pair<int, int>& x, const std::pair<int, int>& y, const Agent& a) const {
        if (x.second != y.second) { return x.second < y.second; }
        if (prevReservations[x.first] == prevReservations[y.first]) {
            return turnCount(a, x.first) < turnCount(a, y.first);
        }
        return prevReservations[x.first] == -1;
    }

    int turnCount(const Agent& a, int toLoc) const {
        const int diff = toLoc - loc(a);
        if (diff == 0) { return 99; }
        const int turns = std::abs(directionOf(diff, env->cols) - dir(a));
        return turns == 3 ? 1 : turns;
    }
};

std::unique_ptr<PIBT> planner;            // one per process: the harness creates a single MAPFPlanner

}  // namespace

void MAPFPlanner::initialize(int preprocess_time_limit) {
    planner = std::make_unique<PIBT>(env);
}

void MAPFPlanner::plan(int time_limit, std::vector<Action>& actions) {
    // the harness gives the limit in seconds (INT_MAX = none); PIBT works in milliseconds
    const long long ms = std::min(2000000000LL, (long long)time_limit * 1000);
    planner->nextStep((int)ms, actions);
}

// Hipparcos with junction clusters, in one file. Replaces src/MAPFPlanner.cpp of the 2024 start kit (MAPFPlanner.h stays as in the kit).
//
// The planner is that of Team Hipparcos (2024 competition code archive) with one change: where its clusters start.
// Team Hipparcos puts the base points on a uniform grid; here they sit on the junctions of the skeleton of the map.
//
//   once per map   1. Skeleton: the thin network of corridors that is left when a map is reduced layer by layer.
//                  2. Base points: the crossings of the skeleton, then its T junctions; a junction closer than a minimum distance to a
//                     base point already chosen is skipped. A part of the map without any base point gets one.
//                  3. Clusters: all base points grow at the same time (a step costs 1, a turn 2); a cell joins the first to reach it.
//                  4. Between every pair of clusters: the distance of their base points and the clusters passed on the way.
//   every step     5. A robot heads for the next cluster on the way to its goal (and for the goal once it is in the goal's cluster);
//                     the distances of its moves come from a resumable A* (RRA*). Its priority is its distance to the goal.
//                  6. PIBT plans up to 20 steps ahead; the rest of the time a large neighbourhood search (LNS) re-plans groups of
//                     10 robots and keeps a new plan when it is shorter.
//
// Setting (environment variable, read once): JUNCTION_MIN_DIST, the minimum distance between two base points (default 10).

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <limits>
#include <list>
#include <memory>
#include <queue>
#include <random>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "MAPFPlanner.h"
#include "const.h"

namespace {

constexpr int kSkeletonStep = 24;         // in big open areas the skeleton keeps a grid of this spacing
constexpr int kDefaultMinDistance = 10;   // smallest distance between two base points

// true if `to` is a free cell of the map next to `from` (or `from` itself)
bool validMove(const SharedEnvironment* env, int to, int from) {
    if (to < 0 || to >= (int)env->map.size() || env->map[to] == 1) { return false; }
    return std::abs(to / env->cols - from / env->cols) + std::abs(to % env->cols - from % env->cols) <= 1;
}

int manhattan(const SharedEnvironment* env, int a, int b) {
    return std::abs(a / env->cols - b / env->cols) + std::abs(a % env->cols - b % env->cols);
}

// ------------------------------------------------------------------------------------------------------------------------------------
// Skeleton of the map: every free cell gets its distance to the nearest obstacle, then the cells next to an obstacle that are not
// needed for the connection are removed layer by layer, and the dead-end spurs after each layer.
class Skeleton {
public:
    Skeleton(const SharedEnvironment* env, int step) : rows(env->rows), cols(env->cols), step(step) {
        start(env);
        while (reduce(true)) { reduce(false); }
    }

    bool contains(int loc) const { return reduced[loc] > 0; }
    int neighbours(int loc) const { return countVertical(loc) + countHorizontal(loc); }

private:
    int rows, cols, step;
    std::vector<int> reduced;             // > 0: part of the skeleton, the distance to the nearest removed cell or obstacle

    // applies visit(cell, row, column, running distance) in four sweeps: left to right, right to left, downwards, upwards
    template <class Visit>
    void sweeps(Visit visit) const {
        int d;
        for (int x = 0; x < rows; ++x)      { d = 0; for (int y = 0; y < cols; ++y)      { visit(0, x, y, d); } }
        for (int x = rows - 1; x >= 0; --x) { d = 0; for (int y = cols - 1; y >= 0; --y) { visit(1, x, y, d); } }
        for (int y = 0; y < cols; ++y)      { d = 0; for (int x = 0; x < rows; ++x)      { visit(2, x, y, d); } }
        for (int y = cols - 1; y >= 0; --y) { d = 0; for (int x = rows - 1; x >= 0; --x) { visit(3, x, y, d); } }
    }

    // distance to the nearest obstacle; on a grid of `step` cells the distance restarts at 0 in open areas
    void start(const SharedEnvironment* env) {
        reduced.assign((size_t)rows * cols, 0);
        sweeps([&](int pass, int x, int y, int& d) {
            const int loc = x * cols + y;
            d = env->map[loc] == 1 ? 0 : (pass == 0 ? d + 1 : std::min(d + 1, reduced[loc]));
            if (d > step && x % step == 0 && y % step == 0) { d = 0; }
            reduced[loc] = d;
        });
    }

    // the same sweeps, every removed cell or obstacle (value 0) is a source
    void update() {
        sweeps([&](int, int x, int y, int& d) {
            const int loc = x * cols + y;
            d = reduced[loc] == 0 ? 0 : std::min(d + 1, reduced[loc]);
            reduced[loc] = d;
        });
    }

    // keepConnected: remove the cells next to an obstacle that are not needed for the connection; otherwise remove the dead-end spurs
    bool reduce(bool keepConnected) {
        bool changed = false;
        const int n = (int)reduced.size();
        if (keepConnected) {
            for (int i = 0; i < n; ++i) {
                if (reduced[i] == 1 && removable(i) && neighbours(i) > 1) { changed = true; reduced[i] = 0; }
            }
        } else {
            std::vector<int> next = reduced;
            for (int i = 0; i < n; ++i) {
                if (reduced[i] == 1 && neighbours(i) == 1) { changed = true; next[i] = 0; }
            }
            reduced = next;
        }
        update();
        return changed;
    }

    // false for a bridge cell and for the corner cases of 2 x 2 blocks
    bool removable(int loc) const {
        const int n = (int)reduced.size(), x = loc / cols, y = loc % cols;
        const int h = countHorizontal(loc), v = countVertical(loc);
        if ((v == 0 && h == 2) || (v == 2 && h == 0)) { return false; }
        const bool north = loc >= cols && reduced[loc - cols] > 0, south = loc + cols < n && reduced[loc + cols] > 0;
        const bool east = y + 1 < cols && reduced[loc + 1] > 0, west = y > 0 && reduced[loc - 1] > 0;
        if (north && east && !(x > 0 && y + 1 < cols && reduced[loc - cols + 1] > 0)) { return false; }
        if (south && east && !(x + 1 < rows && y + 1 < cols && reduced[loc + cols + 1] > 0)) { return false; }
        if (south && west && !(x + 1 < rows && reduced[loc + cols - 1] > 0)) { return false; }
        if (north && west && !(x > 0 && y > 0 && reduced[loc - cols - 1] > 0)) { return false; }
        return true;
    }

    int countVertical(int loc) const {
        return (loc >= cols && reduced[loc - cols] > 0) + (loc + cols < (int)reduced.size() && reduced[loc + cols] > 0);
    }

    int countHorizontal(int loc) const {
        const int y = loc % cols;
        return (y > 0 && reduced[loc - 1] > 0) + (y + 1 < cols && reduced[loc + 1] > 0);
    }
};

// ------------------------------------------------------------------------------------------------------------------------------------
// Clusters (areas) of the map and the distances between them.
struct DijkstraNode {
    int location;
    int parentLocation;
    int g;
    DijkstraNode* parent;

    DijkstraNode(int location, int parentLocation, int g, DijkstraNode* parent = nullptr)
        : location(location), parentLocation(parentLocation), g(g), parent(parent) { }

    struct Compare {
        bool operator()(const DijkstraNode* a, const DijkstraNode* b) const { return a->g > b->g; }
    };
};

class MapUtils {
public:
    static constexpr int EMPTY = -1;
    static constexpr int DEADLOC = 2;

    struct PairHash {
        size_t operator()(const std::pair<int, int>& x) const { return x.first ^ x.second; }
    };

    std::vector<int> deadEndMap;                  // the map with the dead ends marked DEADLOC
    std::vector<int> basePoints;
    std::vector<int> areaMap;                     // cell -> cluster
    std::vector<std::vector<std::pair<int, std::list<int>>>> basePointDistances;   // [from][to]: distance and clusters on the way
    std::unordered_map<int, std::unordered_set<int>> areaLocations;
    std::unordered_map<int, std::unordered_set<std::pair<int, int>, PairHash>> areaBorders;   // (cell, direction) just outside a cluster

    explicit MapUtils(SharedEnvironment* env) : env(env) { }

    void eraseDeadEnds() {
        deadEndMap = env->map;
        bool changed = true;
        while (changed) {
            changed = false;
            for (int i = 0; i < (int)deadEndMap.size(); ++i) {
                if (deadEndMap[i] > 0) { continue; }
                if (isDeadLoc(i)) { deadEndMap[i] = DEADLOC; changed = true; }
            }
        }
    }

    void divideIntoAreas(int minDistance) {
        markJunctionBasePoints(minDistance);
        createAreasAroundBasePoints();
    }

    void calculateDistanceBetweenAreas() {
        basePointDistances.resize(basePoints.size(), std::vector<std::pair<int, std::list<int>>>(basePoints.size(), {EMPTY, {}}));
        boost::asio::thread_pool pool(boost::thread::hardware_concurrency());
        for (int i = 0; i < (int)basePoints.size(); ++i) {
            const int start = basePoints[i];
            boost::asio::post(pool, [this, start, i]() { dijkstra(start, i); });
        }
        pool.join();
    }

private:
    SharedEnvironment* env;

    // a cell with three blocked sides, or two blocked sides and a dead end next to it
    bool isDeadLoc(int location) const {
        int walls = 0, dead = 0;
        const int x = location / env->cols, y = location % env->cols;
        auto look = [&](bool inside, int next) {
            if (!inside || deadEndMap[next] == 1) { ++walls; }
            else if (deadEndMap[next] == DEADLOC) { ++dead; }
        };
        look(y + 1 < env->cols, location + 1);
        look(y - 1 >= 0, location - 1);
        look(x - 1 >= 0, location - env->cols);
        look(x + 1 < env->rows, location + env->cols);
        return walls == 3 || (walls == 2 && dead == 1);
    }

    // Base points on the junctions of the skeleton (crossings first, then T junctions, in map order). A junction is skipped when it is
    // closer than minDistance steps to a base point already chosen. Every connected part of the map that got none gets one.
    void markJunctionBasePoints(int minDistance) {
        const int n = (int)env->map.size();
        areaMap.resize(n, EMPTY);
        const Skeleton skeleton(env, kSkeletonStep);

        std::vector<int> crossings, tees;
        for (int loc = 0; loc < n; ++loc) {
            if (!skeleton.contains(loc)) { continue; }
            const int k = skeleton.neighbours(loc);
            if (k == 4) { crossings.push_back(loc); }
            else if (k == 3) { tees.push_back(loc); }
        }

        // breadth-first search over free cells; visit(cell) for every cell within limit steps of start
        std::vector<int> seen(n, -1), queue, steps(n, 0);
        int stamp = 0;
        auto search = [&](int start, int limit, const std::function<void(int)>& visit) {
            ++stamp;
            queue.assign(1, start);
            seen[start] = stamp;
            steps[start] = 0;
            for (size_t head = 0; head < queue.size(); ++head) {
                const int u = queue[head];
                visit(u);
                if (steps[u] >= limit) { continue; }
                for (int v : {u + 1, u + env->cols, u - 1, u - env->cols}) {
                    if (validMove(env, v, u) && env->map[v] == 0 && seen[v] != stamp) {
                        seen[v] = stamp;
                        steps[v] = steps[u] + 1;
                        queue.push_back(v);
                    }
                }
            }
        };

        std::vector<char> blocked(n, 0);
        auto add = [&](int loc) {
            if (blocked[loc]) { return; }
            basePoints.push_back(loc);
            search(loc, minDistance - 1, [&](int v) { blocked[v] = 1; });
        };
        for (int loc : crossings) { add(loc); }
        for (int loc : tees) { add(loc); }

        std::vector<char> hasBasePoint(n, 0), inPart(n, 0);
        for (int loc : basePoints) { hasBasePoint[loc] = 1; }
        for (int loc = 0; loc < n; ++loc) {
            if (env->map[loc] != 0 || inPart[loc]) { continue; }
            bool found = false;
            search(loc, n, [&](int v) { inPart[v] = 1; if (hasBasePoint[v]) { found = true; } });
            if (!found) { basePoints.push_back(loc); }
        }
    }

    // all base points grow at the same time: round d lets each take the cells it reaches with cost <= d
    void createAreasAroundBasePoints() {
        const int count = (int)basePoints.size();
        std::vector<std::priority_queue<DijkstraNode*, std::vector<DijkstraNode*>, DijkstraNode::Compare>> open(count);
        std::vector<std::unordered_map<int, DijkstraNode*>> closed(count);
        std::vector<std::unordered_map<int, DijkstraNode*>> allNodes(count);

        for (int i = 0; i < count; ++i) {
            DijkstraNode* s = new DijkstraNode(basePoints[i], EMPTY, 0);
            open[i].push(s);
            allNodes[i][s->location] = s;
        }

        int distance = 0;
        bool running = true;
        while (running) {
            running = false;
            for (int i = 0; i < count; ++i) {
                if (open[i].empty()) { continue; }

                running = true;
                DijkstraNode* curr = open[i].top();
                while (!open[i].empty() && curr->g <= distance) {
                    open[i].pop();
                    closed[i][curr->location] = curr;

                    for (const auto& neighbor : getNeighbors(curr->location, curr->parentLocation)) {
                        if (closed[i].find(neighbor.first) != closed[i].end()) { continue; }
                        if (allNodes[i].find(neighbor.first) != allNodes[i].end()) {
                            DijkstraNode* old = allNodes[i][neighbor.first];
                            if (curr->g + neighbor.second < old->g) {
                                old->g = curr->g + neighbor.second;
                                old->parentLocation = curr->location;
                            }
                        } else if (areaMap[neighbor.first] == EMPTY) {
                            DijkstraNode* next = new DijkstraNode(neighbor.first, curr->location, curr->g + neighbor.second);
                            open[i].push(next);
                            allNodes[i][next->location] = next;
                        }
                    }

                    if (areaMap[curr->location] == EMPTY) {
                        areaMap[curr->location] = i;
                        areaLocations[i].emplace(curr->location);
                    }

                    if (!open[i].empty()) { curr = open[i].top(); }
                }
            }
            ++distance;
        }

        for (auto& nodes : allNodes) {
            for (auto& n : nodes) { delete n.second; }
        }
        for (int i = 0; i < count; ++i) { createAreaBorders(i); }
    }

    void createAreaBorders(int areaId) {
        for (int location : areaLocations[areaId]) {
            const int next[4] = {location + 1, location + env->cols, location - 1, location - env->cols};
            for (int d = 0; d < 4; ++d) {
                if (validMove(env, next[d], location) && areaMap[next[d]] != areaId) {
                    areaBorders[areaId].emplace(next[d], (d + 2) % 4);
                }
            }
        }
    }

    // neighbours of a cell with their cost: 1 straight on, 2 with a turn
    std::list<std::pair<int, int>> getNeighbors(int loc, int parent) const {
        std::list<std::pair<int, int>> neighbors;
        for (int candidate : {loc + 1, loc + env->cols, loc - 1, loc - env->cols}) {
            if (candidate == parent || !validMove(env, candidate, loc)) { continue; }
            const int cost = (candidate == loc + (loc - parent) || parent == EMPTY) ? 1 : 2;
            neighbors.emplace_back(candidate, cost);
        }
        return neighbors;
    }

    // from one base point to all others over the whole map
    void dijkstra(int startLoc, int id) {
        std::priority_queue<DijkstraNode*, std::vector<DijkstraNode*>, DijkstraNode::Compare> open;
        std::unordered_map<int, DijkstraNode*> closed;
        std::unordered_map<int, DijkstraNode*> allNodes;

        DijkstraNode* s = new DijkstraNode(startLoc, EMPTY, 0);
        open.push(s);
        allNodes[s->location] = s;

        while (!open.empty()) {
            DijkstraNode* curr = open.top();
            open.pop();
            closed[curr->location] = curr;

            for (const auto& neighbor : getNeighbors(curr->location, curr->parentLocation)) {
                if (closed.find(neighbor.first) != closed.end()) { continue; }
                if (allNodes.find(neighbor.first) != allNodes.end()) {
                    DijkstraNode* old = allNodes[neighbor.first];
                    if (curr->g + neighbor.second < old->g) {
                        old->g = curr->g + neighbor.second;
                        old->parentLocation = curr->location;
                        old->parent = curr;
                    }
                } else {
                    DijkstraNode* next = new DijkstraNode(neighbor.first, curr->location, curr->g + neighbor.second, curr);
                    open.push(next);
                    allNodes[next->location] = next;
                }
            }
        }

        for (int i = 0; i < (int)basePoints.size(); ++i) {
            const auto it = closed.find(basePoints[i]);
            if (it != closed.end()) {
                basePointDistances[id][i].first = it->second->g;
                backTrack(id, i, it->second);
            }
        }

        for (auto& n : allNodes) { delete n.second; }
    }

    // the clusters on the path from one base point to another, each listed once
    void backTrack(int from, int to, DijkstraNode* current) {
        std::list<int>& route = basePointDistances[from][to].second;
        route.push_front(to);
        int areaId = to;
        while (areaId != from) {
            current = current->parent;
            areaId = areaMap[current->location];
            if (route.front() != areaId) { route.push_front(areaId); }
        }

        std::unordered_set<int> seen;
        auto it = route.begin();
        while (it != route.end()) {
            if (seen.find(*it) != seen.end()) { it = route.erase(it); }
            else { seen.insert(*it); ++it; }
        }
    }
};

// ------------------------------------------------------------------------------------------------------------------------------------
// Resumable reverse A* over (cell, direction): searches backwards from the goal (or from the border of the next cluster) and continues
// only as far as the cells that are asked for.
struct RRAstarNode {
    int location;
    int direction;
    int g, h;

    RRAstarNode(int location, int direction, int g, int h) : location(location), direction(direction), g(g), h(h) { }

    int f() const { return g + h; }

    struct Compare {
        bool operator()(const RRAstarNode* a, const RRAstarNode* b) const {
            if (a->f() == b->f()) { return a->g <= b->g; }
            return a->f() > b->f();
        }
    };
};

class RRAstar {
public:
    explicit RRAstar(SharedEnvironment* env) : env(env) { }
    ~RRAstar() { for (auto n : allNodes) { delete n.second; } }

    // search from a goal cell
    void initialize(int startLoc, int startDir, int goal) {
        start = startLoc;
        const int next[4] = {goal + 1, goal + env->cols, goal - 1, goal - env->cols};
        for (int d = 0; d < 4; ++d) {
            if (!validMove(env, next[d], goal)) { continue; }
            const int dir = (d + 2) % 4;
            RRAstarNode* n = new RRAstarNode(next[d], dir, 1, manhattan(env, next[d], startLoc));
            open.push(n);
            allNodes[next[d] * 4 + dir] = n;
        }
        for (int i = 0; i < 4; ++i) {
            RRAstarNode* s = new RRAstarNode(goal, i, 0, 0);
            closed[goal * 4 + i] = s;
            allNodes[goal * 4 + i] = s;
        }
        if (startLoc != goal) { resume(startLoc, startDir); }
    }

    // search from a whole cluster
    void initialize(int startLoc, int startDir, int areaId, const MapUtils* maputils) {
        start = startLoc;
        for (const auto& border : maputils->areaBorders.at(areaId)) {
            RRAstarNode* n = new RRAstarNode(border.first, border.second, 1, manhattan(env, border.first, startLoc));
            open.push(n);
            allNodes[border.first * 4 + border.second] = n;
        }
        for (int areaLocation : maputils->areaLocations.at(areaId)) {
            for (int i = 0; i < 4; ++i) {
                RRAstarNode* s = new RRAstarNode(areaLocation, i, 0, 0);
                closed[areaLocation * 4 + i] = s;
                allNodes[areaLocation * 4 + i] = s;
            }
        }
        resume(startLoc, startDir);
    }

    int abstractDist(int loc, int dir) {
        const int i = loc * 4 + dir;
        const auto it = closed.find(i);
        if (it != closed.end()) { return it->second->g; }
        if (resume(loc, dir)) { return closed[i]->g; }
        return std::numeric_limits<int>::max();
    }

    void reset() {
        for (auto n : allNodes) { delete n.second; }
        allNodes.clear();
        closed.clear();
        open = std::priority_queue<RRAstarNode*, std::vector<RRAstarNode*>, RRAstarNode::Compare>();
    }

private:
    SharedEnvironment* env;
    std::priority_queue<RRAstarNode*, std::vector<RRAstarNode*>, RRAstarNode::Compare> open;
    std::unordered_map<int, RRAstarNode*> closed;
    std::unordered_map<int, RRAstarNode*> allNodes;
    int start = 0;

    bool resume(int loc, int dir) {
        while (!open.empty()) {
            RRAstarNode* curr = open.top();
            open.pop();
            closed[curr->location * 4 + curr->direction] = curr;

            for (const auto& neighbor : getNeighbors(curr->location, curr->direction)) {
                const int i = neighbor.first * 4 + neighbor.second;
                if (closed.find(i) != closed.end()) { continue; }
                if (allNodes.find(i) != allNodes.end()) {
                    RRAstarNode* old = allNodes[i];
                    if (curr->g + 1 < old->g) { old->g = curr->g + 1; }
                } else {
                    RRAstarNode* next = new RRAstarNode(neighbor.first, neighbor.second, curr->g + 1, manhattan(env, neighbor.first, start));
                    open.push(next);
                    allNodes[i] = next;
                }
            }

            if (loc == curr->location && dir == curr->direction) { return true; }
        }
        return false;
    }

    // backwards moves: one cell against the direction, or a turn
    std::list<std::pair<int, int>> getNeighbors(int loc, int dir) const {
        std::list<std::pair<int, int>> neighbors;
        const int candidates[4] = {loc - 1, loc - env->cols, loc + 1, loc + env->cols};
        if (validMove(env, candidates[dir], loc)) { neighbors.emplace_back(candidates[dir], dir); }
        neighbors.emplace_back(loc, (dir + 3) % 4);
        neighbors.emplace_back(loc, (dir + 1) % 4);
        return neighbors;
    }
};

// ------------------------------------------------------------------------------------------------------------------------------------
// One robot: its planned cells and directions, and its distance heuristic.
class Agent {
public:
    const int id;
    int p = -1;                                   // priority: the distance to the goal, smaller plans first
    int goalTime = 0;                             // planned step at which it reaches its goal
    std::vector<std::pair<int, int>> locations;   // (cell, direction) per planned step

    Agent(int id, SharedEnvironment* env, MapUtils* maputils) : id(id), env(env), maputils(maputils), heuristic(env) { }

    int getGoal() const { return goal; }

    int getLoc(int time) const {
        if ((int)locations.size() < time) { assert(false); return -1; }
        return locations[time].first;
    }

    int getDir(int time) const {
        if ((int)locations.size() <= time) { assert(false); return -1; }
        return locations[time].second;
    }

    bool isNewGoal() const {
        if (env->goal_locations[id].empty()) { return goal == -1; }
        return goal != env->goal_locations[id].front().first;
    }

    void boostPriority() { p -= 1000; }

    // new goal: pick the way over the clusters and set the priority
    void setGoal() {
        goal = env->goal_locations[id].empty() ? getLoc(0) : env->goal_locations[id].front().first;
        const int areaFrom = maputils->areaMap[getLoc(0)];
        const int areaTo = maputils->areaMap[goal];
        nextArea = maputils->basePointDistances[areaFrom][areaTo].second.cbegin();
        endArea = maputils->basePointDistances[areaFrom][areaTo].second.cend();
        initializeHeuristic(areaFrom, areaTo, 0);
        goalDist = maputils->basePointDistances[areaFrom][areaTo].first + heuristic.abstractDist(getLoc(0), getDir(0));
        p = goalDist;
    }

    // the moves of a step with their distance: the four neighbours and waiting
    std::vector<std::pair<int, int>> getNeighborsWithDist(int time) {
        std::vector<std::pair<int, int>> neighbours;
        const int loc = getLoc(time);
        const int candidates[4] = {loc + 1, loc + env->cols, loc - 1, loc - env->cols};
        for (int i = 0; i < 4; ++i) {
            if (validMove(env, candidates[i], loc)) { neighbours.emplace_back(candidates[i], heuristic.abstractDist(candidates[i], i)); }
        }
        neighbours.emplace_back(loc, heuristic.abstractDist(loc, getDir(time)));
        return neighbours;
    }

    // computes the distances of the first step ahead of time (called from several threads, one robot each)
    void calculateNeighborDists(int time) {
        setArea(time);
        getNeighborsWithDist(time);
    }

private:
    SharedEnvironment* env;
    MapUtils* maputils;
    RRAstar heuristic;
    int goal = -1;
    int goalDist = -1;
    std::list<int>::const_iterator nextArea;      // next cluster on the way
    std::list<int>::const_iterator endArea;

    // a robot that reached the next cluster searches toward the one after it
    void setArea(int time) {
        const int areaFrom = maputils->areaMap[getLoc(time)];
        if (nextArea != endArea && areaFrom == *nextArea) { initializeHeuristic(areaFrom, maputils->areaMap[goal], time); }
    }

    void initializeHeuristic(int areaFrom, int areaTo, int time) {
        heuristic.reset();
        if (areaFrom == areaTo) {
            nextArea = endArea;
            heuristic.initialize(getLoc(time), getDir(time), goal);
        } else {
            ++nextArea;
            heuristic.initialize(getLoc(time), getDir(time), *nextArea, maputils);
        }
    }
};

// ------------------------------------------------------------------------------------------------------------------------------------
// One LNS iteration: re-plan a group of robots one after another with space-time A*, keep the result if it is shorter in total.
struct AstarNode {
    int location;
    int direction;
    int f, g, h;
    AstarNode* parent;

    AstarNode(int location, int direction, int g, int h, AstarNode* parent)
        : location(location), direction(direction), f(g + h), g(g), h(h), parent(parent) { }

    struct Compare {
        bool operator()(const AstarNode* a, const AstarNode* b) const {
            if (a->f == b->f) { return a->g <= b->g; }
            return a->f > b->f;
        }
    };
};

class Replan {
public:
    Replan(SharedEnvironment* env, std::vector<std::vector<int>>& reservations)
        : env(env), reservations(reservations), reservationsCopy(reservations) { }
    ~Replan() { for (auto a : agentsCopy) { delete a; } }

    bool replan(std::vector<Agent*>& agents) {
        agentsCopy.clear();
        std::random_device rd;
        std::mt19937 g(rd());

        for (int i = 0; i < (int)agents.size(); ++i) {
            Agent* agent = new Agent(agents[i]->id, env, nullptr);
            agent->locations = agents[i]->locations;
            agent->p = i;
            agentsCopy.push_back(agent);
        }
        std::shuffle(agentsCopy.begin(), agentsCopy.end(), g);

        for (int i = 1; i < (int)reservationsCopy.size(); ++i) {
            for (auto a : agentsCopy) {
                if (i < (int)a->locations.size()) { reservationsCopy[i][a->locations[i].first] = -1; }
            }
        }

        for (auto a : agentsCopy) {
            const auto start = a->locations.front();
            const auto goal = a->locations.back();
            a->locations.resize(1);
            if (start.first == goal.first) {
                if (reservations[1][start.first] != -1) { return false; }
                reservations[1][start.first] = a->id;
                a->locations.push_back(start);
                continue;
            }
            const auto result = singleAgentPlan(a->id, start.first, start.second, goal.first, goal.second);
            if (!result.first) { return false; }
            std::copy(result.second.begin(), result.second.end(), std::back_inserter(a->locations));
        }

        int prevSum = 0, currSum = 0;
        for (int i = 0; i < (int)agents.size(); ++i) {
            agentsCopy[i]->goalTime = agentsCopy[i]->locations.size() - 1;
            currSum += agentsCopy[i]->goalTime;
            prevSum += agents[i]->goalTime;
        }
        if (currSum >= prevSum) { return false; }

        reservations = reservationsCopy;
        for (auto a : agentsCopy) {
            agents[a->p]->locations = a->locations;
            agents[a->p]->goalTime = a->goalTime;
        }
        return true;
    }

private:
    SharedEnvironment* env;
    std::vector<std::vector<int>>& reservations;
    std::vector<std::vector<int>> reservationsCopy;
    std::vector<Agent*> agentsCopy;

    std::pair<bool, std::list<std::pair<int, int>>> singleAgentPlan(int id, int start, int startDir, int end, int endDir) {
        std::list<std::pair<int, int>> path;
        std::priority_queue<AstarNode*, std::vector<AstarNode*>, AstarNode::Compare> openList;
        std::unordered_map<int, AstarNode*> allNodes;
        std::unordered_set<int> closeList;
        AstarNode* s = new AstarNode(start, startDir, 0, manhattan(env, start, end), nullptr);
        openList.push(s);
        allNodes[start * 4 + startDir] = s;
        bool success = false;

        while (!openList.empty()) {
            AstarNode* curr = openList.top();
            openList.pop();
            closeList.emplace(curr->location * 4 + curr->direction);
            if (curr->location == end && curr->direction == endDir) {
                while (curr->parent != nullptr) {
                    path.emplace_front(curr->location, curr->direction);
                    assert(reservationsCopy[curr->g][curr->location] == -1);
                    reservationsCopy[curr->g][curr->location] = id;
                    curr = curr->parent;
                }
                success = true;
                break;
            }
            if (curr->f > 40) { break; }

            if ((int)reservationsCopy.size() <= curr->g + 1) { reservationsCopy.push_back(std::vector<int>(env->map.size(), -1)); }
            for (const auto& neighbor : getNeighbors(curr->location, curr->direction, curr->g + 1)) {
                const int key = neighbor.first * 4 + neighbor.second;
                if (closeList.find(key) != closeList.end()) { continue; }
                if (allNodes.find(key) != allNodes.end()) {
                    AstarNode* old = allNodes[key];
                    if (curr->g + 1 < old->g) {
                        old->g = curr->g + 1;
                        old->f = old->h + old->g;
                        old->parent = curr;
                    }
                } else {
                    AstarNode* next = new AstarNode(neighbor.first, neighbor.second, curr->g + 1, manhattan(env, neighbor.first, end), curr);
                    openList.push(next);
                    allNodes[key] = next;
                }
            }
        }
        for (auto n : allNodes) { delete n.second; }
        return {success, path};
    }

    // free moves at a time step: forward, turn left, turn right, wait
    std::list<std::pair<int, int>> getNeighbors(int loc, int dir, int time) {
        std::list<std::pair<int, int>> neighbors;
        const int candidates[4] = {loc + 1, loc + env->cols, loc - 1, loc - env->cols};
        const int forward = candidates[dir];
        if (validMove(env, forward, loc)) {
            const int before = reservationsCopy[time - 1][forward];
            const int stays = reservationsCopy[time][loc];
            if (reservationsCopy[time][forward] == -1 && (before == -1 || before != stays)) { neighbors.emplace_back(forward, dir); }
        }
        if (reservationsCopy[time][loc] == -1) {
            neighbors.emplace_back(loc, (dir + 3) % 4);
            neighbors.emplace_back(loc, (dir + 1) % 4);
            neighbors.emplace_back(loc, dir);
        }
        return neighbors;
    }
};

// ------------------------------------------------------------------------------------------------------------------------------------
class PIBT {
public:
    explicit PIBT(SharedEnvironment* env) : env(env), maputils(env) { }
    ~PIBT() { for (auto a : agents) { delete a; } }

    void initialize() {
        agentsPerThread = env->num_of_agents / threadCnt;
        remainingAgents = env->num_of_agents % threadCnt;

        maputils.eraseDeadEnds();
        const char* minDistance = std::getenv("JUNCTION_MIN_DIST");
        maputils.divideIntoAreas(minDistance ? std::atoi(minDistance) : kDefaultMinDistance);
        maputils.calculateDistanceBetweenAreas();

        agents.reserve(env->num_of_agents);
        agentsById.reserve(env->num_of_agents);
        for (int i = 0; i < env->num_of_agents; ++i) {
            Agent* agent = new Agent(i, env, &maputils);
            agents.push_back(agent);
            agentsById.push_back(agent);
        }
    }

    // timeLimit: milliseconds left in this step; half for PIBT, the rest for LNS
    void nextStep(int timeLimit, std::vector<Action>& actions) {
        const auto startTime = std::chrono::steady_clock::now();
        const auto pibtEndTime = startTime + std::chrono::milliseconds(timeLimit / 2 - 10);
        const auto lnsEndTime = startTime + std::chrono::milliseconds(timeLimit - 20);
        time = 0;
        reservations.push_back(std::vector<int>(env->map.size(), -1));

        for (auto& agent : agents) {
            agent->goalTime = -1;
            agent->locations.clear();
            agent->locations.push_back({env->curr_states[agent->id].location, env->curr_states[agent->id].orientation});
            reservations[time][agent->getLoc(time)] = agent->id;
            if (maputils.deadEndMap[agent->getLoc(time)] == MapUtils::DEADLOC) { agent->boostPriority(); }
        }

        calculateGoalDistances();
        std::sort(agents.begin(), agents.end(), [](const Agent* a, const Agent* b) {
            if (a->p == b->p) { return a->id < b->id; }
            return a->p < b->p;
        });

        // PIBT, one future step after another
        while (time < 20 && std::chrono::steady_clock::now() < pibtEndTime) {
            reservations.push_back(std::vector<int>(env->map.size(), -1));
            for (auto& agent : agents) {
                if (agent->locations[time].first == agent->getGoal() && agent->goalTime == -1) { agent->goalTime = time; }
                if ((int)agent->locations.size() <= time + 1) { getNextLoc(agent, nullptr); }
            }
            std::vector<Action> planned(env->num_of_agents, Action::NA);
            std::vector<bool> visited(env->num_of_agents, false);
            for (auto& a : agents) {
                if (planned[a->id] == Action::NA) { planned[a->id] = getNextAction(planned, visited, a); }
            }
            ++time;
        }
        for (auto& agent : agents) {
            if (agent->goalTime == -1) { agent->goalTime = time; }
        }

        // LNS until the time is used
        std::random_device rd;
        std::mt19937 rng(rd());
        while (std::chrono::steady_clock::now() < lnsEndTime) {
            std::vector<Agent*> group;
            std::sample(agents.begin(), agents.end(), std::back_inserter(group), 10, rng);
            Replan replan(env, reservations);
            replan.replan(group);
        }

        actions = std::vector<Action>(env->num_of_agents, Action::NA);
        for (auto& a : agents) { actions[a->id] = firstAction(a); }
        reservations.clear();
    }

private:
    const unsigned int threadCnt = std::thread::hardware_concurrency();
    int agentsPerThread = 0;
    int remainingAgents = 0;
    SharedEnvironment* env;
    std::vector<Agent*> agentsById;
    std::vector<Agent*> agents;                       // sorted by priority
    std::vector<std::vector<int>> reservations;       // [step][cell] -> robot
    MapUtils maputils;
    int time = 0;

    // Chooses the cell of robot a for the next step; a robot in the way is asked to move first (priority inheritance), b is the asker.
    bool getNextLoc(Agent* a, const Agent* b) {
        a->locations.push_back({-1, -1});
        auto neighbors = a->getNeighborsWithDist(time);
        std::sort(neighbors.begin(), neighbors.end(), [this, &a](const std::pair<int, int>& n1, const std::pair<int, int>& n2) {
            if (maputils.deadEndMap[n1.first] == MapUtils::DEADLOC && a->getGoal() != n1.first) { return false; }
            if (n1.second == n2.second) { return reservations[time][n1.first] == -1; }
            return n1.second < n2.second;
        });

        for (const auto& neighbor : neighbors) {
            if (reservations[time + 1][neighbor.first] != -1) { continue; }
            if (b != nullptr && reservations[time][neighbor.first] == b->id) { continue; }

            a->locations[time + 1].first = neighbor.first;
            reservations[time + 1][neighbor.first] = a->id;

            const int other = reservations[time][neighbor.first];
            if (other != -1 && (int)agentsById[other]->locations.size() <= time + 1) {
                if (!getNextLoc(agentsById[other], a)) { continue; }
            }
            return true;
        }

        a->locations[time + 1].first = a->getLoc(time);
        reservations[time + 1][a->getLoc(time + 1)] = a->id;
        return false;
    }

    // turns the chosen cell into an action (forward, turn or wait) and fixes the cell and direction of the next step
    Action getNextAction(std::vector<Action>& actions, std::vector<bool>& visited, Agent* a) {
        if (a->getLoc(time + 1) == -1) { assert(false); }
        Action action = Action::W;
        int dir = 0;
        const int diff = a->getLoc(time + 1) - a->getLoc(time);

        if (diff == 0) {
            a->locations[time + 1].second = a->locations[time].second;
            return Action::W;
        }
        else if (diff == 1) { dir = 0; }
        else if (diff == env->cols) { dir = 1; }
        else if (diff == -1) { dir = 2; }
        else if (diff == -env->cols) { dir = 3; }
        else { assert(false); }

        if (a->getDir(time) == dir) {
            const int other = reservations[time][a->getLoc(time + 1)];
            assert(other != a->id);
            if (other != -1 && actions[other] == Action::NA && !visited[other]) {
                visited[a->id] = true;
                actions[other] = getNextAction(actions, visited, agentsById[other]);
                action = reservations[time + 1][a->getLoc(time + 1)] == a->id ? Action::FW : Action::W;
            } else if (other != -1 && actions[other] == Action::NA && visited[other]) {
                action = Action::FW;
            } else if (other != -1 && actions[other] != Action::FW) {
                action = Action::W;
            } else {
                action = Action::FW;
            }
            a->locations[time + 1].second = a->locations[time].second;
        } else {
            const int incr = dir - a->getDir(time);
            if (incr == 1 || incr == -3 || incr == 2 || incr == -2) {
                a->locations[time + 1].second = (a->locations[time].second + 1) % 4;
                action = Action::CR;
            } else if (incr == -1 || incr == 3) {
                a->locations[time + 1].second = (a->locations[time].second - 1 + 4) % 4;
                action = Action::CCR;
            }
        }

        if (action != Action::FW) {
            if (reservations[time + 1][a->getLoc(time + 1)] == a->id) { reservations[time + 1][a->getLoc(time + 1)] = -1; }
            reservations[time + 1][a->getLoc(time)] = a->id;
            a->locations[time + 1].first = a->locations[time].first;
        }
        return action;
    }

    // the action that leads from the current state to the first planned step
    Action firstAction(const Agent* a) const {
        if (a->getLoc(0) != a->getLoc(1)) { return Action::FW; }
        const int turn = a->getDir(0) - a->getDir(1);
        if (turn == -1 || turn == 3) { return Action::CR; }
        if (turn == 1 || turn == -3) { return Action::CCR; }
        return Action::W;
    }

    // new goals and the first distances, spread over all processor threads
    void calculateGoalDistances() {
        std::vector<std::thread> threads;
        int remaining = remainingAgents;
        int from = 0;
        for (int i = 0; i < (int)threadCnt; ++i) {
            const int to = from + agentsPerThread + (remaining-- > 0 ? 1 : 0);
            threads.emplace_back([this, from, to]() {
                for (int j = from; j < to; ++j) {
                    if (agents[j]->isNewGoal()) { agents[j]->setGoal(); }
                    agents[j]->calculateNeighborDists(time);
                }
            });
            from = to;
        }
        for (auto& t : threads) { t.join(); }
    }
};

std::unique_ptr<PIBT> planner;            // one per process: the harness creates a single MAPFPlanner

}  // namespace

void MAPFPlanner::initialize(int preprocess_time_limit) {
    planner = std::make_unique<PIBT>(env);
    planner->initialize();
}

// the limit is in milliseconds, and the task scheduler has already used part of it in this step
void MAPFPlanner::plan(int time_limit, std::vector<Action>& actions) {
    const auto used = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count();
    planner->nextStep((int)(time_limit - used - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE), actions);
}

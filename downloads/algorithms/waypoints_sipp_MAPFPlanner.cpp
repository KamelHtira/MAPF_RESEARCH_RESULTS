#include <MAPFPlanner.h>
#include <States.h>
#include <string>
#include <random>
#include <tuple>
#include <algorithm>
// #include <bits/stdc++.h>



const int MAP_PRINT_WIDTH = 330;
const int MAP_PRINT_OFFSET = 0;

const int REDUCEDMAP_CHUNCK_SIZE = 26;
const int CLOSENESS = 12; //REDUCEDMAP_CHUNCK_SIZE * 0.17;    we aim for the next waypoint which is not closer than CLOSENESS
const int TOO_CLOSE_ON_REDUCEDMAPWAYPOINTS = 14;
const int NEAR_WAYPOINT = 8;       //                       we say that we have reached the waypoint when we are in the NEAR_WAYPOINT region of the waypoint
const int WAYPOINT_STEP = 24;       //  there will be no waypoint closer than WAYPOINT_STEP other than the crossroads
const int GRID_SIZE = 24;   
const int DEPTH = 0;            // search depth limit about WAPOINT_STEP + 2 * NEAR_WAYPOINT   ,   DEPTH = 0 means no search limit

const int EAST = 0;   
const int SOUTH = 1;   
const int WEST = 2;   
const int NORTH = 3;   


struct AstarNode
{
    int location;
    int direction;
    int f,g,h;
    AstarNode* parent;
    int t = 0;     // safe interval end, -1 means infinity
    bool closed = false;
    AstarNode(int _location,int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}
    AstarNode(int _location,int _direction, int _g, int _h, int _t, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),t(_t),parent(_parent) {}
};


struct cmp
{
    bool operator()(AstarNode* a, AstarNode* b)
    {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};


void MAPFPlanner::initialize(int preprocess_time_limit)
{
    selfTime = -1;
    reservation.clear();
    reducedMap.clear();
    l0reducedMap.clear();
    l1reducedMap.clear();
    l2reducedMap.clear();

    for (int i = 0; i < env->num_of_agents; i++) 
    {
        list<pair<int,int>> path; // location and orientation
        paths.push_back(path);
        agentPriorities.push_back(i);
        agentRoles.push_back(IDLE);
        agentPrioritiesSorted.push_back(make_pair(i,i));
        simpleWaypointDestinations.push_back(-1);
        assignedDestinations.push_back(-1);
        list<int> waypoint;
        waypoints.push_back(waypoint);
    }
    for (int i = 0; i < env->map.size(); i++) 
    {
        sipp_reservation.push_back(new std::set<int>);
    }

    // ///////////////////////////////// hierarchy version from here ///////////////
    // hierarchyMapStart(GRID_SIZE);
    // l0reducedMap = reducedMap;
    // cout << "l0 reduced map:" << endl; 
    // printReducedMap();

    // hierarchyMapWaypointsStart(GRID_SIZE);
    // l1reducedMap = reducedMap;
    // // // TOREMOVE BEGIN
    // // reducedMap = l0reducedMap;
    // // // TOREMOVE END
    // cout << "l1 reduced map:" << endl;
    // printReducedMap();
    // ///////////////////////////////// hierarchy version end ///////////////


    /////////////////////////////// waypoint version from here ///////////////
    reduceMapStart();
    // while(reduceMap(true))
    // {}
    while(reduceMap(true))
    {
        reduceMap(false);
    }
    l0reducedMap = reducedMap;
    cout << "l0 reduced map:" << endl; 
    printReducedMap();

    reduceMapWaypointsStart();
    // // execute this l1 map reduction map only if reducedMapWaypointsAStar is used
    // while(reduceReduceMapWaypoints(WAYPOINT_STEP))
    // {}
    l1reducedMap = reducedMap;
    cout << "l1 reduced map:" << endl;
    printReducedMap();
    /////////////////////////////// waypoint version end ///////////////

}


//////////////////////////////////////////////
// plan  
//////////////////////////////////////////////
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    // we hook the waypoint planning here, put into comment if you want standard planning 
    plan_with_waypoints(time_limit,actions);
    return;
    //

    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    std::vector<int> agent_hits; 
    std::vector<int> aid_agents;
    list<tuple<int,int,int>> agent_extra_hits;
    bool replan;
    int i;

    selfTime++;
    cout << selfTime << "\n";

    updateAgentMap();

    // we update agentPriorities[] if needed, and fill agentPrioritiesSorted[], and then we sort it
    for (int j = 0; j < env->num_of_agents; j++)
    {
        if (!env->goal_locations[i].empty())
            if (assignedDestinations[j] != env->goal_locations[j].front().first)  // the agent has just received a new goal location, becomes IDLE and get new priority
            {
                agentRoles[j] = IDLE;
                agentPriorities[j] = getManhattanDistance(env->curr_states[j].location,env->goal_locations[j].front().first);
                assignedDestinations[j] = env->goal_locations[j].front().first;
            }
        agentPrioritiesSorted[j].first = agentPriorities[j];
        agentPrioritiesSorted[j].second = j;
    }
    std::sort(agentPrioritiesSorted.begin(), agentPrioritiesSorted.end());

    // plan and reserve a path for each agent in their priority order
    for (int j = 0; j < env->num_of_agents; j++) 
    {
        i = agentPrioritiesSorted[j].second;
        if (env->goal_locations[i].empty()) 
        {
            paths[i].clear(); 
            paths[i].push_back({env->curr_states[i].location, env->curr_states[i].orientation});
        } 
        else if (paths[i].empty() && (agentRoles[i] == IDLE || agentRoles[i] == WORKER))
        // else if (paths[i].empty())
        {
            // cout << "planning for agent " << i << "\n";
            agent_hits.clear();
            agent_hits = std::vector<int>(env->curr_states.size(), 0);
            agent_extra_hits.clear();
            aid_agents.clear();
            paths[i] = state_time_plan(env->curr_states[i].location,
                                    env->curr_states[i].orientation,
                                    selfTime,
                                    env->goal_locations[i].front().first,
                                    0,            // exactly
                                    0,             // depth = 0 means no depth limit
                                    reservation,
                                    agent_hits,
                                    agent_extra_hits,
                                    aid_agents,
                                    true);   // true means that we do not care about extra reservations

            if (paths[i].empty())
            {
                // agentRoles[i] = IDLE;
                paths[i].push_back({env->curr_states[i].location, env->curr_states[i].orientation});
            } 
            reserve_path(paths[i],i); 
        }
    }

    // check and resolve collisions
    pair<bool, std::vector<int>> possibleConflicts = findConflicts();
    while(possibleConflicts.first)
    {
        for(int i = 0; i < env->num_of_agents; i++)
        {
            if(paths[i].empty() || possibleConflicts.second[i] >= 0)
            {
                cancel_path_reservation(paths[i]);
                paths[i].clear();
                int nextOrientation = env->curr_states[i].orientation + 1 > 3 ? 0 : env->curr_states[i].orientation + 1;
                paths[i].emplace_front(make_pair(env->curr_states[i].location, nextOrientation));
                reserve_path(paths[i],i); 
            }
        }
        possibleConflicts = findConflicts();
    }

    for(int i = 0; i < env->num_of_agents; i++)
    {
        // submit the action
        if (paths[i].front().first != env->curr_states[i].location)
        {
            actions[i] = Action::FW; //forward action
        } 
        else if (paths[i].front().second!= env->curr_states[i].orientation)
        {
            int incr = paths[i].front().second - env->curr_states[i].orientation;
            if (incr == 1 || incr == -3)
            {
                actions[i] = Action::CR; //C--counter clockwise rotate
            } 
            else if (incr == -1 || incr == 3)
            {
                actions[i] = Action::CCR; //CCR--clockwise rotate
            } 
        }; 
        // remove submitted step from the path
        if(!paths[i].empty())
        {
            paths[i].pop_front();
        }
    }

    return;
}

//////////////////////////////////////////////
// plan with the waypoints of the reduced map
//////////////////////////////////////////////
void MAPFPlanner::plan_with_waypoints(int time_limit,vector<Action> & actions) 
{
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    std::vector<int> agent_hits;
    std::vector<int> aid_agents;
    list<tuple<int,int,int>> agent_extra_hits;
    bool replan;
    int prev_wp;
    int prev_wp_x;
    int prev_wp_y;
    int wp_x;
    int wp_y;
    int vect_x;
    int vect_y;
    int new_wp;
    int new_wp1;
    int new_wp2;
    selfTime++;
    cout << selfTime << "\n";  

    updateAgentMap();

    // plan and reserve a path for each agent
    for (int i = 0; i < env->num_of_agents; i++) 
    {
        // if(!waypoints[i].empty() && getManhattanDistance(env->curr_states[i].location, waypoints[i].front()) < TOO_CLOSE_ON_REDUCEDMAPWAYPOINTS )
        // {
        //     cancel_path_reservation(paths[i]);
        //     paths[i].clear();
        // }  

        if (env->goal_locations[i].empty()) 
        {
            // paths[i].clear(); 
            paths[i].push_back({env->curr_states[i].location, env->curr_states[i].orientation});
            reserve_path(paths[i],i); 
        } 
        else if (paths[i].empty())
        {
            // //  full path planning version ---  begin
            // int waypointStart = BFStoReducedMap(env->curr_states[i].location);
            // // cout << "waypointStart: " << waypointStart << endl;
            // int waypointEnd = BFStoReducedMap(env->goal_locations[i][0].first);
            // // cout << "waypointEnd: " << waypointEnd << endl;
            // // reducedMapAStar(waypointStart, waypointEnd,WAYPOINT_STEP,waypoints[i]);     // this requires that l1 reduced map is not reduced, because that makes the reduced map very sparse
            // reducedMapWaypointsAStar(waypointStart, waypointEnd,1,waypoints[i]);
            // simpleWaypointDestinations[i] = env->goal_locations[i][0].first;

            // pair<int,int> planning_step = make_pair(env->curr_states[i].location,env->curr_states[i].orientation);
            // int planning_time = selfTime;
            // list<pair<int,int>> etap_path;
            
            // while (!waypoints[i].empty())
            // {
            //     while(!waypoints[i].empty() && (getManhattanDistance(planning_step.first, waypoints[i].front()) < CLOSENESS))
            //     {
            //         // cout << " popping waypoint: " << waypoints[i].front();
            //         waypoints[i].pop_front();
            //     }
            //     if(waypoints[i].size() == 1 && getManhattanDistance(planning_step.first, env->goal_locations[i].front().first) < CLOSENESS)
            //     {
            //         // cout << " popping waypoint: " << waypoints[i].front();
            //         waypoints[i].pop_front();
            //     }
            //     if(!waypoints[i].empty())
            //     {
            //         //cout << "started planning for next etap for agent: " << i << endl;
            //         agent_hits = std::vector<int>(env->curr_states.size(), 0);
            //         agent_extra_hits.clear();
            //         aid_agents.clear();
            //         etap_path = state_time_plan(planning_step.first,
            //                                     planning_step.second,
            //                                     planning_time,
            //                                     waypoints[i].front(),
            //                                     NEAR_WAYPOINT,                         // near 
            //                                     DEPTH,             // depth = 0 means no depth limit
            //                                     reservation,
            //                                     agent_hits,
            //                                     agent_extra_hits,
            //                                     aid_agents,
            //                                     false);   // true means that we do not care about extra reservations
            //     }
            //     else
            //     {
            //         //cout << "started planning for the goal location of agent: " << i << endl;
            //         agent_hits = std::vector<int>(env->curr_states.size(), 0);
            //         aid_agents.clear();
            //         etap_path = state_time_plan(planning_step.first,
            //                                     planning_step.second,
            //                                     planning_time,
            //                                     env->goal_locations[i].front().first,
            //                                     0,                  // exactly
            //                                     DEPTH,             // depth = 0 means no depth limit
            //                                     reservation,
            //                                     agent_hits,
            //                                     agent_extra_hits,
            //                                     aid_agents,
            //                                     false);   // true means that we do not care about extra reservations
            //     }
            //     if(!etap_path.empty())
            //     {
            //         paths[i].splice(paths[i].end(), etap_path);
            //     }
            //     else
            //     {
            //         // paths[i].clear();
            //         // cout << "no etap_path for agent: " << i << " waypoint: " << waypoints[i].front()/env->cols << "," << waypoints[i].front()%env->cols;
            //         // cout << " location: " << env->curr_states[i].location/env->cols << "," << env->curr_states[i].location%env->cols ;
            //         // cout << " goal: " << env->goal_locations[i].front().first/env->cols << "," << env->goal_locations[i].front().first%env->cols << endl;
            //         break;
            //     }
            //     planning_step.first = paths[i].back().first;
            //     planning_step.second = paths[i].back().second;
            //     planning_time = selfTime + paths[i].size();
            // }
            // // reserve the planned path
            // if (paths[i].empty())
            //     paths[i].push_back({env->curr_states[i].location, env->curr_states[i].orientation});
            // reserve_path(paths[i],i); 
            // //  end of ---- full path planning 


            //  path planning per etap  --- begin
            if(simpleWaypointDestinations[i] != env->goal_locations[i][0].first)
            {
                int waypointStart = BFStoReducedMap(env->curr_states[i].location);
                int waypointEnd = BFStoReducedMap(env->goal_locations[i][0].first);
                reducedMapAStar(waypointStart, waypointEnd,WAYPOINT_STEP,waypoints[i]);     // this requires that l1 reduced map is not reduced, because that makes the reduced map very sparse
                // reducedMapWaypointsAStar(waypointStart, waypointEnd,1,waypoints[i]);
                simpleWaypointDestinations[i] = env->goal_locations[i][0].first;
            }
            
            while(!waypoints[i].empty() && (getManhattanDistance(env->curr_states[i].location, waypoints[i].front()) < CLOSENESS))
            {
                waypoints[i].pop_front();
            }
            if(waypoints[i].size() == 1 && getManhattanDistance(env->curr_states[i].location, env->goal_locations[i].front().first) < CLOSENESS)
            {
                waypoints[i].pop_front();
            }
            if(!waypoints[i].empty())
            {
                //cout << "started planning for next waypoint for agent: " << i << endl;
                agent_hits = std::vector<int>(env->curr_states.size(), 0);
                agent_extra_hits.clear();
                aid_agents.clear();
                paths[i] = state_time_plan(env->curr_states[i].location,
                                        env->curr_states[i].orientation,
                                        selfTime,
                                        waypoints[i].front(),
                                        NEAR_WAYPOINT,                         // near 
                                        DEPTH,             // depth = 0 means no depth limit
                                        reservation,
                                        agent_hits,
                                        agent_extra_hits,
                                        aid_agents,
                                        false);   // true means that we do not care about extra reservations
                if (paths[i].empty())
                {
                    if (paths[i].empty())
                        paths[i].push_back({env->curr_states[i].location, env->curr_states[i].orientation});
                    reserve_path(paths[i],i); 
                }
            }
            else
            {
                //cout << "started planning for the goal location of agent: " << i << endl;
                agent_hits = std::vector<int>(env->curr_states.size(), 0);
                aid_agents.clear();
                paths[i] = state_time_plan(env->curr_states[i].location,
                                        env->curr_states[i].orientation,
                                        selfTime,
                                        env->goal_locations[i].front().first,
                                        0,                  // exactly
                                        DEPTH,             // depth = 0 means no depth limit
                                        reservation,
                                        agent_hits,
                                        agent_extra_hits,
                                        aid_agents,
                                        false);   // true means that we do not care about extra reservations
                if (paths[i].empty())
                {
                    if (paths[i].empty())
                        paths[i].push_back({env->curr_states[i].location, env->curr_states[i].orientation});
                    reserve_path(paths[i],i); 
                }
            }

            // if (paths[i].empty())
            // {
            //     paths[i].push_back({env->curr_states[i].location, env->curr_states[i].orientation});
            //     // paths[i].push_back({env->curr_states[i].location, env->curr_states[i].orientation});
            // }
            reserve_path(paths[i],i); 
            //  end of ------ path planning per etap 

        }
    }

    // check and resolve collisions
    pair<bool, std::vector<int>> possibleConflicts = findConflicts();
    while(possibleConflicts.first)
    {
        for(int i = 0; i < env->num_of_agents; i++)
        {
            if(paths[i].empty() || possibleConflicts.second[i] >= 0)
            {
                // cout << "conflict for agent: " << i << " waypoint: " << waypoints[i].front()/env->cols << "," << waypoints[i].front()%env->cols;
                // cout << " location: " << env->curr_states[i].location/env->cols << "," << env->curr_states[i].location%env->cols ;
                // cout << " goal: " << env->goal_locations[i].front().first/env->cols << "," << env->goal_locations[i].front().first%env->cols << endl;
                cancel_path_reservation(paths[i]);
                paths[i].clear();
                int nextOrientation = env->curr_states[i].orientation + 1 > 3 ? 0 : env->curr_states[i].orientation + 1;
                paths[i].emplace_front(make_pair(env->curr_states[i].location, nextOrientation));
                reserve_path(paths[i],i); 
            }
        }
        possibleConflicts = findConflicts();
    }

    for(int i = 0; i < env->num_of_agents; i++)
    {
        // submit the action
        if (paths[i].front().first != env->curr_states[i].location)
        {
            actions[i] = Action::FW; //forward action
        } 
        else if (paths[i].front().second!= env->curr_states[i].orientation)
        {
            int incr = paths[i].front().second - env->curr_states[i].orientation;
            if (incr == 1 || incr == -3)
            {
                actions[i] = Action::CR; //C--counter clockwise rotate
            } 
            else if (incr == -1 || incr == 3)
            {
                actions[i] = Action::CCR; //CCR--clockwise rotate
            } 
        }; 
        // remove submitted step from the path
        if(!paths[i].empty())
        {
            paths[i].pop_front();
        }
    }

    return;
}


void  MAPFPlanner::reserve_path(list<pair<int,int>> path,int agentId)
{
    int lastLocation = -1;
    int t = 1;
    for(auto p : path)
    {
        reservation.insert({make_tuple(p.first,-1,selfTime + t),agentId});
        sipp_reservation[p.first]->insert(selfTime + t); 
        if(lastLocation != -1)
        {
            reservation.insert({make_tuple(lastLocation, p.first, selfTime + t),agentId});
        }
        lastLocation = p.first;
        t++;
    }
    return; 
} 


void  MAPFPlanner::cancel_path_reservation(list<pair<int,int>> path)
{
    int lastLocation = -1;
    int t = 1;
    for(auto p : path)
    {
        reservation.erase(make_tuple(p.first, -1, selfTime + t));
        sipp_reservation[p.first]->erase(selfTime + t);
 
        if(lastLocation != -1)
        {
            reservation.erase(make_tuple(lastLocation, p.first, selfTime + t));
        }
        lastLocation = p.first;
        t++;
    }
    return; 
} 


list<pair<int,int>> MAPFPlanner::state_time_plan(int start,int start_direct,int start_time,int end, int near, int depth, unordered_map<tuple<int,int,int>,int> &reservation, vector<int> &agent_hits, list<tuple<int,int,int>> &agent_extra_hits, vector<int> &aid_agents, bool no_extra)
{
    list<pair<int,int>> path;
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<string,AstarNode*> all_nodes;
    unordered_set<string> close_list;
    std::tuple<int,int,int> tup;
    std::tuple<int,int,int> tup2;
    std::tuple<int,int,int> tup3;
    std::tuple<int,int,int> tup4;
    pair<int,int> neighbor;
    int cost;
    int safe_interval_end;

    safe_interval_end = getSafeIntervalEnd(start,start_time); 
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), safe_interval_end, nullptr);
    open_list.push(s);
    all_nodes[std::to_string(start)+";"+std::to_string(start_direct)+";"+std::to_string(safe_interval_end)] = s;

    while (!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        safe_interval_end = getSafeIntervalEnd(curr->location,start_time + curr->g);
        close_list.emplace(std::to_string(curr->location)+";"+std::to_string(curr->direction)+";"+std::to_string(safe_interval_end));
        // TODO  optimization: move the tuple creation after the first part of the if
        tup3 = make_tuple(curr->location, -1, start_time + curr->g + 1);
        tup4 = make_tuple(curr->location, -1, start_time + curr->g + 2);
        if (getManhattanDistance(curr->location, end) <= near &&  // we have reached our destination region
            ((reservation.find(tup3) == reservation.end() &&    // the goal location is not reserved for the next 2 steps
             reservation.find(tup4) == reservation.end()) ||    // or we do not care about extra reservations
             no_extra ) )
        {
            // not ok if it arrives one step after us, because it will block or chase us
            std::tuple<int,int,int> tup3_back = make_tuple(curr->location, -1, start_time + curr->g + 1);
            if (reservation.find(tup3_back) != reservation.end())
                agent_extra_hits.push_back(tup3_back);
            else
            {
                // not ok if it arrives two steps after us from other direction, because it will block or chase us
                std::tuple<int,int,int> tup4_back = make_tuple(curr->location, -1, start_time + curr->g + 2);
                if (curr->parent!=NULL)  // this is where we came from 
                {
                    std::tuple<int,int,int> tup4_edge = make_tuple(curr->parent->location, curr->location, start_time + curr->g + 2);
                    if (reservation.find(tup4_back) != reservation.end() && reservation.find(tup4_edge) == reservation.end())
                        agent_extra_hits.push_back(tup4_back);
                }
                else
                {
                    // not ok if it arrives three steps after us from other (TODO opposite!) direction, because it will block or chase us
                    std::tuple<int,int,int> tup5_back = make_tuple(curr->location, -1, start_time + curr->g + 3);
                    if (curr->parent!=NULL)  // this is where we came from 
                    {
                        std::tuple<int,int,int> tup5_edge = make_tuple(curr->parent->location, curr->location, start_time + curr->g + 3);
                        if (reservation.find(tup5_back) != reservation.end() && reservation.find(tup5_edge) == reservation.end())
                            agent_extra_hits.push_back(tup4_back);
                    }
                }
            }  
            while(curr->parent!=NULL) 
            {
                AstarNode* before_node = curr->parent;
                int g_diff = curr->g - before_node->g;
                path.emplace_front(make_pair(curr->location, curr->direction));
                if (auto iterator = agentMap.find(curr->location); iterator != agentMap.end()){ aid_agents.push_back(iterator->second);}
                g_diff--;
                while (g_diff>0)
                {
                    path.emplace_front(make_pair(before_node->location, before_node->direction));
                    g_diff--;
                } 
                curr = before_node;
            }
            break;
        }
        // TODO: no need for reservation argument, because reservation is global in this class
        std::list<tuple<pair<int,int>,int,int>> sipp_neighbors = getSippNeighbors(curr->location, curr->direction, start_time + curr->g, reservation);
        for (const tuple<pair<int,int>,int,int>& sipp_neighbor: sipp_neighbors)
        {
            tie(neighbor,cost,safe_interval_end) = sipp_neighbor;
            tup = make_tuple(neighbor.first, -1, start_time + curr->g + cost);
            tup2 = make_tuple(neighbor.first, curr->location, start_time + curr->g + cost);

            if(depth > 0 && curr->g + cost > depth)
                continue;

            if(auto iterator=reservation.find(tup); iterator != reservation.end())
            {
                agent_hits[iterator->second]++; 
                continue;
            } 
            if(auto iterator=reservation.find(tup2); iterator != reservation.end())
            {
                agent_hits[iterator->second]++; 
                continue;
            } 
            
            safe_interval_end = getSafeIntervalEnd(neighbor.first,start_time + curr->g + cost);
            std::string key = std::to_string(neighbor.first)+";"+std::to_string(neighbor.second)+";"+std::to_string(safe_interval_end);
            if (close_list.find(key) != close_list.end())
                continue;
            if (all_nodes.find(key) != all_nodes.end())
            {
                AstarNode* old = all_nodes[key];
                if (curr->g + cost < old->g)
                {
                    old->g = curr->g+cost;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else
            {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+cost,getManhattanDistance(neighbor.first,end), safe_interval_end, curr);
                open_list.push(next_node);
                all_nodes[key] = next_node;
            }
        }
    }
    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    return path;
}


list<pair<int,int>> MAPFPlanner::space_time_plan(int start,int start_direct,int end, unordered_map<tuple<int,int,int>,int> &reservation, vector<int> &agent_hits, bool no_extra)
{
    list<pair<int,int>> path;
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<int,AstarNode*> all_nodes;
    unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;
    std::tuple<int,int,int> tup;
    std::tuple<int,int,int> tup2;
    std::tuple<int,int,int> tup3;
    std::tuple<int,int,int> tup4;
    
    while (!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);
        tup3 = make_tuple(curr->location, -1, selfTime + curr->g + 1);
        tup4 = make_tuple(curr->location, -1, selfTime + curr->g + 2);
        if (curr->location == end &&
            (reservation.find(tup3) == reservation.end() &&    // the goal location is not reserved for the next 2 steps
             reservation.find(tup4) == reservation.end() ||    // or we do not care about extra reservations
             no_extra ) )
        {
            while(curr->parent!=NULL) 
            {
                path.emplace_front(make_pair(curr->location, curr->direction));
                curr = curr->parent;
            }
            break;
        }
        list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction);
        for (const pair<int,int>& neighbor: neighbors)
        {
            tup = make_tuple(neighbor.first, -1, selfTime + curr->g + 1);
            tup2 = make_tuple(neighbor.first, curr->location, selfTime + curr->g + 1);

            if(auto iterator=reservation.find(tup); iterator != reservation.end())
            {
                agent_hits[iterator->second]++; 
                continue;
            } 
            if(auto iterator=reservation.find(tup2); iterator != reservation.end())
            {
                agent_hits[iterator->second]++; 
                continue;
            } 
            
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                continue;
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end())
            {
                AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                if (curr->g + 1 < old->g)
                {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else
            {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(neighbor.first,end), curr);
                open_list.push(next_node);
                all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    return path;
}


list<pair<int,int>> MAPFPlanner::single_agent_plan(int start,int start_direct,int end)
{
    list<pair<int,int>> path;
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<int,AstarNode*> all_nodes;
    unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;

    while (!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);
        if (curr->location == end)
        {
            while(curr->parent!=NULL) 
            {
                path.emplace_front(make_pair(curr->location, curr->direction));
                curr = curr->parent;
            }
            break;
        }
        list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction);
        for (const pair<int,int>& neighbor: neighbors)
        {
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                continue;
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end())
            {
                AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                if (curr->g + 1 < old->g)
                {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else
            {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(neighbor.first,end), curr);
                open_list.push(next_node);
                all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    return path;
}


int MAPFPlanner::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = loc1/env->cols;
    int loc1_y = loc1%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}


bool MAPFPlanner::validateMove(int loc, int loc2)
{
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;

    if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1)
        return false;

    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
        return false;
    return true;

}


list<pair<int,int>> MAPFPlanner::getNeighbors(int location,int direction)
{
    list<pair<int,int>> neighbors;
    //forward
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (forward>=0 && forward < env->map.size() && validateMove(forward,location))
        neighbors.emplace_back(make_pair(forward,new_direction));
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
    neighbors.emplace_back(make_pair(location,new_direction));
    //turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
    neighbors.emplace_back(make_pair(location,new_direction));
    neighbors.emplace_back(make_pair(location,direction)); //wait
    return neighbors;
}


std::list<tuple<pair<int,int>,int,int>> MAPFPlanner::getSippNeighbors(int location, int direction, int curr_time, unordered_map<tuple<int,int,int>,int> &reservation) 
//returns neighbors and their cost and the end of the their safe interval
{
    std::list<tuple<pair<int,int>,int,int>> neighbors;
    int safe_interval_end; 
    int forward_safe_interval_end; 
    int moving_time; 

    safe_interval_end = getSafeIntervalEnd(location,curr_time); 
    //forward
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (forward>=0 && forward < env->map.size() && validateMove(forward,location))
    {
        forward_safe_interval_end = getSafeIntervalEndWithCleanup(forward,curr_time); 
        // forward_safe_interval_end = getSafeIntervalEnd(forward,curr_time); 
        if (forward_safe_interval_end == -1)  // if forward is free forever from current time, then we can immediately try to move there
        { 
            neighbors.emplace_back(make_tuple(make_pair(forward,new_direction),1,forward_safe_interval_end));
        }
        else    // if forward is not free forever from current time, then search for the first free time and try to move then
        { 
            moving_time = curr_time;
            while (forward_safe_interval_end == moving_time) // if forward is not free at moving time, then check for the next one
            {
                moving_time++;
                forward_safe_interval_end = getSafeIntervalEnd(forward,moving_time);
            } 
            // forward is now free at moving time, so we can try to move
            neighbors.emplace_back(make_tuple(make_pair(forward,new_direction),moving_time-curr_time+1,forward_safe_interval_end));
        }
    }
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
    neighbors.emplace_back(make_tuple(make_pair(location,new_direction),1,safe_interval_end));
    //turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
    neighbors.emplace_back(make_tuple(make_pair(location,new_direction),1,safe_interval_end));
    //neighbors.emplace_back(make_pair(location,direction)); //wait
    return neighbors;
}

int MAPFPlanner::getSafeIntervalEnd(int location, int curr_time)
{
    if (sipp_reservation[location]->empty()) return -1;
    // get the end of the safe interval at this location
    auto it = sipp_reservation[location]->lower_bound(curr_time);
    if (it == sipp_reservation[location]->end())
        return -1;   //the end is infinity
    else
        return *it;   // the end of the safe interval
} 


int MAPFPlanner::getSafeIntervalEndWithCleanup(int location, int curr_time)
{
    // get the end of the safe interval at this location
    if (sipp_reservation[location]->empty()) return -1;
    sipp_reservation[location]->erase(sipp_reservation[location]->begin(), sipp_reservation[location]->find(selfTime)); // cleaning before selfTime
    auto it = sipp_reservation[location]->lower_bound(curr_time);
    if (it == sipp_reservation[location]->end())
        return -1;   //the end is infinity
    else
        return *it;   // the end of the safe interval
} 


void MAPFPlanner::updateAgentMap()
{
    // agentMap[0] = 0;
    agentMap.clear();
    for(int i = 0; i < env->num_of_agents; i++)
    {
        agentMap[env->curr_states[i].location] = i;
    }
}


pair<bool, int> MAPFPlanner::steerAway(int agentId, int loc_from, int loc_to)  
// bool = true if it works, int = number of steps needed, agentId = the agent to steer away, loc_from= the other agent moves from, loc_to= the other agent moves to
{
    return make_pair(true,0);

    int location = env->curr_states[agentId].location;
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[env->curr_states[agentId].orientation];  // this could be a default forward
    if(paths[agentId].empty() || paths[agentId].front().first == env->curr_states[agentId].location) // this robot would not move therefore it has to step away from loc_to
    {
        if (validateMove(forward,location))  // the forward step is valid
        { 
            pair<bool, int> result = forwardMoveConflict(agentId);
            if (!result.first)  // the forward step is free
            { 
                cancel_path_reservation(paths[agentId]);
                paths[agentId].clear();
                paths[agentId].emplace_front(make_pair(forward, env->curr_states[agentId].orientation));
                reserve_path(paths[agentId],agentId); 
                return make_pair(true,0);
            }
        }
        int nextOrientation = env->curr_states[agentId].orientation + 1 > 3 ? 0 : env->curr_states[agentId].orientation + 1; // clockwise rotation
        forward = candidates[nextOrientation];
        if (validateMove(forward,location))  // if the step to clockwise direction is valid then make a clockwise rotation
        { 
            cancel_path_reservation(paths[agentId]);
            paths[agentId].clear();
            paths[agentId].emplace_front(make_pair(env->curr_states[agentId].orientation, nextOrientation));
            reserve_path(paths[agentId],agentId); 
            return make_pair(true,0);
        }
    }
    return make_pair(false,0);
}


pair<bool, std::vector<int>> MAPFPlanner::findConflicts()  // bool true if conflict found, vector[i] = j means that i conflicts with j, -1 means that i is ok
{
    bool found = false;
    std::vector<int> conflict = std::vector<int>(env->num_of_agents, -1);
    std::vector<int> surroundingAgents;
    for (int i = 0; i < env->num_of_agents; i++) 
    {
        if(!paths[i].empty() && paths[i].front().first != env->curr_states[i].location) // this robot moves
        {
            pair<bool, int> result = forwardMoveConflict(i);
            if (result.first)
            {
                found = result.first;
                conflict[i] = result.second;
            } 
        }
    }
    return make_pair(found, conflict);
}


pair<bool, int> MAPFPlanner::forwardMoveConflict(int agentId)  // bool true if conflict found, int = j means that agentId conflicts with j, -1 means ok
{
    bool found = false;
    int conflict = -1;
    std::vector<int> surroundingAgents;

    int location = env->curr_states[agentId].location;
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[env->curr_states[agentId].orientation];  // this robot moves to forward
    surroundingAgents.clear();
    if (auto iterator = agentMap.find(forward); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
    if (auto iterator = agentMap.find(forward+1); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
    if (auto iterator = agentMap.find(forward+env->cols); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
    if (auto iterator = agentMap.find(forward-1); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
    if (auto iterator = agentMap.find(forward-env->cols); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
    for(int j = 0; j < surroundingAgents.size(); j++)
    {
        int agentJ = surroundingAgents[j];
        if(agentJ == agentId){ continue; }
        if( (env->curr_states[agentJ].location == forward &&                                 // the other robot is in forward and
            ((paths[agentJ].empty() || paths[agentJ].front().first == forward) ||                // does not move or
            (!paths[agentJ].empty() && paths[agentJ].front().first == location))) ||             // moves in the opposite direction
            (!paths[agentJ].empty() && paths[agentJ].front().first == forward) )             // or the other robot moves to forward 
        {
            // string agentJString = paths[agentJ].empty() ? std::to_string(env->curr_states[agentJ].location)+" empty path" : std::to_string(paths[agentJ].front().first);
            // cout << "CONFLICT FOUND! Agent between agent-location: (" << i << ", " << paths[i].front().first <<  ") <-> (" << agentJ << ", " << agentJString << ")" << endl;
            conflict = agentJ;
            found = true;
        }
    }

    return make_pair(found, conflict);
}


pair<bool, int> MAPFPlanner::crMoveConflict(int agentId)  // bool true if conflict found, int = j means that agentId conflicts with j, -1 means ok
{
    bool found = false;
    int conflict = -1;
    std::vector<int> surroundingAgents;

    int location = env->curr_states[agentId].location;
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int nextOrientation = env->curr_states[agentId].orientation + 1 > 3 ? 0 : env->curr_states[agentId].orientation + 1; 
    int forward = candidates[nextOrientation];  // this robot moves to forward
    surroundingAgents.clear();
    // check if counter rotation is ok
    if (auto iterator = agentMap.find(forward); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
    if (auto iterator = agentMap.find(forward+1); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
    if (auto iterator = agentMap.find(forward+env->cols); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
    if (auto iterator = agentMap.find(forward-1); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
    if (auto iterator = agentMap.find(forward-env->cols); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
    for(int j = 0; j < surroundingAgents.size(); j++)
    {
        int agentJ = surroundingAgents[j];
        if(agentJ == agentId){ continue; }
        if( (env->curr_states[agentJ].location == forward &&                                 // the other robot is in forward and
            ((paths[agentJ].empty() || paths[agentJ].front().first == forward) ||                // does not move or
            (!paths[agentJ].empty() && paths[agentJ].front().first == location))) ||             // moves in the opposite direction
            (!paths[agentJ].empty() && paths[agentJ].front().first == forward) )             // or the other robot moves to forward 
        {
            // string agentJString = paths[agentJ].empty() ? std::to_string(env->curr_states[agentJ].location)+" empty path" : std::to_string(paths[agentJ].front().first);
            // cout << "CONFLICT FOUND! Agent between agent-location: (" << i << ", " << paths[i].front().first <<  ") <-> (" << agentJ << ", " << agentJString << ")" << endl;
            conflict = agentJ;
            found = true;
        }
    }

    if (auto iterator = agentMap.find(forward); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
    if (auto iterator = agentMap.find(forward+1); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
    if (auto iterator = agentMap.find(forward+env->cols); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
    if (auto iterator = agentMap.find(forward-1); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
    if (auto iterator = agentMap.find(forward-env->cols); iterator != agentMap.end()){ surroundingAgents.push_back(iterator->second);}
    for(int j = 0; j < surroundingAgents.size(); j++)
    {
        int agentJ = surroundingAgents[j];
        if(agentJ == agentId){ continue; }
        if( (env->curr_states[agentJ].location == forward &&                                 // the other robot is in forward and
            ((paths[agentJ].empty() || paths[agentJ].front().first == forward) ||                // does not move or
            (!paths[agentJ].empty() && paths[agentJ].front().first == location))) ||             // moves in the opposite direction
            (!paths[agentJ].empty() && paths[agentJ].front().first == forward) )             // or the other robot moves to forward 
        {
            // string agentJString = paths[agentJ].empty() ? std::to_string(env->curr_states[agentJ].location)+" empty path" : std::to_string(paths[agentJ].front().first);
            // cout << "CONFLICT FOUND! Agent between agent-location: (" << i << ", " << paths[i].front().first <<  ") <-> (" << agentJ << ", " << agentJString << ")" << endl;
            conflict = agentJ;
            found = true;
        }
    }

    return make_pair(found, conflict);
}


// map reduction related things

void MAPFPlanner::reducedMapAStar(int start, int end, int step, list<int> & waypoints)
{
    waypoints.clear();
    priority_queue<pair<int,int>,vector<pair<int,int>>, std::greater<pair<int,int>> > open_list;
    unordered_map<int,int> all_nodes;
    unordered_map<int,int> parents;
    pair<int,int> s = make_pair(getManhattanDistance(start,end), start);
    open_list.push(s);
    all_nodes[start] = getManhattanDistance(start,end);
    parents[start] = -1;

    while (!open_list.empty())
    {
        pair<int,int> curr = open_list.top();
        int currLocation = curr.second;
        open_list.pop();
        if (currLocation == end)
        {
            int distance = 0;
            while(currLocation!=-1) 
            {
                if(distance == 0)
                {
                    waypoints.emplace_front(currLocation);
                    distance = step;
                }
                distance--;
                currLocation = parents[currLocation];
            }
            break;
        }
        //cout << "GET NEIGHBOURS" << endl;
        list<int> neighbors = getPlainNeighborsOnReducedMap(currLocation);
        int newG = all_nodes[currLocation] + 1;
        for (const int& neighbor: neighbors)
        {
            if ((all_nodes.find(neighbor) == all_nodes.end()) || newG < all_nodes[neighbor])
            {
                all_nodes[neighbor] = newG;
                open_list.push(make_pair(newG + getManhattanDistance(neighbor, end), neighbor));
                parents[neighbor] = currLocation;
            }
        }
    }
    all_nodes.clear();
    return;
}


void MAPFPlanner::reducedMapWaypointsAStar(int start, int end, int step, list<int> & waypoints)
{
    waypoints.clear();
    priority_queue<pair<int,int>,vector<pair<int,int>>, std::greater<pair<int,int>> > open_list;
    unordered_map<int,int> all_nodes;
    unordered_map<int,int> parents;
    pair<int,int> s = make_pair(getManhattanDistance(start,end), start);
    open_list.push(s);
    all_nodes[start] = getManhattanDistance(start,end);
    parents[start] = -1;

    while (!open_list.empty())
    {
        pair<int,int> curr = open_list.top();
        int currLocation = curr.second;
        open_list.pop();
        if (currLocation == end)
        {
            int distance = 0;
            while(currLocation!=-1) 
            {
                if(distance == 0)
                {
                    waypoints.emplace_front(currLocation);
                    distance = step;
                }
                distance--;
                currLocation = parents[currLocation];
            }
            break;
        }
        //cout << "GET NEIGHBOURS" << endl;
        unordered_map<int,int> tmpMap = reducedMapWaypoints[currLocation];
        list<int> neighbors;
        for(auto kv : tmpMap)
        {
            neighbors.emplace_back(kv.first);
        }
        int currG = all_nodes[currLocation];
        for (const int& neighbor: neighbors)
        {
            int newG = currG + reducedMapWaypoints[currLocation][neighbor];
            if ((all_nodes.find(neighbor) == all_nodes.end()) || newG < all_nodes[neighbor])
            {
                all_nodes[neighbor] = newG;
                open_list.push(make_pair(newG + getManhattanDistance(neighbor, end), neighbor));
                parents[neighbor] = currLocation;
            }
        }
    }
    all_nodes.clear();
    return;
}


int MAPFPlanner::BFStoReducedMap(int start) // returns the closest point on the reduced map
{
    int waypoint;
    priority_queue<pair<int,int>,vector<pair<int,int>>, std::greater<pair<int,int>> > open_list;
    unordered_map<int,int> all_nodes;
    pair<int,int> s = make_pair(0, start);
    open_list.push(s);
    all_nodes[start] = 0;

    while (!open_list.empty())
    {
        pair<int,int> curr = open_list.top();
        open_list.pop();

        if(reducedMap[curr.second] > 0)
        {
            waypoint = curr.second;
            break;
        }

        //cout << "GET NEIGHBOURS" << endl;
        list<int> neighbors = getPlainNeighbors(curr.second);
        int newG = all_nodes[curr.second] + 1;
        for (const int& neighbor: neighbors)
        {

            if ((all_nodes.find(neighbor) == all_nodes.end()) || newG < all_nodes[neighbor])
            {
                all_nodes[neighbor] = newG;
                open_list.push(make_pair(newG, neighbor));
            }
        }
    }
    all_nodes.clear();
    return waypoint;
}


void MAPFPlanner::hierarchyMapStart(int grid)
{
    for(int x = 0; x < env->rows;x++)
    {
        for(int y = 0; y < env->cols;y++)
        {
            int loc = x * env->cols + y;
            reducedMap.push_back(0);
        }
    }
    //Rows from left to right
    int gap_start = -1;
    for(int x = 0; x < env->rows;x++)
    {
        if (x%grid == 0)
        {
            gap_start = -1;
            for(int y = 0; y < env->cols;y++)
            {
                int loc = x * env->cols + y;
                // if we are in a gap and we reach the end of the row then put an exit 
                if (gap_start >=0 && y == env->cols-1)
                {
                    reducedMap[gap_start + (loc - gap_start)/2] = 1;
                    gap_start = -1;
                }
                // if we are in a gap and we reach a grid line then put an exit 
                if (gap_start >=0 && y%grid == 0)
                {
                    reducedMap[gap_start + (loc - gap_start)/2] = 1;
                    gap_start = -1;
                }
                // if we are in a gap and we reach a wall then put an exit 
                if (gap_start >= 0 && env->map[loc] == 1) 
                {
                    reducedMap[gap_start + (loc - gap_start)/2] = 1;
                    gap_start = -1;
                }
                // if we are not in a gap and we reach a gap then start a gap 
                if (gap_start < 0 && env->map[loc] == 0) 
                {
                    gap_start = loc;
                }
            }
        }
    }
    //Cols from top to bottom
    for(int y = 0; y < env->cols;y++)
    {
        if (y%grid == 0)
        {
            gap_start = -1;
            for(int x = 0; x < env->rows;x++)
            {
                int loc = x * env->cols + y;
                // if we are in a gap and we reach the end of the column then put an exit 
                if (gap_start >=0 && x == env->rows-1)
                {
                    reducedMap[(gap_start + (x - gap_start)/2) * env->cols + y] = 1;
                    gap_start = -1;
                }
                // if we are in a gap and we reach a grid line then put an exit 
                if (gap_start >=0 && x%grid == 0)
                {
                    reducedMap[(gap_start + (x - gap_start)/2) * env->cols + y] = 1;
                    gap_start = -1;
                }
                // if we are in a gap and we reach a wall then put an exit 
                if (gap_start >= 0 && env->map[loc] == 1) 
                {
                    reducedMap[(gap_start + (x - gap_start)/2) * env->cols + y] = 1;
                    gap_start = -1;
                }
                // if we are not in a gap and we reach a gap then start a gap 
                if (gap_start < 0 && env->map[loc] == 0) 
                {
                    gap_start = x;
                }
            }
        }
    }

}


void MAPFPlanner::hierarchyMapWaypointsStart(int grid)
{
    for(int i = 0; i < env->map.size(); i++)
    {
        unordered_map<int,int> locationDistanceMap;
        reducedMapWaypoints.push_back(locationDistanceMap);
    }

    for(int top_left_x = 0; top_left_x < env->rows; top_left_x = top_left_x + grid)
    {
        for(int top_left_y = 0; top_left_y < env->cols; top_left_y = top_left_y + grid)
        // for each cell defined by (top_left_x,top_left_y)
        {
            // entry from the left
            for(int x = min(top_left_x + grid,env->rows-1); x > top_left_x; x--)
            {
                int loc_from =  x * env->cols + top_left_y;
                if (reducedMap[loc_from] == 1)
                {
                    // exit on the top
                    for(int y_top = top_left_y; y_top < min(top_left_y + grid,env->cols) ; y_top++)
                    {
                        int loc_to =  top_left_x * env->cols + y_top;
                        if (reducedMap[loc_to] == 1)
                        {
                            int distance = hierarchy_single_agent_plan(loc_from,EAST,loc_to,top_left_x,top_left_y,grid);
                            if (distance>0)
                            {
                                reducedMapWaypoints[loc_from][loc_to] = distance;
                                reducedMapWaypoints[loc_to][loc_from] = distance;    // TODO this my not be exact because of the direction !!!!
                            }
                        }
                    }
                    // exit on the right
                    for(int x_right = top_left_x; x_right < min(top_left_x + grid,env->rows); x_right++)
                    {
                        int loc_to =  x_right * env->cols + top_left_y + grid;
                        if (reducedMap[loc_to] == 1)
                        {
                            int distance = hierarchy_single_agent_plan(loc_from,EAST,loc_to,top_left_x,top_left_y,grid);
                            if (distance>0)
                            {
                                reducedMapWaypoints[loc_from][loc_to] = distance;
                                reducedMapWaypoints[loc_to][loc_from] = distance;    // TODO this my not be exact because of the direction!!!!
                            }
                        }
                    }
                    // exit at the bottom
                    for(int y_bottom = min(top_left_y + grid,env->cols-1); y_bottom > top_left_y; y_bottom--)
                    {
                        int loc_to =  (top_left_x + grid) * env->cols + y_bottom;
                        if (reducedMap[loc_to] == 1)
                        {
                            int distance = hierarchy_single_agent_plan(loc_from,EAST,loc_to,top_left_x,top_left_y,grid);
                            if (distance>0)
                            {
                                reducedMapWaypoints[loc_from][loc_to] = distance;
                                reducedMapWaypoints[loc_to][loc_from] = distance;    // TODO this my not be exact because of the direction !!!!
                            }
                        }
                    }
                }
            }
            // entry from the top
            for(int y = top_left_y; y < min(top_left_y + grid,env->cols) ; y++)
            {
                int loc_from =  top_left_x * env->cols + y;
                if (reducedMap[loc_from] == 1)
                {
                    // exit on the right
                    for(int x_right = top_left_x; x_right < min(top_left_x + grid,env->rows); x_right++)
                    {
                        int loc_to =  x_right * env->cols + top_left_y + grid;
                        if (reducedMap[loc_to] == 1)
                        {
                            int distance = hierarchy_single_agent_plan(loc_from,SOUTH,loc_to,top_left_x,top_left_y,grid);
                            if (distance>0)
                            {
                                reducedMapWaypoints[loc_from][loc_to] = distance;
                                reducedMapWaypoints[loc_to][loc_from] = distance;    // TODO this my not be exact because of the direction!!!!
                            }
                        }
                    }
                    // exit at the bottom
                    for(int y_bottom = min(top_left_y + grid,env->cols-1); y_bottom > top_left_y; y_bottom--)
                    {
                        int loc_to =  (top_left_x + grid) * env->cols + y_bottom;
                        if (reducedMap[loc_to] == 1)
                        {
                            int distance = hierarchy_single_agent_plan(loc_from,SOUTH,loc_to,top_left_x,top_left_y,grid);
                            if (distance>0)
                            {
                                reducedMapWaypoints[loc_from][loc_to] = distance;
                                reducedMapWaypoints[loc_to][loc_from] = distance;    // TODO this my not be exact because of the direction !!!!
                            }
                        }
                    }
                }
            }
            // entry from the right
            for(int x = top_left_x; x < min(top_left_x + grid,env->rows); x++)
            {
                int loc_from =  x * env->cols + top_left_y + grid;
                if (reducedMap[loc_from] == 1)
                {
                    // exit at the bottom
                    for(int y_bottom = min(top_left_y + grid,env->cols-1); y_bottom > top_left_y; y_bottom--)
                    {
                        int loc_to =  (top_left_x + grid) * env->cols + y_bottom;
                        if (reducedMap[loc_to] == 1)
                        {
                            int distance = hierarchy_single_agent_plan(loc_from,WEST,loc_to,top_left_x,top_left_y,grid);
                            if (distance>0)
                            {
                                reducedMapWaypoints[loc_from][loc_to] = distance;
                                reducedMapWaypoints[loc_to][loc_from] = distance;    // TODO this my not be exact because of the direction !!!!
                            }
                        }
                    }
                }
            }
        }
    }
}


int MAPFPlanner::hierarchy_single_agent_plan(int start,int start_direct,int end, int top_left_x, int top_left_y, int grid)  // returns the distance or -1 if there is no path
{
    int distance = -1;
    int bottom_right_x = min(top_left_x + grid,env->rows-1);
    int bottom_right_y = min(top_left_y + grid,env->cols-1);
    list<pair<int,int>> path;
    priority_queue<AstarNode*,vector<AstarNode*>,cmp> open_list;
    unordered_map<int,AstarNode*> all_nodes;
    unordered_set<int> close_list;
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;

    while (!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);
        if (curr->location == end)
        {
            distance = curr->g;
            // // TOREMOVE BEGIN
            // while(curr->parent!=NULL) 
            // {
            //     l0reducedMap[curr->location] = 2;
            //     curr = curr->parent;
            // }
            // // TOREMOVE END
            break;
        }
        list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction);
        for (const pair<int,int>& neighbor: neighbors)
        {
            int neighbor_x = neighbor.first/env->cols;
            int neighbor_y = neighbor.first%env->cols;
            if (neighbor_x < top_left_x || neighbor_y < top_left_y || neighbor_x > bottom_right_x || neighbor_y > bottom_right_y)
                continue;
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
                continue;
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end())
            {
                AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                if (curr->g + 1 < old->g)
                {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            else
            {
                AstarNode* next_node = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(neighbor.first,end), curr);
                open_list.push(next_node);
                all_nodes[neighbor.first*4+neighbor.second] = next_node;
            }
        }
    }
    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    return distance;
}


void MAPFPlanner::reduceMapStart()
{
    int distance = 0;
    //From left to right, top to bottom
    for(int x = 0; x < env->rows;x++)
    {
        distance = 0;
        for(int y = 0; y < env->cols;y++)
        {
            int loc = x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : distance + 1; 
            if (distance > WAYPOINT_STEP && x%WAYPOINT_STEP == 0 && y%WAYPOINT_STEP == 0) distance = 0;
            reducedMap.push_back(distance);
        }
    }

    //From right to left, bottom to top
    for(int x = env->rows-1; x >= 0; x--)
    {
        distance = 0;
        for(int y = env->cols-1; y >= 0; y--)
        {
            int loc =  x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : min(distance + 1, reducedMap[loc]);
            if (distance > WAYPOINT_STEP && x%WAYPOINT_STEP == 0 && y%WAYPOINT_STEP == 0) distance = 0;
            reducedMap[loc] = distance;
        }
    }

    //From top to bottom, left to right
    for(int y = 0; y < env->cols; y++)
    {
        distance = 0;
        for(int x = 0; x < env->rows; x++)
        {
            int loc =  x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : min(distance + 1, reducedMap[loc]);
            if (distance > WAYPOINT_STEP && x%WAYPOINT_STEP == 0 && y%WAYPOINT_STEP == 0) distance = 0;
            reducedMap[loc] = distance;
        }
    }

    //From bottom to top, right to left
    for(int y = env->cols-1; y >= 0;y--)
    {
        distance = 0;
        for(int x = env->rows-1; x >= 0; x--)
        {
            int loc = x * env->cols + y;
            distance = env->map[loc] == 1 ? 0 : min(distance + 1, reducedMap[loc]);
            if (distance > WAYPOINT_STEP && x%WAYPOINT_STEP == 0 && y%WAYPOINT_STEP == 0) distance = 0;
            reducedMap[loc] = distance;
        }
    }
}


void MAPFPlanner::reduceMapUpdate()
{
    int distance = 0;
    //From left to right, top to bottom
    for(int x = 0; x < env->rows; x++)
    {
        distance = 0;
        for(int y = 0; y < env->cols; y++)
        {
            int loc = x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }

    //From right to left, bottom to top
    for(int x = env->rows-1; x >= 0; x--)
    {
        distance = 0;
        for(int y = env->cols-1; y >= 0; y--)
        {
            int loc =  x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }

    //From top to bottom, left to right
    for(int y = 0; y < env->cols; y++)
    {
        distance = 0;
        for(int x = 0; x < env->rows; x++)
        {
            int loc =  x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }

    //From bottom to top, right to left
    for(int y = env->cols-1; y >= 0;y--)
    {
        distance = 0;
        for(int x = env->rows-1; x >= 0; x--)
        {
            int loc = x * env->cols + y;
            distance = reducedMap[loc] == 0 ? 0 : min(distance + 1, reducedMap[loc]);
            reducedMap[loc] = distance;
        }
    }
}

bool MAPFPlanner::reduceMap(bool keepSalient)
{
    bool changed = false;
    if(keepSalient)
    {
        for(int i = 0; i < reducedMap.size(); i++)
        {
            int currentDistance = reducedMap[i];
            if(currentDistance == 1 && areNeighboursTraversable(i) && ((countReducedVerticalNeighbours(i) + countReducedHorizontalNeighbours(i)) > 1))
            {
                changed = true;
                reducedMap[i] = 0;
            }
        }
    }
    else
    {
        vector<int> tmpReducedMap = reducedMap;
        for(int i = 0; i < reducedMap.size(); i++)
        {   
            int currentDistance = reducedMap[i];
            if(currentDistance == 1 && (countReducedVerticalNeighbours(i) + countReducedHorizontalNeighbours(i)) == 1)
            {
                changed = true;
                tmpReducedMap[i] = 0;
            }
        }
        reducedMap = tmpReducedMap;
    }

    reduceMapUpdate();
    return changed;
}


void MAPFPlanner::reduceMapWaypointsStart()
{
    for(int i = 0; i < env->map.size(); i++)
    {
        unordered_map<int,int> locationDistanceMap;
        reducedMapWaypoints.push_back(locationDistanceMap);
        if(reducedMap[i] > 0)
        {
            int locY = i % env->cols;
            if(i >= env->cols && reducedMap[i-env->cols] > 0)
            {
                int n = i-env->cols;
                reducedMapWaypoints[i][n] = 1;
            }
            if(locY + 1 < env->cols && reducedMap[i+1] > 0)
            {
                int e = i+1;
                reducedMapWaypoints[i][e] = 1;
            }
            if(i+env->cols < env->map.size() && reducedMap[i+env->cols] > 0)
            {
                int s = i+env->cols;
                reducedMapWaypoints[i][s] = 1;
            }
            if(locY > 0 && reducedMap[i-1] > 0)
            {
                int w = i-1;
                reducedMapWaypoints[i][w] = 1;
            }
        }
    }
}


bool MAPFPlanner::reduceReduceMapWaypoints(int distance)
{
    bool changed = false;
    for(int i = 0; i < env->map.size();i++)
    {
        unordered_map<int,int> neighbours = reducedMapWaypoints[i];
        if(reducedMap[i] > 0 && neighbours.size() == 2)
        {
            vector<int> keys;
            for(auto kv : neighbours)
            {
                keys.push_back(kv.first);
            }
            int n0 = keys[0];
            int n1 = keys[1];
            int newCost = 999999;
            if (reducedMapWaypoints[n0].find(n1) == reducedMapWaypoints[n0].end())
            {
                int locToN1Cost = reducedMapWaypoints[i][n1];
                int n0ToLocCost = reducedMapWaypoints[n0][i];
                newCost = n0ToLocCost + locToN1Cost;
                if(newCost == 2 && (countReducedVerticalNeighbours(i) != 2) && (countReducedHorizontalNeighbours(i) != 2))
                {
                    newCost++;
                }
                if(newCost < distance)
                {
                    reducedMap[i] = 0;
                    reducedMapWaypoints[i].clear();
                    for (auto it = reducedMapWaypoints[n0].begin(); it != reducedMapWaypoints[n0].end();)
                    {
                        if (it->first == i)
                            it = reducedMapWaypoints[n0].erase(it);
                        else
                            ++it;
                    }
                    reducedMapWaypoints[n0][n1] = newCost;

                    for (auto it = reducedMapWaypoints[n1].begin(); it != reducedMapWaypoints[n1].end();)
                    {
                        if (it->first == i)
                            it = reducedMapWaypoints[n1].erase(it);
                        else
                            ++it;
                    }
                    reducedMapWaypoints[n1][n0] = newCost;
                    changed = true;
                }
            }
        }
    }
    return changed;
}


bool MAPFPlanner::areNeighboursTraversable(int location)
{
    int x = location/env->cols;
    int y = location%env->cols;
    int horizontalNeighbourCount = countReducedHorizontalNeighbours(location);
    int verticalNeighbourCount = countReducedVerticalNeighbours(location);
    //Bridge Node
    if((verticalNeighbourCount == 0 && horizontalNeighbourCount == 2) || 
       (verticalNeighbourCount == 2 && horizontalNeighbourCount == 0))
    {
        return false;
    }
    //North + East + NorthEast
    if((location >= env->cols && reducedMap[location-env->cols] > 0) && 
       (y + 1 < env->cols && reducedMap[location+1] > 0) &&
       isNorthEastEmpty(location, x, y))
    {
        return false;
    }
    //South + East + SouthEast
    if((location+env->cols < env->map.size() && reducedMap[location+env->cols] > 0) && 
       (y + 1 < env->cols && reducedMap[location+1] > 0) &&
       isSouthEastEmpty(location, x, y))
    {
        return false;
    }
    //South + West + SouthWest
    if((location+env->cols < env->map.size() && reducedMap[location+env->cols] > 0) && 
       (y > 0 && reducedMap[location-1] > 0) &&
       isSouthWestEmpty(location, x, y))
    {
        return false;
    }
    //North + West + NorthWest
    if((location >= env->cols && reducedMap[location-env->cols] > 0) && 
       (y > 0 && reducedMap[location-1] > 0) &&
       isNorthWestEmpty(location, x, y))
    {
        return false;
    }
    return true;
}

bool MAPFPlanner::isNorthEastEmpty(int location, int x, int y)
{
    if(x > 0 && y + 1 < env->cols && reducedMap[location-env->cols+1] > 0){return false;}
    return true;
}
bool MAPFPlanner::isSouthEastEmpty(int location, int x, int y)
{
    if(x + 1 < env->rows && y + 1 < env->cols && reducedMap[location+env->cols+1] > 0){return false;}
    return true;
}
bool MAPFPlanner::isNorthWestEmpty(int location, int x, int y)
{
    if(x > 0 && y > 0 && reducedMap[location-env->cols-1] > 0){return false;}
    return true;
}
bool MAPFPlanner::isSouthWestEmpty(int location, int x, int y)
{
    if(x + 1 < env->rows && y + 1 > 0 && reducedMap[location+env->cols-1] > 0){return false;}
    return true;
}

int MAPFPlanner::countReducedVerticalNeighbours(int location)
{
    int count = 0;
    if(location >= env->cols && reducedMap[location-env->cols] > 0) {count++;}
    if(location+env->cols < env->map.size() && reducedMap[location+env->cols] > 0) {count++;}
    return count;
}

int MAPFPlanner::countReducedHorizontalNeighbours(int location)
{
    int count = 0;
    int y = location % env->cols;
    if(y > 0 && reducedMap[location-1] > 0) {count++;}
    if(y+1 < env->cols && reducedMap[location+1] > 0) {count++;}
    return count;
}


void MAPFPlanner::printReducedMap()
{
    // string blackBox = "■" ;
    string blackBox = "*" ;
    string whiteBox = "□" ;

    for(int x = 0; x < env->rows;x++)
    {
        string temp = "";
        for(int y = MAP_PRINT_OFFSET; y < min(env->cols, MAP_PRINT_WIDTH+MAP_PRINT_OFFSET) ;y++)
        {
            int loc = x * env->cols + y;
            if(reducedMap[loc] == 0)
            {
                temp += env->map[loc] == 1 ? blackBox : " ";
            }
            else 
            {
                temp += std::to_string(reducedMap[loc]);
            }
            //temp += reducedMap[loc] == 0 ? blackBox : whiteBox;
            //temp += reducedMap[loc] == 0 ? blackBox : " ";
        }
        cout << temp << endl;
    }
    cout << endl << endl;
}


list<int> MAPFPlanner::getPlainNeighbors(int location)
{
    list<int> neighbors;
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    for(int i = 0; i < 4; i++)
    {
        int forward = candidates[i];
        if(forward >= 0 && forward < env->map.size() && validateMove(forward, location))
        {
            neighbors.emplace_back(forward);
        }
    }
    return neighbors;
}

list<int> MAPFPlanner::getPlainNeighborsOnReducedMap(int location)
{
    list<int> neighbors;
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    for(int i = 0; i < 4; i++)
    {
        int forward = candidates[i];
        if(forward >= 0 && forward < env->map.size() && validateMoveOnReducedMap(forward, location))
        {
            neighbors.emplace_back(forward);
        }
    }
    return neighbors;
}


bool MAPFPlanner::validateMoveOnReducedMap(int loc1, int loc2)
{
    int loc1X = loc1 / env->cols;
    int loc1Y = loc1 % env->cols;

    if (loc1X < 0 || loc1X >= env->rows || loc1Y < 0 || loc1Y > env->cols || reducedMap[loc1] == 0)
    {
        return false;
    }

    int loc2X = loc2 / env->cols;
    int loc2Y = loc2 % env->cols;

    if (abs(loc1X-loc2X) + abs(loc1Y-loc2Y) > 1)
        return false;
    return true;
}




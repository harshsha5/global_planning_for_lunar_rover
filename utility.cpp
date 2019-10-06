//
// Created by Harsh Sharma on 06/10/19.
//

#include "utility.h"
#include "planning_map.h"
#include "node.h"
#include <iostream>
#include <queue>
#include <unordered_set>
#include "map_width_header.h"

int GLOBAL_MAP_WIDTH;

//=====================================================================================================================

bool is_destination(const coordinate &c, const coordinate &goal)
{
    if(c==goal)
        return true;

    return false;
}

//=====================================================================================================================

void expand_state(const Node &node_to_expand,
                  priority_queue<Node, vector<Node>, Comp> &open,
                  vector<vector<Node>> &node_map,
                  const vector<int> &dX,
                  const vector<int> &dY,
                  const planning_map &elevation_map,
                  unordered_set<Node,node_hasher> &closed)
{
    const auto current_x = node_to_expand.c.x;
    const auto current_y = node_to_expand.c.y;
    const int cost_per_step = 1;
    for(size_t dir = 0; dir < dX.size(); dir++)
    {
        int newx = current_x + dX[dir];
        int newy = current_y + dY[dir];
        coordinate new_coordinate {newx,newy};

        if (elevation_map.is_valid(new_coordinate))
        {
            if(!closed.count(node_map[newx][newy]) && (node_map[newx][newy].gcost > node_map[current_x][current_y].gcost + cost_per_step))
            {
                node_map[newx][newy].set_gcost(node_map[current_x][current_y].gcost + cost_per_step);
                node_map[newx][newy].set_parent(node_to_expand.c);
                open.push(node_map[newx][newy]);
            }
        }
    }
}

//=====================================================================================================================

vector<coordinate> backtrack(  const Node &start_node,
                                const Node &goal_node,
                                vector<vector<Node>> &node_map)
{
    vector<coordinate> path;
    Node curr_node = goal_node;
    while(curr_node!=start_node)
    {
        path.emplace_back(curr_node.c);
        curr_node = node_map[curr_node.parent.x][curr_node.parent.y];
    }
    std::reverse(path.begin(),path.end());
    return std::move(path);
}

//=====================================================================================================================

vector<coordinate> astar(const coordinate &start,const coordinate &goal,const planning_map &elevation_map){

    vector<coordinate> path{};
    if (!elevation_map.is_valid(goal)) {
        std::cout << "Destination is an obstacle" << std::endl;
        return path;
        //Destination is invalid
    }

    if (is_destination(start, goal)) {
        std::cout << "Already at the destination" << std::endl;
        return path;
    }
    GLOBAL_MAP_WIDTH =  elevation_map.map[0].size();
    const vector<int> dX = {-1, -1, -1,  0,  0,  1, 1, 1};
    const vector<int> dY {-1,  0,  1, -1,  1, -1, 0, 1};

    const Node random_init_node{coordinate{-1,-1},coordinate{-1,-1},INT_MAX,INT_MAX};
    vector<vector<Node>> node_map(elevation_map.map.size(),vector<Node> (elevation_map.map[0].size(),random_init_node));
    for(int i=0;i<elevation_map.map.size();i++)
    {
        for(int j=0;j<elevation_map.map[0].size();j++)
        {
            node_map[i][j].c = coordinate{i,j};
            node_map[i][j].set_hcost(node_map[i][j].calculate_hcost(goal));
        }
    }

    priority_queue<Node, vector<Node>, Comp> open;
    unordered_set<Node,node_hasher> closed;

    Node start_node(start,coordinate{-1,-1},0,INT_MAX);
    start_node.set_hcost(start_node.calculate_hcost(goal));
    Node goal_node(goal,coordinate{-1,-1},INT_MAX,0);
    node_map[start_node.c.x][start_node.c.y] = start_node;
    node_map[goal_node.c.x][goal_node.c.y] = goal_node;

    open.push(start_node);
    while (!open.empty() && !closed.count(goal_node))
    {
        const auto node_to_expand = open.top();
        open.pop();
        if(closed.count(node_to_expand)==0)       //Added this new condition to avoid multiple expansion of the same state
        {
            closed.insert(node_to_expand);
            expand_state(node_to_expand,open,node_map,dX,dY,elevation_map,closed);
        }
    }
    path = backtrack(node_map[start_node.c.x][start_node.c.y],node_map[goal_node.c.x][goal_node.c.y],node_map);
    return std::move(path);
}

//=====================================================================================================================
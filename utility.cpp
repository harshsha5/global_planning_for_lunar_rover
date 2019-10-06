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
    const auto m_width = elevation_map.map[0].size();
    const Node random_init_node{coordinate{-1,-1},coordinate{-1,-1},INT_MAX,INT_MAX};
    GLOBAL_MAP_WIDTH =  m_width;
    vector<vector<Node>> plan_map(elevation_map.map.size(),vector<Node> (elevation_map.map[0].size(),random_init_node));
    priority_queue<Node, vector<Node>, Comp> open;
    unordered_set<Node,node_hasher> closed;
    Node start_node(start,coordinate{-1,-1},0,INT_MAX);
    start_node.set_hcost(start_node.calculate_hcost(goal));
    start_node.print_node();
    return path;
}
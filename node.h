//
// Created by Harsh Sharma on 06/10/19.
//

#pragma once

#include "coordinate.h"

struct Node
{
    coordinate c;
    coordinate parent;
    double gcost;
    double hcost;
    double fcost;

    Node(coordinate c1,
         coordinate c_parent,
         double g_cost,
         double h_cost):
         c(c1),parent(c_parent),gcost(g_cost),hcost(h_cost){
         fcost = calculate_fcost();
    }

    double calculate_fcost();
    double calculate_h_cost(const coordinate &goal_coordinate);

};

//=====================================================================================================================

inline bool operator < (const Node& lhs, const Node& rhs)
{
    return lhs.fcost < rhs.fcost;
}

//=====================================================================================================================

//
// Created by Harsh Sharma on 06/10/19.
//

#include "node.h"
#include <cmath>

double Node::calculate_fcost()
{
    return gcost + hcost;
}

//=====================================================================================================================

double Node::calculate_h_cost(const coordinate &goal_coordinate)
{
    return (sqrt((this->c.x - goal_coordinate.x)*(this->c.x - goal_coordinate.x)
                             + (this->c.y - goal_coordinate.y)*(this->c.y - goal_coordinate.y)));
}

//=====================================================================================================================
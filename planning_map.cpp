//
// Created by Harsh Sharma on 06/10/19.
//

#include "planning_map.h"

using namespace std;

bool planning_map::is_valid(const coordinate &c) const
{
    if(c.x<0 || c.y<0 || c.x>=map.size() || c.y>=map[0].size() || map[c.x][c.y]>threshold_elevation_max || map[c.x][c.y]<threshold_elevation_min)
        return false;

    return true;
}

//=====================================================================================================================

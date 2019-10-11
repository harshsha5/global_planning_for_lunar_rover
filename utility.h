//
// Created by Harsh Sharma on 06/10/19.
//

#pragma once

#include "coordinate.h"
#include "planning_map.h"
#include <climits>

bool is_destination(const coordinate &c, const coordinate &goal);

vector<coordinate> astar(const coordinate &start,const coordinate &goal,const planning_map &elevation_map);

vector<vector<double>> convert_csv_to_vector(const string &file_name);
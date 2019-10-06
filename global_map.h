//
// Created by Harsh Sharma on 05/10/19.
//

#pragma once

#include <vector>
#include "coordinate.h"

class global_map
{
    int rows;
    int cols;
    int maximum_elevation;
    int minimum_elevation;

public:
    vector<vector<double>> g_map;
    vector<coordinate> way_points;

    global_map(int n_rows,
               int n_col,
               int max_height,
               int min_height
    ) : rows(n_rows),cols(n_col),maximum_elevation(max_height),minimum_elevation(min_height){
        vector<vector<double>> temp(rows,vector<double>(cols,0));
        g_map = std::move(temp);
        generate_map();
    }

    void generate_map();
    vector<pair<int,int>> get_pit_boundary_coordinates();
    vector<coordinate> get_pit_interior_coordinates();
    vector<pair<int,int>> get_pit_bbox_coordinates();
    void make_pit_in_map();

    template <class T>
    void display_vector(const vector<vector<T>> &vec);

    void display_final_map();
    int get_maximum_elevation();
    int get_minimum_elevation();
};



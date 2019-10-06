//
// Created by Harsh Sharma on 05/10/19.
//

#pragma once

#include <iostream>

using namespace std;

struct coordinate
{
    int x;
    int y;
    coordinate(int x_coord,
               int y_coord):
            x(x_coord),y(y_coord){}

    void print_coordinate() const
    {
        cout<<"x: "<<x<<"\t"<<"y: "<<y<<endl;
    }

    friend bool operator== (const coordinate &lhs, const coordinate &rhs);
    friend bool operator!= (const coordinate &lhs, const coordinate &rhs);
};

//=====================================================================================================================

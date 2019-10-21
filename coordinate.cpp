//
// Created by Harsh Sharma on 05/10/19.
//

#include "coordinate.h"
#include <cmath>

bool operator==(const coordinate& lhs, const coordinate& rhs)
{
    return ((lhs.x==rhs.x)&&(lhs.y==rhs.y));
}

bool operator!=(const coordinate& lhs, const coordinate& rhs)
{
    return !(lhs==rhs);
}

//=====================================================================================================================

double coordinate::get_euclidian_distance(const coordinate &c) const
{
    return (sqrt((this->x - c.x)*(this->x - c.x)
                 + (this->y - c.y)*(this->y - c.y)));
}

//=====================================================================================================================


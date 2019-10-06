//
// Created by Harsh Sharma on 05/10/19.
//

#include "coordinate.h"

bool operator==(const coordinate& lhs, const coordinate& rhs)
{
    return ((lhs.x==rhs.x)&&(lhs.y==rhs.y));
}

bool operator!=(const coordinate& lhs, const coordinate& rhs)
{
    return !(lhs==rhs);
}

//=====================================================================================================================


//
// Created by Harsh Sharma on 28/10/19.
//

#pragma once

struct multi_goal_A_star_configuration
{
    double pessimistic_factor;
    double time_difference_weight;
    explicit multi_goal_A_star_configuration(double pessismistic_time_factor = 0.5,double weight_time_difference = 2):
            pessimistic_factor(pessismistic_time_factor),time_difference_weight(weight_time_difference){}

};

//=====================================================================================================================

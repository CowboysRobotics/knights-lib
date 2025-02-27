#pragma once

#ifndef _END_CONDITION_H
#define _END_CONDITION_H

#include "knights/util/position.hpp"

namespace knights {

    struct LinearEndToleranceCondition {
        float distance;

        bool check(knights::Pos curr, knights::Pos end);
    };

    struct LinearPassedCondition {
        bool check(knights::Pos curr, knights::Pos end, knights::Pos start);
    };

    struct AngularEndToleranceCondition {
        float error;

        bool check(float curr_angle, float end_angle);
    };

    struct PursuitEndToleranceCondition {
        float distance;

        bool check(knights::Pos curr, knights::Pos end);
    };

    struct TimeoutCondition {
        float timeout;

        bool check(float curr_time);
    };

    struct CustomCondition {

    };
    

}

#endif
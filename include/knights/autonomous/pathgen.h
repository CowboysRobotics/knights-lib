#pragma once

#ifndef _PATHGEN_H
#define _PATHGEN_H

#include "knights/autonomous/path.h"
#include "knights/util/position.h"

namespace knights {

    /**
     * @brief Generate a route to follow to get to a provided position
     * 
     * @param start Start point of the route, usually current position of the bot
     * @param end End point of the route, make sure to add a heading value
     * @param MAX_VELOCITY Max velocity for the route
     * @param MAX_ACCELERATION Max acceleration for the route
     * @param MAX_JERK Max jerk for the route
     * @return Route containing position to get to target
     */
    Route generate_path_to_pos(knights::Pos start, knights::Pos end, const float MAX_VELOCITY, const float MAX_ACCELERATION, const float MAX_JERK, const float DRIVETRAIN_WIDTH);

    /**
     * @brief Generate a motion profile to follow to get to a provided position
     * 
     * @param start Start point of the route, usually current position of the bot
     * @param end End point of the route, make sure to add a heading value
     * @param MAX_VELOCITY Max velocity for the route
     * @param MAX_ACCELERATION Max acceleration for the route
     * @param MAX_JERK Max jerk for the route
     * @return Motion profile containing position to get to target
     */
    std::vector<squiggles::ProfilePoint> generate_motion_profile(knights::Pos start, knights::Pos end, const float MAX_VELOCITY, const float MAX_ACCELERATION, const float MAX_JERK, const float DRIVETRAIN_WIDTH);
}

#endif
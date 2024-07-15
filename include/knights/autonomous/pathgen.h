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
     * @param NUM_PTS Amount of points to generate on the route
     * @return Route containing position to get to target
     */
    Route generate_path_to_pos(Pos start, Pos end, const int NUM_PTS = 10);

}

#endif
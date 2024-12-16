#pragma once

#ifndef _PATHGEN_H
#define _PATHGEN_H

#include "knights/autonomous/path.h"
#include "knights/util/position.h"

namespace knights {

    /**
     * @brief Generate a path between two points using a 
     * 
     * @param start Position (x,y,heading) to start at
     * @param end Position (x,y,heading) to end at
     * @return Route with a path between the points
     */
    Route generate_path_to_pos(knights::Pos start, knights::Pos end);

}

#endif
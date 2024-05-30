#pragma once

#ifndef _PATH_H
#define _PATH_H

#include <string>
#include <vector>

#include "knights/util/position.h"

namespace knights {

    struct Route {
        std::vector<Pos> positions;

        /**
         * @brief Construct a new Route object
         * 
         * @param positions A vector of positions for the route, this can be read off the SD card or inputted manually
         */
        Route(std::vector<Pos> positions);

        /**
         * @brief Construct a new Route object with an empty position vector
         * 
         */
        Route();
    };

    /**
     * @brief Read a route from an SD card file
     * 
     * @param route_name The name and extension of the file to look for (ex. "file.txt")
     */
    void init_route_from_sd(std::string route_name);
}

#endif
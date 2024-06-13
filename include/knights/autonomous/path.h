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
         * @param positions A vector of positions for the route, this can be read out of a file or inputted manually
         */
        Route(std::vector<Pos> positions);

        /**
         * @brief Construct a new Route object with an empty position vector
         * 
         */
        Route();

    };

    /**
     * @brief Append one route to another
     * 
     * @param r1 the route to append to
     * @param r2 the route to append
     * @return Route 
     */
    Route operator+(const Route &r1, const Route &r2);

    /**
     * @brief Add a position to a route
     * 
     * @param r1 the route to add to
     * @param p1 the position to add
     * @return Route 
     */
    Route operator+(Route r1, const Pos &p1);

    /**
     * @brief Remove a number of positions from the end of a route
     * 
     * @param r1 the route to remove positions from
     * @param amt the amount of poitions to remove
     * @return Route 
     */
    Route operator-(Route r1, const int &amt);

    /**
     * @brief Read a route from an SD card file
     * 
     * @param route_name The name and extension of the file to look for (ex. "file.txt")
     */
    Route init_route_from_sd(std::string route_name);
}

#endif
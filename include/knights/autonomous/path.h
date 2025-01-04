#pragma once

#ifndef _PATHK_H
#define _PATHK_H

#include <string>
#include <vector>
#include <map>

#include "knights/util/position.h"
#include "knights/driver/input.h"
#include "knights/robot/chassis.h"
#include "knights/autonomous/pid.h"

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

        float length_dist();

    };

    enum action_type {
        LATERAL,
        TURN,
        FOLLOW,
        COMMAND
    };

    struct RouteAction { 
       action_type type;  
       std::string route_name = "none";
       float specific;
       float end_tolerance;
       int timeout;
       float lookahead;
       std::string function_name;

        /**
         * @brief Construct a new Route Action object - presumed with follow type
         * 
         * @param type action type for the route (lateral, turn, follow, command)
         * @param route_name name of route to follow - only used if it is a follow
         * @param end_tolerance end tolerance for movement - only used if it is a lateral, turn, or follow
         * @param timeout timeout for movement - only used if it is a lateral, turn, or follow
         * @param lookahead lookahead for pure pursuit - only used if it is a follow
         */
       RouteAction(action_type type, std::string route_name, float end_tolerance, int timeout, float lookahead);

        /**
         * @brief Construct a new Route Action object - presumed with lateral or turn type
         * 
         * @param type action type for the route (lateral, turn, follow, command)
         * @param specific type specific value for the route - distance for lateral, angle for turn
         * @param end_tolerance end tolerance for movement - only used if it is a lateral, turn, or follow
         * @param timeout timeout for movement - only used if it is a lateral, turn, or follow
         */
       RouteAction(action_type type, float specific, float end_tolerance, int timeout);

        /**
         * @brief Construct a new Route Action object - presumed with command type
         * 
         * @param type action type for the route (lateral, turn, follow, command)
         * @param function_name name of the function in the autonomous input map to run
         */
       RouteAction(action_type type, std::string function_name);
    };

    struct AdvancedRoute {
        std::map<std::string, Route> routes;
        std::vector<RouteAction> actions;

        /**
         * @brief Run an advanced route object
         * 
         * @param chassis Pointer to chassis object containing drivetrain that will be used
         * @param lateral_pid Pointer to lateral PID controller to use
         * @param turn_pid  Pointer to turn PID controller to use
         * @param input_map Autonomous Input Map to use
         */
        void execute(knights::RobotChassis *chassis, knights::PIDController *lateral_pid, knights::PIDController *turn_pid, knights::input::AutonomousInputMap *input_map);

        /**
         * @brief Construct a new Advanced Route object with given routes and action list
         * 
         * @param routes Map of routes in the movement; these will be used for the path following algorithm
         * @param actions Array of actions for the route
         */
        AdvancedRoute(std::map<std::string, Route> routes, std::vector<RouteAction> actions);

        /**
         * @brief Construct a new Advanced Route object with empty routes and empty actions
         * 
         */
        AdvancedRoute();
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

/**
 * @brief Read an advanced route from a file on the brain microSD card
 * 
 * @param file_name Name of the file to read from - DO NOT include the /usd/, this will automatically be added (ex: "autonomous.txt")
 * @return knights::AdvancedRoute 
 */
knights::AdvancedRoute advanced_route_from_file(std::string file_name);

#endif
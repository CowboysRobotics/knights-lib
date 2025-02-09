#pragma once

#ifndef _ADVROUTE_H
#define _ADVROUTE_H

#include <string>
#include <vector>
#include <map>

#include "knights/autonomous/controller.hpp"
#include "knights/util/position.hpp"
#include "knights/driver/input.hpp"
#include "knights/robot/chassis.hpp"
#include "knights/autonomous/pid.hpp"
#include "knights/asset.hpp"

namespace knights {
    enum action_type {
        LATERAL,
        TURN,
        FOLLOW,
        COMMAND,
        LATERAL_TO_POS
    };

    struct RouteAction { 
       action_type type;  
       std::string route_name = "none";
       float specific;
       float end_tolerance;
       int timeout;
       float lookahead;
       std::string function_name;
       knights::Pos move_to;

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

       RouteAction(action_type type, float x, float y, float theta, float end_tolerance, int timeout);

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
         * @param input_map Autonomous Input Map to use
         */
        void execute(knights::RobotChassis *chassis, knights::RobotController *robotControl, knights::input::AutonomousInputMap *input_map);

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
}

/**
 * @brief Read an advanced route from a file on the brain microSD card
 * 
 * @param file_name Name of the file to read from - DO NOT include the /usd/, this will automatically be added (ex: "autonomous.txt")
 * @return knights::AdvancedRoute 
 */
knights::AdvancedRoute advanced_route_from_file(std::string file_name);

knights::AdvancedRoute advanced_route_from_asset(AssetStream &buffer);


#endif
#pragma once

#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "knights/autonomous/pid.h"
#include "knights/autonomous/path.h"

#include "knights/robot/chassis.h"

namespace knights {

    class RobotController {
        private:
            PIDController *pid_controller;
            RobotChassis *chassis;
            bool use_motor_encoders = false;

            bool in_motion = false;
        public:
            /**
             * @brief Construct a new Robot Controller object
             * 
             * @param chassis 
             * @param pid_controller 
             * @param use_motor_encoders 
             */
            RobotController(RobotChassis *chassis, PIDController *pid_controller, bool use_motor_encoders = false);

            /**
             * @brief Follow a route that has been read into the 
             * 
             * @param route 
             * @param lookahead_distance 
             * @param max_speed 
             * @param forwards 
             * @param end_tolerance 
             * @param timeout 
             */
            void follow_route_pursuit(knights::Route &route, const float &lookahead_distance = 15.0, const float max_speed = 127.0, const bool forwards = true, const float end_tolerance = 8.0, float timeout = 5000);

            void follow_route_ramsete(knights::Route &route, float timeout = 6000);

            void move_to_point(const Pos point, const float &end_tolerance, float timeout = 3000, const float &num_pts = 10);

            void turn_to_angle(const float angle, const float end_tolerance = 3.0, float timeout = 2000, bool rad = false); // DEGREES

            void lateral_move(const float distance, float end_tolerance = 3.0, float timeout = 2000);

            void turn_for(const float angle, const float end_tolerance = 2.0, float timeout= 2000, bool rad = false); // DEGREES


    };
}

#endif
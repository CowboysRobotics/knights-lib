#pragma once

#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "knights/autonomous/pid.h"
#include "knights/autonomous/ramsete.h"
#include "knights/autonomous/path.h"

#include "knights/robot/chassis.h"

namespace knights {

    class RobotController {
        private:
            PIDController *pid_controller;
            RamseteConstants *ramsete_constants;
            RobotChassis *chassis;
            bool use_motor_encoders = false;

            bool in_motion = false;
        public:
            /**
             * @brief Construct a new Robot Controller object
             * 
             * @param chassis 
             * @param pid_controller 
             * @param ramsete_constants 
             * @param use_motor_encoders 
             */
            RobotController(RobotChassis *chassis, PIDController *pid_controller, RamseteConstants *ramsete_constants, bool use_motor_encoders = false);

            /**
             * @brief Construct a new Robot Controller object
             * 
             * @param chassis 
             * @param pid_controller 
             * @param use_motor_encoders 
             */
            RobotController(RobotChassis *chassis, PIDController *pid_controller, bool use_motor_encoders = false);

            /**
             * @brief Follow a route that has been read into the route memory of the robot
             * 
             * @param route Pointer to the route to follow
             * @param lookahead_distance Distance to look ahead on the route in order to obtain the target position
             * @param max_speed Maximum speed to move the bot at
             * @param forwards Whether the bot should follow with its front or back
             * @param end_tolerance Distance to end the loop at
             * @param timeout Amount of time to wait before ending the movement
             */
            void follow_route_pursuit(knights::Route &route, float lookahead_distance = 15.0, const float max_speed = 127.0, bool forwards = true, float end_tolerance = 8.0, float timeout = 5000, float use_pid = false);


            void move_to_point(const Pos desired_position, const bool forwards = true, const float &end_tolerance = 2.0, float timeout = 1000);

            /**
             * @brief Turn the robot to a specific angle
             * 
             * @param angle Angle to turn to
             * @param direction Whether to turn left (-1), right (1), or best direction (0)
             * @param end_tolerance Angle that the bot will stop moving at (ie if this is 5, the bot will stop moving 5 degrees before the heading) 
             *                      - this is used to account for the center of the bot not being the front
             * @param timeout Amount of time to wait before exiting the movement
             * @param rad Whether the provided angle is in radians or not
             */
            void turn_to_angle(const float angle, int direction,float end_tolerance = 3.0, float timeout = 2000, bool rad = false); // DEGREES

            /**
             * @brief Move in a straight line, forwards or backwards
             * 
             * @param distance Distance to move, positive for forward, negative for backward
             * @param end_tolerance Position that the bot will stop moving at (ie if this is 5, the bot will stop moving 5 inches before the position) 
             *                      - this is used to account for the center of the bot not being the front
             * @param timeout Amount of time to wait before exiting the move
             */
            void lateral_move(const float distance, float end_tolerance = 3.0, float timeout = 750);

            /**
             * @brief Turn the robot left or right for a certain angle
             * 
             * @param angle Amount to change the heading by, negative for right, positive for left (think of unit circle)
             * @param end_tolerance Angle that the bot will stop moving at (ie if this is 5, the bot will stop moving 5 degrees before the heading) 
             *                      - this is used to account for the center of the bot not being the front
             * @param timeout Amount of time to wait before exiting the movement
             * @param rad Whether the provided angle is in radians or not
             */
            void turn_for(const float angle, const float end_tolerance = 2.0, float timeout= 750, bool rad = false); // DEGREES


    };
}

#endif
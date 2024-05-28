#pragma once

#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "knights/autonomous/pid.h"
#include "knights/robot/chassis.h"

namespace knights {

    class Robot_Controller {
        private:
            PID_Controller *pid_controller;
            Robot_Chassis *chassis;
            bool use_motor_encoders = false;

            bool in_motion = false;
        public:

            Robot_Controller(Robot_Chassis *chassis, PID_Controller *pid_controller, bool use_motor_encoders = false);

            void follow_route(std::string route_name, const float &lookahead_distance = 15.0, const float max_speed = 127.0, const bool forwards = true, const float end_tolerance = 8.0, float timeout = 5000);

            void move_to_point(const Pos point, const float &end_tolerance, float timeout = 3000, const float &num_pts = 10);

            void turn_to_angle(const float angle, const float end_tolerance = 3.0, float timeout = 2000, bool rad = false); // DEGREES

            void lateral_move(const float distance, float end_tolerance = 3.0, float timeout = 2000);

            void turn_for(const float angle, const float end_tolerance = 2.0, float timeout= 2000, bool rad = false); // DEGREES


    };
}

#endif
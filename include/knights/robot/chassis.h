#pragma once

#ifndef _CHASSIS_H
#define _CHASSIS_H

#include "api.h"

#include "knights/robot/drivetrain.h"
#include "knights/robot/position_tracker.h"

#include "knights/util/position.h"


namespace knights {

    class Robot_Chassis {
        private:
            Drivetrain *drivetrain = nullptr; // the drivetrain to use for the chassis
            Holonomic *holonomic = nullptr; // the drivetrain to use for the chassis
            Position_Tracker_Group *pos_trackers = nullptr; // the sensors to use for location tracking
            Pos curr_position; // the current position of the robot
            Pos prev_position;

            // previous values of the sensors for odometry control
            float prevRight = 0;
            float prevLeft = 0;
            float prevFront = 0;
            float prevBack = 0;

            // declare the robot chassis class as a friend class, allows access into private objects
            friend class Robot_Controller;
        public:
            // @brief create a robot chassis with provided parameters
            // @param drivetrain the drivetrain to use for the chassis
            // @param pos_trackers the sensors to use for location tracking
            Robot_Chassis(Drivetrain *drivetrain, Position_Tracker_Group *pos_trackers);

            // @brief create a robot chassis with provided parameters
            // @param drivetrain the drivetrain to use for the chassis
            // @param pos_trackers the sensors to use for location tracking
            Robot_Chassis(Holonomic *drivetrain, Position_Tracker_Group *pos_trackers);

            // @brief set the position of the chassis
            // @param x the desired x
            // @param y the desired y
            // @param heading the desired heading
            void set_position(float x, float y, float heading);

            // @brief set the position of the chassis
            // @param position the position to set it to
            void set_position(Pos position);

            // @brief update the position of a chassis using its tracking system
            void update_position();

            // @brief Get the current position of the chassis
            Pos get_position();

            // @brief set the position of the chassis
            // @param x the desired x
            // @param y the desired y
            // @param heading the desired heading
            void set_prev_position(float x, float y, float heading);

            // @brief set the position of the chassis
            // @param position the position to set it to
            void set_prev_position(Pos position);

            // @brief Get the current position of the chassis
            Pos get_prev_position();

    };
}

#endif
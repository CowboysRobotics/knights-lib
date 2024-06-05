#pragma once

#ifndef _CHASSIS_H
#define _CHASSIS_H

#include "api.h"

#include "knights/robot/drivetrain.h"
#include "knights/robot/position_tracker.h"

#include "knights/util/position.h"


namespace knights {

    class RobotChassis {
        private:
            Drivetrain *drivetrain = nullptr; // the drivetrain to use for the chassis
            Holonomic *holonomic = nullptr; // the drivetrain to use for the chassis
            PositionTrackerGroup *pos_trackers = nullptr; // the sensors to use for location tracking
            Pos curr_position; // the current position of the robot
            Pos prev_position;

            // previous values of the sensors for odometry control
            float prevRight = 0;
            float prevLeft = 0;
            float prevFront = 0;
            float prevBack = 0;

            // declare the robot chassis class as a friend class, allows access into private objects
            friend class RobotController;
        public:
            /**
             * @brief Construct a new Robot Chassis object
             * 
             * @param drivetrain a pointer to the drivetrain to use for the chassis
             * @param pos_trackers a pointer to the sensors to use for location tracking
             */
            RobotChassis(Drivetrain *drivetrain, PositionTrackerGroup *pos_trackers);


            /**
             * @brief Construct a new Robot Chassis object
             * 
             * @param drivetrain a pointer to the drivetrain to use for the chassis
             * @param pos_trackers a pointer to the sensors to use for location tracking
             */
            RobotChassis(Holonomic *drivetrain, PositionTrackerGroup *pos_trackers);

            /**
             * @brief Set the position of the chassis
             * 
             * @param x desired X value
             * @param y desired Y value
             * @param heading desired heading value (in radians)
             */
            void set_position(float x, float y, float heading);

            /**
             * @brief Set the position of the chassis
             * 
             * @param position a position variable containing an x, y, and heading
             */
            void set_position(Pos position);

            /**
             * @brief Update the position of the chassis using its tracking method.
             *  
             *  Will not update it no tracking method is set up.
             * 
             */
            void update_position();

            /**
             * @brief Get the current position of the chassis
             * 
             * @return Pos The current position of the chassis
             */
            Pos get_position();


            /**
             * @brief Set the previous position of the chassis
             * 
             * @param x desired X value
             * @param y desired Y value
             * @param heading desired heading value (in radians)
             */
            void set_prev_position(float x, float y, float heading);

            /**
             * @brief Set the previous position of the chassis
             * 
             * @param position a position variable containing an x, y, and heading
             */
            void set_prev_position(Pos position);

            /**
             * @brief Get the prev position of the chassis
             * 
             * @return Pos the previous position of the chassis
             */
            Pos get_prev_position();

    };
}

#endif
#pragma once

#ifndef _POSITION_TRACKER_H
#define _POSITION_TRACKER_H

#include "api.h"

namespace knights {

    class PositionTracker {
        private:
            // the rotation sensor attached to the odom wheel
            pros::Rotation *rotation = NULL;

            // the triwire port sensor attached to the odom wheel
            pros::adi::Encoder *adi_encoder = NULL;

            // the motor that is for tracking
            pros::Motor *motor = NULL;

            // the diameter of the tracking wheel
            float wheel_diameter = 0;

            // the gear ratio of the tracking wheel
            float gear_ratio = 1;

            // the distance from the wheel to the tracking center
            float offset = 0;

            // direction of the odom wheel
            int direction = 1;
        public:
            /**
             * @brief Construct a new position tracking wheel
             * 
             * @param rotation rotation sensor for the wheel
             * @param wheel_diameter diameter of the wheel
             * @param gear_ratio gear ratio of the wheel
             * @param offset vertical or horizontal distance from the tracking wheel to the trackign center
             */
            PositionTracker(pros::Rotation *rotation, float wheel_diameter, float gear_ratio, float offset, int direction = 1);

            /**
             * @brief Construct a new position tracking wheel
             * 
             * @param adi_encoder rotation encoder for the wheel
             * @param wheel_diameter diameter of the wheel
             * @param gear_ratio gear ratio of the wheel
             * @param offset vertical or horizontal distance from the tracking wheel to the trackign center
             */
            PositionTracker(pros::adi::Encoder *adi_encoder, float wheel_diameter, float gear_ratio, float offset, int direction = 1);

            /**
             * @brief Construct a new position tracking wheel
             * 
             * @param motor motor for the wheel
             * @param wheel_diameter diameter of the wheel
             * @param gear_ratio gear ratio of the wheel
             * @param offset vertical or horizontal distance from the tracking wheel to the trackign center
             */
            PositionTracker(pros::Motor *motor, float wheel_diameter, float gear_ratio, float offset, int direction = 1);

            /**
             * @brief Get the total distance the wheel has travelled
             * 
             * @return The position converted to inches
             */
            float get_distance_travelled();

            /**
             * @brief Get the offset of the wheel to the tracking center
             * 
             * @return The distance in inches
             */
            float get_offset();

            /**
             * @brief Reset the value of the attached encoder to 0
             * 
             */
            void reset();
    };

    class PositionTrackerGroup {
        public:
            knights::PositionTracker *right_tracker = nullptr; // the rightmost tracker
            knights::PositionTracker *left_tracker = nullptr; // the leftmost tracker
            knights::PositionTracker *front_tracker = nullptr; // the frontmost tracker
            knights::PositionTracker *back_tracker = nullptr; // the backmost tracker
            pros::IMU *inertial = nullptr; // inertial sensor to use instead of tracking wheels

            /**
             * @brief Construct a new Position Tracker Group object
             * 
             * @param right right tracker, should be parallel to the drivetrain
             * @param left left tracker, should be parallel to the drivetrain
             * @param front front tracker, should be perpendicular to the drivetrain
             * @param back back tracker, should be perpendicular to the drivetrain
             */
            PositionTrackerGroup(knights::PositionTracker *right, knights::PositionTracker *left, knights::PositionTracker *front, knights::PositionTracker *back);

            /**
             * @brief Construct a new Position Tracker Group object
             * 
             * @param right right tracker, should be parallel to the drivetrain
             * @param left left tracker, should be parallel to the drivetrain
             * @param back back tracker, should be perpendicular to the drivetrain
             */
            PositionTrackerGroup(knights::PositionTracker *right, knights::PositionTracker *left, knights::PositionTracker *back);

            /**
             * @brief Construct a new Position Tracker Group object
             * 
             * @param right right tracker, should be parallel to the drivetrain
             * @param left left tracker, should be parallel to the drivetrain
             */
            PositionTrackerGroup(knights::PositionTracker *right, knights::PositionTracker *left);

            /**
             * @brief Construct a new Position Tracker Group object
             * 
             * @param middle middle tracker, should be parallel to the drivetrain
             * @param back back tracker, should be perpendicular to the drivetrain
             * @param inertial inertial sensor (IMU) to use for the heading
             */
            PositionTrackerGroup(knights::PositionTracker *middle, knights::PositionTracker *back, pros::IMU *inertial);
    };

}

#endif
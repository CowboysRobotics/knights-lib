#pragma once

#ifndef _POSITION_TRACKER_H
#define _POSITION_TRACKER_H

#include "api.h"

namespace knights {

    class Position_Tracker {
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
        public:
            // @brief creates a new position tracking wheel
            // @param rotation the rotation sensor for the wheel
            // @param wheel_diameter the diameter of the wheel
            // @param gear_ratio the gear ratio of the wheel
            Position_Tracker(pros::Rotation *rotation, float wheel_diameter, float gear_ratio, float offset);

            // @brief creates a new position tracking wheel
            // @param adi_encoder the threewire encoder for the wheel
            // @param wheel_diameter the diameter of the wheel
            // @param gear_ratio the gear ratio of the wheel
            Position_Tracker(pros::adi::Encoder *adi_encoder, float wheel_diameter, float gear_ratio, float offset);

            // @brief creates a new position tracking wheel
            // @param motor the motor encoder for the wheel
            // @param wheel_diameter the diameter of the wheel
            // @param gear_ratio the gear ratio of the wheel
            Position_Tracker(pros::Motor *motor, float wheel_diameter, float gear_ratio, float offset);

            // @brief track the total distance the tracker has travelled
            // @return the position converted to inches
            float get_distance_travelled();

            // @brief get the offset of the wheel from the tracking center
            // @return the offset of the wheel from the tracking center
            float get_offset();

            // @brief resets the value of the encoder to zero
            void reset();
    };

    class Position_Tracker_Group {
        public:
            knights::Position_Tracker *right_tracker = nullptr; // the rightmost tracker
            knights::Position_Tracker *left_tracker = nullptr; // the leftmost tracker
            knights::Position_Tracker *front_tracker = nullptr; // the frontmost tracker
            knights::Position_Tracker *back_tracker = nullptr; // the backmost tracker
            pros::IMU *inertial = nullptr; // inertial sensor to use instead of tracking wheels

            // @brief create a new group of position trackers
            // @param right the rightmost tracker
            // @param left the leftmost tracker
            // @param front the frontmost tracker
            // @param back the backmost tracker
            Position_Tracker_Group(knights::Position_Tracker *right, knights::Position_Tracker *left, knights::Position_Tracker *front, knights::Position_Tracker *back);

            // @brief create a new group of position trackers
            // @param right the rightmost tracker
            // @param left the leftmost tracker
            // @param back the backmost tracker
            Position_Tracker_Group(knights::Position_Tracker *right, knights::Position_Tracker *left, knights::Position_Tracker *back);

            // @brief create a new group of position trackers
            // @param right the rightmost tracker
            // @param left the leftmost tracker
            Position_Tracker_Group(knights::Position_Tracker *right, knights::Position_Tracker *left);

            // @brief create a new group of position trackers
            // @param middle tracking wheel in middle of bot
            // @param back the backmost tracker
            // @param inertial inertial sensor to use for angle
            Position_Tracker_Group(knights::Position_Tracker *middle, knights::Position_Tracker *back, pros::IMU *inertial);
    };

}

#endif
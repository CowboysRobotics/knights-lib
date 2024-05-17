#pragma once

#ifndef _DRIVETRAIN_H
#define _DRIVETRAIN_H

#include "api.h"

namespace knights {

    class Drivetrain {
        private:
            pros::Motor_Group *right_mtrs; // motors of right side of drivetrain
            pros::Motor_Group *left_mtrs;  // motors of left side of drivetrain
            float track_width; // width of the drivetrain
            float rpm; // max rpm of the drivetrain (ie 450rpm, 600 rpm, etc)
            float wheel_diameter; // diameters of the largest wheels on the drivetrain
            float gear_ratio; // gear ratio of the drivetrain

            friend class Robot_Chassis;
            friend class Robot_Controller;
        public:
            // @brief create a differential drivetrain object
            // @param right_mtrs motors of right side of drivetrain
            // @param left_mtrs motors of left side of drivetrain
            // @param track_width width of the drivetrain
            // @param rpm max rpm of the drivetrain (ie 450rpm, 600 rpm, etc)
            // @param wheel_diameter diameters of the largest wheels on the drivetrain
            // @param gear_ratio gear ratio of the drivetrain
            Drivetrain(pros::Motor_Group *right_mtrs, pros::Motor_Group *left_mtrs, float track_width, float rpm, float wheel_diameter, float gear_ratio);

            // @brief update the velocities of the drivetrain
            // @param rightMtrs,leftMtrs the desired velocity
            void velocity_command(int rightMtrs, int leftMtrs);

            // @brief convert the distance to position for this drivetrain
            // @param distance distance to convert
            // @return required position for the motors
            float distance_to_position(float distance);

            // @brief convert the position to distance for this drivetrain
            // @param position position to conver to distance
            // @return distance that will result from moving that position
            float position_to_distance(float position);
    };

    class Holonomic {
        private:
            pros::Motor *frontRight; // front right motor
            pros::Motor *frontLeft; // front left motor
            pros::Motor *backRight; // back right motor
            pros::Motor *backLeft; // back left motor
            float track_width; // width of the drivetrain
            float rpm; // max rpm of the drivetrain (ie 450rpm, 600 rpm, etc)
            float wheel_diameter; // diameters of the largest wheels on the drivetrain
            float gear_ratio; // gear ratio of the drivetrain
        public:
            // @brief create a differential drivetrain object
            // @param frontRight,frontLeft,backRight,backLeft the motors on the drivetrain
            // @param track_width width of the drivetrain
            // @param rpm max rpm of the drivetrain (ie 450rpm, 600 rpm, etc)
            // @param wheel_diameter diameters of the largest wheels on the drivetrain
            Holonomic(pros::Motor *frontRight, pros::Motor *frontLeft, pros::Motor *backRight, pros::Motor *backLeft, 
                float track_width, float rpm, float wheel_diameter, float gear_ratio = 1);

            // @brief update the velocities of the motors
            // @param frontRight,frontLeft,backRight,backLeft the desired velocities for each motor
            void velocity_command(int frontRight, int frontLeft, int backRight, int backLeft);
    };
}

#endif
#pragma once

#ifndef _DRIVETRAIN_H
#define _DRIVETRAIN_H

#include "api.h"

namespace knights {

    class Drivetrain {
        private:
            pros::MotorGroup *right_mtrs; // motors of right side of drivetrain
            pros::MotorGroup *left_mtrs;  // motors of left side of drivetrain
            float track_width; // width of the drivetrain
            float rpm; // max rpm of the drivetrain (ie 450rpm, 600 rpm, etc)
            float wheel_diameter; // diameters of the largest wheels on the drivetrain
            float gear_ratio; // gear ratio of the drivetrain

            friend class RobotChassis;
            friend class RobotController;
            friend class ProfileGenerator;
        public:
            /**
             * @brief Construct a new differential drivetrain object
             * 
             * @param right_mtrs motor group with the right motors
             * @param left_mtrs motor group with the left motors
             * @param track_width the width from right wheel to left wheel of the drivetrain
             * @param rpm the maximum rotations per minute of the drivetrain
             * @param wheel_diameter the diameter of the wheel contacting the ground on the drivetrain
             * @param gear_ratio gear ratio of the motors to the wheels on the drivetrain
             */
            Drivetrain(pros::MotorGroup *right_mtrs, pros::MotorGroup *left_mtrs, float track_width, float rpm, float wheel_diameter, float gear_ratio);

            /**
             * @brief Update the velocity of both sides of the drivetrain
             * 
             * @param rightMtrs velocity for the right motors
             * @param leftMtrs velocity for the left motors
             */
            void velocity_command(int rightMtrs, int leftMtrs);

            /**
             * @brief Conversion function from distance to motor position (in degrees)
             * 
             * @param distance distance to convert
             * @return The distance converted to rotation (in degrees) of the motors 
             */
            float distance_to_position(float distance);

            /**
             * @brief Conversion function from motor position (in degrees) to distance
             * 
             * @param position position to convert
             * @return The given position converted to distance travelled 
             */
            float position_to_distance(float position);

            /**
             * @brief Calculate the max acceleration of the drivetrain
             * 
             * @param mass Mass of the robot
             * @param motor_amt Amount of motors on the drivetrain
             * @param stall_torque Stall torque of the motors
             * @return Max acceleration of the drivetrain
             */
            float max_acceleration(float mass, float motor_amt, float stall_torque = 0.5);

            /**
             * @brief Calculate the maximum velocity of the drivetrain
             * 
             * @return Maximum velocity of the drivetrain 
             */
            float max_velocity();
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
            /**
             * @brief Construct a new holonomic drivetrain object
             * 
             * @param frontRight pointer to the front right motor
             * @param frontLeft pointer to the front left motor
             * @param backRight pointer to the back right motor
             * @param backLeft pointer to the back left motor
             * @param track_width width between the right and left wheels
             * @param rpm max rotations per minute of the wheels
             * @param wheel_diameter diameter of the biggest wheels used
             * @param gear_ratio gear ratio from the motors to the wheels
             */
            Holonomic(pros::Motor *frontRight, pros::Motor *frontLeft, pros::Motor *backRight, pros::Motor *backLeft, 
                float track_width, float rpm, float wheel_diameter, float gear_ratio = 1);

           /**
            * @brief Update the velocities for the motors of the holonomic object
            * 
            * @param frontRight desired velocity of the front right motor
            * @param frontLeft  desired velocity of the front left motor
            * @param backRight desired velocity of the back right motor
            * @param backLeft desired velocity of the back left motor
            */
            void velocity_command(int frontRight, int frontLeft, int backRight, int backLeft);

            /*
            * @brief 
            */
            void field_centric_drive(int vert_axis, int hori_axis, int rot_axis, pros::Imu* inertial);
            
    };
}

#endif
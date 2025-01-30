#pragma once

#include "pros/optical.hpp"
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "knights/autonomous/pid.hpp"
#include "knights/util/calculation.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/distance.hpp"
#include "knights/robot/position_tracker.hpp"
#include "knights/robot/drivetrain.hpp"
#include "knights/robot/chassis.hpp"

// Controller
extern pros::Controller master_controller;

// Motor Groups
extern pros::MotorGroup right_mtrs;
extern pros::MotorGroup left_mtrs;

// Sensors
extern pros::Rotation mid_odom;
extern pros::Rotation back_odom;
extern pros::IMU imu;
extern pros::Optical colors;


// Position Trackers
extern knights::PositionTracker midOdom;
extern knights::PositionTracker backOdom;

// Drivetrain and Chassis
extern knights::Drivetrain drivetrain;
extern knights::PositionTrackerGroup odomTrackers;
extern knights::RobotChassis chassis;

// Lady Brown Arm Mechanism
extern pros::Motor lady_brown;
extern pros::Rotation lady_brown_rotation;

// Intake Mechanism
extern pros::MotorGroup intake;
extern pros::Motor intake_bottom;
extern pros::Motor intake_top;

// Distance Sensor
extern pros::Distance redirect;

// Pneumatics
extern pros::adi::Pneumatics clamp;
extern pros::adi::Pneumatics doinker;

// Velocity Formula
#define velocity_formula(x) (160 * (1 / (1 + std::pow(M_E, -0.1 * (x) + 5))) + 20)


// Constants
#define INTAKE_VELOCITY 300
#define LADY_BROWN_VELOCITY 127.0
#define LADY_BROWN_kP 1
#define LADY_BROWN_kI 0.001
#define LADY_BROWN_kD 0.1
#define LADY_BROWN_DOWN 0
#define LADY_BROWN_LOAD1 32
#define LADY_BROWN_LOAD2 32
#define LADY_BROWN_SCORE 150
#define LADY_BROWN_END_TOLERANCE 1.0


// Flags
extern bool intake_spinning;
extern bool intake_forward;
extern bool lady_brown_spinning;
extern bool lady_brown_forward;
extern bool clamp_down;
extern bool doinker_activate;
extern bool color_sorting;
extern bool blue_alliance;
extern bool red_alliance;


// Function Prototypes
void intake_in();
void intake_out();
void lady_brown_fwd();
void lady_brown_rev();
void lady_brown_to_angle(float angle, int timeout);
void lady_brown_down();
void lady_brown_load1();
void lady_brown_load2();
void lady_brown_score();
void lady_brown_alliance();
void clamp_toggle();
void doinker_toggle();
void toggle_rush_mech();
void change_color();
void toggle_color_sort();
void red_color_sort();
void blue_color_sort();

#endif // ROBOT_CONFIG_H

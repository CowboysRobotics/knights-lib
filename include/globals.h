#pragma once

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

extern pros::Task *odomTask;
extern pros::Task *ladyBrownTask;
extern pros::Task *colorSortTask;
extern pros::Task *intakeJamTask;

// Controller
extern pros::Controller master_controller;

// Motors and Sensors
extern pros::MotorGroup left_mtrs;
extern pros::MotorGroup right_mtrs;
extern pros::Rotation mid_odom;
extern pros::Rotation back_odom;
extern pros::IMU imu;
extern knights::PositionTracker midOdom;
extern knights::PositionTracker backOdom;
extern knights::Drivetrain drivetrain;
extern knights::PositionTrackerGroup odomTrackers;
extern knights::RobotChassis chassis;

extern pros::Distance left_sensor;
extern pros::Distance back_sensor;
extern pros::Distance right_sensor;
extern pros::Distance front_sensor;

extern knights::DistanceTracker back;
extern knights::DistanceTracker left;
extern knights::DistanceTracker right;

// Intake system
extern pros::MotorGroup intake;
extern pros::Motor intake_bottom;
extern pros::Motor intake_top;
extern bool intake_spinning;
extern bool intake_forward;
extern bool jam_enabled;

void intake_in();
void intake_out();
void unjam_intake_check();

// Color Sorting
extern pros::Optical colors;
extern bool color_sorting;
extern bool auton_color_sorting;
extern bool blue_alliance;
extern bool red_alliance;

void toggle_color_sort();
void change_color();
void red_color_sort();
void blue_color_sort();
void red_color_auton_sort();
void blue_color_auton_sort();


// Lady Brown Mechanism
extern pros::Motor lady_brown;
extern pros::Rotation lady_brown_rotation;
extern knights::PIDController lady_brown_PID;

extern bool lady_brown_spinning;
extern bool lady_brown_forward;
extern float lady_brown_target;

void lady_brown_fwd();
void lady_brown_rev();
float get_lady_brown_command();
void lady_brown_down();
void lady_brown_load1();
void lady_brown_load2();
void lady_brown_score();
void lady_brown_alliance();

void score_wall_stake();

// Pneumatics
extern pros::adi::Pneumatics clamp;
extern pros::adi::Pneumatics doinker;
extern pros::adi::Pneumatics doinker2;

extern bool clamp_down;
extern bool doinker_activate;
extern bool doinker_activate2;

void clamp_toggle();
void doinker_toggle();
void doinker_toggle2();

// Constants
#define velocity_formula(x) 160*(1/(1+std::pow(M_E, -0.1 * x + 5))) + 20
#define INTAKE_VELOCITY 200

#endif // ROBOT_CONFIG_H

#pragma once

#include "knights/robot/chassis.hpp"
#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

#include "knights/api.hpp"
#include "globals.h"

void pid_tuning(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void pp_test(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void skills(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void sig_winpoint_red(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void sig_winpoint_blue(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void ring_5_rush_blue(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void ring_5_rush_red(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void ring_6_rush_blue(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void ring_6_rush_red(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void sig_winpoint_red_cut(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void sig_winpoint_blue_cut(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void goal_side_red(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void goal_side_blue(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void ring_side_red(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void ring_side_blue(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

#endif
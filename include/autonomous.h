#pragma once

#include "knights/robot/chassis.hpp"
#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

#include "knights/api.hpp"
#include "globals.h"

void pid_tuning(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void pp_test(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void skills(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void redone_skills(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void na_skills(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void safer_skills(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void red_left_wp_new(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void blue_right_wp_new(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void red_left_wp(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void red_rush_right_wp(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void blue_right_wp(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void blue_rush_left_wp(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void alt_skills(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void red_left_elim(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void red_rush_right_elim(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void blue_right_elim(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void blue_rush_left_elim(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void right_safe_no_wait(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void rush_4_ring(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void mogo_red_rush(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void mogo_blue_rush(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void sig_red_winpoint(knights::RobotChassis *chassis, bool flip_x, bool flip_y);


#endif
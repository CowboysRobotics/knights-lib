#pragma once

#include "knights/robot/chassis.hpp"
#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

#include "knights/api.hpp"
#include "globals.h"

void pid_tuning(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void pp_test(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void skills_states(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void red_left_wp(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void red_right_wp(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void red_left_elim(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void red_left_wp_safe(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void red_right_wp_safe(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void blue_left_wp_safe(knights::RobotChassis *chassis, bool flip_x, bool flip_y);
void blue_right_wp_safe(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void red_lb_first_wp_right(knights::RobotChassis *chassis, bool flip_x, bool flip_y);

void blue_mogo_rush_left(knights::RobotChassis *chassis, bool flip_x, bool flip_y);



#endif
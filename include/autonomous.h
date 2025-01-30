#pragma once

#include "knights/robot/chassis.hpp"
#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

#include "knights/api.hpp"
#include "globals.h"

void pid_tuning(knights::RobotChassis *chassis);
void pp_test(knights::RobotChassis *chassis);

void skills(knights::RobotChassis *chassis);

void red_left_wp_new(knights::RobotChassis *chassis);
void blue_right_wp_new(knights::RobotChassis *chassis);

void red_left_wp(knights::RobotChassis *chassis);
void red_rush_right_wp(knights::RobotChassis *chassis);
void blue_right_wp(knights::RobotChassis *chassis);
void blue_rush_left_wp(knights::RobotChassis *chassis);
void alt_skills(knights::RobotChassis *chassis);

void red_left_elim(knights::RobotChassis *chassis);
void red_rush_right_elim(knights::RobotChassis *chassis);
void blue_right_elim(knights::RobotChassis *chassis);
void blue_rush_left_elim(knights::RobotChassis *chassis);


void right_safe_no_wait(knights::RobotChassis *chassis);


void rush_4_ring(knights::RobotChassis *chassis);

void mogo_red_rush(knights::RobotChassis *chassis);
void mogo_blue_rush(knights::RobotChassis *chassis);


#endif
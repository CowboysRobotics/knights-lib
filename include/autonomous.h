#pragma once

#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

#include "knights/robot/chassis.h"

void pid_tuning(knights::RobotChassis *chassis);
void pp_test(knights::RobotChassis *chassis);

void skills(knights::RobotChassis *chassis);

void red_left_wp_new(knights::RobotChassis *chassis);
void blue_right_wp_new(knights::RobotChassis *chassis);

void red_left_wp(knights::RobotChassis *chassis);
void red_rush_right_wp(knights::RobotChassis *chassis);
void blue_right_wp(knights::RobotChassis *chassis);
void blue_rush_left_wp(knights::RobotChassis *chassis);

void red_left_elim(knights::RobotChassis *chassis);
void red_rush_right_elim(knights::RobotChassis *chassis);
void blue_right_elim(knights::RobotChassis *chassis);
void blue_rush_left_elim(knights::RobotChassis *chassis);




#endif
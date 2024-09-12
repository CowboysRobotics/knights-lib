#pragma once

#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

#include "knights/robot/chassis.h"

void pid_tuning(knights::RobotChassis *chassis);
void right_wp_auton(knights::RobotChassis *chassis);
void programming_skills(knights::RobotChassis *chassis);
void alex_skills(knights::RobotChassis *chassis);
void pp_test(knights::RobotChassis *chassis);
void unsafe_wp_auton(knights::RobotChassis *chassis);

#endif
#pragma once

#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

#include "knights/robot/chassis.h"

void pid_tuning(knights::RobotChassis *chassis);
void pp_test(knights::RobotChassis *chassis);

#endif
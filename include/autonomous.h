#pragma once

#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

#include "knights/robot/chassis.h"

void skills(knights::RobotChassis *chassis);
void pid_tuning(knights::RobotChassis *chassis);

#endif
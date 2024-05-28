#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"

#include "knights/robot/chassis.h"

#include "knights/util/calculation.h"

void knights::Robot_Controller::turn_to_angle(const float angle, const float end_tolerance, float timeout, bool rad) {
    if (this->in_motion) return;
    this->in_motion = true;

    // turn to an angle provided in degrees

    this->in_motion = false;
    return;
}

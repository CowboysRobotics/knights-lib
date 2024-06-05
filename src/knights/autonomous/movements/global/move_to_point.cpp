#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"

#include "knights/robot/chassis.h"

#include "knights/util/calculation.h"

void knights::RobotController::move_to_point(const Pos point, const float &end_tolerance, float timeout, const float &num_pts) {
    if (this->in_motion) return;
    this->in_motion = true;

    // move to a point using boomerang and a pure pursuit route

    this->in_motion = false;
    return;
}
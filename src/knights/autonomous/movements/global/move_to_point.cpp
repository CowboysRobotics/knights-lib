#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"

#include "knights/robot/chassis.h"

#include "knights/util/calculation.h"

void knights::Robot_Controller::move_to_point(const Pos point, const float &end_tolerance, float timeout, const float &num_pts) {
    // move to a point using boomerang and a pure pursuit route
}
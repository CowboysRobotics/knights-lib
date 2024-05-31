#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"
#include "knights/autonomous/path.h"

#include "knights/robot/chassis.h"

#include "knights/util/calculation.h"

void knights::init_route_from_sd(std::string route_name) {

}

void knights::Robot_Controller::follow_route(knights::Route &route, const float &lookahead_distance, const float max_speed, const bool forwards, 
    const float end_tolerance, float timeout) {
    if (this->in_motion) return;
    this->in_motion = true;

    // follow a pure pursuit route  

    this->in_motion = false;
    return;
}
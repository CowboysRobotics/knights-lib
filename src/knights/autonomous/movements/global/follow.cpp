#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"

#include "knights/robot/chassis.h"

#include "knights/util/calculation.h"

void knights::Robot_Controller::follow_route(std::string route_name, const float &lookahead_distance, const float max_speed, const bool forwards, 
    const float end_tolerance, float timeout) {
    // follow a pure pursuit route  
}
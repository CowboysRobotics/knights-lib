#include "knights/autonomous/controller.hpp"
#include "knights/autonomous/pid.hpp"
#include "knights/autonomous/pathgen.hpp"

#include "knights/logger/logger.hpp"
#include "knights/robot/chassis.hpp"

#include "knights/util/calculation.hpp"

#include "knights/util/position.hpp"

#include <fstream>

void knights::RobotController::lateral_to_position(const Pos desired_position, const bool forwards, const float end_tolerance, const int timeout, const int turn_timeout, const int wait_time) {
    this->turn_to_point(desired_position, forwards, 0, 2.0, turn_timeout);
    pros::delay(wait_time);
    if (forwards)
        this->lateral_move(distance_btwn(this->chassis->curr_position, desired_position), end_tolerance, timeout);
    else
        this->lateral_move(-distance_btwn(this->chassis->curr_position, desired_position), end_tolerance, timeout);
    pros::delay(wait_time);
    this->turn_to_angle(knights::to_deg(desired_position.heading), 0, 2.0, turn_timeout);
}

void knights::RobotController::lateral_to_point(const Pos desired_position, const bool forwards, const float end_tolerance, const int timeout, const int turn_timeout, const int wait_time) {
    this->turn_to_point(desired_position, forwards, 0, 2.0, turn_timeout);
    pros::delay(wait_time);
    if (forwards)
        this->lateral_move(distance_btwn(this->chassis->curr_position, desired_position), end_tolerance, timeout);
    else
        this->lateral_move(-distance_btwn(this->chassis->curr_position, desired_position), end_tolerance, timeout);
    pros::delay(wait_time);
}
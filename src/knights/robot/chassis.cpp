#include "api.h"

#include "knights/robot/chassis.h"
#include "knights/robot/drivetrain.h"
#include "knights/robot/position_tracker.h"

#include "knights/util/calculation.h"

knights::RobotChassis::RobotChassis(Drivetrain *drivetrain, PositionTrackerGroup *pos_trackers)
    : drivetrain(drivetrain), pos_trackers(pos_trackers) {
}

knights::RobotChassis::RobotChassis(Holonomic *drivetrain, PositionTrackerGroup *pos_trackers)
    : holonomic(drivetrain), pos_trackers(pos_trackers) {
}

void knights::RobotChassis::set_position(float x, float y, float heading) {
    this->curr_position.x = x;
    this->curr_position.y = y;
    this->curr_position.heading = heading;
}

knights::Pos knights::RobotChassis::get_position() {
    return this->curr_position;
}

void knights::RobotChassis::set_position(knights::Pos position) {
    this->curr_position = position;
    this->set_prev_position(position);
};

void knights::RobotChassis::set_prev_position(float x, float y, float heading) {
    this->prev_position.x = x;
    this->prev_position.y = y;
    this->prev_position.heading = heading;
}

knights::Pos knights::RobotChassis::get_prev_position() {
    return this->prev_position;
}

void knights::RobotChassis::set_prev_position(knights::Pos position) {
    this->prev_position = position;
};
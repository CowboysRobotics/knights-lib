#include "api.h"

#include "knights/robot/chassis.h"
#include "knights/robot/drivetrain.h"
#include "knights/robot/position_tracker.h"

knights::Robot_Chassis::Robot_Chassis(Drivetrain *drivetrain, Position_Tracker_Group *pos_trackers)
    : drivetrain(drivetrain), pos_trackers(pos_trackers) {
}

knights::Robot_Chassis::Robot_Chassis(Holonomic *drivetrain, Position_Tracker_Group *pos_trackers)
    : holonomic(drivetrain), pos_trackers(pos_trackers) {
}

void knights::Robot_Chassis::set_position(float x, float y, float heading) {
    this->curr_position.x = x;
    this->curr_position.y = y;
    this->curr_position.heading = heading;
}

knights::Pos knights::Robot_Chassis::get_position() {
    return this->curr_position;
}

void knights::Robot_Chassis::set_position(knights::Pos position) {
    this->curr_position = position;
};

void knights::Robot_Chassis::set_prev_position(float x, float y, float heading) {
    this->prev_position.x = x;
    this->prev_position.y = y;
    this->prev_position.heading = heading;
}

knights::Pos knights::Robot_Chassis::get_prev_position() {
    return this->prev_position;
}

void knights::Robot_Chassis::set_prev_position(knights::Pos position) {
    this->prev_position = position;
};
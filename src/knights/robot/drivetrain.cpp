#include "api.h"

#include "knights/robot/chassis.h"
#include "knights/robot/drivetrain.h"

knights::Drivetrain::Drivetrain(pros::Motor_Group *right_mtrs, pros::Motor_Group *left_mtrs, float track_width, float rpm, float wheel_diameter) {
    this->right_mtrs = right_mtrs;
    this->left_mtrs = left_mtrs;
    this->track_width = track_width;
    this->rpm = rpm;
    this->wheel_diameter = wheel_diameter;
}

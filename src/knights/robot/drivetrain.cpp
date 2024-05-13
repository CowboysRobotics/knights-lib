#include "api.h"

#include "knights/robot/chassis.h"
#include "knights/robot/drivetrain.h"

knights::Drivetrain::Drivetrain(pros::Motor_Group *right_mtrs, pros::Motor_Group *left_mtrs, float track_width, float rpm, float wheel_diameter, float gear_ratio) {
    this->right_mtrs = right_mtrs;
    this->left_mtrs = left_mtrs;
    this->track_width = track_width;
    this->rpm = rpm;
    this->wheel_diameter = wheel_diameter;
    this->gear_ratio = gear_ratio;
}

void knights::Drivetrain::velocity_command(int rightMtrs, int leftMtrs) {
    this->right_mtrs->move(rightMtrs);
    this->left_mtrs->move(leftMtrs);
}

float knights::Drivetrain::distance_to_position(float distance) {
    return distance / ((this->gear_ratio * this->wheel_diameter * M_PI) / 360);
};

float knights::Drivetrain::position_to_distance(float position) {
    return ((this->gear_ratio * this->wheel_diameter * M_PI) / 360) * position;
};

knights::Holonomic::Holonomic(pros::Motor *frontRight, pros::Motor *frontLeft, pros::Motor *backRight, pros::Motor *backLeft, float track_width, float rpm, float wheel_diameter) {
    this->frontRight = frontRight;
    this->frontLeft = frontLeft;
    this->backRight = backRight;
    this->backLeft = backLeft;
    this->track_width = track_width;
    this->rpm = rpm;
    this->wheel_diameter = wheel_diameter;
};

void knights::Holonomic::velocity_command(int frontRight, int frontLeft, int backRight, int backLeft) {
    this->frontRight->move(frontRight);
    this->frontLeft->move(frontLeft);
    this->backRight->move(backRight);
    this->backLeft->move(backLeft);
}

#include "pros/imu.hpp"
#include <cmath>
#include "knights/robot/drivetrain.h"

knights::Drivetrain::Drivetrain(pros::MotorGroup *right_mtrs, pros::MotorGroup *left_mtrs, float track_width, float rpm, float wheel_diameter, float gear_ratio) 
    : right_mtrs(right_mtrs), left_mtrs(left_mtrs), track_width(track_width), rpm(rpm), wheel_diameter(wheel_diameter), gear_ratio(gear_ratio) {
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

knights::Holonomic::Holonomic(pros::Motor *frontRight, pros::Motor *frontLeft, pros::Motor *backRight, pros::Motor *backLeft, float track_width, float rpm, float wheel_diameter, float gear_ratio)
    : frontRight(frontRight), frontLeft(frontLeft), backRight(backRight), backLeft(backLeft), track_width(track_width), rpm(rpm), wheel_diameter(wheel_diameter), gear_ratio(gear_ratio) {
}

void knights::Holonomic::velocity_command(int frontRight, int frontLeft, int backRight, int backLeft) {
    this->frontRight->move(frontRight);
    this->frontLeft->move(frontLeft);
    this->backRight->move(backRight);
    this->backLeft->move(backLeft);
}

void knights::Holonomic::field_centric_drive(int vert_axis, int hori_axis, int rot_axis, pros::Imu* inertial) {
    
    // need to convert vertical and horizontal vector to positional control of the bot
    // arccosine arcsine

    float power_magnitude = std::hypotf(vert_axis, hori_axis);

    float theta = atan2(vert_axis,hori_axis);
    float theta2 = theta - inertial->get_heading();

    int l_vert_axis = power_magnitude * std::cos(theta);
    int l_hori_axis = power_magnitude * std::sin(theta);

    float lF, rF, lB, rB;

    lF = l_vert_axis + l_hori_axis + float(rot_axis) / 2;
    lB = l_vert_axis - l_hori_axis + float(rot_axis) / 2;

    rF = -l_vert_axis + l_hori_axis + float(rot_axis) / 2;
    rB = -l_vert_axis - l_hori_axis + float(rot_axis) / 2;

    this->velocity_command(rF, lF, rB, lB);
}

#include "knights/util/calculation.hpp"
#include "pros/imu.hpp"
#include <cmath>
#include "knights/robot/drivetrain.hpp"

#define MAX_VOLTAGE 127.0
#define SECONDS_PER_MIN 60.0

knights::Drivetrain::Drivetrain(pros::MotorGroup *right_mtrs, pros::MotorGroup *left_mtrs, float track_width, float rpm, float wheel_diameter, float gear_ratio) 
    : right_mtrs(right_mtrs), left_mtrs(left_mtrs), track_width(track_width), rpm(rpm), wheel_diameter(wheel_diameter), gear_ratio(gear_ratio) {
}

void knights::Drivetrain::voltage_command(int rightMtrs, int leftMtrs) {
    this->right_mtrs->move(rightMtrs);
    this->left_mtrs->move(leftMtrs);
}

void knights::Drivetrain::velocity_command(float linear_velocity, float angular_velocity, float maximum_lin_vel) {
    float r_speed = linear_velocity + (angular_velocity * track_width / 2.0);
    float l_speed = linear_velocity - (angular_velocity * track_width / 2.0);

    r_speed = (r_speed / (wheel_diameter * M_PI) * (1/gear_ratio)) * 60.0;
    l_speed = (l_speed / (wheel_diameter * M_PI) * (1/gear_ratio)) * 60.0;

    float ratio_maximum_lin_vel = (maximum_lin_vel / (wheel_diameter * M_PI) * (1/gear_ratio)) * 60.0;

    // ratio may be causing the issue
    float ratio_curr_speed = std::fmax(fabs(r_speed), fabs(l_speed)) / (ratio_maximum_lin_vel); 
    if (ratio_curr_speed > 1) {
        r_speed /= ratio_curr_speed;
        l_speed /= ratio_curr_speed;
    }

    this->right_mtrs->move_velocity(r_speed);
    this->left_mtrs->move_velocity(l_speed);
}

void knights::Drivetrain::ramsete_command(float linear_velocity, float angular_velocity, float acceleration, float tuner_velocity, float tuner_accel, float tuner_static, float drivetrain_max) {
    float r_speed = linear_velocity + (angular_velocity * track_width / 2.0);
    float l_speed = linear_velocity - (angular_velocity * track_width / 2.0);

    r_speed = (r_speed / (wheel_diameter * M_PI) * (1/gear_ratio)) * 60.0;
    l_speed = (l_speed / (wheel_diameter * M_PI) * (1/gear_ratio)) * 60.0;

    r_speed = r_speed * tuner_velocity + acceleration * tuner_accel + knights::signum(r_speed) * tuner_static;
    l_speed = l_speed * tuner_velocity + acceleration * tuner_accel + knights::signum(l_speed) * tuner_static;

    float ratio_maximum_lin_vel = ((drivetrain_max / (wheel_diameter * M_PI) * (1/gear_ratio)) * 60.0) * tuner_velocity + acceleration * tuner_accel + knights::signum(r_speed) * tuner_static;

    // ratio may be causing the issue
    float ratio_curr_speed = std::fmax(fabs(r_speed), fabs(l_speed)) / (ratio_maximum_lin_vel); 
    if (ratio_curr_speed > 1) {
        r_speed /= ratio_curr_speed;
        l_speed /= ratio_curr_speed;
    }

    this->right_mtrs->move_velocity(r_speed);
    this->left_mtrs->move_velocity(l_speed);
}

float knights::Drivetrain::distance_to_position(float distance) {
    return distance / ((this->gear_ratio * this->wheel_diameter * M_PI) / 360);
};

float knights::Drivetrain::position_to_distance(float position) {
    return ((this->gear_ratio * this->wheel_diameter * M_PI) / 360) * position;
};

float knights::Drivetrain::max_acceleration(float mass, float motor_amt, float stall_torque) {
    return ((stall_torque / (this->wheel_diameter/2))*motor_amt) / mass;
}

float knights::Drivetrain::max_velocity() {
    // v = circumfrence * rotation rate
    return M_PI * this->wheel_diameter * (this->rpm / 60.0);
}

float knights::Drivetrain::voltage_to_velocity(float voltage) {
    return (voltage/MAX_VOLTAGE) * this->max_velocity();
}

float knights::Drivetrain::velocity_to_voltage(float velocity) {
    return (MAX_VOLTAGE * SECONDS_PER_MIN * velocity) / (this->rpm * M_PI * this->wheel_diameter * this->gear_ratio);
}

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
    // NOT IMPLEMENTED YET
}

#include "knights/autonomous/controller.hpp"
#include "knights/autonomous/pid.hpp"

#include "knights/robot/chassis.hpp"

#include "knights/util/calculation.hpp"

#include "knights/logger/logger.hpp"
#include "knights/util/position.hpp"

void knights::RobotController::turn_to_angle(const float angle, int direction, float end_tolerance, int timeout, bool rad) {
    if (this->in_motion) return;
    this->in_motion = true;

    // get direction to turn (l, r, best)
    int sign = knights::signum(direction);

    float speed,error;
    float desired_angle;
    
    if (rad == true) {// inputs provided in rads
        desired_angle = normalize_angle(angle, true);
    }
    else { // inputs provided in degrees
        end_tolerance = to_rad(end_tolerance);
        desired_angle = normalize_angle(to_rad(angle), true);
    }
    
    // set brake mode to stop so we don't overshoot
    this->chassis->drivetrain->right_mtrs->set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    this->chassis->drivetrain->left_mtrs->set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    this->turn_pid->reset();

    this->turn_pid->switch_values(
        std::abs(min_angle(this->chassis->curr_position.heading, desired_angle, true))
    );

    while(std::abs(min_angle(this->chassis->curr_position.heading, desired_angle, true)) > end_tolerance) {

        if (direction == 0) // if we're taking best direction
            sign = knights::direction(this->chassis->curr_position.heading, desired_angle); // calculate direction

        timeout -= 10;
        if (timeout < 0) break;

        // calculate w/ PID formula
        error = std::abs(min_angle(this->chassis->curr_position.heading, desired_angle, true));
        speed = this->turn_pid->update(error);

        this->chassis->drivetrain->voltage_command(-sign * speed, sign * speed);

        pros::delay(10);
    }

    this->chassis->drivetrain->voltage_command(0, 0);

    this->in_motion = false;
    return;
}

void knights::RobotController::turn_to_point(knights::Pos point, bool forwards, int direction, float end_tolerance, int timeout) {
    if (forwards)
        return this->turn_to_angle(
            std::atan2(point.y - this->chassis->get_position().y, point.x - this->chassis->get_position().x),
            direction,
            knights::to_rad(end_tolerance),
            timeout,
            true
        );
    else
        return this->turn_to_angle(
            std::atan2(this->chassis->get_position().y - point.y, this->chassis->get_position().x - point.x),
            direction,
            knights::to_rad(end_tolerance),
            timeout,
            true
        );
}

void knights::RobotController::right_swing_to_angle(const float angle, float end_tolerance, int timeout, bool rad) {
    if (this->in_motion) return;
    this->in_motion = true;

    float speed,error;
    float desired_angle;

    if (rad == true) // inputs provided in rads
        desired_angle = normalize_angle(angle, true);
    else { // inputs provided in degrees
        end_tolerance = to_rad(end_tolerance);
        desired_angle = normalize_angle(to_rad(angle), true);
    }

    int sign = knights::direction(this->chassis->curr_position.heading, desired_angle); // calculate direction
    
    // set brake mode to stop so we don't overshoot
    this->chassis->drivetrain->right_mtrs->set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    this->chassis->drivetrain->left_mtrs->set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    this->turn_pid->reset();

    this->turn_pid->switch_values(
        std::abs(min_angle(this->chassis->curr_position.heading, desired_angle, true))
    );

    while(std::abs(min_angle(this->chassis->curr_position.heading, desired_angle, true)) > end_tolerance) {

        if (sign == 0) // if we're taking best direction
            sign = knights::direction(this->chassis->curr_position.heading, desired_angle); // calculate direction

        timeout -= 10;
        if (timeout < 0) break;

        // calculate w/ PID formula
        error = std::abs(min_angle(this->chassis->curr_position.heading, desired_angle, true));
        speed = this->turn_pid->update(error);

        this->chassis->drivetrain->voltage_command(-sign * speed, 0);

        pros::delay(10);
    }

    this->chassis->drivetrain->voltage_command(0, 0);

    this->in_motion = false;
    return;
}

void knights::RobotController::left_swing_to_angle(const float angle, float end_tolerance, int timeout, bool rad) {
    if (this->in_motion) return;
    this->in_motion = true;

    float speed,error;
    float desired_angle;

    if (rad == true) // inputs provided in rads
        desired_angle = normalize_angle(angle, true);
    else { // inputs provided in degrees
        end_tolerance = to_rad(end_tolerance);
        desired_angle = normalize_angle(to_rad(angle), true);
    }

    int sign = knights::direction(this->chassis->curr_position.heading, desired_angle); // calculate direction
    
    // set brake mode to stop so we don't overshoot
    this->chassis->drivetrain->right_mtrs->set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    this->chassis->drivetrain->left_mtrs->set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    this->turn_pid->reset();

    this->turn_pid->switch_values(
        std::abs(min_angle(this->chassis->curr_position.heading, desired_angle, true))
    );

    while(std::abs(min_angle(this->chassis->curr_position.heading, desired_angle, true)) > end_tolerance) {

        if (sign == 0) // if we're taking best direction
            sign = knights::direction(this->chassis->curr_position.heading, desired_angle); // calculate direction

        timeout -= 10;
        if (timeout < 0) break;

        // calculate w/ PID formula
        error = std::abs(min_angle(this->chassis->curr_position.heading, desired_angle, true));
        speed = this->turn_pid->update(error);

        this->chassis->drivetrain->voltage_command(-sign * speed, 0);

        pros::delay(10);
    }

    this->chassis->drivetrain->voltage_command(0, 0);

    this->in_motion = false;
    return;
}

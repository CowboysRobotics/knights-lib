#include "knights/autonomous/controller.hpp"
#include "knights/autonomous/pid.hpp"
#include "knights/autonomous/pathgen.hpp"

#include "knights/robot/chassis.hpp"

#include "knights/util/calculation.hpp"

#include "knights/util/position.hpp"

void knights::RobotController::move_to_point(const Pos desired_position, float lead, float correction_dist, const float &end_tolerance, const bool forwards, float timeout) {
    if (this->in_motion) return;
    this->in_motion = true;

    int direction = forwards ? 1 : -1;

    this->chassis->drivetrain->right_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    this->chassis->drivetrain->left_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    this->lateral_pid->reset();
    this->angular_pid->reset();

    bool needs_reverse = false;

    float target_dist_error = distance_btwn(this->chassis->get_position(), desired_position);

    float angular_error;
    float end_theta = desired_position.heading;

    while (target_dist_error > end_tolerance) {
        knights::Pos curr_position = this->chassis->curr_position;

        target_dist_error = distance_btwn(curr_position, desired_position);

        knights::Pos carrot(
            desired_position.x - target_dist_error * cos(end_theta) * lead,
            desired_position.y - target_dist_error * sin(end_theta) * lead,
            desired_position.heading
        );

        float dx = carrot.x - curr_position.x;
        float dy = carrot.y - curr_position.y;

        if (forwards) {
            angular_error = knights::normalize_angle(atan2(dy, dx) - curr_position.heading);
        } else {
            angular_error = knights::normalize_angle(atan2(-dy, -dx) - curr_position.heading);
        }

        float linear_vel = this->lateral_pid->update(target_dist_error) * direction;

        float target_angular_error = knights::normalize_angle(desired_position.heading - curr_position.heading);

        float angular_vel;

        if (target_dist_error < correction_dist) {
            needs_reverse = true;
            angular_vel = angular_pid->update(target_angular_error);
        } else if (target_dist_error < 2 * correction_dist) {
            float scale = (target_dist_error - correction_dist) / correction_dist;
            float output_angular_error = knights::normalize_angle(scale * angular_error + (1 - scale) * target_dist_error);

            angular_vel = angular_pid->update(output_angular_error);
        } else {
            if (fabsf(angular_error) > M_PI/2 && needs_reverse) {
                angular_error = angular_error - signum(angular_error) * M_PI;
                linear_vel *= -1;
            }
        }

        linear_vel *= std::cos(angular_error);
        linear_vel = knights::clamp(linear_vel, -127.0, 127.0);

        chassis->drivetrain->voltage_command(linear_vel - angular_vel, linear_vel + angular_vel);

        pros::delay(20);
    }


    this->in_motion = false;
    return;
}
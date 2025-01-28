#include "knights/autonomous/controller.hpp"
#include "knights/autonomous/pid.hpp"
#include "knights/autonomous/pathgen.hpp"

#include "knights/logger/logger.hpp"
#include "knights/robot/chassis.hpp"

#include "knights/util/calculation.hpp"

#include "knights/util/position.hpp"

#include <fstream>


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

    // #### DEBUG
    std::fstream write_file("/usd/boomerang_output.txt", std::ios_base::out);

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
            angular_error = knights::ref_angle(atan2(dy, dx) - curr_position.heading);
        } else {
            angular_error = knights::ref_angle(atan2(-dy, -dx) - curr_position.heading);
        }

        write_file << "Test angular error: " << angular_error << "\n";

        float linear_vel = this->lateral_pid->update(target_dist_error) * direction;

        float target_angular_error = knights::ref_angle(desired_position.heading - curr_position.heading);

        double angular_vel;
        if (target_dist_error < correction_dist) {
            needs_reverse = true;
            angular_vel = angular_pid->update(target_angular_error);
        } else if (target_dist_error < 2 * correction_dist) {
            double scale_factor = (target_dist_error - correction_dist) / correction_dist;
            double scaled_angle_error = knights::ref_angle(
                scale_factor * angular_error + (1 - scale_factor) * target_angular_error);

            angular_vel = angular_pid->update(scaled_angle_error);
        } else {
            if (fabs(angular_error) > M_PI/2 && needs_reverse) {
                angular_error =
                    angular_error - (angular_error / fabs(angular_error)) * M_PI;
                linear_vel = -linear_vel;
            }

            angular_vel = angular_pid->update(angular_error);
        }

        linear_vel *= std::cos(angular_error);
        linear_vel = knights::clamp(linear_vel, -127.0, 127.0);

        chassis->drivetrain->voltage_command(linear_vel - angular_vel, linear_vel + angular_vel);

        // DEBUG
        write_file << knights::logger::string_format(
            "Current: %lf %lf %lf , Carrot: %lf %lf %lf , Final: %lf %lf %lf , LinearVel: %lf , AngularVel %lf , DistError %lf , AngularError %lf",
            curr_position.x, curr_position.y, curr_position.heading, desired_position.x, desired_position.y, desired_position.heading, carrot.x, carrot.y, carrot.heading, linear_vel, angular_vel, target_dist_error, angular_error
        ) << "\n";

        pros::delay(20);
    }

    write_file << "end at error " << target_dist_error << "\n";
    std::cout << "end at error " << target_dist_error << "\n";

    this->in_motion = false;
    return;
}
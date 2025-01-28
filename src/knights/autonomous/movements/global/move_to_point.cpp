#include "knights/autonomous/controller.hpp"
#include "knights/autonomous/pid.hpp"
#include "knights/autonomous/pathgen.hpp"

#include "knights/logger/logger.hpp"
#include "knights/robot/chassis.hpp"

#include "knights/util/calculation.hpp"

#include "knights/util/position.hpp"

#include <fstream>

// Using math from VOSS's implementation
void knights::RobotController::move_to_position(const Pos desired_position, float lead, float correction_dist, const float &end_tolerance, const bool forwards, float timeout) {
    if (this->in_motion) return;
    this->in_motion = true;

    int direction = forwards ? 1 : -1;

    this->chassis->drivetrain->right_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    this->chassis->drivetrain->left_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    this->lateral_pid->reset();
    this->angular_pid->reset();

    bool needs_reverse = false;

    float distance_error = distance_btwn(this->chassis->get_position(), desired_position);

    float angular_error;

    // #### DEBUG
    std::fstream write_file("/usd/boomerang_output.txt", std::ios_base::out);

    while (distance_error > end_tolerance) {
        Pos current_pos = this->chassis->get_position();

        float distance_error = distance_btwn(desired_position, current_pos);

        Pos carrot(desired_position.x - distance_error * cos(desired_position.heading) * lead,
                            desired_position.y - distance_error * sin(desired_position.heading) * lead,
                            desired_position.heading);

        float dx = carrot.x - current_pos.x;
        float dy = carrot.y - current_pos.y;

        float angular_error;
        if (forwards) {
            angular_error = knights::ref_angle(atan2(dy, dx) - current_pos.heading);
        } else {
            angular_error = knights::ref_angle(atan2(-dy, -dx) - current_pos.heading);
        }

        float lin_speed = this->lateral_pid->update(distance_error) * direction;

        float desired_error = knights::ref_angle(desired_position.heading - current_pos.heading);

        float angular_speed;
        if (distance_error < correction_dist) {
            needs_reverse = true;
            angular_speed = angular_pid->update(desired_error, false);
        } else if (distance_error < 2 * correction_dist) {
            float scale_factor = (distance_error - correction_dist) / correction_dist;
            float scaled_angular_error = knights::ref_angle(
                scale_factor * angular_error + (1 - scale_factor) * desired_error);
            angular_speed = angular_pid->update(scaled_angular_error, false);
        } else {
            if (fabs(angular_error) > M_PI_2 && needs_reverse) {
                angular_error =
                    angular_error - (angular_error / fabs(angular_error)) * M_PI;
                lin_speed = -lin_speed;
            }
            angular_speed = angular_pid->update(angular_error, false);
        }

        lin_speed *= cos(angular_error);
        lin_speed = knights::clamp((float)lin_speed, -127.0, 127.0);

        chassis->drivetrain->voltage_command(lin_speed + angular_speed, lin_speed - angular_speed);

        // DEBUG
        write_file << knights::logger::string_format(
            "Current: %lf %lf %lf , Carrot: %lf %lf %lf , Final: %lf %lf %lf , LinearVel: %lf , AngularVel %lf , DistError %lf , AngularError %lf , R/L: %lf %lf",
            current_pos.x, current_pos.y, current_pos.heading, carrot.x, carrot.y, carrot.heading, desired_position.x, desired_position.y, desired_position.heading, lin_speed, 
            angular_speed, distance_error, angular_error, lin_speed + angular_speed, lin_speed - angular_speed
        ) << "\n";

        pros::delay(20);
    }

    write_file << "end at error " << distance_error << "\n";
    std::cout << "end at error " << distance_error << "\n";

    this->in_motion = false;
    return;
}

void knights::RobotController::lateral_to_position(const Pos desired_position, const float end_tolerance, const int timeout) {
    this->turn_to_point(desired_position, 0, end_tolerance, timeout);
    pros::delay(140);
    this->lateral_move(distance_btwn(this->chassis->curr_position, desired_position), end_tolerance, timeout);
    pros::delay(140);
    this->turn_to_angle(desired_position.heading, 0, end_tolerance, timeout);
}
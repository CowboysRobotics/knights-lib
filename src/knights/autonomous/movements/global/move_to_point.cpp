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

    float distance_error = distance_btwn(this->chassis->get_position(), desired_position);

    float angular_error;

    // #### DEBUG
    std::fstream write_file("/usd/boomerang_output.txt", std::ios_base::out);

    while (distance_error > end_tolerance) {
        Pos current_pos = this->chassis->get_position();

        double dx = desired_position.x - current_pos.x;
        double dy = desired_position.y - current_pos.y;

        double distance_error = sqrt(dx * dx + dy * dy);
        double at = desired_position.heading;

        Pos carrot(desired_position.x - distance_error * cos(at) * lead,
                            desired_position.y - distance_error * sin(at) * lead,
                            desired_position.heading);

        dx = carrot.x - current_pos.x;
        dy = carrot.y - current_pos.y;

        double current_angle = current_pos.heading;

        double angle_error;
        if (forwards) {
            angle_error = atan2(dy, dx) - current_angle;
        } else {
            angle_error = atan2(-dy, -dx) - current_angle;
        }

        angle_error = knights::ref_angle(angle_error);

        double lin_speed = this->lateral_pid->update(distance_error);

        lin_speed *= direction;

        double pose_error = knights::ref_angle(desired_position.heading - current_angle);

        double ang_speed;
        if (distance_error < correction_dist) {
            needs_reverse = true;

            ang_speed = angular_pid->update(pose_error);
        } else if (distance_error < 2 * correction_dist) {
            double scale_factor = (distance_error - correction_dist) / correction_dist;
            double scaled_angle_error = knights::ref_angle(
                scale_factor * angle_error + (1 - scale_factor) * pose_error);

            ang_speed = angular_pid->update(scaled_angle_error);
        } else {
            if (fabs(angle_error) > M_PI_2 && needs_reverse) {
                angle_error =
                    angle_error - (angle_error / fabs(angle_error)) * M_PI;
                lin_speed = -lin_speed;
            }

            ang_speed = angular_pid->update(angle_error);
        }

        lin_speed *= cos(angle_error);

        lin_speed = knights::clamp((float)lin_speed, -127.0, 127.0);

        chassis->drivetrain->voltage_command(lin_speed + ang_speed, lin_speed - ang_speed);

        // DEBUG
        write_file << knights::logger::string_format(
            "Current: %lf %lf %lf , Carrot: %lf %lf %lf , Final: %lf %lf %lf , LinearVel: %lf , AngularVel %lf , DistError %lf , AngularError %lf , R/L: %lf %lf",
            current_pos.x, current_pos.y, current_pos.heading, carrot.x, carrot.y, carrot.heading, desired_position.x, desired_position.y, desired_position.heading, lin_speed, 
            ang_speed, distance_error, angular_error, lin_speed - ang_speed, lin_speed + ang_speed
        ) << "\n";

        pros::delay(20);
    }

    write_file << "end at error " << distance_error << "\n";
    std::cout << "end at error " << distance_error << "\n";

    this->in_motion = false;
    return;
}
#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"

#include "knights/robot/chassis.h"

#include "knights/util/calculation.h"

#include "knights/logger/logger.h"

#define MIN_SPEED 10

void knights::RobotController::turn_to_angle(const float angle, int direction, float end_tolerance, float timeout, bool rad) {
    if (this->in_motion) return;
    this->in_motion = true;

    int sign = knights::signum(direction);

    float speed,error;
    float prev_error = fabsf(to_rad(angle)); float total_error = 0.0;
    float desired_angle;

    if (rad == true) // inputs provided in rads
        desired_angle = normalize_angle(angle, true);
    else {// inputs provided in degrees
        end_tolerance = to_rad(end_tolerance);
        desired_angle = normalize_angle(to_rad(angle), true);
    }

    if (sign == 0)
        sign = knights::direction(this->chassis->curr_position.heading, desired_angle);
    
    printf("des angle: %lf, at: %lf\n", desired_angle, chassis->curr_position.heading);

    while(std::abs(min_angle(this->chassis->curr_position.heading, desired_angle, true)) > end_tolerance) {

        timeout -= 10;
        if (timeout < 0) break;

        error = std::abs(min_angle(this->chassis->curr_position.heading, desired_angle, true));

        total_error += error;

        speed = this->pid_controller->update(error, total_error, prev_error);

        prev_error = error;

        knights::logger::green(knights::logger::string_format("des angle: %lf, curr angle %lf, error %lf, speed: %lf\n", to_deg(desired_angle), to_deg(this->chassis->curr_position.heading), error, speed));

        this->chassis->drivetrain->velocity_command(-sign * speed, sign * speed);

        if (speed < MIN_SPEED) {
            break;
        }

        pros::delay(10);
    }

    this->chassis->drivetrain->velocity_command(0, 0);

    this->in_motion = false;
    return;
}

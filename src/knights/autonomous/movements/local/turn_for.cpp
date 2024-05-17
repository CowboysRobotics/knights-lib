#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"

#include "knights/robot/chassis.h"

#include "knights/util/calculation.h"

void knights::Robot_Controller::turn_for(const float angle, const float end_tolerance, float timeout) {
    // turn the robot a certain amount of degrees, positive is left, negative is right

    if (this->chassis->drivetrain != nullptr) {
        
        float speed,error;
        float prev_error = fabsf(to_rad(angle)); float total_error = 0.0;

        if (this->use_motor_encoders) {
            // figure out how to do this lol
        } else {
            float desired_angle = normalize_angle(this->chassis->curr_position.heading + to_rad(angle), true);

            while(min_angle(this->chassis->curr_position.heading, desired_angle, true) > end_tolerance) {
                error = min_angle(this->chassis->curr_position.heading, desired_angle, true);

                total_error += error;

                speed = this->pid_controller->update(error, total_error, prev_error);

                prev_error = error;

                this->chassis->drivetrain->velocity_command(signum(angle) * speed, -signum(angle) * speed);

                pros::delay(10);
            }
            
        }
    } else {
        // holonomic code
    }
}
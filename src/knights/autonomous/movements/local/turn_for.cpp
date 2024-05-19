#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"

#include "knights/robot/chassis.h"

#include "knights/util/calculation.h"

void knights::Robot_Controller::turn_for(const float angle, const float end_tolerance, float timeout) {
    // turn the robot a certain amount of degrees, positive is left, negative is right

    if (this->in_motion) return;
    this->in_motion = true;

    if (this->chassis->drivetrain != nullptr) {
        
        float speed,error;
        float prev_error = fabsf(to_rad(angle)); float total_error = 0.0;

        if (this->use_motor_encoders) {
            // use circumfrence of circle divided by 360 times degrees to calculate how much one side would need to rotate
            // then divide by 2 bcuz both sides will be rotating

            float desired_position = (this->chassis->drivetrain->track_width * M_PI / 360) * fabsf(angle) / 2;

            // reset all motor encoders
            this->chassis->drivetrain->right_mtrs->set_encoder_units(pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES);
            this->chassis->drivetrain->left_mtrs->set_encoder_units(pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES);
            this->chassis->drivetrain->right_mtrs->tare_position();
            this->chassis->drivetrain->left_mtrs->tare_position();

            // get position of both sets of motors
            float right_pos = knights::avg(this->chassis->drivetrain->right_mtrs->get_position_all());
            float left_pos = knights::avg(this->chassis->drivetrain->left_mtrs->get_position_all());

            while(fabsf((right_pos + left_pos)/2) < fabsf(desired_position)) {
                // decrease timeout and break if went over
                timeout -= 10;
                if (timeout < 0) break;

                // calculate error, convert position to distance so tuning is the same
                error = this->chassis->drivetrain->position_to_distance(fabsf(desired_position) - fabsf((right_pos + left_pos)/2));

                // integrate error
                total_error += error;

                // use pid formula to calculate speed
                speed = this->pid_controller->update(error, total_error, prev_error);

                // save previous error
                prev_error = error;

                // update positions of motors
                right_pos = knights::avg(this->chassis->drivetrain->right_mtrs->get_position_all());
                left_pos = knights::avg(this->chassis->drivetrain->left_mtrs->get_position_all());

                // send command to drivetrain
                this->chassis->drivetrain->velocity_command(signum(angle) * speed, -signum(angle) * speed);

                // delay
                pros::delay(10);
            }

        } else {
            float desired_angle = normalize_angle(this->chassis->curr_position.heading + to_rad(angle), true);

            // TODO: edge case; person is not turning optimally, min angle will not be right - figure this out

            while(min_angle(this->chassis->curr_position.heading, desired_angle, true) > end_tolerance) {

                timeout -= 10;
                if (timeout < 0) break;

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


    this->in_motion = false;
    return;
}
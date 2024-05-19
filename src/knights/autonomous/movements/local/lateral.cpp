#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"

#include "knights/robot/chassis.h"
#include "knights/robot/drivetrain.h"

#include "knights/util/calculation.h"

void knights::Robot_Controller::lateral_move(const float distance, const float end_tolerance, float timeout) {
    if (this->in_motion) return;
    this->in_motion = true;

    // lateral move the chassis of a robot
    if (this->chassis->drivetrain != nullptr) {
        // move function for differential drive
        float speed,error;
        float prev_error = distance; float total_error = 0.0;

        if (this->use_motor_encoders) {
            // reset motor encoders to 0
            this->chassis->drivetrain->right_mtrs->set_encoder_units(pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES);
            this->chassis->drivetrain->left_mtrs->set_encoder_units(pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_DEGREES);
            this->chassis->drivetrain->right_mtrs->tare_position();
            this->chassis->drivetrain->left_mtrs->tare_position();

            // position that drivetrain motors need to reach
            float desired_position = this->chassis->drivetrain->distance_to_position(distance);

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
                speed = this->pid_controller->update(error, total_error, prev_error) * knights::signum(distance);

                // save previous error
                prev_error = error;

                // update positions of motors
                right_pos = knights::avg(this->chassis->drivetrain->right_mtrs->get_position_all());
                left_pos = knights::avg(this->chassis->drivetrain->left_mtrs->get_position_all());

                // send command to drivetrain
                this->chassis->drivetrain->velocity_command(speed,speed);

                // delay
                pros::delay(10);
            }
        } else {
            // create a variable representing the desired position
            Pos desired_position(cos(this->chassis->curr_position.heading) * distance + this->chassis->curr_position.x, 
                sin(this->chassis->curr_position.heading) * -distance + this->chassis->curr_position.y, this->chassis->curr_position.heading);
            
            while(knights::distance_btwn(this->chassis->curr_position, desired_position) > end_tolerance) {
                // decrease timeout and break if went over
                timeout -= 10;
                if (timeout < 0) break;

                // calculate error
                error = knights::distance_btwn(this->chassis->curr_position, desired_position);

                // integrate error
                total_error += error;

                // use pid formula to calculate speed
                speed = this->pid_controller->update(error, total_error, prev_error) * knights::signum(distance);

                // save previous error
                prev_error = error;

                // send command to drivetrain
                this->chassis->drivetrain->velocity_command(speed,speed);

                // delay
                pros::delay(10);
            }

        }

        this->chassis->drivetrain->right_mtrs->set_brake_mode_all(pros::MotorBrake::brake);
        this->chassis->drivetrain->left_mtrs->set_brake_mode_all(pros::MotorBrake::brake);

        this->chassis->drivetrain->right_mtrs->brake();
        this->chassis->drivetrain->left_mtrs->brake();

    } else {
        // holomic lateral movement code
    }

    this->in_motion = false;
    return;
}
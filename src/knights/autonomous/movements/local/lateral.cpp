#include "knights/autonomous/controller.hpp"
#include "knights/autonomous/pid.hpp"

#include "knights/robot/chassis.hpp"
#include "knights/robot/drivetrain.hpp"

#include "knights/util/calculation.hpp"
#include "knights/util/position.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>

void knights::RobotController::lateral_move(const float distance, const float end_tolerance, float timeout, bool async) {
    if (this->in_motion) return;
    this->in_motion = true;

    if (async) {
        pros::Task lateral([=] {
            this->lateral_move(distance, end_tolerance, timeout, false);
        });
        pros::delay(10);
        return;
    }

    this->chassis->drivetrain->right_mtrs->set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    this->chassis->drivetrain->left_mtrs->set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    this->lateral_pid->reset();

    // lateral move the chassis of a robot
    if (this->chassis->drivetrain != nullptr) {
        // move function for differential drive
        float speed,error;

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

                // use pid formula to calculate speed
                speed = this->lateral_pid->update(error) * knights::signum(distance);

                // update positions of motors
                right_pos = knights::avg(this->chassis->drivetrain->right_mtrs->get_position_all());
                left_pos = knights::avg(this->chassis->drivetrain->left_mtrs->get_position_all());

                // send command to drivetrain
                this->chassis->drivetrain->voltage_command(speed,speed);

                // delay
                pros::delay(10);
            }
        } else {
            // create a variable representing the desired position
            Pos desired_position(cos(this->chassis->curr_position.heading) * distance + this->chassis->curr_position.x, 
                sin(this->chassis->curr_position.heading) * distance + this->chassis->curr_position.y, this->chassis->curr_position.heading);
            
            Pos start_pos(this->chassis->curr_position);

            float max_dist = distance_btwn(start_pos, desired_position) + end_tolerance;

            while (knights::distance_btwn(this->chassis->curr_position, desired_position) > end_tolerance 
                || knights::distance_btwn(this->chassis->prev_position, desired_position) < knights::distance_btwn(this->chassis->curr_position, desired_position)
                ) {

                if (distance_btwn(start_pos, this->chassis->curr_position) > max_dist)
                    break;
                
                // decrease timeout and break if went over
                timeout -= 10;
                if (timeout < 0) break;

                // calculate error
                error = knights::distance_btwn(this->chassis->curr_position, desired_position);

                // use pid formula to calculate speed
                speed = this->lateral_pid->update(error) * knights::signum(distance);

                // if (fabs(speed) <= this->pid_controller->min_velocity-10) {
                //     break;
                // }

                // // --- EXPERIMENTAL
                // float angular_curve = curvature(this->chassis->curr_position, desired_position);
                
                // // calculate right and left speed based on curvature
                // float r_speed = speed * (2 - angular_curve * this->chassis->drivetrain->track_width) / 2;
                // float l_speed = speed * (2 + angular_curve * this->chassis->drivetrain->track_width) / 2;

                // // calculate if one is over max alloted speed (might need to be 127.0 - max speed in pros)
                // float max_curr_speed = std::fmax(fabs(r_speed), fabs(l_speed)) / this->pid_controller->max_velocity; 
                // if (max_curr_speed > 1) {
                //     r_speed /= max_curr_speed;
                //     l_speed /= max_curr_speed;
                // }

                // send command to drivetrain
                this->chassis->drivetrain->voltage_command(speed, speed);

                // delay
                pros::delay(10);
            }

        }

        this->chassis->drivetrain->right_mtrs->move(0);
        this->chassis->drivetrain->left_mtrs->move(0);

    } else {
        // holomic lateral movement code - not done yet
    }

    this->in_motion = false;
    return;
}

void knights::RobotController::curve_move(const knights::Pos point, const bool forwards, const float end_tolerance, float timeout, bool async) {
    if (this->in_motion) return;
    this->in_motion = true;

    if (async) {
        pros::Task lateral([=] {
            this->curve_move(point, end_tolerance, timeout, false);
        });
        pros::delay(10);
        return;
    }

    int dir = (forwards) ? 1 : -1;

    this->chassis->drivetrain->right_mtrs->set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    this->chassis->drivetrain->left_mtrs->set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    this->lateral_pid->reset();

    Pos start_pos(this->chassis->curr_position);

    float max_dist = distance_btwn(start_pos, point) + end_tolerance;

    while (knights::distance_btwn(this->chassis->curr_position, point) > end_tolerance 
        || knights::distance_btwn(this->chassis->prev_position, point) < knights::distance_btwn(this->chassis->curr_position, point)
        || distance_btwn(start_pos, this->chassis->curr_position) > max_dist
        ) {
        
        // decrease timeout and break if went over
        timeout -= 10;
        if (timeout < 0) break;

        knights::Pos curr_position = this->chassis->curr_position;
        if (!forwards) {
            curr_position.heading = knights::normalize_angle(curr_position.heading + M_PI);
        }

        // calculate error
        float error = knights::distance_btwn(curr_position, point);

        // use pid formula to calculate speed
        float speed = this->lateral_pid->update(error);

        // if (fabs(speed) <= this->pid_controller->min_velocity-10) {
        //     break;
        // }

        // // --- EXPERIMENTAL
        // float angular_curve = curvature(curr_position, point);
        
        // // calculate right and left speed based on curvature
        // float r_speed = speed * (2 - angular_curve * this->chassis->drivetrain->track_width) / 2;
        // float l_speed = speed * (2 + angular_curve * this->chassis->drivetrain->track_width) / 2;

        // // calculate if one is over max alloted speed (might need to be 127.0 - max speed in pros)
        // float max_curr_speed = std::fmax(fabs(r_speed), fabs(l_speed)) / this->lateral_pid->max_velocity; 
        // if (max_curr_speed > 1) {
        //     r_speed /= max_curr_speed;
        //     l_speed /= max_curr_speed;
        // }

        float angular_velocity = this->angular_pid->update(angular_error(curr_position.heading, 
            std::atan2(point.y - curr_position.y, point.x - curr_position.x), 
        0), true);

        float r_speed = speed + angular_velocity;
        float l_speed = speed - angular_velocity;

        // send command to drivetrain
        if (forwards)
            this->chassis->drivetrain->voltage_command(r_speed, l_speed);
        else
            this->chassis->drivetrain->voltage_command(-l_speed, -r_speed);

        // delay
        pros::delay(10);
    }

    this->chassis->drivetrain->right_mtrs->move(0);
    this->chassis->drivetrain->left_mtrs->move(0);

    this->in_motion = false;
    return;
}
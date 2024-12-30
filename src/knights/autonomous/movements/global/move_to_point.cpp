#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"
#include "knights/autonomous/pathgen.h"

#include "knights/robot/chassis.h"

#include "knights/util/calculation.h"

#include "knights/util/position.h"

void knights::RobotController::move_to_point(const Pos desired_position, const bool forwards, const float &end_tolerance, float timeout) {
    
    // lateral move the chassis of a robot
    if (this->chassis->drivetrain != nullptr) {
        // move function for differential drive
        float speed,error;
        float prev_error = distance_btwn(desired_position, this->chassis->curr_position); float total_error = 0.0;
        
        while (knights::distance_btwn(this->chassis->curr_position, desired_position) > end_tolerance || 
            knights::distance_btwn(this->chassis->prev_position, desired_position) < knights::distance_btwn(this->chassis->curr_position, desired_position)) {
            // decrease timeout and break if went over
            timeout -= 10;
            if (timeout < 0) break;

            // calculate error
            error = knights::distance_btwn(this->chassis->curr_position, desired_position);

            // integrate error
            total_error += error;

            // use pid formula to calculate speed
            speed = this->pid_controller->update(error, total_error, prev_error);

            // end if speed below minimum
            if (fabs(speed) <= this->pid_controller->min_velocity) {
                break;
            }

            // reverse speed to move backward
            if (!forwards)
                speed *= -1;

            // save previous error
            prev_error = error;

            // calculate angular curve to point we want to go at
            float angular_curve = curvature(this->chassis->curr_position, desired_position);
            if (!forwards) {
                angular_curve = curvature(Pos(this->chassis->curr_position.x, this->chassis->curr_position.y, this->chassis->curr_position.heading-M_PI), desired_position);
            }
            
            // calculate right and left speed based on curvature
            float r_speed = speed * (2 - angular_curve * this->chassis->drivetrain->track_width) / 2;
            float l_speed = speed * (2 + angular_curve * this->chassis->drivetrain->track_width) / 2;

            // calculate if one is over max alloted speed (might need to be 127.0 - max speed in pros)
            float max_curr_speed = std::fmax(fabs(r_speed), fabs(l_speed)) / this->pid_controller->max_velocity; 
            if (max_curr_speed > 1) {
                r_speed /= max_curr_speed;
                l_speed /= max_curr_speed;
            }

            // send command to drivetrain
            this->chassis->drivetrain->velocity_command(r_speed,l_speed);

            // delay
            pros::delay(10);
        }

        // stop drivetrain 
        this->chassis->drivetrain->right_mtrs->move(0);
        this->chassis->drivetrain->left_mtrs->move(0);

    }

    return;
}
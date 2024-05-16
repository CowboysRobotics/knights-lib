#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"

#include "knights/robot/chassis.h"

#include "knights/util/calculation.h"

void knights::Robot_Controller::turn_for(const float angle, const float end_tolerance, float timeout) {
    // turn the robot a certain amount of degrees

    if (this->chassis->drivetrain != nullptr) {
        
        float speed,error;
        float prev_error = fabsf(angle); float total_error = 0.0;

        if (this->use_motor_encoders) {
            // figure out how to do this lol
        } else {
            float desired_angle = this->chassis->curr_position.heading + angle;

            
        }
    } else {
        // holonomic code
    }
}
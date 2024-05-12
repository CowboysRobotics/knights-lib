#include "api.h"

#include "knights/autonomous/pid.h"
#include "knights/util/calculation.h"

knights::PID_Controller::PID_Controller(float kP, float kI, float kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
}

knights::PID_Controller::PID_Controller(float kP, float kI, float kD, float min_velocity, float max_velocity) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->min_velocity = min_velocity;
    this->max_velocity = max_velocity;
}

knights::PID_Controller::PID_Controller() {
    this->kP = 0.0;
    this->kI = 0.0;
    this->kD = 0.0; 
}

float knights::PID_Controller::update(float error, float total_error, float prev_error) {
    return knights::clamp(this->kP * error + this->kI * total_error + this->kD * (error - prev_error), this->min_velocity, this->max_velocity);
}

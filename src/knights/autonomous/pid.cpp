#include "api.h"

#include "knights/autonomous/pid.h"
#include "knights/util/calculation.h"

knights::PIDController::PIDController(float kP, float kI, float kD) 
    : kP(kP), kI(kI), kD(kD), min_velocity(0.0), max_velocity(127.0) {
}

knights::PIDController::PIDController(float kP, float kI, float kD, float min_velocity, float max_velocity) 
    : kP(kP), kI(kI), kD(kD), min_velocity(min_velocity), max_velocity(max_velocity) {
}

knights::PIDController::PIDController() 
    : kP(0.0), kI(0.0), kD(0.0), min_velocity(0.0), max_velocity(127.0) {
}

float knights::PIDController::update(float error, float total_error, float prev_error) {
    return knights::clamp(this->kP * error + this->kI * total_error + this->kD * (error - prev_error), this->min_velocity, this->max_velocity);
}

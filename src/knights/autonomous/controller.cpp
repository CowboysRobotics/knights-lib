#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"
#include "knights/autonomous/ramsete.h"

#include "knights/robot/chassis.h"

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

float knights::PIDController::get_max_speed() {
    return std::fabs(this->max_velocity);
}

float knights::PIDController::get_min_speed() {
    return std::fabs(this->min_velocity);
}


knights::RamseteConstants::RamseteConstants(const float &damping, const float &proportional)
    : damping(damping), proportional(proportional) {
}

knights::RobotController::RobotController(RobotChassis *chassis, PIDController *pid_controller, RamseteConstants *ramsete_constants, bool use_motor_encoders)
    : chassis(chassis), pid_controller(pid_controller), ramsete_constants(ramsete_constants), use_motor_encoders(use_motor_encoders) {
}

knights::RobotController::RobotController(RobotChassis *chassis, PIDController *pid_controller, bool use_motor_encoders)
    : chassis(chassis), pid_controller(pid_controller), use_motor_encoders(use_motor_encoders) {
}



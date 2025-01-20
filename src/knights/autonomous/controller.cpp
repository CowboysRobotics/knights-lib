#include "knights/autonomous/controller.hpp"
#include "knights/autonomous/pid.hpp"
#include "knights/autonomous/ramsete.hpp"

#include "knights/robot/chassis.hpp"

#include "knights/util/calculation.hpp"

knights::PIDConstants::PIDConstants(float kP, float kI, float kD)
    : kP(kP), kI(kI), kD(kD) {}

knights::PIDConstants::PIDConstants()
    : kP(0.0), kI(0.0), kD(0.0) {}

knights::PIDController::PIDController(PIDConstants constants) 
    : kP(constants.kP), kI(constants.kI), kD(constants.kD), min_velocity(0.0), max_velocity(127.0) {
}

knights::PIDController::PIDController(PIDConstants constants, float min_velocity, float max_velocity) 
    : kP(constants.kP), kI(constants.kI), kD(constants.kD), min_velocity(min_velocity), max_velocity(max_velocity) {
}

knights::PIDController::PIDController(float kP, float kI, float kD) 
    : kP(kP), kI(kI), kD(kD), min_velocity(0.0), max_velocity(127.0) {
}

knights::PIDController::PIDController(float kP, float kI, float kD, float min_velocity, float max_velocity) 
    : kP(kP), kI(kI), kD(kD), min_velocity(min_velocity), max_velocity(max_velocity) {
}

knights::PIDController::PIDController() 
    : kP(0.0), kI(0.0), kD(0.0), min_velocity(0.0), max_velocity(127.0) {
}

float knights::PIDController::update(float error) {
    total_error += error;
    float result = knights::clamp(this->kP * error + this->kI * total_error + this->kD * (error - prev_error), this->min_velocity, this->max_velocity);
    prev_error = error;
    return result;
}

void knights::PIDController::add_constant(float key, PIDConstants constants) {
    this->avaliable_constants[key] = constants;
}

void knights::PIDController::switch_values(float target_val) {
    float min_diff = 1e8;
    PIDConstants best_constants;

    for (auto const [key, value] : this->avaliable_constants) {
        if (fabsf(key - target_val) > min_diff) {
            min_diff = fabsf(key - target_val);
            best_constants = value;
        }
    }

    this->kP = best_constants.kP;
    this->kI = best_constants.kI;
    this->kD = best_constants.kD;
}

float knights::PIDController::get_max_speed() {
    return std::fabs(this->max_velocity);
}

float knights::PIDController::get_min_speed() {
    return std::fabs(this->min_velocity);
}

void knights::PIDController::reset() {
    this->total_error = 0;
    this->prev_error = 1e2;
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



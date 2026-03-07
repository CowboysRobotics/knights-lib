#include "knights/autonomous/controller.hpp"
#include "knights/autonomous/pid.hpp"
#include "knights/autonomous/ramsete.hpp"

#include "knights/robot/chassis.hpp"

#include "knights/util/calculation.hpp"

#include <fstream>

knights::PIDConstants::PIDConstants(float kP, float kI, float kD)
    : kP(kP), kI(kI), kD(kD) {}

knights::PIDConstants::PIDConstants()
    : kP(0.0), kI(0.0), kD(0.0) {}

knights::PIDController::PIDController(PIDConstants constants) 
    : kP(constants.kP), kI(constants.kI), kD(constants.kD), min_velocity(-127.0), max_velocity(127.0) {
}

knights::PIDController::PIDController(PIDConstants constants, float min_velocity, float max_velocity, float slew_max) 
    : kP(constants.kP), kI(constants.kI), kD(constants.kD), min_velocity(min_velocity), max_velocity(max_velocity), slew_max(slew_max) {
}

knights::PIDController::PIDController(float kP, float kI, float kD) 
    : kP(kP), kI(kI), kD(kD), min_velocity(-127.0), max_velocity(127.0) {
}

knights::PIDController::PIDController(float kP, float kI, float kD, float min_velocity, float max_velocity, float slew_max) 
    : kP(kP), kI(kI), kD(kD), min_velocity(min_velocity), max_velocity(max_velocity), slew_max(slew_max) {
}

knights::PIDController::PIDController() 
    : kP(0.0), kI(0.0), kD(0.0), min_velocity(-127.0), max_velocity(127.0) {
}

float knights::PIDController::update(float error, bool clamp) {
    // integral windup protection: reset integral on sign change
    if ((error > 0 && prev_error < 0) || (error < 0 && prev_error > 0)) {
        total_error = 0;
    }

    total_error += error;

    // integral windup protection: clamp accumulated integral
    if (integral_max >= 0) {
        total_error = knights::clampf(total_error, -integral_max, integral_max);
    }

    // settled detection: track consecutive iterations with small error change
    if (std::fabs(error - prev_error) < settled_threshold) {
        settled_count++;
    } else {
        settled_count = 0;
    }

    float raw = this->kP * error + this->kI * total_error + this->kD * (error - prev_error);
    float result;
    if (clamp) {
        result = knights::clampf(
            std::fabs(raw), 
            this->min_velocity, 
            this->max_velocity
        ) * knights::signum(raw);
    } else {
        result = raw;
    }
    prev_error = error;
    return result;
}

void knights::PIDController::add_constant(float key, PIDConstants constants) {
    this->avaliable_constants[key] = constants;
}

void knights::PIDController::switch_values(float target_val) {
    float min_diff = 1e8;
    PIDConstants best_constants(this->kP, this->kI, this->kD);

    for (auto const [key, value] : this->avaliable_constants) {
        if (fabsf(key - target_val) < min_diff) {
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
    this->prev_error = 0;
    this->prev_speed = 0;
    this->settled_count = 0;
}

bool knights::PIDController::is_settled() {
    return settled_count >= settled_target;
}

void knights::PIDController::set_integral_max(float max) {
    this->integral_max = max;
}

void knights::PIDController::set_settled_params(float threshold, int count) {
    this->settled_threshold = threshold;
    this->settled_target = count;
}

knights::RamseteConstants::RamseteConstants(const float &damping, const float &proportional, const float &curvature_coefficient, const float &tuner_v, const float &tuner_accel, const float &tuner_static)
    : damping(damping), proportional(proportional), curvature_coefficient(curvature_coefficient), tuner_v(tuner_v), tuner_accel(tuner_accel), tuner_static(tuner_static) {
}

knights::RobotController::RobotController(RobotChassis *chassis, PIDController *lateral_pid, PIDController *turn_pid, RamseteConstants *ramsete_constants, bool use_motor_encoders)
    : chassis(chassis), lateral_pid(lateral_pid), turn_pid(turn_pid), angular_pid(nullptr), ramsete_constants(ramsete_constants), use_motor_encoders(use_motor_encoders) {
}

knights::RobotController::RobotController(RobotChassis *chassis, PIDController *lateral_pid, PIDController *turn_pid, PIDController *angular_pid, RamseteConstants *ramsete_constants, bool use_motor_encoders)
    : chassis(chassis), lateral_pid(lateral_pid), turn_pid(turn_pid), angular_pid(angular_pid), ramsete_constants(ramsete_constants), use_motor_encoders(use_motor_encoders) {
}

knights::RobotController::RobotController(RobotChassis *chassis, PIDController *lateral_pid, PIDController *turn_pid, bool use_motor_encoders)
    : chassis(chassis), lateral_pid(lateral_pid), turn_pid(turn_pid), angular_pid(nullptr), use_motor_encoders(use_motor_encoders), ramsete_constants(new knights::RamseteConstants) {
}

knights::RobotController::RobotController(RobotChassis *chassis, PIDController *lateral_pid, PIDController *turn_pid, PIDController *angular_pid, bool use_motor_encoders)
    : chassis(chassis), lateral_pid(lateral_pid), turn_pid(turn_pid), angular_pid(angular_pid), use_motor_encoders(use_motor_encoders), ramsete_constants(new knights::RamseteConstants) {
};



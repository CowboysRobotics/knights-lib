#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"

#include "knights/robot/chassis.h"

knights::RobotController::RobotController(RobotChassis *chassis, PIDController *pid_controller, bool use_motor_encoders)
    : chassis(chassis), pid_controller(pid_controller), use_motor_encoders(use_motor_encoders) {
}

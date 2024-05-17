#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"

#include "knights/robot/chassis.h"

knights::Robot_Controller::Robot_Controller(Robot_Chassis *chassis, PID_Controller *pid_controller, bool use_motor_encoders)
    : chassis(chassis), pid_controller(pid_controller), use_motor_encoders(use_motor_encoders) {
}

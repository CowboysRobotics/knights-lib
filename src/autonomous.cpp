#include "autonomous.h" 
#include "globals.h"

#include "knights/autonomous/path.hpp"
#include "knights/robot/chassis.hpp"
#include "knights/util/calculation.hpp"
#include "knights/util/position.hpp"
#include "pros/rtos.hpp"

#include <cmath>
#include <fstream>
#include <string>

#define RIGHT 1
#define LEFT -1

#define kPos knights::Pos

#define LATERAL_kP 10
#define LATERAL_kI 0
#define LATERAL_kD 60

#define TURN_kP_45 120
#define TURN_kI_45 0.0
#define TURN_kD_45 600

#define TURN_kP_90 90 // 75 - 45 // 48 - 90 // 38 - 135 // 34 - 180
#define TURN_kI_90 0.0 // 0.017 - 45 // 0.017 - 90 // 0.017 - 135 // 0.017 - 180
#define TURN_kD_90 400 // 0.08 - 45 // 0.24 - 90 // 0.24 - 135 // 0.24 - 180

#define TURN_kP_135 98
#define TURN_kI_135 0.0
#define TURN_kD_135 610

#define TURN_kP_180 105
#define TURN_kI_180 0.0
#define TURN_kD_180 700

#define PROS_MAX_VOLTAGE 127

void pid_tuning(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
    knights::RamseteConstants ramsete_constants(0.7, 2, 1.3);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0, 3.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::PIDController angularPID(50, 0, 10, -25.0, 25.0);

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, &angularPID, &ramsete_constants, false);
	
	knights::ProfileGenerator generator(drivetrain, 60);

	knights::MotionProfile profile = generator.generate(chassis->get_position(), 
		knights::Pos(36.0, 0, 0_deg), 100, 15, true);


	
}

void pp_test(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {

	knights::RamseteConstants ramsete_constants(0, 0, 0);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0, 3.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::PIDController angularPID(50, 0, 10, -25.0, 25.0);

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, &angularPID, &ramsete_constants, false);
	
	knights::ProfileGenerator generator(drivetrain, 50);

	pros::delay(300);

	auto profile = generator.generate(
		chassis->get_position(), knights::Pos(24, 24, 90_deg), 100, 30, true); // off

	profile.dump();

	robotControl.follow_profile(profile, 0);

	// drivetrain.voltage_command(60, 60);

}

void empty(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
	return;
}

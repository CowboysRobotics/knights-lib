#include "autonomous.h" 

#include "knights/api.h"
#include "knights/autonomous/path.h"
#include "knights/robot/chassis.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include "knights/logger/logger.h"
#include <iomanip>

#define RIGHT 1
#define LEFT -1

#define INTAKE_VELOCITY 300

#define LATERAL_kP 6
#define LATERAL_kI 0
#define LATERAL_kD 0.0065

#define TURN_kP 54
#define TURN_kI 0.017
#define TURN_kD 0.002

//assign ports to intake, leftside first, rightside second
pros::MotorGroup intake_auton({9,13}, pros::MotorGears::blue);

//assign ports for pneumatics
pros::adi::Pneumatics clamp_auton(8, false); //clamp solenoid
pros::adi::Pneumatics doinker_auton(6, false); //doinker solenoid
pros::adi::Pneumatics big_arm_auton_section(7,false); //big arm solenoid
pros::adi::Pneumatics small_arm_auton_section(5,false); //small arm solenoid
pros::adi::Pneumatics wall_stake_mech_clamp_auton(6,false); //ring clamp solenoid

bool intake_auton_spinning = false;
bool intake_auton_forward = false;

void intake_in() {
	if (intake_auton_spinning == true && intake_auton_forward == true) { // If intake is on or in wrong direction
		intake_auton.move(0); // stop intake
		intake_auton_spinning = false;
	} else {
		intake_auton.move(INTAKE_VELOCITY); // Spin intake forward
		intake_auton_spinning = true;
		intake_auton_forward = true;
	}
}

void intake_out() {
	if (intake_auton_spinning == true && intake_auton_forward == false) { // If intake is spinning or in the wrong direction
		intake_auton.move(0); // stop intake
		intake_auton_spinning = false;
	} else { 
		intake_auton.move(-INTAKE_VELOCITY); // Spin the intake in reverse
		intake_auton_spinning = true;
		intake_auton_forward = false;
	}
}

bool clamp_auton_down = false;

void clamp_auton_toggle() {
	clamp_auton_down = !clamp_auton_down; //toggle whether active or inactive mode
	clamp_auton.set_value(clamp_auton_down); //activate clamp if inactive or deactivate clamp if active
}

bool doinker_auton_activate = false;

void doinker_auton_toggle() {
	doinker_auton_activate = !doinker_auton_activate; //toggle whether active or inactive mode
	doinker_auton.set_value(doinker_auton_activate); //extend doinker if inactive or retract clamp if active
}

bool arm_auton_extended = false;

void arm_auton_extend() {
	arm_auton_extended = !arm_auton_extended; //toggle whether active or inactive mode
	small_arm_auton_section.set_value(arm_auton_extended); //extend arm if active or retract arm if inactive
}

bool arm_up_auton = false;

void wall_stake_mech_auton() {
	arm_up_auton = !arm_up_auton; //toggle whether active or inactive mode
	big_arm_auton_section.set_value(arm_up_auton); //move arm up if active or move down if inactive
}

bool ring_clamp_auton = false;

void close_arm_auton() {
	ring_clamp_auton = !ring_clamp_auton; //toggle whether active or inactive mode
	wall_stake_mech_clamp_auton.set_value(ring_clamp_auton); //activate arm clamp if active or open arm clamp if inactive
}

void pid_tuning(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
}


void pp_test(knights::RobotChassis *chassis) {

	std::string s = "test-forward.txt";

	knights::AdvancedRoute test_route = advanced_route_from_file(s);
	for (auto route : test_route.routes) {
		for (auto pt : route.second.positions) {
			// knights::logger::green(knights::logger::string_format("pt: %lf %lf %lf", pt.x, pt.y, pt.heading));
		}
	}

	for (auto action : test_route.actions) {
		if (action.type == knights::action_type::LATERAL) {
			knights::logger::yellow(knights::logger::string_format("vals: %lf %lf %lf\n", action.specific, action.end_tolerance, action.timeout));
		}
	}

	chassis->set_position(knights::Pos(-54, 28, 3.14159));


    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
	
	knights::input::AutonomousInputMap inputMap;
    inputMap.bind_action("intakeRev", intake_out);
    inputMap.bind_action("intakeFwd", intake_in);
    inputMap.bind_action("clamp", clamp_auton_toggle);
	inputMap.bind_action("armExtend", arm_auton_extend);
	inputMap.bind_action("wallStake", wall_stake_mech_auton);

	test_route.execute(chassis, &lateralPID, &turnPID, &inputMap);
}

#define WAIT 140

void skills(knights::RobotChassis *chassis) {
	knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
	
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

}

void red_left_wp(knights::RobotChassis *chassis) {
	knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
	
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

}


void red_rush_right_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
    
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

}

void red_right_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
    
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

}


void red_rush_right_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
    
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

}

void blue_right_wp(knights::RobotChassis *chassis) {

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
	
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

}


void blue_rush_left_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
    
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

}

void blue_right_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);
	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
    
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

}


void blue_left_rush_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
    
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

}

void empty(knights::RobotChassis *chassis) {
	return;
}

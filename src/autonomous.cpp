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


//assign ports to intake, leftside first, rightside second
pros::MotorGroup intake_auton({17,10}, pros::MotorGears::blue);

//assign ports for pneumatics
pros::adi::Pneumatics clamp_auton(7, false); //clamp solenoid
pros::adi::Pneumatics doinker_auton(4, false); //doinker solenoid
pros::adi::Pneumatics big_arm_auton_section(8,false); //big arm solenoid
pros::adi::Pneumatics small_arm_auton_section(5,false); //small arm solenoid
pros::adi::Pneumatics wall_stake_mech_clamp_auton(6,false); //ring clamp solenoid

bool intake_auton_spinning = false;
bool intake_auton_forward = false;

void intake_auton_fwd() {
	if (intake_auton_spinning == true && intake_auton_forward == true) { // If intake is on or in wrong direction
		intake_auton.move(0); // stop intake
		intake_auton_spinning = false;
	} else {
		intake_auton.move(INTAKE_VELOCITY); // Spin intake forward
		intake_auton_spinning = true;
		intake_auton_forward = true;
	}
}

void intake_auton_rev() {
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

	// knights::PIDController lateralPID(2.8, 0.00225, 0.0015, 0.0, 127.0);
	knights::PIDController lateralPID(4, 0.0, 0.0065, 25.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	// knights::PIDController turnPID(27, 0.056, 0.00, 0.0, 127.0);
	knights::PIDController turnPID(48, 0.017, 0.002, 25.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    // turnController.turn_to_angle(135,0,3,10000,false);
    lateralController.lateral_move(48.0, 1, 2000);
}


void pp_test(knights::RobotChassis *chassis) {

	std::string s = "left_auton.txt";

	knights::AdvancedRoute test_route = advanced_route_from_file(s);

	// knights::logger::green("started route read");

	// for (auto action : test_route.actions) {
	// 	if (action.type == knights::action_type::FOLLOW)
	// 		// knights::logger::red("follow");
	// 	else if (action.type == knights::action_type::LATERAL)
	// 		// knights::logger::red("lateral");
	// 	else if (action.type == knights::action_type::TURN)
	// 		// knights::logger::red("turn");
	// }

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

	chassis->set_position(knights::Pos(-60, 0, 0));


    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(4, 0.0, 0.0065, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(48, 0.017, 0.002, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
	
	knights::input::AutonomousInputMap inputMap;
    inputMap.bind_action("intakeRev", intake_auton_rev);
    inputMap.bind_action("intakeFwd", intake_auton_fwd);
    inputMap.bind_action("clamp", clamp_auton_toggle);

	test_route.execute(chassis, &lateralPID, &turnPID, &inputMap);
}


void red_rush(knights::RobotChassis *chassis){
    
	knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(2.8, 0.00225, 0.0015, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(27, 0.056, 0.00, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    
	intake_auton.set_reversed(true,0);
	intake_auton.set_reversed(false, 1);


    lateralController.lateral_move(-26,0.5,2000);
	turnController.turn_to_angle(60,LEFT,0.5,500);
	lateralController.lateral_move(-22,0.5,2000);
	clamp_auton_toggle();
	intake_auton_fwd();
	turnController.turn_to_angle(350, RIGHT,0.5,500);
	lateralController.lateral_move(10,0.5,500);



}
#include "autonomous.h" 

#include "knights/api.h"
#include "knights/autonomous/path.h"
#include "knights/robot/chassis.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "knights/logger/logger.h"
#include <iomanip>

#define RIGHT 1
#define LEFT -1

#define INTAKE_VELOCITY 300


//assign ports to intake, leftside first, rightside second
pros::MotorGroup intake_auton({9,13}, pros::MotorGears::blue);
pros::Distance redirect_auton(6);
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

	std::string s = "test-skills.txt";

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

	// chassis->set_position(knights::Pos(-54, 28, 3.14159));


    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(4, 0.0, 0.0065, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(48, 0.017, 0.002, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
	
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

	knights::PIDController lateralPID(6, 0.0, 0.0065, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(54, 0.017, 0.002, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
	
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

	clamp_auton_toggle();

	/* lateralController.lateral_move(-10, 2.0, 750);
	intake_in();

	pros::delay(4*WAIT);

	lateralController.lateral_move(11, 2.0, 750);

	clamp_auton_toggle();

	turnController.turn_to_angle(90, LEFT);

	pros::delay(WAIT);

	lateralController.lateral_move(-12);

	lateralController.lateral_move(-14);

	clamp_auton_toggle();

	pros::delay(WAIT);

	turnController.turn_to_angle(0, RIGHT);

	pros::delay(WAIT);

	lateralController.lateral_move(23);

	pros::delay(WAIT);

	turnController.turn_to_angle(270, RIGHT);

	pros::delay(WAIT);

	lateralController.lateral_move(20);

	pros::delay(WAIT);

	turnController.turn_to_angle(180, RIGHT);

	pros::delay(WAIT);

	lateralController.lateral_move(18);

	pros::delay(2*WAIT);

	lateralController.lateral_move(14);

	pros::delay(2*WAIT);

	lateralController.lateral_move(-10);

	pros::delay(WAIT);

	turnController.turn_to_angle(270, LEFT);

	pros::delay(WAIT);

	lateralController.lateral_move(10);

	pros::delay(2*WAIT);

	lateralController.lateral_move(-20);

	pros::delay(WAIT);

	turnController.turn_to_angle(50, LEFT); // 40

	pros::delay(WAIT);

	lateralController.lateral_move(-25);

	intake_in();

	clamp_auton_toggle();

	pros::delay(WAIT);

	lateralController.lateral_move(14); // 9.5

	pros::delay(WAIT);

	turnController.turn_to_angle(325, RIGHT);

	pros::delay(WAIT);

	turnController.turn_to_angle(270, RIGHT);

	lateralController.lateral_move(-40);

	pros::delay(WAIT);

	lateralController.lateral_move(-20);

	pros::delay(WAIT);

	lateralController.lateral_move(-12);

	clamp_auton_toggle();

	intake_in();

	pros::delay(2*WAIT);

	turnController.turn_to_angle(0, LEFT);

	pros::delay(WAIT);

	lateralController.lateral_move(22);

	pros::delay(WAIT);

	turnController.turn_to_angle(90, LEFT);

	pros::delay(WAIT);

	lateralController.lateral_move(22.5);

	pros::delay(WAIT);

	turnController.turn_to_angle(180, LEFT);

	pros::delay(WAIT);

	lateralController.lateral_move(18);

	pros::delay(2*WAIT);

	lateralController.lateral_move(14);

	pros::delay(2*WAIT);

	lateralController.lateral_move(-12);

	pros::delay(WAIT);

	turnController.turn_to_angle(90, RIGHT);

	pros::delay(WAIT);

	lateralController.lateral_move(10);

	pros::delay(2*WAIT);

	lateralController.lateral_move(-18);

	pros::delay(WAIT);

	turnController.turn_to_angle(315, RIGHT);

	pros::delay(WAIT);

	lateralController.lateral_move(-25);

	intake_in();

	clamp_auton_toggle();

	// go to other side now

	pros::delay(WAIT);

	lateralController.lateral_move(18);

	pros::delay(WAIT);

	turnController.turn_to_angle(10, LEFT);

	pros::delay(WAIT);

	lateralController.lateral_move(50, 3.0, 1500); // 50

	// lateralController.lateral_move(17, 3.0, 1500);

	pros::delay(WAIT);

	turnController.turn_to_angle(310, RIGHT);

	pros::delay(WAIT);

	lateralController.lateral_move(32);

	intake_in();

	pros::delay(450);

	intake_in();

	pros::delay(WAIT);

	turnController.turn_to_angle(120, LEFT);

	pros::delay(WAIT);

	lateralController.lateral_move(-18);

	pros::delay(WAIT);

	lateralController.lateral_move(-14);

	clamp_auton_toggle();

	pros::delay(2*WAIT);

	intake_in();

	lateralController.lateral_move(28);

	pros::delay(WAIT);

	turnController.turn_to_angle(90, RIGHT);

	pros::delay(WAIT);

	lateralController.lateral_move(30);

	pros::delay(WAIT);

	turnController.turn_to_angle(0, RIGHT);

	pros::delay(WAIT);

	lateralController.lateral_move(30);

	pros::delay(2*WAIT);

	lateralController.lateral_move(-13);

	pros::delay(WAIT);

	turnController.turn_to_angle(225, RIGHT);

	pros::delay(WAIT);

	lateralController.lateral_move(-33);

	pros::delay(WAIT);

	clamp_auton_toggle();

	pros::delay(2*WAIT);

	lateralController.lateral_move(30);

	pros::delay(WAIT); */

	// final rush

	turnController.turn_to_angle(117, RIGHT);

	pros::delay(WAIT);

	lateralController.lateral_move(-40);

	pros::delay(WAIT);

	turnController.turn_to_angle(180, LEFT);

	pros::delay(WAIT);

	lateralController.lateral_move(-24);

	pros::delay(WAIT);

	turnController.turn_to_angle(90, RIGHT);

	pros::delay(WAIT);

	lateralController.lateral_move(-50);

	lateralController.lateral_move(10);

	lateralController.lateral_move(-40);

	pros::delay(WAIT);

	turnController.turn_to_angle(180, LEFT);

	pros::delay(WAIT);

	intake_in();

	lateralController.lateral_move(50);

	pros::delay(WAIT);

	turnController.turn_to_angle(270, LEFT);


}


void red_left_wp_new(knights::RobotChassis *chassis) {

	// chassis->set_position(knights::Pos(-56.5, 15.0, 3.95728));

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(6, 0.0, 0.0065, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(54, 0.017, 0.002, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
	
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

	arm_auton_extend();
	pros::delay(2*WAIT);
	wall_stake_mech_auton();
	pros::delay(WAIT);
	lateralController.lateral_move(8);
	wall_stake_mech_auton();
	pros::delay(2*WAIT);
	lateralController.lateral_move(-14);

	pros::delay(WAIT);

	turnController.turn_to_angle(175, RIGHT);

	pros::delay(WAIT);

	lateralController.lateral_move(-18);

	// pros::delay(WAIT);

	lateralController.lateral_move(-12);

	clamp_auton_toggle();

	pros::delay(WAIT);

	turnController.turn_to_angle(60, RIGHT);

	intake_in();

	pros::delay(WAIT);

	lateralController.lateral_move(15);

	pros::delay(2*WAIT);

	lateralController.lateral_move(-10);

	pros::delay(WAIT);

	turnController.turn_to_angle(80, LEFT);

	pros::delay(WAIT);

	lateralController.lateral_move(15);

	pros::delay(2*WAIT);

	lateralController.lateral_move(-8);

	pros::delay(WAIT);

	turnController.turn_to_angle(145, LEFT);

	// clamp_auton_toggle();
	intake_in();

	pros::delay(2*WAIT);

	lateralController.lateral_move(14);
	intake_in();
	pros::delay(500);
	intake_in();

	pros::delay(WAIT);

	turnController.turn_to_angle(0, RIGHT);

	clamp_auton_toggle();

	pros::delay(WAIT);

	// lateralController.lateral_move(4);

	pros::delay(WAIT);

	turnController.turn_to_angle(110, LEFT);

	pros::delay(WAIT);

	lateralController.lateral_move(-30, 3.0, 750);

	pros::delay(WAIT);

	turnController.turn_to_angle(68, RIGHT, 3.0, 300);

	pros::delay(WAIT);

	lateralController.lateral_move(-20);

	turnController.turn_to_angle(45, RIGHT);

	lateralController.lateral_move(-12);

	lateralController.lateral_move(-14);


	clamp_auton_toggle();

	intake_in();

	pros::delay(WAIT);

	turnController.turn_to_angle(345, RIGHT, 3.0, 400);

	pros::delay(WAIT);

	lateralController.lateral_move(12);

}

void red_left_wp_safe(knights::RobotChassis *chassis) {

	// chassis->set_position(knights::Pos(-56.5, 15.0, 3.95728));

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(6, 0.0, 0.0065, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(54, 0.017, 0.002, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
	
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

	arm_auton_extend();
	pros::delay(2*WAIT);
	wall_stake_mech_auton();
	pros::delay(WAIT);
	lateralController.lateral_move(8);
	wall_stake_mech_auton();
	pros::delay(2*WAIT);
	lateralController.lateral_move(-14);

	pros::delay(WAIT);

	turnController.turn_to_angle(175, RIGHT);

	pros::delay(WAIT);

	lateralController.lateral_move(-18);

	// pros::delay(WAIT);

	lateralController.lateral_move(-12);

	clamp_auton_toggle();

	pros::delay(WAIT);

	turnController.turn_to_angle(60, RIGHT);

	intake_in();

	pros::delay(WAIT);

	lateralController.lateral_move(15);

	pros::delay(2*WAIT);

	lateralController.lateral_move(-10);

	pros::delay(WAIT);

	turnController.turn_to_angle(80, LEFT);

	pros::delay(WAIT);

	lateralController.lateral_move(15);

	pros::delay(2*WAIT);

	lateralController.lateral_move(-8);

	pros::delay(WAIT);

	turnController.turn_to_angle(145, LEFT);

	// clamp_auton_toggle();
	intake_in();

	pros::delay(2*WAIT);

	lateralController.lateral_move(14);
	intake_in();
	// pros::delay(500);
	// intake_in();

	pros::delay(WAIT);

	turnController.turn_to_angle(90, RIGHT);

	// clamp_auton_toggle();

	pros::delay(WAIT);

	lateralController.lateral_move(-35);

	// pros::delay(WAIT);

	// turnController.turn_to_angle(120, LEFT);

	// pros::delay(WAIT);

	// lateralController.lateral_move(-30, 3.0, 750);

	// pros::delay(WAIT);

	// turnController.turn_to_angle(68, RIGHT, 3.0, 300);

	// pros::delay(WAIT);

	// lateralController.lateral_move(-20);

	// turnController.turn_to_angle(45, RIGHT);

	// lateralController.lateral_move(-22);

	// clamp_auton_toggle();

	// intake_in();

	// pros::delay(WAIT);

	// turnController.turn_to_angle(345, RIGHT, 3.0, 400);

	// pros::delay(WAIT);

	// lateralController.lateral_move(12);

}


void red_left_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

    knights::PIDController lateralPID(4, 0.0, 0.0065, 25.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(48, 0.017, 0.002, 25.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
    
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);


	arm_auton_extend();
	pros::delay(250);
	wall_stake_mech_auton();
	pros::delay(500);
	lateralController.lateral_move(11);
	wall_stake_mech_auton();
	pros::delay(250);
	lateralController.lateral_move(-10);
	turnController.turn_to_angle(205, RIGHT);
	lateralController.lateral_move(-32);
	lateralController.lateral_move(-10);
	clamp_auton_toggle();
	turnController.turn_to_angle(75, RIGHT);
	intake_in();
	lateralController.lateral_move(10);
	pros::delay(250);
	lateralController.lateral_move(-11);
	turnController.turn_to_angle(55, LEFT);
	lateralController.lateral_move(18);
	lateralController.lateral_move(-20);
	turnController.turn_to_angle(90, LEFT);
	lateralController.lateral_move(18);



}


void red_rush_right_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

    // knights::PIDController lateralPID(2.8, 0.00225, 0.0015, 0.0, 127.0);
    knights::PIDController lateralPID(4, 0.0, 0.0065, 25.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	// knights::PIDController turnPID(27, 0.056, 0.00, 0.0, 127.0);
	knights::PIDController turnPID(48, 0.017, 0.002, 25.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
    
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

}

void blue_right_wp_new(knights::RobotChassis *chassis) {

	// chassis->set_position(knights::Pos(-56.5, 15.0, 3.95728));

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(6, 0.0, 0.0065, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(54, 0.017, 0.002, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
	
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

	arm_auton_extend();
	pros::delay(2*WAIT);
	wall_stake_mech_auton();
	pros::delay(WAIT);
	lateralController.lateral_move(8);
	wall_stake_mech_auton();
	pros::delay(2*WAIT);
	lateralController.lateral_move(-14);

	pros::delay(WAIT);

	turnController.turn_to_angle(-1*175, -1*RIGHT);

	pros::delay(WAIT);

	lateralController.lateral_move(-18);

	// pros::delay(WAIT);

	lateralController.lateral_move(-12);

	clamp_auton_toggle();

	pros::delay(WAIT);

	turnController.turn_to_angle(-1*60, -1*RIGHT);

	intake_in();

	pros::delay(WAIT);

	lateralController.lateral_move(15);

	pros::delay(2*WAIT);

	lateralController.lateral_move(-10);

	pros::delay(WAIT);

	turnController.turn_to_angle(-1*80, -1*LEFT);

	pros::delay(WAIT);

	lateralController.lateral_move(15);

	pros::delay(2*WAIT);

	lateralController.lateral_move(-8);

	pros::delay(WAIT);

	turnController.turn_to_angle(-1*145, -1*LEFT);

	// clamp_auton_toggle();
	intake_in();

	pros::delay(2*WAIT);

	lateralController.lateral_move(14);
	intake_in();
	pros::delay(500);
	intake_in();

	pros::delay(WAIT);

	turnController.turn_to_angle(-1*0, -1*RIGHT);

	clamp_auton_toggle();

	pros::delay(WAIT);

	// lateralController.lateral_move(4);

	pros::delay(WAIT);

	turnController.turn_to_angle(-1*120, -1*LEFT);

	pros::delay(WAIT);

	lateralController.lateral_move(-30, 3.0, 750);

	pros::delay(WAIT);

	turnController.turn_to_angle(-1*68, -1*RIGHT, 3.0, 300);

	pros::delay(WAIT);

	lateralController.lateral_move(-20);

	turnController.turn_to_angle(-1*45,-1* RIGHT);

	lateralController.lateral_move(-22);

	clamp_auton_toggle();

	intake_in();

	pros::delay(WAIT);

	turnController.turn_to_angle(-1*345, -1*RIGHT, 3.0, 400);

	pros::delay(WAIT);

	lateralController.lateral_move(12);

}

void blue_right_wp_safe(knights::RobotChassis *chassis) {

	// chassis->set_position(knights::Pos(-56.5, 15.0, 3.95728));

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(6, 0.0, 0.0065, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(54, 0.017, 0.002, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
	
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

	arm_auton_extend();
	pros::delay(2*WAIT);
	wall_stake_mech_auton();
	pros::delay(WAIT);
	lateralController.lateral_move(8);
	wall_stake_mech_auton();
	pros::delay(2*WAIT);
	lateralController.lateral_move(-14);

	pros::delay(WAIT);

	turnController.turn_to_angle(-1*175, -1*RIGHT);

	pros::delay(WAIT);

	lateralController.lateral_move(-18);

	// pros::delay(WAIT);

	lateralController.lateral_move(-12);

	clamp_auton_toggle();

	pros::delay(WAIT);

	turnController.turn_to_angle(-1*60, -1*RIGHT);

	intake_in();

	pros::delay(WAIT);

	lateralController.lateral_move(15);

	pros::delay(2*WAIT);

	lateralController.lateral_move(-10);

	pros::delay(WAIT);

	turnController.turn_to_angle(-1*80, -1*LEFT);

	pros::delay(WAIT);

	lateralController.lateral_move(15);

	pros::delay(2*WAIT);

	lateralController.lateral_move(-8);

	pros::delay(WAIT);

	turnController.turn_to_angle(-1*145, -1*LEFT);

	// clamp_auton_toggle();
	intake_in();

	pros::delay(2*WAIT);

	lateralController.lateral_move(14);
	intake_in();
	// pros::delay(500);
	// intake_in();

	pros::delay(WAIT);

	turnController.turn_to_angle(-1*270, -1*RIGHT);

	// clamp_auton_toggle();

	pros::delay(WAIT);

	lateralController.lateral_move(30);

}


void blue_right_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

    // knights::PIDController lateralPID(2.8, 0.00225, 0.0015, 0.0, 127.0);
    knights::PIDController lateralPID(4, 0.0, 0.0065, 25.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	// knights::PIDController turnPID(27, 0.056, 0.00, 0.0, 127.0);
	knights::PIDController turnPID(48, 0.017, 0.002, 25.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
     
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);
	

   lateralController.lateral_move(-29);
	
	clamp_auton_toggle();
	pros::delay(150);
	intake_in();
	turnController.turn_to_angle(-133, -1*RIGHT);
	pros::delay(100);

	lateralController.lateral_move(23);
	pros::delay(100);
	lateralController.lateral_move(-8);
	turnController.turn_to_angle(-140, -1*LEFT);
    lateralController.lateral_move(17);
	pros::delay(250);
	lateralController.lateral_move(-8);
	turnController.turn_to_angle(-235, -1*LEFT);
    lateralController.lateral_move(16);
}


void blue_rush_left_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

    // knights::PIDController lateralPID(2.8, 0.00225, 0.0015, 0.0, 127.0);
    knights::PIDController lateralPID(4, 0.0, 0.0065, 25.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	// knights::PIDController turnPID(27, 0.056, 0.00, 0.0, 127.0);
	knights::PIDController turnPID(48, 0.017, 0.002, 25.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
    
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

}


void red_right_elim(knights::RobotChassis *chassis) {    
	chassis->set_position(knights::Pos(-48.0, -60.0, 3.1415926535));

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(6, 0.0, 0.0065, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(54, 0.017, 0.002, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
	
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

}


void red_rush_right_elim(knights::RobotChassis *chassis) {
    
	chassis->set_position(knights::Pos(-48.0, -60.0, 3.1415926535));

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(6, 0.0, 0.0065, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(54, 0.017, 0.002, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
	
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

	lateralController.lateral_move(-25);
	turnController.turn_to_angle(220, LEFT);
	lateralController.lateral_move(-10);
	lateralController.lateral_move(-9);
	clamp_auton_toggle();
	intake_in();
	turnController.turn_to_angle(165, RIGHT);
	lateralController.lateral_move(14);
	clamp_auton_toggle();
	pros::delay(WAIT/3);
	intake_in();
	turnController.turn_to_angle(270,LEFT);
	pros::delay(WAIT);
	lateralController.lateral_move(-14);
	lateralController.lateral_move(-10);
	clamp_auton_toggle();
	intake_in();
	pros::delay(WAIT);

	turnController.turn_to_angle(325, RIGHT);

	pros::delay(WAIT);

	// lateralController.lateral_move(30);

	// wall_stake_mech_auton();
	// clamp_auton_toggle();
	// pros::delay(WAIT);
	// lateralController.lateral_move(9);
	// pros::delay(WAIT);
	// lateralController.lateral_move(18);
	// pros::delay(3.5 * WAIT);
	// while (redirect_auton.get_distance() > 45) {
	// 	pros::delay(10);
	// }
	// intake_in();
	// arm_auton_extend();
	// pros::delay(3*WAIT);
	// intake_out();
	// pros::delay(WAIT);
	// intake_out();
	// pros::delay(5*WAIT);
	// wall_stake_mech_auton();
	// intake_in();
	// pros::delay(3*WAIT);
	// turnController.turn_to_angle(200, LEFT);
	// pros::delay(WAIT);
	// lateralController.lateral_move(8);
	// pros::delay(3*WAIT);
	// wall_stake_mech_auton();
	// pros::delay(3*WAIT);
	// lateralController.lateral_move(-8);

	// // pros::delay(WAIT);
	// // turnController.turn_to_angle(215, LEFT);
	// // pros::delay(WAIT);
	// // lateralController.lateral_move(-20);




	// // pros::delay(WAIT);
	// // turnController.turn_to_angle(130, RIGHT);
	// // pros::delay(WAIT);
	// // lateralController.lateral_move(-8);
}

void blue_right_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

    // knights::PIDController lateralPID(2.8, 0.00225, 0.0015, 0.0, 127.0);
    knights::PIDController lateralPID(4, 0.0, 0.0065, 25.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	// knights::PIDController turnPID(27, 0.056, 0.00, 0.0, 127.0);
	knights::PIDController turnPID(48, 0.017, 0.002, 25.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
    
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

}


void blue_left_rush_elim(knights::RobotChassis *chassis) {

	chassis->set_position(knights::Pos(-48.0, -60.0, -3.1415926535));

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(6, 0.0, 0.0065, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(54, 0.017, 0.002, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
	
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

	lateralController.lateral_move(-25);
	turnController.turn_to_angle(-220, -1*LEFT);
	lateralController.lateral_move(-10);
	lateralController.lateral_move(-9);
	clamp_auton_toggle();
	intake_in();
	turnController.turn_to_angle(-165, -1*RIGHT);
	lateralController.lateral_move(14);
	clamp_auton_toggle();
	pros::delay(WAIT/3);
	intake_in();
	turnController.turn_to_angle(-270,-1*LEFT);
	pros::delay(WAIT);
	lateralController.lateral_move(-14);
	lateralController.lateral_move(-10);
	clamp_auton_toggle();
	intake_in();
	pros::delay(WAIT);

	turnController.turn_to_angle(225, LEFT);

	// pros::delay(WAIT);

	// lateralController.lateral_move(30);

	// wall_stake_mech_auton();

	// clamp_auton_toggle();
	// pros::delay(WAIT);
	// lateralController.lateral_move(9);
	// pros::delay(WAIT);
	// lateralController.lateral_move(18);
	// pros::delay(3.5 * WAIT);
	// while (redirect_auton.get_distance() > 45) {
	// 	pros::delay(10);
	// }
	// intake_in();
	// arm_auton_extend();
	// pros::delay(3*WAIT);
	// intake_out();
	// pros::delay(WAIT);
	// intake_out();
	// pros::delay(5*WAIT);
	// wall_stake_mech_auton();
	// intake_in();
	// pros::delay(3*WAIT);
	// turnController.turn_to_angle(-200, -1*LEFT);
	// pros::delay(WAIT);
	// lateralController.lateral_move(8);
	// pros::delay(3*WAIT);
	// wall_stake_mech_auton();
	// pros::delay(3*WAIT);
	// lateralController.lateral_move(-12);
	// pros::delay(WAIT);
	// turnController.turn_to_angle(-1*215, -1*LEFT);
	// pros::delay(WAIT);
	// lateralController.lateral_move(-20);
	// // pros::delay(WAIT);
	// // turnController.turn_to_angle(-1*130, -1*RIGHT);
	// // pros::delay(WAIT);
	// // lateralController.lateral_move(-8);

}

void empty(knights::RobotChassis *chassis) {
	knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(6, 0.0, 0.0065, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(54, 0.017, 0.002, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);
	
	intake_auton.set_reversed(false,0);
	intake_auton.set_reversed(true, 1);

	pros::delay(500);

	lateralController.lateral_move(-12);
}

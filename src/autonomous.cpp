#include "autonomous.h" 
#include "hold-cold-asset/asset.hpp"
#include "knights/autonomous/path.h"

#define RIGHT 1
#define LEFT -1

#define LATERAL_kP 5
#define LATERAL_kI 0
#define LATERAL_kD 0.0065

#define TURN_kP_45 75
#define TURN_kI_45 0.017
#define TURN_kD_45 0.08

#define TURN_kP_90 48 // 75 - 45 // 48 - 90 // 38 - 135 // 34 - 180
#define TURN_kI_90 0.017 // 0.017 - 45 // 0.017 - 90 // 0.017 - 135 // 0.017 - 180
#define TURN_kD_90 0.24 // 0.08 - 45 // 0.24 - 90 // 0.24 - 135 // 0.24 - 180

#define TURN_kP_135 38
#define TURN_kI_135 0.017
#define TURN_kD_135 0.24

#define TURN_kP_180 34
#define TURN_kI_180 0.017
#define TURN_kD_180 0.24

ASSET(output_txt);

void pid_tuning(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID_45(TURN_kP_45, TURN_kI_45, TURN_kD_45, 10.0, 127.0);
	knights::RobotController turnController_45(chassis, &turnPID_45);

	knights::PIDController turnPID_90(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	knights::RobotController turnController_90(chassis, &turnPID_90);

	knights::PIDController turnPID_135(TURN_kP_135, TURN_kI_135, TURN_kD_135, 10.0, 127.0);
	knights::RobotController turnController_135(chassis, &turnPID_135);

	knights::PIDController turnPID_180(TURN_kP_180, TURN_kI_180, TURN_kD_180, 10.0, 127.0);
	knights::RobotController turnController_180(chassis, &turnPID_180);

	// lateralController.lateral_move(24);
	turnController_90.turn_to_angle(270, 0);

	turnController_180.turn_to_angle(90, 0);
}


void pp_test(knights::RobotChassis *chassis) {

	std::string s = "test-skills.txt";

	knights::AdvancedRoute test_route = advanced_route_from_file(s);
	// for (auto route : test_route.routes) {
	// 	for (auto pt : route.second.positions) {
	// 		// knights::logger::green(knights::logger::string_format("pt: %lf %lf %lf", pt.x, pt.y, pt.heading));
	// 	}
	// }

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 100.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID_45(TURN_kP_45, TURN_kI_45, TURN_kD_45, 10.0, 127.0);
	knights::RobotController turnController_45(chassis, &turnPID_45);

	knights::PIDController turnPID_90(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	knights::RobotController turnController_90(chassis, &turnPID_90);

	knights::PIDController turnPID_135(TURN_kP_135, TURN_kI_135, TURN_kD_135, 10.0, 127.0);
	knights::RobotController turnController_135(chassis, &turnPID_135);

	knights::PIDController turnPID_180(TURN_kP_180, TURN_kI_180, TURN_kD_180, 10.0, 127.0);
	knights::RobotController turnController_180(chassis, &turnPID_180);
	
	knights::input::AutonomousInputMap inputMap;
    inputMap.bind_action("intakeRev", intake_out);
    inputMap.bind_action("intakeFwd", intake_in);
    inputMap.bind_action("clamp", clamp_toggle);
	inputMap.bind_action("lbDown", lady_brown_down);
	inputMap.bind_action("lbLoad1", lady_brown_load1);
	inputMap.bind_action("lbScore", lady_brown_score);
	inputMap.bind_action("rushMech", toggle_rush_mech);

	test_route.execute(chassis, &lateralPID, &turnPID_90, &inputMap);
}

#define WAIT 140

#define w pros::delay(WAIT)
#define tw(x) pros::delay((x)*WAIT)

void skills(knights::RobotChassis *chassis) {
	knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::Route otherside_1 = knights::init_route_from_sd("os_skills.txt");

	for (auto pos : otherside_1.positions) {
		printf("pos %lf %lf read\n", pos.x, pos.y);
	}

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID_45(TURN_kP_45, TURN_kI_45, TURN_kD_45, 10.0, 127.0);
	knights::RobotController turnController_45(chassis, &turnPID_45);

	knights::PIDController turnPID_90(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	knights::RobotController turnController_90(chassis, &turnPID_90);

	knights::PIDController turnPID_135(TURN_kP_135, TURN_kI_135, TURN_kD_135, 10.0, 127.0);
	knights::RobotController turnController_135(chassis, &turnPID_135);

	knights::PIDController turnPID_180(TURN_kP_180, TURN_kI_180, TURN_kD_180, 10.0, 127.0);
	knights::RobotController turnController_180(chassis, &turnPID_180);

	lateralController.lateral_move(-8);

	// intake_in();

	tw(4);

	// intake_in();

	lateralController.lateral_move(12); w;

	turnController_90.turn_to_angle(90); w;

	lateralController.lateral_move(-24); tw(2);

	clamp_toggle(); tw(2);
	// intake_in();

	turnController_90.turn_to_angle(0); w;

	lateralController.lateral_move(20); w;

	turnController_45.turn_to_angle(330); w;

	lateralController.lateral_move(50, 3.0, 1500); tw(3);

	lady_brown_load1();
	intake_in();

	turnController_90.turn_to_angle(220); w;

	lateralController.lateral_move(24); tw(3);

	turnController_45.turn_to_angle(270); w;

	lateralController.lateral_move(12); w;

	lady_brown_score(); tw(5); lady_brown_down(); intake_in();

	lateralController.lateral_move(-12); w;

	turnController_90.turn_to_angle(180); w;

	lateralController.lateral_move(56, 3.0, 2000); tw(3);

	lateralController.lateral_move(-12); w;

	turnController_90.turn_to_angle(270); w;

	lateralController.lateral_move(16); tw(2); w;

	turnController_90.turn_to_angle(0); w;

	lateralController.lateral_move(-8); w;

	clamp_toggle(); tw(3);







}

void red_left_wp(knights::RobotChassis *chassis) {

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 100.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID_45(TURN_kP_45, TURN_kI_45, TURN_kD_45, 10.0, 127.0);
	knights::RobotController turnController_45(chassis, &turnPID_45);

	knights::PIDController turnPID_90(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	knights::RobotController turnController_90(chassis, &turnPID_90);

	knights::PIDController turnPID_135(TURN_kP_135, TURN_kI_135, TURN_kD_135, 10.0, 127.0);
	knights::RobotController turnController_135(chassis, &turnPID_135);

	knights::PIDController turnPID_180(TURN_kP_180, TURN_kI_180, TURN_kD_180, 10.0, 127.0);
	knights::RobotController turnController_180(chassis, &turnPID_180);
	
	toggle_rush_mech();

	lateralController.lateral_move(20);

	pros::delay(WAIT);

	turnController_45.turn_to_angle(32, 0);

	pros::delay(WAIT);

	lateralController.lateral_move(19);

	pros::delay(WAIT);

	lateralController.lateral_move(-12);

	pros::delay(WAIT);

	toggle_rush_mech();

	turnController_90.turn_to_angle(100, 0);

	pros::delay(WAIT);

	lateralController.lateral_move(-16);

	clamp_toggle();

	pros::delay(WAIT);

	intake_in();

	pros::delay(2*WAIT);

	lateralController.lateral_move(24);
}


void red_rush_right_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID_45(TURN_kP_45, TURN_kI_45, TURN_kD_45, 10.0, 127.0);
	knights::RobotController turnController_45(chassis, &turnPID_45);

	knights::PIDController turnPID_90(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	knights::RobotController turnController_90(chassis, &turnPID_90);

	knights::PIDController turnPID_135(TURN_kP_135, TURN_kI_135, TURN_kD_135, 10.0, 127.0);
	knights::RobotController turnController_135(chassis, &turnPID_135);

	knights::PIDController turnPID_180(TURN_kP_180, TURN_kI_180, TURN_kD_180, 10.0, 127.0);
	knights::RobotController turnController_180(chassis, &turnPID_180);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}

void red_right_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID_45(TURN_kP_45, TURN_kI_45, TURN_kD_45, 10.0, 127.0);
	knights::RobotController turnController_45(chassis, &turnPID_45);

	knights::PIDController turnPID_90(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	knights::RobotController turnController_90(chassis, &turnPID_90);

	knights::PIDController turnPID_135(TURN_kP_135, TURN_kI_135, TURN_kD_135, 10.0, 127.0);
	knights::RobotController turnController_135(chassis, &turnPID_135);

	knights::PIDController turnPID_180(TURN_kP_180, TURN_kI_180, TURN_kD_180, 10.0, 127.0);
	knights::RobotController turnController_180(chassis, &turnPID_180);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}


void red_rush_right_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID_45(TURN_kP_45, TURN_kI_45, TURN_kD_45, 10.0, 127.0);
	knights::RobotController turnController_45(chassis, &turnPID_45);

	knights::PIDController turnPID_90(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	knights::RobotController turnController_90(chassis, &turnPID_90);

	knights::PIDController turnPID_135(TURN_kP_135, TURN_kI_135, TURN_kD_135, 10.0, 127.0);
	knights::RobotController turnController_135(chassis, &turnPID_135);

	knights::PIDController turnPID_180(TURN_kP_180, TURN_kI_180, TURN_kD_180, 10.0, 127.0);
	knights::RobotController turnController_180(chassis, &turnPID_180);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}

void blue_right_wp(knights::RobotChassis *chassis) {

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID_45(TURN_kP_45, TURN_kI_45, TURN_kD_45, 10.0, 127.0);
	knights::RobotController turnController_45(chassis, &turnPID_45);

	knights::PIDController turnPID_90(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	knights::RobotController turnController_90(chassis, &turnPID_90);

	knights::PIDController turnPID_135(TURN_kP_135, TURN_kI_135, TURN_kD_135, 10.0, 127.0);
	knights::RobotController turnController_135(chassis, &turnPID_135);

	knights::PIDController turnPID_180(TURN_kP_180, TURN_kI_180, TURN_kD_180, 10.0, 127.0);
	knights::RobotController turnController_180(chassis, &turnPID_180);
	
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}


void blue_rush_left_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID_45(TURN_kP_45, TURN_kI_45, TURN_kD_45, 10.0, 127.0);
	knights::RobotController turnController_45(chassis, &turnPID_45);

	knights::PIDController turnPID_90(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	knights::RobotController turnController_90(chassis, &turnPID_90);

	knights::PIDController turnPID_135(TURN_kP_135, TURN_kI_135, TURN_kD_135, 10.0, 127.0);
	knights::RobotController turnController_135(chassis, &turnPID_135);

	knights::PIDController turnPID_180(TURN_kP_180, TURN_kI_180, TURN_kD_180, 10.0, 127.0);
	knights::RobotController turnController_180(chassis, &turnPID_180);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}

void blue_right_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);
	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID_45(TURN_kP_45, TURN_kI_45, TURN_kD_45, 10.0, 127.0);
	knights::RobotController turnController_45(chassis, &turnPID_45);

	knights::PIDController turnPID_90(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	knights::RobotController turnController_90(chassis, &turnPID_90);

	knights::PIDController turnPID_135(TURN_kP_135, TURN_kI_135, TURN_kD_135, 10.0, 127.0);
	knights::RobotController turnController_135(chassis, &turnPID_135);

	knights::PIDController turnPID_180(TURN_kP_180, TURN_kI_180, TURN_kD_180, 10.0, 127.0);
	knights::RobotController turnController_180(chassis, &turnPID_180);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}


void blue_left_rush_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID_45(TURN_kP_45, TURN_kI_45, TURN_kD_45, 10.0, 127.0);
	knights::RobotController turnController_45(chassis, &turnPID_45);

	knights::PIDController turnPID_90(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	knights::RobotController turnController_90(chassis, &turnPID_90);

	knights::PIDController turnPID_135(TURN_kP_135, TURN_kI_135, TURN_kD_135, 10.0, 127.0);
	knights::RobotController turnController_135(chassis, &turnPID_135);

	knights::PIDController turnPID_180(TURN_kP_180, TURN_kI_180, TURN_kD_180, 10.0, 127.0);
	knights::RobotController turnController_180(chassis, &turnPID_180);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}

void empty(knights::RobotChassis *chassis) {
	return;
}

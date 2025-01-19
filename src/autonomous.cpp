#include "autonomous.h" 
#include "globals.h"
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

	lateralController.lateral_move(-8); w;

	intake_in();

	tw(4);

	intake_in();

	lateralController.lateral_move(11); w;

	turnController_90.turn_to_angle(90); w;

	lateralController.lateral_move(-12); tw(1); lateralController.lateral_move(-12); tw(2);

	clamp_toggle(); tw(2);
	intake_in();

	turnController_90.turn_to_angle(0); w;

	lateralController.lateral_move(23); w;

	turnController_90.turn_to_angle(270); w;

	lateralController.lateral_move(20); w;

	turnController_90.turn_to_angle(180); w;

	lateralController.lateral_move(18); w;

	lateralController.lateral_move(16); w;

	pros::delay(800);

	lateralController.lateral_move(-12); w;

	turnController_90.turn_to_angle(270); w;

	lateralController.lateral_move(12); w;

	pros::delay(800);

	lateralController.lateral_move(-16); w;

	turnController_135.turn_to_angle(40); w;

	lateralController.lateral_move(-20); w;

	clamp_toggle();

	pros::delay(600);

	lateralController.lateral_move(13); w;

	turnController_135.turn_to_angle(270); w;

	lateralController.lateral_move(-48, 3.0, 1250); w;

	lateralController.lateral_move(-14); w;

	clamp_toggle();

	turnController_90.turn_to_angle(0); w;

	lateralController.lateral_move(23); w;

	turnController_90.turn_to_angle(-270); w;

	lateralController.lateral_move(20); w;

	turnController_90.turn_to_angle(-180); w;

	lateralController.lateral_move(18); w;

	lateralController.lateral_move(16); w;

	pros::delay(800);

	lateralController.lateral_move(-12); w;

	turnController_90.turn_to_angle(-270); w;

	lateralController.lateral_move(12); w;

	pros::delay(800);

	lateralController.lateral_move(-16); w;

	turnController_135.turn_to_angle(-40); w;

	lateralController.lateral_move(-20); w;

	clamp_toggle();

	pros::delay(600);

	lateralController.lateral_move(10); w;

	// turnController_45.turn_to_angle(315); w;

	// lady_brown_load1();

	// lateralController.lateral_move(28); w;

	// turnController_45.turn_to_angle(270);

	// lateralController.lateral_move(16); w;

	// pros::delay(1000);

	// lady_brown_score();

	// pros::delay(1000);

	// lateralController.lateral_move(-6); w;

	// lady_brown_down(); intake_in(); w;

	// turnController_90.turn_to_angle(180); w;

	// lateralController.lateral_move(34, 3.0, 1250); w;

	// lateralController.lateral_move(-12); w;

	// turnController_90.turn_to_angle(270); w;

	// lateralController.lateral_move(16); pros::delay(800);

	// turnController_90.turn_to_angle(20); w;

	// lateralController.lateral_move(-8); w;

	// clamp_toggle(); tw(3);

	// lateralController.lateral_move(8); w;

// 	turnController_90.turn_to_angle(275); w;

// 	lateralController.lateral_move(-50, 3.0, 1400); tw(1);

//     lateralController.lateral_move(-25, 3.0, 1400); tw(1);

//     clamp_toggle(); tw(3);

//     turnController_90.turn_to_angle(0); w;

//    lateralController.lateral_move(22); w;

//    turnController_90.turn_to_angle(90); w;

//    lateralController.lateral_move(24); w;

//    turnController_90.turn_to_angle(180); w;
   
//    lateralController.lateral_move(20); w;

//    lateralController.lateral_move(13); w;



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

	turnController_45.turn_to_angle(27, 0);

	pros::delay(WAIT);

	lateralController.lateral_move(19);

	pros::delay(WAIT);

	lateralController.lateral_move(-14);

	pros::delay(WAIT);

	toggle_rush_mech();

	turnController_90.turn_to_angle(100, 0);

	pros::delay(WAIT);

	lateralController.lateral_move(-13);

	clamp_toggle();

	pros::delay(2*WAIT);

	intake_in();

	pros::delay(2*WAIT);

	turnController_45.turn_to_angle(28,0);

	w;

	lateralController.lateral_move(11);

	w;
	
	turnController_45.turn_to_angle(80);
	
	w;

	lateralController.lateral_move(19);

	w;

    pros::delay(500);

    lateralController.lateral_move(13);

	w;

    lateralController.lateral_move(-20);

	w;


	turnController_45.turn_to_angle(157,0);

	w;

	lateralController.lateral_move(30);

	w;
	turnController_45.turn_to_angle(195,0);

	w;

	turnController_45.turn_to_angle(136,0);

	w;

	lateralController.lateral_move(17);

	w;

    lateralController.lateral_move(4);

	w;


	lateralController.lateral_move(-10);

	w;

	turnController_135.turn_to_angle(270, 0);

	w;

	lady_brown_load1();

	w;

	lateralController.lateral_move(29);

	w;

	turnController_45.turn_to_angle(226,0);

	w;

	lateralController.lateral_move(10);

	w;

	lady_brown_alliance();


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
    
	lady_brown_load1(); w; intake_in();

	turnController_45.turn_to_angle(135); w;

	intake_in(); w;

	lateralController.lateral_move(9); w;

	// lady_brown_score();
	lady_brown_alliance(); tw(5); lady_brown.move(0);

	lateralController.lateral_move(-6); w;

	tw(4); lady_brown.move(-127);
	turnController_45.turn_to_angle(165); w;

	lateralController.lateral_move(-18); w; lateralController.lateral_move(-15); tw(2);

	clamp_toggle(); w; lady_brown.move(0); w;

	turnController_90.turn_to_angle(270); w;

	intake_in(); w;

	lateralController.lateral_move(26); w;

	turnController_90.turn_to_angle(195); w;

	lateralController.lateral_move(28); w;

	turnController_45.turn_to_angle(223); w;

	drivetrain.velocity_command(50, 50);

	pros::delay(1000);

	drivetrain.velocity_command(0, 0);

	lateralController.lateral_move(-12); pros::delay(1000);

	turnController_180.turn_to_angle(45); w;

	pros::delay(250);

	// clamp_toggle(); tw(2);

	pros::delay(250);

	// lady_brown_score();

	lateralController.lateral_move(36);

	lateralController.lateral_move(16); w;

	// intake_in(); tw(2);
}

void alt_skills(knights::RobotChassis *chassis) {
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

	lateralController.lateral_move(-8); w;

	intake_in();

	tw(4);

	intake_in();

	lateralController.lateral_move(11); w;

	turnController_90.turn_to_angle(90, 0); w;

	lateralController.lateral_move(-12); tw(1); lateralController.lateral_move(-12); tw(2);

	clamp_toggle(); tw(2);

	intake_in();

	turnController_90.turn_to_angle(0, 0); w;

	turnController_45.turn_to_angle(315, 0); w;

	lateralController.lateral_move(27); w;

	lateralController.lateral_move(27); w;

	turnController_45.turn_to_angle(270,0); w;

	turnController_90.turn_to_angle(180,0); w;

	lateralController.lateral_move(24); w;

	lateralController.lateral_move(24); w;

	lateralController.lateral_move(-12); w;

	turnController_90.turn_to_angle(270, 0); w;

	lateralController.lateral_move(12); w;

	lateralController.lateral_move(-24); w;

	turnController_45.turn_to_angle(225,0); w;

	lateralController.lateral_move(26); w;

	clamp_toggle(); tw(2);

	lateralController.lateral_move(-26); w;

	turnController_45.turn_to_angle(270,0); w;

	lateralController.lateral_move(-24); w;

	lateralController.lateral_move(-24); w;

	lateralController.lateral_move(-24); w;

	clamp_toggle(); tw(2);

	turnController_90.turn_to_angle(0); w;

	turnController_45.turn_to_angle(45); w;

	lateralController.lateral_move(27); w;

	lateralController.lateral_move(27); w;

	turnController_45.turn_to_angle(90); w;

	turnController_90.turn_to_angle(180); w;

	lateralController.lateral_move(24); w;

	lateralController.lateral_move(24); w;

	lateralController.lateral_move(-12); w;

	turnController_90.turn_to_angle(90);
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

	turnController_45.turn_to_angle(-1*27, 0);

	pros::delay(WAIT);

	lateralController.lateral_move(19);

	pros::delay(WAIT);

	lateralController.lateral_move(-14);

	pros::delay(WAIT);

	toggle_rush_mech();

	turnController_90.turn_to_angle(-1*100, 0);

	pros::delay(WAIT);

	lateralController.lateral_move(-13);

	clamp_toggle();

	pros::delay(2*WAIT);

	intake_in();

	pros::delay(2*WAIT);

	turnController_45.turn_to_angle(-1*28,0);

	w;

	lateralController.lateral_move(11);

	w;
	
	turnController_45.turn_to_angle(-1*80);
	
	w;

	lateralController.lateral_move(19);

	w;

    pros::delay(500);

    lateralController.lateral_move(13);

	w;

    lateralController.lateral_move(-20);

	w;


	turnController_45.turn_to_angle(-1*157,0);

	w;

	lateralController.lateral_move(30);

	w;
	turnController_45.turn_to_angle(-1*195,0);

	w;

	turnController_45.turn_to_angle(-1*136,0);

	w;

	lateralController.lateral_move(17);

	w;

    lateralController.lateral_move(4);

	w;


	lateralController.lateral_move(-10);

	w;

	turnController_135.turn_to_angle(-1*270, 0);

	w;

	lady_brown_load1();

	w;

	lateralController.lateral_move(29);

	w;

	turnController_45.turn_to_angle(-1*226,0);

	w;

	lateralController.lateral_move(10);

	w;

	lady_brown_alliance();


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
    
	lady_brown_load1(); w; intake_in();

	turnController_45.turn_to_angle(-1*135); w;

	intake_in(); w;

	lateralController.lateral_move(9); w;

	// lady_brown_score();
	lady_brown_alliance(); tw(5); lady_brown.move(0);

	lateralController.lateral_move(-6); w;

	tw(4); lady_brown.move(-127);
	turnController_45.turn_to_angle(-1*165); w;

	lateralController.lateral_move(-18); w; lateralController.lateral_move(-15); tw(2);

	clamp_toggle(); w; lady_brown.move(0); w;

	turnController_90.turn_to_angle(-1*270); w;

	intake_in(); w;

	lateralController.lateral_move(26); w;

	turnController_90.turn_to_angle(-1*195); w;

	lateralController.lateral_move(28); w;

	turnController_45.turn_to_angle(-1*223); w;

	drivetrain.velocity_command(50, 50);

	pros::delay(1000);

	drivetrain.velocity_command(0, 0);

	lateralController.lateral_move(-12); pros::delay(1000);

	turnController_180.turn_to_angle(-1*45); w;

	pros::delay(250);

	// clamp_toggle(); tw(2);

	pros::delay(250);

	// lady_brown_score();

	lateralController.lateral_move(36);

	lateralController.lateral_move(16); w;

	// intake_in(); tw(2);
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

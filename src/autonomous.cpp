#include "autonomous.h" 
#include "globals.h"
#include "knights/api.hpp"
#include "knights/logger/logger.hpp"
#include "knights/util/calculation.hpp"
#include "knights/util/position.hpp"

#define RIGHT 1
#define LEFT -1

#define LATERAL_kP 8.3
#define LATERAL_kI 0
#define LATERAL_kD 50

#define TURN_kP_45 98
#define TURN_kI_45 0.0
#define TURN_kD_45 650

#define TURN_kP_90 78 // 75 - 45 // 48 - 90 // 38 - 135 // 34 - 180
#define TURN_kI_90 0.0 // 0.017 - 45 // 0.017 - 90 // 0.017 - 135 // 0.017 - 180
#define TURN_kD_90 525 // 0.08 - 45 // 0.24 - 90 // 0.24 - 135 // 0.24 - 180

#define TURN_kP_135 78
#define TURN_kI_135 0.0
#define TURN_kD_135 610

#define TURN_kP_180 78
#define TURN_kI_180 0.0
#define TURN_kD_180 700

void pid_tuning(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants;

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	// turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::PIDController angularPID(15, 0.01,10);

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, &angularPID, false);

	// knights::Pos start = chassis->get_position();

	// float target = 24;

	// robotControl.lateral_move(target);

	// pros::delay(500);

	// knights::logger::red(knights::logger::string_format("error: %lf\n", target-knights::distance_btwn(start, chassis->get_position())));

	robotControl.lateral_to_position(knights::Pos(-48,-24,0));
}


void pp_test(knights::RobotChassis *chassis) {

	std::string s = "rush.txt";

	knights::AdvancedRoute test_route = advanced_route_from_file(s);
	// for (auto route : test_route.routes) {
	// 	for (auto pt : route.second.positions) {
	// 		// knights::logger::green(knights::logger::string_format("pt: %lf %lf %lf", pt.x, pt.y, pt.heading));
	// 	}
	// }

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 80.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::PIDController angularPID(50, 0, 10, -60.0, 60.0);

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, &angularPID, false);
	
	knights::input::AutonomousInputMap inputMap;
    inputMap.bind_action("intakeRev", intake_out);
    inputMap.bind_action("intakeFwd", intake_in);
    inputMap.bind_action("clamp", clamp_toggle);
	inputMap.bind_action("lbDown", lady_brown_down);
	inputMap.bind_action("lbLoad1", lady_brown_load1);
	inputMap.bind_action("lbScore", lady_brown_score);
	inputMap.bind_action("rushMech", toggle_rush_mech);

	test_route.execute(chassis, &robotControl, &inputMap);
}

#define WAIT 140
#define kPos knights::Pos
#define rad(x) knights::to_rad(x)

#define w pros::delay(WAIT)
#define tw(x) pros::delay((x)*WAIT)

void redone_skills(knights::RobotChassis *chassis) {

	std::string s = "rush.txt";

	knights::AdvancedRoute test_route = advanced_route_from_file(s);

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 110.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 25.0, 115.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::PIDController angularPID(50, 0, 10, -60.0, 60.0);

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, &angularPID, false);
	
	knights::input::AutonomousInputMap inputMap;
    inputMap.bind_action("intakeRev", intake_out);
    inputMap.bind_action("intakeFwd", intake_in);
    inputMap.bind_action("clamp", clamp_toggle);
	inputMap.bind_action("lbDown", lady_brown_down);
	inputMap.bind_action("lbLoad1", lady_brown_load1);
	inputMap.bind_action("lbScore", lady_brown_score);
	inputMap.bind_action("rushMech", toggle_rush_mech);

	intake_in();

	pros::delay(350);

	intake_in();

	robotControl.lateral_to_position(knights::Pos(-50, 0, knights::to_rad(90))); w;

	robotControl.lateral_move(-12); robotControl.lateral_move(-12); w; clamp_toggle(); w; intake_in();

	robotControl.lateral_to_point(kPos(-22, -24, knights::to_rad(330)), true, 3.0); w;

	robotControl.lateral_to_point(kPos(24, -45, 0), 1, 6.0, 2000);  w;

	pros::delay(300); // grab ring

	robotControl.lateral_to_position(kPos(-5, -48, rad(270))); 

	lady_brown_load1(); w;

	robotControl.lateral_move(20);

	pros::delay(600);

	lady_brown_score();

	pros::delay(900);

	robotControl.lateral_to_position(kPos(0, -48, rad(180)), false); 

	lady_brown_down(); w;

	intake_in();


	robotControl.lateral_to_point(kPos(-54, -48, rad(180)), 1, 3.0, 1250); 

	pros::delay(300);

	robotControl.lateral_to_position(kPos(-48, -48, rad(270)), false); 

	robotControl.lateral_move(14); w;

	robotControl.lateral_move(-18); w; robotControl.turn_to_angle(45); w;

	intake_in();

	clamp_toggle();

	robotControl.lateral_move(-24); w; robotControl.lateral_move(8); w;

	robotControl.lateral_to_position(kPos(-46, 12, rad(270)), false); w;

	robotControl.lateral_move(-10); w;

	clamp_toggle(); 
	
	pros::delay(300);

	intake_in();

	robotControl.lateral_to_position(kPos(-24, 24, rad(90))); w;

	robotControl.lateral_to_position(kPos(-24, 50, rad(180))); w;

	robotControl.lateral_to_point(kPos(-60, 48, 0)); w;

	robotControl.lateral_to_position(kPos(-48, 48, rad(90)), false); w;

	robotControl.lateral_move(14); w;

	robotControl.lateral_move(-18); w; robotControl.turn_to_angle(315); w;

	clamp_toggle();

	robotControl.lateral_move(-24); w;

	robotControl.lateral_move(12); w;

	robotControl.lateral_to_point(kPos(0, 48, rad(90)), true, 6.0, 1250); w;

	lady_brown_load1(); w;

	robotControl.lateral_move(20);

	pros::delay(600);

	lady_brown_score();

	pros::delay(600);

	robotControl.lateral_move(-20); w;

	lady_brown_down(); w;

	robotControl.lateral_to_position(kPos(16, 28, rad(310)), true); w;

	lady_brown_load1();

	intake_in(); 

	robotControl.lateral_move(10);
	
	// intake_in(); w;

	robotControl.lateral_to_position(kPos(28, 8, rad(160)), false);

	intake_in();

	robotControl.lateral_move(-12); w; clamp_toggle();

	pros::delay(200);

	robotControl.lateral_to_position(kPos(53, 0, 10)); w;

	lady_brown_alliance();

	pros::delay(600);

	robotControl.lateral_move(-12); w;

	lady_brown_down(); intake_in();

	robotControl.lateral_to_position(kPos(24, 50, rad(0))); w;

	robotControl.lateral_move(36); w;

	robotControl.turn_to_angle(240); clamp_toggle(); w;

	robotControl.lateral_move(-12); w; robotControl.lateral_move(12);

	robotControl.lateral_to_point(kPos(24, 24, 0));

	robotControl.lateral_to_position(kPos(50, -12, rad(270))); w;

	robotControl.lateral_move(50); w; robotControl.lateral_move(-12); robotControl.lateral_move(24);

	robotControl.lateral_to_position(kPos(24, -24, rad(-135)));

	lady_brown_score(); w; drivetrain.voltage_command(-60, -60);



}

void skills(knights::RobotChassis *chassis) {
	knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::Route otherside_1 = knights::init_route_from_sd("os_skills.txt");

	for (auto pos : otherside_1.positions) {
		printf("pos %lf %lf read\n", pos.x, pos.y);
	}
	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);

	robotControl.lateral_move(-8); w;

	intake_in();

	tw(4);

	intake_in();

	robotControl.lateral_move(13); w;

	robotControl.turn_to_angle(90); w;

	robotControl.lateral_move(-12); tw(1); robotControl.lateral_move(-12); tw(2);

	clamp_toggle(); tw(2);
	intake_in();

	robotControl.turn_to_angle(0); w;

	robotControl.lateral_move(23); w;

	robotControl.turn_to_angle(270); w;

	robotControl.lateral_move(25); w;

	robotControl.turn_to_angle(180); w;

	robotControl.lateral_move(18); w;

	robotControl.lateral_move(16); w;

	pros::delay(800);

	robotControl.lateral_move(-12); w;

	robotControl.turn_to_angle(270); w;

	robotControl.lateral_move(11); w;

	pros::delay(800);

	robotControl.lateral_move(-14); w;

	robotControl.turn_to_angle(40); w;

	robotControl.lateral_move(-20); w;

	clamp_toggle(); intake_in();

	pros::delay(600);

	robotControl.lateral_move(14); w;

	robotControl.turn_to_angle(270); w;

	robotControl.lateral_move(-43, 3.0, 1500); w;

	robotControl.turn_to_angle(270); w;

	robotControl.lateral_move(-16); w;

	clamp_toggle();

	robotControl.turn_to_angle(0); w; intake_in();

	robotControl.lateral_move(23); w;

	robotControl.turn_to_angle(-270); w;

	robotControl.lateral_move(23); w;

	robotControl.turn_to_angle(-180); w;

	robotControl.lateral_move(18); w;

	robotControl.lateral_move(16); w;

	pros::delay(800);

	robotControl.lateral_move(-12); w;

	robotControl.turn_to_angle(-270); w;

	robotControl.lateral_move(12); w;

	pros::delay(800);

	robotControl.lateral_move(-10); w;

	robotControl.turn_to_angle(-40); w;

	clamp_toggle(); intake_in();

	robotControl.lateral_move(-20); w;

	pros::delay(600);

	robotControl.lateral_move(24); w; intake_in(); w;

	robotControl.turn_to_angle(0); w;

	robotControl.lateral_move(41, 3.0, 1500); w;

	robotControl.turn_to_angle(90); w;

	lady_brown_load1();

	robotControl.lateral_move(17); w;

	tw(7);

	lady_brown_score(); tw(4);

	robotControl.lateral_move(-24); w;

	w; lady_brown_rev(); lady_brown_rev();

	robotControl.turn_to_angle(315); w;

	robotControl.lateral_move(28); w;

	intake_in(); tw(4); intake_in();

	robotControl.turn_to_angle(140); w;

	robotControl.lateral_move(-18); w;

	robotControl.lateral_move(-16); tw(2);

	clamp_toggle(); tw(3); intake_in();

	robotControl.lateral_move(26); w;

	robotControl.turn_to_angle(90); w;

	robotControl.lateral_move(32); w;

	robotControl.turn_to_angle(0); w;

	robotControl.lateral_move(39); w;

	robotControl.turn_to_angle(250); w;

	clamp_toggle(); tw(3);

	robotControl.lateral_move(-28); w;

	robotControl.lateral_move(12); w;

	robotControl.turn_to_angle(225); w;

	robotControl.lateral_move(24); w;

	robotControl.turn_to_angle(120); w;

	robotControl.lateral_move(-44); w;

	robotControl.turn_to_angle(90); w;

	robotControl.lateral_move(-20); w;

	robotControl.lateral_move(-50, 3.0, 2000); w;

	robotControl.lateral_move(20);

	robotControl.lateral_move(-30);










}

void red_left_wp(knights::RobotChassis *chassis) {

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);
	
	toggle_rush_mech();

	robotControl.lateral_move(20);

	pros::delay(WAIT);

	robotControl.turn_to_angle(27, 0);

	pros::delay(WAIT);

	robotControl.lateral_move(19);

	pros::delay(WAIT);

	robotControl.lateral_move(-14);

	pros::delay(WAIT);

	toggle_rush_mech();

	robotControl.turn_to_angle(100, 0);

	pros::delay(WAIT);

	robotControl.lateral_move(-13);

	clamp_toggle();

	pros::delay(2*WAIT);

	intake_in();

	pros::delay(2*WAIT);

	robotControl.turn_to_angle(28,0);

	w;

	robotControl.lateral_move(11);

	w;
	
	robotControl.turn_to_angle(80);
	
	w;

	robotControl.lateral_move(19);

	w;

    pros::delay(500);

    robotControl.lateral_move(13);

	w;

    robotControl.lateral_move(-20);

	w;


	robotControl.turn_to_angle(157,0);

	w;

	robotControl.lateral_move(30);

	w;
	robotControl.turn_to_angle(195,0);

	w;

	robotControl.turn_to_angle(136,0);

	w;

	robotControl.lateral_move(17);

	w;

    robotControl.lateral_move(4);

	w;


	robotControl.lateral_move(-10);

	w;

	robotControl.turn_to_angle(270, 0);

	w;

	lady_brown_load1();

	w;

	robotControl.lateral_move(29);

	w;

	robotControl.turn_to_angle(226,0);

	w;

	robotControl.lateral_move(10);

	w;

	lady_brown_alliance();


}


void red_rush_right_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);
    
	lady_brown_load1(); w; intake_in();

	robotControl.turn_to_angle(135); w;

	intake_in(); w;

	robotControl.lateral_move(9); w;

	// lady_brown_score();
	lady_brown_alliance(); tw(5); lady_brown.move(0);

	robotControl.lateral_move(-6); w;

	tw(4); lady_brown.move(-127);
	robotControl.turn_to_angle(165); w;

	robotControl.lateral_move(-18); w; robotControl.lateral_move(-15); tw(2);

	clamp_toggle(); w; lady_brown.move(0); w;

	robotControl.turn_to_angle(270); w;

	intake_in(); w;

	robotControl.lateral_move(26); w;

	robotControl.turn_to_angle(195); w;
	robotControl.turn_to_angle(195); w;

	robotControl.lateral_move(30); w;

	// robotControl.turn_to_angle(223); w;

	drivetrain.voltage_command(50, 50);

	pros::delay(1000);

	drivetrain.voltage_command(0, 0);

	robotControl.lateral_move(-12); pros::delay(1000);

	robotControl.turn_to_angle(45); w;

	pros::delay(250);

	// clamp_toggle(); tw(2);

	pros::delay(250);

	lady_brown_score();

	robotControl.lateral_move(44);

	robotControl.lateral_move(18); w;

	// intake_in(); tw(2);
}

void alt_skills(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);

}


void red_rush_right_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}

void blue_right_wp(knights::RobotChassis *chassis) {

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);
	
	toggle_rush_mech();

	robotControl.lateral_move(20);

	pros::delay(WAIT);

	robotControl.turn_to_angle(-1*27, 0);
	robotControl.turn_to_angle(-1*27, 0);

	pros::delay(WAIT);

	robotControl.lateral_move(19);

	pros::delay(WAIT);

	robotControl.lateral_move(-14);

	pros::delay(WAIT);

	toggle_rush_mech();

	robotControl.turn_to_angle(-1*100, 0);

	pros::delay(WAIT);

	robotControl.lateral_move(-13);

	clamp_toggle();

	pros::delay(2*WAIT);

	intake_in();

	pros::delay(2*WAIT);

	robotControl.turn_to_angle(-1*28,0);

	w;

	robotControl.lateral_move(11);

	w;
	
	robotControl.turn_to_angle(-1*80);
	
	w;

	robotControl.lateral_move(19);

	w;

    pros::delay(500);

    robotControl.lateral_move(13);

	w;

    robotControl.lateral_move(-20);

	w;


	robotControl.turn_to_angle(-1*157,0);

	w;

	robotControl.lateral_move(30);

	w;
	robotControl.turn_to_angle(-1*195,0);

	w;

	robotControl.turn_to_angle(-1*136,0);

	w;

	robotControl.lateral_move(17);

	w;

    robotControl.lateral_move(4);

	w;


	robotControl.lateral_move(-10);

	w;

	robotControl.turn_to_angle(-1*270, 0);

	w;

	lady_brown_load1();

	w;

	robotControl.lateral_move(29);

	w;

	robotControl.turn_to_angle(-1*226,0);

	w;

	robotControl.lateral_move(10);

	w;

	lady_brown_alliance();


}


void blue_rush_left_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);
    
	lady_brown_load1(); w; intake_in();

	robotControl.turn_to_angle(-1*135); w;

	intake_in(); w;

	robotControl.lateral_move(9); w;

	// lady_brown_score();
	lady_brown_alliance(); tw(5); lady_brown.move(0);

	robotControl.lateral_move(-6); w;

	tw(4); lady_brown.move(-127);
	robotControl.turn_to_angle(-1*165); w;

	robotControl.lateral_move(-18); w; robotControl.lateral_move(-15); tw(2);

	clamp_toggle(); w; lady_brown.move(0); w;

	robotControl.turn_to_angle(-1*270); w;

	intake_in(); w;

	robotControl.lateral_move(26); w;

	robotControl.turn_to_angle(-1*195); w;

	robotControl.lateral_move(28); w;

	robotControl.turn_to_angle(-1*223); w;

	drivetrain.voltage_command(50, 50);

	pros::delay(1000);

	drivetrain.voltage_command(0, 0);

	robotControl.lateral_move(-12); pros::delay(1000);

	robotControl.turn_to_angle(-1*45); w;

	pros::delay(250);

	// clamp_toggle(); tw(2);

	pros::delay(250);

	// lady_brown_score();

	robotControl.lateral_move(36);

	robotControl.lateral_move(16); w;

	// intake_in(); tw(2);
}

void blue_right_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}

void mogo_red_rush(knights::RobotChassis *chassis) {
	 knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);

	robotControl.lateral_move(-28);

	robotControl.turn_to_angle(25,0);

	robotControl.lateral_move(-18);

	clamp_toggle();

	robotControl.lateral_move(8); w;

	intake_in();

	robotControl.turn_to_angle(-19,0); w;

	robotControl.lateral_move(16); tw(4);

	clamp_toggle();

	intake_in();

	robotControl.turn_to_angle(80); w;

	robotControl.lateral_move(-23); w;

	clamp_toggle(); w;

	robotControl.lateral_move(28); w;

	robotControl.turn_to_angle(0,0); w;

	intake_in();

	robotControl.lateral_move(28.5); w;

	robotControl.turn_to_angle(45,0); w;

	drivetrain.voltage_command(50, 50); tw(5);

	drivetrain.voltage_command(0, 0); tw(5);

	robotControl.lateral_move(-10); w;

	doinker_toggle();

	robotControl.turn_to_angle(180, -1); w;

	clamp_toggle();

	robotControl.lateral_move(20); w;

	doinker_toggle();

	robotControl.turn_to_angle(0,1);




}

void mogo_blue_rush(knights::RobotChassis *chassis) {
	 knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);

	robotControl.lateral_move(-28);

	robotControl.turn_to_angle(-25,0);

	robotControl.lateral_move(-18);

	clamp_toggle();

	robotControl.lateral_move(8); w;

	intake_in();

	robotControl.turn_to_angle(19,0); w;

	robotControl.lateral_move(16); tw(4);

	clamp_toggle();

	intake_in();

	robotControl.turn_to_angle(-80); w;

	robotControl.lateral_move(-23); w;

	clamp_toggle(); w;

	robotControl.lateral_move(28); w;

	robotControl.turn_to_angle(0,0); w;

	intake_in();

	robotControl.lateral_move(28.5); w;

	robotControl.turn_to_angle(-45,0); w;

	drivetrain.voltage_command(50, 50); tw(5);

	drivetrain.voltage_command(0, 0); tw(5);
	
	robotControl.lateral_move(-10); w;

	doinker_toggle();

	robotControl.turn_to_angle(180, 1); w;

	clamp_toggle();

	robotControl.lateral_move(20); w;

	doinker_toggle();

	robotControl.turn_to_angle(0,-1);

}





void right_safe_no_wait(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);
	

	doinker_toggle();

	robotControl.lateral_move(40,1,1000); w;

	robotControl.lateral_move(-16); doinker_toggle(); tw(2);

	robotControl.turn_to_angle(160,1); w;

	robotControl.lateral_move(-12); w;

	robotControl.lateral_move(-8); w;

	clamp_toggle(); w;

	intake_in();

	robotControl.turn_to_angle(205,-1); w;

	
	robotControl.lateral_move(10); w;

	robotControl.turn_to_angle(330,-1); w;

	robotControl.lateral_move(-10); w;

	robotControl.turn_to_angle(40,-1); w;

	robotControl.lateral_move(-10); w;

	clamp_toggle(); w;

	robotControl.turn_to_angle(240,-1); w;

	robotControl.lateral_move(-12); w;

	robotControl.lateral_move(-5); w;

	clamp_toggle(); w;
}


void rush_4_ring(knights::RobotChassis *chassis) {
	knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);

	robotControl.lateral_move(26);

	doinker_toggle();
	
	intake_bottom.move_velocity(INTAKE_VELOCITY);

	robotControl.turn_to_angle(45);

	robotControl.lateral_move(40);

	robotControl.lateral_move(18);

	robotControl.lateral_move(-14);
	
	intake_in();

	robotControl.turn_to_angle(-90,0); w;

	robotControl.lateral_move(-16); w;
	
	clamp_toggle(); w;

	intake_in();


}

void empty(knights::RobotChassis *chassis) {
	return;
}

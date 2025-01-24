#include "autonomous.h" 
#include "globals.h"
#include "knights/autonomous/path.hpp"
#include "knights/autonomous/controller.hpp"
#include "knights/autonomous/pid.hpp"
#include "knights/logger/logger.hpp"
#include "knights/robot/chassis.hpp"
#include "knights/util/calculation.hpp"
#include "liblvgl/misc/lv_anim_timeline.h"
#include "pros/rtos.hpp"

#define RIGHT 1
#define LEFT -1

#define LATERAL_kP 4
#define LATERAL_kI 0
#define LATERAL_kD 0.0065

#define TURN_kP_45 60
#define TURN_kI_45 0.0
#define TURN_kD_45 8

#define TURN_kP_90 38 // 75 - 45 // 48 - 90 // 38 - 135 // 34 - 180
#define TURN_kI_90 0.0 // 0.017 - 45 // 0.017 - 90 // 0.017 - 135 // 0.017 - 180
#define TURN_kD_90 25 // 0.08 - 45 // 0.24 - 90 // 0.24 - 135 // 0.24 - 180

#define TURN_kP_135 38
#define TURN_kI_135 0.017
#define TURN_kD_135 225

#define TURN_kP_180 30
#define TURN_kI_180 0.017
#define TURN_kD_180 85

void pid_tuning(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::RobotController turnController(chassis, &turnPID);

	float desired = -90.0;

	// lateralController.lateral_move(24);
	// turnController.turn_to_angle(270, 0);

	turnController.turn_to_angle(desired, 0, 2, 5000);

	pros::delay(1000);

	knights::logger::blue(knights::logger::string_format("error: %lf", knights::to_deg(chassis->get_position().heading)-desired));
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

	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 30.0, 120.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 30.0, 120.0);
	
	knights::input::AutonomousInputMap inputMap;
    inputMap.bind_action("intakeRev", intake_out);
    inputMap.bind_action("intakeFwd", intake_in);
    inputMap.bind_action("clamp", clamp_toggle);
	inputMap.bind_action("lbDown", lady_brown_down);
	inputMap.bind_action("lbLoad1", lady_brown_load1);
	inputMap.bind_action("lbScore", lady_brown_score);
	inputMap.bind_action("rushMech", toggle_rush_mech);

	test_route.execute(chassis, &lateralPID, &turnPID, &inputMap);
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

	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::RobotController turnController(chassis, &turnPID);

	intake_in();

	tw(4);

	intake_in();

	lateralController.lateral_move(13); w;

	turnController.turn_to_angle(90); w;

	lateralController.lateral_move(-12); tw(1); lateralController.lateral_move(-12); tw(2);

	clamp_toggle(); tw(2);
	intake_in();

	turnController.turn_to_angle(0); w;

	lateralController.lateral_move(23); w;

	turnController.turn_to_angle(270); w;

	lateralController.lateral_move(25); w;

	turnController.turn_to_angle(180); w;

	lateralController.lateral_move(18); w;

	lateralController.lateral_move(16); w;

	pros::delay(800);

	lateralController.lateral_move(-12); w;

	turnController.turn_to_angle(270); w;

	lateralController.lateral_move(11); w;

	pros::delay(800);

	lateralController.lateral_move(-14); w;

	turnController.turn_to_angle(40); w;

	lateralController.lateral_move(-20); w;

	clamp_toggle(); intake_in();

	pros::delay(600);

	lateralController.lateral_move(14); w;

	turnController.turn_to_angle(270); w;

	lateralController.lateral_move(-46, 3.0, 1500); w;

	turnController.turn_to_angle(270); w;

	lateralController.lateral_move(-16); w;

	clamp_toggle();

	turnController.turn_to_angle(0); w; intake_in();

	lateralController.lateral_move(23); w;

	turnController.turn_to_angle(-270); w;

	lateralController.lateral_move(23); w;

	turnController.turn_to_angle(-180); w;

	lateralController.lateral_move(18); w;

	lateralController.lateral_move(16); w;

	pros::delay(800);

	lateralController.lateral_move(-12); w;

	turnController.turn_to_angle(-270); w;

	lateralController.lateral_move(12); w;

	pros::delay(800);

	lateralController.lateral_move(-10); w;

	turnController.turn_to_angle(-40); w;

	clamp_toggle(); intake_in();

	lateralController.lateral_move(-20); w;

	pros::delay(600);

	lateralController.lateral_move(24); w; intake_in(); w;

	turnController.turn_to_angle(0); w;

	lateralController.lateral_move(41, 3.0, 1500); w;

	turnController.turn_to_angle(90); w;

	lady_brown_load1();

	lateralController.lateral_move(17); w;

	tw(7);

	lady_brown_score(); tw(4);

	lateralController.lateral_move(-21); w;

	w; lady_brown_rev(); lady_brown_rev();

	turnController.turn_to_angle(315); w;

	lateralController.lateral_move(28); w;

	intake_in(); tw(4); intake_in();

	turnController.turn_to_angle(140); w;

	lateralController.lateral_move(-18); w;

	lateralController.lateral_move(-16); tw(2);

	clamp_toggle(); tw(3); intake_in();

	lateralController.lateral_move(26); w;

	turnController.turn_to_angle(90); w;

	lateralController.lateral_move(32); w;

	turnController.turn_to_angle(0); w;

	lateralController.lateral_move(39); w;

	turnController.turn_to_angle(250); w;

	clamp_toggle(); tw(3);

	lateralController.lateral_move(-28); w;

	lateralController.lateral_move(12); w;

	turnController.turn_to_angle(225); w;

	lateralController.lateral_move(24); w;

	turnController.turn_to_angle(120); w;

	lateralController.lateral_move(-44); w;

	turnController.turn_to_angle(90); w;

	lateralController.lateral_move(-20); w;

	lateralController.lateral_move(-50, 3.0, 2000); w;

	lateralController.lateral_move(20);

	lateralController.lateral_move(-30);










}

void red_left_wp(knights::RobotChassis *chassis) {

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 100.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::RobotController turnController(chassis, &turnPID);
	
	toggle_rush_mech();

	lateralController.lateral_move(20);

	pros::delay(WAIT);

	turnController.turn_to_angle(27, 0);

	pros::delay(WAIT);

	lateralController.lateral_move(19);

	pros::delay(WAIT);

	lateralController.lateral_move(-14);

	pros::delay(WAIT);

	toggle_rush_mech();

	turnController.turn_to_angle(100, 0);

	pros::delay(WAIT);

	lateralController.lateral_move(-13);

	clamp_toggle();

	pros::delay(2*WAIT);

	intake_in();

	pros::delay(2*WAIT);

	turnController.turn_to_angle(28,0);

	w;

	lateralController.lateral_move(11);

	w;
	
	turnController.turn_to_angle(80);
	
	w;

	lateralController.lateral_move(19);

	w;

    pros::delay(500);

    lateralController.lateral_move(13);

	w;

    lateralController.lateral_move(-20);

	w;


	turnController.turn_to_angle(157,0);

	w;

	lateralController.lateral_move(30);

	w;
	turnController.turn_to_angle(195,0);

	w;

	turnController.turn_to_angle(136,0);

	w;

	lateralController.lateral_move(17);

	w;

    lateralController.lateral_move(4);

	w;


	lateralController.lateral_move(-10);

	w;

	turnController.turn_to_angle(270, 0);

	w;

	lady_brown_load1();

	w;

	lateralController.lateral_move(29);

	w;

	turnController.turn_to_angle(226,0);

	w;

	lateralController.lateral_move(10);

	w;

	lady_brown_alliance();


}


void red_rush_right_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::RobotController turnController(chassis, &turnPID);
    
	lady_brown_load1(); w; intake_in();

	turnController.turn_to_angle(135); w;

	intake_in(); w;

	lateralController.lateral_move(9); w;

	// lady_brown_score();
	lady_brown_alliance(); tw(5); lady_brown.move(0);

	lateralController.lateral_move(-6); w;

	tw(4); lady_brown.move(127);
	turnController.turn_to_angle(165); w;

	lateralController.lateral_move(-18); w; lateralController.lateral_move(-15); tw(2);

	clamp_toggle(); w; lady_brown.move(0); w;

	turnController.turn_to_angle(270); w;

	intake_in(); w;

	lateralController.lateral_move(26); w;

	turnController.turn_to_angle(195); w;

	lateralController.lateral_move(28); w;

	turnController.turn_to_angle(223); w;

	drivetrain.velocity_command(50, 50);

	pros::delay(1000);

	drivetrain.velocity_command(0, 0);

	lateralController.lateral_move(-12); pros::delay(1000);

	turnController.turn_to_angle(45); w;

	pros::delay(250);

	// clamp_toggle(); tw(2);

	pros::delay(250);

	lady_brown_score();

	lateralController.lateral_move(44);

	lateralController.lateral_move(18); w;

	// intake_in(); tw(2);
}

void alt_skills(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::RobotController turnController(chassis, &turnPID);

	intake_in();

	tw(3);

	intake_in();

	lateralController.lateral_move(14); w;

	turnController.turn_to_angle(90, 0); w;

	lateralController.lateral_move(-12); w; 
	
	lateralController.lateral_move(-12); w;

	clamp_toggle(); tw(2);

	intake_in();

	turnController.turn_to_angle(-2, 0); w;

	lateralController.lateral_move(24); w;

	turnController.turn_to_angle(-90); w;

	lateralController.lateral_move(24); w;

	turnController.turn_to_angle(3,0); w;

	lateralController.lateral_move(46); w;

	turnController.turn_to_angle(90,0); w;

	lateralController.lateral_move(27); w;
	
	intake_in();
	
	lady_brown_load1();

	intake_in();

	lateralController.lateral_move(-12); w;

	lateralController.lateral_move(-12); w;

	turnController.turn_to_angle(180,0); w;

	lateralController.lateral_move(22);

	turnController.turn_to_angle(-90,0);

	lateralController.lateral_move(14); w;

	intake_in(); 

	drivetrain.velocity_command(50, 50); tw(3);

	drivetrain.velocity_command(0, 0); w;

	lady_brown_score(); pros::delay(1000);

	lady_brown_load1(); tw(3);

	intake_in(); pros::delay(1000);

	lady_brown_score(); tw(6);

	lady_brown_down(); w;

	intake_in();

	lateralController.lateral_move(-15); w;

	turnController.turn_to_angle(175); w;

	lateralController.lateral_move(48); w;

	lateralController.lateral_move(12); w;

	lateralController.lateral_move(-12); w;

	turnController.turn_to_angle(267); w;

	lateralController.lateral_move(16); w;

	lateralController.lateral_move(-22); w;

	turnController.turn_to_angle(45); w;
	
	lateralController.lateral_move(-24); w;

	clamp_toggle();






	lateralController.lateral_move(28); w;

	turnController.turn_to_angle(-90,0); w;

	lateralController.lateral_move(-24); w;

	lateralController.lateral_move(-24); w;

	intake_in();









	lateralController.lateral_move(-12); w; 
	
	lateralController.lateral_move(-12); w;

	clamp_toggle(); tw(2);

	intake_in();

	turnController.turn_to_angle(-2, 0); w;

	lateralController.lateral_move(24); w;

	turnController.turn_to_angle(90); w;

	lateralController.lateral_move(24); w;

	turnController.turn_to_angle(-3,0); w;

	lateralController.lateral_move(46); w;

	turnController.turn_to_angle(-90,0); w;

	lateralController.lateral_move(27); w;
	
	intake_in();
	
	lady_brown_load1(); w;

	intake_in();

	lateralController.lateral_move(-12); w;

	lateralController.lateral_move(-12); w;

	turnController.turn_to_angle(-180,0); w;

	lateralController.lateral_move(22);

	turnController.turn_to_angle(-90,0); w;

	lateralController.lateral_move(16); w;

	drivetrain.velocity_command(50, 50); tw(3);

	drivetrain.velocity_command(0, 0); w;

	intake_in();

	lady_brown_score(); tw(8);

	lady_brown_load1(); w;;

	intake_in(); pros::delay(1000);

	lady_brown_score(); tw(8);

	lady_brown_down(); w;

	intake_in();

	lateralController.lateral_move(-15); w;

	turnController.turn_to_angle(-180); w;

	lateralController.lateral_move(48); w;

	lateralController.lateral_move(12); w;

	lateralController.lateral_move(-12); w;

	turnController.turn_to_angle(-270); w;

	lateralController.lateral_move(16); w;

	lateralController.lateral_move(-22); w;

	turnController.turn_to_angle(-35); w;
	
	lateralController.lateral_move(-24); w;

	clamp_toggle();








}


void red_rush_right_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::RobotController turnController(chassis, &turnPID);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}

void blue_right_wp(knights::RobotChassis *chassis) {

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 100.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::RobotController turnController(chassis, &turnPID);
	
	toggle_rush_mech();

	lateralController.lateral_move(20);

	pros::delay(WAIT);

	turnController.turn_to_angle(-1*27, 0);

	pros::delay(WAIT);

	lateralController.lateral_move(19);

	pros::delay(WAIT);

	lateralController.lateral_move(-14);

	pros::delay(WAIT);

	toggle_rush_mech();

	turnController.turn_to_angle(-1*100, 0);

	pros::delay(WAIT);

	lateralController.lateral_move(-13);

	clamp_toggle();

	pros::delay(2*WAIT);

	intake_in();

	pros::delay(2*WAIT);

	turnController.turn_to_angle(-1*28,0);

	w;

	lateralController.lateral_move(11);

	w;
	
	turnController.turn_to_angle(-1*80);
	
	w;

	lateralController.lateral_move(19);

	w;

    pros::delay(500);

    lateralController.lateral_move(13);

	w;

    lateralController.lateral_move(-20);

	w;


	turnController.turn_to_angle(-1*157,0);

	w;

	lateralController.lateral_move(30);

	w;
	turnController.turn_to_angle(-1*195,0);

	w;

	turnController.turn_to_angle(-1*136,0);

	w;

	lateralController.lateral_move(17);

	w;

    lateralController.lateral_move(4);

	w;


	lateralController.lateral_move(-10);

	w;

	turnController.turn_to_angle(-1*270, 0);

	w;

	lady_brown_load1();

	w;

	lateralController.lateral_move(29);

	w;

	turnController.turn_to_angle(-1*226,0);

	w;

	lateralController.lateral_move(10);

	w;

	lady_brown_alliance();


}


void blue_rush_left_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::RobotController turnController(chassis, &turnPID);
    
	lady_brown_load1(); w; intake_in();

	turnController.turn_to_angle(-1*135); w;

	intake_in(); w;

	lateralController.lateral_move(9); w;

	// lady_brown_score();
	lady_brown_alliance(); tw(5); lady_brown.move(0);

	lateralController.lateral_move(-6); w;

	tw(4); lady_brown.move(-127);
	turnController.turn_to_angle(-1*165); w;

	lateralController.lateral_move(-18); w; lateralController.lateral_move(-15); tw(2);

	clamp_toggle(); w; lady_brown.move(0); w;

	turnController.turn_to_angle(-1*270); w;

	intake_in(); w;

	lateralController.lateral_move(26); w;

	turnController.turn_to_angle(-1*195); w;

	lateralController.lateral_move(28); w;

	turnController.turn_to_angle(-1*223); w;

	drivetrain.velocity_command(50, 50);

	pros::delay(1000);

	drivetrain.velocity_command(0, 0);

	lateralController.lateral_move(-12); pros::delay(1000);

	turnController.turn_to_angle(-1*45); w;

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

	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::RobotController turnController(chassis, &turnPID);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}


void blue_left_rush_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::RobotController turnController(chassis, &turnPID);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}

void mogo_red_rush(knights::RobotChassis *chassis) {
	 knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::RobotController turnController(chassis, &turnPID);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

	lateralController.lateral_move(-28);

	turnController.turn_to_angle(25,0);

	lateralController.lateral_move(-17);

	clamp_toggle();

	lateralController.lateral_move(7); w;

	intake_in();

	turnController.turn_to_angle(-20,0); w;

	lateralController.lateral_move(16); tw(3);

	clamp_toggle();

	turnController.turn_to_angle(75); w;

	intake_in();

	lateralController.lateral_move(-20); w;

	clamp_toggle(); w;

	lateralController.lateral_move(28); w;

	turnController.turn_to_angle(5,0); w;

	intake_in();

	lateralController.lateral_move(28); w;

	turnController.turn_to_angle(45,0); w;

	drivetrain.velocity_command(50, 50); tw(3);

	drivetrain.velocity_command(0, 0); tw(5);

	lateralController.lateral_move(-36); w;

	lateralController.lateral_move(-20); w;

	lady_brown_score();

	turnController.turn_to_angle(215); w;

	lateralController.lateral_move(18);

}

void right_safe_no_wait(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::RobotController turnController(chassis, &turnPID);
    
	lady_brown_load1(); w; intake_in();

	turnController.turn_to_angle(135); 

	intake_in();

	lateralController.lateral_move(9);

	// lady_brown_score();
	lady_brown_alliance(); tw(5); lady_brown.move(0);

	lateralController.lateral_move(-6);

	tw(4); lady_brown.move(127);
	turnController.turn_to_angle(165);

	lateralController.lateral_move(-18); w; lateralController.lateral_move(-15); tw(2);

	clamp_toggle(); w; lady_brown.move(0);

	turnController.turn_to_angle(270);

	intake_in(); 

	lateralController.lateral_move(26);

	turnController.turn_to_angle(195);

	lateralController.lateral_move(28);

	turnController.turn_to_angle(223);

	drivetrain.velocity_command(50, 50);

	pros::delay(1000);

	drivetrain.velocity_command(0, 0);

	lateralController.lateral_move(-12); pros::delay(1000);

	turnController.turn_to_angle(45);

	pros::delay(250);

	// clamp_toggle(); tw(2);
	lady_brown_score();

	lateralController.lateral_move(44);

	lateralController.lateral_move(18);

	// intake_in(); tw(2);
}


void rush_4_ring(knights::RobotChassis *chassis) {
	knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::RobotController turnController(chassis, &turnPID);

	turnController.turn_to_angle(-26,0);

	doinker_toggle();

	intake_in();

	lateralController.lateral_move(40);

	lateralController.lateral_move(18);

	lateralController.lateral_move(-14);
	
	intake_in();

	turnController.turn_to_angle(-90,0); w;

	lateralController.lateral_move(-16); w;
	
	clamp_toggle(); w;

	intake_in();


}




void empty(knights::RobotChassis *chassis) {
	return;
}

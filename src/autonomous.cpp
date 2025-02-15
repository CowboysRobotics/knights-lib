#include "autonomous.h" 
#include "globals.h"
#include "knights/api.hpp"
#include "knights/autonomous/advanced_route.hpp"
#include "knights/autonomous/path.hpp"
#include "knights/logger/logger.hpp"
#include "knights/robot/chassis.hpp"
#include "knights/util/calculation.hpp"
#include "knights/util/position.hpp"
#include "pros/rtos.hpp"

#include <cmath>
#include <fstream>

#define RIGHT 1
#define LEFT -1

#define LATERAL_kP 8.1
#define LATERAL_kI 0
#define LATERAL_kD 50

#define TURN_kP_45 98
#define TURN_kI_45 0.0
#define TURN_kD_45 675

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
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::PIDController angularPID(15, 0.01,10);

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, &angularPID, false);

	// knights::ProfileGenerator generator(drivetrain, 100);
	// knights::MotionProfile profile = generator.generate(knights::Pos(0, 0, 90_deg), knights::Pos(24, 24, 0), 80, 0, 0);

	// std::fstream write_file("/usd/motion_output.txt", std::ios_base::out);

	// for (knights::ProfileTimestamp timestamp : profile.timestamps) {
	// 	write_file << "time: " << timestamp.time << " ";
	// 	write_file << "pos: " << timestamp.position.x << " " << timestamp.position.y << " " << timestamp.position.heading << " ";
	// 	write_file << "lin vel: " << timestamp.linear_velocity << " ";
	// 	write_file << "angular vel: " << timestamp.angular_velocity << " ";
	// 	write_file << "dist: " << timestamp.curr_distance << " ";
	// 	write_file << "side vels (r,l): " << timestamp.right_speed << " " << timestamp.left_speed << " ";
	// 	write_file << "end timestamp\n";

	// 	std::cout << "time: " << timestamp.time << " ";
	// 	std::cout << "pos: " << timestamp.position.x << " " << timestamp.position.y << " " << timestamp.position.heading << " ";
	// 	std::cout << "lin vel: " << timestamp.linear_velocity << " ";
	// 	std::cout << "angular vel: " << timestamp.angular_velocity << " ";
	// 	std::cout << "dist: " << timestamp.curr_distance << " ";
	// 	std::cout << "side vels (r,l): " << timestamp.right_speed << " " << timestamp.left_speed << " ";
	// 	std::cout << "end timestamp\n";
	// }

	// auto t = knights::linspace(0, 1, 10);
	// for (auto value : t) {
	// 	knights::display::MapDot dot(5, 5, lv_palette_darken(LV_PALETTE_CYAN, 2));
	// 	std::cout << "pt: " << profile.path.position(value).x << " " << profile.path.position(value).y << " " << profile.path.position(value).heading << "\n";
	// 	dot.set_field_pos(profile.path.position(value));
	// }

	robotControl.lateral_move(24);
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

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
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

	knights::ProfileGenerator generator(drivetrain, 600);
	knights::MotionProfile profile = generator.generate(knights::Pos(0, 0, 90_deg), knights::Pos(24, 24, 0), 80, 0, 0);

	robotControl.follow_route_pursuit(knights::Route(profile), 18.0, 100.0);

	// test_route.execute(chassis, &robotControl, &inputMap);
}

#define WAIT 170
#define kPos knights::Pos
#define rad(x) knights::to_rad(x)

#define w pros::delay(WAIT)
#define tw(x) pros::delay((x)*WAIT)

void redone_skills(knights::RobotChassis *chassis) {

	std::string s = "rush.txt";

	knights::AdvancedRoute test_route = advanced_route_from_file(s);

    knights::RamseteConstants ramsete_constants(1, 0.5);


	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 120.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
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

	intake_in();

	pros::delay(350);

	intake_in();

	robotControl.lateral_to_position(knights::Pos(-48, 0, knights::to_rad(90))); w;

	robotControl.lateral_move(-12); robotControl.lateral_move(-12); w; clamp_toggle(); w; intake_in();

	robotControl.lateral_to_point(kPos(-22, -24, knights::to_rad(330)), true, 3.0); w;

	robotControl.lateral_to_point(kPos(24, -45, 0), 1, 6.0, 2000);  w;

	pros::delay(300); // grab ring

	robotControl.lateral_to_position(kPos(-4, -48, rad(270))); 

	lady_brown_load1(); w;

	robotControl.lateral_move(17);

	pros::delay(600);

	lady_brown_score();

	pros::delay(900);

	robotControl.lateral_move(-10);

	lady_brown_down(); w;

	intake_in();

	robotControl.lateral_to_position(kPos(-24, -48, rad(180)), 1, 2.0, 1500); 

	robotControl.lateral_move(30);

	pros::delay(300);

	// robotControl.lateral_to_position(kPos(-48, -48, rad(270)), false); 

	robotControl.lateral_move(-10); w; robotControl.turn_to_angle(270); w;

	robotControl.lateral_move(10); w;

	robotControl.lateral_to_position(kPos(-45, -46, rad(45)), false);  w;

	intake_in();

	clamp_toggle();

	robotControl.lateral_move(-24); w; robotControl.lateral_move(12); w;

	robotControl.lateral_to_position(kPos(-47, -24, rad(270))); w;

	robotControl.lateral_to_point(kPos(-47, 0, rad(270)), false); w;

	robotControl.turn_to_point(kPos(-48, 24, 0), false);
	
	robotControl.lateral_move(-16); w;

	clamp_toggle(); 
	
	pros::delay(300);

	intake_in();

	robotControl.lateral_to_position(kPos(-24, 24, rad(90))); w;

	robotControl.lateral_to_position(kPos(-24, 47, rad(180))); w;

	robotControl.lateral_to_point(kPos(-52, 47, 0)); w;

	robotControl.lateral_to_position(kPos(-48, 48, rad(90)), false); w;

	robotControl.lateral_move(8); w;

	robotControl.lateral_move(-14); w; robotControl.turn_to_angle(315); w;

	clamp_toggle();

	robotControl.lateral_move(-24); w;

	robotControl.lateral_move(12); w;

	robotControl.lateral_to_position(kPos(-3, 38, rad(90)), true, 4.0, 1250); w;

	lady_brown_load1(); w;

	robotControl.lateral_move(24);

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

	robotControl.lateral_to_point(kPos(28, 8, rad(160)), false);

	intake_in();

	robotControl.lateral_move(-12); w; clamp_toggle();

	pros::delay(300);

	robotControl.lateral_move(12);

	robotControl.lateral_to_point(kPos(51, 0, rad(0)), true); 

	lady_brown_alliance();

	pros::delay(600);

	robotControl.lateral_move(-12); w;

	lady_brown_down(); intake_in();

	// robotControl.lateral_to_position(kPos(24, 50, rad(0))); w;

	// robotControl.lateral_move(36); w;

	// robotControl.turn_to_angle(240); clamp_toggle(); w;

	// robotControl.lateral_move(-12); w; robotControl.lateral_move(12);

	// robotControl.lateral_to_point(kPos(24, 24, 0));

	// robotControl.lateral_to_position(kPos(50, -12, rad(270))); w;

	// robotControl.lateral_move(50); w; robotControl.lateral_move(-12); robotControl.lateral_move(24);

	// robotControl.lateral_to_position(kPos(24, -24, rad(-135)));

	// lady_brown_score(); w; drivetrain.voltage_command(-60, -60);



}

void w400ms() {
	pros::delay(400);
}

void w800ms() {
	pros::delay(800);
}

void w1s() {
	pros::delay(1000);
}

ASSET(skillsfirst_txt)
ASSET(skillssecond_txt)
ASSET(skillsthird_txt)
ASSET(skillsfourth_txt)
ASSET(skills5_txt)
ASSET(skills6_txt)

void skills(knights::RobotChassis *chassis) {

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 110.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
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
	inputMap.bind_action("lbAlliance", lady_brown_alliance);
	inputMap.bind_action("wait1000", w1s);
	inputMap.bind_action("wait800", w400ms);
	inputMap.bind_action("wait400", w800ms);

	AssetStream first(skillsfirst_txt);
	auto first_route = knights::init_route_from_asset(first);
	first_route.add_action(kPos(-12, -36, 0), &lady_brown_load1);

	AssetStream second(skillssecond_txt);
	auto second_route = knights::init_route_from_asset(second);

	AssetStream third(skillsthird_txt);
	auto third_route = knights::init_route_from_asset(third);

	AssetStream fourth(skillsfourth_txt);
	auto fourth_route = knights::init_route_from_asset(fourth);
	fourth_route.add_action(kPos(-12, 12, 0), &intake_in);

	AssetStream fifth(skills5_txt);
	auto fifth_route = knights::init_route_from_asset(fifth);

	AssetStream sixth(skills6_txt);
	auto sixth_route = knights::init_route_from_asset(sixth);

	intake_in();

	pros::delay(350);

	intake_in();

	robotControl.lateral_to_position(knights::Pos(-48, 0, knights::to_rad(90)), 1, 2.0, 500); w;

	robotControl.lateral_move(-12); robotControl.lateral_move(-12);
	
	clamp_toggle(); w; intake_in();

	robotControl.turn_to_angle(0); w;

	// route from first mogo to other side ring
	robotControl.follow_route_pursuit(
		first_route, 18.0, 100, true, 3.0, 1600
	);

	pros::delay(500);

	robotControl.lateral_to_position(kPos(-2, -40, rad(270)), false, 2.0, 1000); w;

	robotControl.lateral_move(28, 2.0, 750); intake_in();

	lady_brown_score(); pros::delay(400); intake_in(); pros::delay(500); lady_brown_down();

	robotControl.lateral_move(-12); w; robotControl.turn_to_angle(180);

	// route to get three rings
	robotControl.follow_route_pursuit(
		second_route, 18.0, 100, true, 4.0, 1500
	);

	robotControl.lateral_move(8);

	pros::delay(500);

	robotControl.turn_to_angle(305, 0, 2.0, 750); w;

	robotControl.lateral_move(12); w;

	pros::delay(200);

	robotControl.turn_to_angle(40); w;

	clamp_toggle(); robotControl.lateral_move(-20, 2.0, 750);

	lady_brown_load1();

	// route from first mogo to other side ring
	robotControl.follow_route_pursuit(
		third_route, 18.0, 100, true, 3.0, 2000
	); w;

	robotControl.lateral_to_point(kPos(46, -48, 0));

	robotControl.turn_to_point(kPos(60, -24, 0), false); w; intake_in();

	// robotControl.turn_to_angle(230, 0, 2.0, 850); intake_in(); w;
	
	robotControl.lateral_move(-12); robotControl.lateral_move(-8);
	clamp_toggle(); pros::delay(300);

	// robotControl.lateral_to_position(kPos(48, -48, rad(135)));

	robotControl.lateral_move(22, 2.0, 750); robotControl.turn_to_angle(135);

	clamp_toggle();

	robotControl.lateral_move(-20, 2.0, 750); w;

	robotControl.lateral_move(24); w;

	robotControl.lateral_to_position(kPos(38, -20, rad(265)), true); w;

	robotControl.lateral_move(-12); robotControl.lateral_move(-12); w; clamp_toggle(); // grab third mogo

	robotControl.turn_to_angle(0); w; intake_in();

	robotControl.lateral_move(24); w; // could maybe reset pos here

	robotControl.lateral_move(-8); w; intake_in();

	lady_brown_load2(); pros::delay(800); w;

	robotControl.lateral_move(-6); w;
	
	// /* TEMP */ chassis->set_position(kPos(48, 0, 0)); clamp_toggle(); pros::delay(500);

	lady_brown_down(); intake_in();

	robotControl.lateral_to_position(kPos(24, -28, rad(145)), true, 2.0, 1250); w; intake_in();

	robotControl.lateral_move(48, 6.0, 3000); intake_in(); w;

	robotControl.lateral_move(36, 2.0, 750);

	pros::delay(500);

	robotControl.turn_to_angle(180); w; robotControl.lateral_move(12); w;

	robotControl.turn_to_angle(50); w; 

	robotControl.lateral_move(8);

	pros::delay(400);

	robotControl.turn_to_angle(-45, 0, 2.0, 750); w; clamp_toggle(); 

	robotControl.lateral_move(-12, 2.0, 750); lady_brown_load1();

	robotControl.follow_route_pursuit(
		fifth_route, 18.0, 90, true, 3.0, 1000
	); w;

	robotControl.turn_to_point(kPos(-48, 24, 0), false);

	robotControl.lateral_move(-18); robotControl.lateral_move(-14); clamp_toggle();

	pros::delay(200);

	robotControl.lateral_move(20); w;

	robotControl.lateral_to_position(kPos(-4, 44, rad(90))); w;

	robotControl.lateral_move(28, 2.0, 1250);

	lady_brown_score(); pros::delay(600); intake_in(); pros::delay(500); lady_brown_down(); intake_in();

	robotControl.lateral_move(-8); w; intake_in();
	
	robotControl.lateral_to_position(kPos(26, 26, rad(90))); w;

}

ASSET(skills2third_txt)
ASSET(skills2fourth_txt)
ASSET(skills2fifth_txt)
ASSET(skills2secondmogo_txt)

#define GUIDE_TO_TRACKING_CENTER_DIST 10 // need to cange
#define WALL_STAKE_POLE_DIST 1

std::fstream write_file("/usd/skills.log", std::ios_base::out);

void safer_skills(knights::RobotChassis *chassis) {

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 110.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
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
	inputMap.bind_action("lbAlliance", lady_brown_alliance);
	inputMap.bind_action("wait1000", w1s);
	inputMap.bind_action("wait800", w400ms);
	inputMap.bind_action("wait400", w800ms);

	AssetStream first(skillsfirst_txt);
	auto first_route = knights::init_route_from_asset(first);
	first_route.add_action(kPos(-12, -36, 0), &lady_brown_load1);

	AssetStream second(skillssecond_txt);
	auto second_route = knights::init_route_from_asset(second);

	AssetStream third(skills2third_txt);
	auto third_route = knights::init_route_from_asset(third);

	AssetStream fourth(skills2fourth_txt);
	auto fourth_route = knights::init_route_from_asset(fourth);

	AssetStream fifth(skills2fifth_txt);
	auto fifth_route = knights::init_route_from_asset(fifth);

	AssetStream second_mogo(skills2secondmogo_txt);
	auto second_mogo_route = knights::init_route_from_asset(second_mogo);

	intake_in();

	pros::delay(350);

	intake_in();

	robotControl.lateral_to_position(knights::Pos(-48, 0, knights::to_rad(90)), 1, 2.0, 500); w;

	robotControl.lateral_move(-12); robotControl.lateral_move(-12);
	
	clamp_toggle(); w; intake_in();

	robotControl.turn_to_angle(0); w;

	// route from first mogo to other side ring
	robotControl.follow_route_pursuit(
		first_route, 18.0, 100, true, 3.0, 1600
	);

	// pros::delay(500);

	robotControl.lateral_to_position(kPos(-2, -43, rad(270)), false, 2.0, 1000); w;

	robotControl.lateral_move(24, 2.0, 750); intake_in();

	lady_brown_score(); pros::delay(700); robotControl.lateral_move(-8, 2.0, 750, false); lady_brown_load1(); pros::delay(500); 
	
	intake_in(); robotControl.lateral_move(10, 2.0, 750, false); pros::delay(500); lady_brown_score(); pros::delay(500);  intake_in();

	robotControl.lateral_move(-10);

	lady_brown_down(); w; robotControl.turn_to_angle(180);

	// route to get three rings
	robotControl.follow_route_pursuit(
		second_route, 18.0, 100, true, 4.0, 1500
	);

	robotControl.lateral_move(8);

	pros::delay(500);

	robotControl.turn_to_angle(305, 0, 2.0, 750); w;

	robotControl.lateral_move(12); w;

	pros::delay(200);

	robotControl.turn_to_angle(40); w;

	clamp_toggle(); robotControl.lateral_move(-20, 2.0, 750); w;

	robotControl.lateral_move(14); w; intake_in();

	// ^ gets one 6 ring mogo pushed into corner

	robotControl.turn_to_angle(270, 0, 2.0, 750); robotControl.lateral_move(-60, 6.0, 2000);
	robotControl.turn_to_point(kPos(-48, 24, 0), false);
	robotControl.lateral_move(-12);
	
	clamp_toggle(); w;

	robotControl.turn_to_angle(0); w; intake_in();

	robotControl.lateral_move(24); w; robotControl.turn_to_angle(80); w;

	robotControl.follow_route_pursuit(
		third_route, 12.0, 80
	); w;

	robotControl.lateral_move(6);

	pros::delay(600);

	robotControl.turn_to_point(kPos(-48, 60, 0)); w;

	robotControl.lateral_move(12); w;

	robotControl.turn_to_angle(330); w; clamp_toggle();

	robotControl.lateral_move(-16); w; robotControl.lateral_move(12);

	lady_brown_load1();

	robotControl.follow_route_pursuit(fourth_route, 18.0, 90); w;

	robotControl.lateral_to_position(kPos(-7, 40, rad(90)), false, 1.0, 3500);

	robotControl.lateral_move(26, 2.0, 750); w; // ram wall stake

	// ---- RESET POSITION ON WALL STAKE ----

	chassis->set_position(kPos(
		cos(chassis->get_position().heading) * GUIDE_TO_TRACKING_CENTER_DIST,
		(72 - WALL_STAKE_POLE_DIST) - sin(chassis->get_position().heading) * GUIDE_TO_TRACKING_CENTER_DIST,
		chassis->get_position().heading
	));

	// ---- END ----

	w; intake_in(); lady_brown_score(); pros::delay(700); robotControl.lateral_move(-8, 2.0, 750, false); lady_brown_load1(); pros::delay(300); 
	
	intake_in(); robotControl.lateral_move(10, 2.0, 750, false); pros::delay(500); lady_brown_score(); pros::delay(500);  intake_in();

	robotControl.lateral_move(-10); w; lady_brown_down(); intake_in();

	// end of 2nd side

	// robotControl.follow_route_pursuit(
	// 	fifth_route, 18.0, 90, false
	// ); w; clamp_toggle();

	// robotControl.turn_to_point(kPos(60, 24, rad(270)), false, 0, 3.0, 1250); w;

	robotControl.turn_to_angle(146, 0, 3.0, 750); w;

	robotControl.lateral_move(-36, 4.0, 1500); w;

	// robotControl.turn_to_point(kPos(60, 24, rad(270)), false, 0, 3.0, 500); w;

	robotControl.lateral_move(-22, 5.0, 750); w; clamp_toggle();

	robotControl.turn_to_angle(90); w; intake_in();

	lady_brown_load1();

	robotControl.lateral_move(24); w;

	doinker_toggle();

	robotControl.turn_to_angle(225); w;
	clamp_toggle();

	robotControl.lateral_move(-12, 3.0, 750); w; intake_in(); doinker_toggle();

	robotControl.lateral_move(16); w;

	robotControl.lateral_to_point(kPos(34, 24, 90));

	// robotControl.turn_to_angle(270); w;
	// robotControl.lateral_move(36);

	robotControl.turn_to_angle(90); w;

	robotControl.lateral_move(-16); robotControl.lateral_move(-12); clamp_toggle(); w;

	robotControl.turn_to_angle(0); w; intake_in();

	robotControl.lateral_move(24); w; // could maybe reset pos here

	robotControl.lateral_move(-8); w; intake_in();

	lady_brown_load2(); pros::delay(600); w;

	robotControl.lateral_move(-12); w; lady_brown_down(); w; intake_in();

	robotControl.turn_to_angle(135); w;

	robotControl.lateral_move(28); w;

	robotControl.turn_to_angle(225); w;

	robotControl.lateral_move(28); w;

	robotControl.turn_to_angle(315); w;

	robotControl.lateral_move(28); w;

	write_file << "skills end\n";

}

void na_skills(knights::RobotChassis *chassis) {
	knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::Route otherside_1 = knights::init_route_from_sd("os_skills.txt");

	for (auto pos : otherside_1.positions) {
		printf("pos %lf %lf read\n", pos.x, pos.y);
	}
	knights::PIDController lateralPID(4, 0, 0.0065, 10.0, 127.0);
	knights::PIDController turnPID(38, 0, 25, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(60, 0, 8));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(38, 0, 25));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(38, 0.017, 225));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(30, 0.017, 85));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);

	// robotControl.lateral_move(-8); w;

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
	
	robotControl.lateral_move(20);

	pros::delay(WAIT);

	robotControl.turn_to_angle(27, 0);

	pros::delay(WAIT);

	robotControl.lateral_move(19);

	pros::delay(WAIT);

	robotControl.lateral_move(-14);

	pros::delay(WAIT);

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

	lady_brown_alliance(); tw(5); 

	robotControl.lateral_move(-5);

	lady_brown_down();

	robotControl.turn_to_angle(76,1); w;
	
	robotControl.lateral_move(6);

	doinker_toggle();

	robotControl.lateral_move(-6); w;

	robotControl.turn_to_angle(140); w; 

	robotControl.lateral_move(-15); w;

	robotControl.turn_to_angle(190); w;

	robotControl.lateral_move(-10); w;

	robotControl.lateral_move(-15); w;
	
	clamp_toggle(); w;

	intake_in();

	robotControl.turn_to_angle(290); w;

	doinker_toggle();

	robotControl.turn_to_angle(270); w;

	robotControl.lateral_move(28); tw(10);

	robotControl.turn_to_angle(0, -1); w;

	clamp_toggle();

	robotControl.turn_to_angle(180, 1);

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

	doinker_toggle2();

	intake_bottom.move_velocity(INTAKE_VELOCITY);

	robotControl.lateral_move(39);

	robotControl.lateral_move(-15);

	doinker_toggle2(); tw(3);

	robotControl.turn_to_angle(180); w;

	robotControl.lateral_move(-16); w;

	clamp_toggle(); tw(2);

	intake_in(); pros::delay(500);

	robotControl.lateral_move(20); w;

	robotControl.turn_to_angle(0,1); w;

	clamp_toggle(); 

	intake_in();

	robotControl.lateral_move(10); w;

	robotControl.turn_to_angle(-90); w;

	robotControl.lateral_move(-18); w;

	clamp_toggle();

	robotControl.turn_to_angle(50); tw(3);

	robotControl.lateral_move(19); w;

	doinker_toggle();

	robotControl.lateral_move(-25); w;

	doinker_toggle(); 

	robotControl.turn_to_angle(35); w;

	intake_in();

	robotControl.lateral_move(15); w;

	robotControl.turn_to_angle(110); w;

	robotControl.lateral_move(-10); w;
}

void blue_rush_left_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);

	doinker_toggle();

	intake_bottom.move_velocity(INTAKE_VELOCITY);

	robotControl.lateral_move(39);

	robotControl.lateral_move(-15);

	doinker_toggle(); tw(3);

	robotControl.turn_to_angle(-180); w;

	robotControl.lateral_move(-16); w;

	clamp_toggle(); tw(2);

	intake_in(); pros::delay(500);

	robotControl.lateral_move(20); w;

	robotControl.turn_to_angle(0,-1); w;

	clamp_toggle(); 

	intake_in();

	robotControl.lateral_move(10); w;

	robotControl.turn_to_angle(90); w;

	robotControl.lateral_move(-18); w;

	clamp_toggle();

	robotControl.turn_to_angle(-50); tw(3);

	robotControl.lateral_move(19); w;

	doinker_toggle2();

	robotControl.lateral_move(-25); w;

	doinker_toggle2(); 

	robotControl.turn_to_angle(-35); w;

	intake_in();

	robotControl.lateral_move(15); w;

	robotControl.turn_to_angle(-110); w;

	robotControl.lateral_move(-10); w;
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
	
	robotControl.lateral_move(20);

	pros::delay(WAIT);

	robotControl.turn_to_angle(-1*27, 0);
	robotControl.turn_to_angle(-1*27, 0);

	pros::delay(WAIT);

	robotControl.lateral_move(19);

	pros::delay(WAIT);

	robotControl.lateral_move(-14);

	pros::delay(WAIT);

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

	lady_brown_alliance(); tw(5); 

	robotControl.lateral_move(-5);

	lady_brown_down();

	robotControl.turn_to_angle(-76,-1); w;
	
	robotControl.lateral_move(6);

	doinker_toggle2();

	robotControl.lateral_move(-6); w;

	robotControl.turn_to_angle(-140); w; 

	robotControl.lateral_move(-15); w;

	robotControl.turn_to_angle(-190); w;

	robotControl.lateral_move(-10); w;

	robotControl.lateral_move(-15); w;
	
	clamp_toggle(); w;

	intake_in();

	robotControl.turn_to_angle(-285); w;

	doinker_toggle2();

	robotControl.turn_to_angle(-265); w;

	robotControl.lateral_move(26); tw(10);

	robotControl.turn_to_angle(0, 1); w;

	clamp_toggle();

	robotControl.turn_to_angle(180, -1);
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

void sig_red_winpoint(knights::RobotChassis *chassis){
	knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);

	color_sorting = true;
	red_alliance = true;

	lady_brown_load2(); tw(3);

	robotControl.lateral_move(-5); w;

	lady_brown_down();

	// robotControl.turn_to_angle(-170, 0, 2.0, 750); w;
	robotControl.turn_to_point(kPos(-24, 24, 0), false, 0, 2.0, 750);

	robotControl.lateral_move(-16);

	robotControl.lateral_move(-16);

	clamp_toggle(); tw(2);

	robotControl.turn_to_angle(40, 0, 2.0, 750); w;

	doinker_toggle2();

	intake_in();

	robotControl.lateral_move(25); w;

	robotControl.lateral_move(-23); w;

	robotControl.turn_to_angle(70, 0, 2.0, 750); w; 

	doinker_toggle2(); w;

	robotControl.turn_to_angle(90, 0, 2.0, 500); w;

	robotControl.lateral_move(20); tw(2);

	robotControl.lateral_to_position(kPos(-40, 0, 90));

	clamp_toggle(); 

	robotControl.turn_to_point(kPos(-24, -24, 0), false, 0, 2.0, 750);

	robotControl.lateral_move(-16); w; clamp_toggle(); w;

	robotControl.turn_to_point(kPos(-24, -48, 0), true, 0, 2.0, 750); w;

	robotControl.lateral_move(16); w;

	robotControl.turn_to_point(kPos(12,0,0), true, 0, 2.0, 750);

	robotControl.lateral_move(12); lady_brown_score();

}

void empty(knights::RobotChassis *chassis) {
	return;
}

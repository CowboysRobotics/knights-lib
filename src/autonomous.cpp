#include "autonomous.h" 
#include "globals.h"
#include "knights/api.hpp"
#include "knights/asset.hpp"
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


void pid_tuning(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
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

ASSET(test_txt)

void pp_test(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {

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

	AssetStream first(test_txt);
	auto first_route = knights::init_route_from_asset(first);

	clamp_toggle();
	intake_in();

	pros::delay(500);

	robotControl.follow_route_pursuit(first_route, 18.0, 100.0);

	// test_route.execute(chassis, &robotControl, &inputMap);
}

#define WAIT 170
#define kPos knights::Pos
#define rad(x) knights::to_rad(x)

#define w pros::delay(WAIT)
#define tw(x) pros::delay((x)*WAIT)

ASSET(skillsfirst_txt)
ASSET(skillssecond_txt)
ASSET(skills2third_txt)
ASSET(skills2fourth_txt)
ASSET(skills2fifth_txt)
ASSET(skills2sixth_txt)
ASSET(skills2secondmogo_txt)
ASSET(skills2thirdmogo_txt)

#define GUIDE_TO_TRACKING_CENTER_DIST 9 // need to cange
#define WALL_STAKE_POLE_DIST 1

void skills(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {

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

	AssetStream first(skillsfirst_txt);
	auto first_route = knights::init_route_from_asset(first);
	first_route.add_action(kPos(-6, -36, 0), &lady_brown_load1);

	AssetStream second(skillssecond_txt);
	auto second_route = knights::init_route_from_asset(second);

	AssetStream third(skills2third_txt);
	auto third_route = knights::init_route_from_asset(third);

	AssetStream fourth(skills2fourth_txt);
	auto fourth_route = knights::init_route_from_asset(fourth);

	AssetStream fifth(skills2fifth_txt);
	auto fifth_route = knights::init_route_from_asset(fifth);

	AssetStream sixth(skills2sixth_txt);
	auto sixth_route = knights::init_route_from_asset(sixth);

	AssetStream second_mogo(skills2secondmogo_txt);
	auto second_mogo_route = knights::init_route_from_asset(second_mogo);

	AssetStream third_mogo(skills2thirdmogo_txt);
	auto third_mogo_route = knights::init_route_from_asset(third_mogo);

	color_sorting = false;

	intake_in();

	pros::delay(200);

	intake_in();

	chassis->set_position(kPos(
		-72 + (knights::to_inches(back_sensor.get_distance()/1000.0)-back.y_displacement)
	, 0, chassis->get_position().heading));

	robotControl.lateral_to_position(knights::Pos(-50, 0, knights::to_rad(90)), true, 2.0, 500, 0, 10); w;
	// robotControl.turn_to_point(kPos(-48, -24, 0), false, 0, 2.0, 600);
	robotControl.turn_to_angle(90);

	// robotControl.lateral_move(13); w; robotControl.turn_to_angle(90);

	robotControl.lateral_move(-12); robotControl.lateral_move(-12);
	
	clamp_toggle(); w; intake_in();

	robotControl.turn_to_angle(0); w;

	// route from first mogo to other side ring
	robotControl.follow_route_pursuit(
		first_route, 18.0, 100, true, 3.0, 1600
	);

	// pros::delay(500);

	robotControl.lateral_to_position(kPos(-4, -43, rad(270)), false, 3.0, 1000, 750);

	robotControl.lateral_move(26, 2.0, 750); intake_in();

	lady_brown_score(); pros::delay(200); intake_in(); pros::delay(300); 
	
	robotControl.lateral_move(-10, 2.0, 750, false); 

	lady_brown_down(); robotControl.turn_to_angle(180, 0, 2.0, 750);

	// route to get three rings
	robotControl.follow_route_pursuit(
		second_route, 18.0, 100, true, 4.0, 1600
	);

	robotControl.lateral_move(12);

	pros::delay(500);

	robotControl.turn_to_angle(305, 0, 2.0, 750); w;

	robotControl.lateral_move(12);

	pros::delay(200);

	robotControl.turn_to_angle(40, 0, 3.0, 750); w;

	clamp_toggle(); robotControl.lateral_move(-20, 2.0, 600);

	// ^ gets one 6 ring mogo pushed into corner

	// intake_in();
	// robotControl.follow_route_pursuit(
	// 	second_mogo_route, 18.0, 90, true, 4.0, 1500
	// );
	// robotControl.turn_to_angle(270, 0, 2.0, 750);
	// robotControl.lateral_move(-12);

	robotControl.lateral_move(14, 3.0, 600); w; intake_in();
	robotControl.turn_to_angle(270, 0, 2.0, 750); 
	// w; 
	// robotControl.lateral_move(-50, 8.0, 1250);
	// robotControl.turn_to_point(kPos(-48, 24, 0), false, 0, 2.0, 750);

	robotControl.curve_move(kPos(-48, 24, 0), false, 32.0, 1400); w;

	robotControl.lateral_move(-16, 3.0, 600);// robotControl.lateral_move(-12, 3.0, 600);
	
	clamp_toggle(); w;

	robotControl.turn_to_angle(0); intake_in(); w;

	robotControl.lateral_move(20, 2.0, 750); w; 
	robotControl.turn_to_angle(60, 0.0, 3.0, 1000); w;

	robotControl.follow_route_pursuit(
		third_route, 12.0, 80
	); w;

	robotControl.lateral_move(10, 2.0, 650);

	pros::delay(350);

	// robotControl.turn_to_point(kPos(-52, 60, 0));

	robotControl.turn_to_angle(60, 0, 2.0, 1000);

	robotControl.lateral_move(11, 4.0, 500);

	pros::delay(300);

	robotControl.turn_to_angle(305, 0.0, 2.0, 750); clamp_toggle();

	robotControl.lateral_move(-14, 2.0, 750); 
	
	robotControl.lateral_move(20, 2.0, 750);

	lady_brown_load1();

	robotControl.follow_route_pursuit(
		fourth_route, 18.0, 90, true, 15.0, 1500
	); w;

	robotControl.lateral_to_position(kPos(-7, 38, rad(90)), false, 4.0, 2200); // used to be point

	// robotControl.turn_to_point(kPos(0, 100, 0), true, 0, 2.0, 750);

	robotControl.lateral_move(28, 2.0, 1000); // ram wall stake

	// ---- RESET POSITION ON WALL STAKE ----

	chassis->set_position(kPos(
		cos(chassis->get_position().heading) * GUIDE_TO_TRACKING_CENTER_DIST,
		(72 - WALL_STAKE_POLE_DIST) - sin(chassis->get_position().heading) * GUIDE_TO_TRACKING_CENTER_DIST,
		chassis->get_position().heading
	));

	// ---- END ----

	intake_in(); lady_brown_score(); pros::delay(400); 
	robotControl.lateral_move(-10, 2.0, 750, false); lady_brown_load1(); 
	pros::delay(300); 
	intake_in(); robotControl.lateral_move(12, 2.0, 750, false); 
	pros::delay(500); lady_brown_score(); pros::delay(500);

	robotControl.lateral_move(-10); lady_brown_load1();

	// end of 2nd side

	// robotControl.follow_route_pursuit(
	// 	fifth_route, 13.0, 90, false, 5.0, 1600
	// );
	// robotControl.curve_move(kPos(40, 44, 0), false, 5.0, 3000); lady_brown_load1();
	robotControl.turn_to_point(kPos(60, 24, 0), false, 0, 3.0, 750); w;
	robotControl.lateral_move(-48, 6.0, 1500); w;
	robotControl.turn_to_point(kPos(60, 24, 0), false, 0, 3.0, 400);
	robotControl.lateral_move(-16, 3.0, 500); 
	clamp_toggle(); lady_brown_load1(); intake_in(); w;

	// robotControl.turn_to_angle(146, 0, 3.0, 750); w;
	// robotControl.lateral_move(-36, 4.0, 1500); w;
	// robotControl.lateral_move(-22, 4.0, 1500); w;

	robotControl.turn_to_angle(95, 0, 2.0, 600);

	robotControl.lateral_move(28, 2.0, 750); w;

	robotControl.turn_to_angle(225, 0, 2.0, 800);
	clamp_toggle(); intake_bottom.move(-INTAKE_VELOCITY);
	
	robotControl.lateral_move(-7, 3.0, 400);

	robotControl.turn_to_angle(225, 0, 2.0, 500);
	robotControl.lateral_move(16, 4.0, 500);

	robotControl.turn_to_angle(90, 0, 3.0, 750);
	
	robotControl.curve_move(kPos(48, 0, 0), false, 24.0, 750); w;
	robotControl.turn_to_angle(90, 0, 3.0, 400);
	robotControl.lateral_move(-12); robotControl.lateral_move(-12);
	
	clamp_toggle();

	robotControl.lateral_to_point(kPos(48, 2, 0), true, 3.0, 400, 0, 0);

	// ^ grab 3rd mogo

	// robotControl.turn_to_point(kPos(72, 0, 0), true, 0, 3.0, 750);
	robotControl.turn_to_angle(0, 0, 3.0, 750);

	robotControl.lateral_move(24, 3.0, 600); // could maybe reset pos here 

	robotControl.lateral_move(-8, 3.0, 600); w;

	lady_brown_alliance(); pros::delay(600); intake_out();

	// ^ score on second alliance
	
	robotControl.lateral_move(-8, 3.0, 750); lady_brown_down();  intake_in();

	if (colorSortTask == nullptr) {
		colorSortTask = new pros::Task {[=] {
			while(true) {
				if (color_sorting || auton_color_sorting) {
					if (color_sorting) {
						if (red_alliance) {
							blue_color_sort();
						}
						else {				
							red_color_sort();
						}
					} else if (auton_color_sorting) {
						if (red_alliance) {
							blue_color_auton_sort();
						}
						else {				
							red_color_auton_sort();
						}
					}
					
					pros::delay(20);
				}
				else {
					pros::delay(150);
				}
			}
		}};

		colorSortTask->set_priority(TASK_PRIORITY_DEFAULT - 2);
	}

	color_sorting = true;
	red_alliance = true;

	robotControl.lateral_to_point(kPos(24, 24, rad(225)), true, 6.0, 750, 750);

	robotControl.lateral_to_point(kPos(-2, 0, rad(225)), true, 8.0, 700, 750);

	robotControl.lateral_to_point(kPos(24, -24, rad(0)), true, 6.0, 800, 750);

	robotControl.lateral_to_point(kPos(46, -46, rad(0)), true, 10.0, 500, 500);
	robotControl.turn_to_angle(0, 0, 2.0, 600);
  
	robotControl.lateral_move(9, 2.0, 600);

	// robotControl.follow_route_pursuit(
	// 	sixth_route, 12.0, 75, true, 14.0, 1250
	// ); w;

	robotControl.turn_to_angle(235, 0, 3.0, 600); w;
	
	robotControl.lateral_move(10, 3.0, 500); doinker_toggle(); w;

	robotControl.turn_to_angle(135, LEFT, 6.0, 3000); clamp_toggle(); doinker_toggle(); w;

	robotControl.lateral_move(-16, 3.0, 600);

	robotControl.lateral_move(8, 3.0, 750); w;

	lady_brown_score();

	robotControl.lateral_to_point(kPos(24, -24, rad(315)), false, 2.0, 1000, 750, 40);
	robotControl.turn_to_angle(315, 0, 3.0, 400);

	pros::delay(20);

	drivetrain.voltage_command(-70, -70);

	// ^ end
}

// coded for red side
void sig_winpoint_red(knights::RobotChassis *chassis, bool flip_x, bool flip_y){
	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);

	auton_color_sorting = true;
	color_sorting = false;

	red_alliance = true;

	lady_brown_alliance();

	pros::delay(400);

	robotControl.lateral_move(-5, 3.0, 250);

	lady_brown_down();

	// robotControl.turn_to_angle(-170, 0, 2.0, 750); w;
	robotControl.turn_to_point(kPos(
		-24, 24, 0), false, 0, 2.0, 750);

	robotControl.lateral_move(-19, 3.0, 750);

	robotControl.lateral_move(-15, 3.0, 750);

	clamp_toggle(); tw(1);

	robotControl.turn_to_angle((42) , 0, 2.0, 750); w;

	intake_in();

	robotControl.lateral_move(21, 3.0, 750);

	robotControl.lateral_move(-20, 3.0, 750);

	robotControl.turn_to_angle((85), 0, 2.0, 500); w;

	robotControl.lateral_move(20, 3.0, 750); w;

	robotControl.lateral_to_point(kPos(-48, 24, knights::to_rad(270)), true, 3.0, 750);

	robotControl.lateral_to_point(kPos(-48, 4, knights::to_rad(270)), true, 3.0, 750, 750);

	w;

	robotControl.lateral_to_point(kPos(-48, -18 ,knights::to_rad(270)), true, 3.0, 750, 200);

	clamp_toggle();

	robotControl.turn_to_point(kPos(-24, -18, 0), false, 0, 2.0, 750);
	// robotControl.turn_to_angle((85 - angle_mod) * x_mod, 0, 2.0, 500); w;


	robotControl.lateral_move(-16, 3.0, 750); 

	robotControl.lateral_move(-10, 3.0, 750); 
	
	w; clamp_toggle(); w;

	auton_color_sorting = false;
	color_sorting = true;

	intake_in(); intake_in();

	robotControl.turn_to_point(kPos(-24, -50, 0), true, 0, 2.0, 750); w;

	robotControl.lateral_move(16, 3.0, 750);

	robotControl.turn_to_point(kPos(-12, 0,0), true, 0, 2.0, 750);

	robotControl.lateral_move(31, 3.0, 750);
}

void sig_winpoint_red_cut(knights::RobotChassis *chassis, bool flip_x, bool flip_y){
	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);

	auton_color_sorting = true;
	color_sorting = false;

	red_alliance = true;

	lady_brown_alliance();

	pros::delay(400);

	robotControl.lateral_move(-5, 3.0, 250);

	lady_brown_down();

	// robotControl.turn_to_angle(-170, 0, 2.0, 750); w;
	robotControl.turn_to_point(kPos(
		-24, 24, 0), false, 0, 2.0, 750);

	robotControl.lateral_move(-19, 3.0, 750);

	robotControl.lateral_move(-15, 3.0, 750);

	clamp_toggle(); tw(1);

	robotControl.turn_to_angle((42) , 0, 2.0, 750); w;

	intake_in();

	robotControl.lateral_move(21, 3.0, 750);

	robotControl.lateral_move(-20, 3.0, 750);

	robotControl.turn_to_angle((85), 0, 2.0, 500); w;

	robotControl.lateral_move(20, 3.0, 750); w;

	robotControl.turn_to_point(kPos(-24, 0, 0)); w;

	robotControl.lateral_move(30);

	auton_color_sorting = false;

}

void sig_winpoint_blue(knights::RobotChassis *chassis, bool flip_x, bool flip_y){

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);

	auton_color_sorting = true;
	color_sorting = false;

	red_alliance = false;

	lady_brown_alliance();

	pros::delay(400);

	robotControl.lateral_move(-5, 3.0, 250);

	lady_brown_down();

	// robotControl.turn_to_angle(-170, 0, 2.0, 750); w;
	robotControl.turn_to_point(kPos(24, 24, 0), false, 0, 2.0, 750);

	robotControl.lateral_move(-19, 3.0, 750);

	robotControl.lateral_move(-15, 3.0, 750);

	clamp_toggle(); tw(1);

	robotControl.turn_to_angle((138), 0, 2.0, 750); w;

	intake_in();

	robotControl.lateral_move(21, 3.0, 750);

	robotControl.lateral_move(-20, 3.0, 750);

	robotControl.turn_to_angle((95), 0, 2.0, 500); tw(2);

	robotControl.lateral_move(20, 3.0, 750); w;

	robotControl.lateral_to_point(kPos(52, 24, knights::to_rad(270)), true, 3.0, 750);

	robotControl.lateral_to_point(kPos(48, 4, knights::to_rad(270)), true, 3.0, 750, 750);

	w;

	robotControl.lateral_to_point(kPos(48, -18, knights::to_rad(270)), true, 3.0, 750, 200);

	clamp_toggle();

	robotControl.turn_to_point(kPos(24, -18, 0), false, 0, 2.0, 750);
	// robotControl.turn_to_angle((85 - angle_mod) * x_mod, 0, 2.0, 500); w;


	robotControl.lateral_move(-16, 3.0, 750); 

	robotControl.lateral_move(-10, 3.0, 750); 
	
	w; clamp_toggle(); w;

	auton_color_sorting = false;
	color_sorting = true;

	intake_in(); intake_in();

	robotControl.turn_to_point(kPos(24, -50, 0), true, 0, 2.0, 750); w;

	robotControl.lateral_move(18, 3.0, 750);

	robotControl.turn_to_point(kPos(12, 0, 0), true, 0, 2.0, 750);

	robotControl.lateral_move(35, 3.0, 750);
}

void sig_winpoint_blue_cut(knights::RobotChassis *chassis, bool flip_x, bool flip_y){

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);

	auton_color_sorting = true;
	color_sorting = false;

	red_alliance = false;

	lady_brown_alliance();

	pros::delay(400);

	robotControl.lateral_move(-5, 3.0, 250);

	lady_brown_down();

	// robotControl.turn_to_angle(-170, 0, 2.0, 750); w;
	robotControl.turn_to_point(kPos(24, 24, 0), false, 0, 2.0, 750);

	robotControl.lateral_move(-19, 3.0, 750);

	robotControl.lateral_move(-15, 3.0, 750);

	clamp_toggle(); tw(1);

	robotControl.turn_to_angle((138), 0, 2.0, 750); w;

	intake_in();

	robotControl.lateral_move(21, 3.0, 750);

	robotControl.lateral_move(-20, 3.0, 750);

	robotControl.turn_to_angle((95), 0, 2.0, 500); tw(2);

	robotControl.lateral_move(20, 3.0, 750); w;

	robotControl.turn_to_point(kPos(24, 0, 0)); w;

	robotControl.lateral_move(-35);

}

ASSET(ringrushfirst_txt)

void ring_5_rush_blue(knights::RobotChassis *chassis, bool flip_x, bool flip_y){ // blue

	float x_mod = flip_x ? -1 : 1;
	float y_mod = flip_y ? -1 : 1;

	float angle_mod = flip_y ? 180 : 0;

	knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);

	AssetStream first(ringrushfirst_txt);
	knights::Route first_route = knights::init_route_from_asset(first);

	color_sorting = true;
	red_alliance = false;

	doinker_toggle();

	intake_bottom.move(INTAKE_VELOCITY);

	robotControl.lateral_move(44,8.0); w;

	robotControl.turn_to_angle(128, 0, 5.0, 500); w;

	robotControl.lateral_move(-15);

	robotControl.lateral_move(-12); w;

	clamp_toggle(); w;

	robotControl.turn_to_angle(100,0,5.0, 600); w; intake_in();

	doinker_toggle();

	robotControl.lateral_to_point(kPos(24,48,0), true, 3.0, 1000, 600); w;

	robotControl.lateral_to_point(kPos(48,24,0)); w;

	robotControl.lateral_to_point(kPos(48,5,0)); w;

	robotControl.lateral_move(24); w;	
}

void ring_5_rush_red(knights::RobotChassis *chassis, bool flip_x, bool flip_y){ // blue

	float x_mod = flip_x ? -1 : 1;
	float y_mod = flip_y ? -1 : 1;

	float angle_mod = flip_y ? 180 : 0;

	knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);

	AssetStream first(ringrushfirst_txt);
	knights::Route first_route = knights::init_route_from_asset(first);

	color_sorting = true;
	red_alliance = false;

	doinker_toggle2();

	intake_bottom.move(INTAKE_VELOCITY);

	robotControl.lateral_move(44,8.0); w;

	robotControl.turn_to_angle(52, 0, 5.0, 500); w;

	robotControl.lateral_move(-15);

	robotControl.lateral_move(-12); w;

	clamp_toggle(); w;

	robotControl.turn_to_angle(80,0,5.0, 600); w; intake_in();

	doinker_toggle2();

	robotControl.lateral_to_point(kPos(-24,48,0), true, 3.0, 1000, 600); w;

	robotControl.lateral_to_point(kPos(-48,24,0)); w;

	robotControl.lateral_to_point(kPos(-48,5,0)); w;

	robotControl.lateral_move(24); w;	
}


void ring_6_rush_blue(knights::RobotChassis *chassis, bool flip_x, bool flip_y){ // blue

	float x_mod = flip_x ? -1 : 1;
	float y_mod = flip_y ? -1 : 1;

	float angle_mod = flip_y ? 180 : 0;

	knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 15.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, false);

	AssetStream first(ringrushfirst_txt);
	knights::Route first_route = knights::init_route_from_asset(first);

	color_sorting = true;
	red_alliance = false;

	doinker_toggle();

	intake_bottom.move(INTAKE_VELOCITY);

	robotControl.lateral_move(44,8.0); w;

	robotControl.turn_to_angle(128, 0, 5.0, 500); w;

	robotControl.lateral_move(-15);

	robotControl.lateral_move(-12); w;

	clamp_toggle(); w;

	robotControl.turn_to_angle(100,0,5.0, 600); w; intake_in();

	doinker_toggle();

	robotControl.lateral_to_point(kPos(24,48,0), true, 3.0, 1000, 600); w;

	robotControl.lateral_to_point(kPos(53,53,0)); w;

	robotControl.turn_to_angle(45); w;

	doinker_toggle2(); w;

	robotControl.turn_to_angle(-45); w;

	doinker_toggle2();

	robotControl.turn_to_angle(45); w;

	robotControl.lateral_move(12); w;

	robotControl.lateral_move(-12); w;	

	robotControl.lateral_to_point(kPos(48,24,0)); w;

	robotControl.lateral_to_point(kPos(48,5,0)); w;

	robotControl.lateral_move(24); w;	
}


void empty(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
	return;
}

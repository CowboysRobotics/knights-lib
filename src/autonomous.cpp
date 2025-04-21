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

ASSET(testroute2_txt)
ASSET(testroute_vaw)

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

	std::fstream write_file("/usd/motion_output.txt", std::ios_base::out);
	for (knights::ProfileTimestamp timestamp : profile.timestamps) {
		write_file << "time: " << timestamp.time << " ";
		write_file << "pos: " << timestamp.position.x << " " << timestamp.position.y << " " << timestamp.position.heading << " ";
		write_file << "lin vel: " << timestamp.linear_velocity << " ";
		write_file << "angular vel: " << timestamp.angular_velocity << " ";
		write_file << "side vels (r,l): " << timestamp.right_speed << " " << timestamp.left_speed << " ";
		write_file << "\n";
	}
	write_file.close();

	// robotControl.follow_profile(profile);

	for (int test_rpm = 0; test_rpm < 600; test_rpm += 50) {

		right_mtrs.move_velocity(test_rpm);
		left_mtrs.move_velocity(test_rpm);

		float prev_x; 
		float prev_time;

		std::fstream velo_write_file("/usd/velocity_" + std::to_string(test_rpm) + ".txt", std::ios_base::out);

		for (int i = 0; i < 40; i++) {

			float curr_x = chassis->get_position().x;

			float instant_velocity = (curr_x - prev_x) / (pros::millis() - prev_time);

			prev_x = curr_x;
			prev_time = pros::millis();

			velo_write_file << instant_velocity << "\n";

			pros::delay(30);
		}

		velo_write_file.close();

		right_mtrs.move_velocity(0);
		left_mtrs.move_velocity(0);

	}
	
}

ASSET(red_left_1_txt)

void red_left_wp(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
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

	lady_brown_alliance();

	robotControl.lateral_move(4);

	pros::delay(300);

	robotControl.lateral_move(-6);

	lady_brown_down();

	robotControl.lateral_move(-23); // -25

	robotControl.lateral_move(-6);

	pros::delay(200);

	clamp_toggle();

	pros::delay(400);

	robotControl.turn_to_angle(30);

	intake_in();

	auto profile = generator.generate(
		chassis->get_position(), knights::Pos(-15, 59, 90_deg), 80); // off

	robotControl.follow_profile(profile);

	// robotControl.lateral_move(-2);

	// pros::delay(500);

	intake_in();

	robotControl.turn_to_angle(215);

	pros::delay(100);

	intake_in();

	robotControl.lateral_move(20, 3.0, 1000, false);

	lady_brown_load1();

	// pros::delay(500);

	robotControl.turn_to_angle(58);

	pros::delay(200);

	robotControl.lateral_move(28);

	lady_brown_score();

	pros::delay(800);

	robotControl.lateral_move(-6);

	lady_brown_down();

	robotControl.turn_to_angle(225);

	intake_in();

	// ramsete_constants.curvature_coefficient = 2;

	// auto profile2 = generator.generate(
	// 	chassis->get_position(), knights::Pos(-82, 60, 135_deg), 110, 15, true); // off bc odom

	// robotControl.follow_profile(profile2, 8.0);

	auto assetstream1 = AssetStream(red_left_1_txt);

	lady_brown_score(); intake_in();

	robotControl.follow_route(knights::init_route_from_asset(assetstream1), 18.0, 80, true, 4.0, 1500);

	pros::delay(400);

	robotControl.lateral_move(-12);

	lady_brown_down();

	robotControl.turn_to_angle(145);

	robotControl.lateral_move(-38, 3.0, 2000);

	intake_in();

	robotControl.lateral_move(-18);
	
}

void pp_test(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {

	std::string s = "rush.txt";

	knights::AdvancedRoute test_route = advanced_route_from_file(s);
	// for (auto route : test_route.routes) {
	// 	for (auto pt : route.second.positions) {
	// 		// knights::logger::green(knights::logger::string_format("pt: %lf %lf %lf", pt.x, pt.y, pt.heading));
	// 	}
	// }

    knights::RamseteConstants ramsete_constants;

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

	// test_route.execute(chassis, &robotControl, &inputMap);
}

#define WAIT 170
#define kPos knights::Pos
#define rad(x) knights::to_rad(x)

#define w pros::delay(WAIT)
#define tw(x) pros::delay((x)*WAIT)

#define GUIDE_TO_TRACKING_CENTER_DIST 9 // need to cange
#define WALL_STAKE_POLE_DIST 1

void skills(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {

    knights::RamseteConstants ramsete_constants;

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

}

void empty(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
	return;
}

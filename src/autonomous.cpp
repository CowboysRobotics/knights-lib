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
ASSET(red_left_wallstake_txt)

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

	pros::delay(200);

	robotControl.lateral_move(-6);

	lady_brown_down();

	robotControl.lateral_move(-23); // -25

	robotControl.lateral_move(-6);


	clamp_toggle();

	pros::delay(200);

	robotControl.turn_to_angle(22);

	intake_in();

	auto profile = generator.generate(
		chassis->get_position(), knights::Pos(-6, 54, 90_deg), 80); // off

	robotControl.follow_profile(profile);

	// robotControl.lateral_move(-2);

	pros::delay(300);

	// intake_in();

	robotControl.turn_to_angle(200);

	pros::delay(300);

	// intake_in();

	robotControl.lateral_move(24, 3.0, 1000, false);

	lady_brown_load1();

	pros::delay(200);

	// // ## NEW METHOD
	robotControl.turn_to_angle(70);
	auto assetstream2 = AssetStream(red_left_wallstake_txt);

	robotControl.follow_route(knights::init_route_from_asset(assetstream2), 7.0, 80, true, 4.0, 600);

	robotControl.turn_to_angle(40, 0, 1.0, 600);

	robotControl.lateral_move(12);

	// // ## OLD METHOD
	// robotControl.turn_to_angle(44);
	// pros::delay(200);
	// robotControl.lateral_move(25);

	lady_brown_score();

	pros::delay(800);

	robotControl.lateral_move(-7);

	lady_brown_down();

	robotControl.turn_to_angle(225);

	intake_in();

	// ramsete_constants.curvature_coefficient = 2;

	// auto profile2 = generator.generate(
	// 	chassis->get_position(), knights::Pos(-82, 60, 135_deg), 110, 15, true); // off bc odom

	// robotControl.follow_profile(profile2, 8.0);

	auto assetstream1 = AssetStream(red_left_1_txt);

	lady_brown_score(); intake_in();

	robotControl.follow_route(knights::init_route_from_asset(assetstream1), 13.0, 80, true, 4.0, 1700);

	pros::delay(400);

	robotControl.lateral_move(-12);

	lady_brown_down();

	robotControl.lateral_to_point(kPos(-22, 22, 225));

	lady_brown_tip();

	// pros::delay(400);

	// intake_in();

	// robotControl.lateral_move(14, 4.0, 750);


	
	
}

ASSET(red_left_tworing_txt)

void red_left_wp_safe(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
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

	auto assetstream2 = AssetStream(red_left_tworing_txt);

	lady_brown_alliance();

	robotControl.lateral_move(4);

	pros::delay(200);

	robotControl.lateral_move(-6);

	lady_brown_down();

	robotControl.lateral_move(-20); // -25

	robotControl.lateral_move(-14);

	clamp_toggle();

	pros::delay(200);

	robotControl.turn_to_angle(48);

	pros::delay(140);

	robotControl.lateral_move(21);

	intake_in();

	robotControl.turn_to_angle(90);

	robotControl.lateral_move(17);

	pros::delay(200);


	// auto profile = generator.generate(
	// 	chassis->get_position(), knights::Pos(-10, 65, 90_deg), 80); // off

	// robotControl.follow_route(knights::init_route_from_asset(assetstream2), 8.0, 60, true, 8.0, 2800, false);

	// robotControl.lateral_move(12, 3.0, 600);

	robotControl.turn_to_angle(220, 0, 2.0, 800);

	// robotControl.lateral_move(24, 3.0, 750);

	lady_brown_score(); pros::delay(200); intake_in();

	auto profile3 = generator.generate(
		chassis->get_position(), knights::Pos(-65, 65, 135_deg), 90); // off

	robotControl.follow_route(knights::Route(profile3), 9.0, 80, true, 6.0, 2500);

	robotControl.lateral_move(15, 3.0, 750);

	pros::delay(400);

	intake_raise.toggle();
	// // 2nd ring
	robotControl.lateral_move(-14);

	pros::delay(400);

	robotControl.lateral_move(14, 3.0, 750);

	intake_raise.toggle();

	robotControl.lateral_move(-22, 5.0, 750);

	pros::delay(200);

	robotControl.turn_to_point(kPos(-51, 0, 270_deg), true, 0, 0.0, 750);

	intake_raise.toggle();

	pros::delay(300);

	lady_brown_down();


	robotControl.lateral_move(27);

	robotControl.lateral_move(18);

	intake_raise.toggle();

	lady_brown_load2();

	pros::delay(200);

	robotControl.turn_to_angle(-5, 0, 0, 750);
	
	pros::delay(200);

	robotControl.lateral_move(14);

	// pros::delay(400);

	// intake_in();

	// robotControl.lateral_move(14, 4.0, 750);


	
	
}

void red_left_elim(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
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

	auto assetstream2 = AssetStream(red_left_tworing_txt);

	lady_brown_alliance();

	robotControl.lateral_move(4);

	pros::delay(200);

	robotControl.lateral_move(-6);

	lady_brown_down();

	robotControl.lateral_move(-20); // -25

	robotControl.lateral_move(-14);

	clamp_toggle();

	pros::delay(200);

	robotControl.turn_to_angle(48);

	pros::delay(140);

	robotControl.lateral_move(21);

	intake_in();

	robotControl.turn_to_angle(90);

	robotControl.lateral_move(17);

	pros::delay(200);


	// auto profile = generator.generate(
	// 	chassis->get_position(), knights::Pos(-10, 65, 90_deg), 80); // off

	// robotControl.follow_route(knights::init_route_from_asset(assetstream2), 8.0, 60, true, 8.0, 2800, false);

	// robotControl.lateral_move(12, 3.0, 600);

	robotControl.turn_to_angle(220, 0, 2.0, 800);

	// robotControl.lateral_move(24, 3.0, 750);

	lady_brown_score(); pros::delay(200); intake_in();

	auto profile3 = generator.generate(
		chassis->get_position(), knights::Pos(-65, 65, 135_deg), 90); // off

	robotControl.follow_route(knights::Route(profile3), 9.0, 80, true, 6.0, 2500);

	robotControl.lateral_move(15, 3.0, 750);

	pros::delay(400);

	intake_raise.toggle();
	// // 2nd ring
	robotControl.lateral_move(-14);

	pros::delay(400);

	robotControl.lateral_move(14, 3.0, 750);

	intake_raise.toggle();

	robotControl.lateral_move(-22, 5.0, 750);

	pros::delay(200);

	robotControl.turn_to_point(kPos(-51, 0, 270_deg), true, 0, 0.0, 750);

	intake_raise.toggle();

	pros::delay(300);

	lady_brown_down();


	robotControl.lateral_move(27);

	robotControl.lateral_move(18);

	intake_raise.toggle();

	lady_brown_load2();

	pros::delay(200);

	robotControl.turn_to_angle(-110, 0, 0, 750);
	
	pros::delay(200);

	robotControl.lateral_move(40);

	// pros::delay(400);

	// intake_in();

	// robotControl.lateral_move(14, 4.0, 750);


	
	
}

ASSET(red_right_middle_ring_txt)

void red_right_wp(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
    knights::RamseteConstants ramsete_constants(0.7, 2, 1.4);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0, 3.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::PIDController angularPID(50, 0, 10, -25.0, 25.0);

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, &angularPID, &ramsete_constants, false);
	
	knights::ProfileGenerator generator(drivetrain, 80);

	lady_brown_alliance();

	pros::delay(700);

	robotControl.lateral_move(-6, 2.0);

	lady_brown_down();

	intake_raise.toggle();

	pros::delay(200);

	robotControl.turn_to_point(kPos(-48, 0, 0)); 
	
	// intake_in();
	intake_bottom.move(300);

	robotControl.lateral_move(16);

	robotControl.lateral_move(12);

	intake_raise.toggle();

	pros::delay(300);

	// intake_bottom.move(0);

	auto profile = generator.generate(
		chassis->get_position(), knights::Pos(-30, -18, 135_deg), 70, 15, false); // off

	profile.dump();

	robotControl.follow_profile(profile);

	clamp_toggle();

	pros::delay(300);

	intake_in();

	robotControl.turn_to_point(kPos(-26, -48, 0));

	lady_brown_load1();

	pros::delay(200);

	robotControl.lateral_move(16);

	pros::delay(200);

	// robotControl.turn_to_angle();

	robotControl.lateral_to_point(kPos(-9, -58, 305), true, 3.0, 750);

	pros::delay(200);

	// robotControl.lateral_move(22);

	intake_in();

	lady_brown_score();

	pros::delay(800);

	lady_brown_down();

	intake_in();

	robotControl.lateral_move(-35, 6.0, 2500);

	lady_brown_score(); intake_in();

	pros::delay(200);

	robotControl.turn_to_point(kPos(-72, -72, 0), true, 0, 0.0, 1000);

	pros::delay(200);

	robotControl.lateral_move(30);

	robotControl.lateral_move(30, 3.0, 900);

	pros::delay(300);

	robotControl.lateral_move(-12);

	lady_brown_down();

	robotControl.lateral_to_position(kPos(-16, -16, 45));

	// robotControl.lateral_move(8);

	lady_brown_tip();

	

	




}


void red_right_wp_safe(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
    knights::RamseteConstants ramsete_constants(0.7, 2, 1.4);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0, 3.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::PIDController angularPID(50, 0, 10, -25.0, 25.0);

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, &angularPID, &ramsete_constants, false);
	
	knights::ProfileGenerator generator(drivetrain, 80);

	lady_brown_alliance();

	pros::delay(700);

	robotControl.lateral_move(-6, 2.0);

	lady_brown_down();

	intake_raise.toggle();

	pros::delay(200);

	robotControl.turn_to_point(kPos(-48, 0, 0)); 
	
	// intake_in();
	intake_bottom.move(300);

	robotControl.lateral_move(16);

	robotControl.lateral_move(12);

	intake_raise.toggle();

	pros::delay(300);

	robotControl.turn_to_angle(110, 0, 3.0, 1000);

	auto profile = generator.generate(
		chassis->get_position(), knights::Pos(-26, -21, 135_deg), 80, 15, false); // off

	robotControl.follow_route(knights::Route(profile),
		6.0, 70, false, 5.0, 2000, true);

	robotControl.lateral_move(-8);

	clamp_toggle();

	pros::delay(400);

	intake_in();

	robotControl.turn_to_angle(215, 0, 3.0, 1000);

	pros::delay(200);

	auto profile2 = generator.generate(
		chassis->get_position(), knights::Pos(-64, -60, 225_deg), 80, 10, true); // off
	
	robotControl.follow_route(knights::Route(profile2),
		7.0, 90, true, 5.0, 2000, true);

	drivetrain.voltage_command(110, 110);

	pros::delay(500);

	drivetrain.voltage_command(0, 0);

	pros::delay(100);

	drivetrain.voltage_command(-60, -60);

	pros::delay(700);

	intake_raise.toggle();

	robotControl.lateral_move(14);

	intake_raise.toggle();

	pros::delay(200);

	robotControl.lateral_move(-20, 5.0, 800);

	pros::delay(200);

	robotControl.lateral_to_point(kPos(-24, -48, 90), true, 5.0, 750);

	pros::delay(140);

	robotControl.turn_to_angle(130);

	lady_brown_load2();

	auto profile3 = generator.generate(
		chassis->get_position(), knights::Pos(-14, -14, 45_deg), 80, 10, true); // off
	
	robotControl.follow_route(knights::Route(profile3),
		7.0, 70, true, 5.0, 2000, true);
	
	robotControl.lateral_move(6);
	

}


void blue_right_wp_safe(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
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

	pros::delay(200);

	robotControl.lateral_move(-6);

	lady_brown_down();

	robotControl.lateral_move(-23); // -25

	robotControl.lateral_move(-6);


	clamp_toggle();

	pros::delay(200);

	robotControl.turn_to_angle(135);

	intake_in();

	auto profile = generator.generate(
		chassis->get_position(), knights::Pos(11, 50, 90_deg), 80); // off

	robotControl.follow_profile(profile);

	robotControl.turn_to_angle(340, 0, 2.0, 800);

	auto assetstream1 = AssetStream(red_left_1_txt);

	lady_brown_score(); pros::delay(200); intake_in();

	auto profile3 = generator.generate(
		chassis->get_position(), knights::Pos(66, 63, 45_deg), 80); // off

	robotControl.follow_route(knights::Route(profile3), 9.0, 90, true, 6.0, 2800);

	// pros::delay(200);

	robotControl.lateral_move(8, 3.0, 750);

	// robotControl.lateral_move(-18);

	drivetrain.voltage_command(-60, -60);

	pros::delay(700);

	intake_raise.toggle();

	robotControl.lateral_move(18);

	robotControl.lateral_move(-20);

	pros::delay(200);

	robotControl.turn_to_point(kPos(56, 0, 270_deg));

	pros::delay(300);

	lady_brown_down();

	robotControl.lateral_move(27);

	robotControl.lateral_move(18);

	intake_raise.toggle();

	pros::delay(200);

	robotControl.turn_to_angle(180, 0, 0, 750);
	
	lady_brown_load2();

	pros::delay(200);

	robotControl.lateral_move(16);

	// pros::delay(400);

	// intake_in();

	// robotControl.lateral_move(14, 4.0, 750);
	
}


void blue_left_wp_safe(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
    knights::RamseteConstants ramsete_constants(0.7, 2, 1.4);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0, 3.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::PIDController angularPID(50, 0, 10, -25.0, 25.0);

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, &angularPID, &ramsete_constants, false);
	
	knights::ProfileGenerator generator(drivetrain, 80);

	lady_brown_alliance();

	pros::delay(700);

	robotControl.lateral_move(-6, 2.0);

	lady_brown_down();

	intake_raise.toggle();

	pros::delay(200);

	robotControl.turn_to_point(kPos(48, 0, 180)); 
	
	// intake_in();
	intake_bottom.move(300);

	robotControl.lateral_move(16);

	robotControl.lateral_move(12);

	intake_raise.toggle();

	pros::delay(300);

	robotControl.turn_to_angle(70, 0, 3.0, 1000);

	auto profile = generator.generate(
		chassis->get_position(), knights::Pos(26, -21, 45_deg), 80, 15, false); // off

	robotControl.follow_route(knights::Route(profile),
		6.0, 70, false, 5.0, 2000, true);

	robotControl.lateral_move(-8);

	clamp_toggle();

	pros::delay(400);

	intake_in();

	robotControl.turn_to_angle(325, 0, 3.0, 1000);

	pros::delay(200);

	auto profile2 = generator.generate(
		chassis->get_position(), knights::Pos(64, -60, 225_deg), 80, 10, true); // off
	
	robotControl.follow_route(knights::Route(profile2),
		7.0, 90, true, 5.0, 2000, true);

	drivetrain.voltage_command(110, 110);

	pros::delay(500);

	drivetrain.voltage_command(0, 0);

	pros::delay(100);

	drivetrain.voltage_command(-60, -60);

	pros::delay(700);

	intake_raise.toggle();

	robotControl.lateral_move(14);

	intake_raise.toggle();

	pros::delay(200);

	robotControl.lateral_move(-20, 5.0, 800);

	pros::delay(200);

	robotControl.lateral_to_point(kPos(24, -48, 90), true, 5.0, 750);

	pros::delay(140);

	robotControl.turn_to_angle(50);

	lady_brown_load2();

	auto profile3 = generator.generate(
		chassis->get_position(), knights::Pos(14, -14, 135_deg), 80, 10, true); // off
	
	robotControl.follow_route(knights::Route(profile3),
		7.0, 70, true, 5.0, 2000, true);
	
	robotControl.lateral_move(6);
	

}


void red_lb_first_wp_right(knights::RobotChassis *chassis, bool flip_x, bool flip_y){
	knights::RamseteConstants ramsete_constants(0.7, 2, 1.4);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0, 3.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::PIDController angularPID(50, 0, 10, -25.0, 25.0);

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, &angularPID, &ramsete_constants, false);
	
	knights::ProfileGenerator generator(drivetrain, 80);

	robotControl.lateral_move(20); pros::delay(250);

	robotControl.lateral_move(21); pros::delay(250);

	robotControl.turn_to_angle(-41);pros::delay(250);

	robotControl.lateral_move(8);pros::delay(250);

	lady_brown_score(); pros::delay(500);

	robotControl.lateral_move(-10);pros::delay(250);

	robotControl.lateral_to_point(kPos(-24, -24, 90), false, 5.0, 750); pros::delay(100);

	robotControl.lateral_move(-10);

	clamp_toggle(); pros::delay(100);

	intake_in();

	robotControl.turn_to_angle(270);

	robotControl.lateral_move(24);

	robotControl.lateral_to_point(kPos(-48, -48, 225), true, 5.0, 750);

	robotControl.turn_to_angle(225);

	robotControl.lateral_move(19);

	drivetrain.voltage_command(110, 110);

	pros::delay(500);

	drivetrain.voltage_command(0, 0);

	pros::delay(100);


	drivetrain.voltage_command(-60, -60);

	pros::delay(250); intake_raise.toggle();

	robotControl.lateral_move(10); intake_raise.toggle();

	robotControl.lateral_move(-24);


}




void pp_test(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {

	knights::RamseteConstants ramsete_constants(0.7, 2, 1.3);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0, 3.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::PIDController angularPID(50, 0, 10, -25.0, 25.0);

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, &angularPID, &ramsete_constants, false);
	
	knights::ProfileGenerator generator(drivetrain, 80);

	auto profile = generator.generate(
		chassis->get_position(), knights::Pos(-4, -45, 160_deg), 80, 15, false); // off

	profile.dump();

	robotControl.follow_route(knights::Route(profile), 14.0, 100.0, false);

	clamp_toggle();

}

#define WAIT 170
#define kPos knights::Pos
#define rad(x) knights::to_rad(x)

#define w pros::delay(WAIT)
#define tw(x) pros::delay((x)*WAIT)

#define GUIDE_TO_TRACKING_CENTER_DIST 9 // need to cange
#define WALL_STAKE_POLE_DIST 1

ASSET(skillsfirst_txt)

void skills(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {

    knights::RamseteConstants ramsete_constants(0.7, 2, 1.4);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0, 3.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::PIDController angularPID(50, 0, 10, -25.0, 25.0);

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, &angularPID, &ramsete_constants, false);
	
	knights::ProfileGenerator generator(drivetrain, 80);

	lady_brown_alliance();

	AssetStream first(skillsfirst_txt);
	auto first_route = knights::init_route_from_asset(first);
	first_route.add_action(kPos(-6, -36, 0), &lady_brown_load1);

	pros::delay(600);

	auto profile = generator.generate(
		chassis->get_position(), knights::Pos(-48, -24, 135_deg), 80, 15, false); // off

	robotControl.follow_route(knights::Route(profile),6.0, 70.0, false, 6.0, 2000);

	clamp_toggle();

	pros::delay(300);

	lady_brown_down();

	intake_in();

	robotControl.turn_to_angle(0);

	robotControl.follow_route(
		first_route, 9.0, 70, true, 3.0, 4000
	);

	pros::delay(200);

	profile = generator.generate(
		chassis->get_position(), knights::Pos(-1, -40, 0), 80, 15, false); // off

	robotControl.follow_route(knights::Route(profile), 8.0, 70.0, false, 2.0, 1500);

	pros::delay(200);

	robotControl.turn_to_angle(270, 0, 0.0, 1000);

	robotControl.lateral_move(23);

	// score on wallstake 1

	lady_brown_score();

	pros::delay(200);

	robotControl.lateral_move(-6);

	lady_brown_load1();

	pros::delay(200);

	intake_in();

	pros::delay(700);

	lady_brown_score();

	robotControl.lateral_move(10);

	pros::delay(200);

	lady_brown_down(); intake_in();

	robotControl.lateral_move(-12);

	robotControl.turn_to_angle(180);

	pros::delay(200);

	// three rings

	profile = generator.generate(
		chassis->get_position(), knights::Pos(-65, -48, 180_deg), 80, 15, true); // off

	robotControl.follow_route(knights::Route(profile), 9.0, 85.0, true, 2.0, 3000);

	pros::delay(200);

	robotControl.turn_to_angle(315);

	pros::delay(200);

	robotControl.lateral_move(8, 3.0, 750);

	robotControl.turn_to_angle(30);

	// first mogo in corner

	clamp_toggle();

	robotControl.lateral_move(-20);

	robotControl.lateral_move(20);

	pros::delay(200);

	robotControl.turn_to_angle(305);

	profile = generator.generate(
		chassis->get_position(), knights::Pos(-48, 28, 270_deg), 80, 15, false); // off

	robotControl.follow_route(knights::Route(profile), 12.0, 70.0, false, 6.0, 3000);

	clamp_toggle();





}

void blue_mogo_rush_left(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
	knights::RamseteConstants ramsete_constants(0.7, 2, 1.4);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0, 3.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::PIDController angularPID(50, 0, 10, -25.0, 25.0);

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, &angularPID, &ramsete_constants, false);
	
	knights::ProfileGenerator generator(drivetrain, 80);

	doinker_toggle();

	robotControl.lateral_move(40, 3.0, 3000);

	doinker_toggle();

	pros::delay(150);

	robotControl.lateral_move(-25, 6.0, 15000); // tug of war
	
	pros::delay(250);

	doinker_toggle();

	robotControl.turn_to_angle(135, 1, 3.0, 450);

	doinker_toggle(); 
	
	pros::delay(250);

	robotControl.turn_to_angle(8, 1); pros::delay(250);

	robotControl.lateral_move(-16);
	robotControl.lateral_move(-10);

	clamp_toggle(); pros::delay(100);

	intake_in();

	robotControl.lateral_move(36, 3.0, 2000); pros::delay(250);

	robotControl.turn_to_angle(85);

	intake_in();

	clamp_toggle(); 

	pros::delay(100);
	
	robotControl.lateral_move(6);

	pros::delay(200);
	
	robotControl.turn_to_point(kPos(24, -28, 0), false, 0, 0, 750); 
	
	pros::delay(250);

	robotControl.lateral_move(-20, 6.0, 1000);

	robotControl.lateral_move(-14, 4.0, 1000);

	clamp_toggle();

	pros::delay(250);

	robotControl.turn_to_angle(267); pros::delay(150);

	intake_in();

	robotControl.lateral_move(26);

	robotControl.turn_to_angle(360);

	pros::delay(150);

	robotControl.lateral_move(40, 6.0, 2200);

	pros::delay(150);

	robotControl.turn_to_point(kPos(72, -72, 0), true, 0, 3.0, 600);

	lady_brown_load1();

	robotControl.lateral_move(14, 3.0, 1000);

	robotControl.lateral_move(-6); pros::delay(150);

	robotControl.lateral_to_point(kPos(16, -65, 235), true, 6.0, 2000, 1000, 170);

	robotControl.turn_to_angle(230);
	
	pros::delay(150);

	robotControl.lateral_move(10);

	lady_brown_load2();

	intake_in();

}

void empty(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
	return;
}

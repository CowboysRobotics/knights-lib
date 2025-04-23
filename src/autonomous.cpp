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
		chassis->get_position(), knights::Pos(-6, 50, 90_deg), 80); // off

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

	lady_brown_alliance();

	robotControl.lateral_move(4);

	pros::delay(200);

	robotControl.lateral_move(-6);

	lady_brown_down();

	robotControl.lateral_move(-23); // -25

	robotControl.lateral_move(-6);


	clamp_toggle();

	pros::delay(200);

	robotControl.turn_to_angle(45);

	intake_in();

	auto profile = generator.generate(
		chassis->get_position(), knights::Pos(-11, 50, 90_deg), 80); // off

	robotControl.follow_profile(profile);

	robotControl.turn_to_angle(200, 0, 2.0, 800);

	auto assetstream1 = AssetStream(red_left_1_txt);

	lady_brown_score(); pros::delay(200); intake_in();

	auto profile3 = generator.generate(
		chassis->get_position(), knights::Pos(-66, 63, 135_deg), 80); // off

	robotControl.follow_route(knights::Route(profile3), 9.0, 90, true, 6.0, 2800);

	// pros::delay(200);

	robotControl.lateral_move(8, 3.0, 750);

	// robotControl.lateral_move(-18);

	drivetrain.voltage_command(-60, -60);

	pros::delay(700);

	robotControl.lateral_move(18);

	robotControl.lateral_move(-20);

	pros::delay(200);

	robotControl.turn_to_point(kPos(-56, 0, 270_deg));

	pros::delay(300);

	lady_brown_down(); intake_raise.toggle();

	robotControl.lateral_move(27);

	robotControl.lateral_move(18);

	intake_raise.toggle();

	pros::delay(200);

	robotControl.turn_to_angle(0, 0, 0, 750);
	
	lady_brown_load2();

	pros::delay(200);

	robotControl.lateral_move(16);

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

	robotControl.lateral_move(14);

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

	pros::delay(600);

	auto profile = generator.generate(
		chassis->get_position(), knights::Pos(-48, -24, 135_deg), 80, 15, true); // off

	robotControl.follow_route(knights::Route(profile),6.0, 60.0, true);

	clamp_toggle();

	pros::delay(300);

	lady_brown_down();

	intake_in();

	robotControl.turn_to_angle(0);

	profile = generator.generate(
		chassis->get_position(), knights::Pos(-24, -24, 315_deg), 80, 15, true); // off

	robotControl.follow_route(knights::Route(profile),10.0, 80.0, true);

	lady_brown_load1();

	profile = generator.generate(
		chassis->get_position(), knights::Pos(24, -48, 350_deg), 80, 15, true); // off

	robotControl.follow_route(knights::Route(profile),10.0, 80.0, true);

	pros::delay(200);

	profile = generator.generate(
		chassis->get_position(), knights::Pos(-3, -40, 330_deg), 80, 15, false); // off

	robotControl.follow_route(knights::Route(profile), 6.0, 70.0, false);

	pros::delay(200);

	robotControl.turn_to_angle(270, 0, 0.0, 1000);

	robotControl.lateral_move(23);

	lady_brown_score();





}

void empty(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
	return;
}

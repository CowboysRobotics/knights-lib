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

#define PROS_MAX_VOLTAGE 127

STATIC_FILE(testpp3_txt)

void pid_tuning(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
    knights::RamseteConstants ramsete_constants(0.7, 2.0);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::PIDController turnPID(TURN_kP_90, TURN_kI_90, TURN_kD_90, 10.0, 127.0);
	turnPID.add_constant(knights::to_rad(45), knights::PIDConstants(TURN_kP_45, TURN_kI_45, TURN_kD_45));
	turnPID.add_constant(knights::to_rad(90), knights::PIDConstants(TURN_kP_90, TURN_kI_90, TURN_kD_90));
	turnPID.add_constant(knights::to_rad(135), knights::PIDConstants(TURN_kP_135, TURN_kI_135, TURN_kD_135));
	turnPID.add_constant(knights::to_rad(180), knights::PIDConstants(TURN_kP_180, TURN_kI_180, TURN_kD_180));
	knights::PIDController angularPID(50, 0, 10, -25.0, 25.0);

	knights::RobotController robotControl(chassis, &lateralPID, &turnPID, &angularPID, false);

	std::fstream write_file("/usd/motion_output.txt", std::ios_base::out);

	write_file << "Profile Generation Start at t: " << pros::millis() << "\n";
	
	knights::ProfileGenerator generator(drivetrain, 100);
	knights::MotionProfile profile = generator.generate(chassis->get_position(), knights::Pos(-12, 36, 180_deg), 80, 30, true);
	
	write_file << "Profile Generation End at t: " << pros::millis() << "\n";
	
	for (knights::ProfileTimestamp timestamp : profile.timestamps) {
		write_file << "time: " << timestamp.time << " ";
		write_file << "pos: " << timestamp.position.x << " " << timestamp.position.y << " " << timestamp.position.heading << " ";
		write_file << "lin vel: " << timestamp.linear_velocity << " ";
		write_file << "angular vel: " << timestamp.angular_velocity << " ";
		write_file << "dist: " << timestamp.curr_distance << " ";
		write_file << "side vels (r,l): " << timestamp.right_speed << " " << timestamp.left_speed << " ";
		write_file << "\n";
	}

	write_file.close();

	// knights::Route test_route = knights::init_route_from_asset(testpp3_txt);

	// robotControl.follow_route_pursuit(
	// 	test_route, 20.0, 90.0, true, 6.0, 4000, true);

	robotControl.follow_profile_ramsete(profile);

	// robotControl.turn_to_angle(90);

	
}

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

}

void empty(knights::RobotChassis *chassis, bool flip_x, bool flip_y) {
	return;
}

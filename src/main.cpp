#include "main.h"
#include "autonomous.h"
#include "knights/autonomous/profile.hpp"
#include "knights/display.hpp"
#include "knights/logger/logger.hpp"
#include "knights/util/calculation.hpp"
#include "globals.h"
#include "knights/util/position.hpp"
#include "liblvgl/misc/lv_color.h"
#include "pros/misc.h"

#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <fstream>

pros::Task *odomTask = nullptr;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	while(imu.is_calibrating()) {
		pros::delay(10);
	}
	//make sure that the imu sensor is accurate before the start of a match
	knights::logger::blue("Initialization Begin");

	// // Different autons, None0 is the default auton
	auton_map["None0"] = knights::Auton(&alt_skills, knights::Pos(-60, 0, 0));

	auton_map["Red1"] = knights::Auton(&red_rush_right_wp, knights::Pos(-58, -15, M_PI));
	auton_map["Red2"] = knights::Auton(&red_left_wp, knights::Pos(-55.1,37.5,0));
	auton_map["Blue1"] = knights::Auton(&blue_rush_left_wp, knights::Pos(-58, -15, M_PI));
	auton_map["Blue2"] = knights::Auton(&blue_right_wp, knights::Pos(-55.1,37.5,0));

	auton_map["Skills0"] = knights::Auton(&skills, knights::Pos(-60, 0, 0));

	lv_display();

	// wait until everything is calibrated
	pros::delay(2000);

	knights::logger::blue("Initialization End");

	// // #### Competition Robot
	// //front of the bot is intake
	// //assign direction to left side drive-train motors 
	// left_mtrs.set_reversed(false, 0);
	// left_mtrs.set_reversed(false, 1);
	// left_mtrs.set_reversed(true, 2);
	// //assign direction to right side drive-train motors
	// right_mtrs.set_reversed(true, 0);
	// right_mtrs.set_reversed(true, 1);
	// right_mtrs.set_reversed(false, 2);
	// // ####

	// intake.set_reversed(false, 0);
	// intake.set_reversed(true, 1);
	lady_brown.set_reversed(true);

	// #### TEST AREA ####

	// path test
    float dist = distance_btwn(knights::Point(0, 0), knights::Point(48, 24));
    knights::HermiteSpline path(
        knights::Point(0, 0),
        knights::Point(48, 24),
        knights::Point(std::cos(90_deg) * dist, std::sin(90_deg) * dist),
        knights::Point(std::cos(90_deg) * dist, std::sin(90_deg) * dist)
    );

	auto t = knights::linspace(0, 1, 10);
	for (auto value : t) {
		knights::display::MapDot dot(5, 5, lv_palette_darken(LV_PALETTE_CYAN, 2));
		std::cout << "pt: " << path.position(value).x << " " << path.position(value).y << " " << path.position(value).heading << "\n";
		dot.set_field_pos(path.position(value));
	}

	// motion profile test
	knights::ProfileGenerator generator(drivetrain, 100);
	std::vector<knights::ProfileTimestamp> profile = generator.generate(knights::Pos(0, 0, 90_deg), knights::Pos(48, 24, 90_deg));

	std::fstream write_file("/usd/motion_output.txt", std::ios_base::out);

	for (knights::ProfileTimestamp timestamp : profile) {
		write_file << "time: " << timestamp.time << "\n";
		write_file << "pos: " << timestamp.position.x << " " << timestamp.position.y << " " << timestamp.position.heading << "\n";
		write_file << "lin vel: " << timestamp.linear_velocity << "\n";
		write_file << "angular vel: " << timestamp.angular_velocity << "\n";
		write_file << "dist: " << timestamp.curr_distance << "\n";
		write_file << "side vels (r,l): " << timestamp.right_speed << " " << timestamp.left_speed << "\n";
		write_file << "end timestamp\n";

		std::cout << "time: " << timestamp.time << " ";
		std::cout << "pos: " << timestamp.position.x << " " << timestamp.position.y << " " << timestamp.position.heading << " ";
		std::cout << "lin vel: " << timestamp.linear_velocity << " ";
		std::cout << "angular vel: " << timestamp.angular_velocity << " ";
		std::cout << "dist: " << timestamp.curr_distance << " ";
		std::cout << "side vels (r,l): " << timestamp.right_speed << " " << timestamp.left_speed << " ";
		std::cout << "end timestamp\n";
	}

}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// Query display for the selected buttons
	knights::display::AutonSelectionPackage package = knights::display::get_selected_auton();

	chassis.set_position(0,0,0);

	if (auton_map.contains(package.get_value())) {
		chassis.set_position(auton_map[package.get_value()].start);
	}

	imu.set_heading(knights::normalize_angle(360-knights::to_deg(chassis.get_position().heading), false));

	midOdom.reset();
	backOdom.reset();

	// run odometry loop
	if (odomTask == nullptr)
		pros::Task *odomTask = new pros::Task {[=] {
			while (true) {
				chassis.update_position(); // query odometry system for position
				
				// Convoluted method of inputting everything to a string
				std::stringstream stream;
				stream << "Curr Pos: ";
				stream << std::fixed << std::setprecision(2) << chassis.get_position().x << " ";
				stream << std::fixed << std::setprecision(2) << chassis.get_position().y << " ";
				stream << std::fixed << std::setprecision(2) << knights::to_deg(chassis.get_position().heading);
				std::string s = stream.str();

				// Set the display label to the current position
				knights::display::set_pos_label(s);

				// Move the current position dot to the desired position
				knights::display::change_curr_pos_dot(chassis.get_position());

				pros::delay(10);
			}
		}};

	// Run the chosen auton
	if (auton_map.contains(package.get_value())) {
		auton_map[package.get_value()].function(&chassis);
	}

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
	// need to find a way to do this dynamically
	chassis.set_position(knights::Pos(12.0_in, 12.0_in, knights::to_rad(90)));
	imu.set_heading(knights::normalize_angle(360-knights::to_deg(chassis.get_position().heading), false));

	midOdom.reset();
	backOdom.reset();

	// run odometry loop
	if (odomTask == nullptr)
		pros::Task *odomTask = new pros::Task {[=] {
			while (true) {
				chassis.update_position(); // query odometry system for position
				
				// Convoluted method of inputting everything to a string
				std::stringstream stream;
				stream << "Curr Pos: ";
				stream << std::fixed << std::setprecision(2) << chassis.get_position().x << " ";
				stream << std::fixed << std::setprecision(2) << chassis.get_position().y << " ";
				stream << std::fixed << std::setprecision(2) << knights::to_deg(chassis.get_position().heading);
				std::string s = stream.str();
				// printf("curr pos: %lf %lf %lf\n", chassis.get_position().x, chassis.get_position().y, chassis.get_position().heading);

				// Set the display label to the current position
				knights::display::set_pos_label(s);

				// Move the current position dot to the desired position
				knights::display::change_curr_pos_dot(chassis.get_position());

				pros::delay(10);
			}
		}};

	float right_velocity = 0; float left_velocity = 0; 

	knights::input::InputMap input;

	// Bind the requied input actions to the input map
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1, intake_in, false); //assign intake forward toggle to controller button L1
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2, intake_out, false); //assign intake reverse toggle to controller button L2
	
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_Y, lady_brown_score, false); //assign lady brown position score to controller button up
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_B, lady_brown_down, false); //assign lady brown down position to controller button down
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_DOWN, lady_brown_load2,false); //assign lady brown position load 1 to controller button left
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_RIGHT,lady_brown_load1,false); //assign lady brown position load 2 to controller button right

	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R2, clamp_toggle, false); //assign clamp toggle to controller button R2
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R1, doinker_toggle, false); //assign doinker toggle to controller button R1


	while (true) {
		// If controller joystick not in deadzone, calculate the velocity
		if (abs(master_controller.get_analog(ANALOG_RIGHT_Y)) > 2)
			right_velocity = velocity_formula(abs(master_controller.get_analog(ANALOG_RIGHT_Y)));
		// Otherwise, stop the right motors
		else
			right_velocity = 0;

		// If controller joystick not in deadzone, calculate the velocity
		if (abs(master_controller.get_analog(ANALOG_LEFT_Y)) > 2)
			left_velocity = velocity_formula(abs(master_controller.get_analog(ANALOG_LEFT_Y)));
		// Otherwise, stop the left motors
		else
			left_velocity = 0;


		// Send the required velocities to the drivetrain
		// Signum function detects if the controller analog value is postive or negative
		// Reversed b/c david uses the back of the robot as the front
		drivetrain.velocity_command(
			left_velocity * -knights::signum((int)master_controller.get_analog(ANALOG_LEFT_Y)),
			right_velocity * -knights::signum((int)master_controller.get_analog(ANALOG_RIGHT_Y))
		);

		// Delay to let other tasks run
		pros::delay(10);

		// Loop through all values in input map
		input.execute_actions(master_controller);
	}
}

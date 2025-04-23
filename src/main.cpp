#include "main.h"
#include "autonomous.h"
#include "globals.h"
#include "knights/api.hpp"
#include "knights/autonomous/profile.hpp"
#include "knights/display.hpp"
#include "knights/util/calculation.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>
#include <math.h>
#include <string>
#include <unordered_map>
#include <fstream>

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

	auton_map["None0"] = knights::Auton(&skills, knights::Pos(-63, -10, 130_deg));

	auton_map["Red1"] = knights::Auton(&red_left_wp_safe, knights::Pos(-53.76, 7.48, 204.27_deg));
	auton_map["Red2"] = knights::Auton(&red_right_wp_safe, knights::Pos(-61.86, -8.94, 124.68_deg)); // red2

	lv_display();

	midOdom.reset();
	backOdom.reset();

	// wait until everything is calibrated
	pros::delay(2000);

	knights::logger::blue("Initialization End");


	// intake.set_reversed(false, 0);
	// intake.set_reversed(true, 1);

	colors.set_led_pwm(color_sorting * 100);

	// #### TEST AREA ####
	// motion profile test
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

	if (odomTask == nullptr) {
		odomTask = new pros::Task {[=] {
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

		odomTask->set_priority(TASK_PRIORITY_DEFAULT + 1);
	}
	
	if (ladyBrownTask == nullptr) {
		ladyBrownTask = new pros::Task {[=] {
			while(true) {
				lady_brown.move(get_lady_brown_command());

				pros::delay(10);
			}
		}};

		ladyBrownTask->set_priority(TASK_PRIORITY_DEFAULT);
	}

	if (intakeJamTask == nullptr) {
		intakeJamTask = new pros::Task {[=] {
			while(true) {
				if (jam_enabled) {
					unjam_intake_check();

					pros::delay(20);
				} else {
					pros::delay(150);
				}
			}
		}};

		intakeJamTask->set_priority(TASK_PRIORITY_DEFAULT - 1);
	}


	if (package.type == "Blue") {
		red_alliance = false;
	} else {
		red_alliance = true;
	}

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
					} 
				else if (auton_color_sorting) {
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

	// Run the chosen auton
	if (auton_map.contains(package.get_value())) {
		auton_map[package.get_value()].function(
			&chassis, 
			auton_map[package.get_value()].flip_x, 
			auton_map[package.get_value()].flip_y
		);
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
	chassis.set_position(knights::Pos(0, 0, knights::to_rad(0)));
	imu.set_heading(knights::normalize_angle(360-knights::to_deg(chassis.get_position().heading), false));

	midOdom.reset();
	backOdom.reset();

	// run odometry loop
	if (odomTask == nullptr) {
		odomTask = new pros::Task {[=] {
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

		odomTask->set_priority(TASK_PRIORITY_DEFAULT - 1);
	}
	
	if (ladyBrownTask == nullptr) {
		ladyBrownTask = new pros::Task {[=] {
			while(true) {
				lady_brown.move(get_lady_brown_command());

				pros::delay(10);
			}
		}};

		ladyBrownTask->set_priority(TASK_PRIORITY_DEFAULT);
	}

	// // TMP
	// if (intakeJamTask == nullptr) {
	// 	intakeJamTask = new pros::Task {[=] {
	// 		while(true) {
	// 			if (jam_enabled) {
	// 				unjam_intake_check();

	// 				pros::delay(20);
	// 			} else {
	// 				pros::delay(150);
	// 			}
	// 		}
	// 	}};

	// 	intakeJamTask->set_priority(TASK_PRIORITY_DEFAULT - 1);
	// }

	float right_velocity = 0; float left_velocity = 0; 

	knights::input::InputMap input;

	// Bind the requied input actions to the input map
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1, intake_in, false); //assign intake forward toggle to controller button L1
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2, intake_out, false); //assign intake reverse toggle to controller button L2
	
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_Y, lady_brown_score, false); //assign lady brown position score to controller button up
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_B, lady_brown_down, false); //assign lady brown down position to controller button down
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_DOWN, lady_brown_load2,false); //assign lady brown position load 1 to controller button left
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_RIGHT,lady_brown_load1,false); //assign lady brown position load 2 to controller button right

	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_X, change_color, false);
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A, toggle_color_sort, false);

	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R2, clamp_toggle, false); //assign clamp toggle to controller button R2
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R1, doinker_toggle, false); //assign doinker toggle to controller button R1
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_UP, lady_brown_tip, false); //assign doinker toggle to controller button R1

	auton_color_sorting = false;

	if (colorSortTask == nullptr) {
		colorSortTask = new pros::Task {[=] {
			while(true) {
				if (color_sorting) {
					if (red_alliance) {
						blue_color_sort();
					}
					else {				
						red_color_sort();
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

	if (intakeJamTask == nullptr) {
		intakeJamTask = new pros::Task {[=] {
			while(true) {
				if (jam_enabled) {
					unjam_intake_check();

					pros::delay(20);
				} else {
					pros::delay(150);
				}
			}
		}};

		intakeJamTask->set_priority(TASK_PRIORITY_DEFAULT - 1);
	}

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
		drivetrain.voltage_command(
			left_velocity * -knights::signum((int)master_controller.get_analog(ANALOG_LEFT_Y)),
			right_velocity * -knights::signum((int)master_controller.get_analog(ANALOG_RIGHT_Y))
		);

		// Delay to let other tasks run
		pros::delay(10);

		// Loop through all values in input map
		input.execute_actions(master_controller);
	}
}

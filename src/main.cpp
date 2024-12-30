#include "main.h"
#include "knights/logger/logger.h"
#include "pros/misc.h"

#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>

pros::Controller master_controller(pros::E_CONTROLLER_MASTER);

// // Competition Robot
// //front of bot is intake side
// //assign ports to right side drive-train
// pros::MotorGroup right_mtrs({1,2,14}, pros::MotorGears::blue); // no reverse
// //assign ports to left side drive-train
// pros::MotorGroup left_mtrs({7,8,16}, pros::MotorGears::blue); // no reverse
// //assign ports to odom pods for position tracking
// pros::Rotation mid_odom(12); // parallel tracking
// pros::Rotation back_odom(19); // perpendicular tracking
// //assign port for imu tracker
// pros::IMU imu(17);
// //dimensions and positions of odom pods for calculations for position tracking
// knights::PositionTracker midOdom(&mid_odom, 2.75, 1, 2.825);
// knights::PositionTracker backOdom(&back_odom, 2.75, 1, 3.1875);
// // #### END

// #### Test Robot
pros::MotorGroup right_mtrs({17,7,3}, pros::MotorGears::blue);
pros::MotorGroup left_mtrs({4,5,6}, pros::MotorGears::blue);
pros::Rotation mid_odom(18);
pros::Rotation back_odom(14);
pros::IMU imu(15);
knights::PositionTracker midOdom(&mid_odom, 2.75, 1, 0);
knights::PositionTracker backOdom(&back_odom, 2.75, 1, 4.0, -1);
// #### END


//assign ports to intake, leftside first, rightside second
pros::MotorGroup intake({9,13}, pros::MotorGears::blue);

//assign port to distance sensor for redirect
pros::Distance redirect(6);


//assign ports for pneumatics
pros::adi::Pneumatics clamp(8, false); //clamp solenoid
pros::adi::Pneumatics doinker(6, false); //doinker solenoid
pros::adi::Pneumatics big_arm_section(7,false); //big arm solenoid
pros::adi::Pneumatics small_arm_section(5,false); //small arm solenoid
pros::adi::Pneumatics wall_stake_mech_clamp(6,false); //ring clamp solenoid

// make sure to take note if IMU is facing z axis up or down, changes how direction is calculated

knights::Drivetrain drivetrain(&right_mtrs, &left_mtrs, 16, 450.0, 3.25, 3/4);
knights::PositionTrackerGroup odomTrackers(&midOdom, &backOdom, &imu);

// knights::PID_Controller pidController(0.4, 0.0001, 0.085, 0, 127);

knights::RobotChassis chassis(
	&drivetrain,
	&odomTrackers
);

// knights::Robot_Controller botController(&chassis, &pidController, false);


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
	//knights::logger::blue("Initialization Begin");

	lv_display();

	// wait until everything is cali-brated
	pros::delay(2000);


	knights::logger::blue("Initialization End");

	// // #### Competition Robot
	// //front of the bot is intake
	// //assign direction to left side drive-train motors 
	// left_mtrs.set_reversed(false, 0);
	// left_mtrs.set_reversed(false, 1);
	// left_mtrs.set_reversed(false, 2);
	// //assign direction to right side drive-train motors
	// right_mtrs.set_reversed(true, 0);
	// right_mtrs.set_reversed(true, 1);
	// right_mtrs.set_reversed(true, 2);
	// // ####

	// #### Test Bot
	left_mtrs.set_reversed(true, 0);
	left_mtrs.set_reversed(true, 1);
	left_mtrs.set_reversed(true, 2);
	// ####

	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


/**
 * Runs the user autonomous code.
 */
void autonomous() {

	// Query display for the selected buttons
	knights::display::AutonSelectionPackage package = knights::display::get_selected_auton();
	
	// Create a map that maps autonomous to selection packages
	std::unordered_map<std::string, std::function<void(knights::RobotChassis*)>> auton_map;

	// // Different autons, None0 is the default auton
	// // auton_map["None0"] = &blue_right;
	// // chassis.set_position(knights::Pos(54.5, 12.5, 4.234));

	// // auton_map["None0"] = &skills;
    // // chassis.set_position(knights::Pos(-59, 0, 0));

	// auton_map["Red1"] = &red_left_wp_new;
	// auton_map["Blue1"] = &blue_right_wp_new;
	// auton_map["Red3"] = &red_rush_right_elim;
	// // auton_map["Blue2"] = &blue_rush_left_elim;

	// auton_map["Blue4"] = &skills;
    // //  chassis.set_position(knights::Pos(38, 48, 4.081));

	auton_map["None0"] = &pp_test;
	chassis.set_position(knights::Pos(36,12,M_PI/2));

	//chassis.set_position(knights::Pos(59, 0, 0));

	// chassis.set_position(knights::Pos(-36, -60, 3*M_PI/2));

	// if (package.type + std::to_string(package.number) == "Red1") {
	// chassis.set_position(knights::Pos(-56.5,15,3.95728));
	// } else if (package.type + std::to_string(package.number) == "Blue1") {
	// chassis.set_position(knights::Pos(-56.5,15,knights::normalize_angle(-3.95728)));
	// } else if (package.type + std::to_string(package.number) == "Blue4") {
	// 	chassis.set_position(knights::Pos(-59, 0, 0));
	// // } else if (package.type + std::to_string(package.number) == "Blue2") {
	// // 	chassis.set_position(knights::Pos(-59, 0, M_PI));
	// } else if (package.type + std::to_string(package.number) == "Red3") {
	// chassis.set_position(knights::Pos(-48.0,-60.0,3.14159265));
	// }

	// need to find a way to do this dynamically
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

	// Run the chosen auton
	auton_map[package.type + std::to_string(package.number)](&chassis);

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
#define velocity_formula(x) 160*(1/(1+std::pow(M_E, -0.1 * x + 5))) + 20 // arbitrarily defined formula to translate joysticks to velocity

#define INTAKE_VELOCITY 300


bool intake_spinning = false;
bool intake_forward = false;

void intake_fwd() {
	if (intake_spinning == true && intake_forward == true) { // If intake is on or in wrong direction
		intake.move(0); // stop intake
		intake_spinning = false;
	} else {
		intake.move(INTAKE_VELOCITY); // Spin intake forward
		intake_spinning = true;
		intake_forward = true;
	}
}

void intake_rev() {
	if (intake_spinning == true && intake_forward == false) { // If intake is spinning or in the wrong direction
		intake.move(0); // stop intake
		intake_spinning = false;
	} else { 
		intake.move(-INTAKE_VELOCITY); // Spin the intake in reverse
		intake_spinning = true;
		intake_forward = false;
	}
}




bool clamp_down = false;

void clamp_toggle() {
	clamp_down = !clamp_down; //toggle whether active or inactive mode
	clamp.set_value(clamp_down); //activate clamp if inactive or deactivate clamp if active
}


bool doinker_activate = false;

void doinker_toggle() {
	doinker_activate = !doinker_activate; //toggle whether active or inactive mode
	doinker.set_value(doinker_activate); //extend doinker if inactive or retract clamp if active
}


bool end_arm_up = false;

void end_arm() {
	end_arm_up = !end_arm_up; //toggle whether active or inactive mode
	small_arm_section.set_value(end_arm_up); //extend doinker if inactive or retract clamp if active
}



bool arm_up = false;

void wall_stake_mech() {
	arm_up = !arm_up; //toggle whether active or inactive mode
	big_arm_section.set_value(arm_up); //move arm up if active or move down if inactive
}

bool redirection_toggle = false;

void redirect_toggle() {
	redirection_toggle = !redirection_toggle;
}



void redirection() {
	if (redirection_toggle == true && (redirect.get_distance() < 45 )){
			intake.set_brake_mode(pros::MotorBrake::coast);
			intake_rev();
			redirection_toggle = false;
			intake.set_brake_mode(pros::MotorBrake::hold);

	}
}

void opcontrol() {
	// need to find a way to do this dynamically
	chassis.set_position(knights::Pos(0, 0, 0));
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
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1, intake_fwd, false); //assign intake forward toggle to controller button L1
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2, intake_rev, false); //assign intake reverse toggle to controller button L2
	
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R2, clamp_toggle, false); //assign clamp toggle to controller button R2
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R1, doinker_toggle, false); //assign doinker toggle to controller button R1

	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A, redirect_toggle, false); //assign the redirection macro to controller button A
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_X, wall_stake_mech, false); //assign arm lift toggle to controller button X
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_B, end_arm, false); //assign the end part of the arm to controller button B
	while (true) {
		// If controller joystick not in deadzone, calculate the velocity
		if (abs(master_controller.get_analog(ANALOG_LEFT_Y)) > 2)
			right_velocity = velocity_formula(abs(master_controller.get_analog(ANALOG_RIGHT_Y)));
		// Otherwise, stop the right motors
		else
			right_velocity = 0;

		// If controller joystick not in deadzone, calculate the velocity
		if (abs(master_controller.get_analog(ANALOG_RIGHT_Y)) > 2)
			left_velocity = velocity_formula(abs(master_controller.get_analog(ANALOG_LEFT_Y)));
		// Otherwise, stop the left motors
		else
			left_velocity = 0;


		// Send the required velocities to the drivetrain
		// Signum function detects if the controller analog value is postive or negative
		drivetrain.velocity_command(
			right_velocity * knights::signum((int)master_controller.get_analog(ANALOG_RIGHT_Y)), 
			left_velocity * knights::signum((int)master_controller.get_analog(ANALOG_LEFT_Y))
		);

		// Delay to let other tasks run
		pros::delay(10);
		
		redirection();

		// Loop through all values in input map
		input.execute_actions(master_controller);
	}
}

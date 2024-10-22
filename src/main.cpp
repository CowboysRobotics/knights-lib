#include "main.h"
#include "knights/autonomous/path.h"
#include "knights/logger/colors.h"
#include "knights/logger/logger.h"
#include "squiggles/squiggles.hpp"

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

pros::Controller master_controller(pros::E_CONTROLLER_MASTER);

// Competition Robot
pros::MotorGroup right_mtrs({3,9,5}, pros::MotorGears::blue); // 8 needs to be rev
pros::MotorGroup left_mtrs({8,1,7}, pros::MotorGears::blue); // 13,17 need rev
pros::Rotation mid_odom(13);
pros::Rotation back_odom(14);
pros::Distance redirect(19);
pros::IMU imu(12);
knights::PositionTracker midOdom(&mid_odom, 2.75, 1, 2.677);
knights::PositionTracker backOdom(&back_odom, 2.75, 1, 0);

// // Test Robot
// pros::MotorGroup right_mtrs({1,7,3}, pros::MotorGears::blue);
// pros::MotorGroup left_mtrs({4,5,6}, pros::MotorGears::blue);
// pros::Rotation mid_odom(18);
// pros::Rotation back_odom(14);
// pros::IMU imu(15);
// knights::PositionTracker midOdom(&mid_odom, 2.75, 1, 0, -1);
// knights::PositionTracker backOdom(&back_odom, 2.75, 1, 4.0, -1);

pros::Motor intake(6, pros::MotorGears::blue);
pros::adi::Pneumatics clamp(4, true);
pros::adi::Pneumatics doinker(2, false);
// make sure to take note if IMU is facing z axis up or down, changes how direction is calculated

knights::Drivetrain drivetrain(&right_mtrs, &left_mtrs, 16, 450.0, 2.75, 0.75);
knights::PositionTrackerGroup odomTrackers(&midOdom, &backOdom, &imu);
pros::MotorGroup snacky_cakes({20, 4}, pros::v5::MotorGears::green);

knights::PIDController wall_stake_mech_PID(10, 0, 0, -127, 127);

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
	//knights::logger::blue("Initialization Begin");

	lv_display();

	// wait until everything is cali-brated
	pros::delay(2000);

    chassis.set_position(knights::Pos(-36, -60, M_PI/2));
	// chassis.set_position(knights::Pos(-36, 60, 0));
	// imu.set_heading(knights::normalize_angle(knights::to_deg(chassis.get_position().heading)-180, false));
	imu.set_heading(knights::normalize_angle(knights::to_deg(chassis.get_position().heading), false));

	midOdom.reset();
	backOdom.reset();

	//knights::logger::blue("Initialization End");

	// Competition Robot
	left_mtrs.set_reversed(false, 0);
	left_mtrs.set_reversed(true, 1);
	left_mtrs.set_reversed(true, 2);

	right_mtrs.set_reversed(true, 0);
	right_mtrs.set_reversed(false, 1);
	right_mtrs.set_reversed(false, 2);

	snacky_cakes.set_reversed(true, 1);
	// // Test Bot
	// left_mtrs.set_reversed(true, 0);
	// left_mtrs.set_reversed(true, 1);
	// left_mtrs.set_reversed(true, 2);

	intake.set_reversed(true, 0);
	snacky_cakes.tare_position();


	// // Squiggles test
	// const double MAX_VEL = drivetrain.max_velocity();     // in meters per second
	// const double MAX_ACCEL = drivetrain.max_acceleration(9, 6);   // in meters per second^2
	// const double MAX_JERK = MAX_ACCEL*2;    // in meters per second^3
	// const double ROBOT_WIDTH = knights::to_meters(15.0); // in meters
	// auto constraints = squiggles::Constraints(MAX_VEL, MAX_ACCEL, MAX_JERK);
	// auto generator = squiggles::SplineGenerator(
	// constraints,
	// std::make_shared<squiggles::TankModel>(ROBOT_WIDTH, constraints));

	// std::vector<squiggles::ProfilePoint> path = generator.generate({
	// 	squiggles::Pose(knights::to_meters(chassis.get_position().x), knights::to_meters(chassis.get_position().y), chassis.get_position().heading), 
	// 	squiggles::Pose(knights::to_meters(-12), knights::to_meters(-36), 0)}
	// );

	// std::cout << path.size() << "\n";

	// int i = 0;

	// for (squiggles::ProfilePoint pt : path) {
	// 	std::stringstream stream;
	// 	stream << "Point Pos: ";
	// 	stream << std::fixed << std::setprecision(2) << knights::to_inches(pt.vector.pose.x) << " ";
	// 	stream << std::fixed << std::setprecision(2) << knights::to_inches(pt.vector.pose.y) << " ";
	// 	stream << std::fixed << std::setprecision(2) << knights::to_deg(pt.vector.pose.yaw);
	// 	stream << " V,J,A: ";
	// 	stream << std::fixed << std::setprecision(2) << pt.vector.vel << " ";
	// 	stream << std::fixed << std::setprecision(2) << pt.vector.jerk << " ";
	// 	stream << std::fixed << std::setprecision(2) << pt.vector.accel << " ";
	// 	//knights::logger::cyan(stream.str());

	// 	if (i % 5 == 0) {
	// 		knights::display::MapDot dot(5, 5, lv_palette_lighten(LV_PALETTE_GREEN,5));
	// 		dot.set_field_pos(knights::Pos(
	// 			knights::to_inches(pt.vector.pose.x), 
	// 			knights::to_inches(pt.vector.pose.y),
	// 			0
	// 		));
	// 	}
	// }
	// // ---- end squiggles test ----

	//knights::logger::blue(//knights::logger::string_format("start pos: %lf %lf %lf", chassis.get_position().x, chassis.get_position().y, chassis.get_position().heading));

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

	// Different autons, None0 is the default auton
	auton_map["None0"] = &pp_test;
	auton_map["Blue1"] = &programming_skills;
	auton_map["Red1"] = &right_wp_auton;
    auton_map["Red2"] = &left_wp_auton;

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
#define velocity_formula(x) 81*(1/(1+std::pow(M_E, -0.1 * x + 5))) + 20 // arbitrarily defined formula to translate joysticks to velocity

#define INTAKE_VELOCITY 300


bool intake_spinning = false;

void intake_fwd() {
	if (intake_spinning == true && intake.get_direction() == -1) { // If intake is on or in wrong direction
		intake.move(0); // stop intake
		intake_spinning = false;
	} else {
		intake.move(INTAKE_VELOCITY); // Spin intake forward
		intake_spinning = true;
	}
}

void intake_rev() {
	if (intake_spinning == true && intake.get_direction() == 1) { // If intake is spinning or in the wrong direction
		intake.move(0); // stop intake
		intake_spinning = false;
	} else { 
		intake.move(-INTAKE_VELOCITY); // Spin the intake in reverse
		intake_spinning = true;
	}
}
bool redirection_true = false;

void redirection_toggle() {
	intake.set_brake_mode(pros::MotorBrake::hold);
	if (redirection_true == true){
		redirection_true = false;
	}
	else {
		redirection_true = true;
	}
}



void redirection() {
	pros::delay(19);
	intake.brake();
	pros::delay(100);
	intake.set_brake_mode(pros::MotorBrake::coast);
	intake_rev();
}




#define WALL_STAKE_MECH_MAX_ANGLE 120
#define WALL_STAKE_MECH_GEAR_RATIO 24/72

void use_wall_stake_mech() {
	float total_error = 0; float prev_error; float error = 1e5;
	int timeout = 3000;

	snacky_cakes.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	while (error > 0) {
		error = WALL_STAKE_MECH_MAX_ANGLE - knights::normalize_angle(snacky_cakes.get_position() * WALL_STAKE_MECH_GEAR_RATIO, false);

		// timeout -= 10;
		// if (timeout < 0)
		// 	break;

		total_error += error;

		float speed = wall_stake_mech_PID.update(error, total_error, prev_error);

		prev_error = error;

		snacky_cakes.move(speed);

		printf("error: %lf, speed, %lf, at pos: %lf, raw: %lf\n", error, speed, snacky_cakes.get_position() * WALL_STAKE_MECH_GEAR_RATIO, snacky_cakes.get_position());
	
		pros::delay(10);
	}

	snacky_cakes.move(0);

	while (error < 6) {
		error = knights::normalize_angle(snacky_cakes.get_position() * WALL_STAKE_MECH_GEAR_RATIO, false) - WALL_STAKE_MECH_MAX_ANGLE;

		// timeout -= 10;
		// if (timeout < 0)
		// 	break;

		total_error += error;

		float speed = wall_stake_mech_PID.update(error, total_error, prev_error);

		prev_error = error;

		snacky_cakes.move(speed);

		printf("error: %lf, speed, %lf, at pos: %lf, raw: %lf\n", error, speed, snacky_cakes.get_position() * WALL_STAKE_MECH_GEAR_RATIO, snacky_cakes.get_position());
	
		pros::delay(10);
	}
}

bool hungry = false;

void snack_eat() {
	if (hungry == true && snacky_cakes.get_direction() == 1) { // If snack is on or in wrong direction
		snacky_cakes.move(0); // stop eating
		hungry = false;
	} else {
		snacky_cakes.move(INTAKE_VELOCITY); // Spin snack forward
		pros::delay(750);
		snacky_cakes.set_brake_mode(pros::MotorBrake::hold);
		snacky_cakes.brake();
		hungry = true;
	}
}

void snack_swallow() {
	if (hungry == true && snacky_cakes.get_direction() == -1) { // If snack is spinning or in the wrong direction
		snacky_cakes.move(0); // stop eating
		hungry = false;
	} else { 
		snacky_cakes.move(-INTAKE_VELOCITY); // Spin the snack in reverse
		pros::delay(250);
		snacky_cakes.set_brake_mode(pros::MotorBrake::coast);
		snacky_cakes.brake();
		hungry = true;
	}
}

bool clamp_down = false;

void clamp_out() {
	clamp_down = !clamp_down;
	clamp.set_value(clamp_down);
}

bool doinked = false;

void doink() {
    doinked = !doinked;
	doinker.set_value(doinked);

}

void opcontrol() {
	float right_velocity = 0; float left_velocity = 0;

	knights::input::InputMap input;

	// Bind the requied input actions to the input map
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1, intake_fwd, false);
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2, intake_rev, false);
	
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R2, clamp_out, false);
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R1, doink, false);
	
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_X, snack_eat, false);
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_B, snack_swallow, false);
	input.bind_action(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A, redirection_toggle, false);

	while (true) {
		// If controller joystick not in deadzone, calculate the velocity
		if (abs(master_controller.get_analog(ANALOG_LEFT_Y)) > 2)
			right_velocity = velocity_formula(abs(master_controller.get_analog(ANALOG_LEFT_Y)));
		// Otherwise, stop the right motors
		else
			right_velocity = 0;

		// If controller joystick not in deadzone, calculate the velocity
		if (abs(master_controller.get_analog(ANALOG_RIGHT_Y)) > 2)
			left_velocity = velocity_formula(abs(master_controller.get_analog(ANALOG_RIGHT_Y)));
		// Otherwise, stop the left motors
		else
			left_velocity = 0;
		
		if	(redirection_true == true and redirect.get() < 10)
			redirection();


		// Send the required velocities to the drivetrain
		// Signum function detects if the controller analog value is postive or negative
		drivetrain.velocity_command(
			right_velocity * -knights::signum((int)master_controller.get_analog(ANALOG_LEFT_Y)), 
			left_velocity * -knights::signum((int)master_controller.get_analog(ANALOG_RIGHT_Y))
		);

		// Delay to let other tasks run
		pros::delay(10);

		// Loop through all values in input map
		input.execute_actions(master_controller);
	}
}

#include "main.h"
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <unordered_map>

pros::Controller master_controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mtrs({8,1,14}, pros::MotorGears::blue); // 8 needs to be rev
pros::MotorGroup right_mtrs({11,13,17}, pros::MotorGears::blue); // 13,17 need rev
pros::MotorGroup intake({16,19}, pros::MotorGears::green);
pros::Rotation mid_odom(7);
pros::Rotation back_odom(20);

pros::adi::Pneumatics clamp(8, true);
pros::adi::Pneumatics doinker(7, false);
// make sure to take note if IMU is facing z axis up or down, changes how direction is calculated
pros::IMU imu(9);

knights::Drivetrain drivetrain(&right_mtrs, &left_mtrs, 16, 450.0, 2.75, 0.75);
knights::PositionTracker midOdom(&mid_odom, 2.75, 1, 2.677);
knights::PositionTracker backOdom(&back_odom, 2.75, 1, 0);
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
	printf("init\n");

	lv_display();

	// wait until everything is calibrated
	pros::delay(2000);

	// knights::Pos starting_position(60,-60,M_PI/2); // used to be -36,60

	// chassis.set_position(starting_position);
	// chassis.set_prev_position(starting_position);
    chassis.set_position(knights::Pos(-60, 0, 0.001));
	imu.set_heading(knights::normalize_angle(knights::to_deg(chassis.get_position().heading)-180, false));

	midOdom.reset();
	backOdom.reset();

	left_mtrs.set_reversed(true, 0);
	left_mtrs.set_reversed(false, 1);
	left_mtrs.set_reversed(false, 2);

	right_mtrs.set_reversed(false, 0);
	right_mtrs.set_reversed(true, 1);
	right_mtrs.set_reversed(true, 2);

	intake.set_reversed(true, 1);

	// knights::Route to_center = knights::generate_path_to_pos(starting_position, knights::Pos(0,0,M_PI/2), 1.0, 2.0, 75.0, 14.0);

	// //for (knights::Pos position : to_center.positions) {
	// for (int i = 0; i < (int)to_center.positions.size(); i+=((int)to_center.positions.size()/40)) {
	// 	knights::Pos position = to_center.positions[i];
	// 	knights::display::MapDot target_position_dot(5,5,lv_palette_lighten(LV_PALETTE_GREY, 0));
	// 	target_position_dot.set_field_pos(position);
	// 	// printf("pos: %lf %lf %lf\n", position.x, position.y, position.heading);
	// }

	if (odomTask == nullptr)
		pros::Task *odomTask = new pros::Task {[=] {
			while (true) {
				chassis.update_position();
				
				std::stringstream stream;
				stream << "Curr Pos: ";
				stream << std::fixed << std::setprecision(2) << chassis.get_position().x << " ";
				stream << std::fixed << std::setprecision(2) << chassis.get_position().y << " ";
				stream << std::fixed << std::setprecision(2) << knights::to_deg(chassis.get_position().heading);
				std::string s = stream.str();
				knights::display::set_pos_label(s);

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
	auton_map["None0"] = &right_wp_auton;
	auton_map["Blue1"] = &programming_skills;
	auton_map["Red1"] = &unsafe_wp_auton;

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
#define velocity_formula(x) 81*(1/(1+std::pow(M_E, -0.1 * x + 5))) + 20

#define INTAKE_VELOCITY 300

bool intake_spinning = false;

void intake_fwd() {
	if (intake_spinning == true && intake.get_direction() == 1) {
		intake.move(0);
		intake_spinning = false;
	} else {
		intake.move(INTAKE_VELOCITY);
		intake_spinning = true;
	}
}

void intake_rev() {
	if (intake_spinning == true && intake.get_direction() == -1) {
		intake.move(0);
		intake_spinning = false;
	} else {
		intake.move(-INTAKE_VELOCITY);
		intake_spinning = true;
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
		drivetrain.velocity_command(-right_velocity * knights::signum((int)master_controller.get_analog(ANALOG_RIGHT_Y)), 
			-left_velocity * knights::signum((int)master_controller.get_analog(ANALOG_LEFT_Y)));

		// Delay to let other tasks run
		pros::delay(10);

		// Loop through all values in input map
		input.execute_actions(master_controller);
	}
}

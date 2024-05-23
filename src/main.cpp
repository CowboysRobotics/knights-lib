#include "main.h"

pros::Controller master_controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup right_mtrs({1,2,3});
pros::MotorGroup left_mtrs({8,9,10});
// pros::Rotation right_odom(3);
// pros::Rotation left_odom(4);
pros::IMU imu(18);

knights::Drivetrain drivetrain(&right_mtrs, &left_mtrs, 10.0, 600.0, 3.25, 0.75);
// knights::Position_Tracker rightOdom(&right_odom, 1.0, 1, 2.5);
// knights::Position_Tracker leftOdom(&left_odom, 1.0, 1, 2.5);
// knights::Position_Tracker_Group odomTrackers(&rightOdom, &leftOdom, &imu);

// knights::PID_Controller pidController(0.4, 0.0001, 0.085, 0, 127);

// pros::Motor right_front(4);
// pros::Motor right_back(3);
// pros::Motor left_front(2);
// pros::Motor left_back(1);

// knights::Holonomic holonomic(&right_front, &left_front, &right_back, &left_back, 10.0, 600.0, 3.25);

// knights::Robot_Chassis chassis(
// 	&drivetrain,
// 	&odomTrackers
// );

// knights::Robot_Controller botController(&chassis, &pidController, false);


pros::Task *odomTask = nullptr;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// if (odomTask == nullptr)
	// 	pros::Task *odomTask = new pros::Task {[=] {
	// 		while (true) {
	// 			chassis.update_position();
	// 			pros::delay(10);
	// 		}
	// 	}};
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
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
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

void opcontrol() {
	float right_velocity, left_velocity;

	while (true) {
		right_velocity = velocity_formula(master_controller.get_analog(ANALOG_RIGHT_Y));
		left_velocity = velocity_formula(master_controller.get_analog(ANALOG_LEFT_Y));

		drivetrain.velocity_command(right_velocity, left_velocity);

		pros::delay(10);
	}
}

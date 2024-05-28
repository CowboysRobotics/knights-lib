#include "main.h"

pros::Controller master_controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup right_mtrs({1,7,3});
pros::MotorGroup left_mtrs({4,5,6});
pros::Rotation mid_odom(18);
pros::Rotation back_odom(14);

// make sure to take note if IMU is facing z axis up or down, changes how direction is calculated
pros::IMU imu(15);

knights::Drivetrain drivetrain(&right_mtrs, &left_mtrs, 10.0, 600.0, 3.25, 0.75);
knights::Position_Tracker midOdom(&mid_odom, 1.0, 2.75, 0);
knights::Position_Tracker backOdom(&back_odom, 1.0, 2.75, 2);
knights::Position_Tracker_Group odomTrackers(&midOdom, &backOdom, &imu);

// knights::PID_Controller pidController(0.4, 0.0001, 0.085, 0, 127);

knights::Robot_Chassis chassis(
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

	printf("init new\n");

	pros::lcd::initialize();

	// wait until IMU is fully calibrated
	pros::delay(2000);

	knights::Pos starting_position(0,0,M_PI/2);

	chassis.set_position(starting_position);
	chassis.set_prev_position(starting_position);

	imu.set_heading(knights::normalize_angle(knights::to_deg(chassis.get_position().heading)-180, false));
	midOdom.reset();
	backOdom.reset();

	left_mtrs.set_reversed(true, 0);
	left_mtrs.set_reversed(true, 1);
	left_mtrs.set_reversed(true, 2);

	// printf("0 : %lf\n", knights::distance_btwn(knights::Point(0, 12), knights::Point(0, 12)));

	// printf("24 : %lf\n", knights::distance_btwn(knights::Point(0, 12), knights::Point(0, -12)));

	// printf("12 : %lf\n", knights::distance_btwn(knights::Point(0, 12), knights::Point(0, 0)));

	// printf("0x : %lf\n", knights::distance_btwn(knights::Point(12, 12), knights::Point(12, 12)));

	// printf("24x : %lf\n", knights::distance_btwn(knights::Point(-12, 12), knights::Point(12, 12)));

	// printf("12x : %lf\n", knights::distance_btwn(knights::Point(12, 12), knights::Point(0, 12)));

	printf("right odom: %lf\n", midOdom.get_distance_travelled());
	printf("back odom: %lf\n", backOdom.get_distance_travelled());

	if (odomTask == nullptr)
		pros::Task *odomTask = new pros::Task {[=] {
			while (true) {
				chassis.update_position();
				pros::lcd::print(1, "pos: %lf %lf %lf\n", chassis.get_position().x, chassis.get_position().y, knights::to_deg(chassis.get_position().heading));
				pros::delay(20);
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
	knights::PID_Controller lateralPID(4, 0.0, 0.0, 0.0, 127.0);
	knights::Robot_Controller lateralController(&chassis, &lateralPID, false);

	knights::PID_Controller turnPID(40, 0.0, 0.0, 0.0, 127.0);
	knights::Robot_Controller turnController(&chassis, &turnPID, false);

	knights::Pos startPos(chassis.get_position());

	// botController.lateral_move(24.0, 1, 3000);
	turnController.turn_for(90, 2.0, 2000);

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
	float right_velocity = 0; float left_velocity = 0;

	long reps = 0;

	while (true) {
		if (abs(master_controller.get_analog(ANALOG_RIGHT_Y)) > 2)
			right_velocity = velocity_formula(abs(master_controller.get_analog(ANALOG_RIGHT_Y)));
		else
			right_velocity = 0;

		if (abs(master_controller.get_analog(ANALOG_LEFT_Y)) > 2)
			left_velocity = velocity_formula(abs(master_controller.get_analog(ANALOG_LEFT_Y)));
		else
			left_velocity = 0;

		drivetrain.velocity_command(right_velocity * knights::signum((int)master_controller.get_analog(ANALOG_RIGHT_Y)), 
			left_velocity * knights::signum((int)master_controller.get_analog(ANALOG_LEFT_Y)));

		pros::delay(10);
		reps++;
	}
}

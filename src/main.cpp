#include "main.h"

pros::Controller master_controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup right_mtrs({1,7,3});
pros::MotorGroup left_mtrs({4,5,6});
pros::Rotation mid_odom(18);
pros::Rotation back_odom(14);

// make sure to take note if IMU is facing z axis up or down, changes how direction is calculated
pros::IMU imu(15);

knights::Drivetrain drivetrain(&right_mtrs, &left_mtrs, 18.0, 600.0, 3.25, 0.75);
knights::PositionTracker midOdom(&mid_odom, 1.0, 2.75, 0);
knights::PositionTracker backOdom(&back_odom, 1.0, 2.75, 2);
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

	// pros::lcd::initialize();
	lv_display();

	// wait until IMU is fully calibrated
	pros::delay(2000);

	knights::Pos starting_position(-36,-60,M_PI/2); // used to be 0,0

	chassis.set_position(starting_position);
	chassis.set_prev_position(starting_position);

	imu.set_heading(knights::normalize_angle(knights::to_deg(chassis.get_position().heading)-180, false));
	midOdom.reset();
	backOdom.reset();

	left_mtrs.set_reversed(true, 0);
	left_mtrs.set_reversed(true, 1);
	left_mtrs.set_reversed(true, 2);

	// knights::display::MapDot target_position_dot(5,5,lv_palette_lighten(LV_PALETTE_GREEN, 3));
	// target_position_dot.set_field_pos(knights::Pos(0,0,0));

	knights::Route to_center = knights::generate_path_to_pos(starting_position, knights::Pos(0,0,M_PI/4),8);

	for (knights::Pos position : to_center.positions) {
		knights::display::MapDot target_position_dot(5,5,lv_palette_lighten(LV_PALETTE_GREEN, 3));
		target_position_dot.set_field_pos(position);
		printf("pos: %lf %lf %lf\n", position.x, position.y, position.heading);
	}

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
	knights::PIDController lateralPID(4, 0.0, 0.0, 0.0, 127.0);
	knights::RobotController lateralController(&chassis, &lateralPID, false);

	knights::PIDController turnPID(40, 0.0, 0.0, 0.0, 127.0);
	knights::RobotController turnController(&chassis, &turnPID, false);

	knights::Pos startPos(chassis.get_position());

	knights::Route test = knights::init_route_from_sd("tst.txt");

	printf("Route of size %i loaded to memory\n", test.positions.size());

	for (knights::Pos Position : test.positions) {
		printf("Read: %lf %lf %lf\n", Position.x, Position.y, Position.heading);
	}

	lateralController.follow_route_pursuit(test, 12.0, 80.0, true, 8.0, 5000);

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

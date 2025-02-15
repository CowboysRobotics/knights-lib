#include "knights/autonomous/pid.hpp"
#include "knights/logger/logger.hpp"
#include "knights/util/calculation.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/distance.hpp"
#include "knights/robot/position_tracker.hpp"
#include "knights/robot/drivetrain.hpp"
#include "knights/robot/chassis.hpp"
#include "pros/rtos.hpp"

#include <cmath>
#include <cstdio>

pros::Controller master_controller(pros::E_CONTROLLER_MASTER);
// Competition Robot
//front of bot is intake side
//assign ports to right side drive-train
pros::MotorGroup left_mtrs({2,3,-4}, pros::MotorGears::blue); // no reverse
//assign ports to left side drive-train
pros::MotorGroup right_mtrs({-14,-12,13}, pros::MotorGears::blue); // no reverse
//assign ports to odom pods for position tracking
pros::Rotation mid_odom(8); // parallel tracking
pros::Rotation back_odom(	11); // perpendicular tracking
//assign port for imu tracker
pros::IMU imu(6);
//dimensions and positions of odom pods for calculations for position tracking
knights::PositionTracker midOdom(&mid_odom, 2, 1, 0.9375, -1);
knights::PositionTracker backOdom(&back_odom, 2, 1, 1.75, -1); // 1.875
// #### END

// // #### Test Robot
// pros::MotorGroup right_mtrs({17,7,3}, pros::MotorGears::blue);
// pros::MotorGroup left_mtrs({-4,-5,-6}, pros::MotorGears::blue);
// pros::Rotation mid_odom(18);
// pros::Rotation back_odom(14);
// pros::IMU imu(15);
// knights::PositionTracker midOdom(&mid_odom, 2.75, 1, 0);
// knights::PositionTracker backOdom(&back_odom, 2.75, 1, 4.0, -1);
// // #### END

//assign ports to Lady Brown arm mech
pros::Motor lady_brown(21, pros::MotorGears::green);
pros::Rotation lady_brown_rotation(7);

//assign ports to intake, leftside first, rightside second
pros::MotorGroup intake({20, 5}, pros::MotorGears::blue);

pros::Motor intake_bottom(5, pros::v5::MotorGears::blue);

pros::Motor intake_top(20,pros::v5::MotorGears::blue);

//assign port to distance sensor for redirect
pros::Optical colors(15);

//assign ports for pneumatics
pros::adi::Pneumatics clamp(1, false); //clamp solenoid
pros::adi::Pneumatics doinker(3, false); //doinker solenoid
pros::adi::Pneumatics doinker2(2, false); //rush mech solenoid

knights::Drivetrain drivetrain(&right_mtrs, &left_mtrs, 16, 450.0, 3.25, 0.75);
knights::PositionTrackerGroup odomTrackers(&midOdom, &backOdom, &imu);

knights::RobotChassis chassis(
	&drivetrain,
	&odomTrackers
);


pros::Task *odomTask = nullptr;
pros::Task *ladyBrownTask = nullptr;
pros::Task *colorSortTask = nullptr;
pros::Task *intakeJamTask = nullptr;

#define velocity_formula(x) 160*(1/(1+std::pow(M_E, -0.1 * x + 5))) + 20 // arbitrarily defined formula to translate joysticks to velocity

#define INTAKE_VELOCITY 200

bool intake_spinning = false;
bool intake_forward = false;

float intake_voltage = 0;

void intake_in() {
	if (intake_spinning == true && intake_forward == true) { // If intake is on or in wrong direction
		intake.move(0); // stop intake
		intake_spinning = false;
	} else {
		intake.move(INTAKE_VELOCITY); // Spin intake forward
		intake_voltage = INTAKE_VELOCITY;
		intake_spinning = true;
		intake_forward = true;
	}
}

void intake_out() {
	if (intake_spinning == true && intake_forward == false) { // If intake is spinning or in the wrong direction
		intake.move(0); // stop intake
		intake_spinning = false;
	} else { 
		intake.move(-INTAKE_VELOCITY); // Spin the intake in reverse
		intake_voltage = -INTAKE_VELOCITY;
		intake_spinning = true;
		intake_forward = false;
	}
}

int jam_times = 0;
bool jam_enabled = false;

void unjam_intake_check() {
	if (intake_spinning && std::abs(intake.get_actual_velocity()) < 1) {
		jam_times++;

		if (jam_times > 20) {

			float original_intake_voltage = intake_voltage;

			intake.move(-1 * intake_voltage);
			pros::delay(200);
			intake.move(intake_voltage);

			jam_times = 0;
		}
	}
}

bool color_sorting = false;
bool red_alliance = true;

void toggle_color_sort(){
	color_sorting = !color_sorting;
	colors.set_led_pwm(color_sorting * 100);
}

void change_color(){
	red_alliance = !red_alliance;
}

void red_color_sort() {
	if (colors.get_hue() < 40 && red_alliance == false && color_sorting == true && colors.get_proximity() > 100){
		pros::delay(20);
		printf("red_color_sorting \n");
		intake_top.move(0);
		pros::delay(100);
		intake_top.move(INTAKE_VELOCITY);
	}
}

void blue_color_sort(){
	if (colors.get_hue() > 140 && red_alliance == true && color_sorting == true && colors.get_proximity() > 100){
		pros::delay(20);
		printf("blue_color_sorting \n");
		intake_top.move(0);
		pros::delay(100);
		intake_top.move(INTAKE_VELOCITY);
	}
}

#define LADY_BROWN_VELOCITY 127.0
#define LADY_BROWN_kP 1.75
#define LADY_BROWN_kI 0.000
#define LADY_BROWN_kD 0.5

knights::PIDController lady_brown_PID(LADY_BROWN_kP, LADY_BROWN_kI, LADY_BROWN_kD, 10.0, 127.0);

#define LADY_BROWN_DOWN 0
#define LADY_BROWN_LOAD1 339
#define LADY_BROWN_LOAD2 160
#define LADY_BROWN_SCORE 206 // 213
#define LADY_BROWN_ALLIANCE 165
#define LADY_BROWN_END_TOLERANCE 1.0

bool lady_brown_spinning = false;
bool lady_brown_forward = false;

void lady_brown_fwd() {
	if (lady_brown_spinning == true && lady_brown_forward == true) { // If intake is on or in wrong direction
		lady_brown.move(0); // stop intake
		lady_brown_spinning = false;
	} else {
		lady_brown.move(-LADY_BROWN_VELOCITY); // Spin intake forward
		lady_brown_spinning = true;
		lady_brown_forward = true;
	}
}

void lady_brown_rev() {
	if (lady_brown_spinning == true && lady_brown_forward == false) { // If intake is spinning or in the wrong direction
		lady_brown.move(0); // stop intake
		lady_brown_spinning = false;
	} else { 
		lady_brown.move(INTAKE_VELOCITY); // Spin the intake in reverse
		lady_brown_spinning = true;
		lady_brown_forward = false;
	}
}

float lady_brown_target = LADY_BROWN_DOWN;

float get_lady_brown_command() {
	float error = knights::angular_error(lady_brown_rotation.get_angle()/100.0, lady_brown_target, 0, false);
	// float error = lady_brown_target - (lady_brown_rotation.get_angle()/100.0);
	float speed = lady_brown_PID.update(error, false);

	if (lady_brown_target < 180 && lady_brown_target > 0 && (lady_brown_rotation.get_angle()/100.0 > lady_brown_target || lady_brown_rotation.get_angle()/100.0 < 20)) {
		speed = -1 * fabs(speed);
	}
	else if (lady_brown_target == LADY_BROWN_DOWN) {
		speed = fabs(speed);
	}

	return speed;
}


void lady_brown_down() {
    // lady_brown_to_angle(LADY_BROWN_DOWN, 1500, true, -1);
	lady_brown_target = LADY_BROWN_DOWN;
}

void lady_brown_load1() {
	color_sorting = false;
    // lady_brown_to_angle(LADY_BROWN_LOAD1, 1500, true);

	lady_brown_target = LADY_BROWN_LOAD1;
}

void lady_brown_load2() {
	// lady_brown_to_angle(LADY_BROWN_LOAD2, 1500, true, 1);

	lady_brown_target = LADY_BROWN_LOAD2;
}

void lady_brown_score() {
	// color_sorting = true;
	intake_spinning = false;
	intake.move(0);
    // lady_brown_to_angle(LADY_BROWN_SCORE, 1500, true);

	lady_brown_target = LADY_BROWN_SCORE;
}

void lady_brown_alliance() {
	intake_spinning = false;
	intake.move(0);
    // lady_brown_to_angle(LADY_BROWN_ALLIANCE, 750, true, 5.0);

	lady_brown_target = LADY_BROWN_ALLIANCE;
}

void score_wall_stake() {
	lady_brown_score();
	pros::delay(600);
	lady_brown_down();
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

bool doinker_activate2 = false;

void doinker_toggle2() {
	doinker_activate2 = !doinker_activate2; //toggle whether active or inactive mode
	doinker2.set_value(doinker_activate2); //extend doinker if inactive or retract clamp if active
}


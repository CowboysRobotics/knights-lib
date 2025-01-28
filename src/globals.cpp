#include "knights/autonomous/pid.hpp"
#include "knights/util/calculation.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/distance.hpp"
#include "knights/robot/position_tracker.hpp"
#include "knights/robot/drivetrain.hpp"
#include "knights/robot/chassis.hpp"

#include <cmath>
#include <cstdio>

pros::Controller master_controller(pros::E_CONTROLLER_MASTER);

// // Competition Robot
// //front of bot is intake side
// //assign ports to right side drive-train
// pros::MotorGroup left_mtrs({2,3,4}, pros::MotorGears::blue); // no reverse
// //assign ports to left side drive-train
// pros::MotorGroup right_mtrs({14,16,13}, pros::MotorGears::blue); // no reverse
// //assign ports to odom pods for position tracking
// pros::Rotation mid_odom(8); // parallel tracking
// pros::Rotation back_odom(	11); // perpendicular tracking
// //assign port for imu tracker
// pros::IMU imu(6);
// //dimensions and positions of odom pods for calculations for position tracking
// knights::PositionTracker midOdom(&mid_odom, 2, 1, 1.25, -1);
// knights::PositionTracker backOdom(&back_odom, 2, 1, 1.875, -1);
// // #### END

// #### Test Robot
pros::MotorGroup right_mtrs({17,7,3}, pros::MotorGears::blue);
pros::MotorGroup left_mtrs({-4,-5,-6}, pros::MotorGears::blue);
pros::Rotation mid_odom(18);
pros::Rotation back_odom(14);
pros::IMU imu(15);
knights::PositionTracker midOdom(&mid_odom, 2.75, 1, 0);
knights::PositionTracker backOdom(&back_odom, 2.75, 1, 4.0, -1);
// #### END

//assign ports to Lady Brown arm mech
pros::Motor lady_brown(21, pros::MotorGears::green);
pros::Rotation lady_brown_rotation(7);

//assign ports to intake, leftside first, rightside second
pros::Motor intake(20, pros::MotorGears::blue);

//assign port to distance sensor for redirect
pros::Distance redirect(6);

//assign ports for pneumatics
pros::adi::Pneumatics clamp(1, false); //clamp solenoid
pros::adi::Pneumatics doinker(3, false); //doinker solenoid
pros::adi::Pneumatics rush_mech(2, false); //rush mech solenoid

knights::Drivetrain drivetrain(&right_mtrs, &left_mtrs, 16, 450.0, 3.25, 3/4);
knights::PositionTrackerGroup odomTrackers(&midOdom, &backOdom, &imu);

knights::RobotChassis chassis(
	&drivetrain,
	&odomTrackers
);

#define velocity_formula(x) 160*(1/(1+std::pow(M_E, -0.1 * x + 5))) + 20 // arbitrarily defined formula to translate joysticks to velocity

#define INTAKE_VELOCITY 300

bool intake_spinning = false;
bool intake_forward = false;

void intake_in() {
	if (intake_spinning == true && intake_forward == true) { // If intake is on or in wrong direction
		intake.move(0); // stop intake
		intake_spinning = false;
	} else {
		intake.move(INTAKE_VELOCITY); // Spin intake forward
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
		intake_spinning = true;
		intake_forward = false;
	}
}

#define LADY_BROWN_VELOCITY 127.0
#define LADY_BROWN_kP 2.3
#define LADY_BROWN_kI 0.000
#define LADY_BROWN_kD 0.6

knights::PIDController lady_brown_PID(LADY_BROWN_kP, LADY_BROWN_kI, LADY_BROWN_kD, 10.0, 127.0);

#define LADY_BROWN_DOWN 0
#define LADY_BROWN_LOAD1 36
#define LADY_BROWN_LOAD2 225
#define LADY_BROWN_SCORE 155
#define LADY_BROWN_ALLIANCE 193
#define LADY_BROWN_END_TOLERANCE 1.0

bool lady_brown_spinning = false;
bool lady_brown_forward = false;

void lady_brown_fwd() {
	if (lady_brown_spinning == true && lady_brown_forward == true) { // If intake is on or in wrong direction
		lady_brown.move(0); // stop intake
		lady_brown_spinning = false;
	} else {
		lady_brown.move(LADY_BROWN_VELOCITY); // Spin intake forward
		lady_brown_spinning = true;
		lady_brown_forward = true;
	}
}

void lady_brown_rev() {
	if (lady_brown_spinning == true && lady_brown_forward == false) { // If intake is spinning or in the wrong direction
		lady_brown.move(0); // stop intake
		lady_brown_spinning = false;
	} else { 
		lady_brown.move(-INTAKE_VELOCITY); // Spin the intake in reverse
		lady_brown_spinning = true;
		lady_brown_forward = false;
	}
}

void lady_brown_to_angle(float angle, int timeout, bool async = true) { // angle in 0-360 deg
	if (async) {
		pros::Task task([&]() {
			lady_brown_to_angle(angle, timeout, false);
		});
		pros::delay(20);
		return;
	}

    float error = angle - lady_brown_rotation.get_angle()/100.0;

    lady_brown_PID.reset();
    lady_brown.set_brake_mode(pros::MotorBrake::brake);
    lady_brown_spinning = true;

    while (fabsf(error) > LADY_BROWN_END_TOLERANCE && lady_brown_spinning) {
        error = fabs(angle - lady_brown_rotation.get_angle()/100.0);
		
        lady_brown.move(
            -lady_brown_PID.update(error) * 
            knights::direction(lady_brown_rotation.get_angle()/100.0, angle, false)
        );


        timeout -= 20;
        if (timeout < 0) {
            break;
        }

        pros::delay(20);
    }

    lady_brown.move(0);
	printf("error: %F \n", error);
	printf("position: %i \n", lady_brown_rotation.get_angle());

}

void lady_brown_down() {
    lady_brown_to_angle(LADY_BROWN_DOWN, 1500);
}

void lady_brown_load1() {
    lady_brown_to_angle(LADY_BROWN_LOAD1, 1500);
}

void lady_brown_load2() {
    lady_brown_to_angle(LADY_BROWN_LOAD2, 1500);
}

void lady_brown_score() {;
	intake_spinning = false;
	intake.move(0);
    lady_brown_to_angle(LADY_BROWN_SCORE, 1500);
}

void lady_brown_alliance() {
	intake_spinning = false;
	intake.move(0);
    lady_brown_to_angle(LADY_BROWN_ALLIANCE, 1500);
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

bool rush_mech_down = false;

void toggle_rush_mech() {
	rush_mech_down = !rush_mech_down;
	rush_mech.set_value(rush_mech_down);
}

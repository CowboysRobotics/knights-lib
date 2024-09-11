#include "autonomous.h" 

#include "knights/api.h"
#include "knights/robot/chassis.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

#define INTAKE_VELOCITY 127
#define RIGHT -1
#define LEFT 1

void right_wp_auton(knights::RobotChassis *chassis) {

    // initialize all controllers and movements
    pros::MotorGroup intake({16,19}, pros::MotorGears::green);
    pros::adi::Pneumatics clamp(8, true);

    chassis->set_position(knights::Pos(-60.5, -14.75, 3*M_PI/2));

    pros::delay(10);

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(5, 0.0, 0.0, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(60, 0.1, 0, 19.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    // move towards blue wall stake
    lateralController.lateral_move(-17, 4.0, 1000);
    turnController.turn_for(90, 2.0, 1000);
    lateralController.lateral_move(-7, 1.0, 400);

    // put ring on wall stake
    intake.move(-INTAKE_VELOCITY);

    pros::delay(900);

    intake.move(INTAKE_VELOCITY);

    pros::delay(300);

    // move away from wall stake
    lateralController.lateral_move(14.0, 4.0, 1000);

    // turn towards mobile goal
    turnController.turn_for(115, 3.0, 1000);

    // drive towards mobile goal
    lateralController.lateral_move(-22, 4.0, 1000);

    intake.move(0);

    // last correction for mobile goal
    turnController.turn_for(25, 5.0, 300);

    // hit mobiel goal
    lateralController.lateral_move(-8, 3.0, 1000);

    // grab mobile goal
    clamp.retract();

    // turn to second disc stack
    turnController.turn_for(135, 2.0, 450);

    // make a new, slower PID controller
    knights::PIDController slowPID(5, 0.0, 0.0, 0.0, 70.0);
	knights::RobotController slowController(chassis, &lateralPID, &ramsete_constants, false);

    // knock over stack
    slowController.lateral_move(17, 4.0, 800);
    slowController.lateral_move(-17, 3.0, 400);

    // turn on intake
    intake.move(-INTAKE_VELOCITY);

    // pick up disc
    slowController.lateral_move(14.0, 3.0, 600);

    // turn towards two stacks of 2
    turnController.turn_for(80, 4.0, 600); // final ring turn

    // knock over stack
    lateralController.lateral_move(12, 4.0, 800);
    slowController.lateral_move(-12, 3.0, 400);

    // turn on intake
    intake.move(-INTAKE_VELOCITY);

    // pick up last disc
    slowController.lateral_move(20.0, 3.0, 600);
    turnController.turn_for(10, 4.0, 250);
     slowController.lateral_move(-20.0, 3.0, 600);
     turnController.turn_for(20, 4.0, 250);
      slowController.lateral_move(20.0, 3.0, 600);
}

void programming_skills(knights::RobotChassis *chassis) {
    // initialize all controllers and movements
    pros::MotorGroup intake({16,19}, pros::MotorGears::green);
    pros::adi::Pneumatics clamp(8, true);

    chassis->set_position(knights::Pos(-60, 0, 0));

    pros::delay(20);

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(5, 0.0, 0.0, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

    // make a new, slower PID controller
    knights::PIDController slowPID(5, 0.0, 0.0, 0.0, 70.0);
	knights::RobotController slowController(chassis, &slowPID, &ramsete_constants, false);

	knights::PIDController turnPID(60, 0.1, 0, 15.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

	intake.set_reversed(true, 1);

    intake.move(-INTAKE_VELOCITY);
    pros::delay(800);
    
    lateralController.lateral_move(15.0, 2.0, 1000);

    printf("pos: %lf %lf %lf\n", chassis->get_position().x, chassis->get_position().y, chassis->get_position().heading);

    turnController.turn_to_angle(90, LEFT, 2.0, 1000, false);

    pros::delay(200);

    lateralController.lateral_move(-10, 2, 1000);

    pros::delay(200);

    slowController.lateral_move(-14, 2, 1250);

    pros::delay(200);

    clamp.retract();

    turnController.turn_to_angle(0, RIGHT, 2, 1000);

    pros::delay(500);

    lateralController.lateral_move(22.0, 2.0, 1000);

    pros::delay(500);

    turnController.turn_to_angle(-90, RIGHT, 2, 1000);

    pros::delay(500);

    lateralController.lateral_move(28.0, 2.0, 1000);

    pros::delay(500);

    turnController.turn_to_angle(180, RIGHT, 2.0, 1000);

    pros::delay(500);

    lateralController.lateral_move(26.0, 3.0, 1000);

    pros::delay(500);

    turnController.turn_to_angle(90, RIGHT, 3.0, 1000);

    lateralController.lateral_move(-20);

    turnController.turn_to_angle(45, RIGHT, 2.0, 750);

    lateralController.lateral_move(24.0, 3.0, 750);

    turnController.turn_to_angle(90, LEFT);


}

void pid_tuning(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(5, 0.0, 0.0, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(60, 0.1, 0, 19.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    lateralController.lateral_move(36.0, 4.0, 4000);
    // turnController.turn_for(180, 10, 2000, false);
}
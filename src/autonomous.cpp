#include "autonomous.h" 

#include "knights/api.h"
#include "knights/robot/chassis.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

#define INTAKE_VELOCITY 127
#define RIGHT -1
#define LEFT 1

void right_wp_auton(knights::RobotChassis *chassis) {

    // initialize all controllers and movements
    pros::MotorGroup intake({16,19}, pros::MotorGears::green);
    pros::adi::Pneumatics clamp(8, true);
    pros::adi::Pneumatics doinker(7,false);

    chassis->set_position(knights::Pos(-60.5, -14.75, 3*M_PI/2));

    pros::delay(10);

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(5, 0.0, 0.0, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(30, 0.15, 0.7, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    intake.set_reversed(true, 1);


    // move towards blue wall stake
    lateralController.lateral_move(-18.0, 4.0, 1000);
    turnController.turn_for(95, 2.0, 1000);
    lateralController.lateral_move(-14, 1.0, 400);

    // put ring on wall stake
    intake.move(-INTAKE_VELOCITY);
    pros::delay(500);

    // intake.move(INTAKE_VELOCITY);
    lateralController.lateral_move(17, 1.0, 400);
    intake.move(0);
    pros::delay(300);
    turnController.turn_for(110,2.0,1000);
    lateralController.lateral_move(-9.0,1.0,1000);
    
    pros::delay(200);
    
    turnController.turn_for(98,0.5,300);
    lateralController.lateral_move(-17,1.0,1000);
    pros::delay(2000);



    lateralController.lateral_move(-10,0.5,600);
    clamp.retract();

    
    pros::delay(300);

    turnController.turn_for(128,0.5,750);
    intake.move(-INTAKE_VELOCITY);
    pros::delay(200);
    lateralController.lateral_move(25,2,400);

    pros::delay(200);

    lateralController.lateral_move(15,0.5,250);

    pros::delay(1000);

    turnController.turn_for(134,5.0,1000);
    lateralController.lateral_move(24.0,0.5,400);

    doinker.extend();

    // lateralController.lateral_move(-5,1.0,400);
    // pros::delay(1000);
    // turnController.turn_for(130,2.0,1000);



    // intake.move(-INTAKE_VELOCITY);

    // pros::delay(2000);


    // lateralController.lateral_move(25,4.0,2000);









  // pros::delay(5000);

//     // move away from wall stake
//     lateralController.lateral_move(14.0, 4.0, 1000);

//     // turn towards mobile goal
//     turnController.turn_for(115, 3.0, 1000);

//     // drive towards mobile goal
//     lateralController.lateral_move(-22, 4.0, 1000);

//     intake.move(0);

//     // last correction for mobile goal
//     turnController.turn_for(25, 5.0, 300);

//     // hit mobiel goal
//     lateralController.lateral_move(-8, 3.0, 1000);

//     // grab mobile goal
//     clamp.retract();

//     // turn to second disc stack
//     turnController.turn_for(135, 2.0, 450);

//     // make a new, slower PID controller
//     knights::PIDController slowPID(5, 0.0, 0.0, 0.0, 70.0);
// 	knights::RobotController slowController(chassis, &lateralPID, &ramsete_constants, false);

//     // knock over stack
//     slowController.lateral_move(17, 4.0, 800);
//     slowController.lateral_move(-17, 3.0, 400);

//     // turn on intake
//     intake.move(-INTAKE_VELOCITY);

//     // pick up disc
//     slowController.lateral_move(14.0, 3.0, 600);

//     // turn towards two stacks of 2
//     turnController.turn_for(80, 4.0, 600); // final ring turn

//     // knock over stack
//     lateralController.lateral_move(12, 4.0, 800);
//     slowController.lateral_move(-12, 3.0, 400);

//     // turn on intake
//     intake.move(-INTAKE_VELOCITY);

//     // pick up last disc
//     slowController.lateral_move(20.0, 3.0, 600);
//     turnController.turn_for(10, 4.0, 250);
//      slowController.lateral_move(-20.0, 3.0, 600);
//      turnController.turn_for(20, 4.0, 250);
//       slowController.lateral_move(20.0, 3.0, 600);
// 
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

	knights::PIDController turnPID(60, 0.1, 0.7, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

	intake.set_reversed(true, 1);

    intake.move(-INTAKE_VELOCITY);
    pros::delay(800);
    
    lateralController.lateral_move(13.0, 2.0, 1000);

    printf("pos: %lf %lf %lf\n", chassis->get_position().x, chassis->get_position().y, chassis->get_position().heading);

    turnController.turn_to_angle(90, LEFT, 2.0, 1000, false);

    pros::delay(1001);

    lateralController.lateral_move(-10, 2, 1000);

    pros::delay(1001);

    slowController.lateral_move(-15, 2, 1250);

    pros::delay(1001);

    clamp.retract();

    turnController.turn_to_angle(0, RIGHT, 2, 1000);

    pros::delay(1001);

    lateralController.lateral_move(26.0, 2.0, 1000);

    pros::delay(1001);

    turnController.turn_to_angle(200, RIGHT, 2, 1000);

    pros::delay(1001);

    lateralController.lateral_move(34.0, 2.0, 1000);

    pros::delay(1001);

    turnController.turn_to_angle(-90, RIGHT, 2.0, 750);

    pros::delay(1001);

    lateralController.lateral_move(18);

    pros::delay(1001);

    turnController.turn_to_angle(110, RIGHT, 2.0, 1001);

    pros::delay(1001);

    lateralController.lateral_move(-16.0, 3.0, 1001);

    clamp.extend();

    pros::delay(300);

    lateralController.lateral_move(24, 3.0, 1001);

    turnController.turn_to_angle(45, RIGHT);

    lateralController.lateral_move(16, 2.0, 1001);

    turnController.turn_to_angle(90, LEFT, 1001);


}

void pid_tuning(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(5.0, 0.0, 0.0, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(30, 0.15, 0.7, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    // lateralController.lateral_move(48.0, 4.0, 4000);
    turnController.turn_for(180, 3, 2000, false);
}

void alex_skills(knights::RobotChassis *chassis) {
// initialize all controllers and movements
    pros::MotorGroup intake({16,19}, pros::MotorGears::green);
    pros::adi::Pneumatics clamp(8, true);
    pros::adi::Pneumatics doinker(7,false);

    chassis->set_position(knights::Pos(-60.5, -14.75, 3*M_PI/2));

    pros::delay(10);

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(5, 0.0, 0.0, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(30, 0.15, 0.7, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    intake.set_reversed(true, 1);

    intake.move(-INTAKE_VELOCITY);
    pros::delay(400);
    lateralController.lateral_move(15.5,1.0,1000);
    intake.move(0);    
    turnController.turn_for(97.0,2.0,1000);
    lateralController.lateral_move(-18.0,1.0,1000);
    clamp.retract();
    pros::delay(200);
    turnController.turn_for(-106,2.0,1000);
    intake.move(-INTAKE_VELOCITY);
    lateralController.lateral_move(20.0,1.0,800);
    lateralController.lateral_move(10.0,0.5,200);
    pros::delay(600);
    turnController.turn_for(-94,1.0,400);
    lateralController.lateral_move(18.0,1.0,800);
    lateralController.lateral_move(8,0.5,200);
    pros::delay(1000);
    turnController.turn_for(-147,1.0,400);
    lateralController.lateral_move(20.0,1.5,500);
    pros::delay(600);
    turnController.turn_for(-30,0.5,200);
    lateralController.lateral_move(10.0,0.5,400);
    pros::delay(600);
    turnController.turn_for(50,1,400);   
    lateralController.lateral_move(-10.,0.5,400);
    turnController.turn_for(60,2.0,600);
    lateralController.lateral_move(14,1.0,400);


}
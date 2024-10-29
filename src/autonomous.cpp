#include "autonomous.h" 

#include "knights/api.h"
#include "knights/robot/chassis.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include "knights/logger/logger.h"
#include <iomanip>

#define RIGHT 1
#define LEFT -1


#define INTAKE_VELOCITY 300

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

	knights::PIDController turnPID(40, 0.15, 0.7, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    intake.set_reversed(true, 1);


    // move towards blue wall stake
    lateralController.lateral_move(-18.0, 4.0, 1000);
    turnController.turn_for(99, 2.0, 700);
    lateralController.lateral_move(-14, 1.0, 400);

    // put ring on wall stake
    intake.move(-INTAKE_VELOCITY);
    pros::delay(500);

    // intake.move(INTAKE_VELOCITY);
    lateralController.lateral_move(13, 1.0, 400);
    intake.move(0);
    pros::delay(300);
    turnController.turn_for(138,2.0,1000);
    lateralController.lateral_move(-9.0,1.0,1000);
    
    pros::delay(200);
    
    // turnController.turn_for(50,2.0,500);
    lateralController.lateral_move(-17,1.0,1000);
    pros::delay(700);



    lateralController.lateral_move(-12,1.0,600);
    clamp.retract();

    
    pros::delay(300);

    turnController.turn_for(137,0.5,750);
    intake.move(-INTAKE_VELOCITY);
    pros::delay(200);
    lateralController.lateral_move(26,2,400);

    pros::delay(200);

    lateralController.lateral_move(15,0.5,250);

    pros::delay(3000);

    turnController.turn_for(118,5.0,1000);
    lateralController.lateral_move(24.0,0.5,400);

    doinker.extend();

}

void unsafe_wp_auton(knights::RobotChassis *chassis) {

    // initialize all controllers and movements
    pros::MotorGroup intake({16,19}, pros::MotorGears::green);
    pros::adi::Pneumatics clamp(8, true);
    pros::adi::Pneumatics doinker(7,false);

    chassis->set_position(knights::Pos(-60.5, -14.75, 3*M_PI/2));

    pros::delay(10);

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(5, 0.0, 0.0, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(40, 0.15, 0.7, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    intake.set_reversed(true, 1);


    // move towards blue wall stake
    lateralController.lateral_move(-18.0, 4.0, 1000);
    turnController.turn_for(99, 2.0, 700);
    lateralController.lateral_move(-14, 1.0, 400);

    // put ring on wall stake
    intake.move(-INTAKE_VELOCITY);
    pros::delay(500);

    // intake.move(INTAKE_VELOCITY);
    lateralController.lateral_move(19, 1.0, 400);
    intake.move(0);
    pros::delay(300);
    turnController.turn_for(140,2.0,1000);
    lateralController.lateral_move(-12.0,1.0,1000);
    
    pros::delay(200);
    
    // turnController.turn_for(30,2.0,500);
    lateralController.lateral_move(-17,1.0,800);
    pros::delay(500);



    lateralController.lateral_move(-10,1.0,600);
    clamp.retract();

    
    pros::delay(300);

    turnController.turn_for(140,0.5,750);
    intake.move(-INTAKE_VELOCITY);
    pros::delay(200);
    lateralController.lateral_move(25,2,400);

    pros::delay(200);

    lateralController.lateral_move(15,0.5,250);

    pros::delay(3000);

    turnController.turn_to_angle(0, LEFT, 2.0, 1000);

}

void left_wp_auton(knights::RobotChassis *chassis){

    // initialize all controllers and movements
    pros::MotorGroup intake({16,19}, pros::MotorGears::green);
    pros::adi::Pneumatics clamp(8, true);
    pros::adi::Pneumatics doinker(7,false);

    chassis->set_position(knights::Pos(-60.5, -14.75, 3*M_PI/2));

    pros::delay(10);

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(5, 0.0, 0.0, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(40, 0.15, 0.7, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    intake.set_reversed(true, 1);


    // move towards blue wall stake
    lateralController.lateral_move(-18.0, 4.0, 1000);
    turnController.turn_for(-99, 2.0, 700);
    lateralController.lateral_move(-14, 1.0, 400);

    // put ring on wall stake
    intake.move(-INTAKE_VELOCITY);
    pros::delay(500);

    // intake.move(INTAKE_VELOCITY);
    lateralController.lateral_move(13, 1.0, 400);
    intake.move(0);
    pros::delay(300);
    turnController.turn_for(-138,2.0,1000);
    lateralController.lateral_move(-9.0,1.0,1000);
    
    pros::delay(200);
    
    // turnController.turn_for(50,2.0,500);
    lateralController.lateral_move(-17,1.0,1000);
    pros::delay(700);



    lateralController.lateral_move(-12,1.0,600);
    clamp.retract();

    
    pros::delay(300);

    turnController.turn_for(-137,0.5,750);
    intake.move(-INTAKE_VELOCITY);
    pros::delay(200);
    lateralController.lateral_move(26,2,400);

    pros::delay(200);

    lateralController.lateral_move(15,0.5,250);

    pros::delay(1500);

    turnController.turn_for(-118,5.0,1000);
    lateralController.lateral_move(24.0,0.5,400);

    doinker.extend();

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

    intake.move(INTAKE_VELOCITY);
    pros::delay(800);
    
    lateralController.lateral_move(14.0, 2.0, 1000);

    printf("pos: %lf %lf %lf\n", chassis->get_position().x, chassis->get_position().y, chassis->get_position().heading);

    turnController.turn_to_angle(90, LEFT, 2.0, 1000, false);

    pros::delay(750);

    lateralController.lateral_move(-10, 2, 1000);

    pros::delay(750);

    slowController.lateral_move(-17, 2, 1250);

    pros::delay(750);

    clamp.retract();

    turnController.turn_to_angle(0, RIGHT, 2, 1000);

    pros::delay(750);

    lateralController.lateral_move(30.0, 2.0, 1000);

    pros::delay(750);

    turnController.turn_to_angle(200, RIGHT, 2, 1000);

    pros::delay(750);

    lateralController.lateral_move(40.0, 2.0, 1000);

    pros::delay(750);

    turnController.turn_to_angle(-90, LEFT, 2.0, 2000);

    pros::delay(750);

    lateralController.lateral_move(18);

    pros::delay(2000);

    turnController.turn_to_angle(90, RIGHT, 2.0, 1000);

    pros::delay(750);

    lateralController.lateral_move(-18.0, 3.0, 1000);

    clamp.extend();

    pros::delay(300);

    lateralController.lateral_move(24, 3.0, 1000);

    turnController.turn_to_angle(45, RIGHT, 3.0, 500);    

    lateralController.lateral_move(14, 2.0, 750);

    turnController.turn_to_angle(100, LEFT, 2.0, 500);

    lateralController.lateral_move(18.0, 3.0, 750);

    turnController.turn_to_angle(180, LEFT, 3.0, 1500);
    turnController.turn_to_angle(-90, LEFT, 3.0, 1500);

    lateralController.lateral_move(-18.0, 3.0, 750);

    lateralController.lateral_move(-16.0, 3.0, 750);

    clamp.retract();

    turnController.turn_to_angle(0,0,2.0,750);

    lateralController.lateral_move(26.0, 2.0, 750);

    turnController.turn_to_angle(90, 0, 3.0, 750);

    lateralController.lateral_move(26.0, 2.0, 750);

    turnController.turn_to_angle(180, 0, 2.0, 750);

    slowController.lateral_move(40.0, 2.0, 750);

    turnController.turn_to_angle(-90, 0, 2.0, 750);

    lateralController.lateral_move(-15, 2.0, 750);

    clamp.extend();

    // turnController.turn_to_angle(0, LEFT, 2.0, 1500);
    // turnController.turn_to_angle(80, LEFT, 2.0, 1500);

    // lateralController.lateral_move(24.0, 2.0, 500);
    // lateralController.lateral_move(12.0, 2.0, 500);
    // lateralController.lateral_move(12.0, 2.0, 500);

    // pros::delay(1500);

    // turnController.turn_to_angle(0, 0);

    // lateralController.lateral_move(-10, 3.0, 750);

    // clamp.extend();

    // pros::delay(1000);

    // lateralController.lateral_move(24, 2.0, 2000);

    // turnController.turn_to_angle(-70, 0);

    // lateralController.lateral_move(24.0, 2.0, 750);

}

void pid_tuning(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(5.9, 0.0016, 0.01, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(60, 0.1, 0.7, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    // turnController.turn_to_angle(0,RIGHT);
    lateralController.lateral_move(24.0, 3.0, 750);
    pros::delay(100);
    turnController.turn_to_angle(0, 0);
    pros::delay(100);
    lateralController.lateral_move(24.0, 4.0, 750);
}



pros::Motor intake2(6, pros::MotorGears::blue);
pros::adi::Pneumatics clamp2(2, false);
pros::adi::Pneumatics doinker2(4, true);
pros::MotorGroup snacky_cakes2({20, 4}, pros::v5::MotorGears::green);


bool intake_spinning2 = false;

void intake_fwd2() {
	if (intake_spinning2 == true && intake2.get_direction() == -1) { // If intake is on or in wrong direction
		intake2.move(0); // stop intake
		intake_spinning2 = false;
	} else {
		intake2.move(INTAKE_VELOCITY); // Spin intake forward
		intake_spinning2 = true;
	}
}

void intake_rev2() {
    printf("reached\n");
	if (intake_spinning2 == true && intake2.get_direction() == 1) { // If intake is spinning or in the wrong direction
		intake2.move(0); // stop intake
		intake_spinning2 = false;
	} else { 
		intake2.move(-INTAKE_VELOCITY); // Spin the intake in reverse
		intake_spinning2 = true;
	}
}

bool clamp_down2 = false;

void clamp_out2() {
	clamp_down2 = !clamp_down2;
	clamp2.set_value(clamp_down2);
}

void left_wp_red(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

    knights::PIDController lateralPID(5.9, 0.0016, 0.01, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(60, 0.1, 0.7, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    chassis->set_position(knights::Pos(-60.5, -14.75, M_PI/2));

    clamp2.set_value(true);

    lateralController.lateral_move(-12);
    pros::delay(200);
    turnController.turn_to_angle(0, 0);
    pros::delay(200);
    lateralController.lateral_move(-7);
    pros::delay(200);
    intake_rev2();
    pros::delay(200);
    lateralController.lateral_move(12.0);
    pros::delay(200);
    turnController.turn_to_angle(-105, 0, 3.0, 750);
    pros::delay(200);
    lateralController.lateral_move(-13);
    pros::delay(200);
    turnController.turn_to_angle(-155, 0, 3.0, 750);
    pros::delay(200);
    lateralController.lateral_move(-12);
    pros::delay(200);
    lateralController.lateral_move(-14);
    pros::delay(200);
    clamp2.set_value(false);
    pros::delay(400);
    turnController.turn_to_angle(100, 0);
    lateralController.lateral_move(14);
    pros::delay(200);
    turnController.turn_to_angle(0, 0);
    pros::delay(200);
    lateralController.lateral_move(12);
    pros::delay(200);
    lateralController.lateral_move(-16);
    pros::delay(200);
    turnController.turn_to_angle(60, 0);
    pros::delay(200);
    doinker2.set_value(false);
    pros::delay(200);
    lateralController.lateral_move(-12);

}

void left_wp_blue(knights::RobotChassis *chassis) {
       knights::RamseteConstants ramsete_constants(1, 0.5);

    knights::PIDController lateralPID(5.9, 0.0016, 0.01, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(60, 0.1, 0.7, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    chassis->set_position(knights::Pos(-60.5, -14.75, 3*M_PI/2));

    clamp2.set_value(true);

    lateralController.lateral_move(-12);
    pros::delay(200);
    turnController.turn_to_angle(0, LEFT); 
    pros::delay(200); 
    lateralController.lateral_move(-7);
    pros::delay(200);
    intake_rev2();
    pros::delay(200);
    lateralController.lateral_move(12.0);
    pros::delay(200);
    turnController.turn_to_angle(105, LEFT, 3.0, 750);
    pros::delay(200);
    lateralController.lateral_move(-13);
    pros::delay(200);
    turnController.turn_to_angle(155, LEFT, 3.0, 750);
    pros::delay(200);
    lateralController.lateral_move(-12);
    pros::delay(200);
    lateralController.lateral_move(-14);
    pros::delay(200);
    clamp2.set_value(false);
    pros::delay(400);
    turnController.turn_to_angle(100+180, LEFT);
    lateralController.lateral_move(14);
    pros::delay(200);
    turnController.turn_to_angle(0, LEFT);
    pros::delay(200);
    lateralController.lateral_move(12);
    pros::delay(200);
    lateralController.lateral_move(-16);
    pros::delay(200);
    turnController.turn_to_angle(240, RIGHT);
    pros::delay(200);
    doinker2.set_value(false);
    pros::delay(200);
    lateralController.lateral_move(-12);
//  */
}

void pp_test(knights::RobotChassis *chassis) {

	std::string s = "left_auton.txt";

	knights::AdvancedRoute test_route = advanced_route_from_file(s);

	// knights::logger::green("started route read");

	// for (auto action : test_route.actions) {
	// 	if (action.type == knights::action_type::FOLLOW)
	// 		// knights::logger::red("follow");
	// 	else if (action.type == knights::action_type::LATERAL)
	// 		// knights::logger::red("lateral");
	// 	else if (action.type == knights::action_type::TURN)
	// 		// knights::logger::red("turn");
	// }

	for (auto route : test_route.routes) {
		for (auto pt : route.second.positions) {
			knights::logger::green(knights::logger::string_format("pt: %lf %lf %lf", pt.x, pt.y, pt.heading));
		}
	}

    // chassis->set_position(-36.0, -60.0, M_PI/2);
    // chassis->set_prev_position(-36.0, -60.0, M_PI/2);

    chassis->set_position(-60.0, 12.0, M_PI/2);
    chassis->set_prev_position(-60.0, 12.0, M_PI/2);



    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(6, 0.00019, 0.01, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(60, 0.1, 0.7, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);


	knights::input::AutonomousInputMap inputMap;
    inputMap.bind_action("intakeRev", intake_rev2);
    inputMap.bind_action("intakeFwd", intake_fwd2);
    inputMap.bind_action("clamp", clamp_out2);

	test_route.execute(chassis, &lateralPID, &turnPID, &inputMap);
}
void blue_right_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

    knights::PIDController lateralPID(5.9, 0.0016, 0.01, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(60, 0.1, 0.7, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    chassis->set_position(knights::Pos(-60.5, -14.75, 3*M_PI/2));

    clamp2.set_value(false);

    lateralController.lateral_move(-12);
    pros::delay(200);
    turnController.turn_to_angle(0, LEFT); 
    pros::delay(200); 
    lateralController.lateral_move(-7);
    pros::delay(200);
    intake_rev2();
    pros::delay(300);
    lateralController.lateral_move(6.0);
    pros::delay(200);
    clamp2.set_value(true);
    intake_fwd2();
    lateralController.lateral_move(6.0);
    pros::delay(200);
    turnController.turn_to_angle(105, LEFT, 3.0, 750);
    pros::delay(200);
    lateralController.lateral_move(-9);
    pros::delay(200);
    turnController.turn_to_angle(155, LEFT, 3.0, 750);
    pros::delay(200);
    lateralController.lateral_move(-10);
    pros::delay(200);
    lateralController.lateral_move(-14);
    pros::delay(200);
    clamp2.set_value(false);
    intake_rev2();
    pros::delay(500);
    turnController.turn_to_angle(255,LEFT);
    pros::delay(400);
    lateralController.lateral_move(17);
    pros::delay(400);
    turnController.turn_to_angle(360,LEFT);
    pros::delay(300);
    lateralController.lateral_move(10);
    pros::delay(400);
    turnController.turn_to_angle(68,LEFT);
    pros::delay(200);
    lateralController.lateral_move(19);
    turnController.turn_to_angle(30, RIGHT);
}

void blue_left_nwp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

    knights::PIDController lateralPID(5.9, 0.0016, 0.01, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(60, 0.1, 0.7, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    chassis->set_position(knights::Pos(-60.5, -14.75, 3*M_PI/2));

    clamp2.set_value(false);

    lateralController.lateral_move(-12);
    pros::delay(200);
    turnController.turn_to_angle(180, RIGHT); 
    pros::delay(200); 
    lateralController.lateral_move(-7);
    pros::delay(200);
    intake_rev2();
    pros::delay(300);
    lateralController.lateral_move(6.0);
    pros::delay(200);
    clamp2.set_value(true);
    intake_fwd2();
    lateralController.lateral_move(6.0);
    pros::delay(200);
    turnController.turn_to_angle(75, RIGHT, 3.0, 750);
    pros::delay(200);
    lateralController.lateral_move(-9);
    pros::delay(200);
    turnController.turn_to_angle(25, RIGHT, 3.0, 750);
    pros::delay(200);
    lateralController.lateral_move(-10);
    pros::delay(200);
    lateralController.lateral_move(-14);
    pros::delay(200);
    clamp2.set_value(false);
    intake_rev2();
    pros::delay(500);
    turnController.turn_to_angle(290,RIGHT);
    pros::delay(400);
    lateralController.lateral_move(20);
    pros::delay(400);

}

void red_right_nwp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

    knights::PIDController lateralPID(5.9, 0.0016, 0.01, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(60, 0.1, 0.7, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    chassis->set_position(knights::Pos(-60.5, -14.75, 3*M_PI/2));

    clamp2.set_value(false);

    lateralController.lateral_move(-12);
    pros::delay(200);
    turnController.turn_to_angle(0, LEFT); 
    pros::delay(200); 
    lateralController.lateral_move(-7);
    pros::delay(200);
    intake_rev2();
    pros::delay(300);
    lateralController.lateral_move(6.0);
    pros::delay(200);
    clamp2.set_value(true);
    intake_fwd2();
    lateralController.lateral_move(6.0);
    pros::delay(200);
    turnController.turn_to_angle(105, LEFT, 3.0, 750);
    pros::delay(200);
    lateralController.lateral_move(-9);
    pros::delay(200);
    turnController.turn_to_angle(155, LEFT, 3.0, 750);
    pros::delay(200);
    lateralController.lateral_move(-10);
    pros::delay(200);
    lateralController.lateral_move(-14);
    pros::delay(200);
    clamp2.set_value(false);
    intake_rev2();
    pros::delay(500);
    turnController.turn_to_angle(260,LEFT);
    pros::delay(400);
}

void red_left_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

    knights::PIDController lateralPID(5.9, 0.0016, 0.01, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(60, 0.1, 0.7, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    chassis->set_position(knights::Pos(-60.5, -14.75, 3*M_PI/2));

    clamp2.set_value(false);

    lateralController.lateral_move(-12);
    pros::delay(200);
    turnController.turn_to_angle(180, RIGHT); 
    pros::delay(200); 
    lateralController.lateral_move(-7);
    pros::delay(200);
    intake_rev2();
    pros::delay(300);
    lateralController.lateral_move(6.0);
    pros::delay(200);
    clamp2.set_value(true);
    intake_fwd2();
    lateralController.lateral_move(6.0);
    pros::delay(200);
    turnController.turn_to_angle(75, RIGHT, 3.0, 750);
    pros::delay(200);
    lateralController.lateral_move(-9);
    pros::delay(200);
    turnController.turn_to_angle(25, RIGHT, 3.0, 750);
    pros::delay(200);
    lateralController.lateral_move(-10);
    pros::delay(200);
    lateralController.lateral_move(-14);
    pros::delay(200);
    clamp2.set_value(false);
    intake_rev2();
    pros::delay(500);
    turnController.turn_to_angle(290,RIGHT);
    pros::delay(400);
    lateralController.lateral_move(20);
    pros::delay(400);
    turnController.turn_to_angle(197,RIGHT);
    pros::delay(300);
    lateralController.lateral_move(12);
    pros::delay(400);
    turnController.turn_to_angle(100,RIGHT);
    pros::delay(200);
    lateralController.lateral_move(18);
    turnController.turn_to_angle(140, LEFT);
}


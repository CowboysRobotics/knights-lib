#include "autonomous.h" 

#define RIGHT 1
#define LEFT -1

#define LATERAL_kP 5
#define LATERAL_kI 0
#define LATERAL_kD 0.0065

#define TURN_kP 54
#define TURN_kI 0.017
#define TURN_kD 0.002

void pid_tuning(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
}


void pp_test(knights::RobotChassis *chassis) {

	std::string s = "test-whole.txt";

	knights::AdvancedRoute test_route = advanced_route_from_file(s);
	// for (auto route : test_route.routes) {
	// 	for (auto pt : route.second.positions) {
	// 		// knights::logger::green(knights::logger::string_format("pt: %lf %lf %lf", pt.x, pt.y, pt.heading));
	// 	}
	// }

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 100.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
	
	knights::input::AutonomousInputMap inputMap;
    inputMap.bind_action("intakeRev", intake_out);
    inputMap.bind_action("intakeFwd", intake_in);
    inputMap.bind_action("clamp", clamp_toggle);


	test_route.execute(chassis, &lateralPID, &turnPID, &inputMap);
}

#define WAIT 140

void skills(knights::RobotChassis *chassis) {
	knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
	
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}

void red_left_wp(knights::RobotChassis *chassis) {
	knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
	
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}


void red_rush_right_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}

void red_right_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}


void red_rush_right_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}

void blue_right_wp(knights::RobotChassis *chassis) {

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
	
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}


void blue_rush_left_wp(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}

void blue_right_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);
	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}


void blue_left_rush_elim(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(LATERAL_kP, LATERAL_kI, LATERAL_kD, 10.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID);

	knights::PIDController turnPID(TURN_kP, TURN_kI, TURN_kD, 10.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID);
    
	intake.set_reversed(false,0);
	intake.set_reversed(true, 1);

}

void empty(knights::RobotChassis *chassis) {
	return;
}

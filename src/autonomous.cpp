#include "autonomous.h" 

#include "knights/api.h"
#include "knights/robot/chassis.h"

void skills(knights::RobotChassis *chassis) {

    chassis->set_position(knights::Pos(-60, 0, 0));

    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(2, 0.0, 0.0, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(40, 0.0, 0.0, 0.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    // drive back towards red stake
    lateralController.lateral_move(-3.0, 2.0, 2000);

    // put ring onto red stake

    // drive forward to go for first mobile goal
    lateralController.lateral_move(20.0, 4.0, 2000);

    

}

void pid_tuning(knights::RobotChassis *chassis) {
    knights::RamseteConstants ramsete_constants(1, 0.5);

	knights::PIDController lateralPID(5, 0.0, 0.0, 0.0, 127.0);
	knights::RobotController lateralController(chassis, &lateralPID, &ramsete_constants, false);

	knights::PIDController turnPID(70, 0.1, 20, 19.0, 127.0);
	knights::RobotController turnController(chassis, &turnPID, &ramsete_constants, false);

    // lateralController.lateral_move(36.0, 4.0, 4000);
    turnController.turn_for(180, 3, 2000, false);
}
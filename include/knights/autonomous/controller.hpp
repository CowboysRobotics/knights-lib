#pragma once

#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "knights/util/position.hpp"
#include "knights/autonomous/pid.hpp"
#include "knights/autonomous/ramsete.hpp"
#include "knights/autonomous/path.hpp"
#include "knights/autonomous/profile.hpp"

#include "knights/robot/chassis.hpp"

#include <queue>

namespace knights {

  enum MovementType {
    // LATERAL,
    TURN_IN_PLACE_FOR,
    TURN_IN_PLACE_TO,
    RIGHT_SWING_TO_ANGLE,
    LEFT_SWING_TO_ANGLE,
    LATERAL_TO_POINT,
    LATERAL_TO_POSITION,
    PURE_PURSUIT_FOLLOW,
    PATH_GENERATE,
    RAMSETE_FOLLOW,
    WAIT
  };

    struct Movement {
      enum MovementType type;
      
      std::map<uint8_t, float> float_params;
      std::map<uint8_t, bool> bool_params;
      std::map<uint8_t, knights::Pos> position_params;
    };

    class RobotController {
        private:
            PIDController *lateral_pid;
            PIDController *turn_pid;
            PIDController *angular_pid;
            RamseteConstants *ramsete_constants;
            RobotChassis *chassis;
            bool use_motor_encoders = false;

            bool in_motion = false;

            std::queue<knights::Movement> actions;

            friend class AdvancedRoute;

        public:
            /**
             * @brief Construct a new Robot Controller object
             * 
             * @param chassis 
             * @param lateral_pid 
             * @param ramsete_constants 
             * @param use_motor_encoders 
             */
            RobotController(RobotChassis *chassis, PIDController *lateral_pid, PIDController *turn_pid, RamseteConstants *ramsete_constants, bool use_motor_encoders = false);

            /**
             * @brief Construct a new Robot Controller object
             * 
             * @param chassis 
             * @param lateral_pid 
             * @param turn_pid 
             * @param angular_pid 
             * @param ramsete_constants 
             * @param use_motor_encoders 
             */
            RobotController(RobotChassis *chassis, PIDController *lateral_pid, PIDController *turn_pid, PIDController *angular_pid, RamseteConstants *ramsete_constants, bool use_motor_encoders = false);

            /**
             * @brief Construct a new Robot Controller object
             * 
             * @param chassis 
             * @param pid_controller 
             * @param use_motor_encoders 
             */
            RobotController(RobotChassis *chassis, PIDController *lateral_pid, PIDController *turn_pid, bool use_motor_encoders = false);

            RobotController(RobotChassis *chassis, PIDController *lateral_pid, PIDController *turn_pid, PIDController *angular_pid, bool use_motor_encoders = false);

            /**
             * @brief Follow a route that has been read into the route memory of the robot
             * 
             * @param route Pointer to the route to follow
             * @param lookahead_distance Distance to look ahead on the route in order to obtain the target position
             * @param max_speed Maximum speed to move the bot at
             * @param forwards Whether the bot should follow with its front or back
             * @param end_tolerance Distance to end the loop at
             * @param timeout Amount of time to wait before ending the movement
             */
            void follow_route(const knights::Route &route, float lookahead_distance = 15.0, const float max_speed = 127.0, bool forwards = true, float end_tolerance = 4.0, float timeout = 5000, bool use_pid = true);

            /**
             * @brief Follow a motion profile
             * 
             * @param profile Motion Profile to follow
             * @param end_tolerance Distance from final position at which to stop (0 recommended)
             * @param forwards Whether or not to follow it forwards
             */
            void follow_profile(const knights::MotionProfile &profile, float end_tolerance = 0.0, bool forwards = true);

            /**
             * @brief Move to a position using simple straight and turn movements
             * 
             * @param desired_position Position to move to
             * @param forwards Whether or not the robot should face forwards
             * @param end_tolerance Distance to end the loop at for all movements
             * @param timeout Amount of time to wait before ending the movement for straight movements
             * @param turn_timeout Amount of time to wait before ending the movement for turn movements
             * @param wait_time Amount of time to wait between movements
             */
            void lateral_to_position(const Pos desired_position, const bool forwards = true, const float end_tolerance = 3.5, const int timeout = 1000, const int turn_timeout = 750, const int wait_time = 140);

            /**
             * @brief Move to a point (meaning only X,Y, NOT heading) using simple straight and turn movements
             * 
             * @param desired_position Position to move to
             * @param forwards Whether or not the robot should face forwards
             * @param end_tolerance Distance to end the loop at for all movements
             * @param timeout Amount of time to wait before ending the movement for straight movements
             * @param turn_timeout Amount of time to wait before ending the movement for turn movements
             * @param wait_time Amount of time to wait between movements
             */
            void lateral_to_point(const Pos desired_position, const bool forwards = true, const float end_tolerance = 3.5, const int timeout = 1000, const int turn_timeout = 750, const int wait_time = 140);

            /**
             * @brief Move to a point while setting an angular velocity to face that point, NOT recommended for small movements
             * 
             * @param point Point to move towards
             * @param forwards Whether to chase the point with the front or back of the robot
             * @param end_tolerance Distance to end away from the point (recommended to be high)
             * @param timeout Amount of time to stop following after
             * @param async Whether or not the movement should be ran in its own task
             */
            void curve_move(const knights::Pos point, const bool forwards = true, const float end_tolerance = 12.0, float timeout = 1250, bool async= false);

            /**
             * @brief Turn the robot to a specific angle
             * 
             * @param angle Angle to turn to
             * @param direction Whether to turn left (-1), right (1), or best direction (0)
             * @param end_tolerance Angle that the bot will stop moving at (ie if this is 5, the bot will stop moving 5 degrees before the heading) 
             *                      - this is used to account for the center of the bot not being the front
             * @param timeout Amount of time to wait before exiting the movement
             * @param rad Whether the provided angle is in radians or not
             */
            void turn_to_angle(const float angle, int direction = 0, float end_tolerance = 3.0, int timeout = 2000, bool rad = false); // DEGREES

            /**
             * @brief Turn the robot to an angle by moving only the right side of the drivetrain
             * 
             * @param angle Angle to turn to
             * @param end_tolerance Angle that the bot will stop moving at (ie if this is 5, the bot will stop moving 5 degrees before the heading) 
             *                      - this is used to account for the center of the bot not being the front
             * @param timeout Amount of time to wait before exiting the movement
             * @param rad Whether the provided angle is in radians or not
             */
            void right_swing_to_angle(const float angle, float end_tolerance = 3.0, int timeout = 2000, bool rad = false); // DEGREES

            /**
             * @brief Turn the robot to an angle by moving only the left side of the drivetrain
             * 
             * @param angle Angle to turn to
             * @param end_tolerance Angle that the bot will stop moving at (ie if this is 5, the bot will stop moving 5 degrees before the heading) 
             *                      - this is used to account for the center of the bot not being the front
             * @param timeout Amount of time to wait before exiting the movement
             * @param
             */
            void left_swing_to_angle(const float angle, float end_tolerance = 3.0, int timeout = 2000, bool rad = false); // DEGREES

            /**
             * @brief 
             * 
             * @param point 
             * @param direction 
             * @param end_tolerance 
             * @param timeout 
             */
            void turn_to_point(knights::Pos point, bool forwards = true, int direction = 0, float end_tolerance = 3.0, int timeout = 2000); // DEGREES

            /**
             * @brief Move in a straight line, forwards or backwards
             * 
             * @param distance Distance to move, positive for forward, negative for backward
             * @param end_tolerance Position that the bot will stop moving at (ie if this is 5, the bot will stop moving 5 inches before the position) 
             *                      - this is used to account for the center of the bot not being the front
             * @param timeout Amount of time to wait before exiting the move
             */
            void lateral_move(const float distance, float end_tolerance = 3.0, float timeout = 1500, bool async = false);

            /**
             * @brief Turn the robot left or right for a certain angle
             * 
             * @param angle Amount to change the heading by, negative for right, positive for left (think of unit circle)
             * @param end_tolerance Angle that the bot will stop moving at (ie if this is 5, the bot will stop moving 5 degrees before the heading) 
             *                      - this is used to account for the center of the bot not being the front
             * @param timeout Amount of time to wait before exiting the movement
             * @param rad Whether the provided angle is in radians or not
             */
            void turn_for(const float angle, const float end_tolerance = 2.0, float timeout= 750, bool rad = false); // DEGREES
    };
}

#endif

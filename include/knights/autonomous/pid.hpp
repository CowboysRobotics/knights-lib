#pragma once

#ifndef _PID_H
#define _PID_H

#include <map>
#include <cmath>

namespace knights {

    struct PIDConstants {
        float kP,kI,kD;

        /**
        * @brief Construct a new PID constants object
        * 
        * @param kP proportional tuner value
        * @param kI integral tuner value
        * @param kD derivative tuner value
        */
        PIDConstants(float kP, float kI, float kD);

        /**
         * @brief Construct a new PIDConstants object
         * 
         */
        PIDConstants();
    };

    class PIDController {
        private:
            // tuner values for the PID calculation
            float kP,kI,kD;

            // values to clamp the PID to
            float max_velocity = 127.0; float min_velocity = 0.0;

            float slew_max = -1.0;

            float prev_speed;

            // PID use values
            float prev_error = 0; float total_error = 0;

            // integral windup limit (-1 means no limit)
            float integral_max = -1.0;

            // settled detection
            float settled_threshold = 0.5; // error change threshold to consider "settled"
            int settled_count = 0;         // consecutive iterations below threshold
            int settled_target = 5;        // consecutive iterations needed to declare settled

            // map of usable constants
            std::map<float, PIDConstants> avaliable_constants;

            friend class RobotController;
        public:
            /**
             * @brief Construct a new PID controller object
             * 
             * @param kP proportional tuner value
             * @param kI integral tuner value
             * @param kD derivative tuner value
             */
            PIDController(float kP, float kI, float kD);

            /**
             * @brief Construct a new PID controller object
             * 
             * @param constants PID Constants object with kP, kI, and kD
             */
            PIDController(PIDConstants constants);

            /**
             * @brief Construct a new PID controller object
             * 
             * @param constants PID Constants object with kP, kI, and kD
             * @param min_velocity minimum value that the system will return
             * @param max_velocity maximum value that the system will return
             */
            PIDController(PIDConstants constants, float min_velocity, float max_velocity, float slew = -1.0);

            /**
             * @brief Construct a new PID controller object
             * 
             * @param kP proportional tuner value
             * @param kI integral tuner value
             * @param kD derivative tuner value
             * @param min_velocity minimum value that the system will return
             * @param max_velocity maximum value that the system will return
             */
            PIDController(float kP, float kI, float kD, float min_velocity, float max_velocity, float slew = -1.0);

            /**
             * @brief Construct a new pid controller object with 0.0 for each tuning value
             */
            PIDController();

            /**
             * @brief Add a constant value with a key to the PID values map
             * 
             * @param key key to add the constant under, this will usually be the target value of the PID motion
             * @param constants kP, kI, kD objects
             */
            void add_constant(float key, PIDConstants constants);

            /**
             * @brief Switch PID Constants to one saved in the controller
             * 
             * @param target_val Target value - will be compared to the keys in the internal PID constants array
             */
            void switch_values(float target_val);

            /**
             * @brief Use the PID formula with the given tuner values in order to calculate a value that is adjusted for error
             * 
             * @param error desired value - current value
             * @return a speed that is calculated with the PID formula
             */
            float update(float error, bool clamp = true);

            /**
             * @brief Reset the internal values (total and previous error) of the PID controller
             * 
             * @return float 
             */
            void reset();

            /**
             * @brief Get the maximum speed of the controller
             * 
             * @return float - Maximum Speed of the controller
             */
            float get_max_speed();

            /**
             * @brief Get the minimum speed of the controller
             * 
             * @return float - Minimum Speed of the controller
             */
            float get_min_speed();

            /**
             * @brief Check if the PID controller has settled (error change below threshold for N iterations)
             * 
             * @return true if settled
             */
            bool is_settled();

            /**
             * @brief Set the integral windup limit. Set to -1 to disable.
             * 
             * @param max Maximum absolute value of the accumulated integral
             */
            void set_integral_max(float max);

            /**
             * @brief Configure settled detection parameters
             * 
             * @param threshold Error change below this is considered settled
             * @param count Number of consecutive settled iterations to declare settled
             */
            void set_settled_params(float threshold, int count);

    };

}

#endif
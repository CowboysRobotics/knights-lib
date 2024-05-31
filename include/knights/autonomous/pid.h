#pragma once

#ifndef _PID_H
#define _PID_H

namespace knights {

    class PID_Controller {
        private:
            // tuner values for the PID calculation
            float kP,kI,kD;

            // values to clamp the PID to
            float max_velocity = 127.0; float min_velocity = 0.0;
        public:
            /**
             * @brief Construct a new PID controller object
             * 
             * @param kP proportional tuner value
             * @param kI integral tuner value
             * @param kD derivative tuner value
             */
            PID_Controller(float kP, float kI, float kD);

            /**
             * @brief Construct a new PID controller object
             * 
             * @param kP proportional tuner value
             * @param kI integral tuner value
             * @param kD derivative tuner value
             * @param min_velocity minimum value that the system will return
             * @param max_velocity maximum value that the system will return
             */
            PID_Controller(float kP, float kI, float kD, float min_velocity, float max_velocity);

            /**
             * @brief Construct a new pid controller object with 0.0 for each tuning value
             */
            PID_Controller();

            /**
             * @brief Use the PID formula with the given tuner values in order to calculate a value that is adjusted for error
             * 
             * @param error desired value - current value
             * @param total_error compounded value of all error values
             * @param prev_error error from one iteration of the loop ago
             * @return a speed that is calculated with the PID formula
             */
            float update(float error, float total_error, float prev_error);

    };

}

#endif
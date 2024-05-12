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
            /*
            @brief create a new PID controller with provided values
            @param kP,kI,kD tuner values for the PID controller
            */
            PID_Controller(float kP, float kI, float kD);

            /*
            @brief create a new PID controller with provided values
            @param kP,kI,kD tuner values for the PID controller
            @param min_velocity,max_velocity values to clamp the PID controller to
            */
            PID_Controller(float kP, float kI, float kD, float min_velocity, float max_velocity);

            /*
            @brief create a new PID controller with provided values
            */
            PID_Controller();

            /*
            @brief run the PID formula with tuner values of the controller and provided values
            @param error desired value - current value
            @param total_error compounded value of all error values
            @param prev_error the previous error
            @return a speed that is calculated with the PID formula
            */
            float update(float error, float total_error, float prev_error);

    };

}

#endif
#pragma once

#ifndef _RAMSETE_CONTANTS_H
#define _RAMSETE_CONTANTS_H

namespace knights {
    class RamseteConstants {
        public:
            float damping, proportional, curvature_coefficient, tuner_v, tuner_accel, tuner_static;
            /**
             * @brief Construct a new Ramsete Constants object
             * 
             * @param damping Damping value for the Ramsete controller, must be within (0,1). default is 0.7 (zeta)
             * @param proportional Proportional value for the Ramsete controller, must be greater than 0. default is 2 (b)
             * @param curvature_coefficient Factor to use when tuning slowing down around curvse, higher avlues will increase speed around curves and vice versa
             * @param tuner_v Tuner velocity value, used to represent the desired weight of the ramsete algorithm calculated velocity in the feedforward equation
             * @param tuner_accel Tuner acceleration value, used to represent the desired weight of the current acceleration in the feedforward equation
             * @param tuner_static Tuner static friction value, used to represent the desired weight of the counter static friction part of the feedforward equation
             */
            RamseteConstants(const float &damping = 0.7, const float &proportional = 2, const float &curvature_coefficient = 2, 
                const float &tuner_v = 1, const float &tuner_accel = 0, const float &tuner_static = 0);
    };
}

#endif
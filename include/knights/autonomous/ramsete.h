#pragma once

#ifndef _RAMSETE_CONTANTS_H
#define _RAMSETE_CONTANTS_H

namespace knights {
    class RamseteConstants {
        public:
            float damping, proportional;
            /**
             * @brief Construct a new Ramsete Constants object
             * 
             * @param damping Damping value for the Ramsete controller, must be greater than 0
             * @param proportional Proportional value for the Ramsete controller, must be within (0,1)
             */
            RamseteConstants(const float &damping, const float &proportional);
    };
}

#endif
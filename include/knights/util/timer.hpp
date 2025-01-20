#pragma once

#ifndef _TIMER_H
#define _TIMER_H

namespace knights {

    struct Timer {
        long double start_time = 0;

        /**
         * @brief Create a new timer object with the start time being the time of creation
         * 
         */
        Timer();

        /**
         * @brief Reset the start time on the timer object to the current time
         * 
         */
        void reset();

        /**
         * @brief Get the current time elapsed since the creation or reset of the timer
         * 
         * @return Time in milliseconds
         */
        long double get();
    };
}

#endif
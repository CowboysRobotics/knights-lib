#pragma once

#ifndef _TIMER_H
#define _TIMER_H

namespace knights {

    struct Timer {
        long double start_time = 0;

        // @brief Create a new timer, the start time is when it is created in milliseconds
        Timer();

        // @brief Reset the timer to 0, sets the start_time variable to the current time in milliseconds
        void reset();

        // @brief Get the current time of the timer based on the defined start_time variable
        // @return Time elapsed since the creation or reset of the timer, in milliseconds
        long double get();
    };
}

#endif
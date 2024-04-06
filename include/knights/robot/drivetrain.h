#pragma once

#ifndef _DRIVETRAIN_H
#define _DRIVETRAIN_H

#include "api.h"

namespace knights {

    class Drivetrain {
        private:
            pros::Motor_Group *right_mtrs; // motors of right side of drivetrain
            pros::Motor_Group *left_mtrs;  // motors of left side of drivetrain
            float track_width; // width of the drivetrain
            float rpm; // max rpm of the drivetrain (ie 450rpm, 600 rpm, etc)
            float wheel_diameter; // diameters of the largest wheels on the drivetrain
        public:
            // @brief create a drivetrain object
            // @param right_mtrs motors of right side of drivetrain
            // @param left_mtrs motors of left side of drivetrain
            // @param track_width width of the drivetrain
            // @param rpm max rpm of the drivetrain (ie 450rpm, 600 rpm, etc)
            // @param wheel_diameter diameters of the largest wheels on the drivetrain
            Drivetrain(pros::Motor_Group *right_mtrs, pros::Motor_Group *left_mtrs, float track_width, float rpm, float wheel_diameter);
    };

}

#endif
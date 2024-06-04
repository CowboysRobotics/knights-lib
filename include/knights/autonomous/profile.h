#pragma once

#include "knights/util/position.h"
#ifndef _PROFILE_H
#define _PROFILE_H

#include "knights/robot/drivetrain.h"

namespace knights {

    struct ProfileTimestamp {
        knights::Pos position;
        float expected_velocity;
        float time;
        float right_speed;
        float left_speed;
    };

    class ProfileGenerator {
        private:
            float rpm = 0;
            float wheel_diameter = 0;
            knights::Drivetrain *chassis = nullptr;
        public:
            float speed = 0;

            ProfileGenerator(knights::Drivetrain);

            ProfileGenerator(float rpm, float wheel_diameter);

            std::vector<ProfileTimestamp> generate_profile();
    };

}

#endif
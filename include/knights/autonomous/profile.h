#pragma once

#ifndef _PROFILE_H
#define _PROFILE_H

#include "knights/robot/drivetrain.h"
#include "knights/util/position.h"
#include "knights/autonomous/path.h"


namespace knights {

    struct ProfileTimestamp {
        knights::Pos position;
        float expected_velocity;
        float time;
        float right_speed;
        float left_speed;

        ProfileTimestamp(knights::Pos position, float expected_velocity, float time,
            float right_speed, float left_speed);
    };

    class ProfileGenerator {
        private:
            float rpm = 0;
            float wheel_diameter = 0;
            knights::Drivetrain *drivetrain = nullptr;
        public:
            float speed = 0;

            ProfileGenerator(knights::Drivetrain* drivetrain);

            ProfileGenerator(float rpm, float wheel_diameter);

            std::vector<ProfileTimestamp> generate_profile(knights::Route route, knights::Pos start, knights::Drivetrain* drivetrain, 
                float lookahead, float speed_max, float speed_min, float interval);
    };

}

#endif
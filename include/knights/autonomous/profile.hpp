#pragma once

#include <vector>
#ifndef _PROFILE_H
#define _PROFILE_H

#include "knights/robot/drivetrain.hpp"
#include "knights/util/position.hpp"
#include "knights/autonomous/path.hpp"


namespace knights {

    class HermiteSpline {
        private:
            knights::Point curr;
            knights::Point target;
            knights::Point curr_tangent;
            knights::Point target_tangent;

        public:
            HermiteSpline(knights::Point curr, knights::Point target, knights::Point curr_tangent, knights::Point target_tangent);

            knights::Pos position(double t);

            knights::Pos derivatives(double t);

            knights::Point second_derivatives(double t);
    };

    struct ProfileTimestamp {
        knights::Pos position;
        float linear_velocity;
        float angular_velocity;
        float curr_distance;
        float time;
        float right_speed;
        float left_speed;

        ProfileTimestamp(knights::Pos position, float linear_velocity, float angular_velocity, float curr_distance, 
            float time, float right_speed, float left_speed);
    };

    class ProfileGenerator {
        // assumed differential drive
        float max_accel;
        float max_velocity;
        float track_width;

        ProfileGenerator(knights::Drivetrain drivetrain, float max_accel);

        ProfileGenerator(float max_velocity, float track_width, float max_accel);

        std::vector<ProfileTimestamp> generate(knights::Pos start, knights::Pos end);

    }

}

#endif
#pragma once

#ifndef _PROFILE_H
#define _PROFILE_H

#include <vector>
#include "knights/robot/drivetrain.hpp"
#include "knights/util/position.hpp"

namespace knights {

    struct QuinticPath {
        knights::Pos curr, target, curr_tangent, target_tangent;
        float curr_acceleration, target_acceleration;

        QuinticPath(knights::Pos curr, knights::Pos target,
            knights::Pos curr_tangent, knights::Pos target_tangent,
            float curr_acceleration, float target_acceleration);
        
        QuinticPath();

        float p00;
        float p01;
        float p02;
        float p03;
        float p04;
        float p05;

        float p10;
        float p11;
        float p12;
        float p13;
        float p14;
        float p15;

        knights::Pos position(float t);

        knights::Pos derivatives(float t);

        knights::Pos second_derivatives(float t);
    };

    struct ProfileTimestamp {
        knights::Pos position;
        float linear_velocity;
        float angular_velocity;
        float time;
        float right_speed;
        float left_speed;

        ProfileTimestamp(knights::Pos position, float linear_velocity, float angular_velocity, float time, float right_speed, float left_speed);
    };

    struct MotionProfile {
        std::vector<ProfileTimestamp> timestamps;
        QuinticPath path;

        // initial conditions
        float max_accel;
        float max_velocity;
        float desired_voltage;

        MotionProfile(std::vector<ProfileTimestamp> timestamps, QuinticPath path, float max_accel, float max_velocity, float desired_voltage);
    };

    class ProfileGenerator {
        public:
            // assumed differential drive
            float max_acceleration;
            float max_velocity;
            float track_width;

            ProfileGenerator(knights::Drivetrain drivetrain, float max_accel);

            ProfileGenerator(float max_velocity, float track_width, float max_acceleration);

            MotionProfile generate(knights::Pos start, knights::Pos end, float desired_voltage, int points_per_sec = 30, bool forwards = true, float curr_accel = 0, float target_accel = 0);
    };

    ProfileTimestamp lerp(const ProfileTimestamp &t1, const ProfileTimestamp &t2, float t);
}

#endif
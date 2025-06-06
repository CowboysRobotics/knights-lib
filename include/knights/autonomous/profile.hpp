#pragma once

#ifndef _PROFILE_H
#define _PROFILE_H

#include <map>
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

        std::vector<std::pair<float, float>> length_map;

        knights::Pos p0, p1, p2, p3, p4, p5;

        knights::Pos position(float t);

        knights::Pos derivatives(float t);

        knights::Pos second_derivatives(float t);

        float get_length();

        void generate_length_map(float samples);
        
        float get_t_from_dist(float dist);
    };

    struct ProfileTimestamp {
        knights::Pos position;
        float linear_velocity;
        float angular_velocity;
        float time;
        float right_speed;
        float left_speed;
        float acceleration;

        ProfileTimestamp(knights::Pos position, float linear_velocity, float angular_velocity, float time, float right_speed, float left_speed, float acceleration);
    };

    struct MotionProfile {
        std::vector<ProfileTimestamp> timestamps;
        QuinticPath path;

        // initial conditions
        float max_accel;
        float max_velocity;
        float cruise_pct;

        MotionProfile(std::vector<ProfileTimestamp> timestamps, QuinticPath path, float max_accel, float max_velocity, float cruise_pct);
    
        void dump();
    };

    class ProfileGenerator {
        public:
            // assumed differential drive
            float max_acceleration;
            float max_velocity;
            float track_width;

            ProfileGenerator(knights::Drivetrain drivetrain, float max_accel);

            ProfileGenerator(float max_velocity, float track_width, float max_acceleration);

            MotionProfile generate(knights::Pos start, knights::Pos end, float cruise_pct, int points_per_sec = 30, bool forwards = true, float curr_accel = 0, float target_accel = 0);
    };

    ProfileTimestamp lerp(const ProfileTimestamp &t1, const ProfileTimestamp &t2, float t);
}

#endif
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

        /**
         * @brief Get position for desired t value
         * 
         * @param t 
         * @return knights::Pos 
         */
        knights::Pos position(float t);

        /**
         * @brief Get dx and dy for desired t value
         * 
         * @param t 
         * @return knights::Pos 
         */
        knights::Pos derivatives(float t);

        /**
         * @brief get second derivative of path for desired t value
         * 
         * @param t 
         * @return knights::Pos 
         */
        knights::Pos second_derivatives(float t);

        /**
         * @brief Get the length of the path
         * 
         * @return float 
         */
        float get_length();

        /**
         * @brief Generate a length map that relates t to length of the path. MUST BE RAN BEFORE RUNNING GET LENGTH
         * 
         * @param samples 
         */
        void generate_length_map(float samples);
        
        /**
         * @brief Get the t value from the current distance along the path
         * 
         * @param dist 
         * @return float 
         */
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

        /**
         * @brief Construct a new Profile Timestamp object
         * 
         * @param position
         * @param linear_velocity
         * @param angular_velocity
         * @param time 
         * @param right_speed 
         * @param left_speed 
         * @param acceleration 
         */
        ProfileTimestamp(knights::Pos position, float linear_velocity, float angular_velocity, float time, float right_speed, float left_speed, float acceleration);
    };

    struct MotionProfile {
        std::vector<ProfileTimestamp> timestamps;
        QuinticPath path;

        // initial conditions
        float max_accel;
        float max_velocity;
        float cruise_pct;

        /**
         * @brief Construct a new Motion Profile object
         * 
         * @param timestamps 
         * @param path 
         * @param max_accel 
         * @param max_velocity 
         * @param cruise_pct from 0-100
         */
        MotionProfile(std::vector<ProfileTimestamp> timestamps, QuinticPath path, float max_accel, float max_velocity, float cruise_pct);
    
        /**
         * @brief Dump the Motion Profile to an SD card to be analyzed
         * 
         */
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

            /**
             * @brief Generate a motion profile with the following parameters
             * 
             * @param start 
             * @param end 
             * @param cruise_pct from 0-100, is then translated into RPM (ex. 80 on a 600 rpm drive would result in a target of 480 rpm)
             * @param points_per_sec Amount of timestamps to generate per second that the robot needs to drive
             * @param forwards 
             * @param curr_accel Current acceleration of the robot, usually set to 0
             * @param target_accel Target final acceleration of the robot, usually set to 0
             * @return MotionProfile 
             */
            MotionProfile generate(knights::Pos start, knights::Pos end, float cruise_pct, int points_per_sec = 30, bool forwards = true, float curr_accel = 0, float target_accel = 0);
    };

    /**
     * @brief Interpolate between two profile timestamps
     * 
     * @param t1 start timestamp
     * @param t2 end timestamp
     * @param t Alpha value, the percent to lerp to
     * @return ProfileTimestamp 
     */
    ProfileTimestamp lerp(const ProfileTimestamp &t1, const ProfileTimestamp &t2, float t);
}

#endif
#include "globals.h"
#include "knights/autonomous/path.hpp"
#include "knights/autonomous/profile.hpp"
#include "knights/util/calculation.hpp"
#include "knights/util/position.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <iostream>
#include <vector>

knights::QuinticPath::QuinticPath(knights::Pos curr, knights::Pos target, knights::Pos curr_tangent, knights::Pos target_tangent, 
    float curr_acceleration, float target_acceleration) : 
    curr(curr), target(target), curr_tangent(curr_tangent), target_tangent(target_tangent), 
    curr_acceleration(curr_acceleration), target_acceleration(target_acceleration) 
{

    this->p0 = curr;
    this->p5 = target;
    this->p1 = p0 + (curr_tangent * (1.0/5.0));
    this->p2 = (this->curr_acceleration / 20) + ((2 * p1) - p0);
    this->p4 = p5 - target_tangent * (1.0/5.0);
    this->p3 = this->target_acceleration / 20 + 2 * this->p4 - this->p5;
};

knights::QuinticPath::QuinticPath() {};

knights::MotionProfile::MotionProfile(std::vector<ProfileTimestamp> timestamps, QuinticPath path, float max_accel, float max_velocity, float cruise_pct) :
    timestamps(timestamps), path(path), max_accel(max_accel), max_velocity(max_velocity), cruise_pct(cruise_pct) {}

knights::ProfileTimestamp::ProfileTimestamp(knights::Pos position, float linear_velocity, float angular_velocity, 
    float time, float right_speed, float left_speed, float acceleration) : position(position), linear_velocity(linear_velocity), angular_velocity(angular_velocity),
    time(time), right_speed(right_speed), left_speed(left_speed), acceleration(acceleration) {};

knights::ProfileGenerator::ProfileGenerator(knights::Drivetrain drivetrain, float max_acceleration) :
    max_velocity(drivetrain.max_velocity()), max_acceleration(max_acceleration), track_width(drivetrain.track_width) {}

knights::ProfileGenerator::ProfileGenerator(float max_velocity, float track_width, float max_acceleration) :
    max_velocity(max_velocity), max_acceleration(max_acceleration), track_width(track_width) {}

knights::Pos knights::QuinticPath::position(float t) {
    return p0 * pow((1 - t), 5) + p1 * 5 * pow((1 - t), 4) * t +
               p2 * 10 * pow((1 - t), 3) * pow(t, 2) + p3 * 10 * pow((1 - t), 2) * pow(t, 3) +
               p4 * 5 * (1 - t) * pow(t, 4) + p5 * pow(t, 5);
}

knights::Pos knights::QuinticPath::derivatives(float t) {
    return p1 * 5 * pow((1 - t), 4) - p0 * 5 * pow((1 - t), 4) +
            p2 * 20 * pow((1 - t), 3) * t - p1 * 20 * pow((1 - t), 3) * t +
            p3 * 30 * pow((1 - t), 2) * pow(t, 2) - p2 * 30 * pow((1 - t), 2) * pow(t, 2) +
            p4 * 20 * (1 - t) * pow(t, 3) - p3 * 20 * (1 - t) * pow(t, 3) +
            p5 * 5 * pow(t, 4) - p4 * 5 * pow(t, 4);
}

knights::Pos knights::QuinticPath::second_derivatives(float t) {
    return 20 * (p2 - 2 * p1 + p0) * pow((1 - t), 3) +
                60 * (p3 - 2 * p2 + p1) * pow((1 - t), 2) * t +
                60 * (p4 - 2 * p3 + p2) * (1 - t) * pow(t, 2) +
                20 * (p5 - 2 * p4 + p3) * pow(t, 3);
}

float knights::QuinticPath::get_length() {
    if (!length_map.empty())
        return length_map.back().first;
    else {
        std::cerr << "Length Map not generated \n"; 
        return 0;
    }
}

void knights::QuinticPath::generate_length_map(float samples) {
    Pos curr;
    float total_dist = 0;
    for (float t = 0; t <= 1; t += 1/samples) {
        Pos p = this->position(t);
        total_dist += distance_btwn(curr, p);
        curr = p;
        this->length_map.push_back(std::make_pair(total_dist, t));
    }
    return;
}

float knights::QuinticPath::get_t_from_dist(float dist) {
    int low = 0;
    int high = this->length_map.size() - 1;

    std::cout << high << " " << this->length_map[high].first << "\n";

    if (dist > this->length_map[high].first) {
        return 1;
    } else if (dist < low) {
        return 0;
    }

    int possible_i = 0;

    // binary search
    while (low <= high) {
        int mid = (high + low)/2;
        if (this->length_map[mid].first < dist) {
            possible_i = mid;
            low = mid + 1;
        }
        else {
            high = mid - 1;
        }
    }

    if (possible_i >= this->length_map.size() - 2) {
        return this->length_map.back().second;
    }

    return knights::lerp(
        this->length_map[possible_i].second, this->length_map[possible_i + 1].second, 
        knights::clampf((this->length_map[possible_i + 1].first - this->length_map[possible_i].first) / this->length_map[possible_i].first, 0, 1)
    );
}

knights::MotionProfile knights::ProfileGenerator::generate(knights::Pos start, knights::Pos end, float cruise_pct, int points_per_sec, bool forwards, float curr_accel, float target_accel) {

    if (!forwards) {
        start.heading = knights::normalize_angle(start.heading + M_PI);
        end.heading = knights::normalize_angle(end.heading + M_PI);
    }

    float distance = distance_btwn(start, end);
    knights::Pos curr_tangent(std::cos(start.heading) * distance, std::sin(start.heading) * distance, 0);
    knights::Pos target_tangent(std::cos(end.heading) * distance, std::sin(end.heading) * distance, 0);

    QuinticPath path(start, end, curr_tangent, target_tangent, curr_accel, target_accel);

    path.generate_length_map(200);

    float total_dist = path.get_length();

    float path_max_velocity = (this->max_velocity) * (fabs(cruise_pct) / 100);

    float acceleration_time = path_max_velocity / this->max_acceleration;

    float halfway_distance = total_dist / 2;
    float acceleration_distance = 0.5 * this->max_acceleration * acceleration_time * acceleration_time;

    if (acceleration_distance > halfway_distance) {
        acceleration_time = std::sqrt(halfway_distance / (0.5 * this->max_acceleration));
    }

    float cruise_time = 0;
    float total_time = 2 * acceleration_time;

    if (acceleration_distance <= halfway_distance) {
        cruise_time = (total_dist / path_max_velocity) - acceleration_time;
        total_time = cruise_time + 2 * acceleration_time;
    }

    float deceleration_time = acceleration_time;
    float deceleration_distance = 0.5 * this->max_acceleration * deceleration_time * deceleration_time;
    float cruise_distance = path_max_velocity * cruise_time;

    bool is_triangular_profile = (acceleration_distance > halfway_distance);

    std::vector<ProfileTimestamp> timestamps;
    float x = 0, y = 0;

    for (double elapsed_time = 0; elapsed_time <= total_time; elapsed_time += 1.0/points_per_sec) {
        float curr_velocity = 0;
        float curr_dist = 0;

        float min_velocity = 8; // for static friction

        float acceleration = 0;

        if (elapsed_time > total_time) {
            curr_dist = total_dist;
            curr_velocity = 0;
        } else if (elapsed_time < acceleration_time) {

            curr_dist = 0.5 * this->max_acceleration * elapsed_time * elapsed_time;
            curr_velocity = std::fmax(this->max_acceleration * elapsed_time, min_velocity);
            acceleration = this->max_acceleration;
        } else if (!is_triangular_profile && elapsed_time < (acceleration_time + cruise_time)) {

            float cruise_current_time = elapsed_time - acceleration_time;
            curr_dist = acceleration_distance + path_max_velocity * cruise_current_time;
            curr_velocity = path_max_velocity;
        } else {
            float distance_at_decel_start;
            float velocity_at_decel_start;
            float time_at_decel_start;

            acceleration = -this->max_acceleration;

            if (is_triangular_profile) {
                distance_at_decel_start = halfway_distance;
                velocity_at_decel_start = this->max_acceleration * acceleration_time;
                time_at_decel_start = acceleration_time;
            } else {

                distance_at_decel_start = acceleration_distance + cruise_distance; 
                velocity_at_decel_start = path_max_velocity;
                time_at_decel_start = acceleration_time + cruise_time; 
            }

            float deceleration_elapsed_time = elapsed_time - time_at_decel_start;

            curr_dist = distance_at_decel_start +
                        velocity_at_decel_start * deceleration_elapsed_time -
                        0.5 * this->max_acceleration * deceleration_elapsed_time * deceleration_elapsed_time;

            curr_velocity = velocity_at_decel_start - this->max_acceleration * deceleration_elapsed_time;
        }

        float path_pct = path.get_t_from_dist(curr_dist);

        path_pct = knights::clampf(path_pct, 0, 1);

        Pos pt = path.position(path_pct);
        Pos deriv = path.derivatives(path_pct);
        Pos deriv2 = path.second_derivatives(path_pct);

        float theta = std::atan2(deriv.y, deriv.x); 

        pt.heading = theta;

        float curvature = (deriv2.y * deriv.x - deriv.y * deriv2.x) / std::pow(std::sqrt(deriv.x * deriv.x + deriv.y * deriv.y), 3);

        // velo curving
        float radius = std::fabs(1 / curvature);
        curr_velocity *= radius / (radius + this->track_width/2);

        float right_speed = curr_velocity + (curr_velocity * curvature) * track_width/2;
        float left_speed = curr_velocity - (curr_velocity * curvature) * track_width/2;


        if (!forwards) {
            curr_velocity *= -1;
            curvature *= -1;

            float tmp = right_speed;
            right_speed = left_speed;
            left_speed = tmp;

            pt.heading = knights::normalize_angle(pt.heading + M_PI);
        }

        timestamps.emplace_back(pt, curr_velocity, curr_velocity * curvature, elapsed_time, right_speed, left_speed, acceleration);

    }

    // second pass to make times of each point more accurates
    float constrained_time = 0;
    float prev_vel = 1e10;
    knights::Pos prev_position = timestamps[0].position;

    for (int i = 0; i < timestamps.size(); i++) {
        if (prev_vel < 1)
            prev_vel = 1;

        constrained_time += 
            std::sqrt(
                std::pow(timestamps[i].position.x - prev_position.x, 2) +
                std::pow(timestamps[i].position.y - prev_position.y, 2)
            ) / (prev_vel);
        
        prev_vel = timestamps[i].linear_velocity;
        prev_position = timestamps[i].position;

        timestamps[i].time = constrained_time;
    }

    return MotionProfile(timestamps, path, this->max_acceleration, path_max_velocity, cruise_pct);
}

void knights::MotionProfile::dump() {
    std::fstream write_file("/usd/motion_output.txt", std::ios_base::out);
	for (knights::ProfileTimestamp timestamp : this->timestamps) {
		write_file << "time: " << timestamp.time << " ";
		write_file << "pos: " << timestamp.position.x << " " << timestamp.position.y << " " << timestamp.position.heading << " ";
		write_file << "lin vel: " << timestamp.linear_velocity << " ";
		write_file << "angular vel: " << timestamp.angular_velocity << " ";
		write_file << "side vels (r,l): " << timestamp.right_speed << " " << timestamp.left_speed << " ";
		write_file << "\n";
	}
	write_file.close();
}

knights::ProfileTimestamp knights::lerp(const knights::ProfileTimestamp &t1, const knights::ProfileTimestamp &t2, float t) {
    return knights::ProfileTimestamp(
        knights::lerp(t1.position, t2.position, t),
        t1.linear_velocity + t * (t2.linear_velocity - t1.linear_velocity),
        t1.angular_velocity + t * (t2.angular_velocity - t1.angular_velocity),
        t1.time + t * (t2.time - t1.time),
        t1.right_speed + t * (t2.right_speed - t1.right_speed),
        t1.left_speed + t * (t2.left_speed - t1.left_speed),
        t1.acceleration + t * (t2.acceleration - t1.acceleration)
    );
}
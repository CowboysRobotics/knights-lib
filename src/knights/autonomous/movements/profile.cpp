#include "knights/autonomous/path.hpp"
#include "knights/autonomous/profile.hpp"
#include "knights/util/calculation.hpp"
#include "knights/util/position.hpp"
#include "knights/util/timer.hpp"
#include <cmath>
#include <vector>

// Hermite basis functions
#define H1(t) (2 * (t) * (t) * (t) - 3 * (t) * (t) + 1)
#define H2(t) (-2 * (t) * (t) * (t) + 3 * (t) * (t))
#define H3(t) ((t) * (t) * (t) - 2 * (t) * (t) + (t))
#define H4(t) ((t) * (t) * (t) - (t) * (t))

// First derivatives
#define H1_PRIME(t) (6 * (t) * (t) - 6 * (t))
#define H2_PRIME(t) (-6 * (t) * (t) + 6 * (t))
#define H3_PRIME(t) (3 * (t) * (t) - 4 * (t) + 1)
#define H4_PRIME(t) (3 * (t) * (t) - 2 * (t))

// Second derivatives
#define H1_PRIME2(t) (12 * (t) - 6)
#define H2_PRIME2(t) (-12 * (t) + 6)
#define H3_PRIME2(t) (6 * (t) - 4)
#define H4_PRIME2(t) (6 * (t) - 2)

knights::HermiteSpline::HermiteSpline(knights::Point curr, knights::Point target, knights::Point curr_tangent, knights::Point target_tangent)
 : curr(curr), target(target), curr_tangent(curr_tangent), target_tangent(target_tangent) {};

knights::Pos knights::HermiteSpline::position(double t) {
    double x = H1(t) * curr.x +
            H2(t) * target.x +
            H3(t) * curr_tangent.x +
            H4(t) * target_tangent.x;

    double y = H1(t) * curr.y +
            H2(t) * target.y +
            H3(t) * curr_tangent.y +
            H4(t) * target_tangent.y;

    knights::Pos deriv = derivatives(t);
    double theta = std::atan2(deriv.y, deriv.x);

    return knights::Pos(x, y, theta);
}

knights::Pos knights::HermiteSpline::derivatives(double t) {
    double dx = H1_PRIME(t) * curr.x +
                H2_PRIME(t) * target.x +
                H3_PRIME(t) * curr_tangent.x +
                H4_PRIME(t) * target_tangent.x;

    double dy = H1_PRIME(t) * curr.y +
                H2_PRIME(t) * target.y +
                H3_PRIME(t) * curr_tangent.y +
                H4_PRIME(t) * target_tangent.y;

    knights::Point deriv2 = second_derivatives(t);
    double omega = (deriv2.y * dx - dy * deriv2.x) / (1 + std::pow(dy / dx, 2));

    return knights::Pos(dx, dy, omega);
}

knights::Point knights::HermiteSpline::second_derivatives(double t) {
    double dx2 = H1_PRIME2(t) * curr.x +
                H2_PRIME2(t) * target.x +
                H3_PRIME2(t) * curr_tangent.x +
                H4_PRIME2(t) * target_tangent.x;

    double dy2 = H1_PRIME2(t) * curr.y +
                H2_PRIME2(t) * target.y +
                H3_PRIME2(t) * curr_tangent.y +
                H4_PRIME2(t) * target_tangent.y;

    return knights::Point(dx2, dy2);
}

knights::ProfileTimestamp::ProfileTimestamp(knights::Pos position, float linear_velocity, float angular_velocity, float curr_distance, 
    float time, float right_speed, float left_speed) : position(position), linear_velocity(linear_velocity), angular_velocity(angular_velocity),
    curr_distance(curr_distance), time(time), right_speed(right_speed), left_speed(left_speed) {};

knights::ProfileGenerator::ProfileGenerator(knights::Drivetrain drivetrain, float max_accel) :
    max_velocity(drivetrain.max_velocity()), max_accel(max_accel), track_width(drivetrain.track_width) {}

knights::ProfileGenerator::ProfileGenerator(float max_velocity, float track_width, float max_accel) :
    max_velocity(max_velocity), max_accel(max_accel), track_width(track_width) {}

std::vector<knights::ProfileTimestamp> knights::ProfileGenerator::generate(knights::Pos start, knights::Pos end) {
    std::cout << this->max_velocity << " " << this->max_accel << " " << this->track_width << "\n";

    // first, generate the path
    float dist = distance_btwn(start, end); // * 2

    knights::HermiteSpline path(
        knights::Point(start.x, start.y),
        knights::Point(end.x, end.y),
        knights::Point(std::cos(start.heading) * dist, std::sin(start.heading) * dist),
        knights::Point(std::cos(end.heading) * dist, std::sin(end.heading) * dist)
    );

    // get total distance
    std::vector<float> path_t = knights::linspace(0, 1, 300);
    float total_dist = 0;

    for (int i = 1; i < path_t.size(); i++) {
        total_dist += distance_btwn(path.position(path_t[i]), path.position(path_t[i-1]));
    }

    // start with necessary values for motion profile
    float accel_time = max_velocity / max_accel;
    float halfway_dist = total_dist / 2;
    float accel_dist = 0.5 * max_accel * (accel_time * accel_time);

    if (accel_dist > halfway_dist) {
        accel_time = sqrt(halfway_dist / (0.5 * max_accel));
    }

    float max_velocity = max_accel * accel_time;

    float cruise_dist = total_dist - 2 * accel_dist;
    float cruise_time = cruise_dist / max_velocity;

    double deaccel_time = accel_time + cruise_time;
    double entire_time = 2*accel_time + cruise_time;
    
    std::vector<float> times_t = knights::linspace(0, entire_time, 300);

    std::vector<knights::ProfileTimestamp> output;

    for (float elapsed_time : times_t) {

        // Calculate distance
        float curr_dist = 0;
        if (elapsed_time > entire_time) {
            curr_dist = total_dist;
        } else if (elapsed_time < accel_time) {
            curr_dist = 0.5 * max_accel * (elapsed_time * elapsed_time);
        } else if (elapsed_time < deaccel_time) {
            accel_dist = 0.5 * max_accel * (accel_time * accel_time);
            curr_dist = accel_dist + max_velocity * (elapsed_time - accel_time);
        } else {
            accel_dist = 0.5 * max_accel * (accel_time * accel_time);
            cruise_dist = max_velocity * cruise_time;
            float time_since_deaccel = elapsed_time - deaccel_time;
            curr_dist = accel_dist + cruise_dist + 
                max_velocity * time_since_deaccel 
                - 0.5 * max_accel * time_since_deaccel * time_since_deaccel;
        }

        // Calculate velocity
        float velocity = 0;
        if (elapsed_time < accel_time) {
            velocity = knights::lerp(0, max_velocity, elapsed_time / accel_time);
        } else if (elapsed_time < accel_time + cruise_time && accel_dist > accel_time) {
            velocity = max_velocity;
        } else {
            velocity = lerp(max_velocity, 0, (elapsed_time - (accel_time + cruise_time)) / accel_time);
        }

        // Calculate Angular Velocity and Position
        Pos curr = path.position(curr_dist/total_dist);
        Pos deriv_curr = path.derivatives(curr_dist/total_dist);

        float left_vel = velocity - (deriv_curr.heading * track_width/2.0);
        float right_vel = velocity + (deriv_curr.heading * track_width/2.0);

        output.emplace_back(curr, velocity, deriv_curr.heading, curr_dist, elapsed_time, right_vel, left_vel);
    }

    return output;
};
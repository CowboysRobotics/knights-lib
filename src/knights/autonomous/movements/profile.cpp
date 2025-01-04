#include "knights/autonomous/path.h"
#include "knights/autonomous/profile.h"
#include "knights/util/calculation.h"
#include "knights/util/timer.h"

// MIGHT NOT BE USEFUL

knights::ProfileTimestamp::ProfileTimestamp(knights::Pos position, float expected_velocity, float time,
    float right_speed, float left_speed) :
    position(position), expected_velocity(expected_velocity), time(time), right_speed(right_speed), left_speed(left_speed) {}

knights::ProfileGenerator::ProfileGenerator(knights::Drivetrain* drivetrain) :
    drivetrain(drivetrain) {}

knights::ProfileGenerator::ProfileGenerator(float rpm, float wheel_diameter) :
    rpm(rpm), wheel_diameter(wheel_diameter) {}


// NOT CURRENTLY OPERATIONAL
std::vector<knights::ProfileTimestamp> knights::ProfileGenerator::generate_profile(knights::Route route, knights::Pos start, knights::Drivetrain* drivetrain, 
    float lookahead, float speed_max, float speed_min, float interval) {

    std::vector<knights::ProfileTimestamp> output;

    int time = 0;

    knights::Pos curr = start;

    int i = 0;
    knights::Pos target = route.positions[i];

    while (distance_btwn(curr, route.positions[route.positions.size()-1]) > 10.0) { // TODO: make new var for end tolerance
        // route

        while (knights::distance_btwn(curr, route.positions[i]) < lookahead && i < route.positions.size()-1) {
            i++;
            target = route.positions[i];
        }

        float speed_curvature = 0.001;
        if (i < route.positions.size() - 2) {
            speed_curvature += knights::curvature(curr, target, route.positions[i + 1]);
        }

        float curr_speed = std::max(std::min(3/speed_curvature, speed_max), speed_min);

        float angular_curvature = knights::curvature(curr, target);

        float r_speed = curr_speed * (2 - angular_curvature * drivetrain->track_width) / 2;
        float l_speed = curr_speed * (2 + angular_curvature * drivetrain->track_width) / 2;

        // normalize on ratio - maybe remove
        float max_curr_speed = std::fmax(fabs(r_speed), fabs(l_speed)) / 127.0;
        if (max_curr_speed > 1) {
            r_speed /= max_curr_speed;
            l_speed /= max_curr_speed;
        }

        output.emplace_back(curr, curr_speed, time, r_speed, l_speed);

        float linear_speed = ((r_speed + l_speed) / 2) * ((drivetrain->rpm * drivetrain->wheel_diameter / 60000) / 127.0); // TODO: convert to inches per millisecond 
        float angular_omega = (r_speed - l_speed) * ((drivetrain->rpm * drivetrain->wheel_diameter / 60000) / 127.0) / drivetrain->track_width;

        curr.x += linear_speed * cosf(curr.heading) * interval; // interval in ms
        curr.y += linear_speed * sinf(curr.heading) * interval; // interval in ms
        curr.heading += angular_omega * interval;

        curr.heading = normalize_angle(curr.heading);

        time += interval;
    }

    return output;
}
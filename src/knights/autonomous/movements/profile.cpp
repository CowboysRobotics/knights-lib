#include "knights/autonomous/path.h"
#include "knights/autonomous/profile.h"
#include "knights/util/timer.h"

knights::ProfileTimestamp::ProfileTimestamp(knights::Pos position, float expected_velocity, float time,
    float right_speed, float left_speed) :
    position(position), expected_velocity(expected_velocity), time(time), right_speed(right_speed), left_speed(left_speed) {}

knights::ProfileGenerator::ProfileGenerator(knights::Drivetrain* drivetrain) :
    drivetrain(drivetrain) {}

knights::ProfileGenerator::ProfileGenerator(float rpm, float wheel_diameter) :
    rpm(rpm), wheel_diameter(wheel_diameter) {}

std::vector<knights::ProfileTimestamp> knights::ProfileGenerator::generate_profile(knights::Route route, knights::Pos start) {
    std::vector<knights::ProfileTimestamp> output;

    knights::Timer timer;

    knights::Pos curr = start;

    while (distance_btwn(curr, route.positions[route.positions.size()-1])) {
        // route
    }
}
#include "knights/autonomous/path.hpp"
#include "knights/autonomous/profile.hpp"
#include "knights/util/calculation.hpp"
#include "knights/util/position.hpp"
#include "knights/util/timer.hpp"
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
    // first, generate the path
    float dist = 2 * distance_btwn(start, end);

    knights::HermiteSpline path(
        knights::Point(start.x, start.y),
        knights::Point(end.x, end.y),
        knights::Point(std::cos(start.heading) * dist, std::sin(start.heading) * dist),
        knights::Point(std::cos(end.heading) * dist, std::sin(end.heading) * dist)
    );

    // get total distance
    std::vector<float> t = knights::linspace(0, 1, 300);
    float total_dist = 0;

    for (int i = 1; i < t.size(); i++) {
        total_dist += distance_btwn(path.position(t[i]), path.position(t[i-1]));
    }
};
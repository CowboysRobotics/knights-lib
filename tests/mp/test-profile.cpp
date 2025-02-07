#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

// Define constants
const double PI = 3.14159265358979323846;

struct QuinticPath {
    std::vector<double> curr, target, curr_tangent, target_tangent;
    double curr_acceleration, target_acceleration;

    QuinticPath(std::vector<double> curr, std::vector<double> target,
                std::vector<double> curr_tangent, std::vector<double> target_tangent,
                double curr_acceleration, double target_acceleration) :
        curr(curr), target(target), curr_tangent(curr_tangent), target_tangent(target_tangent),
        curr_acceleration(curr_acceleration), target_acceleration(target_acceleration) {}

    double p00() { return curr[0]; }
    double p01() { return p00() + curr_tangent[0] / 5; }
    double p02() { return curr_acceleration / 20 + 2 * p01() - p00(); }
    double p03() { return target_acceleration / 20 + 2 * p04() - p05(); }
    double p04() { return p05() - target_tangent[0] / 5; }
    double p05() { return target[0]; }

    double p10() { return curr[1]; }
    double p11() { return p10() + curr_tangent[1] / 5; }
    double p12() { return curr_acceleration / 20 + 2 * p11() - p10(); }
    double p13() { return target_acceleration / 20 + 2 * p14() - p15(); }
    double p14() { return p15() - target_tangent[1] / 5; }
    double p15() { return target[1]; }

    std::vector<double> position(double t) {
        double x = p00() * pow((1 - t), 5) + p01() * 5 * pow((1 - t), 4) * t +
                   p02() * 10 * pow((1 - t), 3) * pow(t, 2) + p03() * 10 * pow((1 - t), 2) * pow(t, 3) +
                   p04() * 5 * (1 - t) * pow(t, 4) + p05() * pow(t, 5);

        double y = p10() * pow((1 - t), 5) + p11() * 5 * pow((1 - t), 4) * t +
                   p12() * 10 * pow((1 - t), 3) * pow(t, 2) + p13() * 10 * pow((1 - t), 2) * pow(t, 3) +
                   p14() * 5 * (1 - t) * pow(t, 4) + p15() * pow(t, 5);

        return {x, y};
    }

    std::vector<double> derivatives(double t) {
        double dx = p01() * 5 * pow((1 - t), 4) - p00() * 5 * pow((1 - t), 4) +
                    p02() * 20 * pow((1 - t), 3) * t - p01() * 20 * pow((1 - t), 3) * t +
                    p03() * 30 * pow((1 - t), 2) * pow(t, 2) - p02() * 30 * pow((1 - t), 2) * pow(t, 2) +
                    p04() * 20 * (1 - t) * pow(t, 3) - p03() * 20 * (1 - t) * pow(t, 3) +
                    p05() * 5 * pow(t, 4) - p04() * 5 * pow(t, 4);

        double dy = p11() * 5 * pow((1 - t), 4) - p10() * 5 * pow((1 - t), 4) +
                    p12() * 20 * pow((1 - t), 3) * t - p11() * 20 * pow((1 - t), 3) * t +
                    p13() * 30 * pow((1 - t), 2) * pow(t, 2) - p12() * 30 * pow((1 - t), 2) * pow(t, 2) +
                    p14() * 20 * (1 - t) * pow(t, 3) - p13() * 20 * (1 - t) * pow(t, 3) +
                    p15() * 5 * pow(t, 4) - p14() * 5 * pow(t, 4);

        return {dx, dy};
    }

    std::vector<double> second_derivatives(double t) {
        double dx2 = 20 * (p02() - 2 * p01() + p00()) * pow((1 - t), 3) +
                     60 * (p03() - 2 * p02() + p01()) * pow((1 - t), 2) * t +
                     60 * (p04() - 2 * p03() + p02()) * (1 - t) * pow(t, 2) +
                     20 * (p05() - 2 * p04() + p03()) * pow(t, 3);

        double dy2 = 20 * (p12() - 2 * p11() + p10()) * pow((1 - t), 3) +
                     60 * (p13() - 2 * p12() + p11()) * pow((1 - t), 2) * t +
                     60 * (p14() - 2 * p13() + p12()) * (1 - t) * pow(t, 2) +
                     20 * (p15() - 2 * p14() + p13()) * pow(t, 3);

        return {dx2, dy2};
    }
};

double dist_between(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

double lerp(double start, double end, double step) {
    return start + (end - start) * step;
}

std::vector<std::vector<double>> generate_motion_profile(double max_acceleration, double max_velocity, double distance, double track_width, QuinticPath& path) {
    // Calculate the time it takes to accelerate to max velocity
    double acceleration_time = max_velocity / max_acceleration;

    double halfway_distance = distance / 2;
    double acceleration_distance = 0.5 * max_acceleration * acceleration_time * acceleration_time;

    if (acceleration_distance > halfway_distance) {
        acceleration_time = std::sqrt(halfway_distance / (0.5 * max_acceleration));
    }

    double cruise_time = 0;
    double total_time = 2 * acceleration_time;

    if (acceleration_distance <= halfway_distance) {
        cruise_time = (distance / max_velocity) - acceleration_time;
        total_time = cruise_time + 2 * acceleration_time;
    }

    double deceleration_time = acceleration_time;
    double deceleration_distance = 0.5 * max_acceleration * deceleration_time * deceleration_time;
    double cruise_distance = max_velocity * cruise_time;
    double total_dist = distance;

    int n_t_values = static_cast<int>(std::round(total_time * 500));
    std::vector<double> t(n_t_values);
    for (int i = 0; i < n_t_values; ++i) {
        t[i] = i * total_time / (n_t_values - 1);
    }

    std::vector<double> dist_arr, vel_arr, omega_arr, side_vel_arr_left, side_vel_arr_right;
    double x = 0, y = 0;

    for (double elapsed_time : t) {
        double curr_dist = 0;
        double curr_velocity = 0;

        if (elapsed_time > total_time) {
            curr_dist = distance;
            curr_velocity = 0;
        } else if (elapsed_time < acceleration_time) {
            curr_dist = 0.5 * max_acceleration * elapsed_time * elapsed_time;
            curr_velocity = max_acceleration * elapsed_time;
        } else if (cruise_time > 0 && elapsed_time < (acceleration_time + cruise_time)) {
            double cruise_current_time = elapsed_time - acceleration_time;
            curr_dist = acceleration_distance + max_velocity * cruise_current_time;
            curr_velocity = max_velocity;
        } else {
            double deceleration_curr_time = (elapsed_time - acceleration_time - cruise_time);
            curr_dist = acceleration_distance + cruise_distance + max_velocity * deceleration_curr_time - max_acceleration * (deceleration_curr_time * deceleration_curr_time) / 2;
            curr_velocity = max_velocity - max_acceleration * deceleration_curr_time;
        }

        auto [x, y] = path.position(elapsed_time / total_time);
        auto [dx, dy] = path.derivatives(elapsed_time / total_time);
        auto [dx2, dy2] = path.second_derivatives(elapsed_time / total_time);

        double theta = std::atan2(dy, dx) / total_time;
        double omega = ((dy2 * dx - dy * dx2) / ((dx * dx) * (1 + (dy / dx) * (dy / dx)))) / total_time;

        dist_arr.push_back(curr_dist);
        vel_arr.push_back(curr_velocity);
        omega_arr.push_back(omega);
        side_vel_arr_left.push_back(curr_velocity - (omega * track_width / 2));
        side_vel_arr_right.push_back(curr_velocity + (omega * track_width / 2));
    }

    return {dist_arr, vel_arr, omega_arr, side_vel_arr_left, side_vel_arr_right};
}

int main() {
    std::vector<double> curr = {0, 0, PI / 2, 0};
    std::vector<double> target = {72, 72, 0, 0};

    double dist = std::sqrt(std::pow(target[0] - curr[0], 2) + std::pow(target[1] - curr[1], 2));
    std::vector<double> curr_tangent = {std::cos(curr[2]) * dist, std::sin(curr[2]) * dist};
    std::vector<double> target_tangent = {std::cos(target[2]) * dist, std::sin(target[2]) * dist};

    double curr_acceleration = dist * curr[3];
    double target_acceleration = dist * target[3];

    QuinticPath path(curr, target, curr_tangent, target_tangent, curr_acceleration, target_acceleration);

    // Calculate max velocity
    const double DESIRED_VOLTAGE = 120;
    const double MAX_VOLTAGE = 127;
    const double WHEEL_DIAMETER = 2.75;
    const double RPM = 450;
    const double TRACK_WIDTH = 15;

    double max_acceleration = 178;
    double max_velocity = (DESIRED_VOLTAGE / MAX_VOLTAGE) * PI * WHEEL_DIAMETER * (RPM / 60.0);

    std::cout << "Max Velocity: " << max_velocity << ", Max Acceleration: " << max_acceleration << std::endl;

    double total_dist = dist;
    auto [dist_arr, vel_arr, omega_arr, side_vel_arr_left, side_vel_arr_right] = generate_motion_profile(max_acceleration, max_velocity, total_dist, TRACK_WIDTH, path);

    // Output or process your results here...

    return 0;
}
#include "knights/autonomous/path.hpp"
#include "knights/autonomous/profile.hpp"
#include "knights/util/calculation.hpp"
#include "knights/util/position.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <vector>

#define PROS_MAX_VOLTAGE 127

knights::QuinticPath::QuinticPath(knights::Pos curr, knights::Pos target, knights::Pos curr_tangent, knights::Pos target_tangent, 
    float curr_acceleration, float target_acceleration) : 
    curr(curr), target(target), curr_tangent(curr_tangent), target_tangent(target_tangent), 
    curr_acceleration(curr_acceleration), target_acceleration(target_acceleration) 
{
    this->p00 = this->curr.x;
    this->p01 = this->p00 + this->curr_tangent.x / 5;
    this->p02 = this->curr_acceleration / 20 + 2 * this->p01 - this->p00;
    this->p05 = this->target.x;
    this->p04 = this->p05 - this->target_tangent.x / 5;
    this->p03 = this->target_acceleration / 20 + 2 * this->p04 - this->p05;

    this->p10 = this->curr.y;
    this->p11 = this->p10 + this->curr_tangent.y / 5;
    this->p12 = this->curr_acceleration / 20 + 2 * this->p11 - this->p10;
    this->p15 = this->target.y;
    this->p14 = this->p15 - this->target_tangent.y / 5;
    this->p__13 = this->target_acceleration / 20 + 2 * this->p14 - this->p15;
};

knights::QuinticPath::QuinticPath() {};

knights::MotionProfile::MotionProfile(std::vector<ProfileTimestamp> timestamps, QuinticPath path, float max_accel, float max_velocity, float desired_voltage) :
    timestamps(timestamps), path(path), max_accel(max_accel), max_velocity(max_velocity), desired_voltage(desired_voltage) {}

knights::ProfileTimestamp::ProfileTimestamp(knights::Pos position, float linear_velocity, float angular_velocity, 
    float time, float right_speed, float left_speed) : position(position), linear_velocity(linear_velocity), angular_velocity(angular_velocity),
    time(time), right_speed(right_speed), left_speed(left_speed) {};

knights::ProfileGenerator::ProfileGenerator(knights::Drivetrain drivetrain, float max_acceleration) :
    max_velocity(drivetrain.max_velocity()), max_acceleration(max_acceleration), track_width(drivetrain.track_width) {}

knights::ProfileGenerator::ProfileGenerator(float max_velocity, float track_width, float max_acceleration) :
    max_velocity(max_velocity), max_acceleration(max_acceleration), track_width(track_width) {}

knights::Pos knights::QuinticPath::position(float t) {
    float x = p00 * pow((1 - t), 5) + p01 * 5 * pow((1 - t), 4) * t +
               p02 * 10 * pow((1 - t), 3) * pow(t, 2) + p03 * 10 * pow((1 - t), 2) * pow(t, 3) +
               p04 * 5 * (1 - t) * pow(t, 4) + p05 * pow(t, 5);

    float y = p10 * pow((1 - t), 5) + p11 * 5 * pow((1 - t), 4) * t +
               p12 * 10 * pow((1 - t), 3) * pow(t, 2) + p13 * 10 * pow((1 - t), 2) * pow(t, 3) +
               p14 * 5 * (1 - t) * pow(t, 4) + p15 * pow(t, 5);

    return Pos(x, y, 0);
}

knights::Pos knights::QuinticPath::derivatives(float t) {
    float dx = p01 * 5 * pow((1 - t), 4) - p00 * 5 * pow((1 - t), 4) +
                p02 * 20 * pow((1 - t), 3) * t - p01 * 20 * pow((1 - t), 3) * t +
                p03 * 30 * pow((1 - t), 2) * pow(t, 2) - p02 * 30 * pow((1 - t), 2) * pow(t, 2) +
                p04 * 20 * (1 - t) * pow(t, 3) - p03 * 20 * (1 - t) * pow(t, 3) +
                p05 * 5 * pow(t, 4) - p04 * 5 * pow(t, 4);

    float dy = p11 * 5 * pow((1 - t), 4) - p10 * 5 * pow((1 - t), 4) +
                p12 * 20 * pow((1 - t), 3) * t - p11 * 20 * pow((1 - t), 3) * t +
                p13 * 30 * pow((1 - t), 2) * pow(t, 2) - p12 * 30 * pow((1 - t), 2) * pow(t, 2) +
                p14 * 20 * (1 - t) * pow(t, 3) - p13 * 20 * (1 - t) * pow(t, 3) +
                p15 * 5 * pow(t, 4) - p14 * 5 * pow(t, 4);

    return Pos(dx, dy, 0);
}

knights::Pos knights::QuinticPath::second_derivatives(float t) {
    float dx2 = 20 * (p02 - 2 * p01 + p00) * pow((1 - t), 3) +
                 60 * (p03 - 2 * p02 + p01) * pow((1 - t), 2) * t +
                 60 * (p04 - 2 * p03 + p02) * (1 - t) * pow(t, 2) +
                 20 * (p05 - 2 * p04 + p03) * pow(t, 3);

    float dy2 = 20 * (p12 - 2 * p11 + p10) * pow((1 - t), 3) +
                 60 * (p13 - 2 * p12 + p11) * pow((1 - t), 2) * t +
                 60 * (p14 - 2 * p13 + p12) * (1 - t) * pow(t, 2) +
                 20 * (p15 - 2 * p14 + p13) * pow(t, 3);

    return Pos(dx2, dy2, 0);
}

#define ACCELERATION_CURVATURE_CONSTANT 3.5
#define VELOCITY_CURVATURE_CONSTANT 1.5

knights::MotionProfile knights::ProfileGenerator::generate(knights::Pos start, knights::Pos end, float desired_voltage, int points_per_sec, bool forwards, float curr_accel, float target_accel) {

    if (!forwards) {
        start.heading = knights::normalize_angle(start.heading + M_PI);
        end.heading = knights::normalize_angle(end.heading + M_PI);
    }

    float distance = distance_btwn(start, end);
    knights::Pos curr_tangent(std::cos(start.heading) * distance, std::sin(start.heading) * distance, 0);
    knights::Pos target_tangent(std::cos(end.heading) * distance, std::sin(end.heading) * distance, 0);

    QuinticPath path(start, end, curr_tangent, target_tangent, curr_accel, target_accel);

    Pos curr;
    float total_dist = 0;
    for (float val = 0; val <= 1; val += 1.0/points_per_sec) {
        Pos p = path.position(val);
        total_dist += distance_btwn(curr, p);
        curr = p;
    }

    float path_max_velocity = (this->max_velocity) * (fabs(desired_voltage) / PROS_MAX_VOLTAGE);

    // Calculate the time it takes to accelerate to max velocity
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

    // std::vector<float> t = knights::linspace(0, total_time, points_per_sec * total_time);

    std::fstream write_file("/usd/generation_output.txt", std::ios_base::out);

    std::vector<ProfileTimestamp> timestamps;
    float x = 0, y = 0;

    for (double elapsed_time = 0; elapsed_time <= total_time; elapsed_time += 1.0/points_per_sec) {
        float curr_velocity = 0;
        float curr_dist = 0;

        write_file << "stage 1 " << pros::micros() << "\n";

        if (elapsed_time > total_time) {
            curr_dist = total_dist;
            curr_velocity = 0;
        } else if (elapsed_time < acceleration_time) {
            curr_dist = 0.5 * this->max_acceleration * elapsed_time * elapsed_time;
            curr_velocity = this->max_acceleration * elapsed_time;
        } else if (cruise_time > 0 && elapsed_time < (acceleration_time + cruise_time)) {
            float cruise_current_time = elapsed_time - acceleration_time;
            curr_dist = acceleration_distance + path_max_velocity * cruise_current_time;
            curr_velocity = path_max_velocity;
        } else {
            float deceleration_curr_time = (elapsed_time - acceleration_time - cruise_time);
            curr_dist = acceleration_distance + cruise_distance + path_max_velocity * deceleration_curr_time - this->max_acceleration * (deceleration_curr_time * deceleration_curr_time) / 2;
            curr_velocity = path_max_velocity - this->max_acceleration * deceleration_curr_time;
        }

        float path_pct = curr_dist / total_dist;

        Pos pt = path.position(path_pct);
        Pos deriv = path.derivatives(path_pct);
        Pos deriv2 = path.second_derivatives(path_pct);
        float curr_acceleratin = max_acceleration;
        float max_speed = 1e4;

        write_file << "stage 3 " << pros::micros() << "\n";

        // #### END

        float theta = std::atan2(deriv.y, deriv.x);

        pt.heading = theta;

        float curvature = (deriv2.y * deriv.x - deriv.y * deriv2.x) /((deriv.x * deriv.x + deriv.y * deriv.y) * std::hypot(deriv.x, deriv.y));

        float right_speed = curr_velocity * (2 - curvature * track_width) / 2;
        float left_speed = curr_velocity * (2 + curvature * track_width) / 2;

        if (!forwards) {
            curr_velocity *= -1;
            curvature *= -1;

            float tmp = right_speed;
            right_speed = left_speed;
            left_speed = tmp;

            pt.heading = knights::normalize_angle(pt.heading + M_PI);
        }

        timestamps.emplace_back(pt, curr_velocity, curr_velocity * curvature, elapsed_time, right_speed, left_speed);

        write_file << "stage 4 " << pros::micros() << "\n";
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


    write_file.close();

    return MotionProfile(timestamps, path, this->max_acceleration, path_max_velocity, desired_voltage);
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
        t1.left_speed + t * (t2.left_speed - t1.left_speed)
    );
}
#include "knights/autonomous/controller.hpp"
#include "knights/autonomous/pid.hpp"
#include "knights/autonomous/path.hpp"

#include "knights/autonomous/profile.hpp"
#include "knights/robot/chassis.hpp"

#include "knights/util/calculation.hpp"
#include "knights/util/position.hpp"

#include "knights/logger/logger.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <math.h>
#include <fstream>

#define PROS_MAX_VOLTAGE 127
#define MOTOR_VOLTS 11.0f

#define SCALE_OMEGA 2

float knights::circle_intersection(knights::Pos nxt, knights::Pos prev, knights::Pos curr, float lookahead_distance) {
    knights::Pos dir = nxt - prev;
    knights::Pos fro = prev - curr;

    // get coefficents then calculate discriminant
    float a = dir * dir;
    float b = 2 * (fro * dir);
    float c = (fro * fro) - lookahead_distance * lookahead_distance;
    float discrim = b * b - 4 * a * c;

    // if there are valid solutions
    if (discrim > 0) {
        // calculate solutions
        discrim = sqrt(discrim);
        float s1 = (-b + discrim) / (2 * a);
        float s2 = (-b - discrim) / (2 * a);

        if (s1 >= 0 && s1 <= 1) // if solution 1 is valid, return it
            return s1;
        else if (s2 >= 0 && s2 <= 1) // if solution 2 is valid, return it
            return s2;
        else
            return -1;
    } else // no or one real solution
        return -1;
}

void knights::RobotController::follow_route_pursuit(const knights::Route &route, float lookahead_distance, const float max_speed, bool forwards, 
    float end_tolerance, float timeout, bool use_pid) {
    // make sure this is only movement running and route is valid
    if (this->in_motion || route.positions.size() < 2) return;
    this->in_motion = true;

    // follow a pure pursuit route  

    // make bot move backwards if lookahead is negative - shorthand
    if (lookahead_distance < 0) {
        forwards = false;
        lookahead_distance = fabs(lookahead_distance);
    }

    // use pid if end tolerance is negative - shorthand
    if (end_tolerance < 0) {
        end_tolerance = fabs(end_tolerance);
    }

    // make sure motors are on brake - prevent drift at end
    this->chassis->drivetrain->right_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    this->chassis->drivetrain->left_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // declare essential values
    knights::Pos target_point = route.positions[0];
    int closest_i = 0;
    int prev_closest_i = 0;
    float closest_dist = 1e5;
    float error = distance_btwn(this->chassis->curr_position, route.positions[route.positions.size()-1]);

    float curr_lookahead = lookahead_distance;
    float max_curr_speed = 127;
    float min_curr_speed = 0;
    float angular_curve;

    this->lateral_pid->reset();

    if (this->angular_pid != nullptr && use_pid) {
        this->angular_pid->reset();
    }

    std::fstream write_file("/usd/pure_pursuit.txt", std::ios_base::out);

    // While the robot has not reached the desired point and is not at the end of the route
    while (error > end_tolerance && closest_i != route.positions.size() - 1) {

        knights::Pos curr_position = this->chassis->curr_position;
        if (!forwards || lookahead_distance < 0) {
            curr_position.heading = knights::normalize_angle(curr_position.heading + M_PI);
        }

        // update error values
        error = distance_btwn(curr_position, route.positions[route.positions.size()-1]);

        closest_dist = 1e5;

        // find nearest point
        for (int i = closest_i; i < route.positions.size(); i++) {
            if (distance_btwn(curr_position,  route.positions[i]) < closest_dist) {
                closest_dist = distance_btwn(curr_position,  route.positions[i]);
                closest_i = i;
            }
        }

        int target_i = closest_i;

        // find lookahead point
        for (; target_i < route.positions.size(); target_i++) {
            if (target_i == route.positions.size() - 1) {
                knights::Pos extended(
                    route.positions.back().x + lookahead_distance * 1.5 * cos(route.positions.back().heading),
                    route.positions.back().y + lookahead_distance * 1.5 * sin(route.positions.back().heading),
                    route.positions.back().heading
                );

                float t = circle_intersection(extended, route.positions[target_i], curr_position, curr_lookahead);

                if (t != -1) {
                    target_point = lerp(route.positions[target_i], extended, t);
                }
            }
            else {
                float t = circle_intersection(route.positions[target_i+1], route.positions[target_i], curr_position, curr_lookahead);

                if (t != -1) {
                    target_point = lerp(route.positions[target_i], route.positions[target_i+1], t);
                }
            }
        }

        const float &effective_lookahead = distance_btwn(curr_position, target_point);

        // lookahead and speed scaling
        max_curr_speed= 127.0;
        if (target_point != route.positions[0] && (closest_i != 0 && closest_i != route.positions.size()-1)) { // make sure we have valid target_i variables, it won't be right if the robot is at the start of the route
            curr_lookahead = clampf(
                lookahead_distance * 
            (0.15/curvature(route.positions[closest_i-1], route.positions[closest_i], route.positions[closest_i+1])), // tuned formula dependent on curvature
            lookahead_distance, lookahead_distance*1.5); // limit lookahead from going too high or too low

            // also change target speed with this
            max_curr_speed = 7.0/curvature(route.positions[closest_i-1], route.positions[closest_i], route.positions[closest_i+1]);
        }
        
        // determine the speed and angular curvature to use for calculating ratio of motor velocities
        float pid_speed = this->lateral_pid->update(error);
        float target_speed = std::fmin(
            knights::clampf(
                pid_speed,
                this->lateral_pid->get_min_speed(), max_curr_speed
            ), max_speed
        );
        angular_curve = curvature(curr_position, target_point);

        // write_file << knights::logger::string_format("tspeed & pts: %lf , %lf %lf , %lf %lf , %lf %lf\n", 
        //     target_speed, route.positions[closest_i].x, route.positions[closest_i].y, route.positions[closest_i+1].x, route.positions[closest_i+1].y,
        //     route.positions[closest_i+2].x, route.positions[closest_i+2].y
        //     );

        float angular_velocity = 0;
        // curve to update angular velocity
        if (this->angular_pid != nullptr && closest_i != 0 && use_pid && 
            distance_btwn(curr_position, route.positions[closest_i]) < end_tolerance) // only use if closely following the path
        {
            angular_velocity = this->angular_pid->update(angular_error(curr_position.heading, route.positions[closest_i].heading, 0), true);
        }

        // calculate right and left speed based on curvature
        float r_speed = target_speed * (2 - angular_curve * this->chassis->drivetrain->track_width) / 2 + angular_velocity;
        float l_speed = target_speed * (2 + angular_curve * this->chassis->drivetrain->track_width) / 2 - angular_velocity;

        // calculate if one is over max alloted speed (might need to be 127.0 - max speed in pros)
        float ratio_curr_speed = std::fmax(fabs(r_speed), fabs(l_speed)) / max_speed; 
        if (ratio_curr_speed > 1) {
            r_speed /= ratio_curr_speed;
            l_speed /= ratio_curr_speed;
        }

        // apply calculated velocities to motors
        if (forwards)
            this->chassis->drivetrain->voltage_command(r_speed, l_speed);
        else
            this->chassis->drivetrain->voltage_command(-l_speed, -r_speed);

        // log for debugging
        write_file << logger::string_format("target: %lf %lf %lf, curr: %lf %lf %lf , target speed(curve/pid): %lf %lf, angular speed: %lf side speeds(r/l): %lf %lf curr lookahead: %lf , error: %lf\n",
            target_point.x, target_point.y, route.positions[closest_i].heading, curr_position.x, curr_position.y, curr_position.heading, max_curr_speed, pid_speed, angular_velocity, r_speed, l_speed, curr_lookahead, error
            
        ) << "\n";
        
        // run all actions between previous closest point and current
        if (prev_closest_i != closest_i) {
            for (int i = prev_closest_i; i < closest_i; i++) {
                if (route.actions.contains(i)) { // ensure we are not accessing empty vector
                    for (std::function<void()> action : route.actions.at(i)) { // run action
                        action();
                    }
                }
            }
        }

        prev_closest_i = closest_i;

        // wait for next iteration of loop
        pros::delay(10);
        timeout -= 10;

        if (timeout < 0) break;
    }

    // stop motors after route over
    this->chassis->drivetrain->voltage_command(0, 0);

    this->in_motion = false;
    return;
    
}

void knights::RobotController::follow_profile_pursuit(const knights::MotionProfile &profile, float lookahead_distance, const float max_speed, bool forwards, 
    float end_tolerance, float timeout) {
    // make sure this is only movement running and route is valid
    if (this->in_motion || profile.timestamps.size() < 2) return;
    this->in_motion = true;

    // follow a pure pursuit route  

    // make bot move backwards if lookahead is negative - shorthand
    if (lookahead_distance < 0) {
        forwards = false;
        lookahead_distance = fabs(lookahead_distance);
    }

    // use pid if end tolerance is negative - shorthand
    if (end_tolerance < 0) {
        end_tolerance = fabs(end_tolerance);
    }

    // make sure motors are on brake - prevent drift at end
    this->chassis->drivetrain->right_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    this->chassis->drivetrain->left_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // declare essential values
    knights::Pos target_point = profile.timestamps[0].position;
    int closest_i = 0;
    float closest_dist = 1e5;
    float error = distance_btwn(this->chassis->curr_position, profile.timestamps.back().position);

    float max_lookahead = lookahead_distance;
    float angular_curve;

    this->lateral_pid->reset();

    if (this->angular_pid != nullptr) {
        this->angular_pid->reset();
    }

    // std::fstream write_file("/usd/pure_pursuit_profile.txt", std::ios_base::out);

    // While the robot has not reached the desired point and is not at the end of the route
    while (error > end_tolerance && closest_i != profile.timestamps.size()-1 ) {
    // while (!(error < end_tolerance || (error < end_tolerance*4 && closest_i == route.positions.size()-1))) {
    // while (error > end_tolerance ) {

        knights::Pos curr_position = this->chassis->curr_position;
        if (!forwards || lookahead_distance < 0) {
            curr_position.heading = knights::normalize_angle(curr_position.heading + M_PI);
        }

        // lookahead scaling
        if (target_point != profile.timestamps[0].position) { // make sure we have valid closest_i variables, it won't be right if the robot is at the start of the route
            lookahead_distance = clampf(
            max_lookahead * 
            (1.5/curvature(profile.timestamps[closest_i].position, profile.timestamps[closest_i+1].position, profile.timestamps[closest_i+2].position)), // tuned formula dependent on curvature
            max_lookahead*0.5, max_lookahead*1.5); // limit lookahead from going too high or too low
        }

        // update error values
        error = distance_btwn(curr_position, profile.timestamps.back().position);

        closest_dist = 1e5;

        // find nearest point
        for (int i = closest_i; i < profile.timestamps.size(); i++) {
            if (distance_btwn(curr_position,  profile.timestamps[i].position) < closest_dist) {
                closest_dist = distance_btwn(curr_position, profile.timestamps[i].position);
                closest_i = i;
            }
        }

        // find lookahead point
        for (int i = closest_i; i < profile.timestamps.size() - 1; i++) {
            float t = circle_intersection(profile.timestamps[i+1].position, profile.timestamps[i].position, curr_position, lookahead_distance);

            if (t != -1) {
                target_point = lerp(profile.timestamps[i].position, profile.timestamps[i+1].position, t);
            }
        }

        // // see if we need to reverse
        // if (fabsf(knights::angular_error(curr_position.heading, atan2f(target_point.y-curr_position.y, target_point.x-curr_position.x), 0)) > M_PI/2) {
        //     write_file << "reverse: " << knights::angular_error(curr_position.heading, atan2f(target_point.y-curr_position.y, target_point.x-curr_position.x), 0) << "\n";
        //     forwards = !forwards;
        //     continue;
        // }

        // determine the speed and angular curvature to use for calculating ratio of motor velocities
        // float target_speed = std::fmax((profile.timestamps[closest_i].linear_velocity/this->chassis->drivetrain->max_velocity()) * 127.0, this->lateral_pid->min_velocity);
        float target_speed = (profile.timestamps[closest_i].linear_velocity/this->chassis->drivetrain->max_velocity()) * 127.0;
        angular_curve = curvature(curr_position, target_point);
        float angular_velocity = (profile.timestamps[closest_i].angular_velocity * this->chassis->drivetrain->track_width / 2 / this->chassis->drivetrain->max_velocity()) * PROS_MAX_VOLTAGE;

        // decrease angular curve if the target point is at the end of the path
        if (distance_btwn(curr_position, target_point)/max_lookahead < 0.3 && distance_btwn(curr_position, profile.timestamps.back().position) < max_lookahead) {
            angular_curve = 0;
            angular_velocity *= 1.5;
            target_speed = std::fmin(target_speed * (distance_btwn(curr_position, target_point)/max_lookahead), target_speed);
        } else if (distance_btwn(curr_position, target_point)/lookahead_distance < 0.7 && distance_btwn(curr_position, profile.timestamps.back().position) < max_lookahead) {
            angular_curve *= ((distance_btwn(curr_position, target_point)/lookahead_distance) * 0.01);
            // target_speed = std::fmin(target_speed * (distance_btwn(curr_position, target_point)/lookahead_distance), target_speed);
        }

        // calculate right and left speed based on curvature
        float r_speed = target_speed * (2 - angular_curve * this->chassis->drivetrain->track_width) / 2 + angular_velocity;
        float l_speed = target_speed * (2 + angular_curve * this->chassis->drivetrain->track_width) / 2 - angular_velocity;

        // calculate if one is over max alloted speed (might need to be 127.0 - max speed in pros)
        float max_curr_speed = std::fmax(fabs(r_speed), fabs(l_speed)) / max_speed; 
        if (max_curr_speed > 1) {
            r_speed /= max_curr_speed;
            l_speed /= max_curr_speed;
        }

        // apply calculated velocities to motors
        if (forwards)
            this->chassis->drivetrain->voltage_command(r_speed, l_speed);
        else
            this->chassis->drivetrain->voltage_command(-l_speed, -r_speed);

        // log for debugging
        // write_file << logger::string_format("target: %lf %lf , curr: %lf %lf %lf , target speed: %lf , used angular: %lf , side speed: %lf %lf , error: %lf  fwd: %d closest_i: %lf %lf %lf , end pt: %lf %lf %d, real angular_curve: %lf, timeout: %lf, curr lhd: %lf, calculated lhd: %lf, angular vel: %lf, angular max: %lf", 
        //     target_point.x, target_point.y, this->chassis->curr_position.x, this->chassis->curr_position.y, this->chassis->curr_position.heading,
        //     target_speed, angular_curve, r_speed, l_speed, error, forwards, profile.timestamps[closest_i].position.x, profile.timestamps[closest_i].position.y, profile.timestamps[closest_i].position.heading, 
        //     profile.timestamps.back().position.x, profile.timestamps.back().position.y, profile.timestamps.size(), angular_curve/((distance_btwn(curr_position, target_point)/max_lookahead) * 0.1), 
        //     timeout, lookahead_distance, distance_btwn(this->chassis->curr_position, target_point), angular_velocity, angular_pid->get_max_speed()
        // ) << "\n";

        // wait for next iteration of loop
        pros::delay(10);
        timeout -= 10;

        if (timeout < 0) break;
    }

    // stop motors after route over
    this->chassis->drivetrain->voltage_command(0, 0);

    this->in_motion = false;
    return;
    
}

void knights::RobotController::follow_profile_ramsete(const knights::MotionProfile &profile, float end_tolerance, bool forwards) {
    // make sure this is only movement running and route is valid
    if (this->in_motion || profile.timestamps.size() < 2) return;
    this->in_motion = true;

    // make sure motors are on brake - prevent drift at end
    this->chassis->drivetrain->right_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    this->chassis->drivetrain->left_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    float start_time = pros::millis();
    int curr_i = 1;

    std::fstream write_file("/usd/ramsete_output.txt", std::ios_base::out);

    printf("Ramsete started with state vars: max_vel %lf \n", profile.max_velocity);

    while (distance_btwn(this->chassis->curr_position, profile.timestamps.back().position) > end_tolerance) {
        knights::Pos curr_position = this->chassis->curr_position;

        float elapsed_time = (pros::millis() - start_time) / 1000.0;

        if (elapsed_time > profile.timestamps.back().time * 1.5 || curr_i >= profile.timestamps.size() - 1) break;

        // find closest timestamp to current
        while (curr_i < profile.timestamps.size() - 1 && profile.timestamps[curr_i+1].time < elapsed_time) {
            curr_i++;
        }

        const ProfileTimestamp& next = profile.timestamps[curr_i+1];
        const ProfileTimestamp& prev = profile.timestamps[curr_i];
        knights::ProfileTimestamp selected = knights::lerp(prev, next, 
            knights::clamp((elapsed_time - prev.time) / (next.time - prev.time), 0.0, 1.0));

        // obtain error values
        float error_x = knights::to_meters(selected.position.x - curr_position.x);
        float error_y = knights::to_meters(selected.position.y - curr_position.y);
        float error_theta = knights::angular_error(curr_position.heading, selected.position.heading, 0, true);

        float local_error_x = cos(curr_position.heading) * error_x + sin(curr_position.heading) * error_y;
        float local_error_y = -sin(curr_position.heading) * error_x + cos(curr_position.heading) * error_y;

        // convert velocities to meters -> ensure default constants work
        float lin_vel = knights::to_meters(selected.linear_velocity);
        float ang_vel = selected.angular_velocity;

        // calculate gain
        float gain = 2 * this->ramsete_constants->damping * std::sqrt(
            ang_vel * ang_vel + this->ramsete_constants->proportional * lin_vel * lin_vel
        );

        // prevent divide by 0
        if (error_theta < 1e-6) {
            error_theta += 1e-4;
        }

        // calculate output velocities
        float curr_lin_vel = lin_vel * cos(error_theta) + gain * local_error_x; // still in m/s
        float curr_ang_vel = ang_vel + gain * error_theta + this->ramsete_constants->proportional * lin_vel * sin(error_theta) * local_error_y / error_theta;

        // Send to drivetrain
        this->chassis->drivetrain->velocity_command(to_inches(curr_lin_vel), curr_ang_vel);

        // debugging velocities
        float linear_rpm = (to_inches(curr_lin_vel) / (chassis->drivetrain->wheel_diameter * M_PI) * (1/chassis->drivetrain->gear_ratio)) * 60.0;
        float angular_lin_vel = (curr_ang_vel * chassis->drivetrain->track_width/2.0);
        float angular_rpm = (angular_lin_vel / (chassis->drivetrain->wheel_diameter * M_PI) * (1/chassis->drivetrain->gear_ratio)) * 60.0;

        float r_speed = linear_rpm + angular_rpm;
        float l_speed = linear_rpm - angular_rpm;
    
        float ratio_curr_speed = std::fmax(fabs(r_speed), fabs(l_speed)) / (chassis->drivetrain->rpm / chassis->drivetrain->gear_ratio); 
        if (ratio_curr_speed > 1) {
            r_speed /= ratio_curr_speed;
            l_speed /= ratio_curr_speed;
        }

        write_file << knights::logger::string_format(
            "right/left vel %lf %lf final l/a vel %lf %lf curr l/a vel %lf %lf gain %lf curr pos %lf %lf %lf des pos %lf %lf %lf global error %lf %lf %lf local error %lf %lf time %lf \n\n",
            r_speed, l_speed, linear_rpm, angular_rpm, lin_vel, ang_vel, gain, curr_position.x, curr_position.y, curr_position.heading,
            selected.position.x, selected.position.y, selected.position.heading, error_x, error_y, error_theta, local_error_x, local_error_y, elapsed_time
        );

        pros::delay(10);
    }

    write_file.close();

    // stop motors after route over
    this->chassis->drivetrain->voltage_command(0, 0);
    this->chassis->drivetrain->right_mtrs->brake();
    this->chassis->drivetrain->left_mtrs->brake();

    this->in_motion = false;
    return;
}

void knights::RobotController::follow_profile_simple(const knights::MotionProfile &profile, float end_tolerance, bool forwards) {
    // make sure this is only movement running and route is valid
    if (this->in_motion || profile.timestamps.size() < 2) return;
    this->in_motion = true;

    // make sure motors are on brake - prevent drift at end
    this->chassis->drivetrain->right_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    this->chassis->drivetrain->left_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    float start_time = pros::millis();
    int curr_i = 1;

    std::fstream write_file("/usd/simple_mp_output.txt", std::ios_base::out);

    while (distance_btwn(this->chassis->curr_position, profile.timestamps.back().position) > end_tolerance) {
        knights::Pos curr_position = this->chassis->curr_position;

        float elapsed_time = (pros::millis() - start_time) / 1000.0;

        if (elapsed_time > profile.timestamps.back().time * 1.5 || curr_i >= profile.timestamps.size() - 1) break;

        // find closest timestamp to current
        while (curr_i < profile.timestamps.size() - 1 && profile.timestamps[curr_i+1].time < elapsed_time) {
            curr_i++;
        }

        const ProfileTimestamp& next = profile.timestamps[curr_i+1];
        const ProfileTimestamp& prev = profile.timestamps[curr_i];
        knights::ProfileTimestamp selected = knights::lerp(prev, next, 
            knights::clamp((elapsed_time - prev.time) / (next.time - prev.time), 0.0, 1.0));

        // convert velocities to meters -> ensure default constants work
        float lin_vel = selected.linear_velocity;
        float ang_vel = selected.angular_velocity;

        // Send to drivetrain
        this->chassis->drivetrain->velocity_command(lin_vel, ang_vel, profile.max_velocity);

        // debugging velocities
        float linear_rpm = (lin_vel / (chassis->drivetrain->wheel_diameter * M_PI) * (1/chassis->drivetrain->gear_ratio)) * 60.0;
        float angular_lin_vel = (ang_vel * chassis->drivetrain->track_width/2.0);
        float angular_rpm = (angular_lin_vel / (chassis->drivetrain->wheel_diameter * M_PI) * (1/chassis->drivetrain->gear_ratio)) * 60.0;

        float r_speed = linear_rpm + angular_rpm;
        float l_speed = linear_rpm - angular_rpm;

        float ratio_maximum_lin_vel = profile.max_velocity / (chassis->drivetrain->wheel_diameter * M_PI) * ((1/chassis->drivetrain->gear_ratio)) * 60.0;
    
        float ratio_curr_speed = std::fmax(fabs(r_speed), fabs(l_speed)) / (ratio_maximum_lin_vel); 
        if (ratio_curr_speed > 1) {
            r_speed /= ratio_curr_speed;
            l_speed /= ratio_curr_speed;
        }

        write_file << knights::logger::string_format(
            "right/left vel %lf %lf final l/a vel %lf %lf curr l/a vel %lf %lf curr pos %lf %lf %lf des pos %lf %lf %lf time %lf \n\n",
            r_speed, l_speed, linear_rpm, angular_rpm, lin_vel, ang_vel, curr_position.x, curr_position.y, curr_position.heading,
            selected.position.x, selected.position.y, selected.position.heading, elapsed_time
        );

        pros::delay(10);
    }

    write_file.close();

    // stop motors after route over
    this->chassis->drivetrain->voltage_command(0, 0);
    this->chassis->drivetrain->right_mtrs->brake();
    this->chassis->drivetrain->left_mtrs->brake();

    this->in_motion = false;
    return;
}

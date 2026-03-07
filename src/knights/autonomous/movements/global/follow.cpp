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

float knights::circle_intersection(knights::Pos nxt, knights::Pos prev, knights::Pos curr, float lookahead_distance) {
    knights::Pos dir = nxt - prev;
    knights::Pos fro = prev - curr;

    // get coefficents then calculate discriminant
    float a = dir * dir;
    float b = 2 * (fro * dir);
    float c = (fro * fro) - lookahead_distance * lookahead_distance;
    float discrim = b * b - 4 * a * c;

    // if there are valid solutions (>= 0 to handle tangent case)
    if (discrim >= 0) {
        // calculate solutions
        discrim = sqrt(discrim);
        float s1 = (-b + discrim) / (2 * a); // larger t (further along segment)
        float s2 = (-b - discrim) / (2 * a);

        if (s1 >= 0 && s1 <= 1) // prefer the further-along intersection
            return s1;
        else if (s2 >= 0 && s2 <= 1)
            return s2;
        else
            return -1;
    } else // no real solution
        return -1;
}

void knights::RobotController::follow_route(const knights::Route &route, float lookahead_distance, const float max_speed, bool forwards, 
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
    float start_time = pros::millis();

    this->lateral_pid->reset();

    if (this->angular_pid != nullptr && use_pid) {
        this->angular_pid->reset();
    }

    // While the robot has not reached the desired point and is not at the end of the route
    while (error > end_tolerance && closest_i != route.positions.size() - 1) {

        knights::Pos curr_position = this->chassis->curr_position;
        if (!forwards) {
            curr_position.heading = knights::normalize_angle(curr_position.heading + M_PI);
        }

        // update error values
        error = distance_btwn(curr_position, route.positions[route.positions.size()-1]);

        closest_dist = 1e5;

        // find nearest point — search backward a few points too for odometry drift recovery
        int search_start = std::max(0, closest_i - 5);
        for (int i = search_start; i < route.positions.size(); i++) {
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
                    break;
                }
            }
            else {
                float t = circle_intersection(route.positions[target_i+1], route.positions[target_i], curr_position, curr_lookahead);

                if (t != -1) {
                    target_point = lerp(route.positions[target_i], route.positions[target_i+1], t);
                    break;
                }
            }
        }

        // used for later calculations
        const float &effective_lookahead = distance_btwn(curr_position, target_point);

        // lookahead and speed scaling
        max_curr_speed= 127.0;
        if (target_point != route.positions[0] && (closest_i != 0 && closest_i != route.positions.size()-1)) { // make sure we have valid target_i variables, it won't be right if the robot is at the start of the route
            float curv = curvature(route.positions[closest_i-1], route.positions[closest_i], route.positions[closest_i+1]);
            if (curv > 1e-6) { // guard against zero curvature (straight line)
                curr_lookahead = clampf(
                    lookahead_distance * (0.15/curv), // tuned formula dependent on curvature
                    lookahead_distance, lookahead_distance*1.5); // limit lookahead from going too high or too low

                // also change target speed with this
                max_curr_speed = 7.0/curv;
            } else {
                curr_lookahead = lookahead_distance;
                max_curr_speed = 127.0;
            }
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

        // wall-clock timeout check
        if ((pros::millis() - start_time) > timeout) break;
    }

    // stop motors after route over
    this->chassis->drivetrain->voltage_command(0, 0);

    this->in_motion = false;
    return;
    
}

void knights::RobotController::follow_profile(const knights::MotionProfile &profile, float end_tolerance, bool forwards) {
    // make sure this is only movement running and route is valid
    if (this->in_motion || profile.timestamps.size() < 3) return;
    this->in_motion = true;

    // make sure motors are on brake - prevent drift at end
    this->chassis->drivetrain->right_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    this->chassis->drivetrain->left_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    float start_time = pros::millis();
    float elapsed_time = 0;
    int curr_i = 1;

    while (distance_btwn(this->chassis->curr_position, profile.timestamps.back().position) > end_tolerance || elapsed_time < profile.timestamps.back().time) {
        knights::Pos curr_position = this->chassis->curr_position;

        // accurately calculate time
        elapsed_time = (pros::millis() - start_time) / 1000.0;

        // end conditions
        if (elapsed_time > profile.timestamps.back().time * 1.5 || curr_i >= profile.timestamps.size() - 1) break;

        // find closest timestamp to current
        while (curr_i < profile.timestamps.size() - 1 && profile.timestamps[curr_i+1].time < elapsed_time) {
            curr_i++;
        }

        // lerp between current timestamp and next tiemstamp
        const ProfileTimestamp& prev = profile.timestamps[curr_i];
        const ProfileTimestamp& next = profile.timestamps[curr_i+1];
        knights::ProfileTimestamp selected = knights::lerp(prev, next, 
            knights::clampf((elapsed_time - prev.time) / (next.time - prev.time), 0.0f, 1.0f));

        // obtain error values
        float error_x = knights::to_meters(selected.position.x - curr_position.x);
        float error_y = knights::to_meters(selected.position.y - curr_position.y);
        float error_theta = knights::angular_error(curr_position.heading, selected.position.heading, 0, true);

        float local_error_x = cos(curr_position.heading) * error_x + sin(curr_position.heading) * error_y;
        float local_error_y = -sin(curr_position.heading) * error_x + cos(curr_position.heading) * error_y;

        // convert velocities to meters -> ensure default constants work
        float lin_vel = knights::to_meters(selected.linear_velocity);
        float ang_vel = selected.angular_velocity;

        // calculate gain — use curvature_coefficient to scale curvature response
        float gain = 2 * this->ramsete_constants->damping * std::sqrt(
            ang_vel * ang_vel + this->ramsete_constants->proportional * lin_vel * lin_vel
        );

        // proper sinc function: sinc(x) = sin(x)/x, with limit sinc(0) = 1
        float sinc_theta = (std::fabs(error_theta) < 1e-6) ? 1.0f : std::sin(error_theta) / error_theta;

        // calculate output velocities (RAMSETE equations)
        // curvature_coefficient scales the cross-track correction term
        float curr_lin_vel = lin_vel * cos(error_theta) + gain * local_error_x; // still in m/s
        float curr_ang_vel = ang_vel + gain * error_theta + this->ramsete_constants->proportional * this->ramsete_constants->curvature_coefficient * lin_vel * sinc_theta * local_error_y;

        float output_lin_vel = to_inches(curr_lin_vel);

        float output_ang_vel = curr_ang_vel;

        // Send to DT - use interpolated acceleration for proper feedforward
        this->chassis->drivetrain->ramsete_command(output_lin_vel, output_ang_vel, 
            selected.acceleration, ramsete_constants->tuner_v, ramsete_constants->tuner_accel, 
            ramsete_constants->tuner_static, profile.max_velocity);

        pros::delay(10);
    }

    // stop motors after route over
    this->chassis->drivetrain->voltage_command(0, 0);
    this->chassis->drivetrain->right_mtrs->brake();
    this->chassis->drivetrain->left_mtrs->brake();

    this->in_motion = false;
    return;
}

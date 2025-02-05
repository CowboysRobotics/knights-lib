#include "knights/autonomous/controller.hpp"
#include "knights/autonomous/pid.hpp"
#include "knights/autonomous/path.hpp"

#include "knights/robot/chassis.hpp"

#include "knights/util/calculation.hpp"
#include "knights/util/position.hpp"

#include "knights/logger/logger.hpp"
#include "pros/motors.h"

#include <cmath>
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

void knights::RobotController::follow_route_pursuit(knights::Route &route, float lookahead_distance, const float max_speed, bool forwards, 
    float end_tolerance, float timeout, float use_pid) {
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
        use_pid = true;
        end_tolerance = fabs(end_tolerance);
    }

    // make sure motors are on brake - prevent drift at end
    this->chassis->drivetrain->right_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    this->chassis->drivetrain->left_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // declare essential values
    knights::Pos target_point = route.positions[0];
    int closest_i = 0;
    float closest_dist = 1e5;
    float error = distance_btwn(this->chassis->curr_position, route.positions[route.positions.size()-1]);

    float max_lookahead = lookahead_distance;
    float angular_curve;

    this->lateral_pid->reset();

    if (this->angular_pid != nullptr) {
        this->angular_pid->reset();
    }

    // std::fstream write_file("/usd/pure_pursuit.txt", std::ios_base::out);

    // While the robot has not reached the desired point and is not at the end of the route
    while (error > end_tolerance && closest_i != route.positions.size()-1 ) {
    // while (!(error < end_tolerance || (error < end_tolerance*4 && closest_i == route.positions.size()-1))) {
    // while (error > end_tolerance ) {

        knights::Pos curr_position = this->chassis->curr_position;
        if (!forwards || lookahead_distance < 0) {
            curr_position.heading = knights::normalize_angle(curr_position.heading + M_PI);
        }

        // lookahead scaling
        if (target_point != route.positions[0]) { // make sure we have valid closest_i variables, it won't be right if the robot is at the start of the route
            lookahead_distance = clampf(
            max_lookahead * 
            (1.5/curvature(route.positions[closest_i], route.positions[closest_i+1], route.positions[closest_i+2])), // tuned formula dependent on curvature
            max_lookahead*0.5, max_lookahead*1.5); // limit lookahead from going too high or too low
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

        // find lookahead point
        for (int i = closest_i; i < route.positions.size() - 1; i++) {
            float t = circle_intersection(route.positions[i+1], route.positions[i], curr_position, lookahead_distance);

            if (t != -1) {
                target_point = lerp(route.positions[i], route.positions[i+1], t);
            }
        }

        // // see if we need to reverse
        // if (fabsf(knights::angular_error(curr_position.heading, atan2f(target_point.y-curr_position.y, target_point.x-curr_position.x), 0)) > M_PI/2) {
        //     write_file << "reverse: " << knights::angular_error(curr_position.heading, atan2f(target_point.y-curr_position.y, target_point.x-curr_position.x), 0) << "\n";
        //     forwards = !forwards;
        //     continue;
        // }

        // determine the speed and angular curvature to use for calculating ratio of motor velocities
        // float target_speed = std::fmin(3/curvature(route.positions[closest_i], route.positions[closest_i+1], route.positions[closest_i+2]), max_speed);
        float target_speed = this->lateral_pid->update(error);
        angular_curve = curvature(curr_position, target_point);

        // write_file << knights::logger::string_format("tspeed & pts: %lf , %lf %lf , %lf %lf , %lf %lf\n", 
        //     target_speed, route.positions[closest_i].x, route.positions[closest_i].y, route.positions[closest_i+1].x, route.positions[closest_i+1].y,
        //     route.positions[closest_i+2].x, route.positions[closest_i+2].y
        //     );

        float angular_velocity = 0;
        // curve to update angular velocity
        if (this->angular_pid != nullptr && closest_i != 0) {
            // write_file << "angular error: " << angular_error(curr_position.heading, route.positions[closest_i].heading, 0) << "\n";
            angular_velocity = this->angular_pid->update(angular_error(curr_position.heading, route.positions[closest_i].heading, 0), true);
        }

        // decrease angular curve if the target point is at the end of the path
        if (distance_btwn(curr_position, target_point)/max_lookahead < 0.3 && distance_btwn(curr_position, route.positions.back()) < max_lookahead) {
            angular_curve = 0;
            angular_velocity *= 1.5;
            target_speed = std::fmin(target_speed * (distance_btwn(curr_position, target_point)/max_lookahead), target_speed);
        } else if (distance_btwn(curr_position, target_point)/lookahead_distance < 0.7 && distance_btwn(curr_position, route.positions.back()) < max_lookahead) {
            angular_curve *= ((distance_btwn(curr_position, target_point)/lookahead_distance) * 0.01);
            // target_speed = std::fmin(target_speed * (distance_btwn(curr_position, target_point)/lookahead_distance), target_speed);
        }


        // if (target_speed < this->lateral_pid->get_min_speed())
        //     break;

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
        //     target_speed, angular_curve, r_speed, l_speed, error, forwards, route.positions[closest_i].x, route.positions[closest_i].y, route.positions[closest_i].heading, 
        //     route.positions.back().x, route.positions.back().y, route.positions.size(), angular_curve/((distance_btwn(curr_position, target_point)/max_lookahead) * 0.1), 
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

#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"
#include "knights/autonomous/path.h"

#include "knights/robot/chassis.h"

#include "knights/util/calculation.h"
#include "knights/util/position.h"

#include "knights/logger/logger.h"
#include "pros/motors.h"

#include <math.h>


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

    // make sure motors are on break - prevent drift at end
    this->chassis->drivetrain->right_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    this->chassis->drivetrain->left_mtrs->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // declare essential values
    knights::Pos target_point = route.positions[0];
    int closest_i = 0;
    float closest_dist = 1e5;
    float error = distance_btwn(this->chassis->curr_position, route.positions[route.positions.size()-1]);
    float prev_error = error; float total_error = 0.0;

    float max_lookahead = lookahead_distance;
    float angular_curve;

    // While the robot has not reached the desired point and is not at the end of the route
    while (error > end_tolerance && closest_i != route.positions.size()-1 ) {

        knights::Pos curr_position = this->chassis->curr_position;
        if (!forwards || lookahead_distance < 0) {
            curr_position.heading = knights::normalize_angle(curr_position.heading + M_PI);
        }

        // lookahead scaling
        if (target_point != route.positions[0]) { // make sure we have valid closest_i variables, it won't be right if the robot is at the start of the route
            lookahead_distance = clamp(
            max_lookahead * 
            (4/curvature(route.positions[closest_i], route.positions[closest_i+1], route.positions[closest_i+2])) // tuned formula dependent on curvature
            /max_speed,
            max_lookahead*0.8, max_lookahead*3); // limit lookahead from going too high or too low
        }

        // update error values
        error = distance_btwn(curr_position, route.positions[route.positions.size()-1]);
        total_error += error;

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

        // determine the speed and angular curvature to use for calculating ratio of motor velocities
        float target_speed = std::fmin(2/curvature(route.positions[closest_i], route.positions[closest_i+1], route.positions[closest_i+2]), max_speed);
        angular_curve = curvature(curr_position, target_point);

        // decrease angular curve if the target point is at the end of the path
        if (distance_btwn(curr_position, target_point)/lookahead_distance < 0.3) {
            angular_curve *= ((distance_btwn(curr_position, target_point)/lookahead_distance) * 0.1);
            target_speed *= (distance_btwn(curr_position, target_point)/lookahead_distance) * 1.5;
        }

        // // determine speed based on PID if selected to use
        // if (use_pid) {
        //     target_speed = this->pid_controller->update(error, total_error, prev_error);
        // }
        // prev_error = error;

        // calculate right and left speed based on curvature
        float r_speed = target_speed * (2 - angular_curve * this->chassis->drivetrain->track_width) / 2;
        float l_speed = target_speed * (2 + angular_curve * this->chassis->drivetrain->track_width) / 2;

        // calculate if one is over max alloted speed (might need to be 127.0 - max speed in pros)
        float max_curr_speed = std::fmax(fabs(r_speed), fabs(l_speed)) / max_speed; 
        if (max_curr_speed > 1) {
            r_speed /= max_curr_speed;
            l_speed /= max_curr_speed;
        }

        // apply calculated velocities to motors
        if (forwards)
            this->chassis->drivetrain->velocity_command(r_speed, l_speed);
        else
            this->chassis->drivetrain->velocity_command(-l_speed, -r_speed);

        // log for debugging
        if (std::fmod(timeout, 75) == 0) {
            logger::green(logger::string_format("target: %lf %lf , curr: %lf %lf %lf , target speed: %lf , used angular: %lf , side speed: %lf %lf , error: %lf  fwd: %d\n closest_i: %lf %lf %d , end pt: %lf %lf %d, real angular_curve: %lf, timeout: %lf, curr lhd: %lf, calculated lhd: %lf", 
                target_point.x, target_point.y, this->chassis->curr_position.x, this->chassis->curr_position.y, this->chassis->curr_position.heading,
                target_speed, angular_curve, r_speed, l_speed, error, forwards, route.positions[closest_i].x, route.positions[closest_i].y, closest_i, 
                route.positions.back().x, route.positions.back().y, route.positions.size(), angular_curve/((distance_btwn(curr_position, target_point)/lookahead_distance) * 0.1), timeout, lookahead_distance, distance_btwn(this->chassis->curr_position, target_point)/lookahead_distance
            ));
        }

        // wait for next iteration of loop
        pros::delay(10);
        timeout -= 10;

        if (timeout < 0) break;
    }

    // stop motors after route over
    this->chassis->drivetrain->velocity_command(0, 0);

    this->in_motion = false;
    return;
    
}

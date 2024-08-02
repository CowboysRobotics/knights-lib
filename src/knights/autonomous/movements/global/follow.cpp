#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"
#include "knights/autonomous/path.h"

#include "knights/robot/chassis.h"

#include "knights/util/calculation.h"
#include "knights/util/position.h"


float knights::circle_intersection(knights::Pos nxt, knights::Pos prev, knights::Pos curr, float lookahead_distance) {
    knights::Pos dir = nxt - prev;
    knights::Pos fro = prev - curr;

    float a = dir * dir;
    float b = 2 * (fro * dir);
    float c = (fro * fro) - lookahead_distance * lookahead_distance;
    float discrim = b * b - 4 * a * c;

    if (discrim > 0) {
        discrim = sqrt(discrim);
        float s1 = (-b + discrim) / (2 * a);
        float s2 = (-b - discrim) / (2 * a);

        if (s1 >= 0 && s1 <= 1) 
            return s1;
        else if (s2 >= 0 && s2 <= 1) 
            return s2;
        else
            return -1;
    } else 
        return -1;
}

void knights::RobotController::follow_route_pursuit(knights::Route &route, const float &lookahead_distance, const float max_speed, const bool forwards, 
    const float end_tolerance, float timeout) {
    if (this->in_motion || route.positions.size() < 2) return;
    this->in_motion = true;

    // follow a pure pursuit route  

    // declare essential values
    knights::Pos target_point = route.positions[0];
    int closest_i = 0;
    float closest_dist = 1e10;

    // While the robot has not reached the desired point and is not at the end of the route
    while (distance_btwn(this->chassis->curr_position, route.positions[route.positions.size()-1]) > end_tolerance && closest_i != route.positions.size()-1) {

        // find nearest point
        for (int i = 0; i < route.positions.size(); i++) {
            if (distance_btwn(this->chassis->curr_position,  route.positions[i]) < closest_dist) {
                closest_dist = distance_btwn(this->chassis->curr_position,  route.positions[i]);
                closest_i = i;
            }
        }

        // find lookahead point
        for (int i = closest_i; i < route.positions.size() - 1; i++) {
            float t = circle_intersection(route.positions[i+1], route.positions[i], this->chassis->curr_position, lookahead_distance);

            if (t != -1) {
                target_point = lerp(route.positions[i], route.positions[i+1], t);
            }
        }

        // determine the speed and angular curvature to use for calculating ratio of motor velocities
        float target_speed = std::fmin(5/curvature(this->chassis->curr_position, target_point, route.positions[closest_i+1]), max_speed);
        float angular_curve = curvature(this->chassis->curr_position, target_point);

        // calculate right and left speed based on curvature
        float r_speed = target_speed * (2 - angular_curve * this->chassis->drivetrain->track_width) / 2;
        float l_speed = target_speed * (2 + angular_curve * this->chassis->drivetrain->track_width) / 2;

        // calculate if one is over max alloted speed (might need to be 127.0 - max speed in pros)
        float max_curr_speed = std::fmax(fabs(r_speed), fabs(l_speed)) / max_speed; 
        if (max_curr_speed > 1) {
            r_speed /= max_curr_speed;
            l_speed /= max_curr_speed;
        }

        printf("target: %lf %lf curr: %lf %lf %lf , speed: %lf , angular: %lf , side speed: %lf %lf\n", 
            target_point.x, target_point.y, this->chassis->curr_position.x, this->chassis->curr_position.y, this->chassis->curr_position.heading,
            target_speed, angular_curve, r_speed, l_speed
        );

        // apply calculated velocities to motors
        this->chassis->drivetrain->velocity_command(r_speed, l_speed);

        // wait for next iteration of loop
        pros::delay(10);
        timeout -= 10;

        if (timeout < 0) break;
    }

    // stop motors after route over
    this->chassis->drivetrain->velocity_command(0,0);

    this->in_motion = false;
    return;
    
}

void knights::RobotController::follow_route_ramsete(std::vector<squiggles::ProfilePoint> profile, const float &lookahead_distance, const float &end_tolerance ,float timeout) {
    if (this->in_motion) return;
    this->in_motion = true;

    squiggles::ProfilePoint end_point = profile[profile.size()-1];

    // declare essential values
    knights::Pos target_point = knights::Pos(profile[0]);
    int closest_i = 0;
    float closest_dist = 1e10;

    // While the robot has not reached the desired point and is not at the end of the route
    while (distance_btwn(this->chassis->curr_position, knights::Pos(end_point)) > end_tolerance && closest_i != profile.size()-1) {

        // find nearest point
        for (int i = 0; i < profile.size(); i++) {
            if (distance_btwn(this->chassis->curr_position,  knights::Pos(profile[i])) < closest_dist) {
                closest_dist = distance_btwn(this->chassis->curr_position,  knights::Pos(profile[i]));
                closest_i = i;
            }
        }

        // find lookahead point
        for (int i = closest_i; i < profile.size() - 1; i++) {
            float t = circle_intersection(knights::Pos(profile[i+1]), knights::Pos(profile[i]), this->chassis->curr_position, lookahead_distance);
            if (t != -1) {
                target_point = lerp(knights::Pos(profile[i]), knights::Pos(profile[i+1]), t);
            }
        }

        // Global error matrix
        knights::Pos global_error(
            target_point.x - this->chassis->curr_position.x, 
            target_point.y - this->chassis->curr_position.y, 
            target_point.heading - this->chassis->curr_position.heading
        );
        
        // Local Error Matrix
        knights::Pos local_error(
            cosf(this->chassis->curr_position.heading) * global_error.x + sinf(this->chassis->curr_position.heading) * global_error.y,
            -sinf(this->chassis->curr_position.heading) * global_error.x + cosf(this->chassis->curr_position.heading) * global_error.y,
            global_error.heading
        );

        float path_angular_velocity = profile[closest_i].vector.vel * profile[closest_i].curvature;

        printf("path next %lf %lf pt: %lf %lf %lf\n", profile[closest_i].vector.vel, path_angular_velocity, target_point.x, target_point.y, target_point.heading);

        // Using gain formula
        float gain = 2 * this->ramsete_constants->damping * std::sqrt(std::pow(path_angular_velocity, 2) 
            + this->ramsete_constants->proportional * std::pow(profile[closest_i].vector.vel, 2));

        // Applying gain to motion profile
        float computed_linear_velocity = (profile[closest_i].vector.vel * cosf(local_error.heading) + gain * local_error.x) / 
            (this->chassis->drivetrain->wheel_diameter * M_PI);
        float computed_angular_velocity = path_angular_velocity + gain * local_error.heading + 
            (this->ramsete_constants->proportional * profile[closest_i].vector.vel * sinf(local_error.heading) * local_error.y)/(local_error.heading);
        

        // apply calculated velocities to motors
        this->chassis->drivetrain->velocity_command(computed_linear_velocity - computed_angular_velocity, computed_linear_velocity + computed_angular_velocity);

        // wait for next iteration of loop
        pros::delay(10);
        timeout -= 10;

        if (timeout < 0) break;
    }

    this->in_motion = false;
    return;
}
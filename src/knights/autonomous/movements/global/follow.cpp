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
    while (distance_btwn(this->chassis->curr_position, route.positions[route.positions.size()-1]) < end_tolerance || target_point != route.positions[route.positions.size()-1]) {

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

        // apply calculated velocities to motors
        this->chassis->drivetrain->velocity_command(r_speed, l_speed);
    }

    // stop motors after route over
    this->chassis->drivetrain->velocity_command(0,0);

    this->in_motion = false;
    return;
}

void knights::RobotController::follow_route_ramsete(knights::Route &route, float timeout) {
    if (this->in_motion) return;
    this->in_motion = true;

    // follow a ramsete

    this->in_motion = false;
    return;
}
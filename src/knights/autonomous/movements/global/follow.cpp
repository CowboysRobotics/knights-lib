#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"
#include "knights/autonomous/path.h"

#include "knights/robot/chassis.h"

#include "knights/util/calculation.h"

void knights::RobotController::follow_route_pursuit(knights::Route &route, const float &lookahead_distance, const float max_speed, const bool forwards, 
    const float end_tolerance, float timeout) {
    if (this->in_motion || route.positions.size() < 2) return;
    this->in_motion = true;

    // follow a pure pursuit route  

    // declare essential values
    knights::Pos targetPoint = route.positions[0];
    int i = 0;

    // While the robot has not reached the desired point and is not at the end of the route
    while (distance_btwn(this->chassis->curr_position, route.positions[route.positions.size()-1]) < end_tolerance || targetPoint != route.positions[route.positions.size()-1]) {

        // TODO: put following code in a loop that moves through the points until it finds a valid one

        // find nearest point
        while (distance_btwn(targetPoint, this->chassis->curr_position) < lookahead_distance) {
            i++;
            targetPoint = route.positions[i];
        }

        // find lookahead point
        knights::Pos dir = route.positions[i+1] - route.positions[i];
        knights::Pos fro = route.positions[i] - this->chassis->curr_position;

        float t;

        float a = dir * dir;
        float b = 2 * (fro * dir);
        float c = (fro * fro) - lookahead_distance * lookahead_distance;
        float discrim = b * b - 4 * a * c;

        if (discrim > 0) {
            discrim = sqrt(discrim);
            float s1 = (-b + discrim) / (2 * a);
            float s2 = (-b - discrim) / (2 * a);

            if (s1 >= 0 && s1 <= 1) 
                t = s1;
            else if (s2 >= 0 && s2 <= 1) 
                t = s2;
            else
                t = -1;
        } else 
            t = -1;

        // TODO: above, determine a correct value of t and extrapolate the target point value from that

        // determine the speed and angular curvature to use for calculating ratio of motor velocities
        float target_speed = std::fmin(5/curvature(this->chassis->curr_position, targetPoint, route.positions[i+1]), max_speed);
        float angular_curve = curvature(this->chassis->curr_position, targetPoint);

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

void knights::RobotController::follow_route_ramsete(knights::Route &route, float timeout = 6000) {
    if (this->in_motion) return;
    this->in_motion = true;

    // follow a ramsete

    this->in_motion = false;
    return;
}
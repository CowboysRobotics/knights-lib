#include "knights/autonomous/controller.h"
#include "knights/autonomous/pid.h"
#include "knights/autonomous/pathgen.h"

#include "knights/robot/chassis.h"

#include "knights/util/calculation.h"

#include "squiggles/squiggles.hpp"

void knights::RobotController::move_to_point(const Pos point, squiggles::Constraints path_constraints, const float &lookahead_distance, const float &end_tolerance, float timeout) {
    std::vector<squiggles::ProfilePoint> motion_profile = knights::generate_motion_profile(
        this->chassis->curr_position, 
        point,
        this->chassis->drivetrain->track_width,
        path_constraints
    );

    this->follow_route_ramsete(motion_profile, lookahead_distance, end_tolerance, timeout);

    return;
}
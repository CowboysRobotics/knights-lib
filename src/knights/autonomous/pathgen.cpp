#include "knights/util/position.h"
#include "knights/autonomous/path.h"

#include "knights/autonomous/pathgen.h"
#include "squiggles/constraints.hpp"
#include "squiggles/physicalmodel/tankmodel.hpp"
#include "squiggles/spline.hpp"

knights::Route knights::generate_path_to_pos(knights::Pos start, knights::Pos end, const float MAX_VELOCITY, const float MAX_ACCELERATION, const float MAX_JERK, const float DRIVETRAIN_WIDTH) {
  
  squiggles::Constraints path_constraints = squiggles::Constraints(MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK);

  squiggles::SplineGenerator path_generator = squiggles::SplineGenerator(
    path_constraints, 
    std::make_shared<squiggles::TankModel>(DRIVETRAIN_WIDTH, path_constraints));

  std::vector<squiggles::ProfilePoint> path = path_generator.generate({
    squiggles::Pose(start.x, start.y, start.heading),
    squiggles::Pose(end.x, end.y, end.heading)});

  std::vector<knights::Pos> positions;

  for (squiggles::ProfilePoint point : path) {
    positions.emplace_back(point.vector.pose.x, point.vector.pose.y, point.vector.pose.yaw);
  }

  return knights::Route(positions);
}
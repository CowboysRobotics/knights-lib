#include "knights/util/position.h"
#include "knights/autonomous/path.h"

#include "knights/autonomous/pathgen.h"

knights::Route knights::generate_path_to_pos(knights::Pos start, knights::Pos end, const int NUM_PTS) {
  float deltaX = end.x - start.x;
  float deltaY = end.y - start.y;
  float deltaHeading = (end.heading + 0.001 ) - start.heading;
  // constants
  float ci = start.heading;
  float a0 = cosf(-ci) * deltaX - sinf(-ci) * deltaY;
  float b0 = sinf(-ci) * deltaX + cosf(-ci) * deltaY;
  float t0 = asinf(1.0 / (1 - (a0 / b0) * tanf(deltaHeading)));
//   printf("t-value: %lf\n", t0);
  float t = ((t0 + (M_PI / 2)) / NUM_PTS);

  // not constant
  float x0;
  float y0;

  std::vector<knights::Pos> positions;

  for (int i = 1; i <= NUM_PTS; i++) {
    x0 = (a0 / cosf(t0)) * cosf((-M_PI / 2) + (i * t));
    y0 = (b0 / (sinf(t0) + 1)) * (sinf((-M_PI /2) + (i * t)) + 1);

    // printf("coordinates: %lf %lf\n", x0, y0);

    positions.emplace_back(
      start.x + ((x0 * cosf(ci)) - y0 * sinf(ci)), 
      start.y + ((x0 * sinf(ci)) + y0 * cosf(ci)),
      0.0
    );
  }

  return knights::Route(positions);
}
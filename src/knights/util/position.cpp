#include "knights/util/position.h"

using namespace knights;

pos::pos(float x, float y, float heading) {
    this->x = x;
    this->y = y;
    this->heading = heading;
}

pos::pos() {
    x = 0.0;
    y = 0.0;
    heading = 0.0;
}

float knights::distance_between_pts(pos pt1, pos pt2) {
    return std::hypot(pt2.x - pt1.x, pt2.y - pt2.x);
}
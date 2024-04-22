#include "knights/util/Position.h"
#include "position.h"

using namespace knights;

Pos::Pos(float x, float y, float heading) {
    this->x = x;
    this->y = y;
    if (heading <= 2*M_PI)
        this->heading = heading;
    else
        this->heading = heading * 180.0 / M_PI;
}

Pos::Pos() {
    x = 0.0;
    y = 0.0;
    heading = 0.0;
}

Point::Point(float x, float y) {
    this->x = x;
    this->y = y;
}

Point::Point() {
    x = 0.0;
    y = 0.0;
}

Pos knights::operator+(const Pos &pt1, const Pos &pt2) {
    return Pos(pt1.x+pt2.x, pt1.x+pt2.x, std::fmod(pt1.heading+pt2.heading + 8*M_PI, 2*M_PI));
};

Pos knights::operator-(const Pos &pt1, const Pos &pt2) {
    return Pos(pt1.x-pt2.x, pt1.x-pt2.x, std::fmod(pt1.heading-pt2.heading + 8*M_PI, 2*M_PI));
};

Point knights::operator+(const Point &pt1, const Point &pt2) {
    return Point(pt1.x+pt2.x, pt1.x+pt2.x);
};

Point knights::operator-(const Point &pt1, const Point &pt2) {
    return Point(pt1.x-pt2.x, pt1.x-pt2.x);
};

float knights::distance_btwn(const Pos &pt1, const Pos &pt2) {
    return std::hypot(pt2.x - pt1.x, pt2.y - pt2.x);
};

float knights::distance_btwn(const Point &pt1, const Point &pt2) {
    return std::hypot(pt2.x - pt1.x, pt2.y - pt2.x);
};

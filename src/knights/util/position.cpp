#include "knights/util/position.h"
#include "knights/util/calculation.h"

using namespace knights;

Pos::Pos(float x, float y, float heading, bool deg) {
    this->x = x;
    this->y = y;
    this->heading = heading;

    if (deg)
        this->heading = to_rad(heading);
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

// TODO: Make sure these four below are correct logic
bool knights::operator!=(const Pos &pt1, const Pos &pt2) {
    return (distance_btwn(pt1, pt2) != 0);
}

bool knights::operator==(const Pos &pt1, const Pos &pt2) {
    return (distance_btwn(pt1, pt2) == 0);
}

bool knights::operator!=(const Point &pt1, const Point &pt2) {
    return (distance_btwn(pt1, pt2) != 0);
}

bool knights::operator==(const Point &pt1, const Point &pt2) {
    return (distance_btwn(pt1, pt2) == 0);
}
// ---- END TODO ----

float knights::operator*(const Pos &pt1, const Pos &pt2) {
    return (pt1.x * pt2.x + pt1.y * pt2.y);
};

float knights::operator*(const Point &pt1, const Point &pt2) {
    return (pt1.x * pt2.x + pt1.y * pt2.y);
};

Point knights::operator+(const Point &pt1, const Point &pt2) {
    return Point(pt1.x+pt2.x, pt1.x+pt2.x);
};

Point knights::operator-(const Point &pt1, const Point &pt2) {
    return Point(pt1.x-pt2.x, pt1.x-pt2.x);
};

float knights::distance_btwn(const Pos &pt1, const Pos &pt2) {
    return std::hypot(pt2.x-pt1.x, pt2.y-pt1.y);
};

Pos knights::closest_point(const Pos &start, const Pos &pt1, const Pos &pt2) {
    if (distance_btwn(start, pt1) <= distance_btwn(start, pt2))
        return pt1;
    else
        return pt2;
}

Point knights::closest_point(const Point &start, const Point &pt1, const Point &pt2) {
    if (distance_btwn(start, pt1) <= distance_btwn(start, pt2))
        return pt1;
    else
        return pt2;
}


float knights::distance_btwn(const Point &pt1, const Point &pt2) {
    return std::hypot(pt2.x-pt1.x, pt2.y-pt1.y);
};

float knights::curvature(const Pos &pt1, const Pos &pt2, const Pos &pt3) {
    // calculate midpoints of segments btwn points
    float mx1 = (pt1.x + pt2.x) / 2.0;
    float my1 = (pt1.y + pt2.y) / 2.0;
    float mx2 = (pt2.x + pt3.x) / 2.0;
    float my2 = (pt2.y + pt3.y) / 2.0;

    // calculate slopes of segments btwn points
    float slope1 = (pt2.y - pt1.y) / (pt2.x - pt1.x);
    float slope2 = (pt3.y - pt2.y) / (pt3.x - pt2.x);

    // calculate slopes of perpendicular bisectors
    float perp_slope1 = -1 / slope1;
    float perp_slope2 = -1 / slope2;

    // check if slopes are infinite (means vertical line) - if they are return 0 for no curvature
    if (std::isinf(slope1) || std::isinf(slope2)) {
        return 0.0;
    }

    // check if slopes are parallel (means points are in a straight line) - if they are return 0 for no curvature
    if (std::fabs(perp_slope1 - perp_slope2) < 1e-6) {
        return 0.0;
    }

    // calculate y-intercepts of perpendicular bisectors
    float b1 = my1 - perp_slope1 * mx1;
    float b2 = my2 - perp_slope2 * mx2;

    // calculate center coordinates
    float center_x = (b2 - b1) / (perp_slope1 - perp_slope2);
    float center_y = perp_slope1 * center_x + b1;

    // calculate radius
    float radius = sqrt((pt1.x - center_x) * (pt1.x - center_x) + (pt1.y - center_y) * (pt1.y - center_y));

    return 1.0 / radius;
}

float knights::curvature(const Point &pt1, const Point &pt2, const Point &pt3) {
    // calculate midpoints of segments btwn points
    float mx1 = (pt1.x + pt2.x) / 2.0;
    float my1 = (pt1.y + pt2.y) / 2.0;
    float mx2 = (pt2.x + pt3.x) / 2.0;
    float my2 = (pt2.y + pt3.y) / 2.0;

    // calculate slopes of segments btwn points
    float slope1 = (pt2.y - pt1.y) / (pt2.x - pt1.x);
    float slope2 = (pt3.y - pt2.y) / (pt3.x - pt2.x);

    // calculate slopes of perpendicular bisectors
    float perp_slope1 = -1 / slope1;
    float perp_slope2 = -1 / slope2;

    // check if slopes are infinite (means vertical line) - if they are return 0 for no curvature
    if (std::isinf(slope1) || std::isinf(slope2)) {
        return 0.0;
    }

    // check if slopes are parallel (means points are in a straight line) - if they are return 0 for no curvature
    if (std::fabs(perp_slope1 - perp_slope2) < 1e-6) {
        return 0.0;
    }

    // calculate y-intercepts of perpendicular bisectors
    float b1 = my1 - perp_slope1 * mx1;
    float b2 = my2 - perp_slope2 * mx2;

    // calculate center coordinates
    float center_x = (b2 - b1) / (perp_slope1 - perp_slope2);
    float center_y = perp_slope1 * center_x + b1;

    // calculate radius
    float radius = sqrt((pt1.x - center_x) * (pt1.x - center_x) + (pt1.y - center_y) * (pt1.y - center_y));

    return 1.0 / radius;
}

float knights::curvature(const Pos &start, const Pos &end) {
    // calculate whether the bot needs to turn right or left
    // simplified form of seeing whether a point on the robot travelling line is r/l of the target
    float side = knights::signum((sinf(start.heading) * (end.x - start.x) - cosf(start.heading) * (end.y - start.y)));

    // now calculate arc
    // using formula ax + by + c = 0 with b = 1, solve for x
    float a = -tanf(start.heading);
    float c = tanf(start.heading) * start.x - start.y;
    float x = fabsf(a * end.x + end.y + c) / sqrt((a * a) + 1);

    // calculate current distance between the start and end point
    float lookaheadDistance = hypotf(end.x-start.x, end.y-start.y);

    // return the curvature
    return side * ((2 * x) / (lookaheadDistance * lookaheadDistance));
}

knights::Pos knights::lerp(const Pos &pt1, const Pos &pt2, const float t) {
    return knights::Pos(pt1.x + (pt2.x - pt1.x) * t, pt1.y + (pt2.y - pt1.y) * t, pt1.heading);
}

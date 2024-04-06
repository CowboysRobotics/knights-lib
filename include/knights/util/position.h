#pragma once

#ifndef _POS_H
#define _POS_H

#include <cmath>

namespace knights {

    struct pos {
        float x, y, heading;

        pos(float x, float y, float heading);

        pos();
    };

    float distance_between_pts(pos pt1, pos pt2);
}

#endif
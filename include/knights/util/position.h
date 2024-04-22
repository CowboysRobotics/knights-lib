#pragma once

#ifndef _POS_H
#define _POS_H

#include <cmath>

namespace knights {

    struct Pos {
        float x, y, heading;

        /*
        @brief create a new position with provided values
        @param x x value of the position
        @param y y value of the position
        @param heading the heading for the position, 
            provided heading if less than 2*pi and converts to radians if greater than 2*pi
        */
        Pos(float x, float y, float heading);

        /*
        @brief create a new position with values (0.0, 0.0, 0.0)
        */
        Pos();
    };

    struct Point {
        float x, y;

        /*
        @brief create a new point with provided values
        @param x x value of the point
        @param y y value of the point
        */
        Point(float x, float y);

        /*
        @brief create a new point with values (0.0, 0.0)
        */
        Point();
    };

    /*
    @brief adds the two positions together
    @param pt1,pt2 position to add
    @return a position containing the combination of both positions
    */
    Pos operator+(const Pos &pt1, const Pos &pt2);

    /*
    @brief subtracts a position from another
    @param pt1 position to subtract from
    @param pt2 position to subtract
    @return a position containing pt2 subtracted from pt1
    */
    Pos operator-(const Pos &pt1, const Pos &pt2);

    /*
    @brief adds the two points together
    @param pt1,pt2 points to add
    @return a points containing the combination of both points
    */
    Point operator+(const Point &pt1, const Point &pt2);

    /*
    @brief subtracts a points from another
    @param pt1 points to subtract from
    @param pt2 points to subtract
    @return a points containing pt2 subtracted from pt1
    */
    Point operator-(const Point &pt1, const Point &pt2);

    float distance_btwn(const Pos &pt1, const Pos &pt2);

    float distance_btwn(const Point &pt1, const Point &pt2);
}

#endif
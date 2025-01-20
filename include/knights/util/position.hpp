#pragma once

#ifndef _POS_H
#define _POS_H

#include <cmath>

namespace knights {

    struct Pos {
        float x, y, heading;

        /**
        @brief create a new position with provided values
        @param x x value of the position
        @param y y value of the position
        @param heading the heading for the position
        @param deg whether or not the provided heading is in degrees
        */
        Pos(float x, float y, float heading, bool deg = false);

        /**
        @brief create a new position with values (0.0, 0.0, 0.0)
        */
        Pos();


    };

    struct Point {
        float x, y;

        /**
        @brief create a new point with provided values
        @param x x value of the point
        @param y y value of the point
        */
        Point(float x, float y);

        /**
        @brief create a new point with values (0.0, 0.0)
        */
        Point();
    };

    /**
    @brief adds the two positions together
    @param pt1,pt2 position to add
    @return a position containing the combination of both positions
    */
    Pos operator+(const Pos &pt1, const Pos &pt2);

    /**
    @brief subtracts a position from another
    @param pt1 position to subtract from
    @param pt2 position to subtract
    @return a position containing pt2 subtracted from pt1
    */
    Pos operator-(const Pos &pt1, const Pos &pt2);

    /**
     * @brief Compare whether two points are equal
     * 
     * @param pt1 first point
     * @param pt2 second point
     * @return Pos 
     */
    bool operator!=(const Pos &pt1, const Pos &pt2);

    /**
     * @brief Compare whether two points are equal
     * 
     * @param pt1 first point
     * @param pt2 second point
     * @return Pos 
     */
    bool operator==(const Pos &pt1, const Pos &pt2);

    /**
    @brief adds the two points together
    @param pt1,pt2 point to add
    @return a points containing the combination of both points
    */
    Point operator+(const Point &pt1, const Point &pt2);

    /**
    @brief subtracts a points from another
    @param pt1 point to subtract from
    @param pt2 point to subtract
    @return a points containing pt2 subtracted from pt1
    */
    Point operator-(const Point &pt1, const Point &pt2);

    /**
     * @brief Compare whether two points are equal
     * 
     * @param pt1 first point
     * @param pt2 second point
     * @return Pos 
     */
    bool operator!=(const Point &pt1, const Point &pt2);

    /**
     * @brief Compare whether two points are equal
     * 
     * @param pt1 first point
     * @param pt2 second point
     * @return Pos 
     */
    bool operator==(const Point &pt1, const Point &pt2);

    /**
     * @brief Dot product of two points
     * 
     * @param pt1 point one
     * @param pt2 point two
     * @return float 
     */
    float operator*(const Point &pt1, const Point &pt2);

    /**
     * @brief Dot product of two points
     * 
     * @param pt1 point one
     * @param pt2 point two
     * @return float 
     */
    float operator*(const Pos &pt1, const Pos &pt2);

    /**
    @brief get the distance between two positions
    @param pt1,pt2 point
    @return the distance between the points
    */
    float distance_btwn(const Pos &pt1, const Pos &pt2);

    /**
    @brief get the distance between two positions
    @param pt1,pt2 point
    @return the distance between the points
    */
    float distance_btwn(const Point &pt1, const Point &pt2);

    /**
     * @brief Get whichever point is closer to a certain point
     * 
     * @param start point to measure from
     * @param pt1 point to consider
     * @param pt2 point to consider
     * @return The closet point out of point 1 and 2 to the start point
     */
    Pos closest_point(const Pos &start, const Pos &pt1, const Pos &pt2);

    /**
     * @brief Get whichever point is closer to a certain point
     * 
     * @param start point to measure from
     * @param pt1 point to consider
     * @param pt2 point to consider
     * @return The closet point out of point 1 and 2 to the start point
     */
    Point closest_point(const Point &start, const Point &pt1, const Point &pt2);

    /**
     * @brief Get the curvature of a circle intersecting a provided amount of points
     * 
     * @param pt1 start point
     * @param pt2 middle point
     * @param pt3 end point
     * @return 1 / radius of the circle (curvature)
     */
    float curvature(const Pos &pt1, const Pos &pt2, const Pos &pt3);

    /**
     * @brief Get the curvature of a circle intersecting a provided amount of points
     * 
     * @param pt1 start point
     * @param pt2 middle point
     * @param pt3 end point
     * @return 1 / radius of the circle (curvature)
     */
    float curvature(const Point &pt1, const Point &pt2, const Point &pt3);

    /**
     * @brief Get the curvature of a parabola intersecting two points
     * 
     * @param start the point to start at, this would represent a zero of the parabola
     * @param end the point to end at, this would represent the vertex of the parabola
     * @return float 
     */
    float curvature(const Pos &start, const Pos &end);

    /**
     * @brief Linear interpolation between two points
     * 
     * @param pt1 start point
     * @param pt2 end point
     * @param t percent of distance between the points [0,1]
     * @return Pos at the specified t value between the two
     */
    Pos lerp(const Pos &pt1, const Pos &pt2, const float t);
}

#endif
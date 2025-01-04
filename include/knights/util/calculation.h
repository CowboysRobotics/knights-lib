#pragma once
#include <vector>

#ifndef _CALCULATION_H
#define _CALCULATION_H

#include "knights/util/position.h"

namespace knights {

    /**
    * @brief convert degrees to radians
    * @param degrees amt of degrees to convert
    * @return the provided value in radians
    */
    float to_rad(float degrees);

    /**
    * @brief convert radians to degrees
    * @param degrees amt of radians to convert
    * @return the provided value in degrees
    */
    float to_deg(float radians);

    /**
    * @brief normalize an angle within the domain of [0,2pi) or [0,360)
    * @param angle the angle to normalize
    * @param rad whether or not the angle is in radians (if false, it is in degrees)
    * @return the angle within the domain [0,2pi) or [0,360)
    */
    float normalize_angle(float angle, bool rad = true);

    /**
    @brief obtain the minimum angle between two angles
    @param start,target the two angles
    @param rad whether or not the angle is in radians (if false, it is in degrees)
    @return the minimum angle between two angles
    */
    float min_angle(float start, float target, bool rad = true);

    /**
     * @brief Get angular error between two angles
     * 
     * @param start Angle to start at
     * @param target Angle to calculate error from
     * @param dir Right or Left, positive for left, negative for right
     * @param rad Whether provided angles are in radians
     * @return Angular error from the start angle to the target angle 
     */
    float angular_error(float start, float target, int dir, bool rad = true);

    /**
     * @brief Get he direction between the two angles
     * 
     * @param init_heading initial heading
     * @param des_heading heading to turn to
     * @param rad whether input is in radians or not (assumed degrees otherwise)
     * @return 1 if left, -1 if right, never will return 0
     */
    int direction(float init_heading, float des_heading, bool rad = true);

    /**
    @brief basic signum function, evaluates whether or not a number is above, below, or at zero
    @param num the number to evaulate
    @return 1 if num > 0, 0 if num = 0, -1 if num < 0
    */
    float signum(float num);

    /**
    @brief basic signum function, evaluates whether or not a number is above, below, or at zero
    @param num the number to evaulate
    @return 1 if num > 0, 0 if num = 0, -1 if num < 0
    */
    int signum(int num);

    /**
    @brief get the average of a vector of values
    @param arr the vector (array) of values
    @return the average
    */
    float avg(std::vector<float>arr);

    /**
    @brief get the average of a vector of values
    @param arr the vector (array) of values
    @return the average
    */
    double avg(std::vector<double>arr);

    /**
    @brief get the average of a vector of values
    @param arr the vector (array) of values
    @return the average
    */
    int avg(std::vector<int>arr);

    /**
    @brief restrict a number to a range of [min, max]
    @param num number to restrict
    @param max maximum limit
    @param min minimum limit
    @return min if num < min, max if num > max, num if else
    */
    float clamp(float num, float min, float max);

    /**
    @brief restrict a number to a range of [min, max]
    @param num number to restrict
    @param max maximum limit
    @param min minimum limit
    @return min if num < min, max if num > max, num if else
    */
    int clamp(int num, int min, int max);

    /**
     * @brief Get the solution for a circle centered at curr with a radius of lookahead with an intersection of the line segment prev and nxt
     * 
     * @param nxt end of line segment
     * @param prev start of line segment
     * @param curr center of the circle
     * @param lookahead_distance radius of the circle
     * @return The 't' parameter 
     */
    float circle_intersection(knights::Pos nxt, knights::Pos prev, knights::Pos curr, float lookahead_distance);

    float to_meters(float inches);
    float to_inches(float meters);
}

#endif
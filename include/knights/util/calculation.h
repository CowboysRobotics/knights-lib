#pragma once
#include <vector>

#ifndef _CALCULATION_H
#define _CALCULATION_H

namespace knights {

    /*
    @brief convert degrees to radians
    @param degrees amt of degrees to convert
    @return the provided value in radians
    */
    float to_rad(float degrees);

    /*
    @brief convert radians to degrees
    @param degrees amt of radians to convert
    @return the provided value in degrees
    */
    float to_deg(float radians);

    /*
    @brief normalize an angle within the domain of [0,2pi) or [0,360)
    @param angle the angle to normalize
    @param rad whether or not the angle is in radians (if false, it is in degrees)
    @return the angle within the domain [0,2pi) or [0,360)
    */
    float normalize_angle(float angle, bool rad = true);

    /*
    @brief obtain the minimum angle between two angles
    @param angle1,angle2 the two angles
    @param rad whether or not the angle is in radians (if false, it is in degrees)
    @return the minimum angle between two angles
    */
    float min_angle(float angle1, float angle2, bool rad = false);

    /*
    @brief basic signum function, evaluates whether or not a number is above, below, or at zero
    @param num the number to evaulate
    @return 1 if num > 0, 0 if num = 0, -1 if num < 0
    */
    float signum(float num);

    /*
    @brief basic signum function, evaluates whether or not a number is above, below, or at zero
    @param num the number to evaulate
    @return 1 if num > 0, 0 if num = 0, -1 if num < 0
    */
    int signum(int num);

    /*
    @brief get the average of a vector of values
    @param arr the vector (array) of values
    @return the average
    */
    float avg(std::vector<float>arr);

    /*
    @brief get the average of a vector of values
    @param arr the vector (array) of values
    @return the average
    */
    int avg(std::vector<int>arr);

    /*
    @brief restrict a number to a range of [min, max]
    @param num number to restrict
    @param max maximum limit
    @param min minimum limit
    @return min if num < min, max if num > max, num if else
    */
    float clamp(float num, float min, float max);

    /*
    @brief restrict a number to a range of [min, max]
    @param num number to restrict
    @param max maximum limit
    @param min minimum limit
    @return min if num < min, max if num > max, num if else
    */
    int clamp(int num, int min, int max);

}

#endif
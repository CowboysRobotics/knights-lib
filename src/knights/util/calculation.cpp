#include "knights/util/calculation.h"
#include "knights/robot/drivetrain.h"

#include <math.h>
#include <numeric>

float knights::to_rad(float degrees) {
    return degrees * (M_PI / 180.0);
};

float knights::to_deg(float radians) {
    return radians * (180.0 / M_PI);
};

float knights::normalize_angle(float angle, bool rad) {
    if (rad)
        return std::fmod(std::fmod(angle, (2*M_PI)) + (8*M_PI), 2*M_PI);
    else
        return std::fmod(std::fmod(angle, 360) + 4*360, 360);
};

float knights::min_angle(float start, float target, bool rad) {
    float max = rad ? M_PI*2 : 360.0;
    float error = normalize_angle(normalize_angle(target, rad), rad) - start;
    return std::remainder(error,max);
};

int knights::direction(float init_heading, float des_heading, bool rad) {
    float max = rad ? M_PI*2 : 360.0; 
    float diff = knights::normalize_angle(des_heading, rad) - knights::normalize_angle(init_heading, rad);

    if (diff < -max/2)
        diff += max;
    if (diff > max/2)
        diff -= max;

    if (diff > 0)
        return -1;
    else
        return 1;
}

float knights::angular_error(float start, float target, int dir, bool rad) {
    // unimplmented yet
    return target-start;
}

float knights::signum(float num) {
    return (float)(num > 0) - (num < 0);
};

int knights::signum(int num) {
    return (int)(num > 0) - (num < 0);
};

float knights::avg(std::vector<float>arr) {
    return std::accumulate(arr.begin(), arr.end(), 0.0) / arr.size();
}

double knights::avg(std::vector<double>arr) {
    return std::accumulate(arr.begin(), arr.end(), 0.0) / arr.size();
}

int knights::avg(std::vector<int>arr) {
    return std::accumulate(arr.begin(), arr.end(), 0) / arr.size();
}

float knights::clamp(float num, float min, float max) {
    return std::fmax(min, std::fmin(num, max));
}

int knights::clamp(int num, int min, int max) {
    return std::max(min, std::min(num, max));
}

float knights::to_meters(float inches) {
    return inches/39.37;
}

float knights::to_inches(float meters) {
    return meters*39.37;
}

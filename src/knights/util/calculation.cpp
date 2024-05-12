#include "knights/util/calculation.h"

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

float knights::min_angle(float angle1, float angle2, bool rad) {
    if (!rad) {
        float diff = fmod((angle2 - angle1 + 180.0), 360.0 - 180.0);
        return diff < -180 ? diff + 360 : diff;
    } else {
        float diff = fmod((angle2 - angle1 + M_PI), (2*M_PI) - M_PI);
        return diff < -M_PI ? diff + (2*M_PI) : diff;
    }
};

float knights::signum(float num) {
    return (float)(num > 0) - (num < 0);
};

int knights::signum(int num) {
    return (int)(num > 0) - (num < 0);
};

float knights::avg(std::vector<float>arr) {
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
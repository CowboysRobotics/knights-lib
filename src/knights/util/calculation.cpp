#include "knights/util/calculation.hpp"
#include "knights/robot/drivetrain.hpp"

#include <cmath>
#include <math.h>
#include <numeric>

float knights::lerp(float start, float end, float t) {
    return start + t * (end - start);
}

float knights::normalize_angle(float angle, bool rad) {
    if (rad)
        return std::fmod(std::fmod(angle, (2*M_PI)) + (8*M_PI), 2*M_PI);
    else
        return std::fmod(std::fmod(angle, 360) + 4*360, 360);
};

float knights::ref_angle(float angle, bool rad) {
    float max = rad ? M_PI*2 : 360.0;
    return std::remainder(angle, max);
}

float knights::min_angle(float start, float target, bool rad) {
    float max = rad ? M_PI*2 : 360.0;
    float error = normalize_angle(target, rad) - normalize_angle(start, rad);
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
        return -1; // counterclockwise
    else
        return 1; // clockwise
}

float knights::angular_error(float start, float target, int dir, bool rad) {
    float max = rad ? M_PI*2 : 360.0; 
    start = normalize_angle(start, rad);
    target = normalize_angle(target, rad);

    if (dir == 1) // clockwise
        return (target - start) < 0 ? (target - start) - max : (target - start);
    else if (dir == -1) // counterclockwise
        return (target - start) < 0 ? (target - start) + max : (target - start);
    else
        return ref_angle(target-start, rad);
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

float knights::clampf(float num, float min, float max) {
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

// https://stackoverflow.com/questions/55102504/populate-a-vector-with-linearly-increased-values
std::vector<float> knights::linspace(float start, float end, int points)
{
  std::vector<float> res(points);
  float step = (end - start) / (points - 1);
  size_t i = 0;
  for (auto& e : res)
  {
    e = start + step * i++;
  }
  return res;
}

#include "knights/util/timer.h"
#include "api.h"

knights::Timer::Timer() {
    this->start_time = pros::millis();
}

void knights::Timer::reset() {
    this->start_time = pros::millis();
}

long double knights::Timer::get() {
    return pros::millis() - this->start_time;
}
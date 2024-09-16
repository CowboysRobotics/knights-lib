#include "knights/logger/colors.h"
#include "knights/logger/logger.h"

#include "api.h"

#include <iostream>

void knights::logger::warn(std::string string) {
    std::cout << START_MAG << std::to_string((float)pros::millis()/1000.0) << "s" << RESET;
    std::cout << START_YEL << string << RESET << "\n";
    return;
}

void knights::logger::error(std::string string) {
    std::cout << START_MAG << std::to_string((float)pros::millis()/1000.0) << "s" << RESET;
    std::cout << START_RED << string << RESET << "\n";
    return;
}

void knights::logger::normal(std::string string) {
    std::cout << START_MAG << std::to_string((float)pros::millis()/1000.0) << "s" << RESET;
    std::cout << START_WHT << string << RESET << "\n";
    return;
}

void knights::logger::data(std::string string) {
    std::cout << START_MAG << std::to_string((float)pros::millis()/1000.0) << "s" << RESET;
    std::cout << START_BLU << string << RESET << "\n";
    return;
}
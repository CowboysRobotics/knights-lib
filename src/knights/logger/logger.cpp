#include "knights/logger/colors.h"
#include "knights/logger/logger.h"

#include "api.h"

#include <iostream>

void knights::logger::yellow(std::string string) {
    std::cout << START_MAG << "[" << std::to_string((float)pros::millis()/1000.0) << "s" << "] " << RESET;
    std::cout << START_YEL << string << RESET << "\n";
    return;
}

void knights::logger::red(std::string string) {
    std::cout << START_MAG << "[" << std::to_string((float)pros::millis()/1000.0) << "s" << "] " << RESET;
    std::cout << START_RED << string << RESET << "\n";
    return;
}

void knights::logger::white(std::string string) {
    std::cout << START_MAG << "[" << std::to_string((float)pros::millis()/1000.0) << "s" << "] " << RESET;
    std::cout << START_WHT << string << RESET << "\n";
    return;
}

void knights::logger::blue(std::string string) {
    std::cout << START_MAG << "[" << std::to_string((float)pros::millis()/1000.0) << "s" << "] " << RESET;
    std::cout << START_BLU << string << RESET << "\n";
    return;
}

void knights::logger::green(std::string string) {
    std::cout << START_MAG << "[" << std::to_string((float)pros::millis()/1000.0) << "s" << "] " << RESET;
    std::cout << START_GRN << string << RESET << "\n";
    return;
}

void knights::logger::cyan(std::string string) {
    std::cout << START_MAG << "[" << std::to_string((float)pros::millis()/1000.0) << "s" << "] " << RESET;
    std::cout << START_CYN << string << RESET << "\n";
    return;
}
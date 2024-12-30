#include "knights/logger/colors.h"
#include "knights/logger/logger.h"

#include "api.h"

#include <iomanip>
#include <iostream>

#include <memory>
#include <string>
#include <stdexcept>


// function from https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
std::string knights::logger::string_format(const std::string fmt, ...) {
    int size = ((int)fmt.size()) * 2 + 50;   // Use a rubric appropriate for your code
    std::string str;
    va_list ap;
    while (1) {     // Maximum two passes on a POSIX system...
        str.resize(size);
        va_start(ap, fmt);
        int n = vsnprintf((char *)str.data(), size, fmt.c_str(), ap);
        va_end(ap);
        if (n > -1 && n < size) {  // Everything worked
            str.resize(n);
            return str;
        }
        if (n > -1)  // Needed size returned
            size = n + 1;   // For null char
        else
            size *= 2;      // Guess at a larger size (OS specific)
    }
    return str;
}

void knights::logger::yellow(std::string string) {
    std::cout << std::setprecision(4) << START_MAG << "[" << std::to_string((float)pros::millis()/1000.0) << "s" << "] " << RESET;
    std::cout << START_YEL << string << RESET << "\n";
    return;
}

void knights::logger::red(std::string string) {
    std::cout << std::setprecision(4) << START_MAG << "[" << std::to_string((float)pros::millis()/1000.0) << "s" << "] " << RESET;
    std::cout << START_RED << string << RESET << "\n";
    return;
}

void knights::logger::white(std::string string) {
    std::cout << std::setprecision(4) << START_MAG << "[" << std::to_string((float)pros::millis()/1000.0) << "s" << "] " << RESET;
    std::cout << START_WHT << string << RESET << "\n";
    return;
}

void knights::logger::blue(std::string string) {
    std::cout << std::setprecision(4) << START_MAG << "[" << std::to_string((float)pros::millis()/1000.0) << "s" << "] " << RESET;
    std::cout << START_BLU << string << RESET << "\n";
    return;
}

void knights::logger::green(std::string string) {
    std::cout << std::setprecision(4) << START_MAG << "[" << std::to_string((float)pros::millis()/1000.0) << "s" << "] " << RESET;
    std::cout << START_GRN << string << RESET << "\n";
    return;
}

void knights::logger::cyan(std::string string) {
    std::cout << std::setprecision(4) << START_MAG << "[" << std::to_string((float)pros::millis()/1000.0) << "s" << "] " << RESET;
    std::cout << START_CYN << string << RESET << "\n";
    return;
}

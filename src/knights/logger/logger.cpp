#include "knights/logger/colors.h"
#include "knights/logger/logger.h"

#include "api.h"

#include <iomanip>
#include <iostream>

#include <memory>
#include <string>
#include <stdexcept>


// https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
template<typename ... Args>
std::string knights::logger::string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
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
#pragma once

#ifndef _LOGGER_H
#define _LOGGER_H

#include <string>

namespace knights::logger {
    
    void warn(std::string string);
    void error(std::string string);
    void normal(std::string string);
    void data(std::string string);
}

#endif
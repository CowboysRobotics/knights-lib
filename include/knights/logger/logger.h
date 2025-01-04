#pragma once

#ifndef _LOGGER_H
#define _LOGGER_H

#include <string>

namespace knights::logger {
    
    void yellow(std::string string);
    void red(std::string string);
    void blue(std::string string);
    void green(std::string string);
    void cyan(std::string string);
    void white(std::string string);

    /**
     * @brief Format the string using a printf-style format string
     * 
     * @param fmt Format string
     * @param ...
     * @return std::string Formatted string
     */
    std::string string_format(const std::string fmt, ...);
}

#endif
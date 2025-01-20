#pragma once

#ifndef _COLORS_H
#define _COLORS_H

#define RESET  "\x1B[0m"
#define START_RED  "\x1B[31m"
#define START_GRN  "\x1B[32m"
#define START_YEL  "\x1B[33m"
#define START_BLU  "\x1B[34m"
#define START_MAG  "\x1B[35m"
#define START_CYN  "\x1B[36m"
#define START_WHT  "\x1B[37m"
#define START_BLD  "\x1B[1m"

#define RED(x) START_RED x RESET
#define GREEN(x) START_GRN x RESET
#define YELLOW(x) START_YEL x RESET
#define BLUE(x) START_BLU x RESET
#define MAGENTA(x) START_MAG x RESET
#define CYAN(x) START_CYN x RESET
#define WHITE(x) START_WHT x RESET
#define BOLD(x) START_BLD x RESET

#endif
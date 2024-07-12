#pragma once

#ifndef _KNIGHTS_DISPLAY_H
#define _KNIGHTS_DISPLAY_H

#include "liblvgl/lvgl.h"
#include "knights/api.h"
#include <string>

extern void lv_display(void);

namespace knights {

    struct AutonSelectionPackage {
        std::string type; // attacker or defender
        int number; // number of auton
    };

    void update_pos(knights::Pos curr_pos);

    AutonSelectionPackage get_selected_auton(void);

    void set_pos_label(std::string str);

}

#endif
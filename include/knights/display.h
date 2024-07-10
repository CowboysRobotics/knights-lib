#pragma once

#ifndef _KNIGHTS_DISPLAY_H
#define _KNIGHTS_DISPLAY_H

#include "liblvgl/lvgl.h"
#include "knights/api.h"
#include <string>

namespace knights {

    struct AutonSelectionPackage {
        std::string type; // attacker or defender
        int number; // number of auton
    };

    void lv_display(void);

    AutonSelectionPackage get_selected_auton();

    void set_pos_label(std::string str);

}

const uint8_t pix_art_his_field_map[];

#endif
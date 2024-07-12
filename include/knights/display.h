#pragma once

#ifndef _KNIGHTS_DISPLAY_H
#define _KNIGHTS_DISPLAY_H

#include "liblvgl/lvgl.h"
#include "knights/api.h"
#include <string>

extern void lv_display(void);

namespace knights::display {

    struct MapDot {
        int width,height;
        lv_color_t color;
        lv_obj_t* dot;

        void init(int width,int height,int field_x,int field_y,lv_color_t color);

        void set_field_pos(knights::Pos pos);
    }

    struct AutonSelectionPackage {
        std::string type; // red or blue
        int number; // number of auton
    };

    /**
     * @brief Update the current position dot on the brain GUI
     * 
     * @param curr_pos Current position of the robot
     */
    void update_pos(knights::Pos curr_pos);

    /**
     * @brief Update the target position dot on the brain GUI (for pure pursuit or other pathfinding)
     * 
     * @param pos Target position of the bot
     */
    void update_target_pos(knights::Pos pos);

    /**
     * @brief Get the currently selection combination of auton
     * 
     * @return An auton package that has a type (red or blue) and a number (1-4)
     */
    AutonSelectionPackage get_selected_auton(void);

    /**
     * @brief Set the current position label to a string
     * 
     * @param str String to set the label to
     */
    void set_pos_label(std::string str);

}

#endif
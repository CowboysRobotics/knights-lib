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

        /**
        * @brief Construct a new dot on the brain display map
        * 
        * @param width Width of the dot
        * @param height Height of the dot
        * @param color Color to set the dot to
        */
        MapDot(int width, int height, lv_color_t color);

        /**
         * @brief Construct a new dot on the brain display map
         */
        MapDot();

        /**
        * @brief Initialize the dot
        * 
        * @param width Width of the dot
        * @param height Height of the dot
        * @param color Color to set the dot to
        */
        void init(int width, int height, lv_color_t color);

        /**
        * @brief Set the position of the dot with respect to the field
        * 
        * @param pos Position to set the dot to, in field positions
        */
        void set_field_pos(knights::Pos pos);
    };

    struct AutonSelectionPackage {
        std::string type = "None"; // red or blue
        int number = 0; // number of auton
    };

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

    /**
     * @brief Move the current position dot
     * 
     * @param pos coordinates to move it to
     */
    void change_curr_pos_dot(Pos pos);

}

#endif
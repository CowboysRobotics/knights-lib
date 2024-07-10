#include "knights/display.h"
#include "knights/api.h"
#include "api.h"

knights::AutonSelectionPackage curr_package;

knights::AutonSelectionPackage knights::get_selected_auton() {
    return curr_package;
}

static lv_obj_t * pos_label;

static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        uint32_t id = lv_btnmatrix_get_selected_btn(obj);
        const char * txt = lv_btnmatrix_get_btn_text(obj, id);

        if (txt == "Red" || txt == "Blue") {
            curr_package.type = txt;
        } else if (txt != "Enter") {
            curr_package.number = std::stoi(txt);
        }
        printf("Selected Auton: %s %i\n", curr_package.type.c_str(), curr_package.number);
    }
}

static const char* type_map[] = {"Red", "Blue", ""};

static const char* num_map[] = {"1", "2", "3", "4", ""};

void knights::lv_display(void)
{
    // Auton Type Selector
    lv_obj_t* btnm1 = lv_btnmatrix_create(lv_scr_act());
    lv_btnmatrix_set_map(btnm1, type_map);
    lv_btnmatrix_set_btn_width(btnm1, 0, 2);
    lv_btnmatrix_set_btn_width(btnm1, 1, 2);
    lv_btnmatrix_set_btn_ctrl_all(btnm1, LV_BTNMATRIX_CTRL_CHECKABLE);
    lv_btnmatrix_set_one_checked(btnm1, true);

    lv_obj_align(btnm1, LV_ALIGN_TOP_LEFT, 10, 30);
    lv_obj_add_event_cb(btnm1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_set_content_width(btnm1, 200);
    lv_obj_set_content_height(btnm1, 50);

    // Auton Number Selection
    lv_obj_t* btnm2 = lv_btnmatrix_create(lv_scr_act());
    lv_btnmatrix_set_map(btnm2, num_map);
    lv_btnmatrix_set_btn_ctrl_all(btnm2, LV_BTNMATRIX_CTRL_CHECKABLE);
    lv_btnmatrix_set_one_checked(btnm2, true);

    lv_obj_align(btnm2, LV_ALIGN_BOTTOM_LEFT, 10, -30);
    lv_obj_add_event_cb(btnm2, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_set_content_width(btnm2, 200);
    lv_obj_set_content_height(btnm2, 50);

    // Display User Label - use for odom etc
    pos_label = lv_label_create(lv_scr_act());
    lv_label_set_text(pos_label, "Curr Pos: 0.00 0.00 0.00");
    lv_label_set_long_mode(pos_label , LV_LABEL_LONG_WRAP);
    lv_obj_align(pos_label, LV_ALIGN_TOP_RIGHT, -10,20);
    lv_obj_set_width(pos_label, 210);

    // Display field setup
    // lv_obj_t* bkgd = lv_obj_create(lv_scr_act());
    // lv_obj_set_style_bg_color(bkgd, lv_palette_lighten(LV_PALETTE_GREY,3), LV_STATE_ANY);
    LV_IMG_DECLARE(pix_art_his_field_map);
    lv_obj_t* bkgd = lv_img_create(lv_scr_act());
    lv_img_set_src(bkgd, &pix_art_his_field_map);
    lv_obj_align(bkgd, LV_ALIGN_BOTTOM_RIGHT, -40,-20);
    lv_obj_set_width(bkgd, 180);
    lv_obj_set_height(bkgd, 180);
}  

void knights::set_pos_label(std::string str) {
    lv_label_set_text(pos_label, str.c_str());
}
#include "knights/display.h"
#include "knights/api.h"
#include "api.h"

knights::display::AutonSelectionPackage curr_package;

LV_IMG_DECLARE(pix_art_his_field);

knights::display::AutonSelectionPackage knights::display::get_selected_auton(void) {
    return curr_package;
}

static lv_obj_t * pos_label;
static lv_obj_t * target_pos_label;
static lv_obj_t * curr_pos_dot;

void knights::display::MapDot::init(int width,int height,lv_color_t color) {
    this->dot = lv_obj_create(lv_scr_act());
    lv_obj_set_size(this->dot, width, height);
    lv_obj_set_style_bg_color(this->dot, color, LV_STATE_ANY);
    lv_obj_move_foreground(this->dot);
}

void knights::display::MapDot::set_field_pos(knights::Pos pos) {
    lv_obj_set_pos(this->dot, (pos.x/24 * TILE + X_MARGIN) + (BG_SIZE/2), (-pos.y/24 * TILE + Y_MARGIN) + (BG_SIZE/2));
}

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

void lv_display(void)
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
    lv_obj_t* bkgd = lv_img_create(lv_scr_act());
    lv_img_set_src(bkgd, &pix_art_his_field);
    lv_obj_align(bkgd, LV_ALIGN_BOTTOM_RIGHT, -40,-20);
    lv_obj_set_width(bkgd, 180);
    lv_obj_set_height(bkgd, 180);
    lv_obj_move_background(bkgd);

    // Current position dot
    curr_pos_dot = lv_obj_create(lv_scr_act());
    lv_obj_set_size(curr_pos_dot, 10, 10);
    lv_obj_set_style_bg_color(curr_pos_dot, lv_palette_lighten(LV_PALETTE_GREY,0), LV_STATE_ANY);
    lv_obj_move_foreground(curr_pos_dot);

    // Target position dot
    target_pos_label = lv_obj_create(lv_scr_act());
    lv_obj_set_size(target_pos_label, 10, 10);
    lv_obj_set_style_bg_color(target_pos_label, lv_palette_lighten(LV_PALETTE_CYAN,0), LV_STATE_ANY);
    lv_obj_move_foreground(target_pos_label);
}  

void knights::display::set_pos_label(std::string str) {
    lv_label_set_text(pos_label, str.c_str());
}

#define TILE 180/6
#define X_MARGIN 257
#define Y_MARGIN 36
#define BG_SIZE 180

void knights::display::update_pos(knights::Pos pos) {
    lv_obj_set_pos(curr_pos_dot, (pos.x/24 * TILE + X_MARGIN) + (BG_SIZE/2), (-pos.y/24 * TILE + Y_MARGIN) + (BG_SIZE/2));
}

void knights::display::update_target_pos(knights::Pos pos) {
    lv_obj_set_pos(target_pos_label, (pos.x/24 * TILE + X_MARGIN) + (BG_SIZE/2), (-pos.y/24 * TILE + Y_MARGIN) + (BG_SIZE/2));
}
#include "knights/display.h"
#include "knights/api.h"
#include "api.h"
#include "display.h"

#define TILE 180/6
#define X_MARGIN 270
#define Y_MARGIN 40
#define BG_SIZE 180

knights::display::AutonSelectionPackage curr_package;

LV_IMG_DECLARE(pix_art_his_field);

knights::display::AutonSelectionPackage knights::display::get_selected_auton(void) {
    return curr_package;
}

static lv_obj_t * pos_label;

knights::display::MapDot::MapDot(int width, int height, lv_color_t color) {
    this->width = width;
    this->height = height;
    this->color = color;

    this->dot = lv_obj_create(lv_scr_act());
    lv_obj_set_size(this->dot, this->width, this->height);
    lv_obj_set_style_bg_color(this->dot, this->color, LV_STATE_ANY);
    lv_obj_move_foreground(this->dot);
}

knights::display::MapDot::MapDot() {
    this->width = 5;
    this->height = 5;
    this->color = lv_palette_lighten(LV_PALETTE_GREY, 0);

    this->dot = lv_obj_create(lv_scr_act());
    lv_obj_set_size(this->dot, this->width, this->height);
    lv_obj_set_style_bg_color(this->dot, this->color, LV_STATE_ANY);
    lv_obj_move_foreground(this->dot);
}

void knights::display::MapDot::init(int width, int height, lv_color_t color) {
    this->width = width;
    this->height = height;
    this->color = color;

    this->dot = lv_obj_create(lv_scr_act());
    lv_obj_set_size(this->dot, this->width, this->height);
    lv_obj_set_style_bg_color(this->dot, this->color, LV_STATE_ANY);
    lv_obj_move_foreground(this->dot);
}

void knights::display::MapDot::set_field_pos(knights::Pos pos) {
    pos.x = knights::clamp(pos.x, -72.0, 72.0);
    pos.y = knights::clamp(pos.y, -72.0, 72.0);
    lv_obj_set_pos(this->dot, (pos.x/24 * TILE + X_MARGIN) + (BG_SIZE/2.0) - this->width/2.0, (-pos.y/24 * TILE + Y_MARGIN) + (BG_SIZE/2.0) - this->height/2.0);
}

knights::display::MapDot curr_position_dot;

void knights::display::change_curr_pos_dot(Pos pos) {
    curr_position_dot.set_field_pos(pos);
}

static void event_handler(lv_event_t * e) {
    // Get the button event and the object it was done to
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    // If object value was changed
    if(code == LV_EVENT_VALUE_CHANGED) {
        // Id is the selected 
        uint32_t id = lv_btnmatrix_get_selected_btn(obj);
        const char * txt = lv_btnmatrix_get_btn_text(obj, id);

        if (strcmp(txt, "Red") || strcmp(txt, "Blue") || strcmp(txt, "Skills")) {
            curr_package.type = txt;
        } else if (!strcmp(txt, "Enter")) {
            curr_package.number = std::stoi(txt);
        }
        printf("Selected Auton: %s %i\n", curr_package.type.c_str(), curr_package.number);
    }
}

static const char* type_map[] = {"Red", "Blue", "Skills", ""};

static const char* num_map[] = {"1", "2", "3", "4", ""};

void lv_display(void) {
    // Auton Type Selector
    lv_obj_t* btnm1 = lv_btnmatrix_create(lv_scr_act());
    lv_btnmatrix_set_map(btnm1, type_map);
    lv_btnmatrix_set_btn_width(btnm1, 0, 2);
    lv_btnmatrix_set_btn_width(btnm1, 1, 2);
    lv_btnmatrix_set_btn_width(btnm1, 2, 2);
    lv_btnmatrix_set_btn_ctrl_all(btnm1, LV_BTNMATRIX_CTRL_CHECKABLE);
    lv_btnmatrix_set_one_checked(btnm1, true);

    lv_obj_align(btnm1, LV_ALIGN_TOP_LEFT, 10, 30);
    lv_obj_add_event_cb(btnm1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_set_content_width(btnm1, 200);
    lv_obj_set_content_height(btnm1, 40);

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
    lv_obj_align(bkgd, LV_ALIGN_BOTTOM_RIGHT, -30,-20);
    lv_obj_set_width(bkgd, 180);
    lv_obj_set_height(bkgd, 180);
    lv_obj_move_background(bkgd);

    curr_position_dot.init(20,20,lv_palette_lighten(LV_PALETTE_BLUE,0));
}  

void knights::display::set_pos_label(std::string str) {
    lv_label_set_text(pos_label, str.c_str());
}
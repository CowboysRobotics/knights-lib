#include "knights/robot/position_tracker.h"

using namespace knights;

Position_Tracker::Position_Tracker(pros::Rotation *rotation, float wheel_diameter, float gear_ratio, float offset) {
    this->rotation = rotation;
    this->wheel_diameter = wheel_diameter;
    this->gear_ratio = gear_ratio;
    this->offset = offset;
};

Position_Tracker::Position_Tracker(pros::ADIEncoder *adi_encoder, float wheel_diameter, float gear_ratio, float offset) {
    this->adi_encoder = adi_encoder;
    this->wheel_diameter = wheel_diameter;
    this->gear_ratio = gear_ratio;
    this->offset = offset;
};

Position_Tracker::Position_Tracker(pros::Motor *motor, float wheel_diameter, float gear_ratio, float offset) {
    this->motor = motor;
    this->wheel_diameter = wheel_diameter;
    this->gear_ratio = gear_ratio;
    this->offset = offset;
};

float Position_Tracker::get_distance_travelled() {
    if (this->rotation != NULL) {
        return this->rotation->get_position() * ((this->wheel_diameter * this->gear_ratio * M_PI) / 360);
    } else if (this->adi_encoder != NULL) {
        return this->adi_encoder->get_value() * ((this->wheel_diameter * this->gear_ratio * M_PI) / 360);
    } else if (this->motor != NULL) {
        return this->motor->get_position() * ((this->wheel_diameter * this->gear_ratio * M_PI) / 360);
    } else {
        return 0.0;
    }
}

Position_Tracker_Group::Position_Tracker_Group(knights::Position_Tracker *right, knights::Position_Tracker *left, knights::Position_Tracker *front, knights::Position_Tracker *back) {
    this->right_tracker = right;
    this->left_tracker = left;
    this->front_tracker = front;
    this->back_tracker = back;
};

Position_Tracker_Group::Position_Tracker_Group(knights::Position_Tracker *right, knights::Position_Tracker *left, knights::Position_Tracker *back) {
    this->right_tracker = right;
    this->left_tracker = left;
    this->back_tracker = back;
};

Position_Tracker_Group::Position_Tracker_Group(knights::Position_Tracker *right, knights::Position_Tracker *left) {
    this->right_tracker = right;
    this->left_tracker = left;
};

void Position_Tracker::reset() {
    if (this->rotation != NULL) {
        this->rotation->reset_position();
        return;
    } else if (this->adi_encoder != NULL) {
        this->adi_encoder->reset();
        return;
    } else if (this->motor != NULL) {
        this->motor->set_zero_position(0.0);
        return;
    }
}

float Position_Tracker::get_offset() {
    return this->offset;
}
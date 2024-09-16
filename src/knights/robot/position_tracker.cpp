#include "knights/robot/position_tracker.h"
#include "knights/util/calculation.h"

using namespace knights;

PositionTracker::PositionTracker(pros::Rotation *rotation, float wheel_diameter, float gear_ratio, float offset, int direction) 
    : rotation(rotation), wheel_diameter(wheel_diameter), gear_ratio(gear_ratio), offset(offset), direction(direction) {
};

PositionTracker::PositionTracker(pros::adi::Encoder *adi_encoder, float wheel_diameter, float gear_ratio, float offset, int direction) 
    : adi_encoder(adi_encoder), wheel_diameter(wheel_diameter), gear_ratio(gear_ratio), offset(offset), direction(direction) {
};

PositionTracker::PositionTracker(pros::Motor *motor, float wheel_diameter, float gear_ratio, float offset, int direction) 
    : motor(motor), wheel_diameter(wheel_diameter), gear_ratio(gear_ratio), offset(offset), direction(direction) {
};

float PositionTracker::get_distance_travelled() {
    if (this->rotation != NULL) {
        return knights::signum(this->direction) * this->rotation->get_position() * ((this->wheel_diameter * this->gear_ratio * M_PI) / 36000); // this works in centidegrees
    } else if (this->adi_encoder != NULL) {
        return knights::signum(this->direction) * this->adi_encoder->get_value() * ((this->wheel_diameter * this->gear_ratio * M_PI) / 360);
    } else if (this->motor != NULL) {
        return knights::signum(this->direction) * this->motor->get_position() * ((this->wheel_diameter * this->gear_ratio * M_PI) / 360);
    } else {
        return 0.0;
    }
}

PositionTrackerGroup::PositionTrackerGroup(knights::PositionTracker *right, knights::PositionTracker *left, knights::PositionTracker *front, knights::PositionTracker *back)
    : right_tracker(right), left_tracker(left), front_tracker(front), back_tracker(back) {
}

PositionTrackerGroup::PositionTrackerGroup(knights::PositionTracker *right, knights::PositionTracker *left, knights::PositionTracker *back)
    : right_tracker(right), left_tracker(left), back_tracker(back) {
}

PositionTrackerGroup::PositionTrackerGroup(knights::PositionTracker *right, knights::PositionTracker *left)
    : right_tracker(right), left_tracker(left) {
}

PositionTrackerGroup::PositionTrackerGroup(knights::PositionTracker *middle, knights::PositionTracker *back, pros::IMU *inertial)
    : right_tracker(middle), back_tracker(back), inertial(inertial) {
}

void PositionTracker::reset() {
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

float PositionTracker::get_offset() {
    return this->offset;
}
#include "knights/robot/position_tracker.hpp"
#include "knights/util/calculation.hpp"

#define SENSOR_MAX_DIST 72 // around 1.8 meters
#define SENSOR_MIN_DIST 1 // around 20 millimeters

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

DistanceTracker::DistanceTracker(pros::Distance *distance, float x_displacement, float y_displacement, float angle_from_front, float max_effective_mm)
    : distance_sensor(distance), x_displacement(x_displacement), y_displacement(y_displacement), angle_from_front(angle_from_front), max_effective_mm(max_effective_mm) {
}


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

PositionTrackerGroup::PositionTrackerGroup(knights::PositionTracker *right, knights::PositionTracker *left, knights::PositionTracker *front, knights::PositionTracker *back, 
    float blend_trust, float tracking_wheel_weight, float distance_sensor_weight)
    : right_tracker(right), left_tracker(left), front_tracker(front), back_tracker(back), blend_trust(blend_trust), 
    tracking_wheel_weight(tracking_wheel_weight), distance_sensor_weight(distance_sensor_weight) {
}

PositionTrackerGroup::PositionTrackerGroup(knights::PositionTracker *right, knights::PositionTracker *left, knights::PositionTracker *back, float blend_trust, float tracking_wheel_weight, float distance_sensor_weight)
    : right_tracker(right), left_tracker(left), back_tracker(back), blend_trust(blend_trust), 
    tracking_wheel_weight(tracking_wheel_weight), distance_sensor_weight(distance_sensor_weight) {
}

PositionTrackerGroup::PositionTrackerGroup(knights::PositionTracker *right, knights::PositionTracker *left, float blend_trust, float tracking_wheel_weight, float distance_sensor_weight)
    : right_tracker(right), left_tracker(left), blend_trust(blend_trust), 
    tracking_wheel_weight(tracking_wheel_weight), distance_sensor_weight(distance_sensor_weight) {
}

PositionTrackerGroup::PositionTrackerGroup(knights::PositionTracker *middle, knights::PositionTracker *back, pros::IMU *inertial, float blend_trust, float tracking_wheel_weight, float distance_sensor_weight)
    : right_tracker(middle), back_tracker(back), inertial(inertial), blend_trust(blend_trust), 
    tracking_wheel_weight(tracking_wheel_weight), distance_sensor_weight(distance_sensor_weight) {
}

void PositionTrackerGroup::add_dist(knights::DistanceTracker *tracker) {
    this->distance_trackers.emplace_back(tracker);
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
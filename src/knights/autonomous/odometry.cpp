#include "api.h"

#include "knights/robot/chassis.h"
#include "knights/robot/drivetrain.h"
#include "knights/robot/position_tracker.h"

#include "knights/util/calculation.h"

void knights::RobotChassis::update_position() {

    float deltaRight, deltaLeft, deltaFront, deltaBack;

    float newHeading, averageHeading, deltaHeading, deltaYOffset;

    float deltaX, deltaY, localX, localY;

    if (this->pos_trackers->right_tracker != nullptr) {
        deltaRight = this->pos_trackers->right_tracker->get_distance_travelled() - this->prevRight;
        this->prevRight = this->pos_trackers->right_tracker->get_distance_travelled(); 
    }
    if (this->pos_trackers->left_tracker != nullptr) {
        deltaLeft = this->pos_trackers->left_tracker->get_distance_travelled() - this->prevLeft;
        this->prevLeft = this->pos_trackers->left_tracker->get_distance_travelled(); 
    }
    if (this->pos_trackers->front_tracker != nullptr) {
        deltaFront = this->pos_trackers->front_tracker->get_distance_travelled() - this->prevFront;
        this->prevFront = this->pos_trackers->front_tracker->get_distance_travelled(); 
    }
    if (this->pos_trackers->back_tracker != nullptr) {
        deltaBack = this->pos_trackers->back_tracker->get_distance_travelled() - this->prevBack;
        this->prevBack = this->pos_trackers->back_tracker->get_distance_travelled();
    }

    // printf("right odom: %lf %lf\n", this->pos_trackers->right_tracker->get_distance_travelled(), this->prevRight);
    // printf("imu %lf %lf\n", this->pos_trackers->inertial->get_heading(), prev_position.heading);

    if (deltaRight && deltaLeft) {
        deltaHeading = ((deltaLeft - deltaRight)/(this->pos_trackers->right_tracker->get_offset() + this->pos_trackers->left_tracker->get_offset()));
        newHeading = curr_position.heading - deltaHeading;
    } else if (this->pos_trackers->inertial != nullptr) {
        // // old 11.15
        // newHeading = knights::normalize_angle(-knights::to_rad(this->pos_trackers->inertial->get_heading()-180), true);
        
        // new 11.15
        newHeading = knights::normalize_angle((knights::to_rad(-this->pos_trackers->inertial->get_heading())), true);
        
        deltaHeading = newHeading - prev_position.heading;
    }

    if (std::isnan(newHeading) || std::isinf(newHeading)) 
        return;

    averageHeading = normalize_angle(newHeading - (deltaHeading / 2), true);

    // calculate change in x and y
    if (this->pos_trackers->right_tracker != nullptr && this->pos_trackers->left_tracker != nullptr) {
        deltaY = (deltaRight+deltaLeft)/2;
        deltaYOffset = (this->pos_trackers->right_tracker->get_offset() + this->pos_trackers->left_tracker->get_offset())/2;
    } else if (this->pos_trackers->right_tracker != nullptr) {
        deltaY = deltaRight;
        deltaYOffset = this->pos_trackers->right_tracker->get_offset();
    } else if (this->pos_trackers->left_tracker != nullptr) {
        deltaY = deltaLeft;
        deltaYOffset = this->pos_trackers->left_tracker->get_offset();
    }

    deltaX = deltaBack;
    // deltaY = deltaRight; // using right wheel for the vertical tracking wheel

    this->prev_position = curr_position;

    // check if moving straight or curved
    if (deltaHeading == 0) {
        // straight
        localX = deltaX;
        localY = deltaY;
    }
    else {
        const float cnst = 2 * sin(deltaHeading / 2);
        // curved
        localX = cnst * (deltaX / deltaHeading + this->pos_trackers->back_tracker->get_offset());
        localY = cnst * (deltaY / deltaHeading + deltaYOffset); // using right wheel for vertical tracking
    }
    // printf("lx+y %lf %lf, dPos %lf %lf %lf\n", localX, localY, deltaX, deltaY, deltaHeading);

    // // using old bot
    // // calculate global x
    // curr_position.x += localX * sin(averageHeading) + localY * -cos(averageHeading);
    // // calculate global y
    // curr_position.y += localX * cos(averageHeading) + localY * -sin(averageHeading);

    // using new bot
    // calculate global x
    curr_position.x += localX * -sin(averageHeading) + localY * cos(averageHeading);
    // calculate global y
    curr_position.y += localX * cos(averageHeading) + localY * sin(averageHeading);

    this->curr_position.heading = newHeading;
}

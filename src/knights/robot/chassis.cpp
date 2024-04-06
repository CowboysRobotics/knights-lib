#include "api.h"

#include "knights/autonomous/odometry.h"
#include "knights/robot/chassis.h"
#include "knights/robot/drivetrain.h"
#include "knights/robot/position_tracker.h"

knights::Robot_Chassis::Robot_Chassis(Drivetrain *drivetrain, Position_Tracker_Group *pos_trackers, pros::IMU *inertial) {
    this->drivetrain = drivetrain;
    this->pos_trackers = pos_trackers;
    this->inertial = inertial;
}

void knights::Robot_Chassis::set_position(float x, float y, float heading) {
    this->curr_position.x = x;
    this->curr_position.y = y;
    this->curr_position.heading = heading;
}

void knights::Robot_Chassis::update_position_loop(bool &odomEnabled) {

    float deltaRight, deltaLeft, deltaFront, deltaBack;

    float prevRight = this->pos_trackers->right_tracker->get_distance_travelled(); 
    float prevLeft = this->pos_trackers->left_tracker->get_distance_travelled(); 
    float prevFront = this->pos_trackers->front_tracker->get_distance_travelled(); 
    float prevBack = this->pos_trackers->back_tracker->get_distance_travelled();

    float newHeading, averageHeading;

    float deltaX, deltaY, localX, localY;

    while (odomEnabled) {
        
        if (this->pos_trackers->right_tracker != nullptr) {
            deltaRight = this->pos_trackers->right_tracker->get_distance_travelled() - prevRight;
        }
        if (this->pos_trackers->left_tracker != nullptr) {
            deltaLeft = this->pos_trackers->left_tracker->get_distance_travelled() - prevLeft;
        }
        if (this->pos_trackers->front_tracker != nullptr) {
            deltaFront = this->pos_trackers->front_tracker->get_distance_travelled() - prevFront;
        }
        if (this->pos_trackers->back_tracker != nullptr) {
            deltaBack = this->pos_trackers->back_tracker->get_distance_travelled() - prevBack;
        }

        newHeading = curr_position.heading - ((deltaLeft - deltaRight)/(this->pos_trackers->right_tracker->get_offset() + this->pos_trackers->left_tracker->get_offset()));
        averageHeading = fmod(curr_position.heading + ((newHeading - curr_position.heading) / 2), 2.0 * M_PI);

        // calculate change in x and y
        deltaX = deltaBack;
        deltaY = deltaRight; // using right wheel for the vertical tracking wheel

        // check if moving straight or curved
        if (fabs(newHeading - curr_position.heading) == 0) {
            // straight
            // can't use other code b/c floating point exception
            localX = deltaX;
            localY = deltaY;
        }
        else {
            // curved - law of cosines
            localX = 2 * sin((newHeading - curr_position.heading) / 2) * (deltaX / (newHeading - curr_position.heading) + this->pos_trackers->back_tracker->get_offset());
            localY = 2 * sin((newHeading - curr_position.heading) / 2) * (deltaY / (newHeading - curr_position.heading) + this->pos_trackers->right_tracker->get_offset()); // using right wheel for vertical tracking
        }

        // calculate global x
        curr_position.x += localY * sin(averageHeading);
        curr_position.x += localX * -cos(averageHeading);

        // calculate global y
        curr_position.y += localY * cos(averageHeading);
        curr_position.y += localX * sin(averageHeading);

        curr_position.heading = newHeading;

        pros::delay(10);
    }
}
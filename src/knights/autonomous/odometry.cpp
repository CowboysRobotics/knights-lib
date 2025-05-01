#include "api.h"

#include "knights/logger/logger.hpp"
#include "knights/robot/chassis.hpp"
#include "knights/robot/drivetrain.hpp"
#include "knights/robot/position_tracker.hpp"

#include "knights/util/calculation.hpp"
#include "knights/util/position.hpp"
#include "pros/rtos.hpp"

#include <cmath>
#include <fstream>

#define SENSOR_MIN_DIST 1 // around 20 millimeters
#define WALL_DIST 72 // vex distance from center of field to wall

knights::Pos knights::RobotChassis::calc_tracking_wheel_position() {

    float deltaRight, deltaLeft, deltaFront, deltaBack;

    float newHeading, averageHeading, deltaHeading, deltaYOffset;

    float deltaX, deltaY, localX, localY;

    if (this->pos_trackers->right_tracker != nullptr) {
        // printf("found right\n");
        deltaRight = this->pos_trackers->right_tracker->get_distance_travelled() - this->prevRight;
        this->prevRight = this->pos_trackers->right_tracker->get_distance_travelled(); 
    }
    if (this->pos_trackers->left_tracker != nullptr) {
        // printf("found left\n");
        deltaLeft = this->pos_trackers->left_tracker->get_distance_travelled() - this->prevLeft;
        this->prevLeft = this->pos_trackers->left_tracker->get_distance_travelled(); 
    }
    if (this->pos_trackers->front_tracker != nullptr) {
        // printf("found front\n");
        deltaFront = this->pos_trackers->front_tracker->get_distance_travelled() - this->prevFront;
        this->prevFront = this->pos_trackers->front_tracker->get_distance_travelled(); 
    }
    if (this->pos_trackers->back_tracker != nullptr) {
        // printf("found back\n");
        deltaBack = this->pos_trackers->back_tracker->get_distance_travelled() - this->prevBack;
        this->prevBack = this->pos_trackers->back_tracker->get_distance_travelled();
    }


    if (this->pos_trackers->inertial != nullptr) {
        newHeading = knights::normalize_angle((knights::to_rad(-this->pos_trackers->inertial->get_heading())), true);
        
        deltaHeading = newHeading - prev_position.heading;
    }
    else if (deltaRight && deltaLeft) {
        deltaHeading = ((deltaLeft - deltaRight)/(this->pos_trackers->right_tracker->get_offset() + this->pos_trackers->left_tracker->get_offset()));
        newHeading = curr_position.heading - deltaHeading;
    }

    if (std::isnan(newHeading) || std::isinf(newHeading)) 
        return prev_position;

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

    knights::Pos estimated_position;

    // calculate global x
    estimated_position.x = curr_position.x + localX * -sin(averageHeading) + localY * cos(averageHeading);
    // calculate global y
    estimated_position.y = curr_position.y + localX * cos(averageHeading) + localY * sin(averageHeading);

    estimated_position.heading = newHeading;

    return estimated_position;

}

std::fstream write_file("/usd/sensor.txt", std::ios_base::out);


std::tuple<knights::Pos, knights::Point> knights::RobotChassis::calc_distance_sensor_position() {
    
    std::vector<float> x_estimates;
    std::vector<float> y_estimates;

    float robot_theta;

    if (this->pos_trackers->inertial != nullptr) {
        robot_theta = (knights::to_rad(-this->pos_trackers->inertial->get_heading()));
    } else {
        robot_theta = this->curr_position.heading;
    }

    for (auto sensor : this->pos_trackers->distance_trackers) {

        float sensor_distance = knights::to_inches(sensor->distance_sensor->get_distance() / 1000.0);
    

        float sensor_angle_rad = knights::normalize_angle(sensor->angle_from_front + robot_theta, true);
        
        knights::Point hit_pos(
            sensor_distance * std::cos(sensor_angle_rad) + this->curr_position.x
              + (-sensor->x_displacement * -std::sin(robot_theta) + sensor->y_displacement * std::cos(robot_theta)),
              sensor_distance * std::sin(sensor_angle_rad) + this->curr_position.y
              + (-sensor->x_displacement * std::cos(robot_theta) + sensor->y_displacement * std::sin(robot_theta))
        );

        // std::cout << 
        // knights::logger::string_format(
        //     "s_dist: %lf hit pos: %lf %lf \n", sensor_distance, hit_pos.x, hit_pos.y
        // );

        if (sensor_distance > knights::to_inches(sensor->max_effective_mm/1000.0) || sensor_distance < SENSOR_MIN_DIST) 
            continue;

        if (std::fabs(hit_pos.x) < WALL_DIST - 10 && std::fabs(hit_pos.y) < WALL_DIST - 10)
            continue;

        if (hit_pos.x > 0 and std::fabs(hit_pos.x) > std::fabs(hit_pos.y)) {
            float x_dist = std::cos(sensor_angle_rad) * sensor_distance + (-sensor->x_displacement * -std::sin(robot_theta) + sensor->y_displacement * std::cos(robot_theta));
            x_estimates.push_back(WALL_DIST - x_dist);

        } else if (hit_pos.y > 0 and std::fabs(hit_pos.y) > std::fabs(hit_pos.x)) {
            float y_dist = std::sin(sensor_angle_rad) * sensor_distance + (-sensor->x_displacement * std::cos(robot_theta) + sensor->y_displacement * std::sin(robot_theta));
            y_estimates.push_back(WALL_DIST - y_dist);

        } else if (hit_pos.x < 0 and std::fabs(hit_pos.x) > std::fabs(hit_pos.y)) {
            float x_dist = std::cos(sensor_angle_rad) * sensor_distance + (-sensor->x_displacement * -std::sin(robot_theta) + sensor->y_displacement * std::cos(robot_theta));
            x_estimates.push_back(-WALL_DIST - x_dist);

        } else if (hit_pos.y < 0 and std::fabs(hit_pos.y) > std::fabs(hit_pos.x)) {
            float y_dist = std::sin(sensor_angle_rad) * sensor_distance + (-sensor->x_displacement * std::cos(robot_theta) + sensor->y_displacement * std::sin(robot_theta));
            y_estimates.push_back(-WALL_DIST - y_dist);
        }



    }

    // std::cout << 
    //     knights::logger::string_format(
    //         "x_est, size: %lf %d y_est, size: %lf %d \n", knights::avg(x_estimates), x_estimates.size(), knights::avg(y_estimates), y_estimates.size()
    //     );

    knights::Pos estimated_position(0.0, 0.0, robot_theta);

    if (x_estimates.size() > 0) {
        estimated_position.x = knights::avg(x_estimates);
    }
    if (y_estimates.size() > 0) {
        estimated_position.y = knights::avg(y_estimates);
    } 


    return std::make_tuple(estimated_position, knights::Point(x_estimates.size()/2.0, y_estimates.size()/2.0));

}


void knights::RobotChassis::update_position() {

    // need new average of dist sensor and tracking wheel

    if (this->current_localization_method == knights::LocalizationMethod::NONE)
        return;
    else if (this->current_localization_method == knights::LocalizationMethod::DISTANCE_SENSOR) {
        auto [dist_sensor_estimate, accuracy] = calc_distance_sensor_position();
        this->prev_position = curr_position;
        this->curr_position = dist_sensor_estimate;
    }
    else if (this->current_localization_method == knights::LocalizationMethod::TRACKING_WHEEL) {
        knights::Pos tracking_wheel_estimate = calc_tracking_wheel_position();
        this->prev_position = curr_position;
        this->curr_position = tracking_wheel_estimate;
    }
    else {
        knights::Pos tracking_wheel_estimate = calc_tracking_wheel_position();
        auto [dist_sensor_estimate, accuracy] = calc_distance_sensor_position();

        knights::Pos best_estimation;

        // calculations here

        // check if one sensor is very off

        bool tracking_wheel_valid = distance_btwn(tracking_wheel_estimate, prev_position) < this->drivetrain->max_velocity()/20;
        bool distance_sensor_valid = distance_btwn(dist_sensor_estimate, prev_position) < this->drivetrain->max_velocity()/20;

        if (!tracking_wheel_valid && !distance_sensor_valid) {
            best_estimation = prev_position;
        }
        else if (!tracking_wheel_valid) {
            best_estimation = dist_sensor_estimate;
        }
        else if (!distance_sensor_valid) {
            best_estimation = tracking_wheel_estimate;
        }
        else { // both valid

            // predict pos
            knights::Pos predicted_pos;

            float predicted_theta = (prev_position.heading + tracking_wheel_estimate.heading) / 2;
            float delta_time = ((pros::millis() - last_estimate_time) / 1000);

            predicted_pos.x = prev_position.x + curr_velocity * std::cos(predicted_theta) * delta_time;
            predicted_pos.y = prev_position.y + curr_velocity * std::sin(predicted_theta) * delta_time;

            // combine positions
            knights::Pos combined_sensor_pos;
            combined_sensor_pos.x = 
                this->pos_trackers->tracking_wheel_weight * tracking_wheel_estimate.x + 
                this->pos_trackers->distance_sensor_weight * accuracy.x * dist_sensor_estimate.x;

            combined_sensor_pos.x /= this->pos_trackers->tracking_wheel_weight + this->pos_trackers->distance_sensor_weight * accuracy.x;

            combined_sensor_pos.y = 
            this->pos_trackers->tracking_wheel_weight * tracking_wheel_estimate.y + 
            this->pos_trackers->distance_sensor_weight * accuracy.y * dist_sensor_estimate.y;

            combined_sensor_pos.y /= this->pos_trackers->tracking_wheel_weight + this->pos_trackers->distance_sensor_weight * accuracy.y;

            // Complementary Filter blending equation:
            // New Best Estimate = (1 - alpha) * Predicted Position + alpha * Combined Sensor Position
            best_estimation.x = (1.0 - this->pos_trackers->blend_trust) * predicted_pos.x + this->pos_trackers->blend_trust * combined_sensor_pos.x;
            best_estimation.y = (1.0 - this->pos_trackers->blend_trust) * predicted_pos.y + this->pos_trackers->blend_trust * combined_sensor_pos.y;
            best_estimation.heading = tracking_wheel_estimate.heading;

        }

        this->prev_position = curr_position;
        this->curr_position = best_estimation;
    }

    curr_velocity = distance_btwn(curr_position, prev_position) / ((pros::millis() - last_estimate_time) / 1000); 
    last_estimate_time = pros::millis();
}

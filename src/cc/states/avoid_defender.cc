//
// Created by hrs_b on 10.01.19.
//

#include "avoid_defender.h"
#include "detect_hoop.h"
#include "../core/controller.h"
#include "math.h"

AvoidDefenderState::AvoidDefenderState(std::string& task):
    task(task)
{

}

void AvoidDefenderState::go_next(Controller &controller) {

    ROS_INFO_STREAM("Avoiding defender.");

    // 1. determine the span (ie distance between the marker and the most exterior surface) of the defender  with color segmentation
    ros::Rate loop_rate(10);
    size_t attempts = 0;
    loop_rate.sleep();
    ros::spinOnce();
    if (not controller.vision_module().defender_visible()) {
        SEARCH_MODE mode = SEARCH_MODE::VERY_SIMPLE_SEARCH;
        float found_at_head_angle;
        std::function<bool ()> f = [&]() { return controller.vision_module().defender_visible(); };
        bool defender_visible = controller.brain().search().search_routine(
                mode,
                f,
                found_at_head_angle
        );
        if (defender_visible) {
            controller.motion_module().request_move_to_position(0.0, 0.0, found_at_head_angle);

        }
    }

    float span = 0.0f;
    float yaw = controller.vision_module().get_defender_rotation();
    int orientation = (yaw > 0) - (yaw < 0); //1 if yaw is postive, -1 else


    /*while (true){
        ROS_INFO_STREAM(controller.vision_module().get_defender_rotation());
        ros::spinOnce();
        loop_rate.sleep();
    }*/

    while (attempts < 30) { // Take 3 seconds to look for the defender
    //while (true) {

        span = controller.vision_module().detect_defender_span();
        //ROS_INFO_STREAM(span);
        ros::spinOnce();
        loop_rate.sleep();
        if (span > 0.10f or span < -0.10f) break;
        attempts++;
    }
    if (span < 0.10f and span > -0.10f){
        span = orientation * 0.40f;
    }

    ROS_INFO_STREAM("The defender has a span of: " + std::to_string(span));
    ROS_INFO_STREAM("The defender is at angle of : " + std::to_string(yaw));
    //if (span == 0.0f) span = 0.40f; //if the span of the defender cannot be estimated, the robot considers the defender to be large
    // 2. Use Vision Module to get location of the defender
    auto defender_position = controller.brain().vision_module().get_marker_mat_t_defender();
    auto marker_x = defender_position.at<float>(0);
    auto marker_z = defender_position.at<float>(2);

    // 3. Use Motion Module to walk towards defender
    float walking_x = marker_z - 0.40f; // Stop 40 cm in front of the defender
    auto walking_angle = (float)tan((double)marker_x/(double)marker_z);

    ROS_INFO_STREAM("Walking: " + std::to_string(walking_x));
    ROS_INFO_STREAM("Turning: " + std::to_string(walking_angle));

    controller.motion_module().request_move_to_position(0.0, 0.0, -walking_angle);
    controller.motion_module().request_move_to_position(walking_x, 0.0, 0.0);

    // 3. Walk around defender using by succession of 90 degree angles
    ROS_INFO_STREAM("Walking around defender ");

    int direction = (span >= 0) - (span < 0); //1 if span is postive, -1 else
    ROS_INFO_STREAM("Direction:" + std::to_string(direction));
    ROS_INFO_STREAM("Span is: " + std::to_string(span));

    controller.motion_module().request_move_to_position(0.0, 0.0, 0.8f*6.0f * yaw);
    ROS_INFO_STREAM("Turning: " + std::to_string(0.8f*6.0f * yaw));

    controller.motion_module().request_move_to_position(0.0, -0.8f*direction*(abs(span) + 0.25f), 0.0); //side steps
    ROS_INFO_STREAM("Side steps: " + std::to_string(-0.8f*direction*(abs(span) + 0.25f)));

    ROS_INFO_STREAM("WALK PAST");
    //controller.motion_module().request_move_to_position(0.0, 0.0, -direction * 3.1415f * 0.15f); //45 degree angle
    controller.motion_module().request_move_to_position(0.8f * span * direction + 0.2f, 0.0, 0.0); //the multiplication by direction ensures span is always positive

    ROS_INFO_STREAM("TURN TO THE HOOP");
    controller.motion_module().request_move_to_position(0.0, 0.0, direction * 3.1415f * 0.25f);
    //controller.motion_module().request_move_to_position(0.60f, 0.0, 0.0);

    controller.set_state(new DetectHoopState(task));
}


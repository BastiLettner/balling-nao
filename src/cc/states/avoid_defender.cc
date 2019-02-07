//
// Created by hrs_b on 10.01.19.
//

#include "avoid_defender.h"
#include "detect_hoop.h"
#include "../core/controller.h"

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
    float span = 0.0f;
    while (attempts < 30) { // Take 3 seconds to look for the defender

        span = controller.vision_module().detect_defender_span();
        ros::spinOnce();
        loop_rate.sleep();
        if (span > 0.10f or span < -0.10f) break;
        attempts++;
    }
    if (span < 0.10f and span > -0.10f){
        span = 0.40f;
    }
    ROS_INFO_STREAM("The defender has a span of: " + std::to_string(span));
    //if (span == 0.0f) span = 0.40f; //if the span of the defender cannot be estimated, the robot considers the defender to be large
    // 2. Use Vision Module to get location of the defender
    auto defender_position = controller.brain().vision_module().get_marker_mat_t_defender();
    auto marker_x = defender_position.at<float>(0);
    auto marker_z = defender_position.at<float>(2);

    // 3. Use Motion Module to walk towards defender
    float walking_x = marker_z - 0.60f; // Stop 40 cm in front of the defender
    auto walking_angle = (float)tan((double)marker_x/(double)marker_z);

    ROS_INFO_STREAM("Walking: " + std::to_string(walking_x));
    ROS_INFO_STREAM("Turning: " + std::to_string(walking_angle));

    controller.motion_module().request_move_to_position(0.0, 0.0, -walking_angle);
    controller.motion_module().request_move_to_position(walking_x, 0.0, 0.0);

    // 3. Walk around defender using by succession of 90 degree angles
    ROS_INFO_STREAM("Walking around defender ");

    int direction = (span > 0) - (span < 0); //1 if span is postive, -1 else

    ROS_INFO_STREAM("SIDE STEP");
    controller.motion_module().request_move_to_position(0.0, -direction * span * 1.5f, 0.0); //side steps
    ROS_INFO_STREAM("WALK PAST");
    //controller.motion_module().request_move_to_position(0.0, 0.0, -direction * 3.1415f * 0.15f); //45 degree angle
    controller.motion_module().request_move_to_position(span * direction, 0.0, 0.0); //the multiplication by direction ensures span is always positive

    ROS_INFO_STREAM("TURN TOP THE HOOP");
    controller.motion_module().request_move_to_position(0.0, 0.0, direction * 3.1415f * 0.25f);
    //controller.motion_module().request_move_to_position(0.60f, 0.0, 0.0);

    controller.set_state(new DetectHoopState(task));
}


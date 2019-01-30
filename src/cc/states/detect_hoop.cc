//
// Created by hrs_b on 10.01.19.
//

#include "detect_hoop.h"
#include "../modules/controller.h"

DetectHoopState::DetectHoopState():
    State()
{

}

void DetectHoopState::go_next(Controller &controller) {

    ros::Rate loop_rate(10);
    bool hoop_detected = false;
    while(!hoop_detected) {
        hoop_detected = controller.vision_module().hoop_visible();
        // TODO Slightly move the head or maybe better the body?
        ros::spinOnce();
        loop_rate.sleep();
    }
    cv::Mat hoop = controller.vision_module().get_marker_mat_t_hoop();
    std::vector<float> converter_point(3);
    converter_point[0] = hoop.at<float>(0);
    converter_point[1] = hoop.at<float>(1);
    converter_point[2] = hoop.at<float>(2);

    //auto transformed_point = controller.motion_module.get_position_relative_torso(converter_point);


    while(true) {
        ROS_INFO_STREAM("hoop visible");
        ros::spinOnce();
        loop_rate.sleep();
    }

}


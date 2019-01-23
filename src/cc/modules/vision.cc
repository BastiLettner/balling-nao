//
// Created by hrs_b on 10.01.19.
//

#include "vision.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

Vision::Vision(ros::NodeHandle &node_handle) {
    image_transport::ImageTransport it(node_handle);
    image_transport::Subscriber image_sub;
    // Subscribe to input video feed
    image_sub = it.subscribe("/nao_robot/camera/top/camera/image_raw", 1,
                               &Vision::imageCb, this);
}

void Vision::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    // Receive image from Robot camera node and convert it to a open cv image type.
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // Display the original image
    cv::imshow("OriginalImage", cv_ptr->image);
}

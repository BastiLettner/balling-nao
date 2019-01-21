//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_VISION_H
#define BALLING_NAO_VISION_H

#include <ros/node_handle.h>
#include <ros/ros.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <aruco/aruco.h>
#include <tf/transform_broadcaster.h>


class Vision {

public:

    explicit Vision(ros::NodeHandle& node_handle);

    ~Vision() = default;

    Vision(Vision&& vision) = delete;

    // Tell the caller whether the ball is currently visible or not
    //
    // Returns
    //     true: Ball is visible
    //     false: Ball is not visible
    bool ball_visible() { return _ball_visible; }

private:

    bool _ball_visible = false;

    //aruco::CameraParameters _cam_params;
    //image_transport::Subscriber _image_sub;


};
#endif //BALLING_NAO_VISION_H

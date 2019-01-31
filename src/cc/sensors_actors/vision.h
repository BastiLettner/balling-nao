//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_VISION_H
#define BALLING_NAO_VISION_H

#include <ros/node_handle.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <tf/transform_broadcaster.h>

class Vision {

public:

    explicit Vision(ros::NodeHandle& node_handle);

    ~Vision() = default;

    Vision(Vision&& vision) = delete;

    // Callback for the raw image
    void image_reception(const sensor_msgs::ImageConstPtr& msg);


    // Tell the caller whether the ball is currently visible or not
    //
    // Returns
    //     true: Ball is visible
    //     false: Ball is not visible
    bool ball_visible();


    // Tell the caller whether the defender is visible
    //
    // Returns
    //     true: Defender visible
    //     false: Defender not visible
    //bool defender_visible();
    bool defender_visible(cv::Mat& position);

    // Tell the caller whether the hoop is visible
    //
    // Returns
    //     true: hoop visible
    //     false: hoop not visible
    bool hoop_visible();
    bool hoop_visible(cv::Mat& position);

    cv::Mat& get_current_image() { return _current_image; }
    cv::Mat& get_marker_mat_t_defender() { return _marker_mat_t_defender; }
    cv::Mat& get_marker_mat_t_hoop() { return _marker_mat_t_hoop; }


    void detect_marker();

    bool marker_visible();

    void tf_publisher(std::vector<float>& point, std::string parent_frame, std::string name);

private:

    bool _ball_visible = false;
    bool _defender_visible = false;
    bool _hoop_visible = false;


    image_transport::ImageTransport _it;
    image_transport::Subscriber _image_sub;
    aruco::CameraParameters _cam_params;

    //raw image retrieved by the camera
    cv::Mat _current_image;

    float _marker_size = 0.064;
    aruco::MarkerDetector _marker_detector;
    std::vector<aruco::Marker> _aruco_markers;

    cv::Mat _marker_mat_t_defender;
    cv::Mat _marker_mat_t_hoop;
    tf::TransformBroadcaster _broadcast;

};
#endif //BALLING_NAO_VISION_H

// Vision header file
// The vision module holds the functions using NAO's cameras
// structure : single file

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
    
    // The vision module.
    // Uses the on board cameras to anlyse the environment.


public:

    // Constructs object
    // - "rotation_server" (service)
    // - "/nao_robot/camera/top/camera/image_raw" (subscribe)
    // - Calibrates the top camera
    explicit Vision(ros::NodeHandle& node_handle);

    ~Vision() = default;

    Vision(Vision&& vision) = delete;

    // Callback for the raw image
    void image_reception(const sensor_msgs::ImageConstPtr& msg);

    // Tell the caller whether the ball is currently visible or not.
    // This is done using color segmentation.
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
    bool defender_visible();

    // Tell the caller whether the hoop is visible
    //
    // Returns
    //     true: hoop visible
    //     false: hoop not visible
    bool hoop_visible();

    // Getters:
    // Retrives the last frame obtained through the video feed.
    //
    // Return:
    //      OpenCV matrix of the frame
    cv::Mat& get_current_image() { return _current_image; }
    // Retrieves the translation matrix of the defender (resp. hoop) marker
    // Return:
    //      OPenCV matrix of the marker translation matrix
    cv::Mat& get_marker_mat_t_defender() { return _marker_mat_t_defender; }
    cv::Mat& get_marker_mat_t_hoop() { return _marker_mat_t_hoop; }

    // Detect markers in the current image
    // This function detects any marker present on the video feed
    // and stores their information in a vector of Marker objects
    void detect_marker();

    // Detects whether a marker is visible in video feed
    //
    // Return:
    //      True: at least one marker was detected
    //      False: No marker detected.
    bool marker_visible();

    // Broadcasts the relative poses of the coordinate frames of the markers.
    // This was used for debugging and not actually called in the main programme.
    void tf_publisher(std::vector<float>& point, std::string parent_frame, std::string name);

    // Calculates the half width of the defender.
    // This function retrieves the distance between the center of the aruco
    // marker on the defender and the most exterior part of the defender 
    // using color segmentation
    //
    // Return:
    //      Distance (in meters) between the middle and the edge of the defender
    float detect_defender_span();

    // Creates a binary image on which the orange parts of the defender are extracted.
    // The output matrix is white were the arms and shoulder of the defender are, black elsewhere
    //
    // Return:
    //      OpenCV binary matrix
    cv::Mat extract_defender();

    // Calculates the distance between the center of the defender marker and 
    // the most exterior white blob (defender body)
    // If the marker is on the right (resp. left), the first (resp. second) function is called
    // The distinction is made in order to maximize the odds that NAO does not bump into a part of the defender it cannot see.
    // If the marker is on the right side of the frame, it is expected that NAO
    // sees more of the left half of the rebot than the right half
    //
    // Args:
    //      binary_defender: binary matrix with the defender extracted
    //      center: position in pixels of center of the arcuo marker relative to the current frame
    //
    // Return:
    //      half wdith, in pixels. Distance between the marker and most exterior blob
    float distance_marker_right_blob(cv::Mat binary_defender, float center);
    float distance_marker_left_blob(cv::Mat binary_defender, float center);

    // Retrieves the roation of the defender thanks to the rotation of its marker.
    //
    // Return:
    //      Returns the angle (in rad) of the defender.
    float get_defender_rotation();
    
    // Calculates the yaw of a specified marker thanks to its Rvec matrix
    //
    // Args:
    //      rvec: vector of size 3 containing the elements of the Rvec matrix of an aruco marker
    //
    // Return:
    //      Yaw (in rad), of the marker
    float get_marker_yaw(std::vector<float>& rvec);

private:

    // booleans used to specify wether an object is detected on the current frame
    bool _ball_visible = false;
    bool _defender_visible = false;
    bool _hoop_visible = false;

    // Subscribtion and image processing
    image_transport::ImageTransport _it;
    image_transport::Subscriber _image_sub;
    aruco::CameraParameters _cam_params;
    
    // Client to convert rotation matrices to ratation vectors.
    ros::ServiceClient _client_get_rotation;

    // Raw image retrieved by the camera
    cv::Mat _current_image;

    // size of the marker cutouts, in meters
    float _marker_size = 0.1;
    
    // Marker detector object and vector of aruco markers
    aruco::MarkerDetector _marker_detector;
    std::vector<aruco::Marker> _aruco_markers;

    // Translation matrices of defender and hoop
    cv::Mat _marker_mat_t_defender;
    cv::Mat _marker_mat_t_hoop;
    
    // tf broadcaster
    tf::TransformBroadcaster _broadcast;


};
#endif //BALLING_NAO_VISION_H

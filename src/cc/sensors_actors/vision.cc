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

using namespace cv;

namespace NaoCameraParameters {

    cv::Mat dist = (
            cv::Mat_<double>(4, 1) <<
                                   -0.0870160932911717,
                    0.128210165050533,
                    0.003379500659424,
                    -0.00106205540818586
    );

    cv::Mat cameraP = (
            cv::Mat_<double>(3, 3) <<
                                   274.139508945831,
                    0.0,
                    141.184472810944,
                    0.0,
                    275.741846757374,
                    106.693773654172,
                    0.0,
                    0.0,
                    1.0
    );
}

int huelow = 0;
int huehigh = 70;
int erosion_size = 0;
int dilation_size = 0;
int ball_detection_area = 1800;

int hoop_id = 2;
int defender_id = 1;

Vision::Vision(ros::NodeHandle &node_handle): _it(node_handle) {
    Vision::_image_sub = _it.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &Vision::image_reception, this);
    _cam_params.setParams(NaoCameraParameters::cameraP, NaoCameraParameters::dist, Size(640, 480));
}

void Vision::image_reception(const sensor_msgs::ImageConstPtr &msg) {

    cv_bridge::CvImageConstPtr cvImagePointer;
    try {
        cvImagePointer = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        Vision::_current_image = cvImagePointer->image.clone();
        Vision::detect_marker();

    }
    catch (cv_bridge::Exception &except) {
        ROS_ERROR("cv_bridge exception: %s", except.what());
        return;
    }

    int const max_elem = 2;
    int const max_kernel_size = 21;

    namedWindow("HSV image");
    namedWindow("Current Image");
    namedWindow("Blob Extraction");
    namedWindow("TrackBars");
    createTrackbar( "Lower boundary of Hue", "TrackBars", &huelow, 255);
    createTrackbar( "Higher boundary of Hue", "TrackBars", &huehigh, 255);
    createTrackbar( "Erosion Kernel size:\n 2n +1", "TrackBars", &erosion_size, max_kernel_size);
    createTrackbar( "Dilation Kernel size:\n 2n +1", "TrackBars", &dilation_size, max_kernel_size);
    createTrackbar( "Size necessary for ball detection", "TrackBars", &ball_detection_area, 4000);

    waitKey(3);
}
/*This function tries to detect a colored ball by selecting a range of hue.
*It gets rids of the background noise by eroding and then dilating the binary picture optained after the isolation of the ball.
*It outputs true if the ball is visible, false otherwise
*/
bool Vision::ball_visible(){
    bool ball_detected = false;
    Mat eroded_image;
    Mat dilated_image;


    Mat HSVImage;
    cvtColor(Vision::_current_image, HSVImage, COLOR_BGR2HSV); //Conversion to HSV
    inRange(HSVImage, cv::Scalar(huelow, 50, 255), cv::Scalar(huehigh, 255, 255), HSVImage); //Color extraction

    Mat element_erosion = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
            cv::Point( erosion_size, erosion_size )
    );
    Mat element_dilation = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
            cv::Point( dilation_size, dilation_size )
    );
    erode(HSVImage, eroded_image, element_erosion);
    dilate(eroded_image, dilated_image, element_dilation);

    std::vector<std::vector<cv::Point> > contours; //vector of contours
    std::vector<Vec4i> hierarchy;
    Mat biggest_blob = dilated_image.clone();
    findContours(biggest_blob, contours, hierarchy, 2/*cv::CV_RETR_CCOMP*/, 2/*cv::CV_CHAIN_APPROX_SIMPLE*/);
    for( int i = 0; i < contours.size(); i++ ) // iterate through each contour.
    {
        double a = contourArea(contours[i], false);  //  Find the area of contour
        if (a > ball_detection_area) {
            ball_detected = true;
            break;
        }
    }

    imshow("HSV image", HSVImage);
    imshow("Current Image", Vision::_current_image);
    imshow("Blob Extraction", dilated_image);

    return ball_detected;
}

void Vision::detect_marker() {

    // Empty vector
    Vision::_aruco_markers.clear();
    Vision::_marker_detector.detect(Vision::_current_image, Vision::_aruco_markers);
    if (not Vision::_aruco_markers.empty()) {
        for(size_t i = 0; i < _aruco_markers.size(); i++) {
            Vision::_aruco_markers[i].calculateExtrinsics(Vision::_marker_size, Vision::_cam_params, true);
            Vision::_aruco_markers[i].draw(Vision::_current_image, cv::Scalar(0, 255, 255), 2);
        }
    }

    cv::imshow("Marker Image", Vision::_current_image);
}

bool Vision::marker_visible() {

    return !Vision::_aruco_markers.empty();

}


void Vision::tf_publisher(std::vector<float>& point, std::string parent_frame, std::string name) {

    // Publish the marker position to make it visible in rviz
    tf::Transform transform;
    transform.setOrigin(
            tf::Vector3(point[0], point[1], point[2])
    );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    Vision::_broadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, name));

}

bool Vision::hoop_visible() {
    for(size_t i = 0; i < _aruco_markers.size(); i++){
        if(_aruco_markers[i].id == hoop_id) {
            Vision::_marker_mat_t_hoop = Vision::_aruco_markers[i].Tvec;
            return true;
        }
    }
    return false;
}


bool Vision::hoop_visible(cv::Mat &position) {
    for(size_t i = 0; i < _aruco_markers.size(); i++){
        if(_aruco_markers[i].id == hoop_id) {
            Vision::_marker_mat_t_hoop = Vision::_aruco_markers[i].Tvec;
            position = Vision::_aruco_markers[i].Tvec;
            return true;
        }
    }
    return false;
};


bool Vision::defender_visible() {

    for(size_t i = 0; i < _aruco_markers.size(); i++){
        if(_aruco_markers[i].id == defender_id) {
            Vision::_marker_mat_t_defender = Vision::_aruco_markers[i].Tvec;
            return true;
        }
    }
    return false;
}


bool Vision::defender_visible(cv::Mat &position) {
    for(size_t i = 0; i < _aruco_markers.size(); i++){
        if(_aruco_markers[i].id == defender_id) {
            Vision::_marker_mat_t_defender = Vision::_aruco_markers[i].Tvec;
            position = Vision::_aruco_markers[i].Tvec;
            return true;
        }
    }
    return false;
}

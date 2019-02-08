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
#include "balling_nao/GetMarkerRotation.h"

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
int erosion_size = 2;
int dilation_size = 1;
int ball_detection_area = 1800;

int hoop_id = 1; //use chev.me/arucogen to generate aruco markers
int defender_id = 0;

Vision::Vision(ros::NodeHandle &node_handle): _it(node_handle) {
    _client_get_rotation = node_handle.serviceClient<balling_nao::GetMarkerRotation>("rotation_server");
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

    //namedWindow("Blob Extraction");
    namedWindow("Defender Blob");
    namedWindow("Defender detection");
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

    imshow("Blob Extraction", dilated_image);

    return ball_detected;
}

void Vision::detect_marker() {

    // Empty vector
    Vision::_aruco_markers.clear();
    Mat marker_image = Vision::_current_image.clone();
    Vision::_marker_detector.detect(marker_image, Vision::_aruco_markers);
    if (not Vision::_aruco_markers.empty()) {
        for(size_t i = 0; i < _aruco_markers.size(); i++) {
            Vision::_aruco_markers[i].calculateExtrinsics(Vision::_marker_size, Vision::_cam_params, true);
            Vision::_aruco_markers[i].draw(marker_image, cv::Scalar(0, 255, 255), 2);
        }
    }

    cv::imshow("Marker Image", marker_image);
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


bool Vision::defender_visible() {

    for(size_t i = 0; i < _aruco_markers.size(); i++){
        if(_aruco_markers[i].id == defender_id) {
            Vision::_marker_mat_t_defender = Vision::_aruco_markers[i].Tvec;
            return true;
        }
    }
    return false;
}





float Vision::detect_defender_span(){

    if (defender_visible()) {

        int image_width = Vision::_current_image.cols;
        aruco::Marker defender;
        for (size_t i = 0; i < _aruco_markers.size(); i++) {
            if (_aruco_markers[i].id == defender_id) {
                defender = _aruco_markers[i]; //stores the defender marker
            }
        }
        float perimeter = defender.getPerimeter();
        Point2f center = defender.getCenter();

        Mat extracted_defender = extract_defender(); //get the extracted defender thanks to color segmentation
        float span;

        if (center.x < image_width / 2.0f) {
            ROS_INFO_STREAM("Center X: " + std::to_string(center.x));
            //marker on the left, search right of the marker
            span = distance_marker_left_blob(extracted_defender, center.x);
        }
        else{
            //marker on the right, search left of the marker
            span = distance_marker_right_blob(extracted_defender, center.x);
        }

        span = span * Vision::_marker_size / (0.25f * perimeter); //gets the real distance between the marker and the most potruding part of the defender
        return span;

    }


}

Mat Vision::extract_defender(){

    // TODO change parameters to be unique and not conflict with ball detection
    Mat eroded_image;
    Mat dilated_image;
    Mat HSVImage;

    cvtColor(Vision::_current_image, HSVImage, COLOR_BGR2HSV); //Conversion to HSV
    inRange(HSVImage, cv::Scalar(huelow, 10, 180), cv::Scalar(huehigh, 200, 240), HSVImage); //Color extraction TODO change hue to correct values

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

    //erode(HSVImage, eroded_image, element_erosion);
    dilate(HSVImage, dilated_image, element_dilation);

    return dilated_image;
}

float Vision::distance_marker_right_blob(Mat binary_defender, float center){

    Mat image_orig = Vision::_current_image.clone();
    int screen_width = Vision::_current_image.cols;
    int min_blob_size = 20;
    Rect blob;
    int edge = center;
    float span = 0.0f;

    std::vector<std::vector<Point> > contours; //vector of contours
    std::vector<Vec4i> hierarchy;
    findContours(binary_defender, contours, hierarchy, 2/*cv::CV_RETR_CCOMP*/, 2/*cv::CV_CHAIN_APPROX_SIMPLE*/);

    for( int i = 0; i < contours.size(); i++ ) // iterate through each contour.
    {
        double a = cv::contourArea( contours[i], false);  //  Find the area of contour
        if(a > min_blob_size) {
            blob = boundingRect(contours[i]);
            if (blob.x < edge) {
                edge = blob.x;
                span = (float)edge - center;
                rectangle(binary_defender, blob, Scalar(255, 255, 255));
                rectangle(image_orig, blob, Scalar(255, 0, 0));
            }
        }
    }
    imshow("Defender Blob", binary_defender);
    imshow("Defender detection", image_orig);
    return span;

}

float Vision::distance_marker_left_blob(Mat binary_defender, float center){

    Mat image_orig = Vision::_current_image.clone();
    int screen_width = Vision::_current_image.cols;
    int min_blob_size = 20; // minimum size of a blob that NAO would consider part of the defender
    Rect blob;
    int edge = center;
    float span = 0.0f;

    std::vector<std::vector<Point> > contours; //vector of contours
    std::vector<Vec4i> hierarchy;
    findContours(binary_defender, contours, hierarchy, 2/*cv::CV_RETR_CCOMP*/, 2/*cv::CV_CHAIN_APPROX_SIMPLE*/);

    for( int i = 0; i < contours.size(); i++ ) // iterate through each contour.
    {
        double a = cv::contourArea( contours[i], false);  //  Find the area of contour
        if(a > min_blob_size) {
            blob = boundingRect(contours[i]);
            if (blob.x + blob.width > edge) {
                edge = blob.x + blob.width;
                span = (float)edge - center;
                rectangle(binary_defender, blob, Scalar(255, 255, 255));
                rectangle(image_orig, blob, Scalar(255, 0, 0));
            }
        }
    }
    imshow("Defender Blob", binary_defender);
    imshow("Defender detection", image_orig);
    return span;

}

float Vision::get_defender_rotation(){
    if (defender_visible()) {
        Mat rvec_matrix;
        for (size_t i = 0; i < _aruco_markers.size(); i++) {
            if (_aruco_markers[i].id == defender_id) {
                rvec_matrix = _aruco_markers[i].Rvec;
            }
        }
        std::vector<float> rvec{
                rvec_matrix.at<float>(0),
                rvec_matrix.at<float>(1),
                rvec_matrix.at<float>(2)
        };
        return get_marker_yaw(rvec);
    }

}

float Vision::get_marker_yaw(std::vector<float> &rvec) {

    balling_nao::GetMarkerRotation srv;
    srv.request.Rvec = rvec;
    if (_client_get_rotation.call(srv)) {
        return srv.response.yaw;
    }
    else {
        throw std::runtime_error("Could not get rotation information");
    }


}
//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_VISION_H
#define BALLING_NAO_VISION_H

#include <ros/node_handle.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>


class Vision {

public:

    explicit Vision(ros::NodeHandle& node_handle);

    ~Vision() = default;

    Vision(Vision&& vision) = delete;

    // Callback for the raw image
    void imageCb(const sensor_msgs::ImageConstPtr& msg);


    // Tell the caller whether the ball is currently visible or not
    //
    // Returns
    //     true: Ball is visible
    //     false: Ball is not visible
    bool ball_visible() { return true/*_ball_visible*/; }


    // Tell the caller whether the defender is visible
    //
    // Returns
    //     true: Defender visible
    //     false: Defender not visible
    bool defender_visible() { return true/*_defender_visible*/;}

    // Tell the call whether the hoop is visible
    //
    // Returns
    //     true: hoop visible
    //     false: hoop not visible
    bool hoop_visible() { return true/*_hoop_visible*/; }

private:

    bool _ball_visible = false;
    bool _defender_visible = false;
    bool _hoop_visible = false;

};
#endif //BALLING_NAO_VISION_H

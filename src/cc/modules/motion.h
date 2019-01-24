//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_MOTION_H
#define BALLING_NAO_MOTION_H

#include <ros/node_handle.h>
#include "tactile.h"
#include <std_msgs/Bool.h>
#include <balling_nao/MoveJointsResponse.h>


class Motion {

public:

    explicit Motion(ros::NodeHandle& node_handle);
    ~Motion() = default;

    Motion(Motion&& motion) = delete;

    // But the robot into request ball position
    // Blocks until the movement is done
    void request_ball_position();

    // Grasp to hold the ball
    // Blocks until motion is done
    void finger_movement(int type);

    std::vector<float> request_cartesian_movement(std::string& name, std::vector<float> &position, float time);

    void request_joint_movement(std::vector<std::string>& names, std::vector<float> &angles, float fractionMaxSpeed, float sleep_time, balling_nao::MoveJointsResponse& response);

    bool check_movement_success(std::vector<float>& angles_request, std::vector<float>& angle_response);

    bool request_hand_action(std::string handName, int state);

    void footContactCallback(const std_msgs::BoolConstPtr& contact);

    void walk_to_position(double x, double y, double theta);

    void stop_walking();

private:

    ros::ServiceClient _client_get_position;
    ros::ServiceClient _client_set_position;
    ros::ServiceClient _client_get_transform;
    ros::ServiceClient _client_set_joints;
    ros::ServiceClient _client_set_hands;

    // Publisher to nao walking
    ros::Publisher _walk_pub;
    ros::ServiceClient _stop_walk_srv;
    // Subscriber for foot contact
    ros::Subscriber _footContact_sub;
    ros::ServiceClient _body_stiffness_enable;
    ros::ServiceClient _body_stiffness_disable;


};

#endif //BALLING_NAO_MOTION_H

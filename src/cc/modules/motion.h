//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_MOTION_H
#define BALLING_NAO_MOTION_H

#include <ros/node_handle.h>
#include "tactile.h"


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
    void grasp();

    std::vector<float> request_cartesian_movement(std::string& name, std::vector<float> &position, float time);

    bool request_joint_movement(std::vector<std::string>& names, std::vector<float> &angles, std::vector<float> &fractionMaxSpeeds, float sleep_time);

    bool request_hand_action(std::string handName, int state);

private:

    ros::ServiceClient _client_get_position;
    ros::ServiceClient _client_set_position;
    ros::ServiceClient _client_get_transform;
    ros::ServiceClient _client_set_joints;
    ros::ServiceClient _client_set_hands;

};

#endif //BALLING_NAO_MOTION_H

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

    // Constructs object
    // - "get_position_server" (service)
    // - "set_position_server" (service)
    // - "get_transformation" (service)
    // - "set_joints_server" (service)
    // - "set_hands_server" (service)
    // - "/body_stiffness/enable" (service)
    // - "/body_stiffness/disable" (service)
    // - "/cmd_pose" (publish)
    // - "/foot_contact" (subscribe)
    explicit Motion(ros::NodeHandle& node_handle);
    ~Motion() = default;

    Motion(Motion&& motion) = delete;

    // Put the robot into request ball position
    // Blocks until the movement is done
    // It will check the success of the movement
    // and will retry until the position is sufficient (check_movement_success)
    //
    void request_ball_position();

    // Do a finger movement with the right hand. Thin wrapper for the request_hand_action function.
    //
    // Args:
    //     type: 0: open, 1: close
    //
    void finger_movement(int type);

    std::vector<float> request_cartesian_movement(std::string& name, std::vector<float> &position, float time);

    // Send a request to the MoveJoints Python service.
    //
    // Args:
    //     names: The names of the joints to move
    //     angles: The desired angle positions
    //     fractionMaxSpeed: The fraction of maximal speed (0, 1)
    //     response: The response. Will be filled out with the python services response.
    //               Contains the new angle values and can be used to make a success check.
    //
    void request_joint_movement(
            std::vector<std::string>& names,
            std::vector<float> &angles,
            float fractionMaxSpeed,
            balling_nao::MoveJointsResponse& response
            );

    // Compares the angle request to the angle response. If there is a difference of 15 degree or more
    // for one angle the function returns False indicating failure, else it returns True indicating success
    //
    // Args:
    //     angle_request: A list of requested angles (in RAD).
    //     angle_response: A list of angles from the response (in RAD)
    //
    // Return:
    //     True: Movement successful
    //     False: Movement incomplete
    //
    bool check_movement_success(std::vector<float>& angles_request, std::vector<float>& angle_response);

    // Request a hand action. (Open or close)
    //
    // Args:
    //    handName: LHand or RHand
    //    state: 0: open 1: close
    //
    // Returns:
    //      success
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

// Header file for the motion class
// The motion modules handles NAO's movement
// structure : single class

//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_MOTION_H
#define BALLING_NAO_MOTION_H

#include <ros/node_handle.h>
#include "tactile.h"
#include <std_msgs/Bool.h>
#include <balling_nao/MoveJointsResponse.h>


struct BaseMotion;
struct MotionSequence;


class Motion {

    // The motion module.
    // Uses different services to interface with the NAO Api
    // Every time the robot moves, this module is responsible

public:

    // Constructs object
    // - "set_joints_server" (service)
    // - "go_to_posture_server" (service)
    // - "move_to_position_server" (service)
    // - "/body_stiffness/enable" (service)
    // - "/body_stiffness/disable" (service)
    // - "/cmd_pose" (publish)
    // - "/foot_contact" (subscribe)
    explicit Motion(ros::NodeHandle& node_handle);
    ~Motion() = default;

    Motion(Motion&& motion) = delete;

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

    // Compares the angle request to the angle response. If there is a difference of thresh degree or more
    // for one angle the function returns False indicating failure, else it returns True indicating success
    //
    // Args:
    //     angle_request: A list of requested angles (in RAD).
    //     angle_response: A list of angles from the response (in RAD)
    //     thresh: The threshold for a movement to be considered successful. I.e. if the returned angles
    //             (the new ones) are within thresh of the request for every joint its a success.
    //
    // Return:
    //     True: Movement successful
    //     False: Movement incomplete
    //
    bool check_movement_success(std::vector<float>& angles_request, std::vector<float>& angle_response, float thresh);

    // Callback for the foot contact topic.
    void footContactCallback(const std_msgs::BoolConstPtr& contact);

    // Call to the stop walking service
    void stop_walking();

    // Execute a standard motion. This puts the robot into a certain joint position.
    //
    // Args:
    //     motion: The motion defining the angles, names and the speed
    //     check: Whether to make the check if the movement was successful (within 15 degree fo the request)
    //
    void perform_standard_motion(BaseMotion& motion, bool check = true);

    // Perform a hole sequence of standard motions
    //
    // Args:
    //     sequence: A vector of motions
    //
    void perform_motion_sequence(MotionSequence& sequence);

    // Calls the posture service to put nao into one of the standard postures.
    void go_to_posture(std::string posture_name, float speed);

    // Disable the robots body stiffness
    //
    // Args:
    //     iteration: How often the call will be made
    //
    void disable_stiffness(int iterations);

    // Lets Nao walk to a position
    //
    // Args:
    //     x: The x-coord
    //     y: The y-coord
    //     theta: The angle
    //
    bool request_move_to_position(float x, float y, float theta);

private:

    // All the services and subscriptions
    ros::ServiceClient _client_set_joints;
    ros::ServiceClient _client_set_posture;
    ros::ServiceClient _client_move_to;
    ros::Publisher _walk_pub;
    ros::ServiceClient _stop_walk_srv;
    ros::Subscriber _footContact_sub;
    ros::ServiceClient _body_stiffness_enable;
    ros::ServiceClient _body_stiffness_disable;


};

#endif //BALLING_NAO_MOTION_H

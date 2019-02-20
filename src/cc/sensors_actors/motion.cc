//
// Created by hrs_b on 10.01.19.
//

#include "motion.h"
#include "sensor_msgs/JointState.h"
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include "../errors/not_implemented_error.h"
#include "balling_nao/GetPosition.h"
#include "balling_nao/Movement.h"
#include "balling_nao/GetTransform.h"
#include "balling_nao/MoveJoints.h"
#include "balling_nao/HandControl.h"
#include "balling_nao/GoToPosture.h"
#include "balling_nao/MoveToPosition.h"
#include "motion_library.h"


Motion::Motion(ros::NodeHandle &node_handle)
{

    _client_set_joints = node_handle.serviceClient<balling_nao::MoveJoints>("set_joints_server");
    _client_set_posture = node_handle.serviceClient<balling_nao::GoToPosture>("go_to_posture_server");
    _client_move_to = node_handle.serviceClient<balling_nao::MoveToPosition>("move_to_position_server");
    _body_stiffness_enable = node_handle.serviceClient<std_srvs::Empty>("/body_stiffness/enable");
    _body_stiffness_disable = node_handle.serviceClient<std_srvs::Empty>("/body_stiffness/disable");
    _walk_pub = node_handle.advertise<geometry_msgs::Pose2D>("/cmd_pose", 1);
    _footContact_sub = node_handle.subscribe<std_msgs::Bool>("/foot_contact", 1, &Motion::footContactCallback, this);

}


void Motion::request_joint_movement(
        std::vector<std::string>& names,
        std::vector<float> &angles,
        float fractionMaxSpeed,
        balling_nao::MoveJointsResponse& response
        ) {

    balling_nao::MoveJoints srv;
    srv.request.names = names;
    srv.request.angles = angles;
    srv.request.fractionMaxSpeed = fractionMaxSpeed;

    if (_client_set_joints.call(srv)) {
        response = srv.response;
    }

}


void Motion::footContactCallback(const std_msgs::BoolConstPtr& contact)
{
    bool has_contact = contact->data;
    std::cout << "Has contact: " << has_contact << std::endl;
    if (1 != has_contact) {
        stop_walking();
    }
}


bool Motion::request_move_to_position(float x, float y, float theta){ //state = 0 to open the hand, 1 to close
    balling_nao::MoveToPosition srv;

    srv.request.x = x;
    srv.request.y = y;
    srv.request.theta = theta;
    if (_client_move_to.call(srv)){
        bool success = srv.response.success;
        return success;
    }
    else {
        throw std::runtime_error("MoveToPosition request was not successful");
    }
}


void Motion::stop_walking()
{
    std_srvs::Empty msg;
    if (_stop_walk_srv.call(msg)) {
        ROS_INFO("Requesting to StopWalk");
        _body_stiffness_disable.call(msg);
    }

}


bool Motion::check_movement_success(std::vector<float> &angles_request, std::vector<float> &angle_response, float thresh) {

    for(size_t i = 0; i < angles_request.size(); i++) {
        float dif = (angles_request[i] - angle_response[i]) / PI_180;
        if(dif > thresh or dif < -thresh) return false;
    }
    return true;
}


void Motion::perform_standard_motion(BaseMotion& motion, bool check) {

    balling_nao::MoveJointsResponse response;

    request_joint_movement(motion.names, motion.angles, motion.speed, response);

    if (check){ //default behavior
        while(!check_movement_success(motion.angles, response.new_angles, motion.success_threshold)) {
            request_joint_movement(motion.names, motion.angles, motion.speed, response);
        }
    }

}


void Motion::go_to_posture(std::string posture_name, float speed) {
    std_srvs::Empty empty;
    _body_stiffness_enable.call(empty); // setting stiffness before going into posture

    balling_nao::GoToPosture srv;
    srv.request.posture_name = posture_name;
    srv.request.speed = speed;
    if(_client_set_posture.call(srv)){
        return;
    }
    else{
        throw std::runtime_error("Could not send posture request");
    }
}


void Motion::disable_stiffness(int iterations){

    std_srvs::Empty empty;
    for (int i =0; i <iterations; i++) _body_stiffness_disable.call(empty);

}


void Motion::perform_motion_sequence(MotionSequence& sequence) {

    for(size_t motion_index = 0; motion_index < sequence.motions.size(); motion_index++) {

        perform_standard_motion(
                sequence.motions[motion_index],
                sequence.make_check[motion_index]
        );
        ros::spinOnce();  // Not really relevant
    }
}

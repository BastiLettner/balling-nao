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

float DT = 0.2;
float PI_180 = 3.141592/180;

Motion::Motion(ros::NodeHandle &node_handle)
{
    _client_get_position = node_handle.serviceClient<balling_nao::GetPosition>("get_position_server");
    _client_set_position = node_handle.serviceClient<balling_nao::Movement>("set_position_server");
    _client_get_transform = node_handle.serviceClient<balling_nao::GetTransform>("get_transformation");
    _client_set_joints = node_handle.serviceClient<balling_nao::MoveJoints>("set_joints_server");
    _client_set_hands = node_handle.serviceClient<balling_nao::HandControl>("set_hands_server");
    _body_stiffness_enable = node_handle.serviceClient<std_srvs::Empty>("/body_stiffness/enable");
    _body_stiffness_disable = node_handle.serviceClient<std_srvs::Empty>("/body_stiffness/disable");
    _walk_pub=node_handle.advertise<geometry_msgs::Pose2D>("/cmd_pose", 1);
    _footContact_sub = node_handle.subscribe<std_msgs::Bool>("/foot_contact", 1, &Motion::footContactCallback, this);
}


void Motion::request_ball_position() {
    ROS_INFO("REQUESTING BALL");
    //http://doc.aldebaran.com/1-14/family/robots/joints_robot.html#robot-joints-v4-right-arm-joints
    std::vector<std::string> names = {"RShoulderRoll", "RShoulderPitch", "RElbowYaw", "RWristYaw", "RElbowRoll"};
    float RShoulderRoll = -5.0 * PI_180;
    float RShoulderPitch = 0.0 * PI_180;
    float RElbowYaw = 110.0 * PI_180;
    float RWristYaw = 100.0 * PI_180;
    float RElbowRoll = 85.0 * PI_180;
    std::vector<float> angles{RShoulderRoll, RShoulderPitch, RElbowYaw, RWristYaw, RElbowRoll};

    std::vector<float> times(5, DT);

    request_joint_movement(names, angles, times, 0.5);

    request_hand_action("RHand", 0);
}


void Motion::grasp() {
    if (true){ //tactile press
        request_hand_action("RHand", 1);
    }
}

std::vector<float> Motion::request_cartesian_movement(std::string &name, std::vector<float> &position, float time) {

    // Request movement using Interpolation and return the new position.
    balling_nao::Movement srv; //initialise a srv file
    srv.request.name = name;
    srv.request.coordinates = position;
    srv.request.time = time;
    srv.request.type = 1;
    if (_client_set_position.call(srv)) {
        std::vector<float> coordinates_6D(srv.response.coordinates.begin(), srv.response.coordinates.end()); //obtain the current position of the robot
        return coordinates_6D;
    }
    else {
        throw std::runtime_error("Could not send movement request");
    }
}

bool Motion::request_joint_movement(std::vector<std::string>& names, std::vector<float> &angles,
        std::vector<float> &fractionMaxSpeeds, float sleep_time) {

    balling_nao::MoveJoints srv;
    srv.request.names = names;
    srv.request.angles = angles;
    srv.request.fractionMaxSpeeds = fractionMaxSpeeds;
    //srv.request.movement_type = NORMAL;
    srv.request.sleep_time = sleep_time;
    bool success = false;
    //execute the order a few times to make sure the limb is inthe correct position
    for (int i = 0; i<5; i++) {
        if (_client_set_joints.call(srv)) {
            sleep(1);
            success = srv.response.success;
        }
    }
    return success;
}

bool Motion::request_hand_action(std::string handName, int state){ //state = 0 to open the hand, 1 to close
    balling_nao::HandControl srv;
    srv.request.action_type = state;
    srv.request.hand_name = handName;
    if (_client_set_hands.call(srv)){
        bool success = srv.response.success;
        return success;
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

void Motion::walk_to_position(double x, double y, double theta)
{
    std_srvs::Empty empty;
    _body_stiffness_enable.call(empty);
    ROS_INFO("Requesting to walk");
    geometry_msgs::Pose2D msg;
    msg.x = x;
    msg.y = y;
    msg.theta = theta;
    _walk_pub.publish(msg);
}


void Motion::stop_walking()
{
    std_srvs::Empty msg;
    if (_stop_walk_srv.call(msg)) {
        ROS_INFO("Requesting to StopWalk");
        _body_stiffness_disable.call(msg);
    }

}


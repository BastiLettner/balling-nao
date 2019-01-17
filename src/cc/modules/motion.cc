//
// Created by hrs_b on 10.01.19.
//

#include "motion.h"

#include "sensor_msgs/JointState.h"
#include "../errors/not_implemented_error.h"
#include "balling_nao/GetPosition.h"
#include "balling_nao/Movement.h"
#include "balling_nao/GetTransform.h"
#include "balling_nao/MoveJoints.h"
#include "balling_nao/HandControl.h"

float DT = 0.2;

Motion::Motion(ros::NodeHandle &node_handle)
{
    _client_get_position = node_handle.serviceClient<balling_nao::GetPosition>("get_position_server");
    _client_set_position = node_handle.serviceClient<balling_nao::Movement>("set_position_server");
    _client_get_transform = node_handle.serviceClient<balling_nao::GetTransform>("get_transformation");
    _client_set_joints = node_handle.serviceClient<balling_nao::MoveJoints>("set_joints_server");
    _client_set_hands = node_handle.serviceClient<balling_nao::HandControl>("set_hands_server");
}


void Motion::request_ball_position() {
    //http://doc.aldebaran.com/1-14/family/robots/joints_robot.html#robot-joints-v4-right-arm-joints
    std::vector<std::string> names = {"RShoulderRoll", "RShoulderPitch", "RElbowYaw", "RWristYaw", "RElbowRoll"};
    std::vector<float> angles{-5.0, 0.0, 110.0, 100.0, 85.0};
    std::vector<float> times(5, DT);

    request_joint_movement(names, angles, times, 0.2);

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
    if (_client_set_joints.call(srv)) {
        bool success = srv.response.success;
        return success;
    }
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

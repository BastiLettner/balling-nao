//
// Created by hrs_b on 10.01.19.
//

#include "motion.h"
#include "sensor_msgs/JointState.h"
#include "../errors/not_implemented_error.h"


Motion::Motion(ros::NodeHandle &node_handle)
{

}


void Motion::request_ball_position() {
    throw NotImplementedError();
}


void Motion::grasp() {
    throw NotImplementedError();
}
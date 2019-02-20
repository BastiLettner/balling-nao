#include <ros/ros.h>
#include "../src/cc/core/controller.h"


int main(int argc, char** argv) {

    ros::init(argc, argv, "balling_nao");

    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    Controller c(nh);  // Create controller

    c.run();  // Run Controller

    return 0;

}
#include <ros/ros.h>
#include "../src/cc/modules/controller.h"


int main(int argc, char** argv) {

    ros::init(argc, argv, "balling_nao");

    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    Controller c(nh);

    c.run();

    return 0;

}
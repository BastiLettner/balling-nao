#include <ros/ros.h>


int main(int argc, char** argv) {

    ros::init(argc, argv, "balling_nao");

    ros::spinOnce();

    return 0;
}
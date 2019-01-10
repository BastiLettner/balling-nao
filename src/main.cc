#include <ros/ros.h>


int main(int argc, char** argv) {

    ros::init(argc, argv, "balling-nao");

    ros::spinOnce();

    return 0;
}
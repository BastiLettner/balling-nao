//
// Created by hrs_b on 15.01.19.
//

#include "../src/cc/core/controller.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "request_ball_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    Controller c(nh);

    c.run();

}
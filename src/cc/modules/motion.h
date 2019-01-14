//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_MOTION_H
#define BALLING_NAO_MOTION_H

#include <ros/node_handle.h>

class Motion {

public:

    explicit Motion(ros::NodeHandle& node_handel);
    ~Motion() = default;

    Motion(Motion&& motion) = delete;

    // But the robot into request ball position
    // Blocks until the movement is done
    void request_ball_position();

    // Grasp to hold the ball
    // Blocks until motion is done
    void grasp();

private:

};

#endif //BALLING_NAO_MOTION_H

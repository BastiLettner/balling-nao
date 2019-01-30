//
// Created by hrs_b on 28.01.19.
//

#ifndef BALLING_NAO_MOTIONS_H
#define BALLING_NAO_MOTIONS_H

#include <vector>
#include <string>


float PI_180 = (float)3.141592/180; // for conversion from degree to rad


struct BaseMotion {

    BaseMotion(std::vector<std::string> n, std::vector<float> a, float s, float thresh):
        names(n),
        angles(a),
        speed(s),
        success_threshold(thresh)
        {}

    std::vector<std::string> names;
    std::vector<float> angles;

    float speed;
    float success_threshold;

};

namespace MOTIONS {

    BaseMotion LOOK_AT_BALL = BaseMotion(
            {"HeadYaw", "HeadPitch"},
            {-40.0f*PI_180, 25.0f*PI_180},
            0.2,
            15.0
            );

    BaseMotion REQUEST_BALL_POSITION = BaseMotion(
            {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RWristYaw", "RElbowRoll"},
            {50.0f*PI_180, -5.0f*PI_180, 110.0f*PI_180, 95.0f*PI_180, 85.0f*PI_180},
            0.2,
            15.0
            );

    BaseMotion HEAD_RESTING_POSITION = BaseMotion(
            {"HeadYaw", "HeadPitch"},
            {0.0f, 0.0f},
            0.2,
            15.0
            );

    BaseMotion CLOSE_RHAND = BaseMotion(
            {"RHand"},
            {20.0f*PI_180},
            0.2,
            15.0
            );

    BaseMotion OPEN_RHAND = BaseMotion(
            {"RHand"},
            {50.0f*PI_180},
            0.2,
            15.0);

}


#endif //BALLING_NAO_MOTIONS_H

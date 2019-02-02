//
// Created by hrs_b on 31.01.19.
//

#include "motion_library.h"


const float PI_180 = (float)3.141592/180; // for conversion from degree to rad


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

    BaseMotion HEAD_FAR_LEFT = BaseMotion(
            {"HeadYaw", "HeadPitch"},
            {110.0f*PI_180, 0.0f*PI_180},
            0.2,
            15.0
    );

    BaseMotion HEAD_FAR_RIGHT = BaseMotion(
            {"HeadYaw", "HeadPitch"},
            {-110.0f*PI_180, 0.0f*PI_180},
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

    BaseMotion HEAD_LEFT = BaseMotion(
            {"HeadYaw", "HeadPitch"},
            {60.0f*PI_180, 0.0f*PI_180},
            0.2,
            15.0
    );

    BaseMotion HEAD_RIGHT = BaseMotion(
            {"HeadYaw", "HeadPitch"},
            {-60.0f*PI_180, 0.0f*PI_180},
            0.2,
            15.0
    );

    BaseMotion EXTEND_RARM = BaseMotion(
            {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RWristYaw", "RElbowRoll"},
            {-20.0f*PI_180, -5.0f*PI_180, 1.0f*PI_180, 1.0f*PI_180, 1.0f*PI_180},
            0.3,
            15.0
    );

    BaseMotion PULLBACK_RARM = BaseMotion(
            {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RWristYaw", "RElbowRoll"},
            {115.0f*PI_180, -5.0f*PI_180, 90.0f*PI_180, 95.0f*PI_180, 1.0f*PI_180},
            0.2,
            15.0
    );


}


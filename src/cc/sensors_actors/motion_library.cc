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
            {40.0f*PI_180, 15.0f*PI_180, 110.0f*PI_180, 75.0f*PI_180, 85.0f*PI_180},
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
            {90.0f*PI_180, 0.0f*PI_180},
            0.2,
            15.0
    );

    BaseMotion HEAD_FAR_RIGHT = BaseMotion(
            {"HeadYaw", "HeadPitch"},
            {-90.0f*PI_180, 0.0f*PI_180},
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
            {45.0f*PI_180, 0.0f*PI_180},
            0.2,
            15.0
    );

    BaseMotion HEAD_RIGHT = BaseMotion(
            {"HeadYaw", "HeadPitch"},
            {-45.0f*PI_180, 0.0f*PI_180},
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

    BaseMotion DUNK_MOTION_1 = BaseMotion(
            {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RWristYaw", "RElbowRoll"},
            {1.5079641342163086, -1.2932038307189941, 1.9343321323394775, 0.042994022369384766, -0.07367396354675293},
            0.2,
            15.0
            );

    BaseMotion DUNK_MOTION_2 = BaseMotion(
            {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RWristYaw", "RElbowRoll"},
            {-1.6060560941696167, -1.1045217514038086, 1.7026981115341187, 0.03490658476948738, -0.07827591896057129},
            0.2,
            15.0
    );

    BaseMotion DUNK_MOTION_3 = BaseMotion(
            {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RWristYaw"},
            {-1.4679961, 0.096600055, 0.0858621, -0.036858081},
            0.2,
            15.0
            );

    BaseMotion DUNK_MOTION_4 = BaseMotion(
            {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RWristYaw", "RElbowRoll"},
            {-0.03984212875366211, 0.3141592741012573, 0.07972598075866699, 0.4525718688964844, -0.013848066329956055},
            0.4,
            15.0
    );



}


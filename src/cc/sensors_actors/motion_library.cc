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


    namespace SEARCH {

        MotionSequence MARKER_CLOSE_SEQ = MotionSequence(
            {
                BaseMotion(
                        {"HeadYaw", "HeadPitch"},
                        {0.0f * PI_180, 20.0f * PI_180},
                        0.2,
                        15.0
                ),
                BaseMotion(
                        {"HeadYaw", "HeadPitch"},
                        {45.0f * PI_180, 20.0f * PI_180},
                        0.2,
                        15.0
                ),
                BaseMotion(
                        {"HeadYaw", "HeadPitch"},
                        {25.0f * PI_180, 20.0f * PI_180},
                        0.2,
                        15.0
                ),
                BaseMotion(
                        {"HeadYaw", "HeadPitch"},
                        {0.0f * PI_180, 20.0f * PI_180},
                        0.2,
                        15.0
                ),
                BaseMotion(
                        {"HeadYaw", "HeadPitch"},
                        {-25.0f * PI_180, 20.0f * PI_180},
                        0.2,
                        15.0
                ),
                BaseMotion(
                        {"HeadYaw", "HeadPitch"},
                        {-45.0f * PI_180, 20.0f * PI_180},
                        0.2,
                        15.0
                )
            },
            {
                true,
                true,
                true,
                true,
                true,
                true,
            }
        );

        MotionSequence SIMPLE_SEARCH = MotionSequence(
                {
                    BaseMotion(
                            {"HeadYaw", "HeadPitch"},
                            {0.0f, 0.0f},
                            0.2,
                            15.0
                    ),
                    BaseMotion(
                            {"HeadYaw", "HeadPitch"},
                            {90.0f*PI_180, 0.0f*PI_180},
                            0.2,
                            15.0
                    ),
                    BaseMotion(
                            {"HeadYaw", "HeadPitch"},
                            {45.0f*PI_180, 0.0f*PI_180},
                            0.2,
                            15.0
                    ),
                    BaseMotion(
                            {"HeadYaw", "HeadPitch"},
                            {0.0f, 0.0f},
                            0.2,
                            15.0
                    ),
                    BaseMotion(
                            {"HeadYaw", "HeadPitch"},
                            {-45.0f*PI_180, 0.0f*PI_180},
                            0.2,
                            15.0
                    ),
                    BaseMotion(
                            {"HeadYaw", "HeadPitch"},
                            {-90.0f*PI_180, 0.0f*PI_180},
                            0.2,
                            15.0
                    )
                },
                {
                    true,
                    true,
                    true,
                    true,
                    true,
                    true
                }
        );

    }

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

    namespace THROW {


        // distance to the marker: 35 cm
        // landing position: straight in front of the robot
        // 10 shot attempts show that the ball land in a 10x20 cm rectangle directly in front of the robot
        MotionSequence STRAIGHT_OVER_HEAD_EXT_SEQ = MotionSequence(
                {
                        BaseMotion(
                                {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"},
                                {1.3883118629455566, -0.1365680694580078, 0.06592011451721191, 0.06753802299499512, -0.009245872497558594},
                                0.4,
                                15.0
                        ),
                        BaseMotion(
                                {"LHipPitch", "RHipPitch"},
                                {-0.05f, -0.05f},
                                0.05,
                                15.0
                        ),
                        BaseMotion(
                                {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"},
                                {0.885159969329834, 0.009161949157714844, 1.6904261112213135, 0.9327139854431152, -1.7012481689453125},
                                0.4,
                                15.0
                        ),
                        BaseMotion(
                                {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"},
                                {-1.8284860849380493, -0.009245872497558594, 1.707300066947937, 0.6029040813446045, -1.526371955871582},
                                0.2,
                                15.0
                        ),
                        BaseMotion(
                                {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"},
                                {-0.3573801517486572, 0.010695934295654297, 0.6457719802856445, 0.03490658476948738, -0.5936999320983887, 50.0f*PI_180},
                                0.8,
                                15.0
                        ),
                },
                {
                    true,
                    true,
                    true,
                    true,
                    false,
                }
        );

        // distance to the marker: 60 cm
        // center of five shot attempts: 60cm distance to the robot, 20 cm to the left of the robot
        MotionSequence LEFT_OVER_HEAD_EXT_SEQ = MotionSequence(
                {
                        BaseMotion(
                                {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"},
                                {1.3883118629455566, -0.1365680694580078, 0.06592011451721191, 0.06753802299499512, -0.009245872497558594},
                                0.4,
                                15.0
                        ),
                        BaseMotion(
                                {"LHipPitch", "RHipPitch"},
                                {-0.05f, -0.05f},
                                0.05,
                                15.0
                        ),
                        BaseMotion(
                                {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"},
                                {0.885159969329834, 0.009161949157714844, 1.6904261112213135, 0.9327139854431152, -1.7012481689453125},
                                0.4,
                                15.0
                        ),
                        BaseMotion(
                                {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"},
                                {-1.2793140411376953, -0.10895586013793945, 1.9113221168518066, 0.8728880882263184, -1.4711480140686035},
                                0.2,
                                15.0
                        ),
                        BaseMotion(
                                {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"},
                                //{-0.7485499382019043, 0.3141592741012573, 2.0386440753936768, 0.03490658476948738, -1.5048961639404297, 50.0f*PI_180},
                                {-0.5629360675811768, 0.2638061046600342, 1.7211060523986816, 0.03490658476948738, -1.412856101989746, 50.0f*PI_180},
                                1.0,
                                15.0
                        ),

                },
                {
                        true,
                        true,
                        true,
                        true,
                        false,
                }
        );

        // distance to the marker: 60 cm
        // center of five shot attempts: 60cm distance to the robot, 20 cm to the left of the robot
        MotionSequence RIGHT_OVER_HEAD_EXT_SEQ = MotionSequence(
                {
                        BaseMotion(
                                {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"},
                                {1.3883118629455566, -0.1365680694580078, 0.06592011451721191, 0.06753802299499512, -0.009245872497558594},
                                0.4,
                                15.0
                        ),
                        BaseMotion(
                                {"LHipPitch", "RHipPitch"},
                                {-0.05f, -0.05f},
                                0.05,
                                15.0
                        ),
                        BaseMotion(
                                {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"},
                                {0.885159969329834, 0.009161949157714844, 1.6904261112213135, 0.9327139854431152, -1.7012481689453125},
                                0.4,
                                15.0
                        ),
                        BaseMotion(
                                {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"},
                                {-1.5278220176696777, 0.0367741584777832, 1.21642005443573, 1.0416278839111328, -1.5723919868469238},
                                0.2,
                                15.0
                        ),
                        BaseMotion(
                                {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"},
                                {-0.720937967300415, -0.09821796417236328, 0.7684919834136963, 0.03490658476948738, -0.902033805847168, 50.0f*PI_180},
                                1.0,
                                15.0
                        ),

                },
                {
                        true,
                        true,
                        true,
                        true,
                        false,
                }
        );

    }

    namespace DUNK {

        MotionSequence DUNK_SEQ = MotionSequence(
            {
                BaseMotion(
                        {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RWristYaw", "RElbowRoll"},
                        {1.5079641342163086, -1.2932038307189941, 1.9343321323394775, 0.042994022369384766, -0.07367396354675293},
                        0.4,
                        15.0
                ),
                BaseMotion(
                        {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RWristYaw", "RElbowRoll"},
                        {-1.6060560941696167, -1.1045217514038086, 1.7026981115341187, 0.03490658476948738, -0.07827591896057129},
                        0.4,
                        15.0
                ),
                BaseMotion(
                        {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RWristYaw"},
                        {-1.4679961, 0.096600055, 0.0858621, -0.036858081},
                        0.4,
                        15.0
                ),
                BaseMotion(
                        {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RWristYaw", "RElbowRoll"},
                        {0.3789398670196533, 0.3141592741012573, 0.11654210090637207, 0.2853660583496094, -0.07827591896057129},
                        0.7,
                        15.0
                ),
                BaseMotion(
                        {"RHand"},
                        {50.0f*PI_180},
                        0.2,
                        15.0
                ),
                BaseMotion(
                        {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RWristYaw", "RElbowRoll"},
                        {1.5079641342163086, -1.2932038307189941, 1.9343321323394775, 0.042994022369384766, -0.07367396354675293},
                        0.4,
                        15.0
                )
            },
            {
                true,
                true,
                true,
                false,
                true,
                true
            }
        );

    }

}


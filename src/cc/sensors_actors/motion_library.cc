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

        MotionSequence STRAIGHT_OVER_HEAD_SEQ = MotionSequence(
            {
                BaseMotion(
                        {"LHipPitch", "RHipPitch"},
                        {-0.15f, -0.15f},
                        0.05,
                        15.0
                ),
                BaseMotion(
                        {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RWristYaw", "RElbowRoll"},
                        {-2.063188076019287, 0.11194014549255371, 0.37885594367980957, 0.14423799514770508, -0.13963603973388672},
                        0.4,
                        15.0
                ),
                BaseMotion(
                        {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RWristYaw", "RElbowRoll"},
                        {-0.9525721073150635, 0.14108610153198242, 0.2791459560394287, 0.14270401000976562, -0.13963603973388672},
                        0.8,
                        15.0
                ),
                BaseMotion(
                        {"RHand"},
                        {50.0f*PI_180},
                        0.2,
                        15.0
                )
            },
            {
                true,
                true,
                false,
                false
            }
        );


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
                                {-2.0754599571228027, 0.047512054443359375, 1.7333780527114868, 0.5875639915466309, -1.6245479583740234},
                                0.2,
                                15.0
                        ),
                        BaseMotion(
                                {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"},
                                {-1.0737581253051758, -0.06753802299499512, 0.09506607055664062, 0.03685808181762695, -0.075207948684692382},
                                1.0,
                                15.0
                        ),
                        BaseMotion(
                                {"RHand"},
                                {50.0f*PI_180},
                                0.4,
                                15.0
                        )
                },
                {
                    true,
                    true,
                    true,
                    true,
                    false,
                    false
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


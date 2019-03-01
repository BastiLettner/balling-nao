// Header file for the motion library
// The motion library holds the description of every movement used during the baksketball routine
// structure : structure and namespace declaration

//
// Created by hrs_b on 28.01.19.
//

#ifndef BALLING_NAO_MOTION_LIBRARY_H
#define BALLING_NAO_MOTION_LIBRARY_H

#include <vector>
#include <string>


extern const float PI_180;


struct BaseMotion {

    // A simple motion defined by a set of angles.

    BaseMotion(std::vector<std::string> n, std::vector<float> a, float s, float thresh):
        names(n),
        angles(a),
        speed(s),
        success_threshold(thresh)
        {}

    std::vector<std::string> names;  // Name of the joints
    std::vector<float> angles; // The angles of the joints

    float speed;  // The maxFractionSpeed argument
    float success_threshold;  // The threshold for the motion to be considered successful

};


struct MotionSequence {

    // A vector of BaseMotions

    MotionSequence(std::vector<BaseMotion> m, std::vector<bool> c):
        motions(m),
        make_check(c) {}

    std::vector<BaseMotion> motions;
    std::vector<bool> make_check;

};

// THE MOTION LIBRARY

namespace MOTIONS {

    extern BaseMotion LOOK_AT_BALL;
    extern BaseMotion REQUEST_BALL_POSITION;
    extern BaseMotion HEAD_RESTING_POSITION;
    extern BaseMotion CLOSE_RHAND;
    extern BaseMotion OPEN_RHAND;
    extern BaseMotion EXTEND_RARM;
    extern BaseMotion PULLBACK_RARM;

    namespace DUNK {

        extern MotionSequence DUNK_SEQ;
    }


    namespace SEARCH {

        extern MotionSequence MARKER_CLOSE_SEQ;
        extern MotionSequence SIMPLE_SEARCH;
        extern MotionSequence VERY_SIMPLE_SEARCH;

    }

    namespace THROW {

        // Marker distance ~55cm
        // No shift
        extern MotionSequence STRAIGHT_OVER_HEAD_EXT_SEQ;

        // Marker distance ~55cm
        // Left Shift: ~25 cm
        extern MotionSequence LEFT_OVER_HEAD_EXT_SEQ;

        // marker distance ~55cm
        // Right Shift: ~21 cm
        extern MotionSequence RIGHT_OVER_HEAD_EXT_SEQ;

    }


}


#endif //BALLING_NAO_MOTION_LIBRARY_H

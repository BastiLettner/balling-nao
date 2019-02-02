//
// Created by hrs_b on 28.01.19.
//

#ifndef BALLING_NAO_MOTION_LIBRARY_H
#define BALLING_NAO_MOTION_LIBRARY_H

#include <vector>
#include <string>


extern const float PI_180;


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

extern BaseMotion LOOK_AT_BALL;
extern BaseMotion REQUEST_BALL_POSITION;
extern BaseMotion HEAD_RESTING_POSITION;
extern BaseMotion HEAD_FAR_LEFT;
extern BaseMotion HEAD_FAR_RIGHT;
extern BaseMotion HEAD_RIGHT;
extern BaseMotion HEAD_LEFT;
extern BaseMotion CLOSE_RHAND;
extern BaseMotion OPEN_RHAND;
extern BaseMotion EXTEND_RARM;
extern BaseMotion PULLBACK_RARM;
extern BaseMotion THROW1;

}






#endif //BALLING_NAO_MOTION_LIBRARY_H

//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_VISION_H
#define BALLING_NAO_VISION_H


class Vision {

public:

    Vision() = default;
    ~Vision() = default;

    Vision(Vision&& vision) = delete;

    // Tell the caller whether the ball is currently visible or not
    //
    // Returns
    //     true: Ball is visible
    //     false: Ball is not visible
    bool ball_visible() { return _ball_visible; }

private:

    bool _ball_visible = false;

};
#endif //BALLING_NAO_VISION_H

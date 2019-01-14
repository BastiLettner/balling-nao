//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_TAKE_BALL_H
#define BALLING_NAO_TAKE_BALL_H

#include "state.h"

class Controller;


class TakeBallState: public State {

public:

    TakeBallState();

    void go_next(Controller& controller) override;

    const std::string& get_state_name()  override { return _state_name; }

private:

    const std::string _state_name = "Take Ball State";

};
#endif //BALLING_NAO_TAKE_BALL_H

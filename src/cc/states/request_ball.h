//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_REQUEST_BALL_H
#define BALLING_NAO_REQUEST_BALL_H

#include "state.h"

class Controller;


class RequestBallState: public State {

    // Request Ball State (Initial State)

public:

    // Initializes object and parent object
    // Sets the available commands
    RequestBallState();

    // Implements the logic of the Request Ball State.
    // The main steps are:
    //     1. Use speech module to say: "Can I have the ball"
    //     2. Compare answers to available commands
    //     3. If 'yes': move to TakeBallState
    //        If 'no': move to DoneState
    //
    // Args:
    //     controller: The controller instance
    //
    void go_next(Controller& controller) override;

    // Get the name of teh state.
    const std::string get_state_name() override { return _state_name; }

private:

    const std::string _state_name = "Request Ball State";

};
#endif //BALLING_NAO_REQUEST_BALL_H

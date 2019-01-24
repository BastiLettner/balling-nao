//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_TAKE_BALL_H
#define BALLING_NAO_TAKE_BALL_H

#include "state.h"

class Controller;


class TakeBallState: public State {

public:

    // Initializes object and parent object
    // Sets the available command
    TakeBallState();

    // Implements the logic of the Request Ball State.
    // The main steps are:
    //     1. Use Motion Module to move arm into <take-ball> position
    //     2. Use Tactile Module to detect tactile button press
    //     3. Use Motion Module to finger_movement
    //     4. Use Vision Module to verify that ball is actually in naos hand
    //     5. if ball is in hand: move to RequestTaskState
    //     6. if ball is not in hand: Open Hand and move into TakeBallState
    //                                TODO: Check if moving arm back is necessary
    //
    // Args:
    //     controller: The controller instance
    //
    void go_next(Controller& controller) override;

    // Get the states name
    const std::string get_state_name() override { return _state_name; }

private:

    const std::string _state_name = "Take Ball State";

};
#endif //BALLING_NAO_TAKE_BALL_H

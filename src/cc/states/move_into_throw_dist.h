//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_MODE_INTO_THROW_DIST_H
#define BALLING_NAO_MODE_INTO_THROW_DIST_H

#include "state.h"

class Controller;

class MoveIntoThrowDistanceState: public State {

public:

    // Constructor
    MoveIntoThrowDistanceState();

    // Implements the logic of the MoveIntoThrowDistanceState.
    // The main steps are:
    //     1. Get the marker position
    //     2. move up to 65 cm towards the hoop.
    //     3. Set nextState to ExecuteThrowMotionState
    //
    // Args:
    //     controller: The controller instance
    //
    void go_next(Controller& controller) override;

    const std::string get_state_name() override { return _state_name; }

private:

    const std::string _state_name = "Move Into Throw Distance State";
};



#endif //BALLING_NAO_MODE_INTO_THROW_DIST_H

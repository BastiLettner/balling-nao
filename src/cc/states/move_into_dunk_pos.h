//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_MOVE_INTO_DUNK_POS_H
#define BALLING_NAO_MOVE_INTO_DUNK_POS_H

#include "state.h"

class Controller;

class MoveIntoDunkPositionState: public State {

public:

    // Constructor
    MoveIntoDunkPositionState();

    // Implements the logic of the MoveIntoDunkDistanceState.
    // The main steps are:
    //     1. Get the marker position
    //     2. Walk half the distance towards the hoop
    //     3. Get the marker position through searching
    //     4. Walk the remaining distance toward the hoop
    //     5. Make another search and adjust position if necessary
    //     6. Set next state to ExecuteDunkMotionState
    //
    // Args:
    //     controller: The controller instance
    //
    void go_next(Controller& controller) override;

    const std::string get_state_name() override { return _state_name; }

private:

    const std::string _state_name = "Move Into Dunk Position State";
};


#endif //BALLING_NAO_MOVE_INTO_DUNK_POS_H

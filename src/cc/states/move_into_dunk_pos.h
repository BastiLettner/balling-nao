//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_MOVE_INTO_DUNK_POS_H
#define BALLING_NAO_MOVE_INTO_DUNK_POS_H

#include "state.h"

class Controller;

class MoveIntoDunkPositionState: public State {

public:

    MoveIntoDunkPositionState();

    void go_next(Controller& controller) override;

    const std::string get_state_name() override { return _state_name; }

private:

    const std::string _state_name = "Move Into Dunk Position State";
};


#endif //BALLING_NAO_MOVE_INTO_DUNK_POS_H

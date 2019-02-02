//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_EXECUTE_DUNK_MOTION_H
#define BALLING_NAO_EXECUTE_DUNK_MOTION_H

#include "state.h"

class Controller;

class ExecuteDunkMotionState: public State {

public:

    ExecuteDunkMotionState();

    void go_next(Controller& controller) override;

    const std::string get_state_name() override { return _state_name; }

private:

    const std::string _state_name = "Execute Dunk Motion State";
};



#endif //BALLING_NAO_EXECUTE_DUNK_MOTION_H

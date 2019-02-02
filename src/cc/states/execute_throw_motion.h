//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_EXECUTE_THROW_MOTION_H
#define BALLING_NAO_EXECUTE_THROW_MOTION_H
#include "state.h"

class Controller;

class ExecuteThrowMotionState: public State {

public:

    ExecuteThrowMotionState();

    void go_next(Controller& controller) override;

    const std::string get_state_name() override { return _state_name; }

private:

    const std::string _state_name = "Execute Throw Motion State";
};



#endif //BALLING_NAO_EXECUTE_THROW_MOTION_H

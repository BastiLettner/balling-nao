//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_REQUEST_TASK_H
#define BALLING_NAO_REQUEST_TASK_H

#include "state.h"

class Controller;


class RequestTaskState: public State {

public:

    // Constructs object
    RequestTaskState();

    // Implements logic of request ball state.
    // The main steps are:
    //     1. Use speech module to say "What should I do?"
    //     2. Receive the answer (Dunk or Throw)
    //     3. Move to SearchDefenderState
    //
    // Args:
    //     controller: The controller instance
    //
    void go_next(Controller& controller) override;

    const std::string get_state_name() override { return _state_name; }

private:

    const std::string _state_name = "Request Task State";

};


#endif //BALLING_NAO_REQUEST_TASK_H

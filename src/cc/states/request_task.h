//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_REQUEST_TASK_H
#define BALLING_NAO_REQUEST_TASK_H

#include "state.h"

class Controller;


class RequestTaskState: public State {

public:

    RequestTaskState();

    void go_next(Controller& controller) override;

    const std::string& get_state_name() override { return _state_name; }

private:

    const std::string _state_name = "Request Ball State";

};


#endif //BALLING_NAO_REQUEST_TASK_H

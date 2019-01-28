//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_AVOID_DEFENDER_ROUTINE_H
#define BALLING_NAO_AVOID_DEFENDER_ROUTINE_H


#include "state.h"

class Controller;


class AvoidDefenderState: public State {

public:

    explicit AvoidDefenderState(std::string& task);

    void go_next(Controller& controller) override;

    const std::string get_state_name() override { return _state_name; }

    std::string task;

private:

    // This is used in the controller to detect when we reached a Done State.
    const std::string _state_name = "Avoid Defender State";

};



#endif //BALLING_NAO_AVOID_DEFENDER_ROUTINE_H

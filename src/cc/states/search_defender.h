// Search defender state header file
// Holds the instructions on what do to when in the request task state
// structure : single class

//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_SEARCH_POTENTIAL_DEFENDER_H
#define BALLING_NAO_SEARCH_POTENTIAL_DEFENDER_H

#include "state.h"

class Controller;


class SearchDefenderState: public State {

public:
    // Constructs object
    //
    // Args:
    //     task: The task (dunk or throw) from the RequestTaskState.
    explicit SearchDefenderState(std::string& task);

    // Implements logic of Search Defender State
    // The main steps are:
    //     1. if defender visible: move to AvoidDefenderState
    //        if defender not visible: move to DetectHoopState
    //
    // Args:
    //     controller: The controller instance
    //
    void go_next(Controller& controller) override;

    const std::string get_state_name() override { return _state_name; }

    std::string task;  // The task (dunk or throw) from the RequestTaskState. Need to be carried by this state.

private:

    const std::string _state_name = "Search Defender State";

};
#endif //BALLING_NAO_SEARCH_POTENTIAL_DEFENDER_H

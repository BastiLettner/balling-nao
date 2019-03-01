// Header file for the defender avoidance state
// This class holds the defender avoidance instructions when in the avoid_defender state
// structure : single class

//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_AVOID_DEFENDER_ROUTINE_H
#define BALLING_NAO_AVOID_DEFENDER_ROUTINE_H


#include "state.h"

class Controller;


class AvoidDefenderState: public State {

public:

    // Constructor
    //
    // Args:
    //     task: Dunk or Throw. Needs to be carried though this state
    explicit AvoidDefenderState(std::string& task);

    // Implements the logic of this state
    // The main steps are:
    //     1. Use the path planning module to calculate a trajectory
    //     2. Walk along this trajectory
    void go_next(Controller& controller) override;

    const std::string get_state_name() override { return _state_name; }

    std::string task;

private:

    // This is used in the controller to detect when we reached a Done State.
    const std::string _state_name = "Avoid Defender State";

};



#endif //BALLING_NAO_AVOID_DEFENDER_ROUTINE_H

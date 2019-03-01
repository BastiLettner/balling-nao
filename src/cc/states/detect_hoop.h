// Hoop detection state header file
// This class holds the hoop detection instructions when in the detect_hoop state
// structure : single class

//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_DETECT_HOOP_H
#define BALLING_NAO_DETECT_HOOP_H


#include "state.h"

class Controller;
enum SEARCH_MODE: int;


class DetectHoopState: public State {

public:

    // Constructs object
    //
    // Args:
    //     task: The task (dunk or throw) from the RequestTaskState.
    //
    explicit DetectHoopState(std::string& task);

    // Implements the logic of this state
    // The main steps are:
    //     1. Detect the hoop using all the search routines if necessary
    //
    void go_next(Controller& controller) override;

    const std::string get_state_name() override { return _state_name; }

    std::string task;  // The task (dunk or throw) from the RequestTaskState. Need to be carried by this state.

    // Implements searching for a given mode
    // This is an extra function since we use all three searches in this state and can reuse this function
    bool state_routine(SEARCH_MODE mode, Controller& controller);

private:

    const std::string _state_name = "Detect Hoop State";

};

#endif //BALLING_NAO_DETECT_HOOP_H

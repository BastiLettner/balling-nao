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

    explicit DetectHoopState(std::string& task);

    void go_next(Controller& controller) override;

    const std::string get_state_name() override { return _state_name; }

    std::string task;  // The task (dunk or throw) from the RequestTaskState. Need to be carried by this state.

    bool state_routine(SEARCH_MODE mode, Controller& controller);

private:

    const std::string _state_name = "Detect Hoop State";

};

#endif //BALLING_NAO_DETECT_HOOP_H

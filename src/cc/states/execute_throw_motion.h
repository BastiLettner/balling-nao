// Execute throw motion header file
// Holds the instructions on what to do when in the execute_throy_motion state
// structure : single class

//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_EXECUTE_THROW_MOTION_H
#define BALLING_NAO_EXECUTE_THROW_MOTION_H
#include "state.h"

class Controller;

class ExecuteThrowMotionState: public State {

public:

    // Constructor
    ExecuteThrowMotionState();

    // Implements the logic of the ExecuteThrowMotionState.
    // The main steps are:
    //     1. Check if we can see the hoop
    //     2. If not search for it and readjust the position
    //     3. Once we see the hoop, use the marker position to select a throwing motion
    //     4. Execute the throwing motion.
    //     5. Move into Done state
    //
    // Args:
    //     controller: The controller instance
    //
    void go_next(Controller& controller) override;

    const std::string get_state_name() override { return _state_name; }

private:

    const std::string _state_name = "Execute Throw Motion State";
};



#endif //BALLING_NAO_EXECUTE_THROW_MOTION_H

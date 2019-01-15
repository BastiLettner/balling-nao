//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_STATE_H
#define BALLING_NAO_STATE_H

#include <vector>
#include <string>


class Controller;


class State {

    // Base Class for States

public:

    // Default Constructs Object
    State() = default;

    // Default destruction
    virtual ~State() = default;

    // Interface for state transition function.
    //
    // Args:
    //     controller: The controller instance
    //
    virtual void go_next(Controller& controller) = 0;

    // Return the name of the state
    virtual const std::string get_state_name() = 0;

    // Returns the available commands
    std::vector<std::string>& get_available_cmds() { return _available_cmds; }

protected:

    std::vector<std::string> _available_cmds;

};

#endif //BALLING_NAO_STATE_H

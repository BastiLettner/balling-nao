//
// Created by hrs_b on 10.01.19.
//

#include "controller.h"
#include "../states/request_ball.h"
#include <iostream>


Controller::Controller():
    _current_state(new RequestBallState())
{
    go_next();
}


void Controller::go_next() {

    // Print out the state name as a log message
    LOG("Entered " + _current_state->get_state_name());

    // Create a message for the speech module to speak out the new states name
    std::string msg("Nao entered ");
    msg += _current_state->get_state_name();
    // TODO: Make call
    // _speech.talk(msg);

    _current_state->go_next(*this);

}


void Controller::LOG(std::string msg) {

    std::cout << "[CONTROLLER] " << msg << std::endl;

}


void Controller::run() {

    LOG("Starting Run Function");

    // As long as we don't enter the 'Done' state we move on.
    while(_current_state->get_state_name() != "Done") {
        go_next();
    }

    LOG("Reached Done State");

}


void Controller::set_state(State* state) {

    // Set to the new state will be copied.
    _current_state.reset(state);

    // Delete the state.
    delete state;

}
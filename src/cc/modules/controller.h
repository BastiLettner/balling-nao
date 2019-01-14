//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_CONTROLLER_H
#define BALLING_NAO_CONTROLLER_H

#include "motion.h"
#include "speech.h"
#include "vision.h"
#include "tactile.h"
#include "../states/state.h"
#include <memory>


class Controller {

public:

    // Initializes the controller.
    // Sets the current state to the RequestBallState
    // and calls its go_next function.
    Controller();

    // Default Destruction
    ~Controller() = default;


    // Copy of the controllers are not allowed
    Controller(Controller&& controller) = delete;

    // Implements state transition.
    // Calls the go next function of the current state.
    //
    void go_next();

    // Prints out logging messages
    void LOG(std::string msg);

    // The main function of the controller
    void run();

    Speech& speech_module() { return _speech; }
    Vision& vision_module() { return _vision; }
    Motion& motion_module() { return _motion; }
    Tactile& tactile_module() { return _tactile; }

    // Set the current state.
    // This function will take ownership of the state passed to it
    // and destroy it after setting it in the unique ptr
    //
    // Args:
    //     state: The new state.
    //
    void set_state(State* state);

private:

    Motion _motion;
    Vision _vision;
    Speech _speech;
    Tactile _tactile;

    std::unique_ptr<State> _current_state;

};

#endif //BALLING_NAO_CONTROLLER_H

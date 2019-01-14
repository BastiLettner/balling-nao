//
// Created by hrs_b on 10.01.19.
//

#include "request_ball.h"
#include "../modules/controller.h"
#include "done.h"
#include "take_ball.h"
#include <assert.h>


RequestBallState::RequestBallState():
    State()
{
    // Set the available commands
    _available_cmds.emplace_back("yes");
    _available_cmds.emplace_back("no");
}


void RequestBallState::go_next(Controller &controller) {

    // 1. Step: Use speech module to say: "Can I have the ball"

    // Create the message
    std::string msg("can i have the ball");

    // TODO: Make Call
    // controller.speech_module().talk(msg);

    // Wait for answer. Create empty msg to fill.
    std::string answer;
    // TODO: Make Call
    // controller.speech_module().listen(answer);

    // 2. Compare it to the valid commands
    if(cmd_valid(answer)) {
        // Either yes or no
        if( answer == "yes" ) {
            // Move to the TakeBallState
            controller.set_state(new TakeBallState());
        }
        else {
            // Assert that the answer is no
            assert(answer == "no");
            // Move to the done state
            controller.set_state(new DoneState());
        }

    }
    // The command was not understood
    else {
        // TODO: Make Call
        // controller.speech_module().not_understood();
        controller.set_state(new RequestBallState());
    }

}
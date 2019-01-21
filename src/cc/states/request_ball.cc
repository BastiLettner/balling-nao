//
// Created by hrs_b on 10.01.19.
//

#include "request_ball.h"
#include "../modules/controller.h"
#include "done.h"
#include "take_ball.h"
#include <assert.h>
#include "../errors/cmd_not_understood_error.h"


RequestBallState::RequestBallState():
    State()
{
    // Set the available commands
    _available_cmds.emplace_back("yes");
    _available_cmds.emplace_back("no");
}


void RequestBallState::go_next(Controller &controller) {

    // 1. Step: Use speech module to say: "Can I have the ball" and retrieve the answer
    std::string answer;
    std::string request = "can i have the ball please";

    // This function only return when a valid command was recorded.
    controller.speech_module().request_response_block(this, request, answer);

    if( answer == "yes " ) {
        // Move to the TakeBallState
        controller.set_state(new TakeBallState());
    }
    else {
        // Assert that the answer is no
        assert(answer == "no ");
        // Move to the done state
        controller.set_state(new DoneState());
    }

}
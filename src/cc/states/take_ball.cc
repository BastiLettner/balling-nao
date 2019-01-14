//
// Created by hrs_b on 10.01.19.
//

#include "take_ball.h"
#include "../modules/controller.h"
#include "request_task.h"


TakeBallState::TakeBallState():
    State()
{
    // No available commands in this State
}

void TakeBallState::go_next(Controller &controller) {

    // 1. Use Motion Module to move arm into <take-ball> position
    controller.motion_module().request_ball_position();

    // Notify the user that the robot is ready to grasp the ball
    // TODO: Make call
    // controller.speech_module().talk("You can hand me the ball now")

    // 2. Use Tactile Module to detect tactile button press
    // Blocks until the button is pressed
    controller.tactile_module().detect_button_pressed("front");

    // 3. Use Motion Module to grasp
    controller.motion_module().grasp();

    // 4. Use vision module to detect if ball is visible
    if (controller.vision_module().ball_visible()) {
        controller.set_state(new RequestTaskState());
    }

    else {
        controller.set_state(new TakeBallState());
    }


}
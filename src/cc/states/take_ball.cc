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

    // Notify the user that the robot is ready to finger_movement the ball
    controller.speech_module().talk("You can hand me the ball now");

    controller.motion_module().finger_movement(0);  // Open the hand

    // 2. Use Tactile Module to detect tactile button press
    // Blocks until the button is pressed
    controller.tactile_module().detect_button_pressed("front");

    // 3. Use Motion Module to finger_movement
    controller.motion_module().finger_movement(1);  // Close hand

    // 4. Use vision module to detect if ball is visible
    if (controller.vision_module().ball_visible()) {
        controller.set_state(new RequestTaskState());
    }

    else {
        controller.set_state(new TakeBallState());
    }

}
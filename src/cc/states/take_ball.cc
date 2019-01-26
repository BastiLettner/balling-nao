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
    controller.motion_module().finger_movement(0);  // Open the hand

    // Notify the user that the robot is ready to finger_movement the ball
    controller.speech_module().talk("You can hand me the ball now");

    // 2. Use Tactile Module to detect tactile button press
    // Blocks until the button is pressed
    ros::Rate loop_rate(10);
    while(!controller.tactile_module().detect_button_pressed("front")){
        ros::spinOnce();
        loop_rate.sleep();
    }

    // 3. Use Motion Module to finger_movement
    controller.motion_module().finger_movement(1);  // Close hand

    bool ball_detected = false;
    while (!ball_detected and !controller.tactile_module().detect_button_pressed("middle")){
        if (controller.vision_module().ball_visible()) ball_detected = true;
        ros::spinOnce();
        loop_rate.sleep();
    }

    // 4. Use vision module to detect if ball is visible
    if (ball_detected) {
        controller.set_state(new RequestTaskState());
    }

    else {
        controller.speech_module().talk("i can not see the ball");
        controller.speech_module().talk("we will have to try again");
        controller.set_state(new TakeBallState());
    }

}
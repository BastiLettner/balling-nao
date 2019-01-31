//
// Created by hrs_b on 10.01.19.
//

#include "take_ball.h"
#include "../core/controller.h"
#include "request_task.h"
#include "../sensors_actors/motion_library.h"


TakeBallState::TakeBallState():
    State()
{
    // No available commands in this State
    _available_cmds.emplace_back("yes");
    _available_cmds.emplace_back("no");
}


void TakeBallState::go_next(Controller &controller) {

    // 1. Use Motion Module to move arm into <take-ball> position
    controller.motion_module().perform_standard_motion(MOTIONS::REQUEST_BALL_POSITION);
    controller.motion_module().perform_standard_motion(MOTIONS::OPEN_RHAND);  // open hand


    // Notify the user that the robot is ready to finger_movement the ball
    controller.speech_module().talk("You can hand me the ball now");

    // 2. Use Tactile Module to detect tactile button press
    // Blocks until the button is pressed
    controller.tactile_module().detect_button_pressed("front");

    // 3. Use Motion Module to finger_movement
    controller.motion_module().perform_standard_motion(MOTIONS::CLOSE_RHAND);  // Close hand

    bool ball_detected = false;
    // Move head towards right hand
    controller.motion_module().perform_standard_motion(MOTIONS::LOOK_AT_BALL);
    ros::Rate loop_rate(10);
    size_t attempts = 0;
    while (!ball_detected and attempts < 30) { // Take three second to look for the ball

        if (controller.vision_module().ball_visible()){
            ball_detected = true;
            controller.speech_module().talk("thank you");
            controller.speech_module().talk("i got the ball");
        }
        ros::spinOnce();
        loop_rate.sleep();
        attempts++;
    }
    controller.motion_module().perform_standard_motion(MOTIONS::HEAD_RESTING_POSITION);

    // 4. Use vision module to detect if ball is visible
    if (ball_detected) {
        controller.set_state(new RequestTaskState());
    }

    // catch case if robot just can not identify the ball in his hand
    else {
        controller.speech_module().talk("i could not see the ball");

        std::string request("do i have the ball?");
        std::string response; // yes or no
        controller.speech_module().request_response_block(this, request, response);

        if(response=="yes "){
            controller.set_state(new RequestTaskState());
        }
        else{
            assert(response == "no ");
            controller.speech_module().talk("then we will have to try again");
            controller.set_state(new TakeBallState());
        }
    }
}
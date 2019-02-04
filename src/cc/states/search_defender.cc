//
// Created by hrs_b on 10.01.19.
//
#include "search_defender.h"
#include "../core/controller.h"
#include "avoid_defender.h"
#include "detect_hoop.h"
#include <functional>


SearchDefenderState::SearchDefenderState(std::string& task):
    State(),
    task(task)
{

}


void SearchDefenderState::go_next(Controller &controller) {

    // 1. if defender visible: move to AvoidDefenderState
    //    if defender not visible: move to DetectHoopState
    SEARCH_MODE mode = SEARCH_MODE::SIMPLE;
    float found_at_head_angle;
    std::function<bool ()> f = [&]() { return controller.vision_module().defender_visible(); };
    bool defender_visible = controller.brain().search().search_routine(
            mode,
            f,
            found_at_head_angle
            );

    if (defender_visible) {
        controller.speech_module().talk("Found Defender");
        ROS_INFO_STREAM("Found defender at angle: ");
        ROS_INFO_STREAM(found_at_head_angle);
        controller.motion_module().request_move_to_position(0.0, 0.0, found_at_head_angle);
        controller.set_state(new AvoidDefenderState(task));
    }
    else {
        ROS_INFO_STREAM("No defender found");
        controller.set_state(new DetectHoopState(task));
    }
}
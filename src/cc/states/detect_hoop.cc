//
// Created by hrs_b on 10.01.19.
//

#include "detect_hoop.h"
#include "../core/controller.h"
#include "move_into_throw_dist.h"
#include "move_into_dunk_pos.h"

DetectHoopState::DetectHoopState(std::string& task):
    State(),
    task(task)
{

}

void DetectHoopState::go_next(Controller &controller) {

    // 1. if hoop visible: move to hoop position
    //    if hoop not visible: start over
    SEARCH_MODE mode = SEARCH_MODE::SIMPLE;
    float found_at_head_angle;
    std::function<bool ()> f = [&]() { return controller.vision_module().hoop_visible(); };
    bool hoop_visible = controller.brain().search().search_routine(
            mode,
            f,
            found_at_head_angle
    );

    if (hoop_visible) {
        controller.speech_module().talk("Found Hoop");
        controller.motion_module().walk_to_position(0.0, 0.0, found_at_head_angle);
        if (task == "dunk") controller.set_state(new MoveIntoThrowDistanceState());
        else if (task == "throw") controller.set_state(new MoveIntoDunkPositionState());

    }
    else {
        controller.set_state(new DetectHoopState(task));
    }

}


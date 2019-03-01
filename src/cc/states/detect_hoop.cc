// Detect hoop class definition

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

    // 1. Search for hoop
    if(state_routine(SEARCH_MODE::VERY_SIMPLE_SEARCH, controller)) return;
    else if(state_routine(SEARCH_MODE::MARKER_CLOSE, controller)) return;
    else if(state_routine(SEARCH_MODE::SIMPLE, controller)) return;
    else {
        controller.speech_module().talk("Could not find hoop after all searching efforts");
        ROS_INFO_STREAM("Could not find the hoop after advanced search. Remaining in DetectHoopState.");
    }
}


bool DetectHoopState::state_routine(SEARCH_MODE mode, Controller& controller) {

    // Make the search
    float found_at_head_angle;
    std::function<bool ()> f = [&]() { return controller.vision_module().hoop_visible(); };
    bool hoop_visible = controller.brain().search().search_routine(
            mode,
            f,
            found_at_head_angle
    );

    // Of the hoop was found
    if (hoop_visible) {
        controller.speech_module().talk("Found Hoop");
        controller.motion_module().request_move_to_position(0.0, 0.0, found_at_head_angle);

        if (task == "dunk ") {
            controller.set_state(new MoveIntoDunkPositionState());
            return hoop_visible;
        }

        else if (task == "throw ") {
            controller.set_state(new MoveIntoThrowDistanceState());
            return hoop_visible;
        }

        else {
            throw std::runtime_error("Incorrect task in DetectHoopState");
        }
    }

    else {
        return false;
    }
}


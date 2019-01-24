//
// Created by hrs_b on 10.01.19.
//

#include "search_defender.h"
#include "../modules/controller.h"
#include "avoid_defender.h"
#include "detect_hoop.h"


SearchDefenderState::SearchDefenderState(std::string& task):
    State(),
    task(task)
{

}


void SearchDefenderState::go_next(Controller &controller) {

    // 1. if defender visible: move to AvoidDefenderState
    //    if defender not visible: move to DetectHoopState
    if (controller.vision_module().defender_visible()) {
        controller.set_state(new )
    }

}
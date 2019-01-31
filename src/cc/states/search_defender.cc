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
    cv::Mat position;
    std::function<bool(cv::Mat)> goal_function = std::bind(&Vision::defender_visible, &controller.vision_module(), std::placeholders::_1);
    auto f = [&](cv::Mat) { return controller.vision_module().defender_visible(position); };
    bool defender_visible = controller.brain().search().search_routine(
            mode,
            f,
            position
            );

    if (defender_visible) {
        controller.set_state(new AvoidDefenderState(task));
    }
    else {
        controller.set_state(new DetectHoopState());
    }

}
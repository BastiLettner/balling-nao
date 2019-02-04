//
// Created by hrs_b on 10.01.19.
//

#include "avoid_defender.h"
#include "detect_hoop.h"
#include "../core/controller.h"

AvoidDefenderState::AvoidDefenderState(std::string& task):
    task(task)
{

}

void AvoidDefenderState::go_next(Controller &controller) {
    ROS_INFO_STREAM("Avoiding defender.");
    controller.set_state(new DetectHoopState(task));
}


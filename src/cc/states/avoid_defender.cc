//
// Created by hrs_b on 10.01.19.
//

#include "avoid_defender.h"
#include "detect_hoop.h"
#include "../core/controller.h"
#include "math.h"

AvoidDefenderState::AvoidDefenderState(std::string& task):
    task(task)
{

}

void AvoidDefenderState::go_next(Controller &controller) {

    ROS_INFO_STREAM("Avoiding defender.");

    std::vector<WayPoint> path;

    // Calculate the way points of the trajectory
    controller.brain().path_planning().calculate_path(path);

    // Walk along the path.
    for(auto& way_point: path) {
        controller.motion_module().request_move_to_position(way_point.x, way_point.y, way_point.theta);
    }

    controller.set_state(new DetectHoopState(task));
}


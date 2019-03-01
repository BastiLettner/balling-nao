// Path finding class definition

//
// Created by hrs_b on 20.02.19.
//

#include "path_planning.h"

PathPlanning::PathPlanning(Search &search, Vision &vision):
    _search(search),
    _vision(vision)
{
    // Nothing to do here
}


void PathPlanning::calculate_path(std::vector<WayPoint> &path) {

    float yaw = get_defender_rotation();  // get the orientation
    float span = get_defender_span();  // get the span
    int orientation = (yaw > 0) - (yaw < 0); //1 if yaw is positive, -1 else

    // If the span is out of this range we assume that the defender was not detected
    // correctly. Thus we set the value to a large obstacle.
    // The values were obtained empirically
    if (span < 0.10f and span > -0.10f){
        span = orientation * 0.40f;
    }
    int direction = (span >= 0) - (span < 0); //1 if span is positive, -1 else

    // Get marker position, extract angle and distance.
    auto defender_position = _vision.get_marker_mat_t_defender();
    auto marker_x = defender_position.at<float>(0);
    auto marker_z = defender_position.at<float>(2);
    float walking_x = marker_z - 0.40f; // Stop 40 cm in front of the defender
    auto walking_angle = (float)tan((double)marker_x/(double)marker_z);

    // Turn towards defender
    WayPoint first;
    first.x = 0.0; first.y = 0.0; first.theta = - walking_angle;
    path.push_back(first);

    // Walk towards defender
    WayPoint second;
    second.x = walking_x; second.y = 0.0; second.theta = 0.0;
    path.push_back(second);

    // Align with defender based on rotation
    WayPoint third;
    third.x = 0.0; third.y = 0.0; third.theta = 0.8f * 6.0f * yaw;
    path.push_back(third);

    // Side step away from defender based on span (size)
    WayPoint fourth;
    fourth.x = 0.0; fourth.y = -0.8f*direction*(abs(span) + 0.25f); fourth.theta = 0.0;
    path.push_back(fourth);

    // Walk past defender based on span (size)
    WayPoint fifth;
    fifth.x = 0.8f * span * direction + 0.2f; fifth.y = 0.0; fifth.theta = 0.0;
    path.push_back(fifth);

    // Turn to the left if dodged right and vice versa.
    WayPoint sixth;
    sixth.y = 0.0; sixth.y = 0.0; sixth.theta = direction * 3.1415f * 0.25f;
    path.push_back(sixth);

}


float PathPlanning::get_defender_rotation() {

    ros::Rate loop_rate(10);
    loop_rate.sleep();
    ros::spinOnce();

    // If we can't see the defender we first search.
    if (not _vision.defender_visible()) {
        SEARCH_MODE mode = SEARCH_MODE::VERY_SIMPLE_SEARCH;  // Using the simple search
        float found_at_head_angle;
        std::function<bool ()> f = [&]() { return _vision.defender_visible(); };
        bool defender_visible = _search.search_routine(
                mode,
                f,
                found_at_head_angle
        );
        if (defender_visible) {
            // If we found the defender we adjust our position
            _search.get_motion().request_move_to_position(0.0, 0.0, found_at_head_angle);

        }
    }
    // now we calculate the rotation
    float yaw = _vision.get_defender_rotation();  // Get the rotation 
    return yaw;

}


float PathPlanning::get_defender_span() {

    size_t attempts = 0;
    ros::Rate loop_rate(10);
    float span = 0.0f;
    while (attempts < 30) { // Take 3 seconds to look for the defender

        span = _vision.detect_defender_span();
        ros::spinOnce();
        loop_rate.sleep();
        // If the span is within this range we assume that we detected something.
        if (span > 0.10f or span < -0.10f) break;
        attempts++;
    }

    return span;
}

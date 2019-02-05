
//
// Created by hrs_b on 31.01.19.
//

#include <utility>
#include "search.h"
#include <exception>
#include "../../sensors_actors/motion_library.h"


Search::Search(Motion &motion):
    _motion(motion)
{

}

bool Search::search_routine(SEARCH_MODE &mode, std::function<bool()>& goal_function, float& found_at_head_angle) {

    if(mode == SEARCH_MODE::SIMPLE) {
        return simple_search(found_at_head_angle, goal_function);
    }
    if(mode == SEARCH_MODE::INTERMEDIATE){
        return simple_search(found_at_head_angle, goal_function);
    }
    if(mode == SEARCH_MODE::ADVANCED) {
        return simple_search(found_at_head_angle, goal_function);
    }
    if(mode == SEARCH_MODE::MARKER_CLOSE) {
        return marker_close_search(found_at_head_angle, goal_function);
    }
}

bool Search::simple_search(float& found_at_head_angle, std::function<bool()>& goal_function) {

    bool has_found = false;

    for(auto& motion : MOTIONS::SEARCH::SIMPLE_SEARCH.motions) {
        if(perform_head_search(motion, goal_function)) {
            found_at_head_angle = motion.angles[0];
            ROS_INFO_STREAM("Found hoop after close search");
            return true;
        }
    }

    return has_found;

}

bool Search::intermediate_search(float& found_at_angle, std::function<bool()>& goal_function) {
    return false;
}

bool Search::advanced_search(float& found_at_angle, std::function<bool()>& goal_function) {
    return false;
}

bool Search::check_goal(std::function<bool()>& goal_function, bool& has_found) {

    ros::Rate loop_rate(10);
    assert(!has_found);

    int attempts = 0;
    // look for 2 secs (= 20 attempts with loop_rate=10)
    while(!has_found and attempts < 10) {
        has_found = goal_function();
        ros::spinOnce();
        loop_rate.sleep();
        attempts++;
    }
}


bool Search::marker_close_search(float &found_at_head_angle, std::function<bool()> &goal_function) {


    bool has_found = false;

    for(auto& motion : MOTIONS::SEARCH::MARKER_CLOSE_SEQ.motions) {
        if(perform_head_search(motion, goal_function)) {
            found_at_head_angle = motion.angles[0];
            return true;
        }
    }

    return has_found;

}


bool Search::perform_head_search(BaseMotion &motion, std::function<bool()> &goal_function) {

    bool has_found = false;

    _motion.perform_standard_motion(motion);
    ros::Rate loop_rate(10);
    ros::spinOnce();
    loop_rate.sleep();
    check_goal(goal_function, has_found);
    if(has_found) {
        _motion.perform_standard_motion(MOTIONS::HEAD_RESTING_POSITION);
        return true;
    }
    else {
        return false;
    }

}


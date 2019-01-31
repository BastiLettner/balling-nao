
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
}

bool Search::simple_search(float& found_at_angle, std::function<bool()>& goal_function) {

    ros::Rate loop_rate(10);
    bool has_found = false;

    _motion.perform_standard_motion(MOTIONS::HEAD_FAR_LEFT);
    check_goal(goal_function, has_found);
    if(has_found) {
        found_at_angle = MOTIONS::HEAD_FAR_LEFT.angles[0];
        _motion.perform_standard_motion(MOTIONS::HEAD_RESTING_POSITION);
        return true;
    }

    _motion.perform_standard_motion(MOTIONS::HEAD_LEFT);
    check_goal(goal_function, has_found);
    if(has_found) {
        found_at_angle = MOTIONS::HEAD_FAR_LEFT.angles[0];
        _motion.perform_standard_motion(MOTIONS::HEAD_RESTING_POSITION);
        return true;
    }


    _motion.perform_standard_motion(MOTIONS::HEAD_RESTING_POSITION);
    check_goal(goal_function, has_found);
    if (has_found) {
        found_at_angle = MOTIONS::HEAD_FAR_LEFT.angles[0];
        _motion.perform_standard_motion(MOTIONS::HEAD_RESTING_POSITION);
        return true;
    }

    _motion.perform_standard_motion(MOTIONS::HEAD_RIGHT);
    check_goal(goal_function, has_found);
    if (has_found) {
        found_at_angle = MOTIONS::HEAD_FAR_LEFT.angles[0];
        _motion.perform_standard_motion(MOTIONS::HEAD_RESTING_POSITION);
        return true;
    }

    _motion.perform_standard_motion(MOTIONS::HEAD_FAR_RIGHT);
    check_goal(goal_function, has_found);
    if (has_found) {
        found_at_angle = MOTIONS::HEAD_FAR_LEFT.angles[0];
        _motion.perform_standard_motion(MOTIONS::HEAD_RESTING_POSITION);
        return true;
    }

    _motion.perform_standard_motion(MOTIONS::HEAD_RESTING_POSITION);

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
    while(!has_found and attempts < 10) {
        has_found = goal_function();
        ros::spinOnce();
        loop_rate.sleep();
        attempts++;
    }
}


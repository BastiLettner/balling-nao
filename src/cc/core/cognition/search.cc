
//
// Created by hrs_b on 31.01.19.
//

#include <utility>
#include "search.h"
#include <exception>
#include "../../sensors_actors/motion_library.h"
#include <opencv2/imgproc/imgproc.hpp>


Search::Search(Motion &motion):
    _motion(motion)
{

}

bool Search::search_routine(SEARCH_MODE &mode, std::function<bool(cv::Mat)>& goal_function, cv::Mat& position) {

    if(mode == SEARCH_MODE::SIMPLE) {
        return simple_search(position, goal_function);
    }
    if(mode == SEARCH_MODE::INTERMEDIATE){
        return simple_search(position, goal_function);
    }
    if(mode == SEARCH_MODE::ADVANCED) {
        return simple_search(position, goal_function);
    }
}

bool Search::simple_search(cv::Mat& position, std::function<bool(cv::Mat)>& goal_function) {

    ros::Rate loop_rate(10);
    bool has_found = false;

    _motion.perform_standard_motion(MOTIONS::HEAD_LEFT);
    check_goal(position, goal_function, has_found);
    if(has_found) {
        return true;
    }


    _motion.perform_standard_motion(MOTIONS::HEAD_RESTING_POSITION);
    check_goal(position, goal_function, has_found);
    if (has_found) {
        return true;
    }

    _motion.perform_standard_motion(MOTIONS::HEAD_RIGHT);
    check_goal(position, goal_function, has_found);

    return has_found;

}

bool Search::intermediate_search(cv::Mat& position, std::function<bool(cv::Mat)>& goal_function) {
    return false;
}

bool Search::advanced_search(cv::Mat& position, std::function<bool(cv::Mat)>& goal_function) {
    return false;
}

bool Search::check_goal(cv::Mat &position, std::function<bool(cv::Mat)>& goal_function, bool& has_found) {

    ros::Rate loop_rate(10);
    assert(!has_found);

    int attempts = 0;
    while(!has_found and attempts < 10) {
        has_found = goal_function(position);
        ros::spinOnce();
        loop_rate.sleep();
        attempts++;
    }
}


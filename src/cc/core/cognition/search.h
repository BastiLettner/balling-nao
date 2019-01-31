//
// Created by hrs_b on 31.01.19.
//

#ifndef BALLING_NAO_SEARCH_H
#define BALLING_NAO_SEARCH_H


#include "../../sensors_actors/motion.h"

namespace cv {
    class Mat;
}


enum SEARCH_MODE { SIMPLE, INTERMEDIATE, ADVANCED };


class Search {

public:

    explicit Search(Motion& motion);

    bool search_routine(SEARCH_MODE& mode, std::function<bool(cv::Mat)>& goal_function, cv::Mat& position);

    bool simple_search(cv::Mat& position, std::function<bool(cv::Mat)>& goal_function);

    bool intermediate_search(cv::Mat& position, std::function<bool(cv::Mat)>& goal_function);

    bool advanced_search(cv::Mat& position, std::function<bool(cv::Mat)>& goal_function);

    bool check_goal(cv::Mat& position, std::function<bool(cv::Mat)>& goal_function, bool& has_found);

private:

    Motion& _motion;

};

#endif //BALLING_NAO_SEARCH_H

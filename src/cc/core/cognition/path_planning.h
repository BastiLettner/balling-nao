//
// Created by hrs_b on 20.02.19.
//

#ifndef BALLING_NAO_PATH_PLANNING_H
#define BALLING_NAO_PATH_PLANNING_H


#include "../../sensors_actors/vision.h"
#include "search.h"


typedef struct {

    float x, y, theta;

} WayPoint;



class PathPlanning {

    // Path Planning module for the defender avoidance


public:

    PathPlanning(Search& search, Vision& vision);

    float get_defender_rotation();

    float get_defender_span();

    void calculate_path(std::vector<WayPoint>& path);


private:

    Search& _search;
    Vision& _vision;

};

#endif //BALLING_NAO_PATH_PLANNING_H

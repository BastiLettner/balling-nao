// Header file for the path planning.
// The path planning functions acquire data on the defender and how to avoid it.
// structure : single class

//
// Created by hrs_b on 20.02.19.
//

#ifndef BALLING_NAO_PATH_PLANNING_H
#define BALLING_NAO_PATH_PLANNING_H


#include "../../sensors_actors/vision.h"
#include "search.h"


// Small helper that implements a way point (Position in the room)
typedef struct {

    float x, y, theta;

} WayPoint;



class PathPlanning {

    // Path Planning module for the defender avoidance

public:

    // Constructor
    //
    // Args:
    //     search: The Search module (also holds motion)
    //     vision: The vision module
    //
    PathPlanning(Search& search, Vision& vision);

    // Calculate the defender rotation using the aruco marker rotation
    // Runs a small search routine first to make sure the marker is visible,
    // then extracts the marker orientation
    //
    // Returns:
    //     orientation
    //
    float get_defender_rotation();

    // Calculate the defender span (width)
    // Uses color segmentation to find the most exterior color blob (assumed to the the outer edge of the
    // defender) Based on the marker position and the color blob we can get span.
    //
    // Returns:
    //     span: Half width of the defending object.
    //
    float get_defender_span();

    // Calculates all the way points that make of the defender avoidance trajectory
    // Uses the span and the defender rotation to calculate the path.
    //
    // Args:
    //     path: Empty vector. This function puts in the way points.
    //
    void calculate_path(std::vector<WayPoint>& path);


private:

    Search& _search;
    Vision& _vision;

};

#endif //BALLING_NAO_PATH_PLANNING_H

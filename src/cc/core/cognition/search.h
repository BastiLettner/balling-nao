// Header file for the searching routines
// Searching is done progressively turning NAO's head to expand its field of view
// structure : one class

//
// Created by hrs_b on 31.01.19.
//

#ifndef BALLING_NAO_SEARCH_H
#define BALLING_NAO_SEARCH_H


#include "../../sensors_actors/motion.h"

namespace cv {
    class Mat;
}

// The different search modes
// For each mode the robot performs a different sequence of motions to
// get a wider field of sight.
enum SEARCH_MODE : int { SIMPLE, MARKER_CLOSE, VERY_SIMPLE_SEARCH };


class Search {

    // Implements cognition task searching.
    // Uses motion and vision to find aruco markers.
    // There are different modes of searching which are appropriate in different situations

public:

    // Constructor
    //
    // Args:
    //     motion: The motion module. We need this to perform the motions.
    //
    explicit Search(Motion& motion);

    // Implements a search routine.
    // Depending on the mode it executes different motions while looking for the goal.
    //
    // Args:
    //     mode: The mode of searching
    //     goal_function: A function that returns true if the goal is seen and false else.
    //     found_at_head_angel: This value will be filled out by the search routine in case the target
    //                          was found. It is the head angle at which the target was found.
    //                          E.g. the robot looked 45 degree to the left while seeing the target.
    //                          Using this value the robots position can be adjusted to face the target better.
    //
    // Return:
    //     true, if target was found
    bool search_routine(SEARCH_MODE& mode, std::function<bool()>& goal_function, float& found_at_head_angle);

    // The simple search routine (MODE: SIMPLE_SEARCH)
    // Uses 5 head angles -90, -45, 0, 45, 90 while looking straight ahead
    //
    // Args:
    //     found_at_head_angle: See search_routine.
    //     goal_function: See search_routine.
    //
    // Return:
    //     true, if target was found
    bool simple_search(float& found_at_head_angle, std::function<bool()>& goal_function);

    // The marker close search routine (MODE: MARKER_CLOSE)
    // Uses 5 head angles -90, -45, 0, 45, 90 while looking straight down
    //
    // Args:
    //     found_at_head_angle: See search_routine.
    //     goal_function: See search_routine.
    //
    // Return:
    //     true, if target was found
    bool marker_close_search(float& found_at_head_angle, std::function<bool()>& goal_function);

    // The very simple search routine (MODE: SIMPLE_SEARCH)
    // Uses 5 head angles -45, 0, 45 while looking straight ahead
    //
    // Args:
    //     found_at_head_angle: See search_routine.
    //     goal_function: See search_routine.
    //
    // Return:
    //     true, if target was found
    bool very_simple_search(float& found_at_head_angle, std::function<bool()>& goal_function);

    // Moves the robot head into a position and checks the goal function
    //
    // Args:
    //     motion: The head position
    //     goal_function: See search_routine
    //
    // Return:
    //     true, if target was found
    bool perform_head_search(BaseMotion &motion, std::function<bool()> &goal_function);

    // Evaluates the goal function
    //
    // Args:
    //     goal_function: See search_routine
    //     has_found: Set true if target is visible
    bool check_goal(std::function<bool()>& goal_function, bool& has_found);

    // Getter
    Motion& get_motion() { return _motion; }

private:

    Motion& _motion;

};

#endif //BALLING_NAO_SEARCH_H

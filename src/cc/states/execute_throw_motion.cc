// Execute throw motion class definition

//
// Created by hrs_b on 10.01.19.
//

#include "execute_throw_motion.h"
#include "../core/controller.h"
#include "../sensors_actors/motion_library.h"
#include "done.h"


ExecuteThrowMotionState::ExecuteThrowMotionState():
        State()
{

}

void ExecuteThrowMotionState::go_next(Controller &controller) {

    ros::spinOnce(); // update image

    // Check if we can see the hoop and if not make another search and readjust the position
    if (!controller.vision_module().hoop_visible()){
        float found_at_head_angle;
        std::function<bool ()> f = [&]() { return controller.vision_module().hoop_visible(); };
        SEARCH_MODE mode = SEARCH_MODE::MARKER_CLOSE;
        bool hoop_visible = controller.brain().search().search_routine(
                mode,
                f,
                found_at_head_angle
        );
        if(hoop_visible) {
            controller.motion_module().request_move_to_position(0.0, 0.0, found_at_head_angle);
        }
        else {
            controller.speech_module().talk("i lost the hoop");
        }
    }
    // Use the marker position to select a shot and execute it
    auto hoop_position_x = controller.vision_module().get_marker_mat_t_hoop().at<float>(0);
    if (-0.10 < hoop_position_x and hoop_position_x <= 0.1) {
        controller.motion_module().perform_motion_sequence(MOTIONS::THROW::STRAIGHT_OVER_HEAD_EXT_SEQ);
        ROS_INFO_STREAM("Executed throwing motion STRAIGHT_OVER_HEAD_EXT_SEQ " + std::to_string(hoop_position_x));
    }
    if (hoop_position_x <= -0.10) {
        controller.motion_module().perform_motion_sequence(MOTIONS::THROW::LEFT_OVER_HEAD_EXT_SEQ);
        ROS_INFO_STREAM("Executed throwing motion LEFT_OVER_HEAD_EXT_SEQ " + std::to_string(hoop_position_x));
    }
    if (hoop_position_x > 0.10) {
        controller.motion_module().perform_motion_sequence(MOTIONS::THROW::RIGHT_OVER_HEAD_EXT_SEQ);
        ROS_INFO_STREAM("Executed throwing motion RIGHT_OVER_HEAD_EXT_SEQ " + std::to_string(hoop_position_x));
    }

    controller.speech_module().talk("Scored!");

    controller.set_state(new DoneState());

}
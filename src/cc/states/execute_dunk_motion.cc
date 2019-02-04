//
// Created by hrs_b on 10.01.19.
//

#include "execute_dunk_motion.h"
#include "../core/controller.h"
#include "../sensors_actors/motion_library.h"
#include "done.h"

ExecuteDunkMotionState::ExecuteDunkMotionState():
    State()
{

}

void ExecuteDunkMotionState::go_next(Controller &controller) {

    controller.motion_module().perform_standard_motion(MOTIONS::DUNK_MOTION_1);
    controller.motion_module().perform_standard_motion(MOTIONS::DUNK_MOTION_2);
    controller.motion_module().perform_standard_motion(MOTIONS::DUNK_MOTION_3);
    controller.motion_module().perform_standard_motion(MOTIONS::DUNK_MOTION_4);
    controller.motion_module().perform_standard_motion(MOTIONS::OPEN_RHAND);
    controller.speech_module().talk("Slam duuuuuuuuuuuuuuunk");
    controller.set_state(new DoneState());

}
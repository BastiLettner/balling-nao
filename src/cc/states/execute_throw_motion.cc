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

    controller.motion_module().perform_motion_sequence(MOTIONS::THROW::STRAIGHT_OVER_HEAD_EXT_SEQ);

    controller.speech_module().talk("Scored!");
    controller.set_state(new DoneState());

}
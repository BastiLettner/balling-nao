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

    // Execute the motion
    controller.motion_module().perform_motion_sequence(MOTIONS::DUNK::DUNK_SEQ);

    controller.speech_module().talk("Slam dunk");
    controller.set_state(new DoneState());

}
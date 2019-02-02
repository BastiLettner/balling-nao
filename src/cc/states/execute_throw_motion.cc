//
// Created by hrs_b on 10.01.19.
//

#include "execute_throw_motion.h"
#include "../core/controller.h"
#include "../sensors_actors/motion_library.h"
#include "done.h"

namespace throwing_motions {

    BaseMotion THROW1 = BaseMotion(
            {"RShoulderPitch"},
            {20.0f * PI_180},
            0.8,
            30.0
    );

    BaseMotion OPEN_RHAND = BaseMotion(
            {"RHand"},
            {50.0f*PI_180},
            0.2,
            15.0
    );
}

ExecuteThrowMotionState::ExecuteThrowMotionState():
        State()
{

}

void ExecuteThrowMotionState::go_next(Controller &controller) {

    controller.motion_module().perform_standard_motion(MOTIONS::CLOSE_RHAND);
    controller.motion_module().perform_standard_motion(MOTIONS::PULLBACK_RARM);
    controller.motion_module().perform_standard_motion(throwing_motions::THROW1, false);
    controller.motion_module().perform_standard_motion(throwing_motions::OPEN_RHAND, false);

    controller.speech_module().talk("Scored!");
    controller.set_state(new DoneState());

}
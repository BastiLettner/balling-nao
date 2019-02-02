//
// Created by hrs_b on 10.01.19.
//

#include "done.h"
#include "../errors/balling_nao_error.h"
#include "../core/controller.h"


DoneState::DoneState():
    State()
{
    // Nothing to do here
}

void DoneState::go_next(Controller &controller) {

    throw BallingNaoError("Calling go next in Done State doesn't make sense");

}
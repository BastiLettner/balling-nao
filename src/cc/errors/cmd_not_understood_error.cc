//
// Created by hrs_b on 15.01.19.
//

#include "cmd_not_understood_error.h"
#include "../modules/speech.h"
#include "../states/state.h"


CmdNotUnderstoodError::CmdNotUnderstoodError(
        std::string error_msg,
        std::vector<std::string>& were_available,
        std::string recorded_sentence
):
    BallingNaoError(std::move(error_msg))
{
    available = were_available;
    recorded = std::move(recorded_sentence);
}

const char* CmdNotUnderstoodError::what() const noexcept {

    std::string error_msg = _what_arg;
    error_msg += ". \n";
    error_msg += "The available command where: ";
    for(const std::string& cmd: available) {
        error_msg += cmd + ", ";
    }
    error_msg += ". \n";
    error_msg += "The recorded sentence is: " + recorded;
    return error_msg.c_str();

}

void CmdNotUnderstoodError::handle(Speech& speech, State* state) {

    speech.talk("i did not hear a valid command");

}

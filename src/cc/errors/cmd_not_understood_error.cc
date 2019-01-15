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

    if(recorded.empty()) {
        speech.talk("i didn't hear anything");
    }
    else {
        std::string nao_notify_user;
        nao_notify_user += "i heard " + recorded;
        nao_notify_user += "i am in state " + state->get_state_name() + " now";
        nao_notify_user += "in this state i understand ";
        if(state->get_available_cmds().empty()) {
            nao_notify_user += "no command";
        }
        else {
            for(const std::string& cmd: state->get_available_cmds()) {
                nao_notify_user.append(cmd);
                nao_notify_user.append(" or ");
            }
        }
        speech.talk(nao_notify_user);
    }

}

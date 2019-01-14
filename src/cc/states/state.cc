//
// Created by hrs_b on 10.01.19.
//


#include "state.h"
#include <algorithm>


bool State::cmd_valid(std::string &cmd) {

    return std::find(_available_cmds.begin(), _available_cmds.end(), cmd) != _available_cmds.end();

}

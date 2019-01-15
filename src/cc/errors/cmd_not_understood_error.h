//
// Created by hrs_b on 15.01.19.
//

#ifndef BALLING_NAO_CMD_NOT_UNDERSTOOD_ERROR_H
#define BALLING_NAO_CMD_NOT_UNDERSTOOD_ERROR_H

#include <vector>
#include "balling_nao_error.h"


class CmdNotUnderstoodError: public BallingNaoError {

public:

    CmdNotUnderstoodError(
            std::string error_msg,
            std::vector<std::string>& were_available,
            std::string recorded_sentence
            );

    std::vector<std::string> available;
    std::string recorded;

    const char* what() const noexcept override;

};


#endif //BALLING_NAO_CMD_NOT_UNDERSTOOD_ERROR_H

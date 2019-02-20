//
// Created by hrs_b on 15.01.19.
//

#ifndef BALLING_NAO_CMD_NOT_UNDERSTOOD_ERROR_H
#define BALLING_NAO_CMD_NOT_UNDERSTOOD_ERROR_H

#include <vector>
#include "balling_nao_error.h"

class Speech;
class State;


class CmdNotUnderstoodError: public BallingNaoError {

    // Exception for not understood commands
    // When the robot records and the results don't match the vocab this exception is used.

public:

    // Constructor
    //
    // Args:
    //     error_msg: Message describing the error
    //     were_available: The commands which were actually available
    //     recorded_sentence: The actual recording
    CmdNotUnderstoodError(
            std::string error_msg,
            std::vector<std::string>& were_available,
            std::string recorded_sentence
            );

    std::vector<std::string> available;
    std::string recorded;

    // Returns the message containing information about the exception
    const char* what() const noexcept override;

    // To handle the exception when caught.
    // Makes the robot say: "Did not understand"
    void handle(Speech& speech, State* state);

};


#endif //BALLING_NAO_CMD_NOT_UNDERSTOOD_ERROR_H

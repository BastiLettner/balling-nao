//
// Created by hrs_b on 14.01.19.
//

#include "balling_nao_error.h"

BallingNaoError::BallingNaoError(const std::string what_arg):
    _what_arg(what_arg)
{

}


const char* BallingNaoError::what() const noexcept {

    return _what_arg.c_str();

}
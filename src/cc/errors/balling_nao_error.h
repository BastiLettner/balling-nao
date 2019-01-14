//
// Created by hrs_b on 14.01.19.
//

#ifndef BALLING_NAO_BALLING_NAO_ERROR_H
#define BALLING_NAO_BALLING_NAO_ERROR_H

#include <exception>
#include <string>


class BallingNaoError: public std::exception {

public:

    explicit BallingNaoError(std::string what_arg);

    const char* what() const noexcept override;

    const std::string get_msg() { return _what_arg; }

protected:

    const std::string _what_arg;

};

#endif //BALLING_NAO_BALLING_NAO_ERROR_H

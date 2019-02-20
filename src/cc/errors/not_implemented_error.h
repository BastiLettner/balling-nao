//
// Created by hrs_b on 14.01.19.
//

#ifndef BALLING_NAO_UNIMPLEMENTED_ERROR_H
#define BALLING_NAO_UNIMPLEMENTED_ERROR_H

#include "balling_nao_error.h"


class NotImplementedError: public BallingNaoError {

    // For not implemented functions. Mainly fo development.

public:

    NotImplementedError();

};


#endif //BALLING_NAO_UNIMPLEMENTED_ERROR_H

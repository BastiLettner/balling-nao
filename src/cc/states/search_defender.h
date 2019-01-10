//
// Created by hrs_b on 10.01.19.
//

#ifndef BALLING_NAO_SEARCH_POTENTIAL_DEFENDER_H
#define BALLING_NAO_SEARCH_POTENTIAL_DEFENDER_H

#include "state.h"

class Controller;


class SearchDefenderState: public State {

public:

    SearchDefenderState() = default;
    virtual ~SearchDefenderState() = default;

    virtual void go_next(Controller& controller) override;

    virtual std::string& get_state_name() override { return _state_name; }

private:

    static const std::string _state_name = "Search Defender State";

};
#endif //BALLING_NAO_SEARCH_POTENTIAL_DEFENDER_H

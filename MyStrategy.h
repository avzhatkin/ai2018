#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include <map>

class MyStrategy : public Strategy {
public:
    MyStrategy();

    void act(const model::Robot& me, const model::Rules& rules, const model::Game& world, model::Action& action) override;
    
private:
    struct MyBot {
        enum Roles {
            Forward,
            Keeper
        } role = Forward;
    };

private:
    std::map<int, MyBot> m_bots;
};

#endif

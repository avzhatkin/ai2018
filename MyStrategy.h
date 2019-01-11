#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include <map>
#include "linal.h"

class MyStrategy : public Strategy {
public:
    MyStrategy();

    void act(const model::Robot& me, const model::Rules& rules, const model::Game& world, model::Action& action) override;

    void init(const model::Rules& rules, const model::Game& game);
    
private:
    struct MyBot {
        enum Roles {
            Forward,
            Keeper
        } role = Forward;
    };

public:
    static int s_tick;

private:
    std::map<int, MyBot> m_bots;
};

#endif // _MY_STRATEGY_H_

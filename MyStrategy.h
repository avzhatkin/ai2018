#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include <map>
#include <list>
#include "linal.h"
using linal::operator""_r;

class MyStrategy : public Strategy {
public:
    MyStrategy();

    void act(const model::Robot& me, const model::Rules& rules, const model::Game& world, model::Action& action) override;

    void init(const model::Rules& rules, const model::Game& game);

    struct DebugSphere {
        linal::vec3 center;
        linal::real_t radius;
        linal::vec3 color;
        linal::real_t alpha;

        DebugSphere(const linal::vec3& p, linal::real_t r, const linal::vec3& c, linal::real_t a) :
            center(p), radius(r * 1.11_r), color(c), alpha(a) {}
    };

    void addDebugSphere(DebugSphere&& sphere);

#ifdef MY_DEBUG
    std::list<DebugSphere> m_debugSpheres;

    std::string custom_rendering() override;
#endif
    
private:
    struct MyBot {
        enum Roles {
            Forward,
            Keeper
        } role = Forward;
        linal::vec3 target;
        linal::vec3 target_speed;
        linal::real_t jump_speed = 0.0;
        bool use_nitro = false;

        bool ready = false;
    };

    size_t m_ready = 0;

public:
    static int s_tick;

private:
    std::map<int, MyBot> m_bots;
};

#endif // _MY_STRATEGY_H_

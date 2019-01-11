#include "MyStrategy.h"
#include <algorithm>
#include "linal.h"
#ifdef MY_DEBUG
#include <chrono>
#endif
using namespace linal;
using namespace std;
using namespace model;

alignas(16) static Rules s_rules;
static Arena& s_arena = s_rules.arena;
static vec3 s_home_pos;
static vec3 s_goal_pos;
int MyStrategy::s_tick = -1;
real_t s_timestep;
real_t s_microstep;

__forceinline real_t sign(real_t v)
{
    return v / abs(v);
}

#ifdef MY_DEBUG
//////////////////////////////////////////////////////////////////////////
//
//
struct PerfCounter
{
    static const size_t bufferSize = 16;

    int64_t buffer[bufferSize] = {};
    int64_t sum = 0;
    int index = 0;
    int size = 0;

    void push(int64_t time)
    {
        if (size < bufferSize)
        {
            ++size;
        }
        else
        {
            sum -= buffer[index];
        }

        sum += time;
        buffer[index++] = time;
        index = index % bufferSize;
    }

    int64_t get()
    {
        return sum / size;
    }
} perf;
#endif

//////////////////////////////////////////////////////////////////////////
//
//
struct alignas(16) Entity
{
    vec3 pos;
    vec3 vel;
    real_t radius = 0.0_r;
    real_t mass = 0.0_r;
    real_t arena_e = 0.0_r;
    bool touch = false;

    static real_t s_entity_e;
};

real_t Entity::s_entity_e = 0.0_r;

struct World
{
    Entity ball;
    vector<Entity> bots;
};

World s_world;

struct TouchInfo
{
    vec3 normal;            // from object to arena or from first object to second
    real_t depth = 0.0_r;   // penetration depth
};

void move(Entity& e, real_t timestep)
{
    e.vel.clamp((real_t)s_rules.MAX_ENTITY_SPEED);
    e.pos += e.vel * timestep;
    e.pos.y -= (real_t)s_rules.GRAVITY * timestep * timestep / 2.0_r;
    e.vel.y -= (real_t)s_rules.GRAVITY * timestep;
}

//////////////////////////////////////////////////////////////////////////
//
//
TouchInfo CheckArenaCollision(const Entity& e)
{
    static vec3 simple_box(
        (real_t)(s_arena.width - s_arena.top_radius)
        , (real_t)(s_arena.height)
        , real_t(s_arena.depth - s_arena.corner_radius)
    );

    TouchInfo ret;

    if (abs(e.pos.z) <= simple_box.z)
    {
        if (abs(e.pos.x) <= simple_box.x)
        {
            ret.normal.y = e.pos.y - simple_box.y;
            ret.depth = abs(ret.normal.y) + e.radius - simple_box.y;
            ret.normal.y /= abs(ret.normal.y);
            if (ret.depth > 0)
            {
                return ret;
            }
            ret.depth = 0.0_r;
            return ret;
        }

        real_t size_y = s_arena.height - (s_arena.top_radius + s_arena.bottom_radius) / 2.0_r;
        real_t pos_y = e.pos.y - (size_y + s_arena.bottom_radius);
        if (abs(pos_y) <= size_y)
        {
            ret.depth = abs(e.pos.x) + e.radius - s_arena.width;
            if (ret.depth > 0)
            {
                ret.normal.x = sign(e.pos.x);
                return ret;
            }

            ret.depth = 0.0_r;
            return ret;
        }

        if (pos_y > 0)
        {
            ret.normal.x = (abs(e.pos.x) - (s_arena.width - s_arena.top_radius)) * sign(e.pos.x);
            ret.normal.y = e.pos.y - (s_arena.height * 2.0_r - s_arena.top_radius);
            ret.depth = ret.normal.len() + e.radius - s_arena.top_radius;
            if (ret.depth > 0)
            {
                ret.normal.normalize();
                return ret;
            }

            ret.depth = 0.0_r;
            return ret;
        }

        if (abs(e.pos.x) < (s_arena.width - s_arena.bottom_radius))
        {
            ret.depth = e.radius - e.pos.y;
            if (ret.depth > 0)
            {
                ret.normal.y = -1.0_r;
                return ret;
            }

            ret.depth = 0.0_r;
            return ret;
        }

        ret.normal.x = (abs(e.pos.x) - (s_arena.width - s_arena.bottom_radius)) * sign(e.pos.x);
        ret.normal.y = e.pos.y - s_arena.bottom_radius;
        ret.depth = ret.normal.len() + e.radius - s_arena.bottom_radius;
        if (ret.depth > 0)
        {
            ret.normal.normalize();
            return ret;
        }

        ret.depth = 0.0_r;
        return ret;
    }

    if (abs(e.pos.x) >= (s_arena.width - s_arena.corner_radius))
    {
        real_t size_y = s_arena.height - (s_arena.top_radius + s_arena.bottom_radius) / 2.0_r;
        real_t pos_y = e.pos.y - (size_y + s_arena.bottom_radius);
        ret.normal.x = (abs(e.pos.x) - (s_arena.width - s_arena.corner_radius)) * sign(e.pos.x);
        ret.normal.z = (abs(e.pos.z) - simple_box.z) * sign(e.pos.z);
        ret.normal.y = ret.normal.len();
        if (abs(pos_y) <= size_y)
        {
            ret.depth = ret.normal.y + e.radius - s_arena.corner_radius;
            if (ret.depth > 0)
            {
                ret.normal.x /= ret.normal.y;
                ret.normal.z /= ret.normal.y;
                ret.normal.y = 0.0_r;
                return ret;
            }

            ret.depth = 0.0_r;
            return ret;
        }

        if (pos_y > 0)
        {
            if (ret.normal.y < (s_arena.corner_radius - s_arena.top_radius))
            {
                ret.depth = e.pos.y + e.radius - s_arena.height * 2.0_r;
                if (ret.depth > 0)
                {
                    ret.normal = vec3(0.0_r, 1.0_r, 0.0_r);
                    return ret;
                }

                ret.depth = 0.0_r;
                return ret;
            }

            size_y = ret.normal.y;
            ret.normal.y = 0.0_r;
            ret.normal = ret.normal.normal() * (size_y - (s_arena.corner_radius - s_arena.top_radius));
            ret.normal.y = e.pos.y - (s_arena.height * 2.0_r - s_arena.top_radius);
            ret.depth = ret.normal.len() + e.radius - s_arena.top_radius;
            if (ret.depth > 0)
            {
                ret.normal.normalize();
                return ret;
            }
            
            ret.depth = 0.0_r;
            return ret;
        }

        if (ret.normal.y < (s_arena.corner_radius - s_arena.bottom_radius))
        {
            ret.depth = e.radius - e.pos.y;
            if (ret.depth > 0)
            {
                ret.normal = vec3(0.0_r, -1.0_r, 0.0_r);
                return ret;
            }

            ret.depth = 0.0_r;
            return ret;
        }

        size_y = ret.normal.y;
        ret.normal.y = 0.0_r;
        ret.normal = ret.normal.normal() * (size_y - (s_arena.corner_radius - s_arena.bottom_radius));
        ret.normal.y = e.pos.y - s_arena.bottom_radius;
        ret.depth = ret.normal.len() + e.radius - s_arena.bottom_radius;
        if (ret.depth > 0)
        {
            ret.normal.normalize();
            return ret;
        }

        ret.depth = 0.0_r;
        return ret;
    }

    if (e.pos.y >= (s_arena.goal_height + s_arena.goal_side_radius))
    {
        if (e.pos.y <= (s_arena.height * 2.0_r - s_arena.top_radius))
        {
            ret.depth = abs(e.pos.z) + e.radius - s_arena.depth;
            if (ret.depth > 0)
            {
                ret.normal = vec3(0.0_r, 0.0_r, sign(e.pos.z));
                return ret;
            }

            ret.depth = 0.0_r;
            return ret;
        }

        if (abs(e.pos.z) <= (s_arena.width - s_arena.top_radius))
        {
            ret.depth = e.pos.y + e.radius - s_arena.height * 2.0_r;
            if (ret.depth > 0)
            {
                ret.normal = vec3(0.0_r, 1.0_r, 0.0_r);
                return ret;
            }

            ret.depth = 0.0_r;
            return ret;
        }

        ret.normal.z = (abs(e.pos.z) - (s_arena.depth - s_arena.top_radius)) * sign(e.pos.z);
        ret.normal.y = e.pos.y - (s_arena.height * 2.0_r - s_arena.top_radius);
        ret.depth = ret.normal.len() + e.radius - s_arena.top_radius;
        if (ret.depth > 0)
        {
            ret.normal.normalize();
            return ret;
        }

        ret.depth = 0.0_r;
        return ret;
    }

    if (abs(e.pos.z) < (s_arena.depth - s_arena.bottom_radius))
    {
        ret.depth = e.radius - e.pos.y;
        if (ret.depth > 0)
        {
            ret.normal = vec3(0.0_r, -1.0_r, 0.0_r);
            return ret;
        }

        ret.depth = 0.0_r;
        return ret;
    }

    if (abs(e.pos.x) <= (s_arena.goal_width - s_arena.goal_top_radius) / 2.0_r)
    {
        ret.depth = e.radius - e.pos.y;
        if (ret.depth > 0)
        {
            ret.normal = vec3(0.0_r, -1.0_r, 0.0_r);
            return ret;
        }

        ret.normal.y = (s_arena.goal_height + s_arena.goal_side_radius) - e.pos.y;
        ret.normal.z = ((s_arena.depth + s_arena.goal_side_radius) - abs(e.pos.z)) * sign(e.pos.z);
        ret.depth = (e.radius + s_arena.goal_side_radius) - ret.normal.len();
        if (ret.depth > 0)
        {
            ret.normal.normalize();
            return ret;
        }

        ret.depth = 0.0_r;
        return ret;
    }

    if (e.pos.x <= (s_arena.depth + s_arena.goal_side_radius))
    {
        ret.normal.x = ((s_arena.goal_width + s_arena.goal_side_radius) / 2.0_r - abs(e.pos.x)) * sign(e.pos.x);
        ret.normal.z = ((s_arena.depth + s_arena.goal_side_radius) - abs(e.pos.z)) * sign(e.pos.z);
        ret.depth = ret.normal.len();
        if (e.pos.y < s_arena.bottom_radius)
        {
            if (ret.depth > s_arena.bottom_radius)
            {
                ret.depth = e.radius - e.pos.y;
                if (ret.depth > 0)
                {
                    ret.normal = vec3(0.0_r, 1.0_r, 0.0_r);
                    return ret;
                }

                ret.depth = 0.0_r;
                return ret;
            }

            ret.normal.y = s_arena.bottom_radius;
            ret.normal = e.pos - ret.normal;
            ret.depth = ret.normal.len() + e.radius - s_arena.bottom_radius;
            if (ret.depth > 0)
            {
                ret.normal.normalize();
                return ret;
            }

            ret.depth = 0.0_r;
            return ret;
        }

        if (e.pos.y <= (s_arena.goal_height - s_arena.goal_top_radius))
        {
            ret.depth = e.radius + s_arena.goal_side_radius - ret.depth;
            if (ret.depth > 0)
            {
                ret.normal.normalize();
                return ret;
            }

            ret.depth = 0.0_r;
            return ret;
        }

        ret.normal.x = 0.0_r;
        ret.normal.y = e.pos.y - (s_arena.goal_height - s_arena.goal_top_radius);
        ret.normal.z = e.pos.z - (s_arena.depth + s_arena.goal_side_radius);
        ret.normal = ret.normal.normal() * (s_arena.goal_top_radius + s_arena.goal_side_radius);
        ret.normal.x = s_arena.depth + s_arena.goal_side_radius;
        ret.normal.y += s_arena.goal_height - s_arena.goal_top_radius;
        ret.normal.z = s_arena.depth + s_arena.goal_side_radius;
        ret.normal = ret.normal - e.pos;
        ret.depth = e.radius + s_arena.goal_side_radius - ret.normal.len();
        if (ret.depth > 0)
        {
            ret.normal.normalize();
            return ret;
        }

        ret.depth = 0.0_r;
        return ret;
    }

    return ret;
}

//////////////////////////////////////////////////////////////////////////
//
//
void BallTick()
{
    auto ball = s_world.ball;
    move(ball, s_timestep);

    auto touch = CheckArenaCollision(ball);
    if (touch.depth > 0)
    {
        if (touch.depth > 10.0_r)
        {
            touch.depth = 0.0_r;
        }
        ball = s_world.ball;
        bool fixed = false;
        for (int i = 0; i < s_rules.MICROTICKS_PER_TICK; ++i)
        {
            move(ball, s_microstep);

            touch = CheckArenaCollision(ball);
            if (touch.depth > 0)
            {
                ball.pos -= touch.normal * touch.depth;
                real_t v = ball.vel.dot(touch.normal);
                if (v > 0)
                {
                    if (fixed && v > 1.0)
                    {
                        v = v;
                    }
                    ball.vel -= touch.normal * (1 + ball.arena_e) * v;
                    fixed = true;
                }
            }
        }
    }

    s_world.ball = ball;
}

//////////////////////////////////////////////////////////////////////////
//
//
MyStrategy::MyStrategy()
{
}

void MyStrategy::init(const model::Rules& rules, const Game& game)
{
    s_rules = rules;
    s_rules.arena.width /= 2.0;
    s_rules.arena.height /= 2.0;
    s_rules.arena.depth /= 2.0;
    s_home_pos = vec3(0.0_r, 0.0_r, (-(real_t)s_rules.arena.depth) + (-(real_t)rules.arena.goal_width / 2.0_r));
    s_goal_pos = vec3(0.0_r, 0.0_r, ((real_t)s_rules.arena.depth) + ((real_t)rules.arena.goal_depth));
    Entity::s_entity_e = (real_t)(rules.MAX_HIT_E - rules.MAX_HIT_E) / 2.0_r;

    s_world.ball.radius = (real_t)rules.BALL_RADIUS;
    s_world.ball.mass = (real_t)rules.BALL_MASS;
    s_world.ball.arena_e = (real_t)rules.BALL_ARENA_E;
    s_timestep = 1.0_r / (real_t)rules.TICKS_PER_SECOND;
    s_microstep = s_timestep / (real_t)rules.MICROTICKS_PER_TICK;

    double dist = s_rules.arena.depth;
    int keeper = -1;
    for (auto& bot : game.robots)
    {
        Entity new_bot;
        new_bot.arena_e = (real_t)rules.ROBOT_ARENA_E;
        new_bot.mass = (real_t)rules.ROBOT_MASS;
        s_world.bots.push_back(new_bot);
        if (!bot.is_teammate)
        {
            continue;
        }
        m_bots.emplace(bot.id, MyBot());
        m_bots[bot.id].index = s_world.bots.size() - 1;
        vec3 bot_pos((real_t)bot.x, (real_t)bot.y, (real_t)bot.z);
        double home_dist = bot_pos.dist(s_home_pos);
        if (home_dist < dist)
        {
            dist = home_dist;
            keeper = bot.id;
        }
    }
    m_bots[keeper].role = MyBot::Keeper;
}

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    static const real_t home_r = (real_t)rules.arena.goal_width / 1.2_r;

    if (0 == game.current_tick)
    {
        init(rules, game);
    }

    if (game.current_tick != s_tick)
    {
        m_ready = 0;
        for (auto& bot : m_bots)
        {
            bot.second.ready = false;
        }

        for (size_t i = 0; i < game.robots.size(); ++i)
        {
            s_world.bots[i].pos = vec3((real_t)game.robots[i].x, (real_t)game.robots[i].y, (real_t)game.robots[i].z);
            s_world.bots[i].vel = vec3((real_t)game.robots[i].velocity_x, (real_t)game.robots[i].velocity_y, (real_t)game.robots[i].velocity_z);
        }

        s_world.ball.pos = vec3((real_t)game.ball.x, (real_t)game.ball.y, (real_t)game.ball.z);
        s_world.ball.vel = vec3((real_t)game.ball.velocity_x, (real_t)game.ball.velocity_y, (real_t)game.ball.velocity_z);

        //BallTick();
        for (real_t tick = 0; tick < 1000.0_r; ++tick)
        {
            BallTick();

            real_t radius = tick * s_rules.ROBOT_MAX_GROUND_SPEED / s_rules.TICKS_PER_SECOND;

            for (auto& item : m_bots)
            {
                auto& bot = item.second;

                if (bot.ready)
                {
                    continue;
                }

                if (vec3::dist(s_world.bots[bot.index].pos, s_world.ball.pos) <= s_rules.ROBOT_RADIUS + s_rules.BALL_RADIUS)
                {
                    bot.ready = true;
                    ++m_ready;
                }
            }

            if (m_bots.size() == m_ready)
            {
                break;
            }
        }

        s_tick = game.current_tick;
    }

    MyBot& me_bot = m_bots[me.id];
    vec3 me_pos(me.x, me.y, me.z);

    action.target_velocity_x = me_bot.target_speed.x;
    action.target_velocity_y = me_bot.target_speed.y;
    action.target_velocity_z = me_bot.target_speed.z;

    action.jump_speed = me_bot.jump_speed;

    action.use_nitro = me_bot.use_nitro;
}

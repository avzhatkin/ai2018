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
static real_t s_max_jump_height;
static real_t s_jump_time;
static real_t s_acceleration_time;
static real_t s_acceleration_distance;
int MyStrategy::s_tick = -1;
real_t s_timestep;
real_t s_microstep;

inline real_t sign(real_t v)
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
    map<int, Entity> bots;
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

        if (abs(e.pos.z) <= (s_arena.depth - s_arena.top_radius))
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

    if (abs(e.pos.x) >= (s_arena.goal_width / 2.0_r + s_arena.goal_side_radius))
    {
        if (e.pos.y > s_arena.bottom_radius)
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

        ret.normal.y = e.pos.y - s_arena.bottom_radius;
        ret.normal.z = (abs(e.pos.z) - (s_arena.depth - s_arena.bottom_radius)) * sign(e.pos.z);
        ret.depth = e.radius + ret.normal.len() - s_arena.bottom_radius;
        if (ret.depth > 0)
        {
            ret.normal.normalize();
            return ret;
        }

        ret.depth = 0.0_r;
        return ret;
    }

    if (abs(e.pos.z) <= (s_arena.depth + s_arena.goal_side_radius))
    {
        if (abs(e.pos.x) <= (s_arena.goal_width / 2.0_r - s_arena.goal_top_radius))
        {
            ret.depth = e.radius - e.pos.y;
            if (ret.depth > 0)
            {
                ret.normal = vec3(0.0_r, -1.0_r, 0.0_r);
                return ret;
            }

            ret.normal.y = (s_arena.goal_height + s_arena.goal_side_radius) - e.pos.y;
            ret.normal.z = ((s_arena.depth + s_arena.goal_side_radius)- abs(e.pos.z)) * sign(e.pos.z);
            ret.depth = (e.radius + s_arena.goal_side_radius) - ret.normal.len();
            if (ret.depth > 0)
            {
                ret.normal.normalize();
                return ret;
            }

            ret.depth = 0.0_r;
            return ret;
        }

        ret.normal.x = ((s_arena.goal_width / 2.0_r + s_arena.goal_side_radius) - abs(e.pos.x)) * sign(e.pos.x);
        ret.normal.z = ((s_arena.depth + s_arena.goal_side_radius) - abs(e.pos.z)) * sign(e.pos.z);
        ret.depth = ret.normal.len();
        if (e.pos.y < s_arena.bottom_radius)
        {
            if (ret.depth > (s_arena.bottom_radius + s_arena.goal_side_radius))
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

            ret.normal = -ret.normal.normal() * (s_arena.bottom_radius + s_arena.goal_side_radius);
            ret.normal.x = (abs(e.pos.x) - (s_arena.goal_width / 2.0_r + s_arena.goal_side_radius + ret.normal.x * sign(e.pos.x))) * sign(e.pos.x);
            ret.normal.z = (abs(e.pos.z) - (s_arena.depth + s_arena.goal_side_radius + ret.normal.z * sign(e.pos.z))) * sign(e.pos.z);
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

        ret.normal.x = (abs(e.pos.x) - (s_arena.goal_width / 2.0_r - s_arena.goal_top_radius)) * sign(e.pos.x);
        ret.normal.y = e.pos.y - (s_arena.goal_height - s_arena.goal_top_radius);
        ret.normal.z = 0.0_r;
        ret.normal = ret.normal.normal() * (s_arena.goal_top_radius + s_arena.goal_side_radius);
         
        ret.normal.x = ((s_arena.goal_width / 2.0_r + ret.normal.x * sign(e.pos.x)) - abs(e.pos.x)) * sign(e.pos.x);
        ret.normal.y = (s_arena.goal_height - s_arena.goal_top_radius + ret.normal.y) - e.pos.y;
        ret.normal.z = ((s_arena.depth + s_arena.goal_side_radius) - abs(e.pos.z)) * sign(e.pos.z);
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
    if (abs(s_world.ball.pos.z) >= (s_arena.depth + s_rules.BALL_RADIUS))
    {
        return;
    }

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
                    ball.vel -= touch.normal * (1 + ball.arena_e) * v;
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
    s_goal_pos = vec3(0.0_r, 0.0_r, (((real_t)s_rules.arena.depth) + ((real_t)rules.arena.goal_depth)));
    Entity::s_entity_e = (real_t)(rules.MAX_HIT_E - rules.MAX_HIT_E) / 2.0_r;

    s_world.ball.radius = (real_t)rules.BALL_RADIUS;
    s_world.ball.mass = (real_t)rules.BALL_MASS;
    s_world.ball.arena_e = (real_t)rules.BALL_ARENA_E;
    s_timestep = 1.0_r / (real_t)rules.TICKS_PER_SECOND;
    s_microstep = s_timestep / (real_t)rules.MICROTICKS_PER_TICK;

    s_jump_time = s_rules.ROBOT_MAX_JUMP_SPEED / s_rules.GRAVITY;
    s_max_jump_height = s_rules.ROBOT_MAX_JUMP_SPEED * s_rules.ROBOT_MAX_JUMP_SPEED / s_rules.GRAVITY / 2.0_r;
    s_acceleration_time = s_rules.ROBOT_MAX_GROUND_SPEED / s_rules.ROBOT_ACCELERATION;
    s_acceleration_distance = s_rules.ROBOT_MAX_GROUND_SPEED * s_rules.ROBOT_MAX_GROUND_SPEED / s_rules.ROBOT_ACCELERATION / 2.0_r;

    double dist = 0;
    int keeper = -1;
    for (auto& bot : game.robots)
    {
        Entity new_bot;
        new_bot.arena_e = (real_t)rules.ROBOT_ARENA_E;
        new_bot.mass = (real_t)rules.ROBOT_MASS;
        s_world.bots.emplace(bot.id, new_bot);
        if (!bot.is_teammate)
        {
            continue;
        }
        m_bots.emplace(bot.id, MyBot());
        vec3 bot_pos((real_t)bot.x, (real_t)bot.y, (real_t)bot.z);
        double center_dist = bot_pos.len();
        if (center_dist > dist)
        {
            dist = center_dist;
            keeper = bot.id;
        }
    }
    m_bots[keeper].role = MyBot::Keeper;

#ifdef MY_DEBUG
    unsigned int currentControl;
    _controlfp_s(&currentControl, ~(_EM_INVALID | _EM_ZERODIVIDE), _MCW_EM);
#endif
}

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    static const real_t home_r = (real_t)rules.arena.goal_width / 1.4_r;

    if (game.current_tick != s_tick)
    {
        if (0 == game.current_tick)
        {
            init(rules, game);
        }

        for (size_t i = 0; i < game.robots.size(); ++i)
        {
            s_world.bots[game.robots[i].id].pos = vec3((real_t)game.robots[i].x, (real_t)game.robots[i].y, (real_t)game.robots[i].z);
            s_world.bots[game.robots[i].id].vel = vec3((real_t)game.robots[i].velocity_x, (real_t)game.robots[i].velocity_y, (real_t)game.robots[i].velocity_z);
            s_world.bots[game.robots[i].id].touch = game.robots[i].touch;
        }

        s_world.ball.pos = vec3((real_t)game.ball.x, (real_t)game.ball.y, (real_t)game.ball.z);
        s_world.ball.vel = vec3((real_t)game.ball.velocity_x, (real_t)game.ball.velocity_y, (real_t)game.ball.velocity_z);

        Entity ball_current = s_world.ball;

        for (auto& item : m_bots)
        {
            auto& bot = item.second;
            auto& bot_body = s_world.bots[item.first];

            s_world.ball = ball_current;

            if (MyBot::Forward == bot.role)
            {
                bot.jump_speed = 0.0_r;

                if (!bot_body.touch)
                {
                    BallTick();
                    vec3 next_pos = bot_body.pos + bot_body.vel / s_rules.TICKS_PER_SECOND;

                    vec3 ball_dir = (s_world.ball.pos - next_pos);

                    if (next_pos.dist(s_world.ball.pos) < (s_rules.BALL_RADIUS + s_rules.ROBOT_RADIUS) && ball_dir.z > 0)
                    {
                        bot.jump_speed = s_rules.ROBOT_MAX_JUMP_SPEED;
                    }

                    continue;
                }

                vec3 ball_dir = (ball_current.pos - s_goal_pos);
                ball_dir.y = 0.0_r;
                ball_dir.normalize();
                ball_dir *= rules.BALL_RADIUS;
                bot.target = ball_current.pos + ball_dir;

                vec3 target_dir = bot.target - bot_body.pos;
                target_dir.y = 0.0_r;
                bot.target_speed = target_dir.normal() * s_rules.ROBOT_MAX_GROUND_SPEED * 2.0_r;

                static const real_t tick_limit = 10.0_r;
                real_t tick = 1.0_r;
                for (; tick <= tick_limit; ++tick)
                {
                    BallTick();

                    addDebugSphere(DebugSphere({ s_world.ball.pos.x, s_world.ball.pos.y, s_world.ball.pos.z }, s_rules.BALL_RADIUS, { 1.0_r, 1.0_r, 1.0_r }, 0.3_r));

                    vec3 bot_pos = bot_body.pos + bot_body.vel * (tick + 1.0_r) / s_rules.TICKS_PER_SECOND;
                    ball_dir = s_world.ball.pos - bot_pos;
                    ball_dir.y = 0.0_r;

                    if (ball_dir.len() < (s_rules.BALL_RADIUS + s_rules.ROBOT_RADIUS) && s_world.ball.pos.y < (s_max_jump_height + s_rules.BALL_RADIUS) && ball_dir.z > 0)
                    {
                        break;
                    }
                }

                if (tick > tick_limit)
                {
                    continue;
                }

                real_t ball_pos_y = s_world.ball.pos.y + s_rules.ROBOT_RADIUS - s_rules.BALL_RADIUS;
                if (ball_pos_y < 0.0_r)
                {
                    continue;
                }

                real_t air_time = sqrt(2.0_r * ball_pos_y / s_rules.GRAVITY);

                if (tick <= air_time * s_rules.TICKS_PER_SECOND)
                {
                    bot.jump_speed = s_rules.ROBOT_MAX_JUMP_SPEED;
                }
            }

            else if (MyBot::Keeper == bot.role)
            {
                vec3 guard_pos = vec3(0.0_r, 0.0_r, -s_arena.depth - s_arena.goal_side_radius);
                real_t guard_r = s_arena.goal_width * s_arena.goal_width / (8.0_r * (s_arena.goal_width / 2.0_r)) + (s_arena.goal_width / 2.0_r) / 2.0_r;
                //addDebugSphere(DebugSphere({ 0.0_r, 0.0_r, -s_arena.depth - s_arena.goal_side_radius }, guard_r, { 0.0_r, 0.0_r, 1.0_r }, 0.2_r));

                vec3 ball_dir = (ball_current.pos - s_home_pos);
                ball_dir.y = 0.0_r;
                ball_dir.normalize();
                bot.target = s_home_pos + ball_dir * home_r;
                if (abs(bot.target.x) >= (s_arena.goal_width / 2.0_r - s_arena.bottom_radius))
                {
                    bot.target.x = (s_arena.goal_width / 2.0_r - s_arena.bottom_radius) * sign(bot.target.x);
                }
                vec3 target_dir = bot.target - bot_body.pos;
                target_dir.y = 0.0_r;
                double target_speed_d = 10.0_r * min(s_rules.MAX_ENTITY_SPEED, target_dir.len());
                bot.target_speed = target_dir.normal() * target_speed_d;
                bot.target_speed.y = 0.0_r;
                bot.jump_speed = 0.0_r;

                if (ball_current.pos.z > 0 || 
                    (ball_current.pos.z > (-s_arena.depth / 2.0_r) && ball_current.vel.z > 0))
                {
                    continue;
                }

                if (!bot_body.touch)
                {
                    BallTick();
                    vec3 next_pos = bot_body.pos + bot_body.vel / s_rules.TICKS_PER_SECOND;

                    if (next_pos.dist(s_world.ball.pos) < (s_rules.BALL_RADIUS + s_rules.ROBOT_RADIUS))
                    {
                        bot.jump_speed = s_rules.ROBOT_MAX_JUMP_SPEED;
                    }

                    continue;
                }

                static const real_t tick_limit = 50.0_r;
                real_t tick = 1.0_r;
                for (; tick <= tick_limit; ++tick)
                {
                    BallTick();

                    addDebugSphere(DebugSphere({ s_world.ball.pos.x, s_world.ball.pos.y, s_world.ball.pos.z }, s_rules.BALL_RADIUS, { 1.0_r, 1.0_r, 1.0_r }, 0.3_r));

                    if (s_world.ball.pos.dist(guard_pos) <= (guard_r + s_rules.BALL_RADIUS) && s_world.ball.pos.y < (s_max_jump_height + s_rules.BALL_RADIUS + s_rules.ROBOT_RADIUS * 1.8_r))
                    {
                        addDebugSphere(DebugSphere({ s_world.ball.pos.x, s_world.ball.pos.y, s_world.ball.pos.z }, s_rules.BALL_RADIUS, { 1.0_r, 1.0_r, 1.0_r }, 1.0_r));
                        break;
                    }
                }

                if (tick > tick_limit)
                {
                    continue;
                }

                vec3 ball_vel = s_world.ball.vel;

                bool force_move = false;
                vec3 target_pos;
                // Если можем допрыгнуть, пробуем увести в сторону горизонтально
                if (s_world.ball.pos.y <= (s_max_jump_height + s_rules.ROBOT_RADIUS))
                {
                    ball_vel.y = 0.0_r;

                    vec3 safe_pos(s_arena.goal_width / 2.0_r + s_rules.BALL_RADIUS * 2.0_r, 0.0_r, -s_arena.depth + s_rules.BALL_RADIUS);
                    vec3 safe_dir_left = (safe_pos - s_world.ball.pos);
                    safe_dir_left.y = 0.0_r;
                    safe_dir_left.normalize();
                    safe_pos.x = -safe_pos.x;
                    vec3 safe_dir_right = (safe_pos - s_world.ball.pos);
                    safe_dir_right.y = 0.0_r;
                    safe_dir_right.normalize();

                    real_t v_left = ball_vel.dot(safe_dir_left);
                    real_t v_right = ball_vel.dot(safe_dir_right);

                    vec3 safe_dir = v_left > v_right ? safe_dir_left : safe_dir_right;

                    vec3 speed_delta = ball_vel - ball_vel.project(safe_dir);

                    real_t k_ball = s_rules.ROBOT_MASS / (s_rules.BALL_MASS + s_rules.ROBOT_MASS);

                    vec3 impulse = speed_delta / k_ball;

                    if (impulse.z > 0)
                    {
                        impulse.z = -impulse.z;
                    }

                    target_pos = s_world.ball.pos + impulse.normal() * (s_rules.BALL_RADIUS + s_rules.ROBOT_RADIUS);
                    bot.target = target_pos;
                    addDebugSphere(DebugSphere({ target_pos.x, target_pos.y, target_pos.z }, 1.0_r, { 0.0_r, 1.0_r, 0.0_r }, 0.5_r));

                    vec3 target_dir_2d = target_pos - bot_body.pos;
                    real_t target_dist_y = target_dir_2d.y;
                    target_dir_2d.y = 0.0_r;
                    real_t target_dist_2d = target_dir_2d.len();

                    real_t closing_vel = bot_body.vel.dot(target_dir_2d.normal());

                    real_t target_time = target_dist_2d / closing_vel;

                    real_t b = -2.0_r * s_rules.ROBOT_MAX_JUMP_SPEED / s_rules.GRAVITY;
                    real_t c = 2.0_r * target_dist_y / s_rules.GRAVITY;
                    real_t d = b * b - 4.0_r * c;

                    real_t air_time;
                    if (d > 0)
                    {
                        air_time = ((-b - sqrt(d)) / 2.0_r);
                    }
                    else
                    {
                        air_time = s_jump_time;
                    }

                    real_t acceleration_time = (s_rules.ROBOT_MAX_GROUND_SPEED - closing_vel) / s_rules.ROBOT_ACCELERATION;
                    real_t acceleration_dist = closing_vel * acceleration_time + s_rules.ROBOT_ACCELERATION * acceleration_time * acceleration_time / 2.0_r;

                    b = 2.0_r * closing_vel / s_rules.ROBOT_ACCELERATION;
                    c = -2.0_r * target_dist_2d / s_rules.ROBOT_ACCELERATION;
                    d = b * b - 4.0_r * c;
                    if (d > 0)
                    {
                        real_t move_time = ((-b - sqrt(d)) / 2.0_r) * s_rules.TICKS_PER_SECOND;

                        if (move_time * s_rules.TICKS_PER_SECOND > tick)
                        {
                            vec3 ball_dir = (ball_current.pos - s_home_pos);
                            ball_dir.y = 0.0_r;
                            ball_dir.normalize();
                            bot.target = s_home_pos + ball_dir * home_r;
                            if (abs(bot.target.x) >= (s_arena.goal_width / 2.0_r - s_arena.bottom_radius))
                            {
                                bot.target.x = (s_arena.goal_width / 2.0_r - s_arena.bottom_radius) * sign(bot.target.x);
                            }
                            vec3 target_dir = bot.target - bot_body.pos;
                            target_dir.y = 0.0_r;
                            double target_speed_d = 10.0_r * min(s_rules.MAX_ENTITY_SPEED, target_dir.len());
                            bot.target_speed = target_dir.normal() * target_speed_d;
                            continue;
                        }

                        if ((acceleration_time - air_time) * s_rules.TICKS_PER_SECOND >= tick)
                        {
                            bot.target_speed = s_rules.MAX_ENTITY_SPEED * 2.0_r * target_dir_2d.normal();
                        }
                        else
                        {
                            bot.target_speed = target_dist_2d / (tick / s_rules.TICKS_PER_SECOND) * target_dir_2d.normal();
                        }
                    }
                    else
                    {
                        vec3 ball_dir = (ball_current.pos - s_home_pos);
                        ball_dir.y = 0.0_r;
                        ball_dir.normalize();
                        bot.target = s_home_pos + ball_dir * home_r;
                        if (abs(bot.target.x) >= (s_arena.goal_width / 2.0_r - s_arena.bottom_radius))
                        {
                            bot.target.x = (s_arena.goal_width / 2.0_r - s_arena.bottom_radius) * sign(bot.target.x);
                        }
                        vec3 target_dir = bot.target - bot_body.pos;
                        target_dir.y = 0.0_r;
                        double target_speed_d = 10.0_r * min(s_rules.MAX_ENTITY_SPEED, target_dir.len());
                        bot.target_speed = target_dir.normal() * target_speed_d;
                        continue;
                    }

                    if (tick <= air_time * s_rules.TICKS_PER_SECOND && bot_body.touch)
                    {
                        bot.jump_speed = s_rules.ROBOT_MAX_JUMP_SPEED;
                    }
                }
                // Отбив получится скорее вверх
                else
                {
                    guard_pos = vec3(0.0_r, 0.0_r, -s_arena.depth - s_arena.goal_side_radius - s_rules.BALL_RADIUS);
                    vec3 ball_dir = guard_pos - s_world.ball.pos;
                    ball_dir.normalize();
                    target_pos = s_world.ball.pos + ball_dir * (s_rules.BALL_RADIUS + s_rules.ROBOT_RADIUS);

                    bot.target = target_pos;
                    addDebugSphere(DebugSphere({ target_pos.x, target_pos.y, target_pos.z }, 1.0_r, { 0.0_r, 0.0_r, 1.0_r }, 0.5_r));

                    vec3 target_dir_2d = target_pos - bot_body.pos;
                    real_t target_dist_y = target_dir_2d.y;
                    target_dir_2d.y = 0.0_r;
                    real_t target_dist_2d = target_dir_2d.len();

                    real_t b = -2.0_r * s_rules.ROBOT_MAX_JUMP_SPEED / s_rules.GRAVITY;
                    real_t c = 2.0_r * target_dist_y / s_rules.GRAVITY;
                    real_t d = b * b - 4.0_r * c;

                    real_t target_time;
                    if (d > 0)
                    {
                        target_time = ((-b - sqrt(d)) / 2.0_r);
                    }
                    else
                    {
                        target_time = s_jump_time;
                    }

                    real_t closing_vel = bot_body.vel.dot(target_dir_2d.normal());

                    if (closing_vel * target_time > target_dist_2d)
                    {
                        bot.target_speed = vec3();
                    }
                    else
                    {
                        real_t acceleration_time = (s_rules.ROBOT_MAX_GROUND_SPEED - closing_vel) / s_rules.ROBOT_ACCELERATION;
                        real_t acceleration_dist = closing_vel * acceleration_time + s_rules.ROBOT_ACCELERATION * acceleration_time * acceleration_time / 2.0_r;

                        real_t b = 2.0_r * closing_vel / s_rules.ROBOT_ACCELERATION;
                        real_t c = -2.0_r * target_dist_2d / s_rules.ROBOT_ACCELERATION;
                        real_t d = b * b - 4.0_r * c;
                        if (d > 0)
                        {
                            bot.target_speed = target_dist_2d / target_time * target_dir_2d.normal();

                            real_t move_time = ((-b - sqrt(d)) / 2.0_r) * s_rules.TICKS_PER_SECOND;

                            if (move_time > target_time)
                            {
                                vec3 ball_dir = (ball_current.pos - s_home_pos);
                                ball_dir.y = 0.0_r;
                                ball_dir.normalize();
                                bot.target = s_home_pos + ball_dir * home_r;
                                if (abs(bot.target.x) >= (s_arena.goal_width / 2.0_r - s_arena.bottom_radius))
                                {
                                    bot.target.x = (s_arena.goal_width / 2.0_r - s_arena.bottom_radius) * sign(bot.target.x);
                                }
                                vec3 target_dir = bot.target - bot_body.pos;
                                target_dir.y = 0.0_r;
                                double target_speed_d = 10.0_r * min(s_rules.MAX_ENTITY_SPEED, target_dir.len());
                                bot.target_speed = target_dir.normal() * target_speed_d;
                                continue;
                            }
                        }
                        else
                        {
                            vec3 ball_dir = (ball_current.pos - s_home_pos);
                            ball_dir.y = 0.0_r;
                            ball_dir.normalize();
                            bot.target = s_home_pos + ball_dir * home_r;
                            if (abs(bot.target.x) >= (s_arena.goal_width / 2.0_r - s_arena.bottom_radius))
                            {
                                bot.target.x = (s_arena.goal_width / 2.0_r - s_arena.bottom_radius) * sign(bot.target.x);
                            }
                            vec3 target_dir = bot.target - bot_body.pos;
                            target_dir.y = 0.0_r;
                            double target_speed_d = 10.0_r * min(s_rules.MAX_ENTITY_SPEED, target_dir.len());
                            bot.target_speed = target_dir.normal() * target_speed_d;
                            continue;
                        }
                    }

                    if (tick <= target_time * s_rules.TICKS_PER_SECOND && bot_body.touch)
                    {
                        bot.jump_speed = s_rules.ROBOT_MAX_JUMP_SPEED;
                    }
                }
            }
        }

        s_tick = game.current_tick;
    }

    MyBot& me_bot = m_bots[me.id];

    action.target_velocity_x = me_bot.target_speed.x;
    action.target_velocity_y = me_bot.target_speed.y;
    action.target_velocity_z = me_bot.target_speed.z;

    action.jump_speed = me_bot.jump_speed;

    action.use_nitro = me_bot.use_nitro;

    addDebugSphere(DebugSphere({ me_bot.target.x, me_bot.target.y, me_bot.target.z }, 1.0_r, { 1.0_r, 0.0_r, 0.0_r }, 0.5_r));
}

void MyStrategy::addDebugSphere(DebugSphere&& sphere)
{
#ifdef MY_DEBUG
    m_debugSpheres.push_back(sphere);
#else
    (sphere);
#endif
}

#ifdef MY_DEBUG
std::string MyStrategy::custom_rendering()
{
    static string str;
    static vector<char> buffer(512);

    str = "[";

    for (auto& sphere : m_debugSpheres)
    {
        sprintf_s(buffer.data(), buffer.size(), R"___(  {
    "Sphere": {
      "x": %lf,
      "y": %lf,
      "z": %lf,
      "radius": %lf,
      "r": %lf,
      "g": %lf,
      "b": %lf,
      "a": %lf
    }
  },
)___"
, (double)sphere.center.x
, (double)sphere.center.y
, (double)sphere.center.z
, (double)sphere.radius
, (double)sphere.color.x
, (double)sphere.color.y
, (double)sphere.color.z
, (double)sphere.alpha
);

        str += buffer.data();
    }

    str[str.size() - 2] = ']';

    m_debugSpheres.clear();

    return str;
}
#endif

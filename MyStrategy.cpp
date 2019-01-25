#include "MyStrategy.h"
#include <algorithm>
#include "linal.h"
#include <vector>
#include <chrono>
using namespace linal;
using namespace std;
using namespace model;

alignas(16) static Rules s_rules;
static Arena& s_arena = s_rules.arena;
static bool s_nitro_game = false;
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
    real_t nitro = 0.0_r;
    real_t radius = 0.0_r;
    real_t mass = 0.0_r;
    real_t arena_e = 0.0_r;
    vec3 normal;
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
static const size_t ballTicksCount = 300;
static vector<Entity> s_ballTicks(ballTicksCount);
static int s_current_tick = 0;
Entity& GetBallTick(int tick)
{
    return s_ballTicks[(s_current_tick + ballTicksCount + tick) % ballTicksCount];
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
         
        ret.normal.x = ((s_arena.goal_width / 2.0_r - s_arena.goal_top_radius + ret.normal.x * sign(e.pos.x)) - abs(e.pos.x)) * sign(e.pos.x);
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

    ret.normal.y = e.pos.y - s_arena.goal_height / 2.0_r;
    
    if (abs(e.pos.z) <= (s_arena.depth + s_arena.goal_depth - s_arena.goal_top_radius))
    {
        if (abs(e.pos.x) <= (s_arena.goal_width / 2.0_r - s_arena.goal_top_radius))
        {
            ret.depth = abs(ret.normal.y) + e.radius - s_arena.goal_height / 2.0_r;
            if (ret.depth > 0)
            {
                ret.normal = vec3(0.0_r, sign(ret.normal.y), 0.0_r);
                return ret;
            }

            ret.depth = 0.0_r;
            return ret;
        }

        if (abs(ret.normal.y) <= (s_arena.goal_height / 2.0_r - s_arena.goal_top_radius))
        {
            ret.depth = abs(e.pos.x) + e.radius - s_arena.goal_width / 2.0_r;
            if (ret.depth > 0)
            {
                ret.normal = vec3(sign(e.pos.x), 0.0_r, 0.0_r);
                return ret;
            }

            ret.depth = 0.0_r;
            return ret;
        }

        ret.normal.x = (abs(e.pos.x) - (s_arena.goal_width / 2.0_r - s_arena.goal_top_radius)) * sign(e.pos.x);
        ret.normal.y = (abs(ret.normal.y) - (s_arena.goal_height / 2.0_r - s_arena.goal_top_radius)) * sign(ret.normal.y);
        ret.normal.z = 0.0_r;
        ret.depth = ret.normal.len() + e.radius - s_arena.goal_top_radius;
        if (ret.depth > 0)
        {
            ret.normal.normalize();
            return ret;
        }

        ret.depth = 0.0_r;
        return ret;
    }

    if (abs(e.pos.x) <= (s_arena.goal_width / 2.0_r - s_arena.goal_top_radius))
    {
        if (abs(ret.normal.y) <= (s_arena.goal_height / 2.0_r - s_arena.goal_top_radius))
        {
            ret.depth = abs(e.pos.z) + e.radius - (s_arena.depth + s_arena.goal_depth);
            if (ret.depth > 0)
            {
                ret.normal = vec3(0.0_r, 0.0_r, sign(e.pos.z));
                return ret;
            }

            ret.depth = 0.0_r;
            return ret;
        }

        ret.normal.x = 0.0_r;
        ret.normal.y = (abs(ret.normal.y) - (s_arena.goal_height / 2.0_r - s_arena.goal_top_radius)) * sign(ret.normal.y);
        ret.normal.z = (abs(e.pos.z) - (s_arena.depth + s_arena.goal_depth - s_arena.goal_top_radius)) * sign(e.pos.z);
        ret.depth = ret.normal.len() + e.radius - s_arena.goal_top_radius;
        if (ret.depth > 0)
        {
            ret.normal.normalize();
            return ret;
        }
        
        ret.depth = 0.0_r;
        return ret;
    }

    if (abs(ret.normal.y) <= (s_arena.goal_height / 2.0_r - s_arena.goal_top_radius))
    {
        ret.normal.x = (abs(e.pos.x) - (s_arena.goal_width / 2.0_r - s_arena.goal_top_radius)) * sign(e.pos.x);
        ret.normal.y = 0.0_r;
        ret.normal.z = (abs(e.pos.z) - (s_arena.depth + s_arena.goal_depth - s_arena.goal_top_radius)) * sign(e.pos.z);
        ret.depth = ret.normal.len() + e.radius - s_arena.goal_top_radius;
        if (ret.depth > 0)
        {
            ret.normal.normalize();
            return ret;
        }

        ret.depth = 0.0_r;
        return ret;
    }

    ret.normal.x = (abs(e.pos.x) - (s_arena.goal_width / 2.0_r - s_arena.goal_top_radius)) * sign(e.pos.x);
    ret.normal.y = (abs(ret.normal.y) - (s_arena.goal_height / 2.0_r - s_arena.goal_top_radius)) * sign(ret.normal.y);
    ret.normal.z = (abs(e.pos.z) - (s_arena.depth + s_arena.goal_depth - s_arena.goal_top_radius)) * sign(e.pos.z);
    ret.depth = ret.normal.len() + e.radius - s_arena.goal_top_radius;
    if (ret.depth > 0)
    {
        ret.normal.normalize();
        return ret;
    }

    ret.depth = 0.0_r;
    return ret;
}

//////////////////////////////////////////////////////////////////////////
//
//
Entity NextTick(const Entity& e)
{
    auto ret = e;
    move(ret, s_timestep);

    auto touch = CheckArenaCollision(ret);
    if (touch.depth > 0)
    {
        ret = e;
        for (int utick = 0; utick < s_rules.MICROTICKS_PER_TICK; ++utick)
        {
            move(ret, s_microstep);

            touch = CheckArenaCollision(ret);
            if (touch.depth > 0)
            {
                ret.pos -= touch.normal * touch.depth;
                real_t v = ret.vel.dot(touch.normal);
                if (v > 0)
                {
                    ret.vel -= touch.normal * (1 + ret.arena_e) * v;
                }
            }
        }
    }

    return std::move(ret);
}

Entity BallTick(const Entity& e)
{
    if (abs(e.pos.z) >= (s_arena.depth + s_rules.BALL_RADIUS))
    {
        return std::move(Entity(e));
    }

    return NextTick(e);
}

//////////////////////////////////////////////////////////////////////////
//
//
void ComputeJump(MyStrategy::MyBot& bot, MyStrategy::NextStep& step, int tick, int ticks)
{
    if (tick == ticks)
    {
        return;
    }

    step.jump_speed = s_rules.ROBOT_MAX_JUMP_SPEED;
    bot.actions.push_back(step);
    step.jump_speed = 0.0_r;
    step.pos.y += s_rules.ROBOT_MAX_RADIUS - s_rules.ROBOT_MIN_RADIUS - (s_rules.GRAVITY * s_microstep / 2.0_r);
    step.vel.y = s_rules.ROBOT_MAX_JUMP_SPEED - s_rules.GRAVITY * s_microstep * 7.0_r / 3.0_r;

    for (; tick < ticks; ++tick)
    {
        step.vel.clamp(s_rules.MAX_ENTITY_SPEED);
        step.pos += step.vel * s_timestep;
        step.pos.y -= s_rules.GRAVITY * s_timestep * s_timestep / 2.0_r;
        step.vel.y -= s_rules.GRAVITY * s_timestep;
        bot.actions.push_back(step);
    }
}

void StepMove(MyStrategy::NextStep& step)
{
    vec3 dv = (step.target_speed - step.vel).clamp(s_rules.ROBOT_ACCELERATION * s_timestep);
    if (dv.len() > s_rules.ROBOT_ACCELERATION * s_microstep)
    {
        for (int i = 0; i < s_rules.MICROTICKS_PER_TICK; ++i)
        {
            dv = (step.target_speed - step.vel).clamp(s_rules.ROBOT_ACCELERATION * s_microstep);
            step.vel += dv;
            step.vel.clamp(s_rules.ROBOT_MAX_GROUND_SPEED);
            step.pos += step.vel * s_microstep;
        }
    }
    else
    {
        step.vel += dv;
        step.vel.clamp(s_rules.ROBOT_MAX_GROUND_SPEED);
        step.pos += step.vel * s_timestep;
    }
}

int ComputeLineMotion(MyStrategy::MyBot& bot, MyStrategy::NextStep& step, int catchTick, int ticks)
{
    auto target_2d = bot.target;
    target_2d.y = step.pos.y;
    real_t target_pos_y = bot.target.y - s_rules.ROBOT_RADIUS;
    real_t b = -2.0_r * s_rules.ROBOT_MAX_JUMP_SPEED / s_rules.GRAVITY;
    real_t c = 2.0_r * target_pos_y / s_rules.GRAVITY;
    real_t d = b * b - 4.0_r * c;

    real_t air_time;
    if (d > 0)
    {
        air_time = max(((-b - sqrt(d)) / 2.0_r), 0.0_r);
    }
    else
    {
        air_time = s_jump_time;
    }
    int air_ticks = (int)floor(air_time / s_timestep);
    // Jump time is higher than time left
    if (air_ticks > ticks)
    {
        return air_ticks - ticks;
    }

    if (0 == ticks)
    {
        return 0;
    }

    int ground_ticks = ticks - air_ticks;
    real_t target_time = ticks * s_timestep;
    real_t ground_time = ground_ticks * s_timestep;
    real_t target_dist_2d = (target_2d - step.pos).len();

    if (target_dist_2d / target_time > s_rules.ROBOT_MAX_GROUND_SPEED)
    {
        return int(ceil((target_dist_2d / s_rules.ROBOT_MAX_GROUND_SPEED - target_time) / s_timestep));
    }

    real_t move_speed = step.vel.len();
    real_t accel_time = min(ceil((s_rules.ROBOT_MAX_GROUND_SPEED - move_speed) / s_rules.ROBOT_ACCELERATION / s_timestep) * s_timestep, ground_time);
    real_t air_speed = move_speed + s_rules.ROBOT_ACCELERATION * accel_time;
    real_t accel_dist = move_speed * accel_time + s_rules.ROBOT_ACCELERATION * accel_time * accel_time / 2.0_r;

    if (accel_time == ground_time && (accel_dist + air_speed * air_time) < target_dist_2d)
    {
        return 0;
    }

    if ((accel_dist + s_rules.ROBOT_MAX_GROUND_SPEED * (target_time - accel_time)) < target_dist_2d)
    {
        return 0;
    }

    vec3 goal_dir = (s_goal_pos - GetBallTick(catchTick).pos).normal();
    if (step.vel.dot(goal_dir) >= -0.8)
    {
        int tick = 0;
        if ((accel_dist + air_speed * air_time) <= target_dist_2d)
        {
            real_t move_time_left = target_time - (air_time + accel_time);

            for (; move_time_left > 0; ++tick)
            {
                real_t desired_move_speed = (target_dist_2d - (accel_dist + air_speed * air_time)) / move_time_left;
                if (desired_move_speed < 0)
                {
                    return 5;
                }
                step.target_speed = (target_2d - step.pos).normal() * desired_move_speed;
                bot.actions.push_back(step);
                StepMove(step);
                target_time -= s_timestep;
                move_speed = step.vel.len();
                accel_time = min(ceil((s_rules.ROBOT_MAX_GROUND_SPEED - move_speed) / s_rules.ROBOT_ACCELERATION / s_timestep) * s_timestep, target_time - air_time);
                air_speed = move_speed + s_rules.ROBOT_ACCELERATION * accel_time;
                accel_dist = move_speed * accel_time + s_rules.ROBOT_ACCELERATION * accel_time * accel_time / 2.0_r;
                move_time_left = target_time - (air_time + accel_time);
                target_dist_2d = (target_2d - step.pos).len();
            }

            if (tick >= (ticks - air_ticks))
            {
                return tick - (ticks - air_ticks);
            }

            for (; tick < (ticks - air_ticks); ++tick)
            {
                step.target_speed = (target_2d - step.pos).normal() * s_rules.ROBOT_ACCELERATION;
                bot.actions.push_back(step);
                StepMove(step);
            }

            ComputeJump(bot, step, tick, ticks);

            return -1;
        }

        return ballTicksCount;
    }

    return ballTicksCount;
}

//////////////////////////////////////////////////////////////////////////
//
//
void MyStrategy::ComputeForward(MyBot& bot, int id)
{
    auto bot_body = s_world.bots[id];

    NextStep next;
    next.pos = bot_body.pos;
    next.vel = bot_body.vel;
    next.nitro = bot_body.nitro;

    if (!bot_body.touch || bot_body.normal.y != 1.0_r)
    {
        vec3 ball_dir = (GetBallTick(1).pos - s_goal_pos);
        ball_dir.y = 0.0_r;
        ball_dir.normalize();
        vec3 target = GetBallTick(1).pos + ball_dir.normal() * s_rules.BALL_RADIUS;

        vec3 target_dir = target - bot_body.pos;
        target_dir.y = 0.0_r;
        next.target_speed = target_dir.normal() * s_rules.ROBOT_MAX_GROUND_SPEED * 2.0_r;

        if (!bot_body.touch && 0 != bot_body.nitro)
        {
            next.target_speed.y = -s_rules.MAX_ENTITY_SPEED;
            next.use_nitro = true;
        }

        bot.actions.clear();
        bot.actions.push_back(next);

        return;
    }

    const int tick_limit = ballTicksCount - 1;
    int catchTick = 1;
    auto ball_target_state = GetBallTick(catchTick);
    for (; catchTick < tick_limit; ++catchTick)
    {
        ball_target_state = GetBallTick(catchTick);
        if (ball_target_state.pos.y > (s_max_jump_height + s_rules.ROBOT_RADIUS)
            || abs(ball_target_state.pos.x) >= (s_arena.width - s_arena.bottom_radius / 2.0_r))
        {
            continue;
        }

        bot.actions.clear();
        next.pos = bot_body.pos;
        next.vel = bot_body.vel;
        next.nitro = bot_body.nitro;

        vec3 ball_goal_dir = (ball_target_state.pos - s_goal_pos).normal();
        vec3 ball_speed_dir = ball_target_state.vel.clamp(1.0);
        ball_speed_dir += ball_goal_dir;
        if (ball_speed_dir.len() > 0 && ball_goal_dir.dot(ball_speed_dir) > 0)
        {
            ball_goal_dir = ball_speed_dir.normal();
        }
        ball_goal_dir.y = min(ball_goal_dir.y - 0.1_r, 0.0_r);
        ball_goal_dir.normalize();
        bot.target = (ball_target_state.pos + ball_goal_dir * (s_rules.BALL_RADIUS + s_rules.ROBOT_RADIUS));
        if (bot.target.y < s_rules.ROBOT_RADIUS)
        {
            real_t xz_target = sqrt((s_rules.BALL_RADIUS + s_rules.ROBOT_RADIUS) * (s_rules.BALL_RADIUS + s_rules.ROBOT_RADIUS) - s_rules.ROBOT_RADIUS * s_rules.ROBOT_RADIUS);
            vec2 xz = ball_target_state.pos.xz() + ball_goal_dir.xz().normal() * xz_target;
            bot.target = vec3(xz.x, s_rules.ROBOT_RADIUS, xz.y);
        }
        if (0 == s_current_tick)
        {
            bot.target = ball_target_state.pos;
            bot.target.z -= s_rules.BALL_RADIUS;
        }
        auto target = bot.target;
        target.y = next.pos.y;
        int botTick = 0;
        for (; botTick <= catchTick; ++botTick)
        {
            next.target_speed = (target - next.pos).normal() * s_rules.ROBOT_MAX_GROUND_SPEED;
            vec3 dv = (next.target_speed - next.vel).clamp(s_rules.ROBOT_ACCELERATION * s_timestep);

            if (dv.deviation(next.target_speed) < 0.0001_r)
            {
                // Straight line from now on
                int skip_ticks = ComputeLineMotion(bot, next, catchTick, catchTick - botTick);
                if (skip_ticks >= 0)
                {
                    catchTick += skip_ticks;
                    break;
                }

                return;
            }

            bot.actions.push_back(next);

            StepMove(next);
        }
    }

    next.pos = bot_body.pos;
    next.vel = bot_body.vel;
    next.nitro = bot_body.nitro;

    vec3 ball_dir = (GetBallTick(1).pos - s_goal_pos);
    ball_dir.y = 0.0_r;
    ball_dir.normalize();
    vec3 target = GetBallTick(1).pos + ball_dir.normal() * s_rules.BALL_RADIUS;

    vec3 target_dir = target - bot_body.pos;
    target_dir.y = 0.0_r;
    next.target_speed = target_dir.normal() * s_rules.ROBOT_MAX_GROUND_SPEED * 2.0_r;

    auto target_2d = target;
    target_2d.y = next.pos.y;
    real_t target_pos_y = target.y - s_rules.ROBOT_RADIUS;
    real_t b = -2.0_r * s_rules.ROBOT_MAX_JUMP_SPEED / s_rules.GRAVITY;
    real_t c = 2.0_r * target_pos_y / s_rules.GRAVITY;
    real_t d = b * b - 4.0_r * c;

    real_t air_time;
    if (d > 0)
    {
        air_time = max(((-b - sqrt(d)) / 2.0_r), 0.0_r);
    }
    else
    {
        air_time = 0;
    }
    int air_ticks = (int)floor(air_time / s_timestep);

    NextStep step = next;
    step.target_speed = step.vel;
    for (int tick = 1; tick < air_ticks; ++tick)
    {
        StepMove(step);
        vec3 ball_pos_2d = GetBallTick(tick).pos;
        ball_pos_2d.y = step.pos.y;
        if (step.pos.dist(ball_pos_2d) < (s_rules.BALL_RADIUS + s_rules.ROBOT_RADIUS))
        {
            next.jump_speed = s_rules.ROBOT_MAX_JUMP_SPEED;
            break;
        }
    }

    bot.actions.clear();
    bot.actions.push_back(next);
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
        if (bot.nitro_amount > 0)
        {
            s_nitro_game = true;
        }
    }
    m_bots[keeper].role = MyBot::Keeper;

#ifdef MY_DEBUG
    unsigned int currentControl;
    _controlfp_s(&currentControl, ~(_EM_INVALID | _EM_ZERODIVIDE), _MCW_EM);
#endif

    printf("v6.%llu\nbuilt %s\n", chrono::duration_cast<chrono::seconds>(chrono::system_clock::now().time_since_epoch()).count(), __DATE__ " " __TIME__);
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

        s_current_tick = game.current_tick;

        for (size_t i = 0; i < game.robots.size(); ++i)
        {
            s_world.bots[game.robots[i].id].pos = vec3((real_t)game.robots[i].x, (real_t)game.robots[i].y, (real_t)game.robots[i].z);
            s_world.bots[game.robots[i].id].vel = vec3((real_t)game.robots[i].velocity_x, (real_t)game.robots[i].velocity_y, (real_t)game.robots[i].velocity_z);
            s_world.bots[game.robots[i].id].normal = vec3(game.robots[i].touch_normal_x, game.robots[i].touch_normal_y, game.robots[i].touch_normal_z);
            s_world.bots[game.robots[i].id].touch = game.robots[i].touch;
            s_world.bots[game.robots[i].id].nitro = game.robots[i].nitro_amount;
        }

        s_world.ball.pos = vec3((real_t)game.ball.x, (real_t)game.ball.y, (real_t)game.ball.z);
        s_world.ball.vel = vec3((real_t)game.ball.velocity_x, (real_t)game.ball.velocity_y, (real_t)game.ball.velocity_z);

        if (abs(s_world.ball.pos.z) >= (s_arena.depth + s_rules.BALL_RADIUS))
        {
            return;
        }

        bool recalc = true;

        if (GetBallTick(0).pos.dist(s_world.ball.pos) > 0.001_r
            || GetBallTick(0).vel.dist(s_world.ball.vel) > 0.001_r)
        {
            GetBallTick(0) = s_world.ball;
            for (int i = 1; i < ballTicksCount; ++i)
            {
                GetBallTick(i) = BallTick(GetBallTick(i - 1));
            }
        }
        else
        {
            GetBallTick(0) = s_world.ball;
            GetBallTick(-1) = BallTick(GetBallTick(-2));
            recalc = false;
        }

        for (auto& item : m_bots)
        {
            auto& bot = item.second;
            auto& bot_body = s_world.bots[item.first];

            if (recalc)
            {
                bot.actions.clear();
            }

            if (MyBot::Forward == bot.role)
            {
                if (!bot.actions.empty())
                {
                    if (bot.actions.front().jump_speed > 0)
                    {
                        recalc = false;
                    }

                    if ((bot.actions.front().pos - bot_body.pos).len() < 0.0001_r
                        && (bot.actions.front().vel - bot_body.vel).len() < 0.0001_r)
                    {
                        continue;
                    }
                }

                ComputeForward(bot, item.first);
            }

            else if (MyBot::Keeper == bot.role)
            {
                if (!bot.actions.empty())
                {
                    if ((bot.actions.front().pos - bot_body.pos).len() < 0.00001_r
                        && (bot.actions.front().vel - bot_body.vel).len() < 0.00001_r)
                    {
                        continue;
                    }
                }

                bot.actions.push_back(NextStep());
                auto& next = bot.actions.back();

                if (!bot_body.touch)
                {
                    vec3 next_pos = bot_body.pos + bot_body.vel * s_timestep;

                    if (next_pos.dist(GetBallTick(1).pos) < (s_rules.BALL_RADIUS + s_rules.ROBOT_RADIUS))
                    {
                        next.jump_speed = s_rules.ROBOT_MAX_JUMP_SPEED;
                    }

                    if (recalc)
                    {
                        bot.target_tick = 0;
                    }

                    if (bot.target_tick > s_current_tick && s_nitro_game)
                    {
                        next.target_speed = (bot.target - bot_body.pos) / ((bot.target_tick - s_current_tick) * s_timestep);
                        next.target_speed.y += s_rules.GRAVITY * s_timestep;
                        next.use_nitro = true;
                    }

                    continue;
                }

                vec3 guard_pos = vec3(0.0_r, 0.0_r, -s_arena.depth - s_arena.goal_side_radius);
                real_t guard_r = s_arena.goal_width * s_arena.goal_width / (8.0_r * (s_arena.goal_width / 2.0_r)) + (s_arena.goal_width / 2.0_r) / 2.0_r;

                vec3 ball_dir = (s_world.ball.pos - s_home_pos);
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
                next.target_speed = target_dir.normal() * target_speed_d;
                next.target_speed.y = 0.0_r;
                next.jump_speed = 0.0_r;

                if (s_world.ball.pos.z > 0 || 
                    (s_world.ball.pos.z > (-s_arena.depth / 2.0_r) && s_world.ball.vel.z > 0))
                {
                    if (s_nitro_game && bot_body.nitro < s_rules.MAX_NITRO_AMOUNT)
                    {
                        vec3 target = vec3();
                        for (auto& pack : game.nitro_packs)
                        {
                            vec3 pack_pos = vec3(pack.x, pack.y, pack.z);
                            if (pack.alive && pack.z < 0 && bot_body.pos.dist(pack_pos) < bot_body.pos.dist(target))
                            {
                                target = pack_pos;
                            }
                        }

                        if (target.z < 0)
                        {
                            target.y = bot_body.pos.y;

                            vec3 target_dir = target - bot_body.pos;
                            next.target_speed = target_dir.normal() * s_rules.ROBOT_MAX_GROUND_SPEED;
                        }
                    }

                    continue;
                }

                const int tick_limit = ballTicksCount - 1;
                int catchTick = 1;
                auto ball_target_state = GetBallTick(catchTick);
                for (; catchTick < tick_limit; ++catchTick)
                {
                    ball_target_state = GetBallTick(catchTick);
                    if (ball_target_state.pos.dist(guard_pos) <= (guard_r + s_rules.BALL_RADIUS) && ball_target_state.pos.y < (s_max_jump_height + s_rules.BALL_RADIUS + s_rules.ROBOT_RADIUS * 1.8_r))
                    {
                        //addDebugSphere(DebugSphere({ ball_target_state.pos.x, ball_target_state.pos.y, ball_target_state.pos.z }, s_rules.BALL_RADIUS, { 1.0_r, 1.0_r, 1.0_r }, 1.0_r));
                        break;
                    }
                }

                if (catchTick == tick_limit)
                {
                    continue;
                }

                real_t tick = (real_t)catchTick;

                vec3 ball_vel = ball_target_state.vel;

                bool force_move = false;
                vec3 target_pos;
                // Если можем допрыгнуть, пробуем увести в сторону горизонтально
                if (ball_target_state.pos.y <= (s_max_jump_height + s_rules.ROBOT_RADIUS))
                {
                    ball_vel.y = 0.0_r;

                    vec3 safe_pos(s_arena.goal_width / 2.0_r + s_rules.BALL_RADIUS * 2.0_r, 0.0_r, -s_arena.depth + s_rules.BALL_RADIUS);
                    vec3 safe_dir_left = (safe_pos - ball_target_state.pos);
                    safe_dir_left.y = 0.0_r;
                    safe_dir_left.normalize();
                    safe_pos.x = -safe_pos.x;
                    vec3 safe_dir_right = (safe_pos - ball_target_state.pos);
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

                    target_pos = ball_target_state.pos + impulse.normal() * (s_rules.BALL_RADIUS + s_rules.ROBOT_RADIUS);
                    bot.target = target_pos;
                    addDebugSphere(DebugSphere({ target_pos.x, target_pos.y, target_pos.z }, 1.0_r, { 0.0_r, 1.0_r, 0.0_r }, 0.5_r));

                    vec3 target_dir_2d = target_pos - bot_body.pos;
                    real_t target_dist_y = target_dir_2d.y;
                    target_dir_2d.y = 0.0_r;
                    real_t target_dist_2d = target_dir_2d.len();

                    real_t closing_vel = bot_body.vel.dot(target_dir_2d.normal());

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
                        real_t move_time = ((-b - sqrt(d)) / 2.0_r) / s_timestep;

                        if (move_time / s_timestep > tick)
                        {
                            vec3 ball_dir = (ball_target_state.pos - s_home_pos);
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
                            next.target_speed = target_dir.normal() * target_speed_d;
                            continue;
                        }

                        if ((acceleration_time - air_time) / s_timestep >= tick)
                        {
                            next.target_speed = s_rules.MAX_ENTITY_SPEED * 2.0_r * target_dir_2d.normal();
                        }
                        else
                        {
                            next.target_speed = target_dist_2d / (tick * s_timestep) * target_dir_2d.normal();
                        }
                    }
                    else
                    {
                        vec3 ball_dir = (ball_target_state.pos - s_home_pos);
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
                        next.target_speed = target_dir.normal() * target_speed_d;
                        continue;
                    }

                    if (tick <= air_time / s_timestep && bot_body.touch)
                    {
                        next.jump_speed = s_rules.ROBOT_MAX_JUMP_SPEED;
                        bot.target_tick = s_current_tick + catchTick;
                    }
                }
                // Отбив получится скорее вверх
                else
                {
                    guard_pos = vec3(0.0_r, 0.0_r, -s_arena.depth - s_arena.goal_side_radius - s_rules.BALL_RADIUS);
                    vec3 ball_dir = guard_pos - ball_target_state.pos;
                    ball_dir.normalize();
                    target_pos = ball_target_state.pos + ball_dir * (s_rules.BALL_RADIUS + s_rules.ROBOT_RADIUS);

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
                        next.target_speed = vec3();
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
                            next.target_speed = target_dist_2d / target_time * target_dir_2d.normal();

                            real_t move_time = ((-b - sqrt(d)) / 2.0_r) / s_timestep;

                            if (move_time > target_time)
                            {
                                vec3 ball_dir = (ball_target_state.pos - s_home_pos);
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
                                next.target_speed = target_dir.normal() * target_speed_d;
                                continue;
                            }
                        }
                        else
                        {
                            vec3 ball_dir = (ball_target_state.pos - s_home_pos);
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
                            next.target_speed = target_dir.normal() * target_speed_d;
                            continue;
                        }
                    }

                    if (tick <= target_time / s_timestep && bot_body.touch)
                    {
                        next.jump_speed = s_rules.ROBOT_MAX_JUMP_SPEED;
                        bot.target_tick = s_current_tick + catchTick;
                    }
                }
            }
        }

        s_tick = game.current_tick;
    }

    MyBot& me_bot = m_bots[me.id];

    if (!me_bot.actions.empty())
    {
        auto& bot_action = me_bot.actions.front();

        action.target_velocity_x = bot_action.target_speed.x;
        action.target_velocity_y = bot_action.target_speed.y;
        action.target_velocity_z = bot_action.target_speed.z;

        action.jump_speed = (double)bot_action.jump_speed;

        if (MyBot::Forward == me_bot.role)
        {
            vec3 next_pos = bot_action.pos + bot_action.vel * s_timestep;

            vec3 ball_dir = (GetBallTick(1).pos - next_pos);

            if (next_pos.dist(s_world.ball.pos) < (s_rules.BALL_RADIUS + s_rules.ROBOT_RADIUS) && ball_dir.z > 0)
            {
                action.jump_speed = (double)s_rules.ROBOT_MAX_JUMP_SPEED;
            }
        }

        action.use_nitro = me_bot.actions.front().use_nitro;

        addDebugSphere(DebugSphere({ me_bot.target.x, me_bot.target.y, me_bot.target.z }, 1.0_r, { 1.0_r, 0.0_r, 0.0_r }, 0.5_r));

        me_bot.actions.pop_front();
    }
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

    //addDebugSphere(DebugSphere({ s_world.ball.pos.x, s_world.ball.pos.y, s_world.ball.pos.z }, s_rules.BALL_RADIUS, { 1.0_r, 1.0_r, 1.0_r }, 0.3_r));
    for (auto& ball : s_ballTicks)
    {
        sprintf_s(buffer.data(), buffer.size(), R"___(  {
    "Sphere": {
      "x": %lf,
      "y": %lf,
      "z": %lf,
      "radius": %lf,
      "r": 1.0,
      "g": 1.0,
      "b": 1.0,
      "a": 0.3
    }
  },
)___"
, (double)ball.pos.x
, (double)ball.pos.y
, (double)ball.pos.z
, (double)s_rules.BALL_RADIUS
);

        str += buffer.data();
    }

    for (auto& bot : m_bots)
    {
        for (auto& step : bot.second.actions)
        {
            sprintf_s(buffer.data(), buffer.size(), R"___(  {
    "Sphere": {
      "x": %lf,
      "y": %lf,
      "z": %lf,
      "radius": %lf,
      "r": 1.0,
      "g": 1.0,
      "b": 0.0,
      "a": 0.3
    }
  },
)___"
, (double)step.pos.x
, (double)step.pos.y
, (double)step.pos.z
, (double)s_rules.ROBOT_RADIUS
);

            str += buffer.data();
        }
    }

    str[str.size() - 2] = ']';

    m_debugSpheres.clear();

    return str;
}
#endif

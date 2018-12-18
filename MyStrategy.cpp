#include "MyStrategy.h"
#include <algorithm>
#include <cmath>
using namespace std;

#define ROBOT_MIN_RADIUS 1.0
#define ROBOT_MAX_RADIUS 1.05
#define ROBOT_MAX_JUMP_SPEED 15.0
#define ROBOT_ACCELERATION 100.0
#define ROBOT_NITRO_ACCELERATION 30.0
#define ROBOT_MAX_GROUND_SPEED 30.0
#define ROBOT_ARENA_E 0.0
#define ROBOT_RADIUS 1.0
#define ROBOT_MASS 2.0
#define TICKS_PER_SECOND 60.0
#define MICROTICKS_PER_TICK 100.0
#define RESET_TICKS (2 * TICKS_PER_SECOND)
#define BALL_ARENA_E 0.7
#define BALL_RADIUS 2.0
#define BALL_MASS 1.0
#define MIN_HIT_E 0.4
#define MAX_HIT_E 0.5
#define MAX_ENTITY_SPEED 100.0
#define MAX_NITRO_AMOUNT 100.0
#define START_NITRO_AMOUNT 50.0
#define NITRO_POINT_VELOCITY_CHANGE 0.6
#define NITRO_PACK_X 20.0
#define NITRO_PACK_Y 1.0
#define NITRO_PACK_Z 30.0
#define NITRO_PACK_RADIUS 0.5
#define NITRO_PACK_AMOUNT 100.0
#define NITRO_RESPAWN_TICKS (10 * TICKS_PER_SECOND)
#define GRAVITY 30.0

struct vec3 {
    double x = 0., y = 0., z = 0.;

    vec3() = default;
    vec3(const vec3&) = default;
    vec3(vec3&&) = default;

    vec3& operator=(const vec3& v) {
        x = v.x, y = v.y, z = v.z;
        return *this;
    }

    vec3(double d) : x(d), y(d), z(d) {}
    vec3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

    double distance(const vec3& v) {
        return sqrt((x - v.x)*(x - v.x) + (y - v.y)*(y - v.y) + (z - v.z)*(z - v.z));
    }

    double length() {
        return sqrt(x * x + y * y + z * z);
    }

    vec3 normal() {
        double len = length();
        return vec3(x / len, y / len, z / len);
    }

    vec3& normalize() {
        double len = length();
        x /= len;
        y /= len;
        z /= len;
        return *this;
    }

    vec3& operator*=(double d) {
        x *= d;
        y *= d;
        z *= d;
        return *this;
    }
};

vec3 operator-(const vec3& first, const vec3& second) {
    return vec3(first.x - second.x, first.y - second.y, first.z - second.z);
}

vec3 operator+(const vec3& first, const vec3& second) {
    return vec3(first.x + second.x, first.y + second.y, first.z + second.z);
}

vec3 operator*(const vec3& v, double d) {
    return vec3(v.x * d, v.y*d, v.z*d);
}

using namespace model;

MyStrategy::MyStrategy() { }

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action) {
    static const vec3 home_pos(0., 0., (-rules.arena.depth / 2.) + (-rules.arena.goal_width / 2.));
    static const vec3 goal_pos(0., 0., (rules.arena.depth / 2.) + (rules.arena.goal_depth));
    static const double home_r = rules.arena.goal_width / 1.2;

    vec3 ball_pos(game.ball.x, game.ball.y, game.ball.z);

    if (m_bots.empty()) {
        double dist = rules.arena.depth;
        int keeper = -1;
        for (auto& bot : game.robots) {
            if (!bot.is_teammate) {
                continue;
            }
            m_bots.emplace(bot.id, MyBot());
            vec3 bot_pos(bot.x, bot.y, bot.z);
            double home_dist = bot_pos.distance(home_pos);
            if (home_dist < dist) {
                dist = home_dist;
                keeper = bot.id;
            }
        }
        m_bots[keeper].role = MyBot::Keeper;
    }

    MyBot& me_bot = m_bots[me.id];
    vec3 me_pos(me.x, me.y, me.z);
    vec3 target;
    if (MyBot::Keeper == me_bot.role) {
        vec3 ball_dir = (ball_pos - me_pos);
        ball_dir.y = 0.;
        ball_dir.normalize();
        target = home_pos + ball_dir * home_r;
    }
    else {
        vec3 ball_dir = (ball_pos - goal_pos);
        ball_dir.y = 0.;
        ball_dir.normalize();
        ball_dir *= BALL_RADIUS;
        target = ball_pos + ball_dir;
    }

    vec3 target_dir = target - me_pos;
    target_dir.y = 0.f;
    double target_speed_d = TICKS_PER_SECOND * min(MAX_ENTITY_SPEED, target_dir.length());
    vec3 target_speed = target_dir.normal() * target_speed_d;
    target_speed.y = 0.;

    action.target_velocity_x = target_speed.x;
    action.target_velocity_y = target_speed.y;
    action.target_velocity_z = target_speed.z;

    vec3 ball_dir = ball_pos - me_pos;
    ball_dir.y = 0.;
    if (ball_dir.length() < BALL_RADIUS * 1.5 && ball_dir.z > 0.) {
        action.jump_speed = ROBOT_MAX_JUMP_SPEED;
    }
}

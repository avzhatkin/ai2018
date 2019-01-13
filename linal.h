#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _LINAL_H_
#define _LINAL_H_

#define USE_MATH
#include <cmath>

namespace linal {

using real_t = double;

constexpr real_t operator"" _r(long double v){
    return (real_t)v;
}

constexpr real_t operator"" _deg(long double v){
    return (real_t)v * (real_t)3.1415926536 / (real_t)180.0;
}

struct alignas(16) vec3
{
    real_t
        x = 0.0,
        y = 0.0,
        z = 0.0;

    vec3(){}

    vec3(real_t d) :
        x(d), y(d), z(d) {}
    vec3(real_t _x, real_t _y, real_t _z) :
        x(_x), y(_y), z(_z) {}

    real_t len() const {
        return sqrt(x*x + y * y + z * z);
    }

    vec3& normalize() {
        real_t _len = len();
        x /= _len, y /= _len, z /= _len;
        return *this;
    }

    vec3 normal() const {
        real_t _len = len();
        return vec3(x / _len, y / _len, z / _len);
    }

    static vec3 normal(const vec3& v) {
        return v.normal();
    }

    real_t dist(const vec3& other) const {
        return (*this - other).len();
    }

    static real_t dist(const vec3& first, const vec3& second)  {
        return first.dist(second);
    }

    real_t dot(const vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    static real_t dot(const vec3& first, const vec3& second) {
        return first.dot(second);
    }

    vec3 project(const vec3& other) const {
        return other.normal() * dot(other);
    }

    static vec3 project(const vec3& first, const vec3& second) {
        return first.project(second);
    }

    vec3& clamp(real_t limit) {
        real_t div = len();
        if (div <= limit)
            return *this;
        div /= limit;
        x /= div, y /= div, z /= div;
        return *this;
    }

    static vec3 clamp(const vec3& v, real_t limit) {
        real_t div = v.len();
        if (div <= limit)
            return vec3(v);
        div /= limit;
        return vec3(v.x / div, v.y / div, v.z / div);
    }
    
    vec3& operator-() {
        x = -x, y = -y, z = -z;
        return *this;
    }

    vec3& operator+=(const vec3& other) {
        x += other.x, y += other.y, z += other.z;
        return *this;
    }

    vec3& operator-=(const vec3& other) {
        x -= other.x, y -= other.y, z -= other.z;
        return *this;
    }

    vec3& operator*=(const vec3& other) {
        x *= other.x, y *= other.y, z *= other.z;
        return *this;
    }

    vec3& operator/=(const vec3& other) {
        x /= other.x, y /= other.y, z /= other.z;
        return *this;
    }

    friend vec3 operator+(vec3 left, const vec3& right) {
        return (left += right);
    }

    friend vec3 operator-(vec3 left, const vec3& right) {
        return (left -= right);
    }

    friend vec3 operator*(vec3 left, const vec3& right) {
        return (left *= right);
    }

    friend vec3 operator/(vec3 left, const vec3& right) {
        return (left /= right);
    }

    vec3& operator*=(real_t v) {
        x *= v, y *= v, z *= v;
        return *this;
    }

    vec3& operator/=(real_t v) {
        x /= v, y /= v, z /= v;
        return *this;
    }

    friend vec3 operator*(vec3 left, real_t v) {
        return (left *= v);
    }

    friend vec3 operator/(vec3 left, real_t v) {
        return (left /= v);
    }

    friend vec3 operator*(real_t v, vec3 right) {
        return (right *= v);
    }

    friend vec3 operator/(real_t v, vec3 right) {
        return (right /= v);
    }
};

};

#endif // _LINAL_H_

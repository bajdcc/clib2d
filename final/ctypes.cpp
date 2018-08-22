//
// Project: clib2d
// Created by bajdcc
//

#include <cassert>
#include "ctypes.h"

namespace clib {

    vec2::vec2() : x(0), y(0) {

    }

    vec2::vec2(decimal _x, decimal _y) : x(_x), y(_y) {

    }

    decimal vec2::operator[](size_t idx) {
        assert(idx <= 2);
        return idx == 0 ? x : y;
    }

    const decimal &vec2::operator[](size_t idx) const {
        assert(idx <= 2);
        return idx == 0 ? x : y;
    }

    vec2 vec2::operator-() const {
        return vec2{-x, -y};
    }

    decimal vec2::magnitude() const {
        return std::sqrt(x * x + y * y);
    }

    vec2 vec2::normal() const {
        return vec2(y, -x).normalized();
    }

    vec2 vec2::normalized() const {
        return *this / magnitude();
    }

    vec2 operator+(const vec2 &a, const vec2 &b) {
        return {a.x + b.x, a.y + b.y};
    }

    void operator+=(vec2 &a, const vec2 &b) {
        a = a + b;
    }

    vec2 operator-(const vec2 &a, const vec2 &b) {
        return {a.x - b.x, a.y - b.y};
    }

    void operator-=(vec2 &a, const vec2 &b) {
        a = a - b;
    }

    vec2 operator*(const vec2 &a, decimal b) {
        return {a.x * b, a.y * b};
    }

    vec2 operator*(decimal a, const vec2 &b) {
        return b * a;
    }

    void operator*=(vec2 &a, decimal b) {
        a = a * b;
    }

    vec2 operator/(const vec2 &a, decimal b) {
        return {a.x / b, a.y / b};
    }

    void operator/=(vec2 &a, decimal b) {
        a = a / b;
    }

    mat22::mat22() : mat22(0, 0, 0, 0) {}

    mat22::mat22(const std::array<vec2, 2> &mat) : _mat(mat) {}

    mat22::mat22(decimal a, decimal b, decimal c, decimal d)
        : _mat{{{a, b}, {c, d}}} {}

    decimal mat22::det() const {
        return _mat[0][0] * _mat[1][1] - _mat[0][1] * _mat[1][0];
    }

    mat22 mat22::inverse() const {
        return (1 / det()) * mat22(_mat[1][1], -_mat[0][1], -_mat[1][0], _mat[0][0]);
    }

    mat22 mat22::transpose() const {
        return mat22{_mat[0][0], _mat[1][0], _mat[0][1], _mat[1][1]};
    }

    vec2 &mat22::operator[](size_t idx) {
        assert(idx <= 2);
        return _mat[idx];
    }

    const vec2 &mat22::operator[](size_t idx) const {
        assert(idx <= 2);
        return _mat[idx];
    }

    const mat22 mat22::I = {1, 0, 0, 1};

    mat22 operator+(const mat22 &a, const mat22 &b) {
        return {a[0].x + b[0].x, a[0].y + b[0].y,
                a[1].x + b[1].x, a[1].y + b[1].y};
    }

    void operator+=(mat22 &a, const mat22 &b) {
        a = a + b;
    }

    mat22 operator+(const mat22 &a, decimal b) {
        return a + b * mat22::I;
    }

    mat22 operator+(decimal a, const mat22 &b) {
        return b + a;
    }

    void operator+=(mat22 &a, decimal b) {
        a = a + b;
    }

    mat22 operator-(const mat22 &a, const mat22 &b) {
        return {a[0].x - b[0].x, a[0].y - b[0].y,
                a[1].x - b[1].x, a[1].y - b[1].y};
    }

    void operator-=(mat22 &a, const mat22 &b) {
        a = a - b;
    }

    mat22 operator-(const mat22 &a, decimal b) {
        return a - b * mat22::I;
    }

    mat22 operator-(decimal a, const mat22 &b) {
        return a * mat22::I - b;
    }

    void operator-=(mat22 &a, decimal b) {
        a = a - b;
    }

    mat22 operator*(const mat22 &a, decimal b) {
        return {a[0].x * b, a[0].y * b,
                a[1].x * b, a[1].y * b};
    }

    mat22 operator*(decimal a, const mat22 &b) {
        return b * a;
    }

    void operator*=(mat22 &a, decimal b) {
        a = a * b;
    }

    vec2 operator*(const vec2 &a, const mat22 &b) {
        return {a[0] * b[0][0] + a[1] * b[1][0],
                a[0] * b[0][1] + a[1] * b[1][1]};
    }

    vec2 operator*(const mat22 &a, const vec2 &b) {
        return {a[0][0] * b[0] + a[0][1] * b[1],
                a[1][0] * b[0] + a[1][1] * b[1]};
    }

    void operator*=(vec2 &a, const mat22 &b) {
        a = a * b;
    }

    mat22 operator*(const mat22 &a, const mat22 &b) {
        return {a[0][0] * b[0][0] + a[0][1] * b[1][0],
                a[0][0] * b[0][1] + a[0][1] * b[1][1],
                a[1][0] * b[0][0] + a[1][1] * b[1][0],
                a[1][0] * b[0][1] + a[1][1] * b[1][1]};
    }

    void operator*=(mat22 &a, const mat22 &b) {
        a = a * b;
    }

    vec2 cross(decimal a, const vec2 &b) {
        return a * vec2(-b.y, b.x);
    }

    decimal cross(const vec2 &a, const vec2 &b) {
        return a.x * b.y - a.y * b.x;
    }

    decimal dot(const vec2 &a, const vec2 &b) {
        return a.x * b.x + a.y * b.y;
    }

    mat22 rotate(decimal theta) {
        const auto _sin = std::sin(theta);
        const auto _cos = std::cos(theta);
        return mat22{_cos, -_sin, _sin, _cos};
    }
}

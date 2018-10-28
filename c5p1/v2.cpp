//
// Project: clib2d
// Created by bajdcc
//

#include "v2.h"

namespace clib {

    v2::v2(decimal _x, decimal _y) : x(_x), y(_y) {}

    v2 v2::operator*(decimal d) const {
        return {x * d, y * d};
    }

    v2 v2::operator/(decimal d) const {
        return {x / d, y / d};
    }

    v2 v2::operator+(const v2 &v) const {
        return {x + v.x, y + v.y};
    }

    v2 v2::operator-(const v2 &v) const {
        return {x - v.x, y - v.y};
    }

    v2 v2::operator+(decimal d) const {
        return {x + d, y + d};
    }

    v2 v2::operator-(decimal d) const {
        return {x - d, y - d};
    }

    v2 &v2::operator+=(const v2 &v) {
        x += v.x;
        y += v.y;
        return *this;
    }

    v2 &v2::operator-=(const v2 &v) {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    v2 operator*(decimal d, const v2 &v) {
        return {d * v.x, d * v.y};
    }

    v2 v2::operator-() const {
        return {-x, -y};
    }

    decimal v2::cross(const v2 &v) const {
        return x * v.y - y * v.x;
    }

    decimal v2::dot(const v2 &v) const {
        return x * v.x + y * v.y;
    }

    decimal v2::magnitude() const {
        return std::sqrt(x * x + y * y);
    }

    decimal v2::magnitude_square() const {
        return x * x + y * y;
    }

    v2 v2::normalize() const {
        return *this / magnitude();
    }

    v2 v2::normal() const {
        return N().normalize();
    }

    v2 v2::N() const {
        return v2{y, -x};
    }

    bool v2::zero(decimal d) const {
        return std::abs(x) < d && std::abs(y) < d;
    }
}
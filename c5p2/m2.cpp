//
// Project: clib2d
// Created by bajdcc
//

#include "m2.h"

namespace clib {

    m2::m2(decimal _x1, decimal _y1, decimal _x2, decimal _y2) : x1(_x1), y1(_y1), x2(_x2), y2(_y2) {}

    m2::m2(decimal d) : x1(d), y1(0), x2(0), y2(d) {}

    m2 m2::operator+(const m2 &m) const {
        return {x1 + m.x1, y1 + m.y1, x2 + m.x2, y2 + m.y2};
    }

    m2 m2::operator*(decimal d) const {
        return {x1 * d, y1 * d, x2 * d, y2 * d};
    }

    v2 m2::operator*(const v2 &v) const {
        return {x1 * v.x + y1 * v.y, x2 * v.x + y2 * v.y};
    }

    m2 operator*(decimal d, const m2 &m) {
        return m * d;
    }

    const m2 &m2::rotate(decimal theta) {
        const auto _sin = std::sin(theta);
        const auto _cos = std::cos(theta);
        *this = m2{_cos, -_sin, _sin, _cos};
        return *this;
    }

    v2 m2::rotate(const v2 &v) const {
        return {x1 * v.x + y1 * v.y, x2 * v.x + y2 * v.y};
    }

    decimal m2::det() const {
        return x1 * y2 - x2 * y1;
    }

    m2 m2::inv() const {
        auto _det = det();
        return _det == 0 ? m2(inf, inf, inf, inf) : ((1 / _det) * m2(y2, -x2, -y1, x1));
    }
}
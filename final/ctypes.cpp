//
// Project: clib2d
// Created by bajdcc
//

#include "ctypes.h"

namespace clib {

    vec2 normal(const vec2 &v) {
        return glm::normalize(vec2(v.y, -v.x));
    }

    decimal dot(const vec2 &a, const vec2 &b) {
        return a.x * b.x + a.y * b.y;
    }

    decimal cross(const vec2 &a, const vec2 &b) {
        return a.x * b.y - a.y * b.x;
    }

    vec2 cross(decimal a, const vec2 &b) {
        return a * vec2(-b.y, b.x);
    }

    mat22 rotate(decimal theta) {
        const auto _sin = std::sin(theta);
        const auto _cos = std::cos(theta);
        return mat22{_cos, -_sin, _sin, _cos};
    }
}

//
// Project: clib2d
// Created by bajdcc
//

#ifndef CGFLUID_CTYPES_H
#define CGFLUID_CTYPES_H

#include <glm/glm.hpp>

#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
    TypeName(const TypeName&) = delete; \
    const TypeName& operator=(const TypeName&) = delete;
#define INF (std::numeric_limits<clib::decimal>::infinity())
#define MAKE_ID(a,b) (((a)<(b))?(((a)<<16)|(b)):(((b)<<16)|(a)))

namespace clib {

    using vec2 = glm::dvec2;
    using vec3 = glm::dvec3;
    using decimal = vec2::value_type;
    using mat22 = glm::dmat2x2;

    const auto inf = INF;

    vec2 normal(const vec2 &);
    decimal dot(const vec2 &, const vec2 &);
    decimal cross(const vec2 &, const vec2 &);
    vec2 cross(decimal a, const vec2& b);
    mat22 rotate(decimal theta);
}

#endif //CGFLUID_CTYPES_H

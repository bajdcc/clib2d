//
// Project: clib2d
// Created by bajdcc
//

#ifndef CGFLUID_CTYPES_H
#define CGFLUID_CTYPES_H

#include <limits>
#include <cmath>
#include <array>

#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
    TypeName(const TypeName&) = delete; \
    const TypeName& operator=(const TypeName&) = delete;
#define MAKE_ID(a, b) (((a)<(b))?(((a)<<16)|(b)):(((b)<<16)|(a)))

namespace clib {

    using decimal = double;

    struct vec2 {
        decimal x, y;

        vec2();
        vec2(decimal, decimal);
        decimal operator[](size_t idx);
        const decimal &operator[](size_t idx) const;
        vec2 operator-() const;
        decimal magnitude() const;;
        vec2 normal() const;
        vec2 normalized() const;
    };

    vec2 operator+(const vec2 &a, const vec2 &b);
    void operator+=(vec2 &a, const vec2 &b);
    vec2 operator-(const vec2 &a, const vec2 &b);
    void operator-=(vec2 &a, const vec2 &b);
    vec2 operator*(const vec2 &a, decimal b);
    vec2 operator*(decimal a, const vec2 &b);
    void operator*=(vec2 &a, decimal b);
    vec2 operator/(const vec2 &a, decimal b);
    void operator/=(vec2 &a, decimal b);
    decimal dot(const vec2 &a, const vec2 &b);
    decimal cross(const vec2 &a, const vec2 &b);
    vec2 cross(decimal a, const vec2 &b);

    struct mat22 {
        std::array<vec2, 2> _mat;

        mat22();
        mat22(const std::array<vec2, 2> &mat);
        mat22(decimal a, decimal b, decimal c, decimal d);
        decimal det() const;
        mat22 inverse() const;
        mat22 transpose() const;
        vec2 &operator[](size_t idx);
        const vec2 &operator[](size_t idx) const;
        static const mat22 I;
    };

    mat22 operator+(const mat22 &a, const mat22 &b);
    void operator+=(mat22 &a, const mat22 &b);
    mat22 operator+(const mat22 &a, decimal b);
    mat22 operator+(decimal a, const mat22 &b);
    void operator+=(mat22 &a, decimal b);
    mat22 operator-(const mat22 &a, const mat22 &b);
    void operator-=(mat22 &a, const mat22 &b);
    mat22 operator-(decimal a, const mat22 &b);
    void operator-=(mat22 &a, decimal b);
    mat22 operator*(const mat22 &a, decimal b);
    mat22 operator*(decimal a, const mat22 &b);
    void operator*=(mat22 &a, decimal b);
    vec2 operator*(const vec2 &a, const mat22 &b);
    vec2 operator*(const mat22 &a, const vec2 &b);
    void operator*=(vec2 &a, const mat22 &b);
    mat22 operator*(const mat22 &a, const mat22 &b);
    void operator*=(mat22 &a, const mat22 &b);
    mat22 rotate(decimal a);

    const auto inf = std::numeric_limits<clib::decimal>::infinity();
}

#endif //CGFLUID_CTYPES_H

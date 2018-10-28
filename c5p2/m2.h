//
// Project: clib2d
// Created by bajdcc
//

#ifndef CLIB2D_M2_H
#define CLIB2D_M2_H

#include "c2d.h"
#include "v2.h"

namespace clib {
    // 二维矩阵
    struct m2 {
        decimal x1{1}, y1{0}, x2{0}, y2{1};

        m2() = default;

        m2(decimal _x1, decimal _y1, decimal _x2, decimal _y2);

        m2(const m2 &m) = default;

        m2 &operator=(const m2 &m) = default;

        m2(decimal d);

        m2 operator+(const m2 &m) const;

        m2 operator*(decimal d) const;

        v2 operator*(const v2 &v) const;

        friend m2 operator*(decimal d, const m2 &m);

        const m2 &rotate(decimal theta);

        v2 rotate(const v2 &v) const;

        decimal det() const;

        m2 inv() const;
    };
}

#endif //CLIB2D_M2_H

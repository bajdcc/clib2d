//
// Project: clib2d
// Created by bajdcc
//

#include "c2d.h"

namespace clib {

    decimal_inv::decimal_inv(decimal v) {
        set(v);
    }

    void decimal_inv::set(decimal v) {
        value = v;
        if (std::isinf(value))
            inv = 0;
        else if (std::abs(value) < EPSILON)
            inv = inf;
        inv = 1 / value;
    }

    decimal_square::decimal_square(decimal v) {
        set(v);
    }

    void decimal_square::set(decimal v) {
        value = v;
        square = value * value;
    }
}
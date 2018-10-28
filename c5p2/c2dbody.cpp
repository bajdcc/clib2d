//
// Project: clib2d
// Created by bajdcc
//

#include "c2dbody.h"

namespace clib {

    c2d_body::c2d_body(uint16_t _id, decimal _mass) : id(_id), mass(_mass) {}

    v2 c2d_body::rotate(const v2 &v) const {
        return m2().rotate(angle).rotate(v);
    }
}
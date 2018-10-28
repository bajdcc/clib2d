//
// Project: clib2d
// Created by bajdcc
//

#include "c2dcontact.h"

namespace clib {

    contact::contact(v2 _pos) : pos(_pos), ta(C2D_POLYGON), tb(C2D_POLYGON) {}

    contact::contact(v2 _pos, size_t index) : contact(_pos) {
        A.polygon.idx = index;
        B.polygon.idx = index;
    }

    bool contact::operator==(const contact &other) const {
        if (ta == C2D_POLYGON) {
            if (tb == C2D_POLYGON) {
                if (A.polygon.idx == other.A.polygon.idx && B.polygon.idx == other.B.polygon.idx) {
                    return true;
                }
                return A.polygon.idx == other.B.polygon.idx && B.polygon.idx == other.A.polygon.idx; // 是否反了
            } else {
                return A.polygon.idx == other.A.polygon.idx;
            }
        } else {
            return true;
        }
    }

    bool contact::operator!=(const contact &other) const {
        return !(*this == other);
    }
}
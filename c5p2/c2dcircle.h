//
// Project: clib2d
// Created by bajdcc
//

#ifndef CLIB2D_C2DCIRCLE_H
#define CLIB2D_C2DCIRCLE_H

#include "c2dbody.h"

namespace clib {
// 圆形刚体（正圆）
    class c2d_circle : public c2d_body {
    public:
        using ptr = std::unique_ptr<c2d_circle>;

        c2d_circle(uint16_t _id, decimal _mass, decimal _r);

        bool contains(const v2 &pt) override;

        void init();

        void impulse(const v2 &p, const v2 &r) override;

        v2 world() const override;

        c2d_body_t type() const override;

        v2 min() const override;

        v2 max() const override;

        void update(v2 gravity, int n) override;

        void pass0();

        void pass1();

        void pass2();

        void pass3(const v2 &gravity);

        void pass4();

        void pass5();

        // 拖拽物体
        void drag(const v2 &pt, const v2 &offset) override;

        void draw() override;

        decimal_square r; // 半径
    };
}

#endif //CLIB2D_C2DCIRCLE_H

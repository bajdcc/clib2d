//
// Project: clib2d
// Created by bajdcc
//

#ifndef CLIB2D_C2DREVOLUTE_H
#define CLIB2D_C2DREVOLUTE_H

#include "c2djoint.h"

namespace clib {
    // 旋转关节
    class c2d_revolute_joint : public c2d_joint {
    public:
        void prepare(const v2 &gravity) override;

        void update(const v2 &gravity) override;

        void draw() override;

        v2 world_anchor_a() const;

        v2 world_anchor_b() const;

        c2d_revolute_joint(c2d_body *_a, c2d_body *_b, const v2 &_anchor);

        c2d_revolute_joint(const c2d_revolute_joint &) = delete; // 禁止拷贝
        c2d_revolute_joint &operator=(const c2d_revolute_joint &) = delete; // 禁止赋值

        // 世界坐标-锚点 World anchor
        v2 anchor;
        // 物体A重心-锚点-本地坐标 Local anchor relative to centroid of body a
        v2 local_anchor_a;
        // 物体B重心-锚点-本地坐标 Local anchor relative to centroid of body b
        v2 local_anchor_b;

        // 上次状态 Cached status in prev step
        // 锚到物体A重心-相对坐标 Anchor point to body a' centroid
        v2 ra;
        // 锚到物体B重心-相对坐标 Anchor point to body b' centroid
        v2 rb;
        // 质量矩阵 The combined mass
        m2 mass;
        // 冲量 Accumulated impulse
        v2 p;
        // 冲量累计 Accumulated impulse
        v2 p_acc;
        // 补偿 The bias for position correction
        v2 bias;
    };
}

#endif //CLIB2D_C2DREVOLUTE_H

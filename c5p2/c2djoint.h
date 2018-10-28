//
// Project: clib2d
// Created by bajdcc
//

#ifndef CLIB2D_C2DJOINT_H
#define CLIB2D_C2DJOINT_H

#include "c2dbody.h"

namespace clib {
    // 关节
    class c2d_joint {
    public:
        using ptr = std::unique_ptr<c2d_joint>;

        virtual void prepare(const v2 &gravity) = 0; // 预处理
        virtual void update(const v2 &gravity) = 0; // 计算
        virtual void draw() = 0; // 绘制

        c2d_joint(c2d_body *_a, c2d_body *_b);

        c2d_joint(const c2d_body &) = delete; // 禁止拷贝
        c2d_joint &operator=(const c2d_joint &) = delete; // 禁止赋值

        c2d_body *a, *b; // 关节涉及的两个物体
    };
}

#endif //CLIB2D_C2DJOINT_H

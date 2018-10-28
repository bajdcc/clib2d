//
// Project: clib2d
// Created by bajdcc
//

#ifndef CLIB2D_C2DCONTACT_H
#define CLIB2D_C2DCONTACT_H

#include "c2dbody.h"

namespace clib {
    // 接触点
    struct contact {
        v2 pos; // 位置
        v2 ra, rb; // 物体重心到接触点的向量
        c2d_body_t ta, tb; // 物体的类型
        decimal sep{0}; // 分离投影（重叠距离）
        decimal mass_normal{0};
        decimal mass_tangent{0};
        decimal bias{0};
        decimal pn{0}; // 法向冲量
        decimal pt{0}; // 切向冲量
        union {
            struct {
                int idx; // （交点属于的）物体a和b的边索引+1，为正则属于B，为负属于A
            } polygon;
            struct {
                // none
            } circle;
        } A{0}, B{0};

        contact(v2 _pos);

        contact(v2 _pos, size_t index);

        bool operator==(const contact &other) const;

        bool operator!=(const contact &other) const;
    };
}

#endif //CLIB2D_C2DCONTACT_H

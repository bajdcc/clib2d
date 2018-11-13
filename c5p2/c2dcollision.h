//
// Project: clib2d
// Created by bajdcc
//

#ifndef CLIB2D_C2DCOLLISION_H
#define CLIB2D_C2DCOLLISION_H

#include <vector>
#include "c2dcontact.h"
#include "c2dpolygon.h"
#include "c2dcircle.h"

namespace clib {
    // 碰撞结构
    struct collision {
        std::vector<contact> contacts; // 接触点列表
        c2d_body *bodyA{nullptr}, *bodyB{nullptr}; // 碰撞的两个物体
        union intern {
            struct {
                size_t idx;
                decimal sat;
            } polygon;
        } A{0}, B{0};
        v2 N; // 法线
    };

    // 碰撞检测-SAT分离轴定理
    // 检测两凸包是否相交
    // 表述：如果两个凸多边形没有相交，那么存在这两个物体在一个轴上的投影不重叠。
    // 轴：只需采用两个凸包的每个条做检测即可
    // 只要最大间隙大于零，即为不相交
    // separation：最大间隙
    // idx：最大间隙的轴
    // 参考Box2D：https://github.com/erincatto/Box2D/blob/master/Box2D/Collision/b2CollidePolygon.cpp#L23
    int max_separating_axis(c2d_body *a, c2d_body *b, collision::intern &c);

    // 先用包围盒方法快速判断碰撞
    bool AABB_collide(c2d_body *a, c2d_body *b);

    // 参考Box2D：https://github.com/erincatto/Box2D/blob/master/Box2D/Collision/b2CollidePolygon.cpp#L64
    static size_t incident_edge(const v2 &N, c2d_body *body);

    // Sutherland-Hodgman（多边形裁剪）
    // 参考Box2D：https://github.com/erincatto/Box2D/blob/master/Box2D/Collision/b2Collision.cpp#L201
    size_t clip(std::vector<contact> &out,
                const std::vector<contact> &in,
                size_t i,
                const v2 &p1, const v2 &p2);

    // 计算碰撞
    bool solve_collision_internal(collision &c);

    // 计算碰撞（返回是否碰撞）
    bool solve_collision(collision &c);

    // 碰撞计算
    void collision_update(collision &c, const collision &old_c);
}

#endif //CLIB2D_C2DCOLLISION_H

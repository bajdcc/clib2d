//
// Project: clib2d
// Created by bajdcc
//

#ifndef CLIB2D_C2DBODY_H
#define CLIB2D_C2DBODY_H

#include <memory>
#include "c2d.h"
#include "v2.h"
#include "m2.h"

namespace clib {
    enum c2d_body_t {
        C2D_POLYGON,
        C2D_CIRCLE,
    };

    // 刚体基类，由于必然要多态，因此不能用struct
    // 该类为动态创建，所以要用unique_ptr承担内存管理
    class c2d_body {
    public:
        using ptr = std::unique_ptr<c2d_body>;

        c2d_body(uint16_t _id, decimal _mass);

        c2d_body(const c2d_body &) = delete; // 禁止拷贝
        c2d_body &operator=(const c2d_body &) = delete; // 禁止赋值

        virtual void drag(const v2 &pt, const v2 &offset) = 0;  // 拖动，施加力矩
        virtual bool contains(const v2 &pt) = 0;  // 是否包含该世界坐标

        virtual void impulse(const v2 &p, const v2 &r) = 0;  // 计算冲量

        virtual v2 world() const = 0; // 世界坐标
        virtual c2d_body_t type() const = 0; // 类型

        virtual v2 min() const = 0; // 下边界
        virtual v2 max() const = 0; // 上边界

        // 分阶段
        // i=0，重置外力
        // i=1，第一阶段：计算速度、角速度
        // i=2，第二阶段，计算位置等其他量
        virtual void update(v2 gravity, int) = 0; // 状态更新
        virtual void draw() = 0; // 绘制

        v2 rotate(const v2 &v) const;

        // 不想写那么多get/set，先public用着
#if ENABLE_SLEEP
        bool sleep{false}; // 是否休眠
#endif
        bool statics{false}; // 是否为静态物体
        int collision{0}; // 参与碰撞的次数
        uint16_t id{0}; // ID
        decimal_inv mass{1}; // 质量
        v2 pos; // 位置（世界坐标，下面未注明均为本地坐标）
        v2 V; // 速度
        decimal angle{0}; // 角度
        decimal angleV{0}; // 角速度
        decimal_inv inertia{0}; // 转动惯量
        decimal f{0.2}; // 滑动/静摩擦系数
        v2 F; // 受力
        v2 Fa; // 受力（累计）
        decimal M{0}; // 力矩
        decimal CO{COLL_CO}; // 弹性碰撞系数
    };
}

#endif //CLIB2D_C2DBODY_H

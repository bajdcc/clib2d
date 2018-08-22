//
// Project: clib2d
// Created by bajdcc
//

#ifndef CGFLUID_CJOINT_H
#define CGFLUID_CJOINT_H

#include <memory>
#include "ctypes.h"
#include "cbody.h"

namespace clib {

    // 关节
    class cjoint {
    public:
        using ptr = std::shared_ptr<cjoint>;

        cjoint(cbody::ptr a, cbody::ptr b);
        virtual ~cjoint() = default;

        virtual void pre_step(decimal dt) = 0;
        virtual void update_impulse() = 0;

        cbody::ptr get_a() const;
        void set_a(cbody::ptr a);
        cbody::ptr get_b() const;
        void set_b(cbody::ptr b);

    protected:
        DISALLOW_COPY_AND_ASSIGN(cjoint)

        std::weak_ptr<cbody> _a, _b; // 关节相联结的两个刚体
    };

    // 旋转关节
    class revolute_joint : public cjoint {
    public:
        using ptr = std::shared_ptr<cjoint>;

        revolute_joint(cbody::ptr a, cbody::ptr b, const vec2& anchor);

        void pre_step(decimal dt) override;
        void update_impulse() override;

        const vec2& anchor() const;
        vec2 world_anchor_a() const;
        vec2 world_anchor_b() const;

    protected:
        DISALLOW_COPY_AND_ASSIGN(revolute_joint)

        vec2 _anchor; // 固定位置（世界坐标）
        vec2 _local_anchor_a; // 刚体a相对坐标
        vec2 _local_anchor_b; // 刚体b相对坐标

        vec2 _ra; // a旋转角度向量
        vec2 _rb; // b旋转角度向量
        mat22 _mass; // 总质量
        vec2 _p; // 动量
        vec2 _bias; // 修正
    };
}

#endif //CGFLUID_CJOINT_H

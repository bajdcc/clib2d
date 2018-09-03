//
// Project: clib2d
// Created by bajdcc
//

#ifndef CGFLUID_WORLD_H
#define CGFLUID_WORLD_H

#include <vector>
#include <memory>
#include <unordered_map>
#include "cbody.h"
#include "cjoint.h"
#include "cpair.h"
#include "ctypes.h"

namespace clib {

    // 物理引擎-世界
    // 管理刚体、关节，检测碰撞，更新位置，进行逐帧计算
    class cworld {
    public:
        cworld(const vec2 &gravity);
        ~cworld() = default;

        using body_list = std::vector<cbody::ptr>;
        using joint_list = std::vector<cjoint::ptr>;
        using pair_list = std::unordered_map<uint32_t, cpair::ptr>;

        void add(cbody::ptr body);
        void add(cjoint::ptr joint);
        const vec2 &get_gravity() const;
        const body_list &get_bodies() const;
        const joint_list &get_joints() const;
        const pair_list &get_arbiters() const;

        void move(const vec2 &v);

        void clear();
        void step(decimal dt);

        bool is_pause() const;
        void set_pause(bool pause);

    private:
        bool _pause{false}; // 是否暂停
        vec2 _gravity; // 重力
        size_t _iterations{10};

        body_list _bodies; // 刚体列表
        joint_list _joints; // 关节数组
        pair_list _arbiters; // 碰撞检测对列表
    };

    class cfactory {
    public:
        static polygon_body::ptr make_box(decimal mass, decimal width, decimal height,
                                          const vec2 &position = vec2());
        static polygon_body::ptr make_polygon(decimal mass,
                                              const polygon_body::vertex_list &vertices,
                                              const vec2 &position = vec2());
        static polygon_body::ptr make_fence(cworld &world);
        static cpair::ptr make_arbiter(cbody::ptr a, cbody::ptr b,
                                       const vec2 &normal,
                                       const cpair::contact_list &contacts = cpair::contact_list());
        static revolute_joint::ptr make_revolute_joint(cbody::ptr a, cbody::ptr b,
                                                       const vec2 &anchor);
    };
}

#endif //CGFLUID_WORLD_H

//
// Project: clib2d
// Created by bajdcc
//

#ifndef CLIB2D_C2DWORLD_H
#define CLIB2D_C2DWORLD_H

#include <vector>
#include <unordered_map>
#include "c2dbody.h"
#include "c2djoint.h"
#include "c2dpolygon.h"
#include "c2dcircle.h"
#include "c2drevolute.h"
#include "c2dcollision.h"
#include "cvm.h"
#include "cparser.h"

namespace clib {

    class c2d_world {
    public:
        c2d_world() = default;
        ~c2d_world() = default;

        c2d_polygon *make_polygon(decimal mass, const std::vector<v2> &vertices, const v2 &pos, bool statics = false);
        c2d_polygon *make_rect(decimal mass, decimal w, decimal h, const v2 &pos, bool statics = false);
        c2d_circle *make_circle(decimal mass, decimal r, const v2 &pos, bool statics = false);
        c2d_revolute_joint *make_revolute_joint(c2d_body *a, c2d_body *b, const v2 &anchor);

        // 根据位置找到物体
        c2d_body *find_body(const v2 &pos);

        static uint32_t make_id(uint16_t a, uint16_t b);
        bool collision_detection(const c2d_body::ptr &a, c2d_body::ptr &b);
        decltype(auto) sleep_bodies() const;
        // 碰撞检测
        void collision_detection();
        // https://github.com/erincatto/Box2D/blob/master/Box2D/Dynamics/Contacts/b2ContactSolver.cpp#L127
        // 碰撞计算准备
        void collision_prepare(collision &c) {
            static const auto kBiasFactor = COLL_BIAS; // 弹性碰撞系数
            const auto &a = *c.bodyA;
            const auto &b = *c.bodyB;
            auto tangent = c.N.normal(); // 接触面
            // 先计算好碰撞系数相关的量
            for (auto &contact : c.contacts) {
                auto nA = contact.ra.cross(c.N);
                auto nB = contact.rb.cross(c.N);
                auto kn = a.mass.inv + b.mass.inv +
                          std::abs(a.inertia.inv) * nA * nA +
                          std::abs(b.inertia.inv) * nB * nB;
                contact.mass_normal = kn > 0 ? COLL_NORMAL_SCALE / kn : 0.0;
                auto tA = contact.ra.cross(tangent);
                auto tB = contact.rb.cross(tangent);
                auto kt = a.mass.inv + b.mass.inv +
                          std::abs(a.inertia.inv) * tA * tA +
                          std::abs(b.inertia.inv) * tB * tB;
                contact.mass_tangent = kt > 0 ? COLL_TANGENT_SCALE / kt : 0.0;
                contact.bias = -kBiasFactor * dt_inv * std::min(0.0, contact.sep);
            }
        }

        // 绘制碰撞情况
        void draw_collision(const collision &c);

        // https://github.com/erincatto/Box2D/blob/master/Box2D/Dynamics/Contacts/b2ContactSolver.cpp#L324
        // 碰撞计算
        void collision_update(collision &c);

#if ENABLE_SLEEP
        // 去除休眠物体的碰撞
        void collision_remove_sleep();
#endif

        void step();
        void move(const v2 &v);
        void rotate(decimal d);
        void offset(const v2 &pt, const v2 &offset);
        void mouse(const v2 &pt, bool down);
        void motion(const v2 &pt);

        // 清除所有物体
        void clear();

        // 建立四周边界
        void make_bound();

        // 场景
        void scene(int id);

        // 初始化
        void init();

        size_t get_collision_size() const;
        size_t get_sleeping_size() const;
        void invert_gravity();

    private:
        void start_animation(uint32_t id);
        void stop_animation();
        void run_animation();

    public:
        static std::chrono::system_clock::time_point last_clock;
        static decimal dt;
        static decimal dt_inv;
        static bool paused; // 是否暂停
        static std::string title; // 标题

    private:
        uint32_t animation_id{0};
        std::string animation_code;
        cvm vm;
        cparser *parser;
        cval *root;

        bool mouse_drag{false};
        v2 global_drag; // 鼠标拖动
        v2 global_drag_offset; // 鼠标拖动位移

        // uint32_t 是由 a、b 两个物体的 id 组成
        std::unordered_map<uint32_t, collision> collisions; // 碰撞情况

        std::vector<c2d_body::ptr> bodies; // 寻常物体
        std::vector<c2d_body::ptr> static_bodies; // 静态物体
        std::vector<c2d_joint::ptr> joints; // 关节
        uint16_t global_id{1};
        v2 gravity{0, GRAVITY}; // 重力
    };

    extern c2d_world *world;
}

#endif //CLIB2D_C2DWORLD_H

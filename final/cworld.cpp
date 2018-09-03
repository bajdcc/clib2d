//
// Project: clib2d
// Created by bajdcc
//

#include "cworld.h"

namespace clib {

    cworld::cworld(const vec2 &gravity) : _gravity(gravity) {}

    void cworld::add(cbody::ptr body) { _bodies.push_back(body); }

    void cworld::add(cjoint::ptr joint) { _joints.push_back(joint); }

    const vec2 &cworld::get_gravity() const { return _gravity; }

    const cworld::body_list &cworld::get_bodies() const { return _bodies; }

    const cworld::joint_list &cworld::get_joints() const { return _joints; }

    const cworld::pair_list &cworld::get_arbiters() const {
        return _arbiters;
    }

    void cworld::move(const vec2 &v) {
        for (auto &body : _bodies) {
            if (body->get_inv_mass() > 0)
                body->update_impulse(v * body->get_inv_mass(), vec2());
        }
    }

    void cworld::step(decimal dt) {
        if (_pause)
            return;
        // 碰撞检测
        for (size_t i = 0; i < _bodies.size(); ++i) {
            for (size_t j = i + 1; j < _bodies.size(); ++j) {
                auto a = std::dynamic_pointer_cast<polygon_body>(_bodies[i]);
                auto b = std::dynamic_pointer_cast<polygon_body>(_bodies[j]);
                if (!a->can_collide(*b)) {
                    continue;
                }
                uint32_t id;
                auto arbiter = cpair::is_collide(a, b, id);
                auto iter = _arbiters.find(id);
                if (arbiter == nullptr) {
                    if (iter != _arbiters.end()) {
                        _arbiters.erase(id);
                    }
                } else if (iter == _arbiters.end()) {
                    _arbiters[id] = arbiter;
                } else {
                    auto &old_arbiter = iter->second;
                    arbiter->update(*old_arbiter);
                    _arbiters[id] = arbiter;
                }
            }
        }
        for (auto &kv : _arbiters) {
            kv.second->pre_step(dt);
        }
        for (auto &joint : _joints) {
            joint->pre_step(dt);
        }

        // 更新动量
        for (size_t i = 0; i < _iterations; ++i) {
            for (auto &kv : _arbiters) {
                kv.second->update_impulse();
            }
            for (auto &joint : _joints) {
                joint->update_impulse();
            }
        }

        // 更新外力
        for (auto &body : _bodies) {
            body->update_force(_gravity, dt);
        }
    }

    void cworld::clear() {
        _arbiters.clear();
        _joints.clear();
        _bodies.clear();
    }

    bool cworld::is_pause() const {
        return _pause;
    }

    void cworld::set_pause(bool pause) {
        _pause = pause;
    }

// ---------------------------------------------------
// cfactory

    static uint16_t global_id = 1;

    polygon_body::ptr cfactory::make_box(decimal mass, decimal width, decimal height, const vec2 &position) {
        // 四边形
        // 注意顶点呈逆时针
        polygon_body::vertex_list vertices = {
            {width / 2,  height / 2},
            {-width / 2, height / 2},
            {-width / 2, -height / 2},
            {width / 2,  -height / 2}
        };
        auto body = std::make_shared<polygon_body>(global_id++, mass, vertices);
        body->set_position(position);
        return body;
    }

    polygon_body::ptr
    cfactory::make_polygon(decimal mass, const polygon_body::vertex_list &vertices, const vec2 &position) {
        auto body = std::make_shared<polygon_body>(global_id++, mass, vertices);
        body->set_position(position);
        return body;
    }

    polygon_body::ptr cfactory::make_fence(cworld &world) {
        auto ground = make_box(inf, 20, 1, {0, -0.5});
        world.add(ground);
        world.add(make_box(inf, 20, 1, {0, 16.5}));
        world.add(make_box(inf, 1, 18, {-9.5, 8}));
        world.add(make_box(inf, 1, 18, {9.5, 8}));
        return ground;
    }

    cpair::ptr
    cfactory::make_arbiter(cbody::ptr a, cbody::ptr b, const vec2 &normal, const cpair::contact_list &contacts) {
        return std::make_shared<cpair>(a, b, normal, contacts);
    }

    revolute_joint::ptr cfactory::make_revolute_joint(cbody::ptr a, cbody::ptr b, const vec2 &anchor) {
        return std::make_shared<revolute_joint>(a, b, anchor);
    }
}

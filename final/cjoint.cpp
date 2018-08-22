//
// Project: clib2d
// Created by bajdcc
//

#include "cjoint.h"

namespace clib {

    cjoint::cjoint(cbody::ptr a, cbody::ptr b) : _a(a), _b(b) {}

    cbody::ptr cjoint::get_a() const {
        return _a.lock();
    }

    void cjoint::set_a(cbody::ptr a) {
        _a = a;
    }

    cbody::ptr cjoint::get_b() const {
        return _b.lock();
    }

    void cjoint::set_b(cbody::ptr b) {
        _b = b;
    }

// ---------------------------------------------------
// revolute_joint

    revolute_joint::revolute_joint(cbody::ptr a, cbody::ptr b, const vec2 &anchor)
        : cjoint(a, b), _anchor(anchor) {
        _local_anchor_a = a->get_rotation().transpose() * (_anchor - a->local_to_world(a->get_centroid()));
        _local_anchor_b = b->get_rotation().transpose() * (_anchor - b->local_to_world(b->get_centroid()));
    }

    void revolute_joint::pre_step(decimal dt) {
        static const decimal kBiasFactor = 0.2;
        auto a = _a.lock();
        auto b = _b.lock();
        _ra = a->get_rotation() * _local_anchor_a;
        _rb = b->get_rotation() * _local_anchor_b;
        auto k = (a->get_inv_mass() + b->get_inv_mass()) * mat22::I +
                 a->get_inv_inertia() * mat22(_ra.y*_ra.y, -_ra.y*_ra.x, -_ra.y*_ra.x, _ra.x*_ra.x) +
                 b->get_inv_inertia() * mat22(_rb.y*_rb.y, -_rb.y*_rb.x, -_rb.y*_rb.x, _rb.x*_rb.x);
        _mass = k.inverse();
        _bias = -kBiasFactor / dt *(b->local_to_world(b->get_centroid())
            + _rb - a->local_to_world(a->get_centroid()) - _ra);
        a->update_impulse(-_p, _ra);
        b->update_impulse(_p, _rb);
    }

    void revolute_joint::update_impulse() {
        auto a = _a.lock();
        auto b = _b.lock();
        auto dv = (b->get_velocity() + cross(b->get_angular_velocity(), _rb)) -
                  (a->get_velocity() + cross(a->get_angular_velocity(), _ra));
        auto p = _mass * (-1.0 * dv + _bias);
        a->update_impulse(-p, _ra);
        b->update_impulse(p, _rb);
        _p += p;
    }

    const vec2 &revolute_joint::anchor() const { return _anchor; }

    vec2 revolute_joint::world_anchor_a() const {
        auto a = _a.lock();
        return a->local_to_world(a->get_rotation() * _local_anchor_a + a->get_centroid());
    }

    vec2 revolute_joint::world_anchor_b() const {
        auto b = _b.lock();
        return b->local_to_world(b->get_rotation() * _local_anchor_b + b->get_centroid());
    }
}

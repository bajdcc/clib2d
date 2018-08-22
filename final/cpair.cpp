//
// Project: clib2d
// Created by bajdcc
//

#include "cpair.h"
#include "cworld.h"

#include <cassert>
#include <algorithm>

namespace clib {

    ccontact::ccontact(const polygon_body &b, size_t idx) {
        indices = {{idx, idx}};
        std::fill(from_a.begin(), from_a.end(), false);
        position = b.local_to_world(b[idx]);
    }

    bool ccontact::operator==(const ccontact &other) const {
        if (from_a == other.from_a && indices == other.indices) {
            return true;
        }
        decltype(from_a) from_a_swap = {{from_a[1], from_a[0]}};
        decltype(indices) indices_swap = {{indices[1], indices[0]}};
        return from_a_swap == other.from_a && indices_swap == other.indices;
    }

    bool ccontact::operator!=(const ccontact &other) const {
        return !(*this == other);
    }

// ---------------------------------------------------
// pair

    cpair::cpair(cbody::ptr a, cbody::ptr b, const vec2 &normal, const cpair::contact_list &contacts)
        : _a(a), _b(b), _normal(normal), _contacts(contacts) {}

    const cpair::contact_list &cpair::get_contacts() const {
        return _contacts;
    }

    const vec2 &cpair::get_normal() const {
        return _normal;
    }

    void cpair::pre_step(decimal dt) {
        static const decimal kAllowedPenetration = 0.01;
        static const decimal kBiasFactor = 0.2; // 弹性碰撞系数，1.0为完全弹性碰撞
        auto tangent = _normal.normal();
        auto a = _a.lock();
        auto b = _b.lock();
        for (auto &contact : _contacts) {
            auto kn = a->get_inv_mass() + b->get_inv_mass() +
                dot(a->get_inv_inertia() * cross(cross(contact.ra, _normal), contact.ra) +
                    b->get_inv_inertia() * cross(cross(contact.rb, _normal), contact.rb), _normal);
            auto kt = a->get_inv_mass() + b->get_inv_mass() +
                dot(a->get_inv_inertia() * cross(cross(contact.ra, tangent), contact.ra) +
                    b->get_inv_inertia() * cross(cross(contact.rb, tangent), contact.rb), tangent);

            contact.mass_normal = 1 / kn;
            contact.mass_tangent = 1 / kt;
            contact.bias = -kBiasFactor / dt * std::min(0.0, contact.separation + kAllowedPenetration);
        }
    }

    void cpair::update_impulse() {
        auto tangent = _normal.normal();
        auto a = _a.lock();
        auto b = _b.lock();
        for (auto &contact : _contacts) {
            vec2 dv = (b->get_velocity() + cross(b->get_angular_velocity(), contact.rb)) -
                      (a->get_velocity() + cross(a->get_angular_velocity(), contact.ra));

            auto vn = dot(dv, _normal);
            auto dpn = (-vn + contact.bias) * contact.mass_normal;
            dpn = std::max(contact.pn + dpn, 0.0) - contact.pn;

            decimal friction = std::sqrt(a->get_friction() * b->get_friction());
            auto vt = dot(dv, tangent);
            auto dpt = -vt * contact.mass_tangent;
            dpt = std::max(-friction * contact.pn, std::min(friction * contact.pn, contact.pt + dpt)) - contact.pt;

            auto p = dpn * _normal + dpt * tangent;
            a->update_impulse(-p, contact.ra);
            b->update_impulse(p, contact.rb);
            contact.pn += dpn;
            contact.pt += dpt;
        }
    }

    void cpair::update(const cpair &old_arbiter) {
        const auto &old_contacts = old_arbiter._contacts;
        auto tangent = _normal.normal();
        auto a = _a.lock();
        auto b = _b.lock();
        for (auto &new_contact : _contacts) {
            auto old_contact = std::find(old_contacts.begin(), old_contacts.end(), new_contact);
            if (old_contact != old_contacts.end()) {
                new_contact.pn = old_contact->pn;
                new_contact.pt = old_contact->pt;

                auto p = new_contact.pn * _normal + new_contact.pt * tangent;
                a->update_impulse(-p, new_contact.ra);
                b->update_impulse(p, new_contact.rb);
            }
        }
    }

    void cpair::add_contact(const ccontact &contact) {
        _contacts.push_back(contact);
    }

// ---------------------------------------------------
// collision detection

    // 找出最小间隙
    static size_t incident_edge(const vec2 &N, const polygon_body &body) {
        size_t idx = SIZE_MAX;
        auto min_dot = inf;
        // 遍历B物体的边
        for (size_t i = 0; i < body.count(); ++i) {
            // 获得边上的法向量
            auto edge_normal = body.edge(i).normal();
            // 获得法向量在SAT轴上的投影长度
            auto _dot = dot(edge_normal, N);
            // 找出最小投影，即最小间隙
            if (_dot < min_dot) {
                min_dot = _dot; // 最小间隙
                idx = i; // 返回索引
            }
        }
        return idx;
    }

    static size_t clip(cpair::contact_list &contacts_out,
                       const cpair::contact_list &contacts_in,
                       size_t idx, const vec2 &v0, const vec2 &v1) {
        size_t num_out = 0;
        // 得到A物体的边v0_v1的单位向量
        auto N = (v1 - v0).normalized();
        // dist0 = v0_B0 X N
        auto dist0 = cross(contacts_in[0].position - v0, N);
        // dist1 = v0_B1 X N
        auto dist1 = cross(contacts_in[1].position - v0, N);
        if (dist0 <= 0) {
            // v0_B0 与 N 共线或顺时针
            contacts_out[num_out++] = contacts_in[0];
        }
        if (dist1 <= 0) {
            // v0_B1 与 N 共线或顺时针
            contacts_out[num_out++] = contacts_in[1];
        }
        if (dist0 * dist1 < 0) { // 一正一负
            auto total_dist = dist0 - dist1;
            auto v = (contacts_in[0].position * -dist1 + contacts_in[1].position * dist0) / total_dist;
            assert(!std::isnan(v.x) && !std::isnan(v.y));
            contacts_out[num_out].position = v;
            contacts_out[num_out].from_a[0] = true;
            contacts_out[num_out].indices[0] = idx;

            ++num_out;
        }
        assert(num_out <= 2);
        return num_out;
    }

    cpair::ptr cpair::is_collide(polygon_body::ptr &pa, polygon_body::ptr &pb, uint32_t &id) {
        auto _pa = &pa;
        auto _pb = &pb;
        size_t ia, ib;
        decimal sa, sb;
        if ((sa = pa->min_separating_axis(ia, *pb)) >= 0) {
            id = MAKE_ID(pa->get_id(), pb->get_id());
            return nullptr;
        }
        if ((sb = pb->min_separating_axis(ib, *pa)) >= 0) {
            id = MAKE_ID(pa->get_id(), pb->get_id());
            return nullptr;
        }
        // 当且仅当SAT全为负，则表示相交
        if (sa < sb) { // 有序排列
            std::swap(sa, sb);
            std::swap(ia, ib);
            std::swap(_pa, _pb);
        }
        auto &a = **_pa;
        auto &b = **_pb;
        // 获得SAT的轴
        auto N = a.edge(ia).normal();
        // 获得最小间隙时的B物体起点索引
        auto idx = incident_edge(N, b);
        // 获得最小间隙时的B物体终点索引
        auto next_idx = (idx + 1) % b.count();
        // 关节列表，暂时认为边 idx-next_idx 是包括在重叠部分中的
        // 下面需要判断B物体其余边是否与A物体重叠
        cpair::contact_list contacts = {{b, idx},
                                        {b, next_idx}};
        auto clipped_contacts = contacts;
        // 遍历A物体
        for (size_t i = 0; i < a.count(); ++i) {
            if (i == ia) { // 非SAT轴的边
                continue;
            }
            // 起点
            auto v0 = a.local_to_world(a[i]);
            // 终点
            auto v1 = a.local_to_world(a[(i + 1) % a.count()]);
            auto num = clip(clipped_contacts, contacts, i, v0, v1);
            if (num < 2) {
                id = MAKE_ID(a.get_id(), b.get_id());
                return nullptr;
            }
            assert(num == 2);
            contacts = clipped_contacts;
        }

        auto va = a.local_to_world(a[ia]);
        auto arbiter = cfactory::make_arbiter(*_pa, *_pb, N);
        for (auto &contact : clipped_contacts) {
            auto sep = dot(contact.position - va, N);
            if (sep <= 0) {
                contact.separation = sep;
                contact.ra = contact.position - a.local_to_world(a.get_centroid());
                contact.rb = contact.position - b.local_to_world(b.get_centroid());
                arbiter->add_contact(contact);
            }
        }
        id = MAKE_ID(a.get_id(), b.get_id());
        return arbiter;
    }
}

//
// Project: clib2d
// Created by bajdcc
//

#include <algorithm>
#include "c2dcollision.h"

namespace clib {

    int max_separating_axis_polygon(c2d_polygon *a, c2d_polygon *b, collision::intern &c) {
        c.polygon.sat = -inf;
        // 遍历几何物体A的所有顶点
        for (size_t i = 0; i < a->edges(); ++i) {
            // 获得A各顶点的世界坐标
            auto va = a->vertex(i);
            // 获得当前顶点到下一顶点的边的单位法向量
            auto N = a->edge(i).normal();
            // 最小分离向量
            auto min_sep = inf;
            // 遍历几何物体B
            for (size_t j = 0; j < b->edges(); ++j) {
                // 获得B各顶点的世界坐标
                auto vb = b->vertex(j);
                // vb - va = 从顶点A到顶点B的向量
                // normal  = 从顶点A到顶点A'的单位向量
                // dot(vb - va, normal) = 若点B到边AA'投影为P，结果为AP的长度
                // 由于这里取最小值，因此
                // min_sep = 以AA'边的法向量N为轴，将B物体各顶点所做投影的最小长度
                min_sep = std::min(min_sep, (vb - va).dot(N));
            }
            if (min_sep > c.polygon.sat) {
                c.polygon.sat = min_sep; // 寻找最大间隙
                c.polygon.idx = i; // 轴
            }
        }
        return c.polygon.sat > 0 ? 0 : 1; // 0则不相交
    }

    int max_separating_axis_polygon_circle(c2d_polygon *a, c2d_circle *b, collision::intern &c) {
        c.polygon.sat = inf;
        // 遍历几何物体A的所有顶点
        for (size_t i = 0; i < a->edges(); ++i) {
            // 获得A各顶点的世界坐标
            const auto va = a->vertex(i);
            // 获得当前顶点到下一顶点的边
            const auto edge = a->edge(i);
            // 获得当前顶点到下一顶点的边长度
            const auto edgeL = edge.magnitude();
            // 获得当前顶点到下一顶点的边的单位向量
            const auto ab = edge.normalize();
            // 起点到圆心
            const auto ac = b->pos - va;
            // 投影
            const auto sat = ab.dot(ac);
            // 交点在AB线段上
            if (sat >= 0 && sat <= edgeL) { // AB上
                // 圆心到AB的距离
                const auto sep = std::sqrt(ac.magnitude_square() - sat * sat) - b->r.value;
                if (sep < 0 && sep < c.polygon.sat) {
                    c.polygon.sat = sep; // 寻找最大间隙
                    c.polygon.idx = i; // 轴
                }
            } else if (sat > -b->r.value) { // AB，A前
                const auto sep = (b->pos - va).magnitude() - b->r.value;
                if (sep < 0 && sep < c.polygon.sat) {
                    c.polygon.sat = sep; // 寻找最大间隙
                    c.polygon.idx = i; // 轴
                }
            } else if (sat <= edgeL + b->r.value) { // AB，B后
                const auto sep = a->vertex(i + 1).magnitude() - b->r.value;
                if (sep < 0 && sep < c.polygon.sat) {
                    c.polygon.sat = sep; // 寻找最大间隙
                    c.polygon.idx = i; // 轴
                }
            }
        }
        return c.polygon.sat > 0 ? 0 : 1; // 0则不相交
    }

    int max_separating_axis_circle(c2d_circle *a, c2d_circle *b) {
        const auto r = a->r.value + b->r.value;
        if ((a->pos - b->pos).magnitude_square() < r * r) { // 距离小于半径和，为相交
            return 1;
        }
        return 0;
    }

    int max_separating_axis(c2d_body *a, c2d_body *b, collision::intern &c) {
        const auto typeA = a->type();
        const auto typeB = b->type();
        if (typeA == C2D_POLYGON) {
            if (typeB == C2D_POLYGON) {
                return max_separating_axis_polygon(
                    dynamic_cast<c2d_polygon *>(a),
                    dynamic_cast<c2d_polygon *>(b),
                    c);
            } else if (typeB == C2D_CIRCLE) {
                const auto val = max_separating_axis_polygon_circle(
                    dynamic_cast<c2d_polygon *>(a),
                    dynamic_cast<c2d_circle *>(b),
                    c);
                return val == 1 ? 1 : 2;
            }
        } else if (typeA == C2D_CIRCLE) {
            if (typeB == C2D_POLYGON) {
                return max_separating_axis(b, a, c);
            } else if (typeB == C2D_CIRCLE) {
                const auto val = max_separating_axis_circle(
                    dynamic_cast<c2d_circle *>(a),
                    dynamic_cast<c2d_circle *>(b));
                return val == 1 ? 1 : 2;
            }
        }
        return 0;
    }

    bool AABB_collide(c2d_body *a, c2d_body *b) {
        const auto boundMinA = a->min();
        const auto boundMinB = b->min();
        const auto boundMaxA = a->max();
        const auto boundMaxB = b->max();
        auto centerA = (boundMaxA + boundMinA) / 2; // 矩形包围圈中心点
        auto centerB = (boundMaxB + boundMinB) / 2;
        auto sizeA = (boundMaxA - boundMinA) / 2; // 矩形包围圈大小的二分之一
        auto sizeB = (boundMaxB - boundMinB) / 2;
        return std::abs(centerB.x - centerA.x) <= (sizeA.x + sizeB.x) &&
               std::abs(centerB.y - centerA.y) <= (sizeA.y + sizeB.y);
    }

    size_t incident_edge(const v2 &N, c2d_polygon *body) {
        size_t idx = SIZE_MAX;
        auto min_dot = inf;
        // 遍历B物体的边
        for (size_t i = 0; i < body->edges(); ++i) {
            // 获得边上的法向量
            auto edge_normal = body->edge(i).normal();
            // 获得法向量在SAT轴上的投影长度
            auto dot = edge_normal.dot(N);
            // 找出最小投影，即最小间隙
            if (dot < min_dot) {
                min_dot = dot; // 最小间隙
                idx = i; // 返回索引
            }
        }
        return idx;
    }

    size_t clip(std::vector<contact> &out, const std::vector<contact> &in, size_t i, const v2 &p1, const v2 &p2) {
        size_t num_out = 0;
        auto N = (p2 - p1).normal();
        // 计算投影
        auto dist0 = N.dot(in[0].pos - p1);
        auto dist1 = N.dot(in[1].pos - p1);

        // 如果投影都小于零，则B中两点都在A内
        if (dist0 <= 0) out[num_out++] = in[0];
        if (dist1 <= 0) out[num_out++] = in[1];

        // 否则两点一个在A内，一个在A外
        if (dist0 * dist1 < 0) {
            // 计算比率
            auto interp = dist0 / (dist0 - dist1);
            // 计算p1,p2与in1,in2交点
            out[num_out].pos = in[0].pos + interp * (in[1].pos - in[0].pos);
            out[num_out].A.polygon.idx = -(int) i - 1;
            ++num_out;
        }

        return num_out;
    }

    bool solve_collision_polygon(collision &c) {
        auto bodyA = dynamic_cast<c2d_polygon *>(c.bodyA);
        auto bodyB = dynamic_cast<c2d_polygon *>(c.bodyB);
        // 计算SAT的轴法线
        // edge = A物体离B物体最近的边
        // N = edge的法线，指向B物体
        c.N = bodyA->edge(c.A.polygon.idx).normal();
        // 此时要找到B物体中离A物体最近的边
        c.B.polygon.idx = incident_edge(c.N, bodyB);

        decltype(c.contacts) contacts;
        // 假定两个接触点（即idxB两端点）
        contacts.emplace_back(bodyB->vertex(c.B.polygon.idx), bodyB->index(c.B.polygon.idx) + 1);
        contacts.emplace_back(bodyB->vertex(c.B.polygon.idx + 1), bodyB->index(c.B.polygon.idx + 1) + 1);
        auto tmp = contacts;

        // 将idxB线段按bodyA进行多边形裁剪
        for (size_t i = 0; i < bodyA->edges(); ++i) {
            if (i == c.A.polygon.idx)
                continue;
            if (clip(tmp, contacts, i, bodyA->vertex(i), bodyA->vertex(i + 1)) < 2)
                return false;
            contacts = tmp;
        }

        auto va = bodyA->vertex(c.A.polygon.idx);

        auto &pos0 = contacts[0].pos;
        auto &pos1 = contacts[1].pos;
        const auto CO = bodyA->CO * bodyB->CO;
        auto dist = std::abs((va - pos0).dot(c.N));
        auto bias = std::log10(1 + dist) * CO;
        pos0 -= c.N * bias;
        dist = std::abs((va - pos1).dot(c.N));
        bias = std::log10(1 + dist) * CO;
        pos1 -= c.N * bias;

        // 筛选交点
        for (auto &contact : contacts) {
            // 交点：contact.pos
            // 参考点：接触边端点va
            // 接触边法向量（指向物体B）
            auto sep = (contact.pos - va).dot(c.N);
            if (sep <= 0) { // 找在idxA向bodyA一侧的（bodyA内的接触点）
                contact.sep = sep; // sep越小，端点va到交点pos所成线段的斜率越接近法线N
                contact.ra = contact.pos - c.bodyA->world();
                contact.rb = contact.pos - c.bodyB->world();
                c.contacts.push_back(contact);
            }
        }

        return true;
    }

    bool solve_collision_polygon_circle(collision &c) {
        auto bodyA = dynamic_cast<c2d_polygon *>(c.bodyA);
        auto bodyB = dynamic_cast<c2d_circle *>(c.bodyB);

        decltype(c.contacts) contacts;
        // 假定两个接触点（即idxA两端点）
        contacts.emplace_back(bodyA->vertex(c.A.polygon.idx), -bodyA->index(c.A.polygon.idx) - 1);
        contacts.emplace_back(bodyA->vertex(c.A.polygon.idx + 1), -bodyA->index(c.A.polygon.idx + 1) - 1);

        auto &pos0 = contacts[0].pos;
        auto &pos1 = contacts[1].pos;

        const auto va = pos0;

        const auto abL = (pos1 - pos0).magnitude();
        const auto ab = (pos1 - pos0).normalize();

        c.N = ab.normal();

        const auto sat = ab.dot(bodyB->pos - pos0);
        const auto CO = bodyA->CO * bodyB->CO;

        if (sat >= 0 && sat <= abL) { // 圆与线相交
            const auto p1 = pos0 + ab * sat; // 垂足
            const auto p0 = bodyB->pos - c.N * (bodyB->r.value); // 圆上一点
            const auto dist = (p1 - p0).dot(c.N);
            if (dist > COLL_CIR_POLY_BIAS) {
                const auto bias = std::log10(1 + dist) * CO;
                pos0 = p0 - c.N * bias;
                contacts.erase(contacts.begin() + 1);
            }
        } else if (c.bodyB->contains(pos0)) { // 起点在圆内
            const auto ca = (bodyB->pos - pos0).normalize();
            const auto pt = bodyB->pos - ca * bodyB->r.value;
            c.N = ca;
            const auto dist = (pos0 - pt).dot(c.N);
            const auto bias = std::log10(1 + dist) * CO;
            pos0 = pt - c.N * bias;
            contacts.erase(contacts.begin() + 1);
        } else if (c.bodyB->contains(pos1)) { // 终点在圆内
            const auto ca = (bodyB->pos - pos1).normalize();
            const auto pt = bodyB->pos - ca * bodyB->r.value;
            c.N = ca;
            const auto dist = (pos1 - pt).dot(c.N);
            const auto bias = std::log10(1 + dist) * CO;
            pos0 = pt - c.N * bias;
            contacts.erase(contacts.begin() + 1);
        }

        // 筛选交点
        for (auto &contact : contacts) {
            // 交点：contact.pos
            // 参考点：接触边端点va
            // 接触边法向量（指向物体B）
            auto sep = (contact.pos - va).dot(c.N);
            if (sep <= 0) { // 找在idxA向bodyA一侧的（bodyA内的接触点）
                contact.sep = sep; // sep越小，端点va到交点pos所成线段的斜率越接近法线N
                contact.ta = C2D_POLYGON;
                contact.tb = C2D_CIRCLE;
                contact.ra = contact.pos - c.bodyA->world();
                contact.rb = contact.pos - c.bodyB->world();
                c.contacts.push_back(contact);
            }
        }

        return true;
    }

    bool solve_collision_circle(collision &c) {
        auto bodyA = dynamic_cast<c2d_circle *>(c.bodyA);
        auto bodyB = dynamic_cast<c2d_circle *>(c.bodyB);

        c.N = (bodyB->pos - bodyA->pos).normalize();

        decltype(c.contacts) contacts;
        contacts.emplace_back(bodyA->pos + c.N * bodyA->r.value);
        contacts.emplace_back(bodyB->pos - c.N * bodyB->r.value);

        auto &pos0 = contacts[0].pos;
        auto &pos1 = contacts[1].pos;

        const auto CO = bodyA->CO * bodyB->CO;
        const auto dist = (pos0 - pos1).dot(c.N);
        const auto bias = std::log10(1 + dist) * CO;
        pos0 += c.N * bias;
        pos1 -= c.N * bias;

        const auto va = (contacts[0].pos + contacts[1].pos) / 2 - c.N.normal();

        // 筛选交点
        for (auto &contact : contacts) {
            // 交点：contact.pos
            // 参考点：接触边端点va
            // 接触边法向量（指向物体B）
            auto sep = (contact.pos - va).dot(c.N);
            if (sep <= 0) { // 找在idxA向bodyA一侧的（bodyA内的接触点）
                contact.sep = sep; // sep越小，端点va到交点pos所成线段的斜率越接近法线N
                contact.ta = C2D_CIRCLE;
                contact.tb = C2D_CIRCLE;
                contact.ra = contact.pos - c.bodyA->world();
                contact.rb = contact.pos - c.bodyB->world();
                c.contacts.push_back(contact);
            }
        }

        return true;
    }

    bool solve_collision(collision &c) {
        const auto typeA = c.bodyA->type();
        const auto typeB = c.bodyB->type();
        if (typeA == C2D_POLYGON) {
            if (typeB == C2D_POLYGON) {
                if (c.A.polygon.sat < c.B.polygon.sat) { // 排列：A比B的SAT更大，更接近零
                    std::swap(c.bodyA, c.bodyB);
                    std::swap(c.A, c.B);
                }
                return solve_collision_polygon(c);
            } else if (typeB == C2D_CIRCLE) {
                return solve_collision_polygon_circle(c);
            }
        } else if (typeA == C2D_CIRCLE) {
            if (typeB == C2D_POLYGON) {
                std::swap(c.bodyA, c.bodyB);
                std::swap(c.A, c.B);
                return solve_collision_polygon_circle(c);
            } else if (typeB == C2D_CIRCLE) {
                return solve_collision_circle(c);
            }
        }
        return false;
    }

    void collision_update(collision &c, const collision &old_c) {
        auto &a = *c.bodyA;
        auto &b = *c.bodyB;
        const auto &old_contacts = old_c.contacts;
        for (auto &new_contact : c.contacts) {
            auto old_contact = std::find(old_contacts.begin(), old_contacts.end(), new_contact);
            if (old_contact != old_contacts.end()) { // 同一个碰撞点的更新
                new_contact.pn = old_contact->pn;
                new_contact.pt = old_contact->pt;

                auto tangent = c.N.normal(); // 新的切线
                auto p = new_contact.pn * c.N + new_contact.pt * tangent; // 新的冲量
                a.impulse(-p, new_contact.ra); // 施加力矩
                b.impulse(p, new_contact.rb);
            }
        }
    }
}
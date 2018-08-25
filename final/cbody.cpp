//
// Project: clib2d
// Created by bajdcc
//

#include "cbody.h"

namespace clib {

    cbody::cbody(uint16_t id, decimal mass) : _id(id) { set_mass(mass); }

    // 判断碰撞条件
    bool cbody::can_collide(const cbody &other) const {
        // 只要质量不是无穷大，都可以发生碰撞
        // 当且仅当两者都是无穷大时，不检测碰撞
        // 这种情况是当：两者为边界时
        return !(std::isinf(_mass) && std::isinf(other._mass));
    }

    // 更新动量
    void cbody::update_impulse(const vec2 &impulse, const vec2 &r) {
        if (std::isinf(_mass))
            return;
        // 线动量：P = m * v, v += delta_P / m
        _velocity += impulse * _inv_mass;
        // 角动量：omega = I * beta, beta += delta_I / omega
        // omega = r X P
        _angular_velocity += _inv_inertia * cross(r, impulse);
    }

    // 更新力
    void cbody::update_force(const vec2 &gravity, decimal dt) {
        if (std::isinf(_mass))
            return;
        // 合外力 = 质量 × 线加速度 = 线动量的变化率
        // V += a * t, a = g + F / m
        _velocity += (gravity + _force * _inv_mass) * dt;
        // 合外力矩 = 转动惯量 × 角加速度 = 角动量的变化率
        // beta += (M / I) * t
        _angular_velocity += (_torque * _inv_inertia) * dt;
        // s += v * t
        _position += _velocity * dt;
        // theta += beta * t
        _rotation = rotate(_angular_velocity * dt) * _rotation;
    }

    vec2 cbody::local_to_world(const vec2 &local_point) const {
        return _position + local_point;
    }

    uint16_t cbody::get_id() const {
        return _id;
    }

    decimal cbody::get_mass() const {
        return _mass;
    }

    void cbody::set_mass(decimal mass) {
        _mass = mass;
        _inv_mass = 1 / _mass;
    }

    decimal cbody::get_inv_mass() const {
        return _inv_mass;
    }

    decimal cbody::get_inertia() const {
        return _inertia;
    }

    void cbody::set_inertia(decimal inertia) {
        _inertia = inertia;
        _inv_inertia = std::isinf(inertia) ? 0 : 1 / inertia;
    }

    decimal cbody::get_inv_inertia() const {
        return _inv_inertia;
    }

    const vec2 &cbody::get_centroid() const {
        return _centroid;
    }

    void cbody::set_centroid(const vec2 &centroid) {
        _centroid = centroid;
    }

    const vec2 &cbody::get_position() const {
        return _position;
    }

    void cbody::set_position(const vec2 &position) {
        _position = position;
    }

    const mat22 &cbody::get_rotation() const {
        return _rotation;
    }

    void cbody::set_rotation(const mat22 &rotation) {
        _rotation = rotation;
    }

    const vec2 &cbody::get_velocity() const {
        return _velocity;
    }

    void cbody::set_velocity(const vec2 &velocity) {
        _velocity = velocity;
    }

    decimal cbody::get_angular_velocity() const {
        return _angular_velocity;
    }

    void cbody::set_angular_velocity(decimal angular_velocity) {
        _angular_velocity = angular_velocity;
    }

    const vec2 &cbody::get_force() const {
        return _force;
    }

    void cbody::set_force(const vec2 &force) {
        _force = force;
    }

    decimal cbody::get_torque() const {
        return _torque;
    }

    void cbody::set_torque(decimal torque) {
        _torque = torque;
    }

    decimal cbody::get_friction() const {
        return _friction;
    }

    void cbody::set_friction(decimal friction) {
        _friction = friction;
    }

// ---------------------------------------------------
// polygon_body

    // 计算多边形面积
    static decimal calc_polygon_area(const std::vector<vec2> &vertices) {
        decimal area = 0;
        auto size = vertices.size();
        // 求所有三角形的面积之和
        for (size_t i = 0; i < size; ++i) {
            auto j = (i + 1) % size;
            // 叉乘求两相邻向量所成平行四边形面积
            // 所以要求三角形面积就要除以2
            area += cross(vertices[i], vertices[j]);
        }
        return area / 2;
    }

    // 计算多边形重心
    static vec2 calc_polygon_centroid(const std::vector<vec2> &vertices) {
        vec2 gc;
        auto size = vertices.size();
        // 重心 = (各三角形重心 * 其面积) / 总面积
        // 三角形重心 = 两向量之和 / 3
        for (size_t i = 0; i < size; ++i) {
            auto j = (i + 1) % size;
            gc += (vertices[i] + vertices[j]) * cross(vertices[i], vertices[j]);
        }
        return gc / 6.0 / calc_polygon_area(vertices);
    }

    // 计算多边形转动惯量
    static decimal calc_polygon_inertia(decimal mass, const cbody::vertex_list &vertices) {
        decimal acc0 = 0, acc1 = 0;
        auto size = vertices.size();
        // 转动惯量 = m / 6 * (各三角形面积 * 其(a*a+a*b+b*b)) / (总面积)
        for (size_t i = 0; i < size; ++i) {
            auto a = vertices[i], b = vertices[(i + 1) % size];
            auto _cross = std::abs(cross(a, b));
            acc0 += _cross * (dot(a, a) + dot(b, b) + dot(a, b));
            acc1 += _cross;
        }
        return mass * acc0 / 6 / acc1;
    }

    polygon_body::polygon_body(uint16_t id, decimal mass, const cbody::vertex_list &vertices)
        : cbody(id, mass), _vertices(vertices) {
        set_inertia(calc_polygon_inertia(mass, vertices));
        set_centroid(calc_polygon_centroid(vertices));
    }

    size_t polygon_body::count() const { return _vertices.size(); }

    vec2 polygon_body::operator[](size_t idx) const {
        // 顶点进行旋转变换，返回旋转后位置
        return _rotation * (_vertices[idx] - _centroid) + _centroid;
    }

    vec2 polygon_body::edge(size_t idx) const {
        return (*this)[(idx + 1) % _vertices.size()] - (*this)[idx];
    }

    // 碰撞检测-SAT分离轴定理
    // 检测两凸包是否相交
    // 表述：如果两个凸多边形没有相交，那么存在这两个物体在一个轴上的投影不重叠。
    // 轴：只需采用两个凸包的每个条做检测即可
    decimal polygon_body::min_separating_axis(size_t &idx, const polygon_body &other) const {
        decimal separation = -inf;
        // 遍历几何物体A的所有顶点
        for (size_t i = 0; i < _vertices.size(); ++i) {
            // 获得A各顶点的世界坐标
            auto va = local_to_world((*this)[i]);
            // 获得当前顶点到下一顶点的边的单位法向量
            auto N = edge(i).normal();
            // 最小分离向量
            auto min_sep = inf;
            // 遍历几何物体B
            for (size_t j = 0; j < other.count(); ++j) {
                // 获得B各顶点的世界坐标
                auto vb = other.local_to_world(other[j]);
                // vb - va = 从顶点A到顶点B的向量
                // normal  = 从顶点A到顶点A'的单位向量
                // dot(vb - va, normal) = 若点B到边AA'投影为P，结果为AP的长度
                // 由于这里取最小值，因此
                // min_sep = 以AA'边的法向量N为轴，将B物体各顶点所做投影的最小长度
                min_sep = std::min(min_sep, dot(vb - va, N));
            }
            if (min_sep > separation) {
                separation =  min_sep; // 寻找最大间隙
                idx = i; // 轴
            }
        }
        return separation; // 非负则表示不相交
    }
}

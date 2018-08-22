//
// Project: clib2d
// Created by bajdcc
//

#ifndef _CGFLUID_CBODYH
#define _CGFLUID_CBODYH

#include <vector>
#include <memory>
#include "ctypes.h"

namespace clib {

    // 刚体
    class cbody {
    public:
        using vertex_list = std::vector<vec2>;
        using ptr = std::shared_ptr<cbody>;

        cbody(uint16_t id, decimal mass);
        virtual ~cbody() = default;

        bool can_collide(const cbody &other) const;
        void update_impulse(const vec2 &impulse, const vec2 &r);
        void update_force(const vec2 &gravity, decimal dt);

        vec2 local_to_world(const vec2 &) const;

        uint16_t get_id() const;

        decimal get_mass() const;
        void set_mass(decimal);

        decimal get_inv_mass() const;

        decimal get_inertia() const;
        void set_inertia(decimal);

        decimal get_inv_inertia() const;

        const vec2 &get_centroid() const;
        void set_centroid(const vec2 &);

        const vec2 &get_position() const;
        void set_position(const vec2 &);

        const mat22 &get_rotation() const;
        void set_rotation(const mat22 &);

        const vec2 &get_velocity() const;
        void set_velocity(const vec2 &);

        decimal get_angular_velocity() const;
        void set_angular_velocity(decimal);

        const vec2 &get_force() const;
        void set_force(const vec2 &);

        decimal get_torque() const;
        void set_torque(decimal);

        decimal get_friction() const;
        void set_friction(decimal);

    protected:
        DISALLOW_COPY_AND_ASSIGN(cbody)

        uint16_t _id; // 编号
        decimal _mass; // 质量
        decimal _inv_mass; // 质量倒数（缓存，用于运算）
        decimal _inertia; // 转动惯量
        decimal _inv_inertia; // 转动惯性倒数（缓存，用于运算）
        vec2 _centroid; // 重心
        vec2 _position; // 位置
        mat22 _rotation{mat22::I}; // 旋转矩阵
        vec2 _velocity; // 速度
        decimal _angular_velocity{0}; // 角速度
        vec2 _force; // 受力
        decimal _torque{0}; // 扭矩
        decimal _friction{1}; // 摩擦力
    };

    // 几何刚体
    class polygon_body : public cbody {
    public:
        using ptr = std::shared_ptr<polygon_body>;

        polygon_body(uint16_t id, decimal mass, const vertex_list& vertices);

        size_t count() const;

        vec2 operator[](size_t idx) const;
        vec2 edge(size_t idx) const;

        decimal min_separating_axis(size_t& idx, const polygon_body& other) const;

    protected:
        DISALLOW_COPY_AND_ASSIGN(polygon_body)

        vertex_list _vertices;
    };
}

#endif //_CGFLUID_CBODYH

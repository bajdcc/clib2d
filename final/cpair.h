//
// Project: clib2d
// Created by bajdcc
//

#ifndef CGFLUID_CPAIR_H
#define CGFLUID_CPAIR_H

#include <memory>
#include "ctypes.h"
#include "cbody.h"

namespace clib {

    // 接触点
    struct ccontact {
        vec2 position; // 位置
        vec2 ra, rb;
        std::array<bool, 2> from_a;
        std::array<size_t, 2> indices;
        decimal separation;
        decimal pn{0};
        decimal pt{0};
        decimal bias{0};
        decimal mass_normal{0};
        decimal mass_tangent{0};

        ccontact(const polygon_body &b, size_t idx);

        bool operator==(const ccontact &other) const;
        bool operator!=(const ccontact &other) const;
    };

    // 碰撞检测对
    class cpair {
    public:
        using contact_list = std::vector<ccontact>;
        using ptr = std::shared_ptr<cpair>;

        cpair(cbody::ptr a, cbody::ptr b, const vec2 &normal, const contact_list &contacts);

        const contact_list& get_contacts() const;
        const vec2& get_normal() const;

        void pre_step(decimal dt);
        void update_impulse();
        void update(const cpair &old_arbiter);

        void add_contact(const ccontact &contact);

        static ptr is_collide(polygon_body::ptr &pa, polygon_body::ptr &pb, uint32_t &id);

    private:
        std::weak_ptr<cbody> _a, _b; // 参与碰撞检测的两个刚体
        vec2 _normal; // 法向量
        contact_list _contacts; // 接触点列表
    };
}

#endif //CGFLUID_CPAIR_H

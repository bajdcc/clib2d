//
// Project: clib2d
// Created by bajdcc
//

#include <algorithm>
#include <random>
#include "c2dworld.h"
#include "cparser.h"
#include "csub.h"

namespace clib {

    std::chrono::system_clock::time_point c2d_world::last_clock = std::chrono::high_resolution_clock::now();
    decimal c2d_world::dt = FRAME_SPAN;
    decimal c2d_world::dt_inv = 1.0 * FPS;
    bool c2d_world::paused = false; // 是否暂停
    std::string c2d_world::title("[TITLE]"); // 标题
    c2d_world *world = nullptr;

    c2d_polygon *c2d_world::make_polygon(decimal mass, const std::vector<v2> &vertices, const v2 &pos, bool statics) {
        auto polygon = std::make_unique<c2d_polygon>(global_id++, mass, vertices);
        polygon->pos = pos;
        polygon->refresh();
        auto obj = polygon.get();
        if (statics) {
            polygon->mass.set(inf);
            polygon->statics = true;
            static_bodies.push_back(std::move(polygon));
        } else {
            bodies.push_back(std::move(polygon));
        }
        return obj;
    }

    c2d_polygon *c2d_world::make_rect(decimal mass, decimal w, decimal h, const v2 &pos, bool statics) {
        w = std::abs(w);
        h = std::abs(h);
        std::vector<v2> vertices = { // 设置四个顶点，逆时针
            {w / 2,  h / 2},
            {-w / 2, h / 2},
            {-w / 2, -h / 2},
            {w / 2,  -h / 2}
        };
        return make_polygon(mass, vertices, pos, statics);
    }

    c2d_circle *c2d_world::make_circle(decimal mass, decimal r, const v2 &pos, bool statics) {
        auto circle = std::make_unique<c2d_circle>(global_id++, mass, r);
        circle->pos = pos;
        auto obj = circle.get();
        if (statics) {
            circle->mass.set(inf);
            circle->statics = true;
            static_bodies.push_back(std::move(circle));
        } else {
            bodies.push_back(std::move(circle));
        }
        return obj;
    }

    c2d_revolute_joint *c2d_world::make_revolute_joint(c2d_body *a, c2d_body *b, const v2 &anchor) {
        auto joint = std::make_unique<c2d_revolute_joint>(a, b, anchor);
        auto obj = joint.get();
        joints.push_back(std::move(joint));
        return obj;
    }

    c2d_body *c2d_world::find_body(const v2 &pos) {
        auto body = std::find_if(bodies.begin(), bodies.end(), [&](auto &b) {
            return b->contains(pos);
        });
        if (body != bodies.end())
            return (*body).get();
        return nullptr;
    }

    uint32_t c2d_world::make_id(uint16_t a, uint16_t b) {
        return std::min(a, b) << 16 | (std::max(a, b));
    }

    bool c2d_world::collision_detection(const c2d_body::ptr &a, c2d_body::ptr &b) {
        auto bodyA = a.get();
        auto bodyB = b.get();
        auto id = make_id(bodyA->id, bodyB->id);
        auto _axis = 0;

        collision c;
        c.bodyA = bodyA;
        c.bodyB = bodyB;

        if (!AABB_collide(bodyA, bodyB) ||
            (((_axis = max_separating_axis(bodyA, bodyB, c.A)) != 1) ||
             (_axis == 2 ? true : (max_separating_axis(bodyB, bodyA, c.B) != 1)))) { // 是则不碰撞
            auto prev = collisions.find(id); // 查找下先前是否有碰撞
            if (prev != collisions.end()) { // 先前碰撞过，标记成不碰撞
                collisions.erase(prev); // 从碰撞数组中删掉
                bodyA->collision--; // 碰撞次数减一
                bodyB->collision--;
            }
            return false; // max_sa < 0 不相交
        }

        // 相交，产生碰撞
        auto prev = collisions.find(id); // 查找下先前是否有碰撞
        if (prev == collisions.end()) { // 之前没有产生过碰撞
            if (solve_collision(c)) { // 计算碰撞点
                collisions.insert(std::make_pair(id, c));
                // A和B标记成碰撞
                bodyA->collision++; // 碰撞次数加一
                bodyB->collision++;
#if ENABLE_SLEEP
                bodyA->sleep = false;
                bodyB->sleep = false;
#endif
            }
            return true;
        } else { // 先前产生过碰撞
            if (solve_collision(c)) { // 计算碰撞点
                clib::collision_update(c, collisions[id]);
                collisions[id] = c; // 替换碰撞结构
                return true;
            } else { // 没有碰撞
                collisions.erase(prev);
                bodyA->collision--; // 碰撞次数减一
                bodyB->collision--;
                return false;
            }
        }
    }

    decltype(auto) c2d_world::sleep_bodies() const {
#if ENABLE_SLEEP
        return std::count_if(bodies.begin(), bodies.end(), [&](auto &b) {
            return b->sleep;
        });
#else
        return 0;
#endif
    }

    void c2d_world::collision_detection() {
        auto size = bodies.size();
        for (size_t i = 0; i < size; i++) {
            if (bodies[i]->sleep) continue;
            for (size_t j = 0; j < size; j++) {
                if (bodies[j]->sleep || i < j)
                    collision_detection(bodies[i], bodies[j]);
            }
            for (auto &body : static_bodies) {
                collision_detection(bodies[i], body);
            }
        }
    }

    void c2d_world::draw_collision(const collision &c) {
        glColor3f(0.2f, 0.5f, 0.4f);
        // 绘制A、B经过SAT计算出来的边
        glBegin(GL_LINES);
        v2 ptA1, ptA2;
        const auto typeA = c.bodyA->type();
        const auto typeB = c.bodyB->type();
        if (!c.bodyA->statics) {
            if (typeA == C2D_POLYGON) {
                auto bodyA = dynamic_cast<c2d_polygon *>(c.bodyA);
                ptA1 = bodyA->vertex(c.A.polygon.idx);
                ptA2 = bodyA->vertex(c.A.polygon.idx + 1);
                glVertex2d(ptA1.x, ptA1.y);
                glVertex2d(ptA2.x, ptA2.y);
            }
        }
        if (!c.bodyB->statics) {
            if (typeB == C2D_POLYGON) {
                auto bodyB = dynamic_cast<c2d_polygon *>(c.bodyB);
                auto ptB1 = bodyB->vertex(c.B.polygon.idx);
                auto ptB2 = bodyB->vertex(c.B.polygon.idx + 1);
                glVertex2d(ptB1.x, ptB1.y);
                glVertex2d(ptB2.x, ptB2.y);
            }
        }
        glEnd();
        /*if (!c.bodyA->statics) {
            // 绘制A的SAT边法线
            glColor3f(0.1f, 0.4f, 0.2f);
            glBegin(GL_LINES);
            auto pt3 = (ptA1 + ptA2) / 2;
            auto pt4 = pt3 + c.N * 0.3;
            glVertex2d(pt3.x, pt3.y);
            glVertex2d(pt4.x, pt4.y);
            glEnd();
        }*/
        // 绘制接触点
        glColor3f(1.0f, 0.2f, 0.2f);
        glPointSize(2.0f);
        glBegin(GL_POINTS);
        for (auto &contact : c.contacts) {
            glVertex2d(contact.pos.x, contact.pos.y);
        }
        glEnd();
    }

    void c2d_world::collision_update(collision &c) {
        auto &a = *c.bodyA;
        auto &b = *c.bodyB;
        auto tangent = c.N.normal(); // 接触面
        for (auto &contact : c.contacts) {
            auto dv = (b.V + (-b.angleV * contact.rb.N())) -
                      (a.V + (-a.angleV * contact.ra.N()));

            // 法向力
            auto vn = dv.dot(c.N);
            auto dpn = (-vn + contact.bias) * contact.mass_normal;
            auto _pn = std::max(contact.pn + dpn, 0.0);
            dpn = _pn - contact.pn;
            contact.pn = _pn;

            a.update(gravity, 0);
            b.update(gravity, 0); // 初始化力和力矩

            auto p = dpn * c.N;
            a.impulse(-p, contact.ra);
            b.impulse(p, contact.rb);

            a.update(gravity, 1);
            b.update(gravity, 1); // 计算力和力矩，得出速度和角速度

            dv = (b.V + (-b.angleV * contact.rb.N())) -
                 (a.V + (-a.angleV * contact.ra.N()));

            // 切向力
            auto vt = dv.dot(tangent);
            auto dpt = -vt * contact.mass_tangent;
            auto friction = sqrt(a.f * b.f) * contact.pn;
            auto _pt = std::max(-friction, std::min(friction, contact.pt + dpt));
            dpt = _pt - contact.pt;
            contact.pt = _pt;

            a.update(gravity, 0);
            b.update(gravity, 0); // 初始化力和力矩

            p = dpt * tangent;
            a.impulse(-p, contact.ra);
            b.impulse(p, contact.rb);

            a.update(gravity, 1);
            b.update(gravity, 1); // 计算力和力矩，得出速度和角速度
        }
    }

#if ENABLE_SLEEP
    void c2d_world::collision_remove_sleep() {
        erase_if(collisions, [&](auto &c) {
            if (c.second.bodyA->statics)
                return c.second.bodyB->sleep;
            if (c.second.bodyB->statics)
                return c.second.bodyA->sleep;
            return c.second.bodyA->sleep && c.second.bodyB->sleep;
        });
    }
#endif

    void c2d_world::step() {
        glMatrixMode(GL_MODELVIEW); // 转换视图开始绘制
        glLoadIdentity();
        glTranslatef(0.0f, 0.0f, -10.0f);

        if (!paused) {
            if (animation_id > 0)
                run_animation();

            collision_detection();

            // 碰撞预处理
            for (auto &col : collisions) {
                collision_prepare(col.second);
            }

            // 关节预处理
            for (auto &joint : joints) {
                joint->prepare(gravity);
            }

            for (auto &body : bodies)
                body->update(gravity, 4); // 合外力累计清零

            // 迭代十次，防止穿越
            for (auto i = 0; i < COLLISION_ITERATIONS; ++i) {

                // 碰撞处理
                for (auto &col : collisions) {
                    collision_update(col.second);
                }

                // 关节处理
                for (auto &joint : joints) {
                    joint->update(gravity);
                }
            }

            for (auto &body : bodies) {
                body->update(gravity, 0); // 初始化力和力矩
                body->update(gravity, 3); // 添加重力
                body->update(gravity, 1); // 计算力和力矩，得出速度和角速度
                body->update(gravity, 2); // 计算位移和角度
                body->update(gravity, 5); // 判定休眠
            }
        }

#if ENABLE_SLEEP
        collision_remove_sleep();
#endif

        for (auto &body : static_bodies) {
            body->draw();
        }
        for (auto &body : bodies) {
            body->draw();
        }
        for (auto &col : collisions) {
            draw_collision(col.second);
        }
        for (auto &joint : joints) {
            joint->draw();
        }

        if (mouse_drag) {
            glLineWidth(1.0f);
            glColor3f(0.6f, 0.6f, 0.6f);
            glBegin(GL_LINES);
            glVertex2d(global_drag.x, global_drag.y);
            glVertex2d(global_drag.x + global_drag_offset.x, global_drag.y + global_drag_offset.y);
            glEnd();
            glColor3f(0.9f, 0.7f, 0.4f);
            glPointSize(4.0f);
            glBegin(GL_POINTS);
            glVertex2d(global_drag.x, global_drag.y);
            glVertex2d(global_drag.x + global_drag_offset.x, global_drag.y + global_drag_offset.y);
            glEnd();
        }
    }

    void c2d_world::move(const v2 &v) {
        for (auto &body : bodies) {
#if ENABLE_SLEEP
            body->sleep = false;
#endif
            body->V += v;
        }
    }

    void c2d_world::rotate(decimal d) {
        for (auto &body : bodies) {
#if ENABLE_SLEEP
            body->sleep = false;
#endif
            body->angleV += d;
        }
    }

    void c2d_world::offset(const v2 &pt, const v2 &offset) {
        auto body = find_body(pt);
        if (body) {
#if ENABLE_SLEEP
            body->sleep = false;
#endif
            body->drag(pt, offset * body->mass.value);
        }
    }

    void c2d_world::mouse(const v2 &pt, bool down) {
        if (down) {
            mouse_drag = true;
            global_drag = pt;
            global_drag_offset.x = 0;
            global_drag_offset.y = 0;
        } else {
            mouse_drag = false;
            global_drag_offset.x = (pt.x - global_drag.x);
            global_drag_offset.y = (pt.y - global_drag.y);
            offset(global_drag, global_drag_offset);
            global_drag.x = pt.x;
            global_drag.y = pt.y;
        }
    }

    void c2d_world::motion(const v2 &pt) {
        if (mouse_drag) {
            global_drag_offset.x = (pt.x - global_drag.x);
            global_drag_offset.y = (pt.y - global_drag.y);
        }
    }

    void c2d_world::clear() {
        stop_animation();
        global_id = 1;
        bodies.clear();
        static_bodies.clear();
        collisions.clear();
        joints.clear();
    }

    void c2d_world::make_bound() {
        make_rect(inf, 10, 0.1, {0, 3}, true)->f = 0.8;
        make_rect(inf, 10, 0.1, {0, -3}, true)->f = 0.8;
        make_rect(inf, 0.1, 6, {5, 0}, true)->f = 0.8;
        make_rect(inf, 0.1, 6, {-5, 0}, true)->f = 0.8;
    }

    void c2d_world::scene(int id) {
        clear();
        switch (id) {
            case 1: { // 一矩形、两三角形
                title = "[SCENE 1] One rectangle and two triangles";
                make_bound();
                std::vector<v2> vertices = {
                    {-0.5, 0},
                    {0.5,  0},
                    {0,    0.5}
                };
                make_polygon(2, vertices, {-0.5, -2.9})->f = 0.2;
                make_polygon(2, vertices, {0.5, -2.9})->f = 0.2;
                make_rect(2, 1.2, 2, {0, 1.5})->f = 0.2;
            }
                break;
            case 2: { // 堆叠的方块
                title = "[SCENE 2] Rectangle stack";
                make_bound();
                std::default_random_engine e((uint32_t) time(nullptr));
                std::normal_distribution<decimal> dist{-0.1, 0.1};
                for (auto i = 0; i < 10; ++i) {
                    auto x = dist(e);
                    auto body = make_rect(1, 0.5, 0.4, {x, -2.6 + 0.4 * i});
                    body->f = 0.2;
                }
            }
                break;
            case 3: { // 金字塔
                title = "[SCENE 3] Rectangle pyramid";
                make_bound();
                //make_circle(10, 0.4, {-0.15, 2})->f = 0.2;
                v2 x{-2.0, -2.4};
                v2 y;
                int n = 10;
                for (auto i = 0; i < n; ++i) {
                    y = x;
                    for (auto j = i; j < n; ++j) {
                        make_rect(1, 0.4, 0.4, y)->f = 0.2;
                        y += {0.41, 0.0};
                    }
                    x += {0.205, 0.41};
                }
            }
                break;
            case 4: { // 牛顿摆
                title = "[SCENE 4] Newton's cradle";
                auto ground = make_rect(inf, 10, 0.1, {0, -3}, true);
                auto box1 = make_rect(100, 0.5, 0.5, {5.75, 3});
                box1->CO = 0.99;
                make_revolute_joint(ground, box1, {1.75, 3});
                for (size_t i = 0; i < 6; ++i) {
                    auto box2 = make_rect(100, 0.5, 0.5, {1.25 - i * 0.500001, -1});
                    box2->CO = 0.99;
                    make_revolute_joint(ground, box2, {1.25 - i * 0.500001, 3});
                }
            }
                break;
            case 5: { // 铰链
                title = "[SCENE 5] Joints";
                auto ground = make_rect(1, 10, 0.1, {0, -3}, true);
                ground->f = 0.8;
                const auto mass = 10.0;
                const auto y = 3.0;
                auto last = ground;
                for (int i = 0; i < 14; ++i) {
                    auto box = make_rect(mass, 0.4, 0.1, {0.2 + 0.5 * i, y});
                    box->f = 0.4;
                    make_revolute_joint(last, box, {0.5 * i, y});
                    last = box;
                }
            }
                break;
            case 6: { // 金字塔（圆与多边形）
                title = "[SCENE 6] Rectangle and circle pyramid";
                make_bound();
                v2 x{-2.0, -2.4};
                v2 y;
                int n = 10;
                std::default_random_engine e((uint32_t) time(nullptr));
                std::uniform_real_distribution<decimal> dist{0.15, 0.2};
                std::uniform_int_distribution<int> dist2{0, 4};
                for (auto i = 0; i < n; ++i) {
                    y = x;
                    for (auto j = i; j < n; ++j) {
                        switch (dist2(e)) {
                            case 1:
                                make_rect(1, 0.4, 0.4, y)->f = 0.2;
                                break;
                            case 2: {
                                static const auto sqrt_1_3 = 1 / std::sqrt(3);
                                static const std::vector<v2> vertices = {
                                    {0.2,  -0.2 * sqrt_1_3},
                                    {0,    0.4 * sqrt_1_3},
                                    {-0.2, -0.2 * sqrt_1_3}
                                };
                                make_polygon(1, vertices, y)->f = 0.2;
                            }
                                break;
                            case 3: {
                                static const auto sqrt_3 = std::sqrt(3);
                                static const std::vector<v2> vertices = {
                                    {0.2,  0},
                                    {0.1,  0.1 * sqrt_3},
                                    {-0.1, 0.1 * sqrt_3},
                                    {-0.2, 0},
                                    {-0.1, -0.1 * sqrt_3},
                                    {0.1,  -0.1 * sqrt_3},
                                };
                                make_polygon(1, vertices, y)->f = 0.2;
                            }
                                break;
                            default:
                                make_circle(1, dist(e), y)->f = 0.2;
                                break;
                        }
                        y += {0.41, 0.0};
                    }
                    x += {0.205, 0.41};
                }
            }
                break;
            case 7: { // 歌词
                title = "[SCENE 7] Font animation";
                make_bound();
                start_animation(1);
            }
                break;
            default: {
                title = "[SCENE DEFAULT] Rectangle, triangle and circle";
                make_bound();
                make_rect(1, 1, 1, {0, 0})->f = 0.2;
                make_circle(1, 0.5, {1, 0})->f = 0.2;
                static std::vector<v2> vertices = {
                    {0, 0},
                    {1, 0},
                    {0, 1}
                };
                make_polygon(1, vertices, {0, 1})->f = 0.2;
            }
                break;
        }
    }

    void c2d_world::init() {
        scene(0);
    }

    size_t c2d_world::get_collision_size() const {
        return collisions.size();
    }

    size_t c2d_world::get_sleeping_size() const {
        return sleep_bodies();
    }

    void c2d_world::invert_gravity() {
        gravity.y = gravity.y < 0 ? 0 : GRAVITY;
        for (auto &body : bodies) {
#if ENABLE_SLEEP
            body->sleep = false;
#endif
        }
    }

    void c2d_world::start_animation(uint32_t id) {
        if (animation_id != id) {
            if (id == 1) {
                animation_code = "box `(pos 0.5d 0.5d) `(size 0.4d 0.5d) `(mass 1d)";
                vm.reset();
                try {
                    vm.save();
                    parser = new cparser(animation_code);
                    auto node = parser->parse();
                    vm.prepare(node);
                } catch (const std::exception &e) {
                    printf("RUNTIME ERROR: %s\n", e.what());
                    vm.restore();
                    vm.gc();
                    root = nullptr;
                    delete parser;
                    parser = nullptr;
                    return;
                }
            }
            animation_id = id;
        }
    }

    void c2d_world::stop_animation() {
        if (animation_id != 0) {
            animation_id = 0;
        }
    }

    void c2d_world::run_animation() {
        try {
            if (vm.run(1) != nullptr) {
                vm.gc();
                stop_animation();
            }
        } catch (const std::exception &e) {
            printf("RUNTIME ERROR: %s\n", e.what());
            vm.restore();
            vm.gc();
        }
    }
}
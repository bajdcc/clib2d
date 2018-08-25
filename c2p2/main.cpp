//
// Project: clib2d
// Created by bajdcc
//

#include <GL/freeglut.h>
#include <chrono>
#include <cassert>
#include <cstdio>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <unordered_map>

#define FPS 30
#define GRAVITY -0.2
#define FRAME_SPAN (1.0 / FPS)

static auto last_clock = std::chrono::high_resolution_clock::now();
static auto dt = FRAME_SPAN;
static auto paused = false; // 是否暂停

// -------------------------------------------------
// 以下为物理引擎部分

using decimal = double; // 浮点类型
static const auto inf = std::numeric_limits<decimal>::infinity();

// 二维向量
struct v2 {
    decimal x{0}, y{0}; // X、Y坐标

    // 构造函数
    v2() = default;

    v2(decimal _x, decimal _y) : x(_x), y(_y) {}

    v2(const v2 &v) = default;

    v2 &operator=(const v2 &v) = default;

    inline v2 operator*(decimal d) const {
        return v2{x * d, y * d};
    }

    inline v2 operator/(decimal d) const {
        return v2{x / d, y / d};
    }

    inline v2 operator+(const v2 &v) const {
        return v2{x + v.x, y + v.y};
    }

    inline v2 operator-(const v2 &v) const {
        return v2(x - v.x, y - v.y);
    }

    inline v2 &operator+=(const v2 &v) {
        x += v.x;
        y += v.y;
        return *this;
    }

    friend inline v2 operator*(decimal d, const v2 &v) {
        return v2{d * v.x, d * v.y};
    }

    // 叉乘
    decimal cross(const v2 &v) const {
        return x * v.y - y * v.x;
    }

    // 点乘
    decimal dot(const v2 &v) const {
        return x * v.x + y * v.y;
    }

    decimal magnitude() const {
        return std::sqrt(x * x + y * y);
    }

    v2 normalize() const {
        return *this / magnitude();
    }

    // 法线向量
    v2 normal() const {
        return v2(y, -x).normalize();
    }
};

// 二维矩阵
struct m2 {
    decimal x1{1}, y1{0}, x2{0}, y2{1};

    m2() = default;

    m2(decimal _x1, decimal _y1, decimal _x2, decimal _y2) : x1(_x1), y1(_y1), x2(_x2), y2(_y2) {}

    m2(const m2 &m) = default;

    m2 &operator=(const m2 &m) = default;

    void rotate(decimal theta) {
        const auto _sin = std::sin(theta);
        const auto _cos = std::cos(theta);
        *this = m2{_cos, -_sin, _sin, _cos};
    }

    v2 rotate(const v2 &v) const {
        return v2{x1 * v.x + y1 * v.y, x2 * v.x + y2 * v.y};
    }
};

v2 gravity{0, GRAVITY}; // 重力

// 刚体基类，由于必然要多态，因此不能用struct
// 该类为动态创建，所以要用unique_ptr承担内存管理
class c2d_body {
public:
    using ptr = std::unique_ptr<c2d_body>;

    c2d_body(uint16_t _id, decimal _mass) : id(_id), mass(_mass) {}

    c2d_body(const c2d_body &) = delete; // 禁止拷贝
    c2d_body &operator=(const c2d_body &) = delete; // 禁止赋值

    virtual void drag(const v2 &pt, const v2 &offset) = 0;  // 拖动，施加力矩
    virtual bool contains(const v2 &pt) = 0;  // 是否包含该世界坐标

    // 分阶段
    // i=0，重置外力
    // i=1，第一阶段：计算速度、角速度
    // i=2，第二阶段，计算位置等其他量
    virtual void update(int) = 0; // 状态更新
    virtual void draw() = 0; // 绘制

    // 不想写那么多get/set，先public用着
    bool sleep{false}; // 是否休眠
    bool statics{false}; // 是否为静态物体
    int collision{0}; // 参与碰撞的次数
    uint16_t id{0}; // ID
    decimal mass{0}; // 质量
    v2 pos; // 位置（世界坐标，下面未注明均为本地坐标）
    v2 center; // 重心
    v2 V; // 速度
    decimal angle{0}; // 角度
    decimal angleV{0}; // 角速度
    decimal inertia{0}; // 转动惯量
    m2 R; // 旋转矩阵
    v2 F; // 受力
};

// 多边形刚体（仅支持凸多边形，且点集为有序排列）
class c2d_polygon : public c2d_body {
public:
    c2d_polygon(uint16_t _id, decimal _mass, const std::vector<v2> &_vertices)
        : c2d_body(_id, _mass), vertices(_vertices), verticesWorld(_vertices) {
        init();
    }

    // 计算多边形面积
    static decimal calc_polygon_area(const std::vector<v2> &vertices) {
        decimal area = 0;
        auto size = vertices.size();
        // 求所有三角形的面积之和
        for (size_t i = 0; i < size; ++i) {
            auto j = (i + 1) % size;
            // 叉乘求两相邻向量所成平行四边形面积
            // 所以要求三角形面积就要除以2
            area += vertices[i].cross(vertices[j]);
        }
        return area / 2;
    }

    // 计算多边形重心
    static v2 calc_polygon_centroid(const std::vector<v2> &vertices) {
        v2 gc;
        auto size = vertices.size();
        // 重心 = (各三角形重心 * 其面积) / 总面积
        // 三角形重心 = 两向量之和 / 3
        for (size_t i = 0; i < size; ++i) {
            auto j = (i + 1) % size;
            gc += (vertices[i] + vertices[j]) * vertices[i].cross(vertices[j]);
        }
        return gc / 6.0 / calc_polygon_area(vertices);
    }

    // 计算多边形转动惯量
    static decimal calc_polygon_inertia(decimal mass, const std::vector<v2> &vertices) {
        decimal acc0 = 0, acc1 = 0;
        auto size = vertices.size();
        // 转动惯量 = m / 6 * (各三角形面积 * 其(a*a+a*b+b*b)) / (总面积)
        for (size_t i = 0; i < size; ++i) {
            auto a = vertices[i], b = vertices[(i + 1) % size];
            auto _cross = std::abs(a.cross(b));
            acc0 += _cross * (a.dot(a) + b.dot(b) + a.dot(b));
            acc1 += _cross;
        }
        return mass * acc0 / 6 / acc1;
    }

    // 计算边界（矩形包围）
    void calc_bounds() {
        boundMin = boundMax = vertex(0);
        for (size_t i = 1; i < verticesWorld.size(); ++i) {
            boundMin.x = std::min(boundMin.x, vertex(i).x);
            boundMin.y = std::min(boundMin.y, vertex(i).y);
            boundMax.x = std::max(boundMax.x, vertex(i).x);
            boundMax.y = std::max(boundMax.y, vertex(i).y);
        }
    }

    // 判断在边界内
    bool contains_in_bound(const v2 &pt) {
        return boundMin.x < pt.x &&
               boundMax.x > pt.x &&
               boundMin.y < pt.y &&
               boundMax.y > pt.y;
    }

    // 判断在多边形内（判断连线向量方向的一致性，要求凸包、顶点连续、逆时针排列）
    bool contains_in_polygon(const v2 &pt) {
        const auto size = verticesWorld.size();
        if (size < 3) return false;
        if ((pt - vertex(0)).cross(vertex(1) - vertex(0)) > 0)
            return false;
        if ((pt - vertex(0)).cross(vertex(size - 1) - vertex(0)) < 0)
            return false;

        // 判断剩下的连线方向的一致性
        size_t i = 2, j = size - 1;
        auto line = SIZE_MAX;

        // 二分法
        while (i <= j) {
            auto mid = (i + j) >> 1;
            if ((pt - vertex(0)).cross(vertex(mid) - vertex(0)) > 0) {
                line = mid;
                j = mid - 1;
            } else i = mid + 1;
        }
        return (pt - vertex(line - 1)).cross(vertex(line) - vertex(line - 1)) < 0;
    }

    bool contains(const v2 &pt) override {
        // 先快速判断是否在包围框（外包矩形）中
        // 是则具体判断在多边形中（这步计算大）
        return contains_in_bound(pt) && contains_in_polygon(pt);
    }

    void init() {
        inertia = calc_polygon_inertia(mass, vertices);
        center = calc_polygon_centroid(vertices);
        refresh();
    }

    void refresh() {
        R.rotate(angle);
        for (size_t i = 0; i < edges(); ++i) {
            auto v = R.rotate(vertices[i] - center) + center;
            vertex(i) = pos + v; // 本地坐标转换为世界坐标
        }
        calc_bounds();
    }

    void update(int n) override {
        switch (n) {
            case 0:
                pass0();
                break;
            case 1:
                pass1();
                break;
            case 2:
                pass2();
                break;
            default:
                break;
        }
    }

    void pass0() {
        F = gravity * mass;
    }

    void pass1() {
        V += F / mass * dt;
    }

    void pass2() {
        pos += V * dt;
        angle += angleV * dt;
        R.rotate(angle);
        for (size_t i = 0; i < edges(); ++i) {
            auto v = R.rotate(vertices[i] - center) + center;
            vertex(i) = pos + v; // 本地坐标转换为世界坐标
        }
        calc_bounds();
    }

    // 拖拽物体
    void drag(const v2 &pt, const v2 &offset) override {
        V += 1.0 / mass * offset;
        angleV += 1.0 / inertia * (pt - pos - center).cross(offset);
    }

    void draw() override {
        if (statics) { // 画静态物体
            glColor3f(0.9f, 0.9f, 0.9f);
            glBegin(GL_LINE_LOOP);
            for (auto &v : verticesWorld) {
                glVertex2d(v.x, v.y);
            }
            glEnd();
            return;
        }
        // 开启反走样
        glEnable(GL_BLEND);
        glEnable(GL_LINE_SMOOTH);
        glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor3f(0.12f, 0.12f, 0.12f); // 坑：所有设置要放begin之前，切记！
        glBegin(GL_LINE_LOOP);
        glVertex2d(boundMin.x, boundMin.y);
        glVertex2d(boundMin.x, boundMax.y);
        glVertex2d(boundMax.x, boundMax.y);
        glVertex2d(boundMax.x, boundMin.y);
        glEnd();
        if (collision > 0)
            glColor3f(0.8f, 0.2f, 0.4f);
        else
            glColor3f(0.8f, 0.8f, 0.0f);
        glBegin(GL_LINE_LOOP);
        for (auto &v : verticesWorld) {
            glVertex2d(v.x, v.y);
        }
        glEnd();
        // 这里默认物体是中心对称的，重心就是中心，后面会计算重心
        auto p = pos + center;
        auto v = p + V * 0.2;
        glLineWidth(0.6f);
        glColor3f(0.8f, 0.2f, 0.2f);
        glBegin(GL_LINES);
        glVertex2d(p.x, p.y);
        glVertex2d(p.x + F.x * 0.8, p.y + F.y * 0.8); // 力向量
        glEnd();
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex2d(p.x, p.y);
        glVertex2d(v.x, v.y); // 速度向量
        glEnd();
        glColor3f(0.2f, 0.2f, 0.2f);
        glBegin(GL_LINES);
        glVertex2d(p.x, p.y);
        glVertex2d(p.x + R.x1 * 0.2, p.y + R.x2 * 0.2); // 方向向量
        glEnd();
        glColor3f(0.0f, 1.0f, 0.0f);
        glPointSize(3.0f);
        glBegin(GL_POINTS);
        glVertex2d(p.x, p.y); // 中心
        glEnd();
        glDisable(GL_BLEND);
        glDisable(GL_LINE_SMOOTH);
        glLineWidth(1.0f);
    }

    // 以idx为起点，下一顶点为终点的向量
    v2 edge(size_t idx) const {
        return verticesWorld[(idx + 1) % verticesWorld.size()] - verticesWorld[idx];
    }

    v2 &vertex(size_t idx) {
        return verticesWorld[idx % verticesWorld.size()];
    }

    size_t edges() const {
        return verticesWorld.size();
    }

    std::vector<v2> vertices; // 多边形的顶点（本地坐标）
    std::vector<v2> verticesWorld; // 多边形的顶点（世界坐标）
    v2 boundMin, boundMax; // 外包矩形
};

static std::vector<c2d_body::ptr> bodies; // 寻常物体
static std::vector<c2d_body::ptr> static_bodies; // 静态物体
static uint16_t global_id = 1;

static bool mouse_drag = false;
static v2 global_drag; // 鼠标拖动
static v2 global_drag_offset; // 鼠标拖动位移

static c2d_polygon *make_polygon(decimal mass, const std::vector<v2> &vertices, const v2 &pos, bool statics = false) {
    auto polygon = std::make_unique<c2d_polygon>(global_id++, mass, vertices);
    polygon->pos = pos;
    polygon->refresh();
    auto obj = polygon.get();
    if (statics) {
        polygon->statics = true;
        static_bodies.push_back(std::move(polygon));
    } else {
        bodies.push_back(std::move(polygon));
    }
    return obj;
}

static c2d_polygon *make_rect(decimal mass, decimal w, decimal h, const v2 &pos, bool statics = false) {
    std::vector<v2> vertices = { // 设置四个顶点，逆时针
        {w / 2,  h / 2},
        {-w / 2, h / 2},
        {-w / 2, -h / 2},
        {w / 2,  -h / 2}
    };
    return make_polygon(mass, vertices, pos, statics);
}

// 根据位置找到物体
c2d_body *find_body(const v2 &pos) {
    auto body = std::find_if(bodies.begin(), bodies.end(), [&](auto &b) {
        return b->contains(pos);
    });
    if (body != bodies.end())
        return (*body).get();
    return nullptr;
}

// 接触点
struct contact {
    v2 pos; // 位置

    contact(v2 _pos) : pos(_pos) {}
};

// 碰撞结构
struct collision {
    std::vector<contact> contacts; // 接触点列表
    c2d_body *bodyA{nullptr}, *bodyB{nullptr}; // 碰撞的两个物体
    size_t idxA{0}, idxB{0}; // 碰撞的两个物体的轴
    decimal satA{0}, satB{0}; // 碰撞的两个SAT
    v2 N; // 法线
};

// uint32_t 是由 a、b 两个物体的 id 组成
std::unordered_map<uint32_t, collision> collisions; // 碰撞情况

uint32_t make_id(uint16_t a, uint16_t b) {
    return std::min(a, b) << 16 | (std::max(a, b));
}

// 碰撞检测-SAT分离轴定理
// 检测两凸包是否相交
// 表述：如果两个凸多边形没有相交，那么存在这两个物体在一个轴上的投影不重叠。
// 轴：只需采用两个凸包的每个条做检测即可
// 只要最大间隙大于零，即为不相交
// separation：最大间隙
// idx：最大间隙的轴
bool max_separating_axis(c2d_polygon *a, c2d_polygon *b,
                         decimal &separation, size_t &idx) {
    separation = -inf;
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
        if (min_sep > separation) {
            separation = min_sep; // 寻找最大间隙
            idx = i; // 轴
        }
    }
    return separation > 0; // 是则不相交
}

// 先用包围盒方法快速判断碰撞
bool AABB_collide(c2d_polygon *a, c2d_polygon *b) {
    auto centerA = (a->boundMax + a->boundMin) / 2; // 矩形包围圈中心点
    auto centerB = (b->boundMax + b->boundMin) / 2;
    auto sizeA = (a->boundMax - a->boundMin) / 2; // 矩形包围圈大小的二分之一
    auto sizeB = (b->boundMax - b->boundMin) / 2;
    return std::abs(centerB.x - centerA.x) <= (sizeA.x + sizeB.x) &&
           std::abs(centerB.y - centerA.y) <= (sizeA.y + sizeB.y);
}

static size_t incident_edge(const v2 &N, c2d_polygon *body) {
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

// Sutherland-Hodgman（多边形裁剪）
size_t clip(std::vector<contact> &out,
          const std::vector<contact> &in,
          size_t i,
          const v2 &p1, const v2 &p2) {
    size_t num_out = 0;
    auto N = (p2 - p1).normal();
    // 计算投影
    auto dist0 = N.dot(in[0].pos - p1);
    auto dist1 = N.dot(in[1].pos - p1);

    // 如果投影都小于零，则B中两点都在A内
    if (dist0 <= 0) out[num_out++] = in[0].pos;
    if (dist1 <= 0) out[num_out++] = in[1].pos;

    // 否则两点一个在A内，一个在A外
    if (dist0 * dist1 < 0) {
        // 计算比率
        auto interp = dist0 / (dist0 - dist1);
        // 计算p1,p2与in1,in2交点
        out[num_out++] = in[0].pos + interp * (in[1].pos - in[0].pos);
    }

    return num_out;
}

// 计算碰撞（返回是否碰撞）
bool solve_collision(collision &c) {
    if (c.satA < c.satB) { // 排列：A比B的SAT更大，更接近零
        std::swap(c.bodyA, c.bodyB);
        std::swap(c.idxA, c.idxB);
        std::swap(c.satA, c.satB);
    }
    auto bodyA = dynamic_cast<c2d_polygon *>(c.bodyA);
    auto bodyB = dynamic_cast<c2d_polygon *>(c.bodyB);
    // 计算SAT的轴法线
    // edge = A物体离B物体最近的边
    // N = edge的法线，指向B物体
    c.N = bodyA->edge(c.idxA).normal();
    // 此时要找到B物体中离A物体最近的边
    c.idxB = incident_edge(c.N, bodyB);

    decltype(c.contacts) contacts;
    // 假定两个接触点（即idxB两端点）
    contacts.emplace_back(bodyB->vertex(c.idxB));
    contacts.emplace_back(bodyB->vertex(c.idxB + 1));
    auto tmp = contacts;

    // 将idxB线段按bodyA进行多边形裁剪
    for (size_t i = 0; i < bodyA->edges(); ++i) {
        if (i == c.idxA)
            continue;
        if (clip(tmp, contacts, i, bodyA->vertex(i), bodyA->vertex(i + 1)) < 2)
            return false;
        contacts = tmp;
    }
    // 最后才裁剪idxA边
    auto va = bodyA->vertex(c.idxA);
    if (clip(tmp, contacts, c.idxA, va, bodyA->vertex(c.idxA + 1)) < 2)
        return false;
    contacts = tmp;

    // 筛选交点
    for (auto &contact : contacts) {
        auto sep = (contact.pos - va).dot(c.N);
        if (sep <= 0) { // 找在idxA向bodyA一侧的（bodyA内的接触点）
            c.contacts.push_back(contact);
        }
    }

    return true;
}

// 两物体碰撞检测
void collision_detection(const c2d_body::ptr &a, c2d_body::ptr &b) {
    auto bodyA = dynamic_cast<c2d_polygon *>(a.get());
    auto bodyB = dynamic_cast<c2d_polygon *>(b.get());
    decimal satA, satB;
    size_t idxA, idxB;
    auto id = make_id(bodyA->id, bodyB->id);

    if (!AABB_collide(bodyA, bodyB) ||
        (max_separating_axis(bodyA, bodyB, satA, idxA) ||
         max_separating_axis(bodyB, bodyA, satB, idxB))) { // 是则不碰撞
        auto prev = collisions.find(id); // 查找下先前是否有碰撞
        if (prev != collisions.end()) { // 先前碰撞过，标记成不碰撞
            collisions.erase(prev); // 从碰撞数组中删掉
            bodyA->collision--; // 碰撞次数减一
            bodyB->collision--;
        }
        return; // max_sa < 0 不相交
    }
    // 相交，产生碰撞
    auto prev = collisions.find(id); // 查找下先前是否有碰撞
    if (prev == collisions.end()) { // 之前没有产生过碰撞
        collision c; // 新建碰撞结构
        c.bodyA = bodyA;
        c.bodyB = bodyB;
        c.idxA = idxA;
        c.idxB = idxB;
        c.satA = satA;
        c.satB = satB;
        if (solve_collision(c)) { // 计算碰撞点
            collisions.insert(std::make_pair(id, c));
            // A和B标记成碰撞
            bodyA->collision++; // 碰撞次数加一
            bodyB->collision++;
        }
    } else { // 先前产生过碰撞
        // 先空着
        collision c; // 新建碰撞结构
        c.bodyA = bodyA;
        c.bodyB = bodyB;
        c.idxA = idxA;
        c.idxB = idxB;
        c.satA = satA;
        c.satB = satB;
        if (solve_collision(c)) { // 计算碰撞点
            collisions[id] = c;
        } else { // 没有碰撞
            collisions.erase(prev);
            bodyA->collision--; // 碰撞次数减一
            bodyB->collision--;
        }
    }
}

// 碰撞检测
void collision_detection() {
    auto size = bodies.size();
    for (size_t i = 0; i < size; i++) {
        if (bodies[i]->sleep) // 合外力为零则为sleep
            continue;
        for (size_t j = i + 1; j < size; j++) { // 避免重复
            collision_detection(bodies[i], bodies[j]);
        }
        for (auto &body:static_bodies) {
            collision_detection(bodies[i], body);
        }
    }
}

// 绘制碰撞情况
void draw_collision(collision &c) {
    auto bodyA = dynamic_cast<c2d_polygon *>(c.bodyA);
    auto bodyB = dynamic_cast<c2d_polygon *>(c.bodyB);
    glColor3f(0.2f, 0.5f, 0.4f);
    // 绘制A、B经过SAT计算出来的边
    glBegin(GL_LINES);
    v2 ptA1, ptA2;
    if (!c.bodyA->statics) {
        ptA1 = bodyA->vertex(c.idxA);
        ptA2 = bodyA->vertex(c.idxA + 1);
        glVertex2d(ptA1.x, ptA1.y);
        glVertex2d(ptA2.x, ptA2.y);
    }
    if (!c.bodyB->statics) {
        auto ptB1 = bodyB->vertex(c.idxB);
        auto ptB2 = bodyB->vertex(c.idxB + 1);
        glVertex2d(ptB1.x, ptB1.y);
        glVertex2d(ptB2.x, ptB2.y);
    }
    glEnd();
    if (!c.bodyA->statics) {
        // 绘制A的SAT边法线
        glColor3f(0.1f, 0.4f, 0.2f);
        glBegin(GL_LINES);
        auto pt3 = (ptA1 + ptA2) / 2;
        auto pt4 = pt3 + c.N * 0.3;
        glVertex2d(pt3.x, pt3.y);
        glVertex2d(pt4.x, pt4.y);
        glEnd();
    }
    // 绘制接触点
    glColor3f(1.0f, 0.2f, 0.2f);
    glPointSize(2.0f);
    glBegin(GL_POINTS);
    for (auto &contact : c.contacts) {
        glVertex2d(contact.pos.x, contact.pos.y);
    }
    glEnd();
}

// 每步操作
static void c2d_step() {
    glMatrixMode(GL_MODELVIEW); // 转换视图开始绘制
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -10.0f);

    if (!paused) {
        collision_detection();

        for (auto &body : bodies)
            body->update(0);
        for (auto &body : bodies)
            body->update(1);
        for (auto &body : bodies)
            body->update(2);
    }

    for (auto &body : static_bodies) {
        body->draw();
    }
    for (auto &body : bodies) {
        body->draw();
    }
    for (auto &col : collisions) {
        draw_collision(col.second);
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

// 移动（调试）
void c2d_move(const v2 &v) {
    for (auto &body : bodies) {
        body->V += v;
    }
}

// 旋转（调试）
void c2d_rotate(decimal d) {
    for (auto &body : bodies) {
        body->angleV += d;
    }
}

// 拖动（调试）
void c2d_offset(const v2 &pt, const v2 &offset) {
    auto body = find_body(pt);
    if (body) {
        body->drag(pt, offset);
    }
}

// 清除所有物体
void clear() {
    global_id = 1;
    bodies.clear();
    static_bodies.clear();
    collisions.clear();
}

// 建立四周边界
void make_bound() {
    make_rect(inf, 10, 0.1, {0, 3}, true);
    make_rect(inf, 10, 0.1, {0, -3}, true);
    make_rect(inf, 0.1, 6, {5, 0}, true);
    make_rect(inf, 0.1, 6, {-5, 0}, true);
}

// 场景
void scene(int i) {
    clear();
    make_bound();
    switch (i) {
        case 2: {
            auto a = make_rect(1, 1, 1, v2(0, 0));
            a->V = v2(0.2, 0);
            auto b = make_rect(1, 1, 1, v2(1, 0));
            b->V = v2(-0.2, 0);
        }
            break;
        case 3: {
            auto a = make_rect(1, 1, 1, v2(0, 0));
            a->V = v2(0.2, 0);
            a->angleV = 0.2;
            auto b = make_rect(1, 1, 1, v2(1, 0));
            b->V = v2(-0.2, 0);
            b->angleV = -0.2;
        }
            break;
        case 4: {
            std::vector<v2> vertices = {
                {0, 0},
                {1, 0},
                {0, 1}
            };
            auto a = make_polygon(1, vertices, v2(0, 0));
            a->angleV = 0.2;
        }
            break;
        default:
            make_rect(1, 1, 1, v2(0, 0));
            make_rect(1, 1, 1, v2(1, 0));
            break;
    }
}

// 初始化
void init() {
    scene(1);
}

// 以上为物理引擎部分
// -------------------------------------------------

/**
 * 绘制文字
 * @param x X坐标
 * @param y Y坐标
 * @param format 格式化字符串
 */
static void draw_text(int x, int y, const char *format, ...) {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    int w = glutGet(GLUT_WINDOW_WIDTH);
    int h = glutGet(GLUT_WINDOW_HEIGHT);
    gluOrtho2D(0, w, h, 0); // 正射投影，无3D透视效果，直接打到屏幕上
    // gluOrtho2D 裁剪面（最终窗口呈现的）是一个左下角点为(left,bottom)、右上角点为(right,top)的矩形
    // 这个投影跟Windows的窗口绘制一样，以左上为(left,top)，右下为(right,top)，但与数学上的直角坐标系不同！
    glMatrixMode(GL_MODELVIEW); // 为什么要添加这句话，因为在绘制物体中修改了视图
    glPushMatrix();
    glLoadIdentity();

    glColor3f(0.9f, 0.9f, 0.9f); // 文字颜色为90%白
    glRasterPos2i(x, y); // 设置文字的起始位置

    char buffer[256]; // 这里暂不做缓冲区溢出判断
    va_list args;
    va_start(args, format);
    int len = vsprintf(buffer, format, args); // 格式化字符串
    va_end(args);
    for (int i = 0; i < len; ++i) {
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, buffer[i]); // 第一个参数为字体，第二个参数为字符
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // 清空屏幕

    int h = glutGet(GLUT_WINDOW_HEIGHT); // 窗口的高
    int w = glutGet(GLUT_WINDOW_WIDTH); // 窗口的宽

    c2d_step(); // 坐标轴同直角坐标系

    // 绘制文字
    draw_text(10, 20, "clib-2d @bajdcc"); // 暂不支持中文
    draw_text(w - 110, 20, "FPS: %.1f", 1.0 / dt);
    draw_text(10, h - 20, "#c2p2");
    if (paused)
        draw_text(w / 2 - 30, 20, "PAUSED");

    glutSwapBuffers(); // 切换双缓冲
}

void reshape(int width, int height) {
    glViewport(0, 0, width, height); // 改变视口大小
    glMatrixMode(GL_PROJECTION);// 透视投影
    glLoadIdentity(); // 重置成单位矩阵
    gluPerspective(45.0, width / (float) height, 0.1, 100.0); // 透视投影
}

void keyboard(unsigned char key, int x, int y) {
    if (key >= '0' && key <= '9') {
        scene(key - '0');
    } else {
        switch (key) {
            case 27:
                glutLeaveMainLoop(); // 按ESC退出
                break;
            case ' ':
                paused = !paused;
                break;
            case 'w':
                c2d_move(v2(0, 0.1));
                break;
            case 'a':
                c2d_move(v2(-0.1, 0));
                break;
            case 's':
                c2d_move(v2(0, -0.1));
                break;
            case 'd':
                c2d_move(v2(0.1, 0));
                break;
            case 'q':
                c2d_rotate(0.1);
                break;
            case 'e':
                c2d_rotate(-0.1);
                break;
            case 'g':
                gravity.y = gravity.y < 0 ? 0 : GRAVITY;
                break;
            default:
                break;
        }
    }
}

// 屏幕坐标到世界坐标的变换
v2 screen2world(int x, int y) {
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posX, posY, posZ;

    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    winX = x;
    winY = viewport[3] - y;
    winZ = 0.9; // 0.1=近=>0   100=远=>1
    gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);

    return v2(posX, posY) * 10; // 没搞明白这个10，先用着
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            mouse_drag = true;
            global_drag = screen2world(x, y);
            global_drag_offset.x = 0;
            global_drag_offset.y = 0;
        } else {
            mouse_drag = false;
            auto pt = screen2world(x, y);
            global_drag_offset.x = (pt.x - global_drag.x);
            global_drag_offset.y = (pt.y - global_drag.y);
            c2d_offset(global_drag, global_drag_offset);
            global_drag.x = pt.x;
            global_drag.y = pt.y;
        }
    }
}

void motion(int x, int y) {
    if (mouse_drag) {
        auto pt = screen2world(x, y);
        global_drag_offset.x = (pt.x - global_drag.x);
        global_drag_offset.y = (pt.y - global_drag.y);
    }
}

void idle() {
    auto now = std::chrono::high_resolution_clock::now();
    // 计算每帧时间间隔
    dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_clock).count();

    // 锁帧
    if (dt > FRAME_SPAN) {
        last_clock = now;
        display();
    }
}

int main(int argc, char *argv[]) {
    glutInit(&argc, argv);
    glutInitWindowSize(800, 600);
    glutInitWindowPosition(50, 50);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE); // GLUT_DOUBLE 启用双缓冲，避免闪屏
    glutCreateWindow("Physics Engine -- bajdcc");
    init(); // 初始化
    glutDisplayFunc(&idle); // 绘制
    glutReshapeFunc(&reshape); // 窗口大小改变事件
    glutMouseFunc(&mouse); // 鼠标点击事件
    glutMotionFunc(&motion); // 鼠标拖动事件
    glutKeyboardFunc(&keyboard); // 键盘输入
    glutIdleFunc(&idle); // 没有事件输入时调用，这里不用它
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
    glutMainLoop(); // 主事件循环
    return 0;
}
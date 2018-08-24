//
// Project: clib2d
// Created by bajdcc
//

#include <GL/freeglut.h>
#include <chrono>
#include <cstdio>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

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
        return v2(x * d, y * d);
    }

    inline v2 operator/(decimal d) const {
        return v2(x / d, y / d);
    }

    inline v2 operator+(const v2 &v) const {
        return v2(x + v.x, y + v.y);
    }

    inline v2 operator-(const v2 &v) const {
        return v2(x - v.x, y - v.y);
    }

    inline v2 &operator+=(const v2 &v) {
        x += v.x;
        y += v.y;
        return *this;
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
        boundMin = boundMax = verticesWorld[0];
        for (size_t i = 1; i < verticesWorld.size(); ++i) {
            boundMin.x = std::min(boundMin.x, verticesWorld[i].x);
            boundMin.y = std::min(boundMin.y, verticesWorld[i].y);
            boundMax.x = std::max(boundMax.x, verticesWorld[i].x);
            boundMax.y = std::max(boundMax.y, verticesWorld[i].y);
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
        if ((pt - verticesWorld[0]).cross(verticesWorld[1] - verticesWorld[0]) > 0)
            return false;
        if ((pt - verticesWorld[0]).cross(verticesWorld[size - 1] - verticesWorld[0]) < 0)
            return false;

        // 判断剩下的连线方向的一致性
        int i = 2, j = size - 1;
        int line = -1;

        // 二分法
        while (i <= j) {
            int mid = (i + j) >> 1;
            if ((pt - verticesWorld[0]).cross(verticesWorld[mid] - verticesWorld[0]) > 0) {
                line = mid;
                j = mid - 1;
            } else i = mid + 1;
        }
        return (pt - verticesWorld[line - 1]).cross(verticesWorld[line] - verticesWorld[line - 1]) < 0;
    }

    bool contains(const v2 &pt) override {
        // 先快速判断是否在包围框（外包矩形）中
        // 是则具体判断在多边形中（这步计算大）
        return contains_in_bound(pt) && contains_in_polygon(pt);
    }

    void init() {
        inertia = calc_polygon_inertia(mass, vertices);
        center = calc_polygon_centroid(vertices);
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
        for (size_t i = 0; i < vertices.size(); ++i) {
            auto v = R.rotate(vertices[i] - center) + center;
            verticesWorld[i] = pos + v; // 本地坐标转换为世界坐标
        }
        calc_bounds();
    }

    // 拖拽物体
    void drag(const v2 &pt, const v2 &offset) override {
        angleV += 1.0 / inertia * (pt - pos - center).cross(offset);
    }

    void draw() override {
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

    std::vector<v2> vertices; // 多边形的顶点（本地坐标）
    std::vector<v2> verticesWorld; // 多边形的顶点（世界坐标）
    v2 boundMin, boundMax; // 外包矩形
};

static std::vector<c2d_body::ptr> bodies;
static uint16_t global_id = 1;

static bool mouse_drag = false;
static v2 global_drag; // 鼠标拖动
static v2 global_drag_offset; // 鼠标拖动位移

static c2d_polygon *make_polygon(decimal mass, const std::vector<v2> &vertices, const v2 &pos) {
    auto polygon = std::make_unique<c2d_polygon>(global_id++, mass, vertices);
    polygon->pos = pos;
    auto obj = polygon.get();
    bodies.push_back(std::move(polygon)); // 搬走polygon
    return obj;
}

static c2d_polygon *make_rect(decimal mass, decimal w, decimal h, const v2 &pos) {
    std::vector<v2> vertices = { // 设置四个顶点，逆时针
        {w / 2,  h / 2},
        {-w / 2, h / 2},
        {-w / 2, -h / 2},
        {w / 2,  -h / 2}
    };
    return make_polygon(mass, vertices, pos);
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

// 每步操作
static void c2d_step() {
    glMatrixMode(GL_MODELVIEW); // 转换视图开始绘制
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -10.0f);

    if (!paused) {
        for (auto &body : bodies)
            body->update(0);
        for (auto &body : bodies)
            body->update(1);
        for (auto &body : bodies)
            body->update(2);
    }

    for (auto &body : bodies) {
        body->draw();
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
}

// 场景
void scene(int i) {
    clear();
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
    draw_text(10, h - 20, "#c1p3");
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
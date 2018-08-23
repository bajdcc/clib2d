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

#define FPS 30
#define FRAME_SPAN (1.0 / FPS)

static auto last_clock = std::chrono::high_resolution_clock::now();
static auto dt = FRAME_SPAN;
static auto paused = false; // 是否暂停

// -------------------------------------------------
// 以下为物理引擎部分

using decimal = double; // 浮点类型

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

// 刚体基类，由于必然要多态，因此不能用struct
// 该类为动态创建，所以要用unique_ptr承担内存管理
class c2d_body {
public:
    using ptr = std::unique_ptr<c2d_body>;

    c2d_body(uint16_t _id, decimal _mass) : id(_id), mass(_mass) {}

    c2d_body(const c2d_body &) = delete; // 禁止拷贝
    c2d_body &operator=(const c2d_body &) = delete; // 禁止赋值

    // 分阶段，i=1，第一阶段：计算速度、角速度
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
    m2 R; // 旋转矩阵
    v2 F; // 受力
};

// 多边形刚体
class c2d_polygon : public c2d_body {
public:

    c2d_polygon(uint16_t _id, decimal _mass, const std::vector<v2> &_vertices)
        : c2d_body(_id, _mass), vertices(_vertices), verticesWorld(_vertices) {}

    void update(int n) override {
        if (n == 2) {
            pos += V * dt;
            angle += angleV * dt;
            R.rotate(angle);
            for (size_t i = 0; i < vertices.size(); ++i) {
                auto v = R.rotate(vertices[i] - center) + center;
                verticesWorld[i] = pos + v; // 本地坐标转换为世界坐标
            }
        }
    }

    void draw() override {
        glColor3f(0.8f, 0.8f, 0.0f);
        glBegin(GL_LINE_LOOP);
        for (auto &v : verticesWorld) {
            glVertex2d(v.x, v.y);
        }
        glEnd();
        // 这里默认物体是中心对称的，重心就是中心，后面会计算重心
        auto p = pos + center;
        auto v = p + V * 0.2;
        glBegin(GL_LINES);
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex2d(p.x, p.y);
        glVertex2d(v.x, v.y); // 速度向量
        glColor3f(0.2f, 0.2f, 0.2f);
        glVertex2d(p.x, p.y);
        glVertex2d(p.x + R.x1 * 0.2, p.y + R.x2 * 0.2); // 方向向量
        glEnd();
        glColor3f(0.0f, 1.0f, 0.0f);
        glPointSize(3.0f);
        glBegin(GL_POINTS);
        glVertex2d(p.x, p.y); // 中心
        glEnd();
    }

    std::vector<v2> vertices; // 多边形的顶点（本地坐标）
    std::vector<v2> verticesWorld; // 多边形的顶点（世界坐标）
};

std::vector<c2d_body::ptr> bodies;
uint16_t global_id = 1;

static c2d_polygon *make_polygon(decimal mass, const std::vector<v2> &vertices, const v2 &pos) {
    auto polygon = std::make_unique<c2d_polygon>(global_id++, mass, vertices);
    polygon->pos = pos;
    auto obj = polygon.get();
    bodies.push_back(std::move(polygon)); // 搬走polygon
    return obj;
}

static c2d_polygon *make_rect(decimal mass, decimal w, decimal h, const v2 &pos) {
    std::vector<v2> vertices = { // 设置四个顶点
        {w / 2,  h / 2},
        {-w / 2, h / 2},
        {-w / 2, -h / 2},
        {w / 2,  -h / 2}
    };
    auto polygon = std::make_unique<c2d_polygon>(global_id++, mass, vertices);
    polygon->pos = pos;
    auto obj = polygon.get();
    bodies.push_back(std::move(polygon)); // 搬走polygon
    return obj;
}

static void c2d_step() {
    glMatrixMode(GL_MODELVIEW); // 转换视图开始绘制
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -10.0f);

    if (!paused) {
        for (auto &body : bodies) {
            body->update(1);
        }
        for (auto &body : bodies) {
            body->update(2);
        }
    }

    for (auto &body : bodies) {
        body->draw();
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

    c2d_step();

    // 绘制文字
    draw_text(10, 20, "clib-2d @bajdcc"); // 暂不支持中文
    draw_text(w - 110, 20, "FPS: %.1f", 1.0 / dt);
    draw_text(10, h - 20, "#c1p2");
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

void mouse(int button, int state, int x, int y) {
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
            default:
                break;
        }
    }
}

void motion(int x, int y) {
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
//
// Project: clib2d
// Created by bajdcc
//

#include <GL/freeglut.h>
#include "c2dworld.h"

using namespace clib;

static auto &last_clock = c2d_world::last_clock;
static auto &dt =  c2d_world::dt;
static auto &dt_inv =  c2d_world::dt_inv;
static auto &paused = c2d_world::paused;
static auto &title = c2d_world::title;

// 每步操作
static void c2d_step() {
    glMatrixMode(GL_MODELVIEW); // 转换视图开始绘制
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -10.0f);

    world->step();
}

// 移动（调试）
void c2d_move(const v2 &v) {
    world->move(v);
}

// 旋转（调试）
void c2d_rotate(decimal d) {
    world->rotate(d);
}

// 拖动（调试）
void c2d_offset(const v2 &pt, const v2 &offset) {
    world->offset(pt, offset);
}

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
    draw_text(w - 110, 20, "FPS: %.1f", dt_inv);
    draw_text(10, h - 20, "#c4p2");
    draw_text(w - 290, h - 20, "Collisions: %d, Zombie: %d", world->get_collision_size(), world->get_sleeping_size());
    if (paused)
        draw_text(w / 2 - 30, 20, "PAUSED");

    draw_text(w / 2 - 200, (glutGet(GLUT_SCREEN_WIDTH) < 1920) ? 60 : 80, title.c_str());

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
        world->scene(key - '0');
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
                world->invert_gravity();
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
        world->mouse(screen2world(x, y), state == GLUT_DOWN);
    }
}

void motion(int x, int y) {
    world->motion(screen2world(x, y));
}

void idle() {
    auto now = std::chrono::high_resolution_clock::now();
    // 计算每帧时间间隔
    dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_clock).count();

    // 锁帧
    if (dt > FRAME_SPAN) {
        dt_inv = 1.0 / dt;
        last_clock = now;
        display();
    }
}

void entry(int state) {
    paused = state == GLUT_LEFT;
}

int main(int argc, char *argv[]) {
    glutInit(&argc, argv);
    if (glutGet(GLUT_SCREEN_WIDTH) < 1920) {
        glutInitWindowSize(800, 600);
        glutInitWindowPosition(50, 50);
    } else {
        glutInitWindowSize(1200, 900);
        glutInitWindowPosition(50, 50);
    }
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE); // GLUT_DOUBLE 启用双缓冲，避免闪屏
    glutCreateWindow("Physics Engine -- bajdcc");
    world = new c2d_world();
    world->init(); // 初始化
    glutDisplayFunc(&idle); // 绘制
    glutReshapeFunc(&reshape); // 窗口大小改变事件
    glutMouseFunc(&mouse); // 鼠标点击事件
    glutMotionFunc(&motion); // 鼠标拖动事件
    glutKeyboardFunc(&keyboard); // 键盘输入
    glutIdleFunc(&idle); // 没有事件输入时调用，这里不用它
    glutEntryFunc(&entry); // 没有事件输入时调用，这里不用它
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
    glutMainLoop(); // 主事件循环
    delete world;
    return 0;
}
//
// Project: clib2d
// Created by bajdcc
//

#include <GL/freeglut.h>
#include <chrono>
#include <cstdio>

#define FPS 30
#define FRAME_SPAN (1.0 / FPS)

static auto last_clock = std::chrono::high_resolution_clock::now();
static auto dt = FRAME_SPAN;

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

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
}

void init() {
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // 清空屏幕

    int h = glutGet(GLUT_WINDOW_HEIGHT); // 窗口的高
    int w = glutGet(GLUT_WINDOW_WIDTH); // 窗口的宽

    // 绘制文字
    draw_text(10, 20, "clib-2d @bajdcc"); // 暂不支持中文
    draw_text(w - 110, 20, "FPS: %.1f", 1.0 / dt);
    draw_text(10, h - 20, "#c1p1");

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
    switch (key) {
        case 27:
            glutLeaveMainLoop(); // 按ESC退出
        default:
            break;
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
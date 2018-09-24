# clib2d（C++ 2D物理引擎）

在[apollonia](https://github.com/wgtdkp/apollonia)的基础上，结合先前的[C# 2D物理引擎](https://github.com/bajdcc/PhysicsEngine)，从零开始打造一个简单的2D物理引擎。

**目前已经完成基本功能。**

## 介绍

先前所做C#版本的[PhysicsEngine](https://github.com/bajdcc/PhysicsEngine)代码翻译自JS物理引擎[matter.js](https://github.com/liabru/matter-js)。

本次借鉴OpenGL工程[apollonia](https://github.com/wgtdkp/apollonia)，两者取长补短。

代码为CMake跨平台，需要使用库：

- freeglut
- opengl32
- glu32
- ~~glm（已移除）~~

Win7上使用CLion+MinGW编译没有问题，Linux、MacOS系统下暂未尝试，64位系统需要“-m32”编译。

先前立下雄心壮志[制作简单的2D物理引擎（零）](https://www.cnblogs.com/bajdcc/p/5925837.html)，如今在一点点实现中~

----

## 使用

- 键盘数字1到5：切换经典场景
- WASD：给所有物体分别添加四个方向的力
- QE：给所有物体添加旋转
- G：切换重力
- 空格/鼠标离开：暂停

## 计划

进度与注释：

- [x] 第一部分已完成
- [x] 第二部分已完成
- [x] 第三部分已完成，单文件共**1537**行代码
- [x] **第四部分进行中**，实现圆与多边形的碰撞，单文件共**2069**行代码

分阶段最小化原则添加代码，功能由简到繁。前期的代码99%引自apollonia（为第一时间可以运行），后面会从头开始写。

文章发表在知乎专栏：[学习C++](https://zhuanlan.zhihu.com/learncpp)。

目录（暂定）：

1. [x] 【c1p1】使用OpenGL搭建基本框架
2. [x] 【c1p2】工厂模式，渲染第一个*polygon body*矩形物体
3. [x] 【c1p3】时钟同步，给物体添加重力*gravity*
4. [x] 【c1p2】绘制物体的受力*force*及速度向量*velocity*
5. [x] 【c1p2】给物体添加旋转*rotate*
6. [x] 【c1p3】求几何凸多边形的*centroid*重心和*inertia*转动惯量（刚体的数据结构）
7. [x] 【c1p3】给物体添加线冲量和角冲量
8. [x] 【c2p1】**collision detection 碰撞检测**
    1. [x] 【c2p1】**AABB方法**（仅限矩形）及**SAT方法**（仅限凸包）
    2. [x] 【c2p2】**计算压力作用点（仅限两个以下）位置（及绘制）**，多边形裁剪
    3. [x] 【c2p3】计算压力方向
    4. [x] 【c2p3】实现摩擦力*friction*
    5. [x] 【c2p3】设置弹性碰撞系数*bias*
    6. [x] 【c2p3】碰撞检测的动态更新*vector<pair>*
    7. [x] 【c2p3】休眠状态的实现
9. [x] 【c3p1】joint 关节（铰链）的实现
    1. [x] 【c3p1】关节的数据结构
    2. [x] 【c3p1】关节的受力分析
10. [x] 实现几种基本场景（分别按键盘上的数字1-5）
    1. [x] 【c2p3】三角形
    2. [x] 【c2p3】金字塔
    3. [x] 【c2p3】方块堆叠
    4. [x] 【c3p1】牛顿摆（可测试完全弹性碰撞）
    5. [x] 【c3p1】铰链（测试关节joint结构）
    6. [x] 【c4p2】圆与多边形金字塔（测试圆的碰撞）
11. [ ] 实现其他特性
    1. [x] 【c4p1】添加圆物体
    2. [x] 【c4p2】碰撞检测（多边形与圆）
    3. [x] 【c4p2】碰撞检测（圆与圆）
12. [ ] 算法测试（有空做）
    1. [ ] 遗传算法
    2. [ ] 神经网络
    3. [ ] 流体力学

## 改进

想到的如下优化：

1. [x] 物体休眠，优化碰撞检测
2. [x] 修正碰撞点，添加弹力碰撞系数

改进了一些问题：

- [x] ~~拖动窗口导致的问题~~
- [x] ~~标题位置随分辨率不一致的问题~~

目前遇到的问题：

- [x] ~~碰撞机制存在问题~~
- [x] ~~圆与多边形的碰撞检测存在问题~~
- [x] ~~圆粘住边界的问题~~，~~圆无法弹跳~~，~~牛顿摆的问题~~
- [ ] 圆的摩擦力问题

## 文章

- [开篇](http://zhuanlan.zhihu.com/p/42669063)
- [创建窗口](http://zhuanlan.zhihu.com/p/42773209)

## 截图

![joint](https://pic4.zhimg.com/v2-c73942281170acf8d4474ffedcf49d94_1200x500.jpg)

## 参考

1. [apollonia](https://github.com/wgtdkp/apollonia)
2. [matter.js](https://github.com/liabru/matter-js)
3. [PhysicsEngine](https://github.com/bajdcc/PhysicsEngine)
4. [OpenGL](https://www.opengl.org/)
5. [GLUT](https://www.opengl.org/resources/libraries/glut/)
6. [Box2D](http://box2d.org/)
7. [Chipmunk2D](https://chipmunk-physics.net/)
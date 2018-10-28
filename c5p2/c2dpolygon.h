//
// Project: clib2d
// Created by bajdcc
//

#ifndef CLIB2D_C2DPOLYGON_H
#define CLIB2D_C2DPOLYGON_H

#include <vector>
#include <GL/freeglut.h>
#include "c2dbody.h"

namespace clib {
    // 多边形刚体（仅支持凸多边形，且点集为有序排列）
    class c2d_polygon : public c2d_body {
    public:
        using ptr = std::unique_ptr<c2d_polygon>;

        c2d_polygon(uint16_t _id, decimal _mass, const std::vector<v2> &_vertices);

        // 计算多边形面积
        static decimal calc_polygon_area(const std::vector<v2> &vertices);

        // 计算多边形重心
        static v2 calc_polygon_centroid(const std::vector<v2> &vertices);

        // 计算多边形转动惯量
        static decimal calc_polygon_inertia(decimal mass, const std::vector<v2> &vertices);

        // 计算边界（矩形包围）
        void calc_bounds();

        // 判断在边界内
        bool contains_in_bound(const v2 &pt);

        // 判断在多边形内（判断连线向量方向的一致性，要求凸包、顶点连续、逆时针排列）
        bool contains_in_polygon(const v2 &pt);

        bool contains(const v2 &pt) override;

        void init();

        void refresh();

        void impulse(const v2 &p, const v2 &r) override;

        v2 world() const override;

        c2d_body_t type() const override;

        v2 min() const override;

        v2 max() const override;

        void update(v2 gravity, int n) override;

        void pass0();

        void pass1();

        void pass2();

        void pass3(const v2 &gravity);

        void pass4();

        void pass5();

        // 拖拽物体
        void drag(const v2 &pt, const v2 &offset) override;

        void draw() override;

        // 以idx为起点，下一顶点为终点的向量
        v2 edge(size_t idx) const;

        v2 &vertex(size_t idx);

        size_t index(size_t idx) const;

        size_t edges() const;

        v2 center; // 重心
        m2 R; // 旋转矩阵
        std::vector<v2> vertices; // 多边形的顶点（本地坐标）
        std::vector<v2> verticesWorld; // 多边形的顶点（世界坐标）
        v2 boundMin, boundMax; // 外包矩形
    };
}

#endif //CLIB2D_C2DPOLYGON_H

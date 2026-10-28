#pragma once

#include <Eigen/Dense>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>
#include "BezierInterpolation.hpp"

template<int Dim>
class LinearBezierBlendInterpolation {
public:
    using Point = Eigen::Matrix<double, Dim, 1>;
    using Points = std::vector<Point>;
    using BezierSegment = BezierInterpolation<Dim>;

    // 构造函数
    LinearBezierBlendInterpolation(const Points& viaPoints, double delta, double beta_bar = 0);

    // 计算混合轨迹上的点
    Point evaluate(double u_hat) const;
    
    // 计算导数
    Point derivative(double u_hat, int order = 1) const;

    // 获取总长度（û的范围）
    double totalLength() const { return total_length_; }

    // 获取再参数化的û值
    const std::vector<double>& uHat() const { return u_hat_; }
    const Points& getQPrime() const { return q_prime_; }
    const Points& getQDPrime() const { return q_dprime_; }

private:
    enum SegmentType { LINEAR, BEZIER };

    struct Segment {
        SegmentType type;
        Point p0, p1;  // 起点和终点
        BezierSegment bezier;  // 贝塞尔段对象（仅BEZIER类型使用）
        double lambda;  // 该段的λ值
    };

    // 计算偏移点q'_k和q''_k
    void computeOffsetPoints();
    
    // 构建所有段
    void constructSegments();
    
    // 计算再参数化的û值
    void computeUhat();
    
    // 找到û对应的段和局部参数u
    std::pair<size_t, double> findSegment(double u_hat) const;
    
    // 计算五次贝塞尔曲线的控制点
    void computeQuinticBezierControlPoints(int k, typename BezierSegment::Points& control_points);

    Points via_points_;        // 原始路径点
    double delta_;             // 混合半径δ
    double beta_bar_;          // β_bar参数
    Points q_prime_;           // q'_k 点
    Points q_dprime_;          // q''_k 点
    std::vector<Segment> segments_;  // 所有段（线性和贝塞尔）
    std::vector<double> u_hat_;      // 各段的û_i值
    double total_length_;      // 总长度（û_max）
};

// 包含实现
#include "LinearBezierBlendInterpolation.tpp"
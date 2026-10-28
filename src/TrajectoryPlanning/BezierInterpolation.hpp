#pragma once

#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>

template<int Dim>
class BezierInterpolation {
public:
    using Point = Eigen::Matrix<double, Dim, 1>;
    using Points = std::vector<Point>;
    using ControlPoints = std::vector<Points>;

    BezierInterpolation(const Points& viaPoints, 
                       const std::vector<Point>& tangents,
                       const std::vector<Point>& curvatures,
                       int degree = 5,
                       double beta_bar = 0.51);
     // 构造函数2: 默认构造函数
    BezierInterpolation() = default;
    
    // 构造函数3: 通过控制点直接构造
    explicit BezierInterpolation(const Points& controlPoints) : single_ctrl_pts_(controlPoints){};
    
    void compute();
    Point evaluate(double u) const;
    Point single_evaluate(double u) const;
    Point derivative(double u, int order = 1) const;
    Point single_derivative(double u, int order = 1) const;

    const ControlPoints& getControlPoints() const { return bezier_control_points_; }
    // const std::vector<double>& getSegmentLengths() const { return segment_lengths_; }
    // const std::vector<double>& getUtime() const { return segment_lengths_; }
   
    double totalLength() const { return total_length_; }
    const std::vector<double>& uHat() const { return u_hat_; }
    int getDegree() const { return degree_; }
    Point evaluateBezier(const Points& ctrl_pts, double u) const;

private:
    Points via_points_;
    std::vector<Point> tangents_;
    std::vector<Point> curvatures_;
    int degree_;
    double beta_bar_;
    ControlPoints bezier_control_points_;
    Points single_ctrl_pts_;
    std::vector<double> segment_lengths_;

    void computeCubicSegment(size_t k);
    void computeQuinticSegment(size_t k);
    double solveQuadratic(double a, double b, double c) const;
    double solveQuartic(double a, double b, double c, double d, double e) const;
    
    Point derivativeBezier(const Points& ctrl_pts, double u, int order) const;
    std::vector<double> u_hat_;  // 累积长度参数 û_i
    double total_length_ = 0.0;  // 路径总长度

};

// 包含模板实现
#include "BezierInterpolation.tpp"
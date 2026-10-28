#pragma once
#include <vector>
#include <Eigen/Core>
#include <sophus/se3.hpp>

class SE3BaseInterp {
public:
    // 单点插值 β∈[0,1]
    static Sophus::SE3d interpolate(const Sophus::SE3d& T0, const Sophus::SE3d& T1, double beta);
    

    // 批量插值（含起点/终点）
    std::vector<Sophus::SE3d>
    interpolate(const Sophus::SE3d& T0,
                const Sophus::SE3d& T1,
                std::size_t num_steps) const;
};

class SE3BezierInterpolator {
public:
    SE3BezierInterpolator(const std::vector<Sophus::SE3d>& control_points);
    
    // 计算贝塞尔曲线上参数t对应的位姿 (t ∈ [0,1])
    Sophus::SE3d evaluate(double t) const;
    Sophus::SE3d   bezier(const std::vector<Sophus::SE3d>& controls,   int n,   double t) const;
    Sophus::SE3d RationalBezier(const std::vector<Sophus::SE3d>& controls,
                        int n,
                        double t, const std::vector<double>& ratio) const;

private:
    // 递归De Casteljau算法
    Sophus::SE3d deCasteljau(const std::vector<Sophus::SE3d>& points, double t) const;
    
    std::vector<Sophus::SE3d> control_points_;
};

class SE3BSplineInterpolator {
public:
    SE3BSplineInterpolator(const std::vector<Sophus::SE3d>& control_points, 
                          int degree = 3);
    
    // 计算B样条曲线上参数t对应的位姿 (t ∈ [0,1])
    Sophus::SE3d evaluate(double t) const;

private:
    // Cox-de Boor递归基函数
    double basisFunction(int i, int p, double u) const;
    
    // 计算节点向量
    void computeKnotVector();
    
    std::vector<Sophus::SE3d> control_points_;
    std::vector<double> knots_;
    int degree_;
    int n_control_;
};

class QuatAndTInterpolator {
public:
    static Eigen::Matrix4d Interpolate(const Eigen::Matrix4d& pose_src, const Eigen::Matrix4d& pose_dst, double beta);

};
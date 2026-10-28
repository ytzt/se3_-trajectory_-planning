#include "SE3Interpolators.hpp"
#include <stdexcept>
#include <algorithm>

SE3BSplineInterpolator::SE3BSplineInterpolator(
    const std::vector<Sophus::SE3d>& control_points, int degree)
    : control_points_(control_points), 
      degree_(degree),
      n_control_(control_points.size()) {
    if (n_control_ <= degree_) {
        throw std::invalid_argument(
            "Control points size must be greater than degree");
    }
    computeKnotVector();
}

void SE3BSplineInterpolator::computeKnotVector() {
    int n_knots = n_control_ + degree_ + 1;
    knots_.resize(n_knots);
    
    // 创建均匀分布的节点向量
    for (int i = 0; i <= degree_; ++i) {
        knots_[i] = 0.0; // 起始重复度 = degree + 1
    }
    
    double internal_knots = n_control_ - degree_;
    for (int i = degree_ + 1; i < n_knots - degree_ - 1; ++i) {
        knots_[i] = static_cast<double>(i - degree_) / internal_knots;
    }
    
    for (int i = n_knots - degree_ - 1; i < n_knots; ++i) {
        knots_[i] = 1.0; // 结束重复度 = degree + 1
    }
}

double SE3BSplineInterpolator::basisFunction(int i, int p, double u) const {
    if (p == 0) {
        return (knots_[i] <= u && u < knots_[i+1]) ? 1.0 : 0.0;
    }
    
    double denom1 = knots_[i+p] - knots_[i];
    double term1 = (denom1 > 1e-6) ? 
        ((u - knots_[i]) / denom1) * basisFunction(i, p-1, u) : 0.0;
    
    double denom2 = knots_[i+p+1] - knots_[i+1];
    double term2 = (denom2 > 1e-6) ? 
        ((knots_[i+p+1] - u) / denom2) * basisFunction(i+1, p-1, u) : 0.0;
    
    return term1 + term2;
}

Sophus::SE3d SE3BSplineInterpolator::evaluate(double t) const {
    // 将t映射到有效参数范围 [u_{p}, u_{m-p}]
    double u = knots_[degree_] + t * (knots_[n_control_] - knots_[degree_]);
    
    // 计算每个控制点的权重
    Eigen::VectorXd weights(n_control_);
    for (int i = 0; i < n_control_; ++i) {
        weights(i) = basisFunction(i, degree_, u);
    }
    
    // 使用对数映射到切空间
    Eigen::Matrix<double, 6, Eigen::Dynamic> tangents(6, n_control_);
    for (int i = 0; i < n_control_; ++i) {
        tangents.col(i) = control_points_[i].log();
    }
    
    // 加权平均切向量
    Eigen::Matrix<double, 6, 1> avg_tangent = tangents * weights;
    avg_tangent /= weights.sum();
    
    // 映射回李群
    return Sophus::SE3d::exp(avg_tangent);
}
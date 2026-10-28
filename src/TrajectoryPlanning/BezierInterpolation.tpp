#include <iostream>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <cassert>

template<int Dim>
BezierInterpolation<Dim>::BezierInterpolation(
    const Points& viaPoints, 
    const std::vector<Point>& tangents,
    const std::vector<Point>& curvatures,
    int degree,
    double beta_bar)
    : via_points_(viaPoints), 
      tangents_(tangents),
      curvatures_(curvatures),
      degree_(degree),
      beta_bar_(beta_bar) 
{
    if (viaPoints.size() < 2) {
        throw std::invalid_argument("At least two via points required");
    }
    if (degree_ != 3 && degree_ != 5) {
        throw std::invalid_argument("Degree must be 3 or 5");
    }
    if (viaPoints.size() != tangents.size()) {
        throw std::invalid_argument("Tangents size mismatch with via points");
    }
    if (degree_ == 5 && viaPoints.size() != curvatures.size()) {
        throw std::invalid_argument("Curvatures size mismatch for quintic Bezier");
    }
    
    bezier_control_points_.resize(via_points_.size() - 1);
    segment_lengths_.resize(via_points_.size() - 1, 0.0);
}




template<int Dim>
void BezierInterpolation<Dim>::compute() {
    u_hat_.resize(via_points_.size());  // 准备存储累积参数
    
    // 初始化累积参数
    u_hat_[0] = 0.0;
    total_length_ = 0.0;

    for (size_t k = 0; k < via_points_.size() - 1; ++k) {
        if (degree_ == 3) {
            computeCubicSegment(k);
        } else { // degree_ == 5
            computeQuinticSegment(k);
        }
        // 累加总长度
        total_length_ += segment_lengths_[k];
        // 根据公式(9.14)计算下一个路径点的累积参数: û_i = û_{i-1} + λ_{i-1}
        u_hat_[k+1] = u_hat_[k] + segment_lengths_[k];
    }
}

template<int Dim>
void BezierInterpolation<Dim>::computeCubicSegment(size_t k) {
    const Point& qk = via_points_[k];
    const Point& qk1 = via_points_[k+1];
    const Point& tk = tangents_[k];
    const Point& tk1 = tangents_[k+1];

    // 计算二次方程系数 (8.52)
    double a = 16.0 - (tk + tk1).squaredNorm();
    double b = 12.0 * (qk1 - qk).dot(tk + tk1);
    double c = -36.0 * (qk1 - qk).squaredNorm();

    // 求解二次方程
    double alpha_k = solveQuadratic(a, b, c);

    // 计算控制点 (8.51)
    Points ctrl_pts(4);
    ctrl_pts[0] = qk; // p0,k
    ctrl_pts[1] = ctrl_pts[0] + (alpha_k / 3.0) * tk; // p1,k
    ctrl_pts[3] = qk1; // p3,k
    ctrl_pts[2] = ctrl_pts[3] - (alpha_k / 3.0) * tk1; // p2,k

    bezier_control_points_[k] = ctrl_pts;
    // segment_lengths_[k] = 3.0 * (ctrl_pts[1] - ctrl_pts[0]).norm(); // λ_k = 3|p1,k - p0,k|
    double seg_length = 0.0;
    for (int i = 0; i < 3; ++i) {  // 三次贝塞尔有4个控制点，3段折线
        seg_length += (ctrl_pts[i+1] - ctrl_pts[i]).norm();
    }
    segment_lengths_[k] = seg_length;
}

template<int Dim>
void BezierInterpolation<Dim>::computeQuinticSegment(size_t k) {
    const Point& qk = via_points_[k];
    const Point& qk1 = via_points_[k+1];
    const Point& tk = tangents_[k];
    const Point& tk1 = tangents_[k+1];
    const Point& nk = curvatures_[k];
    const Point& nk1 = curvatures_[k+1];
    double alpha_k = 0;
    if (beta_bar_ < 1e-5) {
        // 使用公式(8.66)计算α_k
        double a = 256.0 - 49.0 * (tk + tk1).squaredNorm();
        double b = 420.0 * (qk1 - qk).dot(tk + tk1);
        double c = -900.0 * (qk1 - qk).squaredNorm();
        // ... 解二次方程
         alpha_k = solveQuadratic(a, b, c);

    } else{
        // 计算四次方程系数 (8.56)
        double a = beta_bar_ * beta_bar_ * (nk1 - nk).squaredNorm();
        double b = -28.0 * beta_bar_ * (tk + tk1).dot(nk1 - nk);
        double c = 196.0 * (tk + tk1).squaredNorm() + 
                120.0 * beta_bar_ * (qk1 - qk).dot(nk1 - nk) - 1024.0;
        double d = -1680.0 * (qk1 - qk).dot(tk + tk1);
        double e = 3600.0 * (qk1 - qk).squaredNorm();

        // 求解四次方程
        alpha_k = solveQuartic(a, b, c, d, e);
        
    }
    std::cout<<"alpha_k " << alpha_k  <<"\t";
    

    // 计算控制点 (8.57)
    Points ctrl_pts(6);
    ctrl_pts[0] = qk; // p0,k
    ctrl_pts[1] = ctrl_pts[0] + (alpha_k / 5.0) * tk; // p1,k
    ctrl_pts[2] = 2*ctrl_pts[1] - ctrl_pts[0] + 
                 (beta_bar_ * alpha_k * alpha_k / 20.0) * nk; // p2,k
    ctrl_pts[5] = qk1; // p5,k
    ctrl_pts[4] = ctrl_pts[5] - (alpha_k / 5.0) * tk1; // p4,k
    ctrl_pts[3] = 2*ctrl_pts[4] - ctrl_pts[5] + 
                 (beta_bar_ * alpha_k * alpha_k / 20.0) * nk1; // p3,k

    bezier_control_points_[k] = ctrl_pts;
    // segment_lengths_[k] = 5.0 * (ctrl_pts[1] - ctrl_pts[0]).norm(); // λ_k = 5|p1,k - p0,k|
    double seg_length = 0.0;
    for (int i = 0; i < 5; ++i) {
        seg_length += (ctrl_pts[i+1] - ctrl_pts[i]).norm();
    }
    segment_lengths_[k] = seg_length;  // 更新为折线总长度
}

template<int Dim>
double BezierInterpolation<Dim>::solveQuadratic(double a, double b, double c) const {
    double discriminant = b*b - 4*a*c;
    if (discriminant < 0) {
        return 1.0; // 默认值
    }
    
    double root1 = (-b + std::sqrt(discriminant)) / (2*a);
    double root2 = (-b - std::sqrt(discriminant)) / (2*a);
    
    // 返回正根（优先返回较大的正根）
    if (root1 > 0 && root2 > 0) {
        return std::max(root1, root2);
    } else if (root1 > 0) {
        return root1;
    } else if (root2 > 0) {
        return root2;
    }
    return 1.0; // 默认值
}

template<int Dim>
double BezierInterpolation<Dim>::solveQuartic(double a, double b, double c, double d, double e) const {
    // 使用Eigen多项式求解器
    Eigen::VectorXd coeffs(5);
    coeffs << e, d, c, b, a; // 升序排列：常数项、一次项、二次项、三次项、四次项

    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coeffs);
    
    // 获取实根
    std::vector<double> real_roots;
    Eigen::VectorXcd roots = solver.roots();
    
    for (int i = 0; i < roots.size(); ++i) {
        if (std::abs(roots[i].imag()) < 1e-10) {
            double root = roots[i].real();
            if (root > 0) {
                real_roots.push_back(root);
            }
        }
    }
    
    if (real_roots.empty()) {
        return 1.0; // 默认值
    }
    
    // 返回最小正实根
    return *std::min_element(real_roots.begin(), real_roots.end());
}

template<int Dim>
typename BezierInterpolation<Dim>::Point 
BezierInterpolation<Dim>::evaluateBezier(const Points& ctrl_pts, double u) const {
    // 使用de Casteljau算法求值
    std::vector<Point> pts = ctrl_pts;
    const int degree = static_cast<int>(ctrl_pts.size()) - 1;
    
    for (int k = 1; k <= degree; ++k) {
        for (int i = 0; i < degree - k + 1; ++i) {
            pts[i] = (1.0 - u) * pts[i] + u * pts[i+1];
        }
    }
    return pts[0];
}

template<int Dim>
typename BezierInterpolation<Dim>::Point 
BezierInterpolation<Dim>::derivativeBezier(const Points& ctrl_pts, double u, int order) const {
    // 根据贝塞尔曲线导数公式计算
    if (order == 0) return evaluateBezier(ctrl_pts, u);
    
    const int degree = static_cast<int>(ctrl_pts.size()) - 1;
    Points deriv_pts(degree);
    
    for (int i = 0; i < degree; ++i) {
        deriv_pts[i] = degree * (ctrl_pts[i+1] - ctrl_pts[i]);
    }
    
    // 递归计算高阶导数
    if (order == 1) {
        return evaluateBezier(deriv_pts, u);
    }
    return derivativeBezier(deriv_pts, u, order - 1) * (static_cast<double>(degree) / (degree - order + 1));
}

template<int Dim>
typename BezierInterpolation<Dim>::Point 
BezierInterpolation<Dim>::evaluate(double u) const {
    // 边界处理
    if (u <= 0.0) return via_points_.front();
    if (u >= total_length_) return via_points_.back();
    
    // 查找u所在的段（使用u_hat_参数化）
    size_t segment_index = 0;
    while (segment_index < u_hat_.size() - 1 && u >= u_hat_[segment_index + 1]) {
        ++segment_index;
    }
    
    // 确保在有效范围内
    segment_index = std::min(segment_index, u_hat_.size() - 2);
    
    // 计算局部参数s ∈ [0, 1]
    const double seg_start = u_hat_[segment_index];
    const double seg_end = u_hat_[segment_index + 1];
    const double seg_length = seg_end - seg_start;
    const double s = (u - seg_start) / seg_length;
    
    // 在对应段的贝塞尔曲线上求值
    return evaluateBezier(bezier_control_points_[segment_index], s);
}

template<int Dim>
typename BezierInterpolation<Dim>::Point 
BezierInterpolation<Dim>:: single_evaluate(double u) const{
    // 检查控制点是否为空
    if (single_ctrl_pts_.empty()) {
        throw std::runtime_error("Control points are empty for single_derivative");
    }
     // 检查参数范围
    if (u < 0 || u > 1) {
        throw std::out_of_range("Parameter u must be in [0, 1] for single_evaluate");
    }
    return evaluateBezier(single_ctrl_pts_, u);
}

template<int Dim>
typename BezierInterpolation<Dim>::Point 
BezierInterpolation<Dim>::derivative(double u, int order) const {
    // 边界处理
    if (u <= 0.0) u = 0.0;
    if (u >= total_length_) u = total_length_;
    
    // 查找u所在的段（使用u_hat_参数化）
    size_t segment_index = 0;
    while (segment_index < u_hat_.size() - 1 && u >= u_hat_[segment_index + 1]) {
        ++segment_index;
    }
    
    // 确保在有效范围内
    segment_index = std::min(segment_index, u_hat_.size() - 2);
    
    // 计算局部参数s ∈ [0, 1] 和段长度λ
    const double seg_start = u_hat_[segment_index];
    const double seg_end = u_hat_[segment_index + 1];
    const double seg_length = seg_end - seg_start;
    const double s = (u - seg_start) / seg_length;
    
    // 计算贝塞尔曲线关于局部参数s的导数
    Point ds_derivative = derivativeBezier(bezier_control_points_[segment_index], s, order);
    return ds_derivative / seg_length;
}
template<int Dim>
typename BezierInterpolation<Dim>::Point 
BezierInterpolation<Dim>::single_derivative(double u, int order) const{
    // 检查控制点是否为空
    if (single_ctrl_pts_.empty()) {
        throw std::runtime_error("Control points are empty for single_derivative");
    }
     if (u < 0 || u > 1) {
        throw std::out_of_range("Parameter u must be in [0, 1] for single_evaluate");
    }
    Point ds_derivative = derivativeBezier(single_ctrl_pts_, u, order);
    return ds_derivative;
}
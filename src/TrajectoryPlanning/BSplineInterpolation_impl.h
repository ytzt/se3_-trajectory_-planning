#include <cmath>
#include <iostream>
#include <algorithm>

template<int Dim>
typename BSplineInterpolation<Dim>::CurveFunction 
BSplineInterpolation<Dim>::createCurveFunction() const {
    return [this](double u) -> Eigen::Matrix<double, Dim, 1> {
        return evaluate(u);
    };
}

template<int Dim>
Eigen::Matrix<double, Dim, 1> 
BSplineInterpolation<Dim>::evaluate(double u) const {
    if (controlPoints_.empty() || basisFunctions_.empty()) {
        throw std::runtime_error("No interpolation data available");
    }
    
    Eigen::Matrix<double, Dim, 1> result = 
        Eigen::Matrix<double, Dim, 1>::Zero();
    
    for (size_t i = 0; i < controlPoints_.size(); ++i) {
        double basisVal = basisFunctions_[i](u);
        result += basisVal * controlPoints_[i];
    }
    
    return result;
}
/**
 * 计算默认边界导数
 */
template<int Dim>
void BSplineInterpolation<Dim>::computeDefaultBoundaryDerivatives() {
    const size_t n = points_.size() - 1;
    if (uBar_.size() < 2) {
        throw std::runtime_error("Parameter vector too small");
    }
    
    // 计算起始点导数: t0 = (q1 - q0) / (uBar1 - uBar0)
    default_t0_ = (points_[1] - points_[0]) / (uBar_[1] - uBar_[0]);
    
    // 计算终点导数: tn = (qn - qn-1) / (uBarn - uBarn-1)
    default_tn_ = (points_[n] - points_[n-1]) / (uBar_[n] - uBar_[n-1]);
}

template<int Dim>
void BSplineInterpolation<Dim>::interpolateCubic(
    const std::vector<Eigen::Matrix<double, Dim, 1>>& points,
    const Eigen::Matrix<double, Dim, 1>& t0,
    const Eigen::Matrix<double, Dim, 1>& tn,
    ParametrizationMethod method)
{
    const size_t n = points.size() - 1;
    if (n < 1) {
        throw std::invalid_argument("At least 2 points required for interpolation");
    }

    // 1. 计算参数化值
    std::vector<double> uBar = computeParametrization(points, method);
    
    // 2. 构建节点向量
    std::vector<double> knots = buildKnotVector(uBar);
    
    // 3. 计算边界控制点
    Eigen::Matrix<double, Dim, 1> p0, p1, pn1, pn2;
    computeBoundaryControlPoints(points, uBar, t0, tn, p0, p1, pn1, pn2);
    
    // 4. 准备三对角方程组
    const size_t numEquations = n - 1;
    Eigen::VectorXd diag(numEquations);
    Eigen::VectorXd lower, upper;
    if (numEquations > 1) {
        lower.resize(numEquations - 1);
        upper.resize(numEquations - 1);
    }
    std::vector<Eigen::Matrix<double, Dim, 1>> R(numEquations);

    // 5. 计算基函数生成器
    bspline::BSplineGenerator<double> generator(knots);
    basisFunctions_ = generator.generateBSplines<3>(); // 三次B样条基函数（阶数3）
    
    // 6. 填充三对角矩阵和右侧项
    for (size_t k = 1; k <= n-1; ++k) {
        const double u = uBar[k];
        
        // 计算非零基函数值
        double B0 = basisFunctions_[k](u);     // 对应控制点k
        double B1 = basisFunctions_[k+1](u);   // 对应控制点k+1
        double B2 = basisFunctions_[k+2](u);   // 对应控制点k+2
        
        // 设置矩阵元素
        const size_t idx = k - 1;
        diag(idx) = B1;
        
        if (idx < numEquations - 1) {
            upper(idx) = B2;
        }
        if (idx > 0) {
            lower(idx-1) = B0;
        }
        
        // 计算右侧项
        R[idx] = points[k];
        if (k == 1) {
            R[idx] -= B0 * p1;
        } else if (k == n-1) {
            R[idx] -= B2 * pn1;
        }
    }

    // 7. 求解三对角方程组 (每个维度独立求解)
    std::vector<Eigen::Matrix<double, Dim, 1>> pMiddle(numEquations);
    for (int d = 0; d < Dim; ++d) {
        Eigen::VectorXd rhs(numEquations);
        for (size_t i = 0; i < numEquations; ++i) {
            rhs(i) = R[i](d);
        }
        
        // Eigen::VectorXd sol;
        // solveTridiagonalSystem(diag, lower, upper, rhs, sol);
        Eigen::VectorXd sol = solveTridiagonal(diag, upper, lower, rhs);
        
        for (size_t i = 0; i < numEquations; ++i) {
            pMiddle[i](d) = sol(i);
        }
    }

    // 8. 组装所有控制点
    controlPoints_.clear();
    controlPoints_.reserve(n + 3);
    controlPoints_.push_back(p0);
    controlPoints_.push_back(p1);
    controlPoints_.insert(controlPoints_.end(), pMiddle.begin(), pMiddle.end());
    controlPoints_.push_back(pn1);
    controlPoints_.push_back(pn2);
}

template<int Dim>
void BSplineInterpolation<Dim>::interpolateCubic(
    const Eigen::Matrix<double, Dim, 1>& t0,
    const Eigen::Matrix<double, Dim, 1>& tn)
{
    const size_t n = points_.size() - 1;
    
    // 检查节点向量是否已构建
    if (knots_.empty()) {
        throw std::runtime_error("Knot vector not initialized. Call constructor first.");
    }
    
    // 计算边界控制点
    Eigen::Matrix<double, Dim, 1> p0, p1, pn1, pn2;
    computeBoundaryControlPoints(points_, uBar_, t0, tn, p0, p1, pn1, pn2);
    
    // 准备三对角方程组
    const size_t numEquations = n - 1;
    Eigen::VectorXd diag(numEquations);
    Eigen::VectorXd lower, upper;
    if (numEquations > 1) {
        lower.resize(numEquations - 1);
        upper.resize(numEquations - 1);
    }
    std::vector<Eigen::Matrix<double, Dim, 1>> R(numEquations);

    // 计算基函数生成器
    bspline::BSplineGenerator<double> generator(knots_);
    basisFunctions_ = generator.generateBSplines<3>(); // 三次B样条基函数（阶数3）
    
    // 填充三对角矩阵和右侧项
    for (size_t k = 1; k <= n-1; ++k) {
        const double u = uBar_[k];
        // 计算非零基函数值
        double B0 = basisFunctions_[k](u);     // 对应控制点k
        double B1 = basisFunctions_[k+1](u);   // 对应控制点k+1
        double B2 = basisFunctions_[k+2](u);   // 对应控制点k+2
        
        // 设置矩阵元素
        const size_t idx = k - 1;
        diag(idx) = B1;
        
        if (idx < numEquations - 1) {
            upper(idx) = B2;
        }
        if (idx > 0) {
            lower(idx-1) = B0;
        }
        
        // 计算右侧项
        R[idx] = points_[k];
        if (k == 1) {
            R[idx] -= B0 * p1;
        } else if (k == n-1) {
            R[idx] -= B2 * pn1;
        }
    }

    // 求解三对角方程组 (每个维度独立求解)
    std::vector<Eigen::Matrix<double, Dim, 1>> pMiddle(numEquations);
    for (int d = 0; d < Dim; ++d) {
        Eigen::VectorXd rhs(numEquations);
        for (size_t i = 0; i < numEquations; ++i) {
            rhs(i) = R[i](d);
        }
        
        // Eigen::VectorXd sol;
        // solveTridiagonalSystem(diag, lower, upper, rhs, sol);
        Eigen::VectorXd sol = solveTridiagonal(diag, upper, lower, rhs);
        
        for (size_t i = 0; i < numEquations; ++i) {
            pMiddle[i](d) = sol(i);
        }
    }

    // 组装所有控制点
    controlPoints_.clear();
    controlPoints_.reserve(n + 3);
    controlPoints_.push_back(p0);
    controlPoints_.push_back(p1);
    controlPoints_.insert(controlPoints_.end(), pMiddle.begin(), pMiddle.end());
    controlPoints_.push_back(pn1);
    controlPoints_.push_back(pn2);
}

template<int Dim>
const std::vector<Eigen::Matrix<double, Dim, 1>>& 
BSplineInterpolation<Dim>::getControlPoints() const {
    return controlPoints_;
}
template<int Dim>
const std::vector<double>& BSplineInterpolation<Dim>::getUbar() const{
    return uBar_;
}

template<int Dim>
const std::vector<bspline::Spline<double, 3>>& 
BSplineInterpolation<Dim>::getBasisFunctions() const {
    return basisFunctions_;
}

template<int Dim>
std::vector<double> BSplineInterpolation<Dim>::computeParametrization(
    const std::vector<Eigen::Matrix<double, Dim, 1>>& points,
    ParametrizationMethod method) const
{
    const size_t n = points.size() - 1;
    std::vector<double> uBar(n + 1);
    uBar[0] = 0.0;

    // 计算弦长总和
    double totalLength = 0.0;
    std::vector<double> segmentLengths;
    for (size_t k = 1; k <= n; ++k) {
        double dist = (points[k] - points[k-1]).norm();
        segmentLengths.push_back(dist);
        
        if (method == ParametrizationMethod::CENTRIPETAL) {
            totalLength += std::sqrt(dist);
        } else {
            totalLength += dist;
        }
    }

    // 计算参数化值
    for (size_t k = 1; k <= n; ++k) {
        double delta = segmentLengths[k-1];
        if (method == ParametrizationMethod::CENTRIPETAL) {
            delta = std::sqrt(delta);
        }
        uBar[k] = uBar[k-1] + delta / totalLength;
    }

    return uBar;
}

template<int Dim>
void BSplineInterpolation<Dim>::computeBoundaryControlPoints(
    const std::vector<Eigen::Matrix<double, Dim, 1>>& points,
    const std::vector<double>& uBar,
    const Eigen::Matrix<double, Dim, 1>& t0,
    const Eigen::Matrix<double, Dim, 1>& tn,
    Eigen::Matrix<double, Dim, 1>& p0,
    Eigen::Matrix<double, Dim, 1>& p1,
    Eigen::Matrix<double, Dim, 1>& pn1,
    Eigen::Matrix<double, Dim, 1>& pn2) const
{
    const size_t n = points.size() - 1;
    p0 = points[0];
    pn2 = points[n];
    
    // 计算u4和u_{n+3}
    const double u4 = uBar[1];  // 索引4对应uBar[1]
    const double un3 = uBar[n-1]; // 索引n+2对应uBar[n-1]
    
    p1 = points[0] + (u4 / 3.0) * t0;
    pn1 = points[n] - ((1.0 - un3) / 3.0) * tn;
}

// 静态方法实现
template<int Dim>
std::vector<double> BSplineInterpolation<Dim>::buildKnotVector(
    const std::vector<double>& uBar) 
{
    const size_t n = uBar.size() - 1;
    std::vector<double> knots;

    // 开头4个节点 (u0重复4次)
    for (int i = 0; i < 4; ++i) {
        knots.push_back(uBar[0]);
    }

    // 中间节点 (u1到un)
    for (size_t j = 1; j <= n; ++j) {
        knots.push_back(uBar[j]);
    }

    // 结尾4个节点 (un重复4次)
    for (int i = 0; i < 4; ++i) {
        knots.push_back(uBar[n]);
    }

    return knots;
}

template<int Dim>
std::vector<double> BSplineInterpolation<Dim>::buildKnotVector(
    const std::vector<double>& uBar, int p) 
{
    if (uBar.empty()) return {};
    const size_t n = uBar.size() - 1;  // 区间数
    std::vector<double> knots;

    // 开头重复p+1次
    for (int i = 0; i <= p; ++i) {
        knots.push_back(uBar.front());
    }

    // 中间节点
    if (p % 2 == 1) {
        // 奇数阶：使用公式(4.42)/(8.39)
        // 中间节点为uBar[1]到uBar[n-1]
        for (size_t j = 1; j < n; ++j) {
            knots.push_back(uBar[j]);
        }
    } else {
        // 偶数阶：使用公式(4.43)/(8.40)
        // 中间节点为相邻参数值的中点
        for (size_t j = 0; j < n; ++j) {
            double mid = (uBar[j] + uBar[j+1]) / 2.0;
            knots.push_back(mid);
        }
    }

    // 结尾重复p+1次
    for (int i = 0; i <= p; ++i) {
        knots.push_back(uBar.back());
    }

    return knots;
}

template<int Dim>
void BSplineInterpolation<Dim>::solveTridiagonalSystem(
    const Eigen::VectorXd& diag,
    const Eigen::VectorXd& lower,
    const Eigen::VectorXd& upper,
    const Eigen::VectorXd& rhs,
    Eigen::VectorXd& solution) 
{
    const size_t n = diag.size();
    Eigen::VectorXd c_prime(n);
    Eigen::VectorXd d_prime(n);
    solution.resize(n);

    // 前向消元
    c_prime(0) = upper(0) / diag(0);
    d_prime(0) = rhs(0) / diag(0);
    
    for (size_t i = 1; i < n; ++i) {
        double denom = diag(i) - lower(i-1) * c_prime(i-1);
        if (i < n-1) {
            c_prime(i) = upper(i) / denom;
        }
        d_prime(i) = (rhs(i) - lower(i-1) * d_prime(i-1)) / denom;
    }
    
    // 回代
    solution(n-1) = d_prime(n-1);
    for (int i = n - 2; i >= 0; --i) {
        solution(i) = d_prime(i) - c_prime(i) * solution(i+1);
    }
}

// 三对角矩阵求解算法 (Thomas算法)
template<int Dim>
Eigen::VectorXd BSplineInterpolation<Dim>::solveTridiagonal(
    const Eigen::VectorXd& diag,
    const Eigen::VectorXd& upper,
    const Eigen::VectorXd& lower,
    const Eigen::VectorXd& rhs) const 
{
    const int n = diag.size();
    if (n == 0) return Eigen::VectorXd();
    if (n == 1) return (rhs(0) / diag(0)) * Eigen::VectorXd::Ones(1);

    // 可修改副本
    Eigen::VectorXd A_diag = diag;
    Eigen::VectorXd A_upper = upper;
    Eigen::VectorXd A_lower = lower;
    Eigen::VectorXd d = rhs;
    Eigen::VectorXd x(n);
    x.setZero();

    // 前向消元
    for (int i = 1; i < n; ++i) {
        double w = A_lower(i - 1) / A_diag(i - 1);
        A_diag(i) -= w * A_upper(i - 1);
        d(i) -= w * d(i - 1);
    }

    // 回代
    x(n - 1) = d(n - 1) / A_diag(n - 1);
    for (int i = n - 2; i >= 0; --i) {
        x(i) = (d(i) - A_upper(i) * x(i + 1)) / A_diag(i);
    }

    return x;
}


template<int Dim>
int BSplineInterpolation<Dim>::findSpan(double u) const {
    if (knots_.empty()) {
        throw std::runtime_error("Knot vector is empty");
    }
    
    const int n = knots_.size() - 1;  // 最大节点索引
    const int p = p_;  // 使用类成员变量p_（样条阶数）
    
    // 处理边界情况
    if (u >= knots_[n]) {
        // 当u等于最后一个节点时，返回n-p（考虑节点多重性）
        return n - p;
    }
    if (u <= knots_[0]) {
        return p;
    }
    
    // 初始化二分查找边界
    int low = p;
    int high = n - p;  // 有效区间上限
    int mid = (low + high) / 2;
    
    // 直接处理u等于high节点的情况（节点多重性处理）
    if (u == knots_[high]) {
        return high;
    }
    
    // 优化二分查找
    while (u < knots_[mid] || u >= knots_[mid + 1]) {
        if (u == knots_[mid + 1]) {
            // 处理节点多重性：当u等于右节点时直接返回mid+1
            return mid + 1;
        }
        
        if (u < knots_[mid]) {
            high = mid;  // 搜索左半部分
        } else {
            low = mid + 1;  // 搜索右半部分
        }
        mid = (low + high) / 2;
    }
    
    return mid;
}

template<int Dim>
void BSplineInterpolation<Dim>::dersBasisFuns(double u, int span, int p, int n, 
                                             std::vector<std::vector<double>>& ders) const {
    // 初始化输出矩阵
    ders.resize(n+1);
    for (int i = 0; i <= n; i++) {
        ders[i].resize(p+1, 0.0);
    }
    
    // 临时存储矩阵
    std::vector<std::vector<double>> ndu(p+1, std::vector<double>(p+1, 0.0));
    std::vector<double> left(p+1, 0.0);
    std::vector<double> right(p+1, 0.0);
    
    // 计算基函数值
    ndu[0][0] = 1.0;
    for (int j = 1; j <= p; j++) {
        left[j] = u - knots_[span+1-j];
        right[j] = knots_[span+j] - u;
        double saved = 0.0;
        
        for (int r = 0; r < j; r++) {
            // 下三角矩阵
            ndu[j][r] = right[r+1] + left[j-r];
            double temp = ndu[r][j-1] / ndu[j][r];
            
            // 上三角矩阵
            ndu[r][j] = saved + right[r+1] * temp;
            saved = left[j-r] * temp;
        }
        ndu[j][j] = saved;
    }
    
    // 保存基函数值 (0阶导数)
    for (int j = 0; j <= p; j++) {
        ders[0][j] = ndu[j][p];
    }
    
    // 计算导数
    for (int r = 0; r <= p; r++) {
        int s1 = 0;
        int s2 = 1;
        std::vector<std::vector<double>> a(2, std::vector<double>(p+1, 0.0));
        a[0][0] = 1.0;
        
        for (int k = 1; k <= n; k++) {
            double d = 0.0;
            int rk = r - k;
            int pk = p - k;
            
            if (r >= k) {
                a[s2][0] = a[s1][0] / ndu[pk+1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }
            
            int j1 = (rk >= -1) ? 1 : -rk;
            int j2 = (r-1 <= pk) ? k-1 : p - r;
            
            for (int j = j1; j <= j2; j++) {
                a[s2][j] = (a[s1][j] - a[s1][j-1]) / ndu[pk+1][rk+j];
                d += a[s2][j] * ndu[rk+j][pk];
            }
            
            if (r <= pk) {
                a[s2][k] = -a[s1][k-1] / ndu[pk+1][r];
                d += a[s2][k] * ndu[r][pk];
            }
            
            ders[k][r] = d;
            std::swap(s1, s2); // 交换源和目标
        }
    }
    
    // 乘以阶乘系数
    int r = p;
    for (int k = 1; k <= n; k++) {
        for (int j = 0; j <= p; j++) {
            ders[k][j] *= r;
        }
        r *= (p - k);
    }
}


template<int Dim>
Eigen::Matrix<double, Dim, 1> 
BSplineInterpolation<Dim>::derivative(double u, unsigned int k, int p) const {
    p = p_;
    if (p <= 0) {
        throw std::invalid_argument("Spline order p must be positive");
    }
    
    // 检查k是否超过样条阶数p
    if (k > static_cast<unsigned int>(p)) {
        // 对于高于p阶的导数，返回零向量
        return Eigen::Matrix<double, Dim, 1>::Zero();
    }
    
    if (controlPoints_.empty() || knots_.empty()) {
        throw std::runtime_error("No interpolation data available. Call interpolateCubic first.");
    }
    // ===== 修复1：边界处理 - 避免在精确节点值处计算 =====
    const double epsilon = 1e-5;
    
    
    // 如果是最后一个节点，稍微向内调整参数值
    if (u >= knots_.back() - epsilon) {
        u = knots_.back() - epsilon;
    }
    
    // 找到u所在的节点区间
    int span = findSpan(u);
    
    // 计算基函数及其导数
    std::vector<std::vector<double>> ders;
    dersBasisFuns(u, span, p, k, ders);

    // 打印ders矩阵内容
    // std::cout << "\n=== ders Matrix (k=" << k << ", p=" << p << ", u=" << u << ") ===" << std::endl;
    // for (size_t i = 0; i < ders.size(); ++i) {
    //     std::cout << "Order " << i << ": [";
    //     for (size_t j = 0; j < ders[i].size(); ++j) {
    //         std::cout << ders[i][j];
    //         if (j < ders[i].size() - 1) std::cout << ", ";
    //     }
    //     std::cout << "]" << std::endl;
    // }
    // std::cout << "=============================\n" << std::endl;
    
    // 计算导数
    Eigen::Matrix<double, Dim, 1> result = 
        Eigen::Matrix<double, Dim, 1>::Zero();
    
    // 遍历该节点区间内的基函数
    for (int j = 0; j <= p; j++) {
        int idx = span - p + j; // 控制点索引
        if (idx >= 0 && idx < static_cast<int>(controlPoints_.size())) {
            // result += controlPoints_[idx] * ders[k][j];
            // ===== 修复4：检查无效值 =====
            if (std::isnan(ders[k][j])) {
                // 处理特殊情况：当导数为NaN时使用线性近似
                if (k == 1 && j > 0 && j < p) {
                    result += controlPoints_[idx] * 
                             ((ders[0][j] - ders[0][j-1]) / 
                             (knots_[span+j+1] - knots_[span+j]));
                } else {
                    result += controlPoints_[idx] * 0.0;
                }
            } else {
                result += controlPoints_[idx] * ders[k][j];
            }
        }
    }
    
    return result;
}


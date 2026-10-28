#include "SE3Interpolators.hpp"

SE3BezierInterpolator::SE3BezierInterpolator(
    const std::vector<Sophus::SE3d>& control_points)
    : control_points_(control_points) {}

Sophus::SE3d SE3BezierInterpolator::evaluate(double t) const {
    return deCasteljau(control_points_, t);
}

Sophus::SE3d SE3BezierInterpolator::deCasteljau(const std::vector<Sophus::SE3d>& points, double t) const {
    if (points.size() == 1) {
        return points[0];
    }
    
    std::vector<Sophus::SE3d> new_points;
    for (size_t i = 0; i < points.size() - 1; ++i) {
        new_points.push_back(
            SE3BaseInterp::interpolate(points[i], points[i+1], t)
        );
    }
    
    return deCasteljau(new_points, t);
}

// 预计算 n+1 个 Bernstein 系数 B(i,n,t)
static std::vector<double>
bernstein_coeffs(int n, double t) {
    std::vector<double> B(n + 1);
    for (int i = 0; i <= n; ++i)
        B[i] = std::tgamma(n + 1) /
               (std::tgamma(i + 1) * std::tgamma(n - i + 1)) *
               std::pow(t, i) * std::pow(1.0 - t, n - i);
    return B;
}

std::vector<double> bernstein_coeffs_ratio(int n, double t, const std::vector<double>&ratio) {
    std::vector<double> B(n + 1);
    for (int i = 0; i <= n; ++i)
        B[i] = std::tgamma(n + 1) /
               (std::tgamma(i + 1) * std::tgamma(n - i + 1)) *
               std::pow(t, i) * std::pow(1.0 - t, n - i)*ratio[i];
    return B;
}

Sophus::SE3d
SE3BezierInterpolator::bezier(const std::vector<Sophus::SE3d>& controls,
                        int n,
                        double t) const {
    if (n < 0 || controls.size() != static_cast<std::size_t>(n + 1))
        return Sophus::SE3d();          // 返回单位元
    const std::vector<double> B = bernstein_coeffs(n, t);
    Sophus::SE3d sum = Sophus::SE3d();  // 单位元
    
    for (int i = 0; i <= n; ++i)
        sum = sum * Sophus::SE3d::exp(B[i] * controls[i].log());
        
    return sum;
}

Sophus::SE3d
SE3BezierInterpolator::RationalBezier(const std::vector<Sophus::SE3d>& controls,
                        int n,
                        double t, const std::vector<double>& ratio) const {
    if (n < 0 || controls.size() != static_cast<std::size_t>(n + 1))
        return Sophus::SE3d();          // 返回单位元
    const std::vector<double> B = bernstein_coeffs_ratio(n, t, ratio);
    Eigen::Isometry3d Tmp = Eigen::Isometry3d::Identity();   // ✅ 官方推荐
   
    Sophus::SE3d sum_num(Tmp.rotation(), Tmp.translation());
    double sum_den = 0;
    for (int i = 0; i <= n; ++i){
        sum_num = sum_num * Sophus::SE3d::exp(B[i] * controls[i].log());
        sum_den += B[i];
    }
        
    sum_num = Sophus::SE3d::exp((1.0 / sum_den) * sum_num.log());
    return sum_num;
}

// template<typename Scalar = double>
// class RationalBezier {
// public:
//     /* 2 阶有理贝塞尔 **********************************************************/
//     template<typename Derived>
//     static Eigen::Matrix<Scalar, Derived::RowsAtCompileTime, 1>
//     eval2(Scalar t,
//           const std::array<Scalar, 3>& w,
//           const std::array<Eigen::MatrixBase<Derived>, 3>& r) {
//         const Scalar mt  = Scalar(1) - t;
//         const Scalar mt2 = mt * mt;
//         const Scalar t2  = t * t;

//         const Scalar f0 = mt2;
//         const Scalar f1 = Scalar(2) * mt * t;
//         const Scalar f2 = t2;

//         const Scalar basis = f0 + f1 + f2;
//         return (f0 * w[0] * r[0] + f1 * w[1] * r[1] + f2 * w[2] * r[2]) / basis;
//     }

//     /* 3 阶有理贝塞尔 **********************************************************/
//     template<typename Derived>
//     static Eigen::Matrix<Scalar, Derived::RowsAtCompileTime, 1>
//     eval3(Scalar t,
//           const std::array<Scalar, 4>& w,
//           const std::array<Eigen::MatrixBase<Derived>, 4>& r) {
//         const Scalar mt  = Scalar(1) - t;
//         const Scalar mt2 = mt * mt;
//         const Scalar mt3 = mt2 * mt;
//         const Scalar t2  = t * t;
//         const Scalar t3  = t2 * t;

//         const Scalar f0 = mt3;
//         const Scalar f1 = Scalar(3) * mt2 * t;
//         const Scalar f2 = Scalar(3) * mt * t2;
//         const Scalar f3 = t3;

//         const Scalar basis = f0 + f1 + f2 + f3;
//         return (f0 * w[0] * r[0] + f1 * w[1] * r[1] + f2 * w[2] * r[2] + f3 * w[3] * r[3]) / basis;
//     }
// };
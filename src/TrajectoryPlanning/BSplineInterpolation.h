#pragma once

#include <Eigen/Dense>
#include <vector>
#include <stdexcept>
#include <functional>
#include <bspline/Core.h>  // 包含bspline库的核心功能


template<int Dim>
class BSplineInterpolation {
public:
    // 参数化方法枚举
    enum class ParametrizationMethod {
        UNIFORM,      // 等距参数化
        CHORD_LENGTH, // 弦长参数化
        CENTRIPETAL   // 向心参数化
    };

    // 曲线函数类型
    using CurveFunction = std::function<Eigen::Matrix<double, Dim, 1>(double)>;

    /**
     * 构造函数（带时间参数）
     * @param points 路径点序列
     * @param p 样条阶数
     * @param times 点对应的参数化时间值（可选）
     */
    BSplineInterpolation(const std::vector<Eigen::Matrix<double, Dim, 1>>& points,
                         int p,
                         const std::vector<double>& times = {})
        : p_(p), points_(points) {
        if (points.size() < 2) {
            throw std::invalid_argument("At least 2 points required");
        }
        
        if (!times.empty()) {
            // 使用提供的时间参数
            if (times.size() != points.size()) {
                throw std::invalid_argument("Number of times must match number of points");
            }
            uBar_ = times;
        } else {
            // 计算弦长参数化
            uBar_ =  computeParametrization(points, ParametrizationMethod::CHORD_LENGTH);
        }
        
         // 计算并保存默认边界导数
        computeDefaultBoundaryDerivatives();

        // 构建节点向量
        knots_ = buildKnotVector(uBar_, p);

         // 打印参数值 uBar_
        std::cout << "\nB样条参数化信息 (uBar_):" << std::endl;
        std::cout << "途径点数: " << points.size() << std::endl;
        std::cout << "参数值序列: [";
        for (size_t i = 0; i < uBar_.size(); ++i) {
            std::cout << std::fixed << std::setprecision(4) << uBar_[i];
            if (i < uBar_.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        // 打印节点向量 knots_
        std::cout << "节点向量 (knots_):" << std::endl;
        std::cout << "阶数 p = " << p << std::endl;
        std::cout << "节点序列: [";
        for (size_t i = 0; i < knots_.size(); ++i) {
            std::cout << std::fixed << std::setprecision(4) << knots_[i];
            if (i < knots_.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    
    /**
     * 三次B样条曲线插值
     * 
     * @param points 插值点序列 (q0, q1, ..., qn)
     * @param t0 起点导数值 (可选)
     * @param tn 终点导数值 (可选)
     * @param method 参数化方法 (默认弦长参数化)
     */
    void interpolateCubic(
        const std::vector<Eigen::Matrix<double, Dim, 1>>& points,
        const Eigen::Matrix<double, Dim, 1>& t0 = Eigen::Matrix<double, Dim, 1>::Zero(),
        const Eigen::Matrix<double, Dim, 1>& tn = Eigen::Matrix<double, Dim, 1>::Zero(),
        ParametrizationMethod method = ParametrizationMethod::CHORD_LENGTH);
    

    void interpolateCubic(
    const Eigen::Matrix<double, Dim, 1>& t0,
    const Eigen::Matrix<double, Dim, 1>& tn);

     /**
     * 三次B样条插值（使用默认边界导数）
     */
    void interpolateCubic() {
        interpolateCubic(default_t0_, default_tn_);
    }

    /**
     * 获取控制点
     */
    const std::vector<Eigen::Matrix<double, Dim, 1>>& getControlPoints() const;
    
    /**
     * 获取基函数
     */
    const std::vector<bspline::Spline<double, 3>>& getBasisFunctions() const;

    /**
     * 获取Ubar
     */
    const std::vector<double>& getUbar() const;
    
    /**
     * 创建Ubar
     */
    CurveFunction createCurveFunction() const;

    /**
     * 直接求值函数
     */
    Eigen::Matrix<double, Dim, 1> evaluate(double u) const;
    Eigen::Matrix<double, Dim, 1> derivative(double u, unsigned int k, int p = 3) const;
    


private:
    int p_;  // 样条阶数
    std::vector<Eigen::Matrix<double, Dim, 1>> points_;  // 存储路径点
    std::vector<Eigen::Matrix<double, Dim, 1>> controlPoints_;
    std::vector<bspline::Spline<double, 3>> basisFunctions_;
    std::vector<double> knots_;   // 节点向量
    std::vector<double> uBar_;    // 参数化值
    Eigen::Matrix<double, Dim, 1> default_t0_;  // 默认起始点导数
    Eigen::Matrix<double, Dim, 1> default_tn_;  // 默认终点导数

    // 计算参数化值
    std::vector<double> computeParametrization(
        const std::vector<Eigen::Matrix<double, Dim, 1>>& points,
        ParametrizationMethod method) const;
    
    // 构建节点向量
    static std::vector<double> buildKnotVector(
        const std::vector<double>& uBar);
    /**
     * 构建节点向量
     * @param uBar 参数化值向量
     * @param p 样条阶数
     * @return 节点向量
     */
    static std::vector<double> buildKnotVector(
        const std::vector<double>& uBar, int p);
    
    void computeDefaultBoundaryDerivatives();
    // 计算边界控制点
    void computeBoundaryControlPoints(
        const std::vector<Eigen::Matrix<double, Dim, 1>>& points,
        const std::vector<double>& uBar,
        const Eigen::Matrix<double, Dim, 1>& t0,
        const Eigen::Matrix<double, Dim, 1>& tn,
        Eigen::Matrix<double, Dim, 1>& p0,
        Eigen::Matrix<double, Dim, 1>& p1,
        Eigen::Matrix<double, Dim, 1>& pn1,
        Eigen::Matrix<double, Dim, 1>& pn2) const;
    
    // 三对角方程组求解
    void solveTridiagonalSystem(
        const Eigen::VectorXd& diag,
        const Eigen::VectorXd& lower,
        const Eigen::VectorXd& upper,
        const Eigen::VectorXd& rhs,
        Eigen::VectorXd& solution);
    Eigen::VectorXd solveTridiagonal(
        const Eigen::VectorXd& diag,
        const Eigen::VectorXd& upper,
        const Eigen::VectorXd& lower,
        const Eigen::VectorXd& rhs) const;
    int findSpan(double u) const;
    void dersBasisFuns(double u, int span, int p, int n, 
                       std::vector<std::vector<double>>& ders) const;
    
    
};

#include "BSplineInterpolation_impl.h" // 模板实现
#pragma once
#include <vector>
#include <functional>
#include <Eigen/Dense>

#include "DoubleSTrajectoryPlanner.hpp"

template<int Dim>
class PathToTrajectory {
public:
    // 路径函数类型定义
    using PathFunction = std::function<Eigen::Matrix<double, Dim, 1>(double u)>;
    using DerivativeFunction = std::function<Eigen::Matrix<double, Dim, 1>(double u, int order)>;

    // 构造函数 - 添加 u_min 和 u_max 参数
    PathToTrajectory(PathFunction path_func, 
                     DerivativeFunction deriv_func,
                     double u_min = 0.0, 
                     double u_max = 1.0)
        : path_func_(path_func), 
          deriv_func_(deriv_func),
          u_min_(u_min),
          u_max_(u_max) {}

    // 设置运动参数
    void setMotionParameters(double v_max, double Ts, double v_c = 0) {
        v_max_ = v_max;
        Ts_ = Ts;
        v_c_ = (v_c > 0) ? v_c : v_max;
        calculateScalingFactor();
    }
    void setMotionParameters(double v_max, double a_max, double j_max, 
                             double Ts, double v_c = 0) {
        v_max_ = v_max;
        a_max_ = a_max;
        j_max_ = j_max;
        Ts_ = Ts;
        v_c_ = (v_c > 0) ? v_c : v_max;
    }

    // 计算缩放因子λ
    void calculateScalingFactor();

    // 轨迹生成方法
    void generateDirectParametrization(double t_total, 
                                       std::vector<double>& time_points,
                                       std::vector<double>& u_points,
                                       std::vector<Eigen::Matrix<double, Dim, 1>>& positions,
                                       std::vector<Eigen::Matrix<double, Dim, 1>>& velocities);
    
    void generateScaledParametrization(std::vector<double>& time_points,
                                       std::vector<double>& u_points,
                                       std::vector<Eigen::Matrix<double, Dim, 1>>& positions,
                                       std::vector<Eigen::Matrix<double, Dim, 1>>& velocities);
    
    void generateFirstOrderTaylor(std::vector<double>& time_points,
                                  std::vector<double>& u_points,
                                  std::vector<Eigen::Matrix<double, Dim, 1>>& positions,
                                  std::vector<Eigen::Matrix<double, Dim, 1>>& velocities);
    
    void generateSecondOrderTaylor(std::vector<double>& time_points,
                                   std::vector<double>& u_points,
                                   std::vector<Eigen::Matrix<double, Dim, 1>>& positions,
                                   std::vector<Eigen::Matrix<double, Dim, 1>>& velocities);

    

                         /**
     * @brief 计算参数曲线从 u_start 到 u_end 的弧长
     * 
     * @param u_start 起始参数
     * @param u_end 结束参数
     * @param steps 积分步数
     * @return double 弧长
     */
    double computeArcLength(double u_start, double u_end, int steps=1000) const;
    /**
     * @brief 生成通用进给速度曲线轨迹
     * 
     * @param T_s 采样时间
     * @param v_max 最大速度
     * @param a_max 最大加速度
     * @param j_max 最大加加速度
     * @param v_start 起始速度 (默认0)
     * @param v_end 结束速度 (默认0)
     * @return std::tuple<std::vector<double>, 
     *                    std::vector<typename PathType::PointType>,
     *                    std::vector<typename PathType::VectorType>,
     *                    std::vector<typename PathType::VectorType>>
     *         返回元组: [时间, 位置, 速度向量, 加速度向量]
     */
    void generateTrajectoryWithVariableFeedrate(
        std::vector<double>& time_points,
        std::vector<double>& u_points,
        std::vector<Eigen::Matrix<double, Dim, 1>>& positions,
        std::vector<Eigen::Matrix<double, Dim, 1>>& velocities,
        double v_start=0, double v_end=0) const;

private:
    PathFunction path_func_;      // 路径函数 p(u)
    DerivativeFunction deriv_func_; // 导数函数 dp/du, d²p/du²
    double u_min_ = 0.0;          // 参数u的最小值
    double u_max_ = 1.0;          // 参数u的最大值
    double v_max_ = 0.0;          // 最大速度限制
    double a_max_ = 0.0;          // 最大加速度限制
    double j_max_ = 0.0;          // 最大加加速度限制
    double v_c_ = 0.0;            // 期望恒定速度
    double Ts_ = 0.01;            // 采样时间
    double lambda_ = 1.0;          // 时间缩放因子
};

// 模板实现包含在头文件中
#include "PathToTrajectory.tpp"
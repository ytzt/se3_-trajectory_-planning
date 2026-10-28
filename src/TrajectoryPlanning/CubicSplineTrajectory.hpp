#pragma once

#include <vector>
#include <tuple>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

class CubicSplineTrajectory {
public:
    /**
     * @brief 三次样条轨迹生成器（带速度和加速度约束）
     * 
     * @param times 时间点序列 [t0, t1, ..., tn]
     * @param positions 位置点序列 [q0, q1, ..., qn]
     * @param v0 初始速度
     * @param vn 最终速度
     * @param a0 初始加速度 (默认为NaN，表示不约束)
     * @param an 最终加速度 (默认为NaN，表示不约束)
     */
    CubicSplineTrajectory(const std::vector<double>& times, 
                         const std::vector<double>& positions,
                         double v0, double vn,
                         double a0 = std::numeric_limits<double>::quiet_NaN(),
                         double an = std::numeric_limits<double>::quiet_NaN());
    
    /**
     * @brief 评估轨迹在特定时间点的状态
     * 
     * @param t 查询时间点
     * @return std::tuple<double, double, double> 
     *         位置, 速度, 加速度
     */
    std::tuple<double, double, double> evaluate(double t) const;

    /**
     * @brief 生成离散轨迹
     * 
     * @param Ts 采样时间
     * @return std::tuple<std::vector<double>, std::vector<double>, 
     *                    std::vector<double>, std::vector<double>> 
     *         时间, 位置, 速度, 加速度
     */
    std::tuple<std::vector<double>, std::vector<double>, 
               std::vector<double>, std::vector<double>> 
    generate(double Ts) const;

    // 获取额外点和加速度信息（用于调试）
    const std::vector<double>& get_extra_times() const { return extra_times_; }
    const std::vector<double>& get_extra_positions() const { return extra_positions_; }
    const std::vector<double>& get_accelerations() const { return accelerations_; }
    const std::vector<double>& get_velocities() const { return velocities_; }

private:
    std::vector<double> times_;
    std::vector<double> positions_;
    std::vector<double> velocities_;
    std::vector<std::vector<double>> coefficients_;
    
    // 加速度约束相关数据
    std::vector<double> extra_times_;
    std::vector<double> extra_positions_;
    std::vector<double> accelerations_;
    bool use_accel_constraints_ = false;

    // 三对角矩阵求解算法
    std::vector<double> solveTridiagonal(
        const std::vector<double>& diag,
        const std::vector<double>& upper,
        const std::vector<double>& lower,
        const std::vector<double>& rhs) const;

    // 计算带加速度约束的样条
    void computeWithAccelerationConstraints(
        const std::vector<double>& times,
        const std::vector<double>& positions,
        double v0, double vn,
        double a0, double an);

    // 计算标准速度约束样条
    void computeWithVelocityConstraints(
        const std::vector<double>& times,
        const std::vector<double>& positions,
        double v0, double vn);
};
#pragma once

#include <vector>
#include <cmath>
#include <stdexcept>
#include <tuple>

class DoubleSTrajectoryPlanner {
public:
    /**
     * @brief 构造函数，设置轨迹约束参数
     * 
     * @param v_max 最大速度（最小值自动设为 -v_max）
     * @param a_max 最大加速度（最小值自动设为 -a_max）
     * @param j_max 最大加加速度（最小值自动设为 -j_max）
     */
    DoubleSTrajectoryPlanner(double v_max, double a_max, double j_max);

    /**
     * @brief 生成双S轨迹
     * 
     * @param T_s 采样时间 (秒)
     * @param q_0 起始位置
     * @param q_1 终点位置
     * @param v_0 起始速度
     * @param v_1 终点速度
     * 
     * @return std::tuple<std::vector<double>, std::vector<double>, 
     *                    std::vector<double>, std::vector<double>> 
     *         包含四个向量的元组: [时间, 位置, 速度, 加速度,加加速度]
     */
    std::tuple<std::vector<double>, std::vector<double>, 
               std::vector<double>, std::vector<double>, std::vector<double>> 
    plan(double T_s, double q_0, double q_1, double v_0, double v_1) const;

private:
    // 约束参数
    const double vd_max_;
    const double vd_min_;
    const double ad_max_;
    const double ad_min_;
    const double jd_max_;
    const double jd_min_;

    // 辅助函数：符号函数
    static int sign(double value) {
        return (value > 0) ? 1 : ((value < 0) ? -1 : 0);
    }
};
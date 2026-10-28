#ifndef TRAPEZOIDAL_PLANNER_H
#define TRAPEZOIDAL_PLANNER_H

#include <vector>
#include <stdexcept>
#include <cmath>
#include <fstream>
#include <string>
#include <iostream>

#include <tuple>        // std::tuple, std::tuple_size

class TrapezoidalPlanner {
public:
    /**
     * @brief 构造函数
     * @param a_max 最大加速度（绝对值）
     * @param v_max 最大速度（绝对值）
     * @param dt 时间步长（默认0.01秒）
     */
    TrapezoidalPlanner(double a_max, double v_max, double dt = 0.01);
    
    /**
     * @brief 生成梯形速度轨迹
     * @param q0 初始位置
     * @param q1 终点位置
     * @param v0 初始速度
     * @param v1 终点速度
     * @return std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> 
     *         包含时间、位置、速度、加速度的元组
     */
    std::tuple<std::vector<double>, 
               std::vector<double>, 
               std::vector<double>, 
               std::vector<double>> 
    plan(double q0, double q1, double v0, double v1);

    // 多段轨迹规划（新增）
    std::tuple<std::vector<double>, 
               std::vector<double>, 
               std::vector<double>, 
               std::vector<double>> 
    planMultiSegment(const std::vector<double>& waypoints);
    
    /**
     * @brief 保存轨迹数据到CSV文件
     * @param file_path 文件路径
     */
    // void saveToCSV(const std::string& file_path);

   


private:
    double ad_max_;  // 最大加速度（绝对值）期望
    double vd_max_;  // 最大速度（绝对值）
    double ad_min_;  // 最大加速度（绝对值）期望
    double vd_min_;  // 最大速度（绝对值）
    double dt_;     // 时间步长
    
    // 轨迹数据
    std::vector<double> time_;
    std::vector<double> position_;
    std::vector<double> velocity_;
    std::vector<double> acceleration_;
    // 内部辅助函数（新增）
    double calculateIntermediateVelocity(size_t index, 
                                        const std::vector<double>& waypoints) const;
     // 辅助函数：符号函数
    static int sign(double value) {
        return (value > 0) ? 1 : ((value < 0) ? -1 : 0);
    }
};

#endif // TRAPEZOIDAL_PLANNER_H
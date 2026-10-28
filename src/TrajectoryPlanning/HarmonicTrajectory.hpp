#pragma once

#include <vector>
#include <cmath>
#include <tuple>

class HarmonicTrajectory {
public:
    /**
     * @brief 生成谐波轨迹
     * 
     * @param t0 起始时间
     * @param t1 结束时间
     * @param q0 起始位置
     * @param q1 结束位置
     * @param Ts 采样时间
     * 
     * @return std::tuple<std::vector<double>, std::vector<double>, 
     *                    std::vector<double>, std::vector<double>> 
     *         包含四个向量的元组: [时间, 位置, 速度, 加速度]
     */
    static std::tuple<std::vector<double>, std::vector<double>, 
                      std::vector<double>, std::vector<double>, std::vector<double>> 
    generate(double t0, double t1, double q0, double q1, double Ts);
};
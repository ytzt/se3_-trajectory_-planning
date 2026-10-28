#include "DoubleSTrajectoryPlanner.hpp"
#include <iostream>
#include <algorithm>
#include <tuple>
#include <cmath>

using namespace std;

// 构造函数初始化约束参数（最小值自动设为对应最大值的负数）
DoubleSTrajectoryPlanner::DoubleSTrajectoryPlanner(
    double v_max, double a_max, double j_max)
    : vd_max_(v_max), vd_min_(-v_max),
      ad_max_(a_max), ad_min_(-a_max),
      jd_max_(j_max), jd_min_(-j_max) {
    
    // 验证约束参数的合理性
    if (vd_max_ <= 0) {
        throw invalid_argument("最大速度必须大于0");
    }
    if (ad_max_ <= 0) {
        throw invalid_argument("最大加速度必须大于0");
    }
    if (jd_max_ <= 0) {
        throw invalid_argument("最大加加速度必须大于0");
    }
}

// 规划接口
tuple<vector<double>, vector<double>, vector<double>, vector<double>, vector<double>> 
DoubleSTrajectoryPlanner::plan(
    double T_s, double qd_0, double qd_1, double vd_0, double vd_1) const {
    
    // 验证输入参数
    if (T_s <= 0) {
        throw invalid_argument("采样时间必须大于0");
    }
    
    // BLOCK 1: 参数变换
    int sigma = sign(qd_1 - qd_0);
    
    double q_0 = sigma * qd_0;
    double q_1 = sigma * qd_1;
    double v_0 = sigma * vd_0;
    double v_1 = sigma * vd_1;
    
    double v_max = (sigma + 1) / 2.0 * vd_max_ + (sigma - 1) / 2.0 * vd_min_;
    double v_min = (sigma + 1) / 2.0 * vd_min_ + (sigma - 1) / 2.0 * vd_max_;
    double a_max = (sigma + 1) / 2.0 * ad_max_ + (sigma - 1) / 2.0 * ad_min_;
    double a_min = (sigma + 1) / 2.0 * ad_min_ + (sigma - 1) / 2.0 * ad_max_;
    double j_max = (sigma + 1) / 2.0 * jd_max_ + (sigma - 1) / 2.0 * jd_min_;
    double j_min = (sigma + 1) / 2.0 * jd_min_ + (sigma - 1) / 2.0 * jd_max_;

    // 初始化时间参数
    double Tj_1 = 0.0, Tj_2 = 0.0, Ta = 0.0, Td = 0.0, Tv = 0.0;
    double a_lim_a = 0.0, a_lim_d = 0.0, v_lim = 0.0;

    // 计算加速段时间
    if (((v_max - v_0) * j_max) < pow(a_max, 2)) {
        Tj_1 = sqrt((v_max - v_0) / j_max);
        Ta = 2 * Tj_1;
    } else {
        Tj_1 = a_max / j_max;
        Ta = Tj_1 + (v_max - v_0) / a_max;
    }

    // 计算减速段时间
    if (((v_max - v_1) * j_max) < pow(a_max, 2)) {
        Tj_2 = sqrt((v_max - v_1) / j_max);
        Td = 2 * Tj_2;
    } else {
        Tj_2 = a_max / j_max;
        Td = Tj_2 + (v_max - v_1) / a_max;
    }

    // 计算匀速段时间
    Tv = (q_1 - q_0) / v_max - Ta / 2.0 * (1 + v_0 / v_max) - Td / 2.0 * (1 + v_1 / v_max);

    // 处理未达到最大速度的情况
    if (Tv < 0) {
        Tv = 0;
        bool solutionFound = false;
        double original_a_max = a_max;
        double original_a_min = a_min;

        // 尝试降低加速度约束 (gamma从1降到0.001)
        for (double gamma = 1.0; gamma > 0.001; gamma -= 0.001) {
            a_max = gamma * original_a_max;
            a_min = gamma * original_a_min;

            Tj_1 = a_max / j_max;
            Tj_2 = a_max / j_max;
            double Tj = Tj_1;

            double delta = pow(a_max, 4) / pow(j_max, 2) + 2 * (pow(v_0, 2) + pow(v_1, 2)) +
                           a_max * (4 * (q_1 - q_0) - 2 * a_max / j_max * (v_0 + v_1));

            // 重新计算加速段和减速段时间
            Ta = (pow(a_max, 2) / j_max - 2 * v_0 + sqrt(delta)) / (2 * a_max);
            Td = (pow(a_max, 2) / j_max - 2 * v_1 + sqrt(delta)) / (2 * a_max);

            // 检查时间有效性
            if (Ta < 0) {
                Ta = 0;
                Td = 2 * (q_1 - q_0) / (v_1 + v_0);
                Tj_2 = (j_max * (q_1 - q_0) - 
                        sqrt(j_max * (j_max * pow(q_1 - q_0, 2) + 
                        pow(v_1 + v_0, 2) * (v_1 - v_0)))) / 
                       (j_max * (v_1 + v_0));
            } else if (Td < 0) {
                Td = 0;
                Ta = 2 * (q_1 - q_0) / (v_1 + v_0);
                Tj_1 = (j_max * (q_1 - q_0) - 
                        sqrt(j_max * (j_max * pow(q_1 - q_0, 2) - 
                        pow(v_1 + v_0, 2) * (v_1 - v_0)))) / 
                       (j_max * (v_1 + v_0));
            }

            // 检查是否满足时间约束
            if (Ta > 2 * Tj && Td > 2 * Tj) {
                solutionFound = true;
                break;
            }
        }

        if (!solutionFound) {
            throw runtime_error("无法找到有效的轨迹解决方案");
        }
    }

    // 计算轨迹参数
    a_lim_a = j_max * Tj_1;
    a_lim_d = -j_max * Tj_2;
    v_lim = v_0 + (Ta - Tj_1) * a_lim_a;
    double T = Ta + Tv + Td;

    // 离散化时间
    vector<double> time;
    for (double t = 0; t <= T; t += T_s) {
        time.push_back(t);
    }

    // 轨迹数据容器
    vector<double> qd, qdp, qdpp, qdppp;

    // 生成轨迹点
    for (double t : time) {
        double position = 0.0, velocity = 0.0, acceleration = 0.0, jerk = 0.0;

        // 加速段
        if (t <= Tj_1) {
            position = q_0 + v_0 * t + j_max * pow(t, 3) / 6.0;
            velocity = v_0 + j_max * pow(t, 2) / 2.0;
            acceleration = j_max * t;
            jerk = j_max;
        } 
        else if (t > Tj_1 && t <= (Ta - Tj_1)) {
            position = q_0 + v_0 * t + a_lim_a / 6.0 * (3 * pow(t, 2) - 3 * Tj_1 * t + pow(Tj_1, 2));
            velocity = v_0 + a_lim_a * (t - Tj_1 / 2.0);
            acceleration = j_max * Tj_1;
            jerk = 0;
        } 
        else if (t > (Ta - Tj_1) && t <= Ta) {
            position = q_0 + (v_lim + v_0) * Ta / 2.0 - v_lim * (Ta - t) - j_min * pow(Ta - t, 3) / 6.0;
            velocity = v_lim + j_min * pow(Ta - t, 2) / 2.0;
            acceleration = -j_min * (Ta - t);
            jerk = j_min;
        } 
        // 匀速段
        else if (t > Ta && t <= (Ta + Tv)) {
            position = q_0 + (v_lim + v_0) * Ta / 2.0 + v_lim * (t - Ta);
            velocity = v_lim;
            acceleration = 0;
            jerk = 0;
        } 
        // 减速段
        else if (t > (Ta + Tv) && t <= (Ta + Tv + Tj_2)) {
            double tau = t - (Ta + Tv);
            position = q_1 - (v_lim + v_1) * Td / 2.0 + v_lim * tau - j_max * pow(tau, 3) / 6.0;
            velocity = v_lim - j_max * pow(tau, 2) / 2.0;
            acceleration = -j_max * tau;
            jerk = j_min;
        } 
        else if (t > (Ta + Tv + Tj_2) && t <= (Ta + Tv + Td - Tj_2)) {
            double tau = t - (Ta + Tv);
            position = q_1 - (v_lim + v_1) * Td / 2.0 + v_lim * tau + 
                       a_lim_d / 6.0 * (3 * pow(tau, 2) - 3 * Tj_2 * tau + pow(Tj_2, 2));
            velocity = v_lim + a_lim_d * (tau - Tj_2 / 2.0);
            acceleration = -j_max * Tj_2;
            jerk = 0;
        } 
        else if (t > (Ta + Tv + Td - Tj_2) && t <= T) {
            double tau = T - t;
            position = q_1 - v_1 * tau - j_max * pow(tau, 3) / 6.0;
            velocity = v_1 + j_max * pow(tau, 2) / 2.0;
            acceleration = -j_max * tau;
            jerk = j_max;
        }
        // 终点
        // else if (t > T) {
        //     position = q_1;
        //     velocity = v_1;
        //     acceleration = 0;
        // }

        // 反变换
        qd.push_back(sigma * position);
        qdp.push_back(sigma * velocity);
        qdpp.push_back(sigma * acceleration);
        qdppp.push_back(sigma * jerk);
    }

    return make_tuple(time, qd, qdp, qdpp, qdppp);
}
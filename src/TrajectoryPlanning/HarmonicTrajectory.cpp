#include "HarmonicTrajectory.hpp"
#include <cmath>
#include <vector>
#include <tuple>
#include <stdexcept>

using namespace std;

tuple<vector<double>, vector<double>, vector<double>, vector<double>, vector<double>> 
HarmonicTrajectory::generate(double t0, double t1, double q0, double q1, double Ts) {
    // 验证参数有效性
    if (t1 <= t0) {
        throw invalid_argument("结束时间必须大于起始时间");
    }
    if (Ts <= 0) {
        throw invalid_argument("采样时间必须大于0");
    }
    
    // 计算轨迹参数
    const double h = q1 - q0;       // 位置变化量
    const double T = t1 - t0;       // 总时间
    const double pi = M_PI;         // π值
    
    // 预计算常数
    const double a1 = pi * h / (2 * T);
    const double a2 = pow(pi, 2) * h / (2 * pow(T, 2));
    const double a3 = pow(pi, 3) * h / (2 * pow(T, 3));
    
    // 生成时间序列
    vector<double> time;
    for (double t = t0; t <= t1; t += Ts) {
        time.push_back(t);
    }
    
    // 轨迹数据容器
    vector<double> positions;
    vector<double> velocities;
    vector<double> accelerations;
    vector<double> jerks;
    
    // 计算每个时间点的轨迹值
    for (double t : time) {
        double tau = t - t0;  // 相对于起始时间的时间偏移
        
        // 位置: q(t) = q0 + (h/2) * [1 - cos(π(t - t0)/T)]
        double position = q0 + (h / 2.0) * (1 - cos(pi * tau / T));
        
        // 速度: q̇(t) = (πh/2T) * sin(π(t - t0)/T)
        double velocity = a1 * sin(pi * tau / T);
        
        // 加速度: q̈(t) = (π²h/2T²) * cos(π(t - t0)/T)
        double acceleration = a2 * cos(pi * tau / T);
        
        // 加加速度: q³(t) = -(π³h/2T³) * sin(π(t - t0)/T)
        double jerk = -a3 * sin(pi * tau / T);
        
        positions.push_back(position);
        velocities.push_back(velocity);
        accelerations.push_back(acceleration);
        jerks.push_back(jerk);
    }
    
    return make_tuple(time, positions, velocities, accelerations, jerks);
}
#include "CycloidalTrajectory.hpp"
#include <cmath>
#include <vector>
#include <tuple>
#include <stdexcept>

using namespace std;

tuple<vector<double>, vector<double>, vector<double>, vector<double>, vector<double>> 
CycloidalTrajectory::generate(double t0, double t1, double q0, double q1, double Ts) {
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
    const double two_pi = 2.0 * pi; // 2π值
    
    // 预计算常数
    const double v_factor = h / T;
    const double a_factor = two_pi * h / (T * T);
    const double j_factor = 4.0 * pi * pi * h / (T * T * T);
    
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
        double theta = two_pi * tau / T;  // 角度参数
        
        // 位置: q(t) = q0 + h * [ (tau/T) - (1/(2π)) * sin(2π*tau/T) ]
        double position = q0 + h * (tau/T - sin(theta) / (two_pi));
        
        // 速度: q̇(t) = (h / T) * [1 - cos(2π*tau/T)]
        double velocity = v_factor * (1 - cos(theta));
        
        // 加速度: q̈(t) = (2πh / T²) * sin(2π*tau/T)
        double acceleration = a_factor * sin(theta);
        
        // 加加速度: q³(t) = (4π²h / T³) * cos(2π*tau/T)
        double jerk = j_factor * cos(theta);
        
        positions.push_back(position);
        velocities.push_back(velocity);
        accelerations.push_back(acceleration);
        jerks.push_back(jerk);
    }
    
    return make_tuple(time, positions, velocities, accelerations, jerks);
}
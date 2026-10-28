
#include "InterpPlanner.h"
#include <cmath>
#include <iostream>

// 一阶多项式插值 (线性插值)
std::vector<double> InterpPlannerBase::linearInterp(double q0, double q1, double T) {
    if (T <= 0) throw std::invalid_argument("Time interval T must be positive");
    
    std::vector<double> coeffs(2);
    coeffs[0] = q0;          // a0
    coeffs[1] = (q1 - q0)/T; // a1
    
    return coeffs;
}

// 二次多项式插值
std::vector<double> InterpPlannerBase::quadraticInterp(double q0, double v0, double q1, double T) {
    if (T <= 0) throw std::invalid_argument("Time interval T must be positive");
    
    double h = q1 - q0;
    std::vector<double> coeffs(3);
    coeffs[0] = q0;
    coeffs[1] = v0;
    coeffs[2] = (3*h - 2*v0*T) / (T*T);
    
    return coeffs;
}

// 三次多项式插值
std::vector<double> InterpPlannerBase::cubicInterp(double q0, double v0, double q1, double v1, double T) {
    if (T <= 0) throw std::invalid_argument("Time interval T must be positive");
    
    double h = q1 - q0;
    std::vector<double> coeffs(4);
    coeffs[0] = q0;
    coeffs[1] = v0;
    coeffs[2] = (3*h - (2*v0 + v1)*T) / (T*T);
    coeffs[3] = (-2*h + (v0 + v1)*T) / (T*T*T);
    
    return coeffs;
}

// 五次多项式插值
std::vector<double> InterpPlannerBase::quinticInterp(
    double q0, double v0, double acc0, 
    double q1, double v1, double acc1, 
    double T) 
{
    if (T <= 0) throw std::invalid_argument("Time interval T must be positive");
    
    double h = q1 - q0;
    std::vector<double> coeffs(6);
    coeffs[0] = q0;
    coeffs[1] = v0;
    coeffs[2] = acc0 / 2.0;
    
    double T2 = T * T;
    double T3 = T * T2;
    double T4 = T * T3;
    double T5 = T * T4;
    
    coeffs[3] = (20*h - (8*v1 + 12*v0)*T - (3*acc0 - acc1)*T2) / (2*T3);
    coeffs[4] = (-30*h + (14*v1 + 16*v0)*T + (3*acc0 - 2*acc1)*T2) / (2*T4);
    coeffs[5] = (12*h - 6*(v0 + v1)*T + (acc1 - acc0)*T2) / (2*T5);
    
    return coeffs;
}

// 计算多项式状态
std::vector<double> InterpPlannerBase::evalPolynomial(const std::vector<double>& coeffs, double t) {
    if (coeffs.empty()) throw std::invalid_argument("Coefficient vector is empty");
    
    double pos = 0.0;
    double vel = 0.0;
    double acc = 0.0;
    
    for (size_t i = 0; i < coeffs.size(); ++i) {
        pos += coeffs[i] * std::pow(t, i);
    }
    
    for (size_t i = 1; i < coeffs.size(); ++i) {
        vel += i * coeffs[i] * std::pow(t, i-1);
    }
    
    for (size_t i = 2; i < coeffs.size(); ++i) {
        acc += i * (i-1) * coeffs[i] * std::pow(t, i-2);
    }
    
    return {pos, vel, acc};
}

// 将结果写入文件
void InterpPlannerBase::writeToFile(const std::string& filename, 
                               const std::vector<double>& timePoints,
                               const std::vector<double>& positions,
                               const std::vector<double>& velocities,
                               const std::vector<double>& accelerations) 
{
    std::ofstream outFile(filename);
    if (!outFile) {
        throw std::runtime_error("无法打开文件: " + filename);
    }
    
    // 写入表头
    outFile << "Time,Position,Velocity,Acceleration\n";
    
    // 设置精度
    outFile << std::fixed << std::setprecision(6);
    
    // 写入数据
    for (size_t i = 0; i < timePoints.size(); ++i) {
        outFile << timePoints[i] << ","
                << positions[i] << ","
                << velocities[i] << ","
                << accelerations[i] << "\n";
    }
    
    outFile.close();
    std::cout << "数据已保存到: " << filename << std::endl;
}

void InterpPlannerBase::writeToFile(const std::string& filename,
                                    const std::vector<double>& timePoints,
                                    const std::vector<double>& positions,
                                    const std::vector<double>& velocities,
                                    const std::vector<double>& accelerations,
                                    const std::vector<double>& jerks)
{
    if (!(timePoints.size() == positions.size() &&
          positions.size() == velocities.size() &&
          velocities.size() == accelerations.size() &&
          accelerations.size() == jerks.size()))
        throw std::invalid_argument("所有输入向量长度必须一致");

    std::ofstream outFile(filename);
    if (!outFile)
        throw std::runtime_error("无法打开文件: " + filename);

    outFile << "Time,Position,Velocity,Acceleration,Jerk\n";
    outFile << std::fixed << std::setprecision(6);

    for (size_t i = 0; i < timePoints.size(); ++i) {
        outFile << timePoints[i]        << ','
                << positions[i]         << ','
                << velocities[i]        << ','
                << accelerations[i]     << ','
                << jerks[i]             << '\n';
    }
    outFile.close();
    std::cout << "数据（含 jerk）已保存到: " << filename << std::endl;
}
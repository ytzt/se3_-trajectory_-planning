#ifndef INTERP_PLANNER_H
#define INTERP_PLANNER_H

#include <vector>
#include <stdexcept>
#include <string>
#include <fstream>
#include <iomanip>

class InterpPlannerBase {
public:
    // 一阶多项式插值 (线性插值)
    static std::vector<double> linearInterp(double q0, double q1, double T);
    
    // 二次多项式插值
    static std::vector<double> quadraticInterp(double q0, double v0, double q1, double T);
    
    // 三次多项式插值
    static std::vector<double> cubicInterp(double q0, double v0, double q1, double v1, double T);
    
    // 五次多项式插值
    static std::vector<double> quinticInterp(double q0, double v0, double acc0, 
                                             double q1, double v1, double acc1, 
                                             double T);
    
    // 计算多项式状态
    static std::vector<double> evalPolynomial(const std::vector<double>& coeffs, double t);
    
    // 将结果写入文件
    static void writeToFile(const std::string& filename, 
                           const std::vector<double>& timePoints,
                           const std::vector<double>& positions,
                           const std::vector<double>& velocities,
                           const std::vector<double>& accelerations);
    static void writeToFile(const std::string& filename,
                                    const std::vector<double>& timePoints,
                                    const std::vector<double>& positions,
                                    const std::vector<double>& velocities,
                                    const std::vector<double>& accelerations,
                                    const std::vector<double>& jerks);
};

#endif // INTERP_PLANNER_H
#include "CubicSplineTrajectory.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>

using namespace std;

// CubicSplineTrajectory::CubicSplineTrajectory(const vector<double>& times,
//                                             const vector<double>& positions,
//                                             double v0, double vn)
//     : times_(times), positions_(positions) {
    
//     // 验证输入参数
//     if (times.size() != positions.size()) {
//         throw invalid_argument("时间点和位置点数量必须相同");
//     }
//     if (times.size() < 2) {
//         throw invalid_argument("至少需要两个点定义轨迹");
//     }
//     for (size_t i = 1; i < times.size(); ++i) {
//         if (times[i] <= times[i-1]) {
//             throw invalid_argument("时间点必须严格递增");
//         }
//     }

//     const size_t n = times.size() - 1;  // 段数
//     const size_t m = times.size();      // 点数

//     // 计算时间间隔 T_k = t_{k+1} - t_k
//     vector<double> T;
//     for (size_t i = 0; i < n; ++i) {
//         T.push_back(times[i+1] - times[i]);
//     }

//     // 初始化速度向量 (包括已知的v0和vn)
//     velocities_.resize(m);
//     velocities_[0] = v0;
//     velocities_[m-1] = vn;

//     // 单段情况 (不需要求解方程组)
//     if (n == 1) {
//         const double h = positions[1] - positions[0];
        
//         // 计算系数
//         vector<double> coeffs(4);
//         coeffs[0] = positions[0];  // a0
//         coeffs[1] = v0;            // a1
//         coeffs[2] = (3*h - (2*v0 + vn)*T[0]) / (T[0]*T[0]);  // a2
//         coeffs[3] = (-2*h + (v0 + vn)*T[0]) / (T[0]*T[0]*T[0]);  // a3
        
//         coefficients_.push_back(coeffs);
//         return;
//     }

//     // 构建三对角矩阵 A 和右侧向量 c
//     vector<double> diag(n-1, 0.0);    // 主对角线
//     vector<double> upper(n-2, 0.0);   // 上对角线
//     vector<double> lower(n-2, 0.0);   // 下对角线
//     vector<double> rhs(n-1, 0.0);     // 右侧向量

//     // 填充矩阵和右侧向量
//     for (size_t i = 0; i < n-1; ++i) {
//         // 对角线元素: 2(T_i + T_{i+1})
//         diag[i] = 2.0 * (T[i] + T[i+1]);
        
//         // 右侧向量: 
//         // 3 * [T_i^2 (q_{i+2}-q_{i+1}) + T_{i+1}^2 (q_{i+1}-q_i)] / (T_i T_{i+1})
//         double term1 = T[i]*T[i] * (positions[i+2] - positions[i+1]);
//         double term2 = T[i+1]*T[i+1] * (positions[i+1] - positions[i]);
//         rhs[i] = 3.0 * (term1 + term2) / (T[i] * T[i+1]);
//     }

//     // 添加边界条件
//     rhs[0] -= T[1] * v0;          // 减去 T_{k+1} v0
//     rhs[n-2] -= T[n-2] * vn;      // 减去 T_k vn

//     // 填充上下对角线
//     for (size_t i = 0; i < n-2; ++i) {
//         upper[i] = T[i];        // T_{k+1} (注意索引对应关系)
//         lower[i] = T[i+2];        // T_{k+1} (注意索引对应关系)
//     }

//     // 求解三对角方程组得到中间速度 v1, v2, ..., v_{n-1}
//     vector<double> v_mid = solveTridiagonal(diag, upper, lower, rhs);
    
//     // 将求解的速度存入速度向量
//     for (size_t i = 0; i < n-1; ++i) {
//         velocities_[i+1] = v_mid[i];
//     }

//     // 计算每段的样条系数
//     for (size_t k = 0; k < n; ++k) {
//         const double h_k = positions[k+1] - positions[k];
//         const double v_k = velocities_[k];
//         const double v_k1 = velocities_[k+1];
//         const double T_k = T[k];

//         vector<double> coeffs(4);
//         coeffs[0] = positions[k];                     // a0
//         coeffs[1] = v_k;                              // a1
//         coeffs[2] = (3*h_k - (2*v_k + v_k1)*T_k) / (T_k*T_k);  // a2
//         coeffs[3] = (-2*h_k + (v_k + v_k1)*T_k) / (T_k*T_k*T_k); // a3

//         coefficients_.push_back(coeffs);
//     }
// }

CubicSplineTrajectory::CubicSplineTrajectory(const vector<double>& times,
                                            const vector<double>& positions,
                                            double v0, double vn,
                                            double a0, double an)
    : times_(times), positions_(positions) {
    
    // 验证输入参数
    if (times.size() != positions.size()) {
        throw invalid_argument("时间点和位置点数量必须相同");
    }
    if (times.size() < 2) {
        throw invalid_argument("至少需要两个点定义轨迹");
    }
    for (size_t i = 1; i < times.size(); ++i) {
        if (times[i] <= times[i-1]) {
            throw invalid_argument("时间点必须严格递增");
        }
    }

    // 检查是否指定初始加速度
    use_accel_constraints_ = !(isnan(a0) || isnan(an));
    
    if (use_accel_constraints_) {
        computeWithAccelerationConstraints(times, positions, v0, vn, a0, an);
    } else {
        computeWithVelocityConstraints(times, positions, v0, vn);
    }
}

void CubicSplineTrajectory::computeWithAccelerationConstraints(
    const std::vector<double>& times,
    const std::vector<double>& positions,
    double v0, double vn,
    double a0, double an) {
    
    const size_t n_original = positions.size();
    if (n_original < 3) {
        throw invalid_argument("加速度约束需要至少3个点");
    }

    // 1. 计算额外点的时间（首尾区间中点）
    const double t_extra1 = (times[0] + times[1]) / 2.0;
    const double t_extra2 = (times[n_original-2] + times[n_original-1]) / 2.0;

    // 2. 构建新的时间序列
    vector<double> new_times;
    new_times.push_back(times[0]);      // t0
    new_times.push_back(t_extra1);      // 额外点1
    for (size_t i = 1; i < n_original-1; ++i) {
        new_times.push_back(times[i]);  // t2, t3, ..., tn-2
    }
    new_times.push_back(t_extra2);      // 额外点2
    new_times.push_back(times.back());  // tn

    // 3. 初始化新位置序列
    vector<double> new_positions(new_times.size(), 0.0);
    new_positions[0] = positions[0];                // q0
    for (size_t i = 1; i < n_original-1; ++i) {
        new_positions[i+1] = positions[i];          // q2, q3, ..., qn-2
    }
    new_positions[new_positions.size()-1] = positions.back(); // qn

    // 4. 计算新区间长度
    vector<double> T_new;
    for (size_t i = 0; i < new_times.size()-1; ++i) {
        T_new.push_back(new_times[i+1] - new_times[i]);
    }

    // 5. 设置中间加速度向量大小 (n_original+2-2个中间加速度)
    const size_t n = new_times.size() - 1;
    const size_t n_accel = n - 1;
    accelerations_.resize(n_accel, 0.0);

    // 6. 构建线性方程组 Aω = c
    vector<double> diag(n_accel, 0.0);
    vector<double> upper(n_accel-1, 0.0);
    vector<double> lower(n_accel-1, 0.0);
    vector<double> rhs(n_accel, 0.0);

    // 填充矩阵和右侧向量 (根据公式4.28)
    for (size_t k = 0; k < n_accel; ++k) {
        if (k == 0) {
            // 第一个方程
            diag[k] = 2*T_new[1] + T_new[0]*(3 + T_new[0]/T_new[1]);
            if (n_accel > 1){
                upper[k] = T_new[1];
                lower[k] = T_new[1] - T_new[0]*T_new[0]/T_new[1];
            } 
            
            double term1 = (new_positions[2] - new_positions[0]) / T_new[1];
           
            rhs[k] = 6 * (term1 - v0*(1 + T_new[0]/T_new[1]) - a0*(T_new[0]/2 + T_new[0]*T_new[0]/(3*T_new[1])));
        }
        else if (k == n_accel-1) {
            // 最后一个方程
            diag[k] = 2*T_new[n-2] + T_new.back()*(3 + T_new.back()/T_new[n-2]);
            if (n_accel > 1){
                lower[k-1] = T_new[n-2];
                upper[k-1] = T_new[n-2] - T_new[n-1]*T_new[n-1]/T_new[n-2];
            } 
            
            double term1 = (new_positions[n-2] - new_positions[n]) / T_new[n-2];

            rhs[k] = 6 * (term1 + vn*(1 + T_new.back()/T_new[n-2]) - an*(T_new.back()/2 + T_new.back()*T_new.back()/(3*T_new[n-2])));
        }
        else {
            // 中间方程
            diag[k] = 2*(T_new[k] + T_new[k+1]);
            if (k <= n_accel-2){
                upper[k] = T_new[k+1];
                lower[k] = T_new[k+1];

            }
            if(k==1){
                double term1 = (new_positions[3] - new_positions[2]) / T_new[2];
                double term2 = (new_positions[2] - new_positions[0]) / T_new[1];
                rhs[k] = 6 * (term1 - term2 + v0*(T_new[0]/T_new[1]) + a0*(T_new[0]*T_new[0]/(3*T_new[1])));

            }else if(k == n_accel-2){
               
                double term1 = (new_positions[n] - new_positions[n-2]) / T_new[n-2];
                double term2 = (new_positions[n-2] - new_positions[n-3]) / T_new[n-3];
                rhs[k] = 6 * (term1 - term2 - vn*(T_new[n-1]/T_new[n-2]) + an*(T_new[n-1]*T_new[n-1]/(3*T_new[n-2])));

            }else{
                double term1 = (new_positions[k+2] - new_positions[k+1]) / T_new[k+1];
                double term2 = (new_positions[k+1] - new_positions[k]) / T_new[k];
                rhs[k] = 6 * (term1 - term2);
            }

            
        }
    }

    // 7. 求解加速度
    accelerations_ = solveTridiagonal(diag, upper, lower, rhs);

    // 8. 计算额外点位置 (公式4.26和4.27)
    // 第一个额外点
    new_positions[1] = positions[0] 
        + T_new[0]*v0 
        + (T_new[0]*T_new[0]/3)*a0 
        + (T_new[0]*T_new[0]/6)*accelerations_[0];
    
    // 最后一个额外点
    size_t last_idx = new_positions.size()-2;
    new_positions[last_idx] = positions.back() 
        - T_new.back()*vn 
        + (T_new.back()*T_new.back()/3)*an 
        + (T_new.back()*T_new.back()/6)*accelerations_.back();

    // 9. 保存额外点信息
    extra_times_ = {new_times[1], new_times[new_times.size()-2]};
    extra_positions_ = {new_positions[1], new_positions[last_idx]};

    // 10. 直接计算多项式系数
    times_ = new_times;
    positions_ = new_positions;
    velocities_.clear();
    coefficients_.clear();

    // 创建完整的加速度向量（包括边界加速度）
    vector<double> omega_full;
    omega_full.push_back(a0);  // 初始加速度
    omega_full.insert(omega_full.end(), accelerations_.begin(), accelerations_.end());
    omega_full.push_back(an);  // 最终加速度

    // 计算每个区间的多项式系数
    const size_t n_segments = times_.size() - 1;
    for (size_t k = 0; k < n_segments; k++) {
        const double T_k = times_[k+1] - times_[k];
        const double q_k = positions_[k];
        const double q_k1 = positions_[k+1];
        const double omega_k = omega_full[k];
        const double omega_k1 = omega_full[k+1];

        vector<double> coeffs(4);
        coeffs[0] = q_k;  // a0
        coeffs[1] = (q_k1 - q_k)/T_k - T_k*(omega_k1 + 2*omega_k)/6.0;  // a1
        coeffs[2] = omega_k / 2.0;  // a2
        coeffs[3] = (omega_k1 - omega_k)/(6.0*T_k);  // a3

        coefficients_.push_back(coeffs);
    }
}
    

void CubicSplineTrajectory::computeWithVelocityConstraints(
    const vector<double>& times,
    const vector<double>& positions,
    double v0, double vn) 
{
    
    const size_t n = times.size() - 1;
    const size_t m = times.size();

    // 计算时间间隔
    vector<double> T;
    for (size_t i = 0; i < n; ++i) {
        T.push_back(times[i+1] - times[i]);
    }

    // 初始化速度向量
    velocities_.resize(m);
    velocities_[0] = v0;
    velocities_[m-1] = vn;

    // 单段情况
    if (n == 1) {
        const double h = positions[1] - positions[0];
        vector<double> coeffs(4);
        coeffs[0] = positions[0];
        coeffs[1] = v0;
        coeffs[2] = (3*h - (2*v0 + vn)*T[0]) / (T[0]*T[0]);
        coeffs[3] = (-2*h + (v0 + vn)*T[0]) / (T[0]*T[0]*T[0]);
        coefficients_.push_back(coeffs);
        return;
    }

    // 构建三对角矩阵
    vector<double> diag(n-1, 0.0);
    vector<double> upper(n-2, 0.0);
    vector<double> lower(n-2, 0.0);
    vector<double> rhs(n-1, 0.0);

    for (size_t i = 0; i < n-1; ++i) {
        diag[i] = 2.0 * (T[i] + T[i+1]);
        double term1 = T[i]*T[i] * (positions[i+2] - positions[i+1]);
        double term2 = T[i+1]*T[i+1] * (positions[i+1] - positions[i]);
        rhs[i] = 3.0 * (term1 + term2) / (T[i] * T[i+1]);
    }

    rhs[0] -= T[1] * v0;
    rhs[n-2] -= T[n-2] * vn;

    for (size_t i = 0; i < n-2; ++i) {
        upper[i] = T[i];
        lower[i] = T[i+2];
    }

    vector<double> v_mid = solveTridiagonal(diag, upper, lower, rhs);
    for (size_t i = 0; i < n-1; ++i) {
        velocities_[i+1] = v_mid[i];
    }

    // 计算样条系数
    for (size_t k = 0; k < n; ++k) {
        const double h_k = positions[k+1] - positions[k];
        const double v_k = velocities_[k];
        const double v_k1 = velocities_[k+1];
        const double T_k = T[k];

        vector<double> coeffs(4);
        coeffs[0] = positions[k];
        coeffs[1] = v_k;
        coeffs[2] = (3*h_k - (2*v_k + v_k1)*T_k) / (T_k*T_k);
        coeffs[3] = (-2*h_k + (v_k + v_k1)*T_k) / (T_k*T_k*T_k);
        coefficients_.push_back(coeffs);
    }
}

// 三对角矩阵求解算法 (Thomas算法)
std::vector<double> CubicSplineTrajectory::solveTridiagonal(
    const std::vector<double>& diag,
    const std::vector<double>& upper,
    const std::vector<double>& lower,
    const std::vector<double>& rhs) const {
    
    const size_t n = diag.size();
    if (n == 0) return {};
    if (n == 1) return {rhs[0] / diag[0]};

    // 创建可修改的矩阵和向量副本
    std::vector<double> A_diag = diag;
    std::vector<double> A_upper = upper;
    std::vector<double> A_lower = lower;
    std::vector<double> d = rhs;
    std::vector<double> x(n, 0.0);

    // 前向消元
    for (size_t i = 1; i < n; ++i) {
        double w = A_lower[i-1] / A_diag[i-1];
        A_diag[i] = A_diag[i] - w * A_upper[i-1];
        d[i] = d[i] - w * d[i-1];
    }

    // 回代
    x[n-1] = d[n-1] / A_diag[n-1];
    for (int i = n-2; i >= 0; --i) {
        x[i] = (d[i] - A_upper[i] * x[i+1]) / A_diag[i];
    }

    return x;
}

// 评估特定时间点的轨迹状态
tuple<double, double, double> CubicSplineTrajectory::evaluate(double t) const {
    // 查找时间点所在的段
    if (t <= times_.front()) {
        t = times_.front();  // 限制在起始时间
    } else if (t >= times_.back()) {
        t = times_.back();   // 限制在结束时间
    }

    // 二分查找确定所在段
    auto it = upper_bound(times_.begin(), times_.end(), t);
    size_t segment = (it == times_.begin()) ? 0 : distance(times_.begin(), it) - 1;
    if (segment >= coefficients_.size()) {
        segment = coefficients_.size() - 1;
    }

    // 计算相对于段起始时间的时间偏移
    const double tau = t - times_[segment];
    const auto& coeffs = coefficients_[segment];

    // 计算位置、速度、加速度
    double position = coeffs[0] + coeffs[1]*tau + coeffs[2]*tau*tau + coeffs[3]*tau*tau*tau;
    double velocity = coeffs[1] + 2*coeffs[2]*tau + 3*coeffs[3]*tau*tau;
    double acceleration = 2*coeffs[2] + 6*coeffs[3]*tau;

    return make_tuple(position, velocity, acceleration);
}

// 生成离散轨迹
tuple<vector<double>, vector<double>, vector<double>, vector<double>> 
CubicSplineTrajectory::generate(double Ts) const {
    if (Ts <= 0) {
        throw invalid_argument("采样时间必须大于0");
    }

    vector<double> time_vec;
    vector<double> pos_vec;
    vector<double> vel_vec;
    vector<double> acc_vec;

    // 生成时间序列
    for (double t = times_.front(); t <= times_.back(); t += Ts) {
        time_vec.push_back(t);
        auto [pos, vel, acc] = evaluate(t);
        pos_vec.push_back(pos);
        vel_vec.push_back(vel);
        acc_vec.push_back(acc);
    }

    // 确保包含终点
    if (time_vec.empty() || time_vec.back() < times_.back()) {
        time_vec.push_back(times_.back());
        auto [pos, vel, acc] = evaluate(times_.back());
        pos_vec.push_back(pos);
        vel_vec.push_back(vel);
        acc_vec.push_back(acc);
    }

    return make_tuple(time_vec, pos_vec, vel_vec, acc_vec);
}
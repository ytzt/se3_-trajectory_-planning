#include "TrapezoidalPlanner.hpp"

TrapezoidalPlanner::TrapezoidalPlanner(double a_max, double v_max, double dt)
    : vd_max_(v_max), vd_min_(-v_max),
      ad_max_(a_max), ad_min_(-a_max), dt_(dt) 
{
    if (ad_max_ <= 0) {
        throw std::invalid_argument("加速度必须为正数");
    }
    if (vd_max_ <= 0) {
        throw std::invalid_argument("速度必须为正数");
    }
    if (dt_ <= 0) {
        throw std::invalid_argument("时间步长必须为正数");
    }
}

std::tuple<std::vector<double>, 
           std::vector<double>, 
           std::vector<double>, 
           std::vector<double>> 
TrapezoidalPlanner::plan(double q0, double q1, double v0, double v1) {
    // 清空之前的数据
    time_.clear();
    position_.clear();
    velocity_.clear();
    acceleration_.clear();
    
    // 计算位移和方向
    
    // BLOCK 1: 参数变换
    int sigma = sign(q1 - q0);
    
    q0 = sigma * q0;
    q1 = sigma * q1;
    v0 = sigma * v0;
    v1 = sigma * v1;
    
    double v_max = (sigma + 1) / 2.0 * vd_max_ + (sigma - 1) / 2.0 * vd_min_;
    double v_min = (sigma + 1) / 2.0 * vd_min_ + (sigma - 1) / 2.0 * vd_max_;
    double a_max = (sigma + 1) / 2.0 * ad_max_ + (sigma - 1) / 2.0 * ad_min_;
    double a_min = (sigma + 1) / 2.0 * ad_min_ + (sigma - 1) / 2.0 * ad_max_;
    
    double h = q1 - q0;
    double T_a, T_d, T_c, T_total;
    double v_v;  // 实际达到的最大速度
    
    // 判断两种情况
    bool condition = (a_max * h) > (v_max * v_max - (v0*v0 + v1*v1)/2.0);
    
    if (condition) {
        // Case 1: 能达到最大速度（有匀速段）
        v_v = v_max;
        T_a = (v_v - v0) / a_max;
        T_d = (v_v - v1) / a_max;
        
        // 计算各段位移
        double S_a = (v0 + v_v) * T_a / 2.0;
        double S_d = (v_v + v1) * T_d / 2.0;
        double S_c = h - S_a - S_d;
        
        // 匀速段时间
        T_c = S_c / v_v;
        T_total = T_a + T_c + T_d;
        
        // 验证时间参数
        if (T_a < 0 || T_d < 0 || T_c < 0) {
            throw std::runtime_error("无效的时间参数（T_a, T_d, T_c）");
        }
    } else {
        // Case 2: 无法达到最大速度（无匀速段）
        v_v = std::sqrt((2.0 * a_max * h + v0*v0 + v1*v1) / 2.0);
        T_a = (v_v - v0) / a_max;
        T_d = (v_v - v1) / a_max;
        T_c = 0.0;
        T_total = T_a + T_d;
        
        // 验证速度约束
        if (v0 > v_v || v1 > v_v) {
            throw std::invalid_argument("初始速度或终止速度大于可达到的最大速度");
        }
    }
    
    // 生成时间序列
    for (double t = 0.0; t <= T_total; t += dt_) {
        double pos, vel, acc;
        
        if (t < T_a) {
            // 加速段
            acc = a_max ;
            vel = v0 + acc * t;
            pos = q0 + v0 * t + 0.5 * acc * t * t;
        } else if (t < T_a + T_c) {
            // 匀速段
            acc = 0.0;
            vel = v_v ;
            // 加速段结束时的位置
            double pos_acc_end = q0 + v0 * T_a + 0.5 * (a_max ) * T_a * T_a;
            pos = pos_acc_end + vel * (t - T_a);
        } else {
            // 减速段
            double t_dec = t - (T_a + T_c);  // 减速段内的时间
            acc = -a_max ;
            vel = v_v  + acc * t_dec;
            // 匀速段结束时的位置
            double pos_const_end = q0 + v0 * T_a + 0.5 * (a_max ) * T_a * T_a + (v_v ) * T_c;
            pos = pos_const_end + (v_v ) * t_dec + 0.5 * acc * t_dec * t_dec;
        }
        
        time_.push_back(t);
        position_.push_back(sigma*pos);
        velocity_.push_back(sigma*vel);
        acceleration_.push_back(sigma*acc);
    }
    
    // // 确保包含终点
    // if (time_.empty() || time_.back() < T_total) {
    //     time_.push_back(T_total);
    //     position_.push_back(q1);
    //     velocity_.push_back(v1);
    //     acceleration_.push_back(0.0);
    // }
    
    return std::make_tuple(time_, position_, velocity_, acceleration_);
}

// 新增：计算中间点速度（根据公式3.16）
double TrapezoidalPlanner::calculateIntermediateVelocity(
    size_t index, 
    const std::vector<double>& waypoints) const 
{
    // 首尾点速度固定为0
    if (index == 0 || index == waypoints.size() - 1) {
        return 0.0;
    }
    
    // 计算相邻位移
    double h_prev = waypoints[index] - waypoints[index - 1];
    double h_next = waypoints[index + 1] - waypoints[index];
    
    // 获取位移方向符号
    int sign_prev = (h_prev >= 0) ? 1 : -1;
    int sign_next = (h_next >= 0) ? 1 : -1;
    
    // 根据公式(3.16)确定速度
    if (sign_prev != sign_next) {
        return 0.0;  // 方向变化点，速度设为0
    } else {
        return sign_prev * vd_max_;  // 方向不变点，速度设为最大速度
    }
}

// 新增：多段轨迹规划
std::tuple<std::vector<double>, 
           std::vector<double>, 
           std::vector<double>, 
           std::vector<double>> 
TrapezoidalPlanner::planMultiSegment(const std::vector<double>& waypoints) {
    // 清空之前的数据
    std::vector<double> time1;
    std::vector<double> position1;
    std::vector<double> velocity1;
    std::vector<double> acceleration1;
    
    // 验证输入
    if (waypoints.size() < 2) {
        throw std::invalid_argument("至少需要两个路径点");
    }
    
    // 计算中间点速度
    std::vector<double> segment_velocities(waypoints.size(),0);
    for (size_t i = 0; i < waypoints.size(); ++i) {
        segment_velocities[i] = calculateIntermediateVelocity(i, waypoints);
    }
    

    double t_offset = 0.0;                 // ① 累加前一段的终点时间
    // 为每个段生成轨迹
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        // 生成当前段的轨迹
        auto [t_seg, s_seg, v_seg, a_seg] = plan(
            waypoints[i], 
            waypoints[i + 1],
            segment_velocities[i],
            segment_velocities[i + 1]
        );
        // ② 丢掉第一段的首点（除第一段外），避免重复
        size_t start = (i == 0) ? 0 : 1;
        // 拼接轨迹
        for (size_t k = start; k < t_seg.size(); ++k) {
            time1.push_back(t_seg[k] + t_offset);
            position1.push_back(s_seg[k]);
            velocity1.push_back(v_seg[k]);
            acceleration1.push_back(a_seg[k]);
        }

        // ③ 更新偏移量
        t_offset += t_seg.back();
    }
    
    return std::make_tuple(time1, position1, velocity1, acceleration1);
}


// void TrapezoidalPlanner::saveToCSV(const std::string& file_path) {
//     if (time_.empty()) {
//         throw std::runtime_error("没有轨迹数据可保存");
//     }
    
//     std::ofstream out_file(file_path);
//     if (!out_file) {
//         throw std::runtime_error("无法打开文件: " + file_path);
//     }
    
//     // 写入表头
//     out_file << "Time,Position,Velocity,Acceleration\n";
    
//     // 设置精度
//     out_file << std::fixed << std::setprecision(6);
    
//     // 写入数据
//     for (size_t i = 0; i < time_.size(); ++i) {
//         out_file << time_[i] << ","
//                  << position_[i] << ","
//                  << velocity_[i] << ","
//                  << acceleration_[i] << "\n";
//     }
    
//     out_file.close();
//     std::cout << "轨迹数据已保存至: " << file_path << std::endl;
// }
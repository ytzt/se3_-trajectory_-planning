template<int Dim>
void PathToTrajectory<Dim>::calculateScalingFactor() {
    const int samples = 1000;
    double max_speed = 0.0;
    
    for (int i = 0; i <= samples; ++i) {
        double u = u_min_ + static_cast<double>(i) / samples * (u_max_ - u_min_);
        Eigen::Matrix<double, Dim, 1> deriv = deriv_func_(u, 1);
        double speed = deriv.norm();
        if (speed > max_speed) max_speed = speed;
    }
    
    lambda_ = v_max_ / max_speed;
}

template<int Dim>
void PathToTrajectory<Dim>::generateDirectParametrization(
    double t_total, 
    std::vector<double>& time_points,
    std::vector<double>& u_points,
    std::vector<Eigen::Matrix<double, Dim, 1>>& positions,
    std::vector<Eigen::Matrix<double, Dim, 1>>& velocities) 
{
    time_points.clear();
    u_points.clear();
    positions.clear();
    velocities.clear();
    
    for (double t = 0; t <= u_max_; t += Ts_) {
        double u = t;
        Eigen::Matrix<double, Dim, 1> pos = path_func_(u);
        Eigen::Matrix<double, Dim, 1> vel = deriv_func_(u, 1);
        
        time_points.push_back(t);
        u_points.push_back(u);
        positions.push_back(pos);
        velocities.push_back(vel);
    }
}

template<int Dim>
void PathToTrajectory<Dim>::generateScaledParametrization(
    std::vector<double>& time_points,
    std::vector<double>& u_points,
    std::vector<Eigen::Matrix<double, Dim, 1>>& positions,
    std::vector<Eigen::Matrix<double, Dim, 1>>& velocities) 
{
    time_points.clear();
    u_points.clear();
    positions.clear();
    velocities.clear();
    
    double t = 0;
    while (true) {
        double u = u_min_ + lambda_ * t;
        if (u > u_max_) break;
        
        Eigen::Matrix<double, Dim, 1> pos = path_func_(u);
        Eigen::Matrix<double, Dim, 1> du = deriv_func_(u, 1);
        Eigen::Matrix<double, Dim, 1> vel = du * lambda_;
        
        time_points.push_back(t);
        u_points.push_back(u);
        positions.push_back(pos);
        velocities.push_back(vel);
        
        t += Ts_;
    }
}

template<int Dim>
void PathToTrajectory<Dim>::generateFirstOrderTaylor(
    std::vector<double>& time_points,
    std::vector<double>& u_points,
    std::vector<Eigen::Matrix<double, Dim, 1>>& positions,
    std::vector<Eigen::Matrix<double, Dim, 1>>& velocities) 
{
    time_points.clear();
    u_points.clear();
    positions.clear();
    velocities.clear();
    
    double t = 0;
    double u = u_min_;
    while (u <= u_max_) {
        Eigen::Matrix<double, Dim, 1> du = deriv_func_(u, 1);
        double speed = du.norm();
        
        if (speed < 1e-6) break;
        
        // 记录当前状态
        time_points.push_back(t);
        u_points.push_back(u);
        positions.push_back(path_func_(u));
        velocities.push_back(du);
        
        // 下一时间步计算 (公式9.9a)
        u += (v_c_ * Ts_) / speed;
        t += Ts_;
    }
}

template<int Dim>
void PathToTrajectory<Dim>::generateSecondOrderTaylor(
    std::vector<double>& time_points,
    std::vector<double>& u_points,
    std::vector<Eigen::Matrix<double, Dim, 1>>& positions,
    std::vector<Eigen::Matrix<double, Dim, 1>>& velocities) 
{
    time_points.clear();
    u_points.clear();
    positions.clear();
    velocities.clear();
    
    double t = 0;
   double u = u_min_;
    while (u <= u_max_) {
        // 计算一阶和二阶导数
        Eigen::Matrix<double, Dim, 1> du = deriv_func_(u, 1);
        Eigen::Matrix<double, Dim, 1> ddu = deriv_func_(u, 2);
        double speed = du.norm();
        
        if (speed < 1e-6) break;
        
        // 记录当前状态
        time_points.push_back(t);
        u_points.push_back(u);
        positions.push_back(path_func_(u));
        // velocities.push_back((du / speed) * v_c_);
        velocities.push_back(du);
        
        // 计算二阶修正项 (公式9.9b)
        double speed4 = pow(speed, 4);
        double dot_product = du.dot(ddu);
        double correction = (v_c_ * Ts_ * v_c_ * Ts_) * (dot_product) / (2 * speed4);
        
        // 下一时间步计算
        u += (v_c_ * Ts_) / speed - correction;
        t += Ts_;
    }
}



/**
     * @brief 计算参数曲线从 u_start 到 u_end 的弧长
     * 
     * @param u_start 起始参数
     * @param u_end 结束参数
     * @param steps 积分步数
     * @return double 弧长
     */
template<int Dim>
double PathToTrajectory<Dim>::computeArcLength(double u_start, double u_end, int steps) const {
     if (steps <= 0) {
        throw std::invalid_argument("积分步数必须大于0");
    }
    if (u_start == u_end) {
        return 0.0;
    }

    double length = 0.0;
    double du = (u_end - u_start) / steps;
    
    auto dr_du_prev = deriv_func_(u_start, 1);
    double norm_prev = dr_du_prev.norm();
    // std::cout<< dr_du_prev<< std::endl;
    if (std::isnan(norm_prev)) {
        throw std::runtime_error("在参数u=" + std::to_string(u_start) + "处计算导数时得到NaN");
    }
    
    for (int i = 1; i <= steps; ++i) {
        
        double u = u_start + i * du;
        if(u > u_end) {
            u = u_end; // 确保不超过上限
        }
        auto dr_du = deriv_func_(u, 1);
        double norm_curr = dr_du.norm();
        // std::cout<< dr_du<< std::endl;
        if (std::isnan(norm_curr)) {
            std::cout<< dr_du<< std::endl;
            throw std::runtime_error("在参数u=" + std::to_string(u) + "处计算导数时得到NaN");
        }
        // 梯形法则积分
        length += 0.5 * (norm_prev + norm_curr) * du;
        norm_prev = norm_curr;
        if (u >= u_end) break;
    }
    
    return length;
}

    /**
     * @brief 生成通用进给速度曲线轨迹
     * 
     * @param T_s 采样时间
     * @param v_max 最大速度
     * @param a_max 最大加速度
     * @param j_max 最大加加速度
     * @param v_start 起始速度 (默认0)
     * @param v_end 结束速度 (默认0)
     * @return std::tuple<std::vector<double>, 
     *                    std::vector<typename PathType::PointType>,
     *                    std::vector<typename PathType::VectorType>,
     *                    std::vector<typename PathType::VectorType>>
     *         返回元组: [时间, 位置, 速度向量, 加速度向量]
     */
template<int Dim>
void PathToTrajectory<Dim>::generateTrajectoryWithVariableFeedrate(
    std::vector<double>& time_points,
    std::vector<double>& u_points,
    std::vector<Eigen::Matrix<double, Dim, 1>>& positions,
    std::vector<Eigen::Matrix<double, Dim, 1>>& velocities,
    double v_start, double v_end) const 
{

    // 清空输出容器
    time_points.clear();
    u_points.clear();
    positions.clear();
    velocities.clear();

    // 1. 计算曲线总长度
    double total_length = computeArcLength(u_min_, u_max_);
    std::cout << "计算完成 - 曲线总长度: " << total_length << " 米\n";
    
    // 2. 生成双S曲线运动规律 (位置为弧长s)
    DoubleSTrajectoryPlanner planner(v_max_, a_max_, j_max_);
    auto [time_vec, s_vec, v_vec, a_vec, _] = 
        planner.plan(Ts_, 0.0, total_length, v_start, v_end);
    
    // 增加长度和时间检查
    if (time_vec.empty()) {
        throw std::runtime_error("双S曲线规划失败: 返回的时间向量为空");
    }
    
    size_t num_points = time_vec.size();
    double trajectory_duration = time_vec.back();
    
    std::cout << "轨迹信息: "
              << "\n  轨迹点数: " << num_points
              << "\n  总时长: " << trajectory_duration << " 秒"
              << "\n  起始速度: " << v_start << " m/s"
              << "\n  结束速度: " << v_end << " m/s\n";
    
    // 3. 准备轨迹容器
    // std::vector<double> trajectory_time;
    // std::vector<Eigen::Matrix<double, Dim, 1>> positions;
    // std::vector<Eigen::Matrix<double, Dim, 1>>velocities;
    // std::vector<Eigen::Matrix<double, Dim, 1>> accelerations;
    // std::vector<double> u_points;
    
    // 4. 初始化参数
    double u = u_min_;
    double u_dot = 0.0;
    
    // 5. 轨迹生成主循环
    for (size_t k = 0; k < time_vec.size(); ++k) {
        if(u > u_max_){
            break;
        }
        // 获取当前时刻的期望速度和加速度
        double v_k = v_vec[k];
        double a_k = a_vec[k];
        
        // 计算路径导数
        auto dp_du = deriv_func_(u, 1);   // 一阶导数 dr/du
        auto d2p_du2 = deriv_func_(u, 2); // 二阶导数 d²r/du²
        
        double dp_du_norm = dp_du.norm();
        double dp_du_norm2 = dp_du_norm * dp_du_norm;
        double dp_du_norm4 = dp_du_norm2 * dp_du_norm2;
       
        
        // 计算点积 (dr/du · d²r/du²)
        double dot_product = dp_du.dot(d2p_du2);
        
        // 计算参数导数 (公式9.11)
        u_dot = v_k / dp_du_norm;
        
        // 计算参数二阶导数 (公式9.11b)
        double u_ddot = (a_k / dp_du_norm) - 
                       (v_k * v_k * dot_product) / dp_du_norm4;
        
        // 更新参数 u (公式9.12)
        double u_next = u + Ts_ * u_dot + 0.5 * Ts_ * Ts_ * u_ddot;
        
        // 参数边界检查
        if (u_next > u_max_) u_next = u_max_;
        if (u_next < u_min_) u_next = u_min_;
        
        // 计算轨迹点
        auto position = path_func_(u);
        auto velocity = dp_du * u_dot;
        
        // 加速度计算: a = d²r/dt² = d²r/du² (du/dt)² + dr/du d²u/dt²
        auto acceleration = d2p_du2 * (u_dot * u_dot) + dp_du * u_ddot;
        
        // 保存结果
        time_points.push_back(time_vec[k]);
        positions.push_back(position);
        velocities.push_back(velocity);
        // accelerations.push_back(acceleration);
        u_points.push_back(u);
        
        // 更新参数
        u = u_next;
    }
    
    // return std::make_tuple(trajectory_time, u_points, positions, velocities);
}


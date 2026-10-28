#include <iostream>
#include <vector>
#include <Eigen/Dense>

// 保存轨迹数据到CSV文件
template<int Dim>
void saveTrajectoryData(const std::string& filename,
                        const std::vector<double>& time_points,
                        const std::vector<double>& u_points,
                        const std::vector<Eigen::Matrix<double, Dim, 1>>& positions,
                        const std::vector<Eigen::Matrix<double, Dim, 1>>& velocities,
                        bool include_numerical_derivative = true) 
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    
    // 写入CSV头部
    file << "time,u,position_x,position_y";
    if constexpr (Dim == 2) {
        file << ",velocity_x,velocity_y,speed";
    } else if constexpr (Dim == 3) {
        file << ",position_z,velocity_x,velocity_y,velocity_z,speed";
    }
    file << "\n";
    
    // 计算数值导数速度
    std::vector<Eigen::Matrix<double, Dim, 1>> numerical_velocities;
    if (include_numerical_derivative && time_points.size() > 1) {
        numerical_velocities.resize(time_points.size());
        
        // 第一个点：前向差分
        if (time_points.size() > 1) {
            double dt = time_points[1] - time_points[0];
            if (dt > 1e-6) {
                numerical_velocities[0] = (positions[1] - positions[0]) / dt;
            }
        }
        
        // 中间点：中心差分
        for (size_t i = 1; i < time_points.size() - 1; i++) {
            double dt = time_points[i+1] - time_points[i-1];
            if (dt > 1e-6) {
                numerical_velocities[i] = (positions[i+1] - positions[i-1]) / dt;
            }
        }
        
        // 最后一个点：后向差分
        if (time_points.size() > 1) {
            double dt = time_points[time_points.size()-1] - time_points[time_points.size()-2];
            if (dt > 1e-6) {
                numerical_velocities.back() = (positions.back() - positions[positions.size()-2]) / dt;
            }
        }
    }

    // 写入数据
    for (size_t i = 0; i < time_points.size(); ++i) {
        file << std::setprecision(6) << std::fixed
             << time_points[i] << ","
             << u_points[i] << ","
             << positions[i].x();
        
        if constexpr (Dim >= 2) file << "," << positions[i].y();
        if constexpr (Dim >= 3) file << "," << positions[i].z();
        
        if constexpr (Dim == 2) {
            
            // 添加数值导数速度
            if (include_numerical_derivative && !numerical_velocities.empty()) {
                file << "," << numerical_velocities[i].x()
                     << "," << numerical_velocities[i].y()
                     << "," << numerical_velocities[i].norm();}
            else{
                file << "," << velocities[i].x()
                 << "," << velocities[i].y()
                 << "," << velocities[i].norm();

            }

        } else if constexpr (Dim == 3) {
            
            // 添加数值导数速度
            if (include_numerical_derivative && !numerical_velocities.empty()) {
                file << "," << numerical_velocities[i].x()
                     << "," << numerical_velocities[i].y()
                     << "," << numerical_velocities[i].z()
                     << "," << numerical_velocities[i].norm();
            } else{
                file << "," << velocities[i].x()
                 << "," << velocities[i].y()
                 << "," << velocities[i].z()
                 << "," << velocities[i].norm();

            } 
        }
        file << "\n";
    }
    file.close();
    std::cout << "Saved " << time_points.size() << " points to " << filename << std::endl;
}

// 修改后的 saveTrajectoryData 函数，支持加速度和额外数据保存
template<int Dim>
void saveTrajectoryData(const std::string& filename,
                        const std::vector<double>& time_points,
                        const std::vector<double>& u_points,
                        const std::vector<Eigen::Matrix<double, Dim, 1>>& positions,
                        const std::vector<Eigen::Matrix<double, Dim, 1>>& velocities,
                        const std::vector<Eigen::Matrix<double, Dim, 1>>& accelerations,
                        bool include_numerical_derivative = true) 
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    
    // 写入CSV头部 - 添加加速度模长列
    file << "time,u,position_x,position_y";
    if constexpr (Dim == 3) file << ",position_z";
    file << ",velocity_x,velocity_y";
    if constexpr (Dim == 3) file << ",velocity_z";
    file << ",acceleration_x,acceleration_y";
    if constexpr (Dim == 3) file << ",acceleration_z";
    file << ",speed,acceleration_norm\n";  // 添加加速度模长列
    
    // 计算数值导数速度（如果需要）
    std::vector<Eigen::Matrix<double, Dim, 1>> numerical_velocities;
    std::vector<Eigen::Matrix<double, Dim, 1>> numerical_accelerations;
    
    if (include_numerical_derivative && time_points.size() > 1) {
    numerical_velocities.resize(time_points.size());
    numerical_accelerations.resize(time_points.size());
    
    // 第一个点：前向差分
    if (time_points.size() > 1) {
        double dt1 = time_points[1] - time_points[0];
        if (dt1 > 1e-6) {
            numerical_velocities[0] = (positions[1] - positions[0]) / dt1;
            
            // 二阶导数需要至少两个点
            if (time_points.size() > 2) {
                double dt2 = time_points[2] - time_points[1];
                if (dt2 > 1e-6) {
                    Eigen::Vector3d vel1 = (positions[1] - positions[0]) / dt1;
                    Eigen::Vector3d vel2 = (positions[2] - positions[1]) / dt2;
                    numerical_accelerations[0] = (vel2 - vel1) / (dt1 + dt2);
                }
            }
        }
    }
    
    // 中间点：使用公式(8.48)的中心差分
    for (size_t i = 1; i < time_points.size() - 1; i++) {
        double dt_prev = time_points[i] - time_points[i-1];
        double dt_next = time_points[i+1] - time_points[i];
        double total_dt = dt_prev + dt_next;  // ū_{k+1} - ū_{k-1}
        
        if (total_dt > 1e-6) {
            // 速度计算 (公式8.48的第一部分)
            numerical_velocities[i] = (positions[i+1] - positions[i-1]) / total_dt;
            
            // 加速度计算 (公式8.48的第二部分)
            Eigen::Vector3d vel_next = (positions[i+1] - positions[i]) / dt_next;
            Eigen::Vector3d vel_prev = (positions[i] - positions[i-1]) / dt_prev;
            numerical_accelerations[i] = (vel_next - vel_prev) / total_dt;
        }
    }
    
    // 最后一个点：后向差分
    if (time_points.size() > 1) {
        int last = time_points.size() - 1;
        double dt_prev = time_points[last] - time_points[last-1];
        if (dt_prev > 1e-6) {
            numerical_velocities[last] = (positions[last] - positions[last-1]) / dt_prev;
            
            // 二阶导数需要至少两个点
            if (time_points.size() > 2) {
                int last_prev = last - 1;
                double dt_prev2 = time_points[last_prev] - time_points[last_prev-1];
                if (dt_prev2 > 1e-6) {
                    Eigen::Vector3d vel1 = (positions[last_prev] - positions[last_prev-1]) / dt_prev2;
                    Eigen::Vector3d vel2 = (positions[last] - positions[last_prev]) / dt_prev;
                    numerical_accelerations[last] = (vel2 - vel1) / (dt_prev2 + dt_prev);
                }
            }
        }
    }
    }


    // 写入数据
    for (size_t i = 0; i < time_points.size(); ++i) {
        file << std::setprecision(6) << std::fixed
             << time_points[i] << ","
             << u_points[i] << ","
             << positions[i].x() << "," << positions[i].y();
        
        if constexpr (Dim == 3) file << "," << positions[i].z();
        
        // 使用数值导数速度（如果启用）或解析导数速度
        const auto& vel = (include_numerical_derivative && !numerical_velocities.empty()) ? 
                          numerical_velocities[i] : velocities[i];
        
        file << "," << vel.x() << "," << vel.y();
        if constexpr (Dim == 3) file << "," << vel.z();
        
        // 加速度数据（使用数值导数或解析导数）
        const auto& accel = (include_numerical_derivative && !numerical_accelerations.empty()) ?
                           numerical_accelerations[i] : accelerations[i];
        
        file << "," << accel.x() << "," << accel.y();
        if constexpr (Dim == 3) file << "," << accel.z();
        
        // 计算速度大小和加速度模长
        const double speed = vel.norm();
        const double accel_norm = accel.norm();
        file << "," << speed << "," << accel_norm << "\n";
    }
    file.close();
    std::cout << "Saved " << time_points.size() << " points to " << filename << std::endl;
}


template<int Dim>
void savePathData(const std::string& filename, 
                 const std::vector<Eigen::Matrix<double, Dim, 1>>& points) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }
    
    // 写入CSV头部
    if constexpr (Dim >= 1) file << "position_x";
    if constexpr (Dim >= 2) file << ",position_y";
    if constexpr (Dim >= 3) file << ",position_z";
    
    file << "\n";
    
    // 写入点数据
    for (const auto& p : points) {
        for (int i = 0; i < Dim; i++) {
            if (i > 0) file << ",";
            file << p[i];
        }
        file << "\n";
    }
    
    file.close();
    std::cout << "已保存 " << points.size() << " 个点到 " << filename << std::endl;
}


// 保存路径点、切向量和曲率向量
template<int Dim>
void savePathData(const std::string& filename,
                  const std::vector<Eigen::Matrix<double, Dim, 1>>& points,
                  const std::vector<Eigen::Matrix<double, Dim, 1>>& tangents,
                  const std::vector<Eigen::Matrix<double, Dim, 1>>& curvatures) 
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    
    file << "position_x,position_y";
    if constexpr (Dim == 3) file << ",position_z";
    file << ",tangent_x,tangent_y";
    if constexpr (Dim == 3) file << ",tangent_z";
    file << ",curvature_x,curvature_y";
    if constexpr (Dim == 3) file << ",curvature_z";
    file << "\n";
    
    for (size_t i = 0; i < points.size(); ++i) {
        file << std::setprecision(6) << std::fixed
             << points[i].x() << "," << points[i].y();
        if constexpr (Dim == 3) file << "," << points[i].z();
        
        file << "," << tangents[i].x() << "," << tangents[i].y();
        if constexpr (Dim == 3) file << "," << tangents[i].z();
        
        file << "," << curvatures[i].x() << "," << curvatures[i].y();
        if constexpr (Dim == 3) file << "," << curvatures[i].z();
        
        file << "\n";
    }
    file.close();
    std::cout << "Saved " << points.size() << " path points to " << filename << std::endl;
}

// 保存贝塞尔控制点
template<int Dim>
void saveControlPoints(const std::string& filename,
                       const std::vector<std::vector<Eigen::Matrix<double, Dim, 1>>>& control_points) 
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    
    file << "segment,index,point_x,point_y";
    if constexpr (Dim == 3) file << ",point_z";
    file << "\n";
    
    for (size_t seg = 0; seg < control_points.size(); ++seg) {
        for (size_t idx = 0; idx < control_points[seg].size(); ++idx) {
            file << seg << "," << idx 
                 << "," << control_points[seg][idx].x() 
                 << "," << control_points[seg][idx].y();
            if constexpr (Dim == 3) file << "," << control_points[seg][idx].z();
            file << "\n";
        }
    }
    file.close();
    std::cout << "Saved " << control_points.size() << " segments to " << filename << std::endl;
}
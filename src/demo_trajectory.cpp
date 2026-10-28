
#include "InterpPlanner.h"
#include <iostream>
#include <vector>
#include "TrapezoidalPlanner.hpp"
#include "DoubleSTrajectoryPlanner.hpp"
#include "HarmonicTrajectory.hpp"
#include "CycloidalTrajectory.hpp"
#include "CubicSplineTrajectory.hpp"
#include "BSplineInterpolation.h"
#include  "PathToTrajectory.hpp"
#include "BezierInterpolation.hpp"
#include "saveFile.hpp"
#include "LinearBezierBlendInterpolation.hpp"


void TESET_Polynomial_of_degree_1(){
    // ================= 一阶多项式示例 =================
        std::vector<double> linear_times, linear_pos, linear_vel, linear_acc;
        auto linear_coeffs = InterpPlannerBase::linearInterp(0, 5, 3);
        for (double t = 0; t <= 3.01; t += 0.1) {
            auto state = InterpPlannerBase::evalPolynomial(linear_coeffs, t);
            linear_times.push_back(t);
            linear_pos.push_back(state[0]);
            linear_vel.push_back(state[1]);
            linear_acc.push_back(state[2]);
        }
        InterpPlannerBase::writeToFile("linear.csv", linear_times, linear_pos, linear_vel, linear_acc);

}

void TESET_Polynomial_of_degree_2(){
    // ================= 二次多项式示例 =================
        std::vector<double> quad_times, quad_pos, quad_vel, quad_acc;
        auto quad_coeffs = InterpPlannerBase::quadraticInterp(0, 1, 5, 2);
        for (double t = 0; t <= 2.01; t += 0.1) {
            auto state = InterpPlannerBase::evalPolynomial(quad_coeffs, t);
            quad_times.push_back(t);
            quad_pos.push_back(state[0]);
            quad_vel.push_back(state[1]);
            quad_acc.push_back(state[2]);
        }
        InterpPlannerBase::writeToFile("quadratic.csv", quad_times, quad_pos, quad_vel, quad_acc);
    
}

void TESET_Polynomial_of_degree_3(){
    // ================= 三次多项式示例 =================
        std::vector<double> cubic_times, cubic_pos, cubic_vel, cubic_acc;
        auto cubic_coeffs = InterpPlannerBase::cubicInterp(0, 1, 5, 0, 3);
        for (double t = 0; t <= 3.01; t += 0.1) {
            auto state = InterpPlannerBase::evalPolynomial(cubic_coeffs, t);
            cubic_times.push_back(t);
            cubic_pos.push_back(state[0]);
            cubic_vel.push_back(state[1]);
            cubic_acc.push_back(state[2]);
        }
        InterpPlannerBase::writeToFile("cubic.csv", cubic_times, cubic_pos, cubic_vel, cubic_acc);
    
}
void TESET_Polynomial_of_degree_5(){
    // ================= 五次多项式示例 =================
        std::vector<double> quintic_times, quintic_pos, quintic_vel, quintic_acc;
        auto quintic_coeffs = InterpPlannerBase::quinticInterp(0, 0, 0, 10, 0, 0, 5);
        for (double t = 0; t <= 5.01; t += 0.1) {
            auto state = InterpPlannerBase::evalPolynomial(quintic_coeffs, t);
            quintic_times.push_back(t);
            quintic_pos.push_back(state[0]);
            quintic_vel.push_back(state[1]);
            quintic_acc.push_back(state[2]);
        }
        InterpPlannerBase::writeToFile("quintic.csv", quintic_times, quintic_pos, quintic_vel, quintic_acc);
    
}

void TESET_Harmonic_trajectory(){
     /****************************** 谐波轨迹 ************************/
    // 轨迹参数设置 (示例2.14)
    double t0 = 0.0;    // 起始时间
    double t1 = 8.0;    // 结束时间
    double q0 = 0.0;    // 起始位置
    double q1 = 10.0;   // 结束位置
    double Ts = 0.01;   // 10ms采样时间
    
    // 生成谐波轨迹
    auto [time, positions, velocities, accelerations, jerks] = 
        HarmonicTrajectory::generate(t0, t1, q0, q1, Ts);
    
    // 保存到CSV
    // ofstream out("harmonic_trajectory.csv");
    // out << "Time,Position,Velocity,Acceleration\n";
    // for (size_t i = 0; i < time.size(); ++i) {
    //     out << time[i] << "," 
    //         << positions[i] << "," 
    //         << velocities[i] << "," 
    //         << accelerations[i] << "\n";
    // }
    // out.close();
    InterpPlannerBase::writeToFile(
        "harmonic_trajectory.csv",
        time,   // 时间向量
        positions,    // 位置向量
        velocities,    // 速度向量
        accelerations,     // 加速度向量
        jerks
    );
    std::cout << "谐波轨迹已保存到 harmonic_trajectory.csv" << std::endl;
   
    
}
void TESET_Cycloidal_trajectory(){
     /****************************** 摆线轨迹 ************************/
    
    // 轨迹参数设置 (示例值)
    double t0 = 0.0;    // 起始时间
    double t1 = 8.0;    // 结束时间
    double q0 = 0.0;    // 起始位置
    double q1 = 10.0;   // 结束位置
    double Ts = 0.01;   // 10ms采样时间
    
    // 生成摆线轨迹
    auto [time, positions, velocities, accelerations, jerks] = 
        CycloidalTrajectory::generate(t0, t1, q0, q1, Ts);
    
    // 保存到CSV
    InterpPlannerBase::writeToFile(
        "cycloidal_trajectory.csv",
        time,   // 时间向量
        positions,    // 位置向量
        velocities,    // 速度向量
        accelerations,     // 加速度向量
        jerks
    );
    std::cout << "摆线轨迹已保存到 cycloidal_trajectory.csv" << std::endl;
  
    
}
void TESET_Trapezoidal(){
    /***********T velocity trajectory *********/
    
    // 创建规划器（最大加速度2.0，最大速度1.5，时间步长0.01）
    TrapezoidalPlanner planner(20, 15, 0.01);
    
    // 定义路径点
    std::vector<double> waypoints = {0.0, 20, 50, 40, 20};
    
    // 生成多段轨迹
    auto [time, pos, vel, acc] = planner.planMultiSegment(waypoints);
    // auto [time, pos, vel, acc] = planner.plan(5.0, 0.0,  0.5, 0);
    
    // 保存结果 - 使用新的保存方法
    // planner.saveToCSV("multi_segment_trajectory.csv");
    
    // 或者直接调用InterpPlannerBase::writeToFile
    InterpPlannerBase::writeToFile(
        "T_multi_segment_trajectory.csv",
        time,   // 时间向量
        pos,    // 位置向量
        vel,    // 速度向量
        acc     // 加速度向量
    );
    
    std::cout << "多段轨迹生成成功！" << std::endl;
    
    
}
void TESET_Double_S(){
    /***********Double S velocity trajectory *********/
    
    // 约束参数设置 (只需提供最大值)
    DoubleSTrajectoryPlanner planner(
        10.0,  // 最大速度 (最小速度自动为 -5.0)
        20.0,  // 最大加速度 (最小加速度自动为 -2.0)
        60.0   // 最大加加速度 (最小加加速度自动为 -5.0)
    );

    // 轨迹参数设置
    double T_s = 0.001;  // 1ms 采样时间
    double q_0 = 30.0, q_1 = 0.0;  // 起始和终点位置
    double v_0 = 0.0, v_1 = 1;   // 起始和终点速度

    // 生成轨迹
    auto [time, positions, velocities, accelerations,jerk] = 
        planner.plan(T_s, q_0, q_1, v_0, v_1);

    // 保存到CSV

    InterpPlannerBase::writeToFile(
        "double_s_trajectory.csv",
        time,   // 时间向量
        positions,    // 位置向量
        velocities,    // 速度向量
        accelerations,     // 加速度向量
        jerk
    );

    // std::cout << "轨迹已保存到 double_s_trajectory.csv" << std::endl;
   
    
}

void TESET_Cubic_Splines(){
     /*  cubic spline  multipoint*/
    using namespace std;
   
    // 示例数据 (来自文献图4.9)
    vector<double> times = {0, 5, 7, 8, 10, 15, 18, 20};
    vector<double> positions = {3, -2, -5, 0, 6, 12, 8,15};
    double v0 = 0;   // 初始速度
    double vn = 0;  // 最终速度
    double a0 = 0.0;   // 初始加速度
    double an = 0.0;   // 最终加速度

    // 创建带加速度约束的三次样条轨迹
    CubicSplineTrajectory spline(times, positions, v0, vn, a0, an);
    
    // 生成离散轨迹 (100ms采样)
    auto [time, pos, vel, acc] = spline.generate(0.1);
    
    // 保存到CSV
    // ofstream out("accel_constrained_spline.csv");
    // out << "Time,Position,Velocity,Acceleration\n";
    // for (size_t i = 0; i < time.size(); ++i) {
    //     out << time[i] << "," << pos[i] << "," << vel[i] << "," << acc[i] << "\n";
    // }
    // out.close();
    InterpPlannerBase::writeToFile(
        "cubic_spline.csv",
        time,   // 时间向量
        pos,    // 位置向量
        vel,    // 速度向量
        acc
    );
    // cout << "中间点速度: ";
    // auto velocities = spline.get_velocities();
    // for (double a : velocities) cout << a << " ";
    // cout << endl;
    // 输出额外点信息
    auto extra_times = spline.get_extra_times();
    auto extra_positions = spline.get_extra_positions();
    auto accelerations = spline.get_accelerations();
    
    cout << "额外点时间: " << extra_times[0] << ", " << extra_times[1] << endl;
    cout << "额外点位置: " << extra_positions[0] << ", " << extra_positions[1] << endl;
    cout << "中间点加速度: ";
    for (double a : accelerations) cout << a << " ";
    cout << endl;
    
    cout << "轨迹已保存到 cubic_spline.csv" << endl;
        
   
    
}

void TESET_B_spline_path_interpolation(){
     /* BSplineInterpolation*/
    // 3D插值点
    using Eigen::Vector3d;
    std::vector<Vector3d> points = {
        {83, -54, 119}, {-64, 10, 124}, {42, 79, 226}, 
        {-98, 23, 222}, {-13, 125, 102}, {140, 81, 92},
        {43, 32, 92}, {-65, -17, 134}, {-45, -89, 182}, {71, 90, 192}
    };

    // 边界导数
    Vector3d t0 = {-1236, 538, 42};
    Vector3d tn = {732, 1130, 63};

    // 创建BSplineInterpolation对象
    BSplineInterpolation<3> interpolator(points,3);
    
    // 弦长参数化插值（不再返回值）
    interpolator.interpolateCubic(
        points, t0, tn, BSplineInterpolation<3>::ParametrizationMethod::CHORD_LENGTH);

    // 获取控制点
    const auto& controlPoints = interpolator.getControlPoints();
    
    // 创建曲线求值函数
    auto curve = interpolator.createCurveFunction();
    
    // 或者直接使用evaluate方法
    // auto evaluateFunc = [&](double u) { return interpolator.evaluate(u); };

    // 保存原始插值点
    std::ofstream origFile("original_points.txt");
    for (const auto& p : points) {
        origFile << p.x() << "," << p.y() << "," << p.z() << "\n";
    }
    origFile.close();

    // 保存控制点
    std::ofstream ctrlFile("control_points.txt");
    for (const auto& p : controlPoints) {
        ctrlFile << p.x() << "," << p.y() << "," << p.z() << "\n";
    }
    ctrlFile.close();

    // 保存插值曲线点（在u=0到u=1之间均匀采样）
    const int numSamples = 200;
    std::ofstream curveFile("curve_points.txt");
    for (int i = 0; i <= numSamples; ++i) {
        double u = static_cast<double>(i) / numSamples;
        // Vector3d point = curve(u); // 使用curve函数求值
        Vector3d point = interpolator.evaluate(u);
        curveFile << point.x() << "," << point.y() << "," << point.z() << "\n";
    }
    curveFile.close();

    std::cout << "Data saved to files:\n";
    std::cout << " - Original points: original_points.txt\n";
    std::cout << " - Control points: control_points.txt\n";
    std::cout << " - Curve points: curve_points.txt\n";


    /* 测试k阶导数 */

    // 使用自定义时间参数
    int p = 3;
    std::vector<double> times = {0.0, 1, 2, 4, 7};
    std::vector<Eigen::Matrix<double,1,1>> points1D = {
        Eigen::Matrix<double,1,1>(83),
        Eigen::Matrix<double,1,1>(-54),
        Eigen::Matrix<double,1,1>(119),
        Eigen::Matrix<double,1,1>(80),
        Eigen::Matrix<double,1,1>(50)
    };
    // std::vector<double> raw = {83, -54, 119, 80, 50};
    // std::vector<Eigen::Matrix<double,1,1>> points1D;
    // points1D.reserve(raw.size());
    // for (double v : raw) points1D.emplace_back(v);
    BSplineInterpolation<1> interpolator1(points1D, p, times);

    // 进行插值计算
    Eigen::Matrix<double, 1, 1> t0_(0);
    Eigen::Matrix<double, 1, 1> tn_(0);
    interpolator1.interpolateCubic(t0_,tn_);
    
    // 计算导数
    Eigen::Matrix<double,1,1> deriv = interpolator1.derivative(4.5, 1 , p);
     // 打印最终结果
    std::cout << "\n=== Final Derivative Result ===" << std::endl;
    std::cout << "Parameter u: 4.5" << std::endl;
    std::cout << "Derivative order: 3" << std::endl;
    std::cout << "Value: " << deriv(0,0) << std::endl;
    std::cout << "===============================" << std::endl;
    
}

void TESET_path_to_trajectory(){

    /* test for From Geometric Paths to Trajectories*/
    // 1. 创建B样条路径
    std::vector<Eigen::Vector2d> points = {
        {0, 0}, {1, 2},{2, 1}, {4, 3}, {5, 2}, {6, 3}, 
        {7, 0}, {8, -1}, {9, -1}, {10, 2}
    };
    
    int p = 3; // 三次B样条
    BSplineInterpolation<2> spline_interp(points, p);
    // 进行插值计算
    Eigen::Matrix<double, 2, 1> t0_ = {8.07,16.14};
    Eigen::Matrix<double, 2, 1> tn_ = {5.70,17.11};
    spline_interp.interpolateCubic(t0_,tn_);

     const auto& controlPoints = spline_interp.getControlPoints();
    std::cout << " - Control points: \n";
    for (const auto& p : controlPoints) {
        // std::cout << p.x() << "," << p.y() << "\n";
        std::cout << p.transpose() << '\n';   // Eigen 自动用逗号分隔
    }

    // 2. 定义路径函数和导数函数
    auto path_func = [&](double u) -> Eigen::Vector2d {
        return spline_interp.evaluate(u);
    };
    
    auto deriv_func = [&](double u, int order) -> Eigen::Vector2d {
        return spline_interp.derivative(u, order);
    };
    
    // 3. 创建轨迹生成器
    PathToTrajectory<2> trajectory(path_func, deriv_func);
    
    // 4. 设置运动参数
    // trajectory.setMotionParameters(10.0, 0.001); // v_max=10, Ts=0.01s
    // 设置运动参数
    trajectory.setMotionParameters(
        10.0,    // 最大速度 m/s
        50.0,    // 最大加速度 m/s²
        100.0,    // 最大加加速度 m/s³
        0.01,  // 采样时间 1ms
        0.0     // 期望恒定速度 (未使用)
    );
    
    // 5. 生成不同参数化轨迹
    std::vector<double> time_points, u_points;
    std::vector<Eigen::Vector2d> positions, velocities;
    
     // 直接参数化
    trajectory.generateDirectParametrization(1.0, time_points, u_points, positions, velocities);
    saveTrajectoryData<2>("direct_parametrization.csv", time_points, u_points, positions, velocities);
    
    // 缩放参数化
    trajectory.generateScaledParametrization(time_points, u_points, positions, velocities);
    saveTrajectoryData<2>("scaled_parametrization.csv", time_points, u_points, positions, velocities);
    
    // 一阶泰勒近似
    trajectory.generateFirstOrderTaylor(time_points, u_points, positions, velocities);
    saveTrajectoryData<2>("first_order_taylor.csv", time_points, u_points, positions, velocities);
    
    // 二阶泰勒近似
    trajectory.generateSecondOrderTaylor(time_points, u_points, positions, velocities);
    saveTrajectoryData<2>("second_order_taylor.csv", time_points, u_points, positions, velocities);
    
    
    trajectory.generateTrajectoryWithVariableFeedrate(
        time_points, u_points, positions, velocities );
    
    // 将速度向量全部置零
    for (auto& velocity : velocities) {
        velocity.setZero();
    }
    saveTrajectoryData<2>("s_motion_law.csv", time_points, u_points, positions, velocities);

    
}

void TESET_bezier_trajectory(){

     using Point3d = Eigen::Vector3d;
    using Points3d = std::vector<Point3d>;

 std::vector<Eigen::Vector3d> via_points = {
        {0, 0, 0},  // q0
        {1, 2, 1},  // q1
        {2, 3, 0},  // q2
        {4, 3, 0},  // q3
        {5, 2, 2},  // q4
        {6, 0, 2}   // q5
    };
    
    int p = 3; // 三次B样条
    BSplineInterpolation<3> spline_interp(via_points, p);
   
   
    spline_interp.interpolateCubic();

     const auto& controlPoints = spline_interp.getControlPoints();
    std::cout << " BSpline Control points: \n";
    for (const auto& p : controlPoints) {
        // std::cout << p.x() << "," << p.y() << "\n";
        std::cout << p.transpose() << '\n';   // Eigen 自动用逗号分隔
    }

    // 计算切向量和曲率向量
    std::vector<Point3d> tangents;
    std::vector<Point3d> curvatures;
    std::vector<double> u_bar = spline_interp.getUbar();
    if(u_bar.size()!= via_points.size()){
        std::cout << "ubar erro" << '\n';  
    }
    // 在路径点位置计算导数和二阶导数
    for (size_t i = 0; i < via_points.size(); ++i) {
        double u = u_bar[i];
        tangents.push_back(spline_interp.derivative(u, 1).normalized());
        curvatures.push_back(spline_interp.derivative(u, 2).normalized());
    }

      // 保存路径点数据（原始点、切向量、曲率向量）
    savePathData<3>("bezier_path_points.csv", via_points, tangents, curvatures);

    // 创建五次贝塞尔插值器 (3D)
    BezierInterpolation<3> bezier_interp(via_points, tangents, curvatures, 5, 0.51);
    

    // 计算控制点
    bezier_interp.compute();
    std::vector<Points3d> bezier_control_points =  bezier_interp.getControlPoints();

    // 保存贝塞尔控制点
    saveControlPoints<3>("bezier_control_points.csv", bezier_control_points);

      // 计算轨迹总长度
    std::vector<double> u_time = bezier_interp.uHat();
    double total_length = u_time.back() ;
    
    
    // 以0.01采样率遍历轨迹
    const double ts = 0.01;
    const int num_samples = static_cast<int>(total_length / ts) + 1;
    
    // 保存结果到CSV文件
   // 准备轨迹数据
    std::vector<double> Stime;
    std::vector<double> Su_points;
    std::vector<Point3d> Spositions;
    std::vector<Point3d> Svelocities;
    std::vector<Point3d> Saccelerations;
    
    for (int i = 0; i <= num_samples; ++i) {
        double t = i * ts;
        if (t > total_length) t = total_length;
        
        Stime.push_back(t);
        Su_points.push_back(t); // u参数与时间相同（弧长参数化）
        Spositions.push_back(bezier_interp.evaluate(t));
        Svelocities.push_back(bezier_interp.derivative(t, 1));
        Saccelerations.push_back(bezier_interp.derivative(t, 2));
    }
    
    // 保存轨迹数据
    saveTrajectoryData<3>("bezier_trajectory.csv", 
                         Stime, Su_points, 
                         Spositions, Svelocities, Saccelerations,false);
    
    
    std::cout << "轨迹数据已保存到 bezier_trajectory.csv" << std::endl;
    std::cout << "总轨迹长度: " << total_length << std::endl;
    std::cout << "采样点数: " << num_samples + 1 << std::endl;


}


void  TEST_linear_bezier_blend() {
    // 创建路径点（3维）
    int dim =3;
    using Point3d = Eigen::Vector3d;
    using Points3d = std::vector<Point3d>;

    LinearBezierBlendInterpolation<3>::Points viaPoints = {
        {0.0, 0.0, 0.0},
        {0.0, 0.05, 0.0},
        {0.05, 0.05, 0.0},
        {0.05, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };
    
    // 创建插值器（δ=0.5，β_bar=0.0 使用公式8.66）
    LinearBezierBlendInterpolation<3> interpolator(viaPoints, 0.005, 0.0);
    
    // 计算总长度
    double total_length = interpolator.totalLength();
    std::vector<double> u_time = interpolator.uHat();
    std::cout << "u重参数化：";
    for (double v : u_time) std::cout << v << ' ';
    std::cout << std::endl;

     // 保存结果到CSV文件
   // 准备轨迹数据
    std::vector<double> Stime;
    std::vector<double> Su_points;
    std::vector<Point3d> Spositions;
    std::vector<Point3d> Svelocities;
    std::vector<Point3d> Saccelerations;
    const double ts = 0.001;
    double current_time = 0.0;
    // 遍历整个轨迹
    for (double u_hat = 0; u_hat <= total_length; u_hat += ts) {
        Stime.push_back(current_time);
        Su_points.push_back(u_hat);
        
        // 计算位置、速度、加速度
        Spositions.push_back(interpolator.evaluate(u_hat));
        Svelocities.push_back(interpolator.derivative(u_hat, 1));
        Saccelerations.push_back(interpolator.derivative(u_hat, 2));
        
        current_time += ts;
    }
    
    // 保存轨迹数据
    saveTrajectoryData<3>("linear_bezier_trajectory.csv", 
                         Stime, Su_points, 
                         Spositions, Svelocities, Saccelerations, false);
    
    // 获取原始点
    const auto& q_prime_ = interpolator.getQPrime();
    const auto& q_dprime_ = interpolator.getQDPrime();
    // 保存原始点数据
    // savePathData<3>("linear_bezier_original_via_points.csv", viaPoints);
    // savePathData<3>("linear_bezier_q_prime_points.csv", q_prime_);
    // savePathData<3>("linear_bezier_q_dprime_points.csv", q_dprime_);

    Points3d allKeyPoints;
    // std::vector<std::string> pointTypes;
    
    // // 添加原始路径点
    // for (const auto& p : viaPoints) {
    //     allKeyPoints.push_back(p);
    //     // pointTypes.push_back("ViaPoint");
    // }
    
    // 添加q_prime_点
    for (const auto& p : q_prime_) {
        allKeyPoints.push_back(p);
        // pointTypes.push_back("QPrime");
    }
    
    // 添加q_dprime_点
    for (const auto& p : q_dprime_) {
        allKeyPoints.push_back(p);
        // pointTypes.push_back("QDPrime");
    }
    
    // 保存所有关键点到同一个文件
    savePathData<3>("linear_bezier_key_points.csv", allKeyPoints);

    std::cout << "轨迹数据已保存到 linear_bezier_trajectory.csv" << std::endl;
    std::cout << "总轨迹长度: " << total_length << std::endl;
    
}

void  TEST_linear_bezier_blend_trajectory() {
    // 创建路径点（3维）
    int dim =3;
    using Point3d = Eigen::Vector3d;
    using Points3d = std::vector<Point3d>;

    LinearBezierBlendInterpolation<3>::Points viaPoints = {
        {0.0, 0.0, 0.0},
        {0.0, 0.1, 0.0},
        {0.1, 0.1, 0.0},
        {0.1, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };
    
    // 创建插值器（δ=0.5，β_bar=0.0 使用公式8.66）
    LinearBezierBlendInterpolation<3> interpolator(viaPoints, 0.01, 0.0);
    
    // 计算总长度
    double total_length = interpolator.totalLength();
    std::vector<double> u_time = interpolator.uHat();
    std::cout << "u重参数化：";
    for (double v : u_time) std::cout << v << ' ';
    std::cout << std::endl;
     // 获取原始点
    const auto& q_prime_ = interpolator.getQPrime();
    const auto& q_dprime_ = interpolator.getQDPrime();
    // 保存原始点数据
    // savePathData<3>("linear_bezier_original_via_points.csv", viaPoints);
    // savePathData<3>("linear_bezier_q_prime_points.csv", q_prime_);
    // savePathData<3>("linear_bezier_q_dprime_points.csv", q_dprime_);

    Points3d allKeyPoints;
    
    
   
    // 添加q_prime_点
    for (const auto& p : q_prime_) {
        allKeyPoints.push_back(p);
        // pointTypes.push_back("QPrime");
    }
    
    // 添加q_dprime_点
    for (const auto& p : q_dprime_) {
        allKeyPoints.push_back(p);
        // pointTypes.push_back("QDPrime");
    }
    
    // 保存所有关键点到同一个文件
    savePathData<3>("linBeziBlend_key_points.csv",allKeyPoints);
     // 2. 定义路径函数和导数函数
    auto path_func = [&](double u) -> Eigen::Vector3d {
        return interpolator.evaluate(u);
    };
    
    auto deriv_func = [&](double u, int order) -> Eigen::Vector3d {
        return interpolator.derivative(u, order);
    };
    
    // 3. 创建轨迹生成器
    double u_length = total_length;
    PathToTrajectory<3> trajectory(path_func, deriv_func, 0.0, total_length);
    // 设置运动参数
    trajectory.setMotionParameters(
        0.03,   // 最大速度 m/s
        0.1,  // 最大加速度 m/s²
        0.1, // 最大加加速度 m/s³
        0.01,  // 采样时间 1ms
        0.0     // 期望恒定速度 (未使用)
    );

     // 保存结果到CSV文件
   // 准备轨迹数据
    std::vector<double> time_points;
    std::vector<double> u_points;
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector3d> velocities;
    
   // 生成轨迹
    trajectory.generateTrajectoryWithVariableFeedrate(
        time_points,
        u_points,
        positions,
        velocities,
        0.0,    // 起始速度
        0.0    // 结束速度
    );

   // 保存轨迹数据
    saveTrajectoryData<3>("linBeziBlend_s_motion_law.csv", 
                         time_points, 
                         u_points, 
                         positions, 
                         velocities, 
                         false);
    

    std::vector<Point3d> Saccelerations;
    saveTrajectoryData<3>("linBeziBlend_s_motion_law.csv", 
                         time_points, 
                         u_points, 
                         positions, 
                         velocities, 
                         false);
                         
    std::cout << "轨迹数据已保存到 linBeziBlend_s_motion_law.csv" << std::endl;
    std::cout << "轨迹点数: " << time_points.size() << std::endl;

    
}


void TESET_Cubic_Splines_v0vn(){
    /*  cubic spline 多点：仅指定初始速度、最终速度 (README 示例) */
    using namespace std;
    vector<double> times = {0, 5, 7, 8, 10, 15, 18};
    vector<double> positions = {3, -2, -5, 0, 6, 12, 8};
    double v0 = 2.0;   // 初始速度
    double vn = -3.0;  // 最终速度

    CubicSplineTrajectory spline(times, positions, v0, vn);
    auto [time, pos, vel, acc] = spline.generate(0.1);
    InterpPlannerBase::writeToFile(
        "cubic_spline_v0vn.csv",
        time, pos, vel, acc
    );
    cout << "轨迹已保存到 cubic_spline_v0vn.csv" << endl;
}

int main() {

    try {

        // TESET_Polynomial_of_degree_1();
        // TESET_Polynomial_of_degree_2();
        // TESET_Polynomial_of_degree_3();
        // TESET_Polynomial_of_degree_5();
        // TESET_bezier_trajectory();
        // TEST_linear_bezier_blend();
        // TEST_linear_bezier_blend_trajectory();

        TESET_Harmonic_trajectory();
        TESET_Cycloidal_trajectory();
        TESET_Trapezoidal();
        TESET_Double_S();
        TESET_Cubic_Splines();
        TESET_Cubic_Splines_v0vn();

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}


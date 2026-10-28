import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from pathlib import Path
from mpl_toolkits.mplot3d import Axes3D

# 设置绘图风格
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams.update({
    'font.size': 12,
    'figure.figsize': (14, 18),
    'figure.titlesize': 16,
    'axes.titlesize': 14
})

# 分析单个文件（现在接收路径点数据作为参数）
def analyze_trajectory(file_path, path_points_df=None):
    # 读取轨迹数据
    df = pd.read_csv(file_path)
    method_name = Path(file_path).stem.replace('_', ' ').title()
    
    # 检查数据列是否存在
    has_acceleration = all(col in df.columns for col in ['acceleration_x', 'acceleration_y', 'acceleration_z'])
    has_acceleration_norm = 'acceleration_norm' in df.columns
    has_velocity = all(col in df.columns for col in ['velocity_x', 'velocity_y', 'velocity_z'])
    has_speed = 'speed' in df.columns
    
    # ========== 创建交互式3D轨迹图 ==========
    fig_3d = plt.figure(figsize=(12, 9))
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    
    # 绘制轨迹
    ax_3d.plot(df['position_x'], df['position_y'], df['position_z'], 
               'b-', linewidth=1.5, alpha=0.8, label='Trajectory')
    
    # 如果提供了路径点数据，绘制路径点
    if path_points_df is not None:
        # 绘制路径点
        ax_3d.scatter(path_points_df['position_x'], path_points_df['position_y'], path_points_df['position_z'],
                     c='red', s=80, marker='o', edgecolors='k', label='Waypoints')
        
        # 标记路径点编号
        for i, row in path_points_df.iterrows():
            ax_3d.text(row['position_x'], row['position_y'], row['position_z'], 
                      f'P{i}', fontsize=10, color='black',
                      bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', boxstyle='round,pad=0.2'))
    else:
        print(f"警告: 未提供路径点数据 - {file_path}")
    
    ax_3d.set_title(f'3D Tool Path: {method_name}', fontsize=16)
    ax_3d.set_xlabel('X Position', fontsize=12)
    ax_3d.set_ylabel('Y Position', fontsize=12)
    ax_3d.set_zlabel('Z Position', fontsize=12)
    ax_3d.legend(fontsize=10)
    ax_3d.grid(True)
    ax_3d.view_init(elev=30, azim=45)
    
    # 保存交互式3D图
    output_3d_file = os.path.join(Path(file_path).parent, f'{method_name.replace(" ", "_")}_3d_path.png')
    plt.savefig(output_3d_file, dpi=300, bbox_inches='tight')
    print(f"保存3D路径图到 {output_3d_file}")
    
    # 显示交互式图形（在支持的环境中）
    if 'DISPLAY' in os.environ:
        plt.show(block=True)  # 非阻塞显示
        plt.pause(0.1)  # 短暂暂停以确保图形显示
    else:
        print("警告: 无图形显示环境，跳过交互显示")
    
    # ========== 创建4行2列的分析图 ==========
    fig_analysis = plt.figure(figsize=(16, 20))
    fig_analysis.suptitle(f'Trajectory Analysis: {method_name}', fontsize=16)
    
    # 2D轨迹投影图 (XY平面) - 位置1
    ax1 = fig_analysis.add_subplot(421)
    ax1.plot(df['position_x'], df['position_y'], 'b-', linewidth=1.5, alpha=0.8)
    if path_points_df is not None:
        ax1.scatter(path_points_df['position_x'], path_points_df['position_y'], 
                   c='red', s=60, marker='o', edgecolors='k')
        # 标记路径点
        for i, row in path_points_df.iterrows():
            ax1.text(row['position_x'], row['position_y'], f'P{i}', 
                    fontsize=9, color='black',
                    bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
    ax1.set_title('2D Projection (XY Plane)')
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.grid(True)
    ax1.axis('equal')
    
    # 速度大小图（如果有速度数据）- 位置2
    ax2 = fig_analysis.add_subplot(422)
    if has_speed:
        ax2.plot(df['time'], df['speed'], 'g-', linewidth=2)
        ax2.set_title('Speed Profile')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed')
        ax2.grid(True)
    else:
        ax2.text(0.5, 0.5, 'No Speed Data Available', 
                ha='center', va='center', fontsize=12, color='gray')
        ax2.set_title('Speed Profile (No Data)')
        ax2.axis('off')
    
    # 速度分量图（如果有速度分量数据）- 位置3
    ax4 = fig_analysis.add_subplot(423)
    if has_velocity:
        ax4.plot(df['time'], df['velocity_x'], 'b-', label='Vx', linewidth=1.5)
        ax4.plot(df['time'], df['velocity_y'], 'r-', label='Vy', linewidth=1.5)
        ax4.plot(df['time'], df['velocity_z'], 'g-', label='Vz', linewidth=1.5)
        ax4.set_title('Velocity Components')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Velocity')
        ax4.legend()
        ax4.grid(True)
    else:
        ax4.text(0.5, 0.5, 'No Velocity Data Available', 
                ha='center', va='center', fontsize=12, color='gray')
        ax4.set_title('Velocity Components (No Data)')
        ax4.axis('off')
    
    # 位置分量图 (X) - 位置4
    ax6 = fig_analysis.add_subplot(424)
    ax6.plot(df['time'], df['position_x'], 'm-', linewidth=2)
    ax6.set_title('X Position vs Time')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('X Position')
    ax6.grid(True)
    
    # 位置分量图 (Y) - 位置5
    ax7 = fig_analysis.add_subplot(425)
    ax7.plot(df['time'], df['position_y'], 'c-', linewidth=2)
    ax7.set_title('Y Position vs Time')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Y Position')
    ax7.grid(True)
    
    # 位置分量图 (Z) - 位置6
    ax8 = fig_analysis.add_subplot(426)
    ax8.plot(df['time'], df['position_z'], 'y-', linewidth=2)
    ax8.set_title('Z Position vs Time')
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('Z Position')
    ax8.grid(True)
    
    # 加速度大小图（如果有加速度数据）- 位置7（最后一排左侧）
    ax3 = fig_analysis.add_subplot(427)
    if has_acceleration_norm:
        ax3.plot(df['time'], df['acceleration_norm'], 'm-', linewidth=2)
        ax3.set_title('Acceleration Magnitude')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Acceleration')
        ax3.grid(True)
    else:
        ax3.text(0.5, 0.5, 'No Acceleration Data Available', 
                ha='center', va='center', fontsize=12, color='gray')
        ax3.set_title('Acceleration Magnitude (No Data)')
        ax3.axis('off')
    
    # 加速度分量图（如果有加速度分量数据）- 位置8（最后一排右侧）
    ax5 = fig_analysis.add_subplot(428)
    if has_acceleration:
        ax5.plot(df['time'], df['acceleration_x'], 'b-', label='Ax', linewidth=1.5)
        ax5.plot(df['time'], df['acceleration_y'], 'r-', label='Ay', linewidth=1.5)
        ax5.plot(df['time'], df['acceleration_z'], 'g-', label='Az', linewidth=1.5)
        ax5.set_title('Acceleration Components')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Acceleration')
        ax5.legend()
        ax5.grid(True)
    else:
        ax5.text(0.5, 0.5, 'No Acceleration Data Available', 
                ha='center', va='center', fontsize=12, color='gray')
        ax5.set_title('Acceleration Components (No Data)')
        ax5.axis('off')
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # 保存分析图
    output_analysis_file = os.path.join(Path(file_path).parent, f'{method_name.replace(" ", "_")}_analysis.png')
    plt.savefig(output_analysis_file, dpi=300, bbox_inches='tight')
    plt.close(fig_analysis)
    
    print(f"保存分析图到 {output_analysis_file}")
    
    # 计算统计数据
    stats = {
        'method': method_name,
        'total_time': df['time'].max(),
        'data_points': len(df)
    }
    
    # 添加速度统计数据（如果有）
    if has_speed:
        stats.update({
            'max_speed': df['speed'].max(),
            'min_speed': df['speed'].min(),
            'avg_speed': df['speed'].mean(),
            'speed_variation': df['speed'].max() - df['speed'].min(),
        })
    else:
        stats.update({
            'max_speed': np.nan,
            'min_speed': np.nan,
            'avg_speed': np.nan,
            'speed_variation': np.nan,
        })
    
    # 添加加速度统计数据（如果有）
    if has_acceleration_norm:
        stats.update({
            'max_accel': df['acceleration_norm'].max(),
            'min_accel': df['acceleration_norm'].min(),
            'accel_variation': df['acceleration_norm'].max() - df['acceleration_norm'].min(),
        })
    else:
        stats.update({
            'max_accel': np.nan,
            'min_accel': np.nan,
            'accel_variation': np.nan,
        })
    
    return stats

# 主分析函数
def main():
    # 获取当前脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 构建 build 目录路径
    build_dir = os.path.join(script_dir, '..', 'build')
    
    # 确保路径存在
    if not os.path.exists(build_dir):
        print(f"警告: 构建目录不存在 - {build_dir}")
        print("将尝试使用当前目录")
        build_dir = os.getcwd()
    
    # 轨迹文件列表
    trajectory_files = [
        # os.path.join(build_dir, 'linear_bezier_trajectory.csv')
        
        os.path.join(build_dir, 'linBeziBlend_s_motion_law.csv')
    ]
    
    # 路径点文件列表（与轨迹文件对应）
    path_point_files = [
        # os.path.join(build_dir, 'linear_bezier_key_points.csv')  # 使用新的路径点文件
        os.path.join(build_dir, 'linBeziBlend_key_points.csv')
    ]
    
    all_stats = []
    
    for traj_file, path_file in zip(trajectory_files, path_point_files):
        try:
            # 检查轨迹文件是否存在
            if not os.path.exists(traj_file):
                print(f"轨迹文件不存在: {traj_file}")
                continue
                
            # 尝试读取路径点数据
            path_points_df = None
            if os.path.exists(path_file):
                try:
                    path_points_df = pd.read_csv(path_file)
                    print(f"成功读取路径点文件: {path_file}")
                except Exception as e:
                    print(f"读取路径点文件 {path_file} 时出错: {str(e)}")
            else:
                print(f"警告: 路径点文件不存在 - {path_file}")
            
            # 分析轨迹
            stats = analyze_trajectory(traj_file, path_points_df)
            all_stats.append(stats)
            print(f"分析完成: {traj_file}")
            
        except Exception as e:
            print(f"分析文件 {traj_file} 时出错: {str(e)}")
    
    # 创建比较报告
    if all_stats:
        stats_df = pd.DataFrame(all_stats)
        print("\n性能比较:")
        print(stats_df)
        
        # 保存比较结果到构建目录
        comparison_file = os.path.join(build_dir, 'trajectory_comparison.csv')
        stats_df.to_csv(comparison_file, index=False)
        print(f"保存比较结果到 {comparison_file}")
        
        # 创建速度变化比较图
        plt.figure(figsize=(12, 8))
        for i, row in stats_df.iterrows():
            plt.bar(row['method'], row['speed_variation'], 
                    label=f"{row['method']} ({row['speed_variation']:.3f})")
        
        plt.title('Speed Variation Comparison')
        plt.ylabel('Speed Variation (max - min)')
        plt.xticks(rotation=15)
        plt.tight_layout()
        
        # 保存到构建目录
        speed_var_file = os.path.join(build_dir, 'speed_variation_comparison.png')
        plt.savefig(speed_var_file, dpi=200)
        print(f"保存速度变化比较图到 {speed_var_file}")
        
        # 创建加速度变化比较图
        plt.figure(figsize=(12, 8))
        for i, row in stats_df.iterrows():
            plt.bar(row['method'], row['accel_variation'], 
                    label=f"{row['method']} ({row['accel_variation']:.3f})")
        
        plt.title('Acceleration Variation Comparison')
        plt.ylabel('Acceleration Variation (max - min)')
        plt.xticks(rotation=15)
        plt.tight_layout()
        
        # 保存到构建目录
        accel_var_file = os.path.join(build_dir, 'acceleration_variation_comparison.png')
        plt.savefig(accel_var_file, dpi=200)
        print(f"保存加速度变化比较图到 {accel_var_file}")

if __name__ == "__main__":
    main()
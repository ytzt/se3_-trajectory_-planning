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

# 分析单个文件
def analyze_trajectory(file_path):
    # 读取轨迹数据
    df = pd.read_csv(file_path)
    method_name = Path(file_path).stem.replace('_', ' ').title()
    
    # ========== 创建交互式3D轨迹图 ==========
    fig_3d = plt.figure(figsize=(12, 9))
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    
    # 绘制轨迹
    ax_3d.plot(df['position_x'], df['position_y'], df['position_z'], 
               'b-', linewidth=1.5, alpha=0.8, label='Trajectory')
    
    # 尝试读取路径点数据
    path_points_file = os.path.join(Path(file_path).parent, 'bezier_path_points.csv')

    path_points = None
    if os.path.exists(path_points_file):
        try:
            path_points = pd.read_csv(path_points_file)
            # 绘制路径点
            ax_3d.scatter(path_points['position_x'], path_points['position_y'], path_points['position_z'],
                         c='red', s=80, marker='o', edgecolors='k', label='Waypoints')
            
            # 标记路径点编号
            for i, row in path_points.iterrows():
                ax_3d.text(row['position_x'], row['position_y'], row['position_z'], 
                          f'P{i}', fontsize=10, color='black',
                          bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', boxstyle='round,pad=0.2'))
        except Exception as e:
            print(f"错误: 无法读取路径点文件 {path_points_file}: {str(e)}")
    else:
        print(f"警告: 路径点文件不存在 - {path_points_file}")
    
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
    else:
        print("警告: 无图形显示环境，跳过交互显示")
    
    # ========== 创建4行2列的分析图 ==========
    fig_analysis = plt.figure(figsize=(16, 20))
    fig_analysis.suptitle(f'Trajectory Analysis: {method_name}', fontsize=16)
    
    # 2D轨迹投影图 (XY平面)
    ax1 = fig_analysis.add_subplot(421)
    ax1.plot(df['position_x'], df['position_y'], 'b-', linewidth=1.5, alpha=0.8)
    if path_points is not None:
        ax1.scatter(path_points['position_x'], path_points['position_y'], 
                   c='red', s=60, marker='o', edgecolors='k')
        # 标记路径点
        for i, row in path_points.iterrows():
            ax1.text(row['position_x'], row['position_y'], f'P{i}', 
                    fontsize=9, color='black',
                    bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
    ax1.set_title('2D Projection (XY Plane)')
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.grid(True)
    ax1.axis('equal')
    
    # 速度大小图
    ax2 = fig_analysis.add_subplot(422)
    ax2.plot(df['time'], df['speed'], 'g-', linewidth=2)
    ax2.set_title('Speed Profile')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Speed')
    ax2.grid(True)
    
    # 加速度大小图
    ax3 = fig_analysis.add_subplot(423)
    ax3.plot(df['time'], df['acceleration_norm'], 'm-', linewidth=2)
    ax3.set_title('Acceleration Magnitude')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Acceleration')
    ax3.grid(True)
    
    # 速度分量图
    ax4 = fig_analysis.add_subplot(424)
    ax4.plot(df['time'], df['velocity_x'], 'b-', label='Vx', linewidth=1.5)
    ax4.plot(df['time'], df['velocity_y'], 'r-', label='Vy', linewidth=1.5)
    ax4.plot(df['time'], df['velocity_z'], 'g-', label='Vz', linewidth=1.5)
    ax4.set_title('Velocity Components')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Velocity')
    ax4.legend()
    ax4.grid(True)
    
    # 加速度分量图
    ax5 = fig_analysis.add_subplot(425)
    ax5.plot(df['time'], df['acceleration_x'], 'b-', label='Ax', linewidth=1.5)
    ax5.plot(df['time'], df['acceleration_y'], 'r-', label='Ay', linewidth=1.5)
    ax5.plot(df['time'], df['acceleration_z'], 'g-', label='Az', linewidth=1.5)
    ax5.set_title('Acceleration Components')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Acceleration')
    ax5.legend()
    ax5.grid(True)
    
    # 位置分量图 (X)
    ax6 = fig_analysis.add_subplot(426)
    ax6.plot(df['time'], df['position_x'], 'm-', linewidth=2)
    ax6.set_title('X Position vs Time')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('X Position')
    ax6.grid(True)
    
    # 位置分量图 (Y)
    ax7 = fig_analysis.add_subplot(427)
    ax7.plot(df['time'], df['position_y'], 'c-', linewidth=2)
    ax7.set_title('Y Position vs Time')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Y Position')
    ax7.grid(True)
    
    # 位置分量图 (Z)
    ax8 = fig_analysis.add_subplot(428)
    ax8.plot(df['time'], df['position_z'], 'y-', linewidth=2)
    ax8.set_title('Z Position vs Time')
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('Z Position')
    ax8.grid(True)
    
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
        'max_speed': df['speed'].max(),
        'min_speed': df['speed'].min(),
        'avg_speed': df['speed'].mean(),
        'max_accel': df['acceleration_norm'].max(),
        'min_accel': df['acceleration_norm'].min(),
        'speed_variation': df['speed'].max() - df['speed'].min(),
        'accel_variation': df['acceleration_norm'].max() - df['acceleration_norm'].min(),
        'data_points': len(df)
    }
    
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
    
    # 添加构建目录前缀到文件名
    files = [
        os.path.join(build_dir, 'bezier_trajectory.csv')
    ]
    
    all_stats = []
    
    for file in files:
        try:
            if os.path.exists(file):
                stats = analyze_trajectory(file)
                all_stats.append(stats)
                print(f"分析完成: {file}")
            else:
                print(f"文件不存在: {file}")
        except Exception as e:
            print(f"分析文件 {file} 时出错: {str(e)}")
    
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
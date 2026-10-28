import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from pathlib import Path

# 设置绘图风格
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams.update({
    'font.size': 12,
    'figure.figsize': (12, 8),
    'figure.titlesize': 16,
    'axes.titlesize': 14
})

# 分析单个文件
def analyze_trajectory(file_path):
    # 读取数据
    df = pd.read_csv(file_path)
    method_name = Path(file_path).stem.replace('_', ' ').title()
    
    # 创建图形
    fig, axs = plt.subplots(3, 2, figsize=(16, 14))
    fig.suptitle(f'Trajectory Analysis: {method_name}', fontsize=16)
    
    # u-t 图
    axs[0, 0].plot(df['time'], df['u'], 'b-', linewidth=2)
    axs[0, 0].set_title('Parameter u vs Time')
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('Parameter u')
    axs[0, 0].grid(True)
    
    # 位置图 (x-y)
    axs[0, 1].plot(df['position_x'], df['position_y'], 'r-', linewidth=2)
    axs[0, 1].set_title('Tool Path')
    axs[0, 1].set_xlabel('X Position')
    axs[0, 1].set_ylabel('Y Position')
    axs[0, 1].grid(True)
    axs[0, 1].axis('equal')
    
    # 速度大小图
    axs[1, 0].plot(df['time'], df['speed'], 'g-', linewidth=2)
    axs[1, 0].set_title('Speed vs Time')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('Speed')
    axs[1, 0].grid(True)
    
    # X 位置随时间变化
    axs[1, 1].plot(df['time'], df['position_x'], 'm-', linewidth=2)
    axs[1, 1].set_title('X Position vs Time')
    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].set_ylabel('X Position')
    axs[1, 1].grid(True)
    
    # Y 位置随时间变化
    axs[2, 0].plot(df['time'], df['position_y'], 'c-', linewidth=2)
    axs[2, 0].set_title('Y Position vs Time')
    axs[2, 0].set_xlabel('Time (s)')
    axs[2, 0].set_ylabel('Y Position')
    axs[2, 0].grid(True)
    
    # 速度分量图
    axs[2, 1].plot(df['time'], df['velocity_x'], 'b-', label='Vx', linewidth=2)
    axs[2, 1].plot(df['time'], df['velocity_y'], 'r-', label='Vy', linewidth=2)
    axs[2, 1].set_title('Velocity Components vs Time')
    axs[2, 1].set_xlabel('Time (s)')
    axs[2, 1].set_ylabel('Velocity')
    axs[2, 1].legend()
    axs[2, 1].grid(True)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # 保存到构建目录
    output_file = os.path.join(Path(file_path).parent, f'{method_name.replace(" ", "_")}_analysis.png')
    plt.savefig(output_file, dpi=300)
    plt.close()
    
    print(f"Saved analysis to {output_file}")
    
    # 计算统计数据
    stats = {
        'method': method_name,
        'total_time': df['time'].max(),
        'max_speed': df['speed'].max(),
        'min_speed': df['speed'].min(),
        'avg_speed': df['speed'].mean(),
        'speed_variation': df['speed'].max() - df['speed'].min(),
        'data_points': len(df)
    }
    
    return stats

# 主分析函数
def main():
    # 获取当前脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 构建 build 目录路径（假设 build 目录在脚本目录的上一级目录中）
    build_dir = os.path.join(script_dir, '..', 'build')
    
    # 确保路径存在
    if not os.path.exists(build_dir):
        print(f"警告: 构建目录不存在 - {build_dir}")
        print("将尝试使用当前目录")
        build_dir = os.getcwd()
    
    # 添加构建目录前缀到文件名
    files = [
        os.path.join(build_dir, 'direct_parametrization.csv'),
        os.path.join(build_dir, 'scaled_parametrization.csv'),
        os.path.join(build_dir, 'first_order_taylor.csv'),
        os.path.join(build_dir, 'second_order_taylor.csv'),
        os.path.join(build_dir, 's_motion_law.csv')
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
        plt.figure(figsize=(10, 6))
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

if __name__ == "__main__":
    main()
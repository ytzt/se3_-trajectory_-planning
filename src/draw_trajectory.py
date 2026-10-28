import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# 设置绘图风格
plt.style.use('seaborn-v0_8-darkgrid')
# plt.rcParams['font.sans-serif'] = ['SimHei']  # 中文支持
plt.rcParams['axes.unicode_minus'] = False   # 解决负号显示问题

def plot_trajectory(csv_name: str, title: str, save_dir: str):
    """画单个轨迹（位置/速度/加速度/[jerk]）"""
    csv_path = os.path.join(save_dir, csv_name)
    if not os.path.exists(csv_path):
        print(f'[Warn] {csv_path} not found, skipping')
        return

    df = pd.read_csv(csv_path)
    has_jerk = 'Jerk' in df.columns

    n_rows = 4 if has_jerk else 3
    fig, axes = plt.subplots(n_rows, 1, figsize=(10, 4 * n_rows), sharex=True)
    fig.suptitle(f'{title} Polynomial Trajectory', fontsize=16)

    # 统一把 axes 转成 list，方便索引
    if n_rows == 1:
        axes = [axes]

    # ---- 1. Position ----
    axes[0].plot(df['Time'], df['Position'], 'b-', lw=2)
    axes[0].set_ylabel('Position')

    # ---- 2. Velocity ----
    axes[1].plot(df['Time'], df['Velocity'], 'r-', lw=2)
    axes[1].set_ylabel('Velocity')

    # ---- 3. Acceleration ----
    axes[2].plot(df['Time'], df['Acceleration'], 'g-', lw=2)
    axes[2].set_ylabel('Acceleration')

    # ---- 4. Jerk (optional) ----
    if has_jerk:
        axes[3].plot(df['Time'], df['Jerk'], 'm-', lw=2)
        axes[3].set_ylabel('Jerk')
        axes[3].set_xlabel('Time (s)')
    else:
        axes[2].set_xlabel('Time (s)')

    for ax in axes:
        ax.grid(True, ls='--', alpha=0.7)

    plt.tight_layout()
    out_file = os.path.join(save_dir, f'{title.lower()}_trajectory.png')
    plt.savefig(out_file, dpi=300)
    plt.show()
    plt.close()
    print(f'Saved {out_file}')

def plot_comparison():
    """绘制所有多项式对比图"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 12))
    
    # 读取所有数据
    linear = pd.read_csv('linear.csv')
    quad = pd.read_csv('quadratic.csv')
    cubic = pd.read_csv('cubic.csv')
    quintic = pd.read_csv('quintic.csv')
    
    # 位置对比
    axes[0].plot(linear['Time'], linear['Position'], 'm-', label='一阶')
    axes[0].plot(quad['Time'], quad['Position'], 'b-', label='二阶')
    axes[0].plot(cubic['Time'], cubic['Position'], 'r--', label='三阶')
    axes[0].plot(quintic['Time'], quintic['Position'], 'g-.', label='五阶')
    axes[0].set_ylabel('位置', fontsize=12)
    axes[0].legend()
    axes[0].grid(True)
    
    # 速度对比
    axes[1].plot(linear['Time'], linear['Velocity'], 'm-', label='一阶')
    axes[1].plot(quad['Time'], quad['Velocity'], 'b-', label='二阶')
    axes[1].plot(cubic['Time'], cubic['Velocity'], 'r--', label='三阶')
    axes[1].plot(quintic['Time'], quintic['Velocity'], 'g-.', label='五阶')
    axes[1].set_ylabel('速度', fontsize=12)
    axes[1].grid(True)
    
    # 加速度对比
    axes[2].plot(linear['Time'], linear['Acceleration'], 'm-', label='一阶')
    axes[2].plot(quad['Time'], quad['Acceleration'], 'b-', label='二阶')
    axes[2].plot(cubic['Time'], cubic['Acceleration'], 'r--', label='三阶')
    axes[2].plot(quintic['Time'], quintic['Acceleration'], 'g-.', label='五阶')
    axes[2].set_ylabel('加速度', fontsize=12)
    axes[2].set_xlabel('时间 (秒)', fontsize=12)
    axes[2].grid(True)
    
    plt.suptitle('多项式轨迹对比', fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig('polynomial_comparison.png', dpi=300)
    plt.show()

def plot_comparison(save_dir):
    """叠加对比 1st/2nd/3rd/5th"""
    orders = ['linear', 'quadratic', 'cubic', 'quintic']
    labels = ['1st', '2nd', '3rd', '5th']
    styles = ['m-', 'b-', 'r--', 'g-.']

    fig, axes = plt.subplots(3, 1, figsize=(12, 12))

    for order, lab, sty in zip(orders, labels, styles):
        csv_path = os.path.join(save_dir, f'{order}.csv')
        if not os.path.exists(csv_path):
            continue
        df = pd.read_csv(csv_path)
        axes[0].plot(df['Time'], df['Position'], sty, label=lab)
        axes[1].plot(df['Time'], df['Velocity'], sty, label=lab)
        axes[2].plot(df['Time'], df['Acceleration'], sty, label=lab)

    for ax in axes:
        ax.grid(True)
        ax.legend()
    axes[0].set_ylabel('Position')
    axes[1].set_ylabel('Velocity')
    axes[2].set_ylabel('Acceleration')
    axes[2].set_xlabel('Time (s)')

    plt.suptitle('Polynomial Trajectories Comparison', fontsize=16)
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, 'polynomial_comparison.png'), dpi=300)
    plt.close()
    print('Saved polynomial_comparison.png')

if __name__ == "__main__":
    # 绘制各多项式单独轨迹
    BUILD_DIR = os.path.join(os.path.dirname(__file__), '..', 'build')
    # csv_path = os.path.join(BUILD_DIR, 'linear.csv')
    # plot_trajectory(csv_path, '一阶')
    # csv_path = os.path.join(BUILD_DIR, 'quadratic.csv')
    # plot_trajectory(csv_path, '二阶')
    # csv_path = os.path.join(BUILD_DIR, 'cubic.csv')
    # plot_trajectory(csv_path, '三阶')
    # csv_path = os.path.join(BUILD_DIR, 'quintic.csv')
    # plot_trajectory(csv_path, '五阶')

    for order, title in zip(['linear', 'quadratic', 'cubic', 'quintic'],
                            ['1st', '2nd', '3rd', '5th']):
        plot_trajectory(f'{order}.csv', title, BUILD_DIR)
    # plot_trajectory('harmonic_trajectory.csv', 'harmonic',BUILD_DIR)
    # plot_trajectory('cycloidal_trajectory.csv', 'cycloidal',BUILD_DIR)
    #draw trapezoidal trajectory
    #plot_trajectory('T_multi_segment_trajectory.csv', 'trapezoidal',BUILD_DIR)
    #draw double_s_trajectory trajectory
    #plot_trajectory('double_s_trajectory.csv', 'double_s',BUILD_DIR)
    # plot_trajectory('cubic_spline.csv', 'cubic_spline',BUILD_DIR)
    
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os  # 导入os模块用于路径操作

def visualize_bspline(build_dir='../build'):
    """
    可视化B样条曲线
    
    参数:
    build_dir (str): 包含数据文件的构建目录路径
    """
    # 构建完整文件路径
    original_file = os.path.join(build_dir, 'original_points.txt')
    control_file = os.path.join(build_dir, 'control_points.txt')
    curve_file = os.path.join(build_dir, 'curve_points.txt')
    output_image = os.path.join(build_dir, 'bspline_visualization.png')
    
    # 从文件加载数据
    try:
        original_points = np.loadtxt(original_file, delimiter=',')
        control_points = np.loadtxt(control_file, delimiter=',')
        curve_points = np.loadtxt(curve_file, delimiter=',')
    except FileNotFoundError as e:
        print(f"错误: 文件未找到 - {e}")
        print("请确保已运行C++程序生成数据文件")
        return
    
    # 创建3D图形
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制原始插值点（红色实心点）
    ax.scatter(original_points[:, 0], original_points[:, 1], original_points[:, 2], 
               c='red', s=80, label='Original Points', depthshade=False)
    
    # 绘制控制点（蓝色实心点）
    ax.scatter(control_points[:, 0], control_points[:, 1], control_points[:, 2], 
               c='blue', s=60, label='Control Points', depthshade=False)
    
    # 绘制控制多边形（蓝色虚线）
    ax.plot(control_points[:, 0], control_points[:, 1], control_points[:, 2], 
            'b--', linewidth=1, label='Control Polygon')
    
    # 绘制插值曲线（绿色实线）
    ax.plot(curve_points[:, 0], curve_points[:, 1], curve_points[:, 2], 
            'g-', linewidth=2, label='B-Spline Curve')
    
    # 设置图例和标签
    ax.legend(fontsize=10)
    ax.set_xlabel('X', fontsize=12)
    ax.set_ylabel('Y', fontsize=12)
    ax.set_zlabel('Z', fontsize=12)
    ax.set_title('3D B-Spline Interpolation', fontsize=14)
    
    # 美化图形
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('w')
    ax.yaxis.pane.set_edgecolor('w')
    ax.zaxis.pane.set_edgecolor('w')
    
    # 设置视角
    ax.view_init(elev=30, azim=45)
    
    # 保存图像到build目录
    plt.savefig(output_image, dpi=300, bbox_inches='tight')
    print(f"图像已保存至: {output_image}")
    plt.show()

if __name__ == "__main__":
    # 获取当前脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # 构建build目录路径（假设build目录在脚本目录的上一级目录中）
    build_dir = os.path.join(script_dir, '..', 'build')
    
    # 确保路径存在
    if not os.path.exists(build_dir):
        print(f"警告: 构建目录不存在 - {build_dir}")
        print("将尝试使用当前目录")
        build_dir = os.getcwd()
    
    visualize_bspline(build_dir)
# -*- coding: utf-8 -*-
"""
轨迹可视化工具
从 CSV 文件读取位姿数据，生成 3D 空间轨迹图

功能:
1. 3D 轨迹曲线可视化
2. 起点/终点标记
3. 姿态方向箭头显示（可选）
4. 轨迹统计信息
5. 多视角投影

Author: GitHub Copilot
Date: 2025-12-20
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import os
import matplotlib

# 设置中文字体支持 - Windows 系统
matplotlib.rcParams['font.family'] = ['Microsoft YaHei', 'SimHei', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# 忽略字体警告
import warnings
warnings.filterwarnings('ignore', category=UserWarning, module='matplotlib')


def load_trajectory_csv(csv_path: str, convert_to_mm: bool = True) -> pd.DataFrame:
    """
    加载 CSV/Excel 轨迹文件
    
    支持的CSV格式:
    1. HighFrequencyTrackerRecorder: X_mm, Y_mm, Z_mm (已经是mm)
    2. CSVTrajectoryServoCapture: Target_X_mm, Target_Y_mm, Target_Z_mm
    3. 其他格式: 自动检测列名
    
    Args:
        csv_path: CSV或Excel文件路径
        convert_to_mm: 是否将数据转换为毫米（检测到米单位时）
        
    Returns:
        pandas DataFrame (统一为 X_mm, Y_mm, Z_mm 列名)
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"文件不存在: {csv_path}")
    
    # 自动识别文件类型
    file_ext = os.path.splitext(csv_path)[1].lower()
    if file_ext in ['.xlsx', '.xls']:
        df = pd.read_excel(csv_path)
        print(f"[INFO] 读取Excel文件格式")
    else:
        # 跳过以 # 开头的注释行
        df = pd.read_csv(csv_path, comment='#')
    
    print(f"[INFO] 加载轨迹文件: {csv_path}")
    print(f"[INFO] 总帧数: {len(df)}")
    print(f"[INFO] 原始列名: {list(df.columns)}")
    
    # ========== 自动识别和映射列名 ==========
    column_mapping = {}
    
    # 位置列名映射 (优先级从高到低)
    x_candidates = ['X_mm', 'Target_X_mm', 'Movej_TCP_X_m', 'Servo_TCP_X_m', 'x_mm', 'X', 'x']
    y_candidates = ['Y_mm', 'Target_Y_mm', 'Movej_TCP_Y_m', 'Servo_TCP_Y_m', 'y_mm', 'Y', 'y']
    z_candidates = ['Z_mm', 'Target_Z_mm', 'Movej_TCP_Z_m', 'Servo_TCP_Z_m', 'z_mm', 'Z', 'z']
    
    # 四元数列名映射
    qx_candidates = ['QX', 'Target_QX', 'qx', 'Qx']
    qy_candidates = ['QY', 'Target_QY', 'qy', 'Qy']
    qz_candidates = ['QZ', 'Target_QZ', 'qz', 'Qz']
    qw_candidates = ['QW', 'Target_QW', 'qw', 'Qw']
    
    # 旋转矢量列名映射
    rx_candidates = ['RX_rad', 'rx_rad', 'RX', 'rx']
    ry_candidates = ['RY_rad', 'ry_rad', 'RY', 'ry']
    rz_candidates = ['RZ_rad', 'rz_rad', 'RZ', 'rz']
    
    # 时间列名映射
    time_candidates = ['TimeFromStart_s', 'Time_s', 'time_s', 'Time', 'time', 't']
    
    def find_column(candidates, df_columns):
        """从候选列名中找到存在的列"""
        for c in candidates:
            if c in df_columns:
                return c
        return None
    
    # 查找位置列
    x_col = find_column(x_candidates, df.columns)
    y_col = find_column(y_candidates, df.columns)
    z_col = find_column(z_candidates, df.columns)
    
    if not all([x_col, y_col, z_col]):
        print(f"[ERROR] 无法找到位置列！")
        print(f"  期望: X_mm/Y_mm/Z_mm 或 Target_X_mm/Target_Y_mm/Target_Z_mm")
        print(f"  实际: {list(df.columns)}")
        raise ValueError("CSV文件缺少位置数据列")
    
    print(f"[INFO] 检测到位置列: {x_col}, {y_col}, {z_col}")
    
    # 检测数据单位
    # 判断依据：如果最大值小于5，很可能是米单位（正常轨迹范围在0.1-2米之间）
    x_max = abs(df[x_col].max())
    y_max = abs(df[y_col].max())
    z_max = abs(df[z_col].max())
    max_value = max(x_max, y_max, z_max)
    
    # 列名中包含 '_m' 且不包含 '_mm' 表示米单位
    is_meter_column = ('_m' in x_col.lower() and '_mm' not in x_col.lower())
    # 或者数值很小（<5）也认为是米单位
    is_meter_by_value = max_value < 5.0
    
    is_meter_unit = is_meter_column or is_meter_by_value
    
    if is_meter_unit:
        print(f"[INFO] 检测到数据为米单位 (max_value={max_value:.4f})")
        if convert_to_mm:
            print(f"[INFO] 自动转换为毫米单位")
            scale = 1000.0
        else:
            scale = 1.0
    else:
        print(f"[INFO] 检测到数据为毫米单位 (max_value={max_value:.1f})")
        scale = 1.0
    
    # 创建标准化的DataFrame
    result_df = pd.DataFrame()
    result_df['X_mm'] = df[x_col] * scale
    result_df['Y_mm'] = df[y_col] * scale
    result_df['Z_mm'] = df[z_col] * scale
    
    # 复制四元数列（如果存在）
    qx_col = find_column(qx_candidates, df.columns)
    qy_col = find_column(qy_candidates, df.columns)
    qz_col = find_column(qz_candidates, df.columns)
    qw_col = find_column(qw_candidates, df.columns)
    
    if all([qx_col, qy_col, qz_col, qw_col]):
        result_df['QX'] = df[qx_col]
        result_df['QY'] = df[qy_col]
        result_df['QZ'] = df[qz_col]
        result_df['QW'] = df[qw_col]
        print(f"[INFO] 检测到四元数列: {qx_col}, {qy_col}, {qz_col}, {qw_col}")
    
    # 复制旋转矢量列（如果存在）
    rx_col = find_column(rx_candidates, df.columns)
    ry_col = find_column(ry_candidates, df.columns)
    rz_col = find_column(rz_candidates, df.columns)
    
    if all([rx_col, ry_col, rz_col]):
        result_df['RX_rad'] = df[rx_col]
        result_df['RY_rad'] = df[ry_col]
        result_df['RZ_rad'] = df[rz_col]
        print(f"[INFO] 检测到旋转矢量列: {rx_col}, {ry_col}, {rz_col}")
    
    # 复制时间列（如果存在）
    time_col = find_column(time_candidates, df.columns)
    if time_col:
        result_df['TimeFromStart_s'] = df[time_col]
        print(f"[INFO] 检测到时间列: {time_col}")
    else:
        # 如果没有时间列，根据帧号生成（假设100Hz）
        result_df['TimeFromStart_s'] = np.arange(len(df)) / 100.0
        print(f"[WARN] 未找到时间列，使用帧号生成（假设100Hz）")
    
    # 打印转换后的范围
    print(f"[INFO] 数据范围:")
    print(f"  X: [{result_df['X_mm'].min():.2f}, {result_df['X_mm'].max():.2f}] mm")
    print(f"  Y: [{result_df['Y_mm'].min():.2f}, {result_df['Y_mm'].max():.2f}] mm")
    print(f"  Z: [{result_df['Z_mm'].min():.2f}, {result_df['Z_mm'].max():.2f}] mm")
    
    return result_df


def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """
    四元数转旋转矩阵
    
    Args:
        qx, qy, qz, qw: 四元数分量
        
    Returns:
        3x3 旋转矩阵，如果四元数无效则返回单位矩阵
    """
    # 检查是否为 NaN
    if np.isnan(qx) or np.isnan(qy) or np.isnan(qz) or np.isnan(qw):
        return np.eye(3)
    
    # 检查四元数范数
    norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    if norm < 1e-6:  # 零范数四元数
        return np.eye(3)  # 返回单位矩阵
    
    # 归一化四元数
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    
    try:
        r = R.from_quat([qx, qy, qz, qw])
        return r.as_matrix()
    except:
        return np.eye(3)


def calculate_trajectory_stats(df: pd.DataFrame) -> dict:
    """
    计算轨迹统计信息
    
    Args:
        df: 轨迹数据
        
    Returns:
        统计信息字典
    """
    positions = df[['X_mm', 'Y_mm', 'Z_mm']].values
    
    # 计算轨迹长度
    diffs = np.diff(positions, axis=0)
    segment_lengths = np.linalg.norm(diffs, axis=1)
    total_length = np.sum(segment_lengths)
    
    # 计算边界框
    min_pos = positions.min(axis=0)
    max_pos = positions.max(axis=0)
    
    # 计算时间跨度
    time_span = df['TimeFromStart_s'].iloc[-1] - df['TimeFromStart_s'].iloc[0]
    
    stats = {
        'total_frames': len(df),
        'total_length_mm': total_length,
        'time_span_s': time_span,
        'avg_speed_mm_s': total_length / time_span if time_span > 0 else 0,
        'min_x': min_pos[0], 'max_x': max_pos[0],
        'min_y': min_pos[1], 'max_y': max_pos[1],
        'min_z': min_pos[2], 'max_z': max_pos[2],
        'range_x': max_pos[0] - min_pos[0],
        'range_y': max_pos[1] - min_pos[1],
        'range_z': max_pos[2] - min_pos[2],
    }
    
    return stats


def plot_3d_trajectory(df: pd.DataFrame, 
                       show_orientation: bool = True,
                       orientation_interval: int = 100,
                       arrow_length: float = None,
                       figsize: tuple = (14, 10),
                       save_path: str = None):
    """
    绘制 3D 轨迹图
    
    Args:
        df: 轨迹数据
        show_orientation: 是否显示姿态箭头
        orientation_interval: 姿态箭头间隔（每隔多少帧显示一个）
        arrow_length: 箭头长度 (mm)
        figsize: 图形尺寸
        save_path: 保存路径（None 则不保存）
    """
    # 提取位置数据
    x = df['X_mm'].values
    y = df['Y_mm'].values
    z = df['Z_mm'].values
    
    # 计算统计信息
    stats = calculate_trajectory_stats(df)
    
    # 自动计算箭头长度（如果未指定）
    if arrow_length is None:
        max_range = max(stats['range_x'], stats['range_y'], stats['range_z'])
        arrow_length = max_range * 0.05  # 箭头长度为最大范围的5%
    
    # 创建图形
    fig = plt.figure(figsize=figsize)
    
    # ===== 主 3D 视图 =====
    ax_main = fig.add_subplot(2, 2, 1, projection='3d')
    
    # 绘制轨迹曲线（使用连续线条代替散点）
    ax_main.plot(x, y, z, c='royalblue', linewidth=2, alpha=0.8, label='轨迹路径')
    
    # 在轨迹上添加渐变色散点标记（稀疏采样）
    sample_interval = max(1, len(x) // 100)  # 最多显示100个点
    ax_main.scatter(x[::sample_interval], y[::sample_interval], z[::sample_interval], 
                   c=range(len(x[::sample_interval])), cmap='viridis', 
                   s=20, alpha=0.7, edgecolors='none', zorder=3)
    
    # 绘制起点和终点
    ax_main.scatter(x[0], y[0], z[0], c='green', s=150, marker='o', 
                    label=f'起点 ({x[0]:.1f}, {y[0]:.1f}, {z[0]:.1f})', 
                    edgecolors='darkgreen', linewidths=2, zorder=5)
    ax_main.scatter(x[-1], y[-1], z[-1], c='red', s=150, marker='s', 
                    label=f'终点 ({x[-1]:.1f}, {y[-1]:.1f}, {z[-1]:.1f})', 
                    edgecolors='darkred', linewidths=2, zorder=5)
    
    # 绘制姿态箭头（如果启用）
    if show_orientation and 'QX' in df.columns:
        arrow_indices = range(0, len(df), orientation_interval)
        arrow_count = 0
        for i in arrow_indices:
            qx = df['QX'].iloc[i]
            qy = df['QY'].iloc[i]
            qz = df['QZ'].iloc[i]
            qw = df['QW'].iloc[i]
            
            rot_mat = quaternion_to_rotation_matrix(qx, qy, qz, qw)
            
            # Z 轴方向（工具方向）
            z_axis = rot_mat[:, 2] * arrow_length
            ax_main.quiver(x[i], y[i], z[i], 
                          z_axis[0], z_axis[1], z_axis[2],
                          color='red', alpha=0.5, arrow_length_ratio=0.3, 
                          linewidth=1.5, zorder=2)
            arrow_count += 1
        print(f"[INFO] 显示了 {arrow_count} 个姿态箭头")
    
    ax_main.set_xlabel('X (mm)', fontsize=10)
    ax_main.set_ylabel('Y (mm)', fontsize=10)
    ax_main.set_zlabel('Z (mm)', fontsize=10)
    ax_main.set_title('3D 轨迹视图', fontsize=12, fontweight='bold')
    ax_main.legend(loc='upper left', fontsize=8)
    
    # 设置等比例坐标轴
    max_range = max(stats['range_x'], stats['range_y'], stats['range_z']) / 2
    mid_x = (stats['min_x'] + stats['max_x']) / 2
    mid_y = (stats['min_y'] + stats['max_y']) / 2
    mid_z = (stats['min_z'] + stats['max_z']) / 2
    ax_main.set_xlim(mid_x - max_range * 1.1, mid_x + max_range * 1.1)
    ax_main.set_ylim(mid_y - max_range * 1.1, mid_y + max_range * 1.1)
    ax_main.set_zlim(mid_z - max_range * 1.1, mid_z + max_range * 1.1)
    
    # ===== 添加鼠标滚轮缩放功能 =====
    # 保存初始范围用于缩放限制
    initial_max_range = max_range * 1.1
    
    def on_scroll(event):
        """鼠标滚轮缩放回调函数（增强版：更大步长 + 范围限制）"""
        if event.inaxes != ax_main:
            return
        
        # 获取当前坐标轴范围
        xlim = ax_main.get_xlim3d()
        ylim = ax_main.get_ylim3d()
        zlim = ax_main.get_zlim3d()
        
        # 计算当前中心点
        x_center = (xlim[0] + xlim[1]) / 2
        y_center = (ylim[0] + ylim[1]) / 2
        z_center = (zlim[0] + zlim[1]) / 2
        
        # 计算当前范围
        x_range = (xlim[1] - xlim[0]) / 2
        y_range = (ylim[1] - ylim[0]) / 2
        z_range = (zlim[1] - zlim[0]) / 2
        
        # 缩放因子（向上滚动缩小，向下滚动放大）- 增大到25%步长
        scale_factor = 0.75 if event.button == 'up' else 1.25
        
        # 应用缩放
        new_x_range = x_range * scale_factor
        new_y_range = y_range * scale_factor
        new_z_range = z_range * scale_factor
        
        # 缩放范围限制（防止过度缩小或放大）
        max_allowed_range = initial_max_range * 50  # 最大放大50倍
        min_allowed_range = initial_max_range * 0.1  # 最小缩小到0.1倍
        
        new_max_range = max(new_x_range, new_y_range, new_z_range)
        if new_max_range > max_allowed_range or new_max_range < min_allowed_range:
            return  # 超出限制，不执行缩放
        
        # 更新坐标轴范围
        ax_main.set_xlim3d([x_center - new_x_range, x_center + new_x_range])
        ax_main.set_ylim3d([y_center - new_y_range, y_center + new_y_range])
        ax_main.set_zlim3d([z_center - new_z_range, z_center + new_z_range])
        
        fig.canvas.draw_idle()
    
    # 连接滚轮事件
    fig.canvas.mpl_connect('scroll_event', on_scroll)
    
    # ===== XY 投影 (俯视图) =====
    ax_xy = fig.add_subplot(2, 2, 2)
    ax_xy.plot(x, y, 'b-', linewidth=1.5, alpha=0.7)
    ax_xy.scatter(x[::sample_interval], y[::sample_interval], 
                 c=range(len(x[::sample_interval])), cmap='viridis', s=15, alpha=0.7)
    ax_xy.scatter(x[0], y[0], c='green', s=100, marker='o', edgecolors='darkgreen', linewidths=2, zorder=5)
    ax_xy.scatter(x[-1], y[-1], c='red', s=100, marker='s', edgecolors='darkred', linewidths=2, zorder=5)
    ax_xy.set_xlabel('X (mm)', fontsize=10)
    ax_xy.set_ylabel('Y (mm)', fontsize=10)
    ax_xy.set_title('XY 投影 (俯视图)', fontsize=12, fontweight='bold')
    ax_xy.set_aspect('equal')
    ax_xy.grid(True, alpha=0.3)
    
    # ===== XZ 投影 (正视图) =====
    ax_xz = fig.add_subplot(2, 2, 3)
    ax_xz.plot(x, z, 'b-', linewidth=1.5, alpha=0.7)
    ax_xz.scatter(x[::sample_interval], z[::sample_interval], 
                 c=range(len(x[::sample_interval])), cmap='viridis', s=15, alpha=0.7)
    ax_xz.scatter(x[0], z[0], c='green', s=100, marker='o', edgecolors='darkgreen', linewidths=2, zorder=5)
    ax_xz.scatter(x[-1], z[-1], c='red', s=100, marker='s', edgecolors='darkred', linewidths=2, zorder=5)
    ax_xz.set_xlabel('X (mm)', fontsize=10)
    ax_xz.set_ylabel('Z (mm)', fontsize=10)
    ax_xz.set_title('XZ 投影 (正视图)', fontsize=12, fontweight='bold')
    ax_xz.set_aspect('equal')
    ax_xz.grid(True, alpha=0.3)
    
    # ===== 统计信息面板 =====
    ax_info = fig.add_subplot(2, 2, 4)
    ax_info.axis('off')
    
    info_text = f"""
    ========================================
              Trajectory Statistics
    ========================================
      Total Frames:    {stats['total_frames']:>10}
      Length:          {stats['total_length_mm']:>10.2f} mm
      Duration:        {stats['time_span_s']:>10.2f} s
      Avg Speed:       {stats['avg_speed_mm_s']:>10.2f} mm/s
    ----------------------------------------
      X Range: [{stats['min_x']:>8.2f}, {stats['max_x']:>8.2f}] mm
      Y Range: [{stats['min_y']:>8.2f}, {stats['max_y']:>8.2f}] mm
      Z Range: [{stats['min_z']:>8.2f}, {stats['max_z']:>8.2f}] mm
    ----------------------------------------
      X Span:          {stats['range_x']:>10.2f} mm
      Y Span:          {stats['range_y']:>10.2f} mm
      Z Span:          {stats['range_z']:>10.2f} mm
    ========================================
    
    Green Circle = Start Point
    Red Square = End Point  
    Red Arrow = Tool Z-axis Direction
    Blue Line = Trajectory Path
    """
    
    ax_info.text(0.1, 0.5, info_text, transform=ax_info.transAxes,
                 fontsize=10, verticalalignment='center',
                 fontfamily='Consolas',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    
    # 保存图形
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[INFO] 图形已保存: {save_path}")
    
    plt.show()
    
    return stats


def plot_trajectory_components(df: pd.DataFrame, figsize: tuple = (14, 8), save_path: str = None):
    """
    绘制轨迹各分量随时间变化图
    
    Args:
        df: 轨迹数据
        figsize: 图形尺寸
        save_path: 保存路径
    """
    time = df['TimeFromStart_s'].values
    
    fig, axes = plt.subplots(2, 3, figsize=figsize)
    
    # 位置分量
    axes[0, 0].plot(time, df['X_mm'], 'r-', linewidth=0.8)
    axes[0, 0].set_xlabel('时间 (s)')
    axes[0, 0].set_ylabel('X (mm)')
    axes[0, 0].set_title('X 位置')
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].plot(time, df['Y_mm'], 'g-', linewidth=0.8)
    axes[0, 1].set_xlabel('时间 (s)')
    axes[0, 1].set_ylabel('Y (mm)')
    axes[0, 1].set_title('Y 位置')
    axes[0, 1].grid(True, alpha=0.3)
    
    axes[0, 2].plot(time, df['Z_mm'], 'b-', linewidth=0.8)
    axes[0, 2].set_xlabel('时间 (s)')
    axes[0, 2].set_ylabel('Z (mm)')
    axes[0, 2].set_title('Z 位置')
    axes[0, 2].grid(True, alpha=0.3)
    
    # 姿态分量（旋转向量）
    if 'RX_rad' in df.columns:
        axes[1, 0].plot(time, np.rad2deg(df['RX_rad']), 'r-', linewidth=0.8)
        axes[1, 0].set_xlabel('时间 (s)')
        axes[1, 0].set_ylabel('RX (deg)')
        axes[1, 0].set_title('RX 旋转')
        axes[1, 0].grid(True, alpha=0.3)
        
        axes[1, 1].plot(time, np.rad2deg(df['RY_rad']), 'g-', linewidth=0.8)
        axes[1, 1].set_xlabel('时间 (s)')
        axes[1, 1].set_ylabel('RY (deg)')
        axes[1, 1].set_title('RY 旋转')
        axes[1, 1].grid(True, alpha=0.3)
        
        axes[1, 2].plot(time, np.rad2deg(df['RZ_rad']), 'b-', linewidth=0.8)
        axes[1, 2].set_xlabel('时间 (s)')
        axes[1, 2].set_ylabel('RZ (deg)')
        axes[1, 2].set_title('RZ 旋转')
        axes[1, 2].grid(True, alpha=0.3)
    
    plt.suptitle('轨迹位姿分量随时间变化', fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[INFO] 分量图已保存: {save_path}")
    
    plt.show()


def plot_velocity_profile(df: pd.DataFrame, figsize: tuple = (12, 6), save_path: str = None):
    """
    绘制速度剖面图
    
    Args:
        df: 轨迹数据
        figsize: 图形尺寸
        save_path: 保存路径
    """
    positions = df[['X_mm', 'Y_mm', 'Z_mm']].values
    time = df['TimeFromStart_s'].values
    
    # 计算位移和速度
    diffs = np.diff(positions, axis=0)
    dt = np.diff(time)
    dt[dt == 0] = 1e-6  # 避免除零
    
    velocities = np.linalg.norm(diffs, axis=1) / dt
    
    # 计算加速度
    accelerations = np.diff(velocities) / dt[:-1]
    
    fig, axes = plt.subplots(1, 2, figsize=figsize)
    
    # 速度曲线
    axes[0].plot(time[1:], velocities, 'b-', linewidth=0.8)
    axes[0].axhline(y=np.mean(velocities), color='r', linestyle='--', 
                    label=f'平均速度: {np.mean(velocities):.2f} mm/s')
    axes[0].set_xlabel('时间 (s)')
    axes[0].set_ylabel('速度 (mm/s)')
    axes[0].set_title('速度剖面')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # 加速度曲线
    axes[1].plot(time[2:], accelerations, 'g-', linewidth=0.8)
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_ylabel('加速度 (mm/s²)')
    axes[1].set_title('加速度剖面')
    axes[1].grid(True, alpha=0.3)
    
    plt.suptitle('运动学分析', fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[INFO] 速度图已保存: {save_path}")
    
    plt.show()


def plot_dual_trajectory_3d(df1: pd.DataFrame, 
                           df2: pd.DataFrame,
                           label1: str = "轨迹1",
                           label2: str = "轨迹2",
                           color1: str = 'royalblue',
                           color2: str = 'orangered',
                           show_orientation: bool = False,
                           orientation_interval: int = 100,
                           arrow_length: float = None,
                           figsize: tuple = (16, 12),
                           save_path: str = None):
    """
    绘制双轨迹对比 3D 图（大图幅，仅3D视图，支持鼠标旋转和滚轮缩放）
    
    Args:
        df1: 第一条轨迹数据
        df2: 第二条轨迹数据
        label1: 第一条轨迹标签
        label2: 第二条轨迹标签
        color1: 第一条轨迹颜色
        color2: 第二条轨迹颜色
        show_orientation: 是否显示姿态箭头
        orientation_interval: 姿态箭头间隔
        arrow_length: 箭头长度 (mm)
        figsize: 图形尺寸
        save_path: 保存路径
        
    交互操作:
        - 鼠标左键拖动：旋转视角
        - 鼠标右键拖动：平移视图
        - 鼠标滚轮：缩放视图
    """
    # 提取位置数据
    x1, y1, z1 = df1['X_mm'].values, df1['Y_mm'].values, df1['Z_mm'].values
    x2, y2, z2 = df2['X_mm'].values, df2['Y_mm'].values, df2['Z_mm'].values
    
    # 计算统计信息
    stats1 = calculate_trajectory_stats(df1)
    stats2 = calculate_trajectory_stats(df2)
    
    # 计算误差（如果两条轨迹点数相同）
    error_stats = None
    if len(df1) == len(df2):
        pos1 = df1[['X_mm', 'Y_mm', 'Z_mm']].values
        pos2 = df2[['X_mm', 'Y_mm', 'Z_mm']].values
        errors = np.linalg.norm(pos1 - pos2, axis=1)
        error_stats = {
            'mean': np.mean(errors),
            'std': np.std(errors),
            'rmse': np.sqrt(np.mean(errors**2)),
            'max': np.max(errors),
            'min': np.min(errors)
        }
        print(f"\n[INFO] 误差统计: 平均={error_stats['mean']:.4f}mm, RMSE={error_stats['rmse']:.4f}mm")
    
    # 创建大图幅的3D图形
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制两条轨迹
    ax.plot(x1, y1, z1, c=color1, linewidth=2.5, alpha=0.9, label=label1)
    ax.plot(x2, y2, z2, c=color2, linewidth=2.5, alpha=0.9, label=label2)
    
    # 起点和终点标记
    ax.scatter(x1[0], y1[0], z1[0], c=color1, s=200, marker='o', 
               edgecolors='darkgreen', linewidths=3, zorder=5)
    ax.scatter(x1[-1], y1[-1], z1[-1], c=color1, s=200, marker='s', 
               edgecolors='darkred', linewidths=3, zorder=5)
    
    ax.scatter(x2[0], y2[0], z2[0], c=color2, s=200, marker='o', 
               edgecolors='darkgreen', linewidths=3, zorder=5)
    ax.scatter(x2[-1], y2[-1], z2[-1], c=color2, s=200, marker='s', 
               edgecolors='darkred', linewidths=3, zorder=5)
    
    # 设置坐标轴标签（更大字体）
    ax.set_xlabel('X (mm)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Y (mm)', fontsize=14, fontweight='bold')
    ax.set_zlabel('Z (mm)', fontsize=14, fontweight='bold')
    
    # 标题（包含误差信息）
    if error_stats:
        title = f'双轨迹 3D 对比\n误差: 平均={error_stats["mean"]:.2f}mm, RMSE={error_stats["rmse"]:.2f}mm'
    else:
        title = f'双轨迹 3D 对比\n{label1}: {len(df1)}点, {label2}: {len(df2)}点'
    ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
    
    # 图例（更大字体）
    legend = ax.legend(loc='upper left', fontsize=12, framealpha=0.9)
    legend.get_frame().set_facecolor('white')
    legend.get_frame().set_edgecolor('gray')
    
    # 设置等比例坐标轴
    all_x = np.concatenate([x1, x2])
    all_y = np.concatenate([y1, y2])
    all_z = np.concatenate([z1, z2])
    max_range = max(all_x.max() - all_x.min(), 
                   all_y.max() - all_y.min(), 
                   all_z.max() - all_z.min()) / 2
    mid_x = (all_x.max() + all_x.min()) / 2
    mid_y = (all_y.max() + all_y.min()) / 2
    mid_z = (all_z.max() + all_z.min()) / 2
    ax.set_xlim(mid_x - max_range * 1.1, mid_x + max_range * 1.1)
    ax.set_ylim(mid_y - max_range * 1.1, mid_y + max_range * 1.1)
    ax.set_zlim(mid_z - max_range * 1.1, mid_z + max_range * 1.1)
    
    # ===== 添加鼠标滚轮缩放功能 =====
    # 保存初始范围用于缩放限制
    initial_max_range = max_range * 1.1
    
    def on_scroll(event):
        """鼠标滚轮缩放回调函数（增强版：更大步长 + 范围限制）"""
        if event.inaxes != ax:
            return
        
        # 获取当前坐标轴范围
        xlim = ax.get_xlim3d()
        ylim = ax.get_ylim3d()
        zlim = ax.get_zlim3d()
        
        # 计算当前中心点
        x_center = (xlim[0] + xlim[1]) / 2
        y_center = (ylim[0] + ylim[1]) / 2
        z_center = (zlim[0] + zlim[1]) / 2
        
        # 计算当前范围
        x_range = (xlim[1] - xlim[0]) / 2
        y_range = (ylim[1] - ylim[0]) / 2
        z_range = (zlim[1] - zlim[0]) / 2
        
        # 缩放因子（向上滚动缩小，向下滚动放大）- 增大到30%步长
        scale_factor = 0.7 if event.button == 'up' else 1.3
        
        # 应用缩放
        new_x_range = x_range * scale_factor
        new_y_range = y_range * scale_factor
        new_z_range = z_range * scale_factor
        
        # 缩放范围限制（防止过度缩小或放大）
        max_allowed_range = initial_max_range * 50  # 最大放大50倍
        min_allowed_range = initial_max_range * 0.1  # 最小缩小到0.1倍
        
        new_max_range = max(new_x_range, new_y_range, new_z_range)
        if new_max_range > max_allowed_range or new_max_range < min_allowed_range:
            return  # 超出限制，不执行缩放
        
        # 更新坐标轴范围
        ax.set_xlim3d([x_center - new_x_range, x_center + new_x_range])
        ax.set_ylim3d([y_center - new_y_range, y_center + new_y_range])
        ax.set_zlim3d([z_center - new_z_range, z_center + new_z_range])
        
        fig.canvas.draw_idle()
    
    # 连接滚轮事件
    fig.canvas.mpl_connect('scroll_event', on_scroll)
    
    # 设置网格
    ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    
    # 设置背景色
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('lightgray')
    ax.yaxis.pane.set_edgecolor('lightgray')
    ax.zaxis.pane.set_edgecolor('lightgray')
    
    # 调整布局
    plt.tight_layout()
    
    # 打印操作提示
    print("\n" + "="*60)
    print("交互操作提示:")
    print("  - 鼠标左键拖动：旋转视角")
    print("  - 鼠标右键拖动：平移视图")
    print("  - 鼠标滚轮：缩放视图")
    print("="*60)
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[INFO] 双轨迹对比图已保存: {save_path}")
    
    plt.show()
    
    return error_stats if error_stats else {'message': '点数不同，无法计算误差'}


# ==================== 主程序 ====================
if __name__ == "__main__":
    # ===== 使用模式选择 =====
    # 设置为 True: 双轨迹对比模式
    # 设置为 False: 单轨迹分析模式
    DUAL_TRAJECTORY_MODE = True
    
    if DUAL_TRAJECTORY_MODE:
        # ========== 双轨迹对比模式 ==========
        print("\n" + "="*60)
        print("双轨迹对比可视化模式")
        print("="*60)
        
        # 配置两个 CSV 文件路径
        csv_path1 = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\tcp6 - tcp.csv"
        csv_path2 = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\tracker6 - tcp_transformed.csv"
        
        # 加载两条轨迹
        print("\n[INFO] 加载第一条轨迹...")
        df1 = load_trajectory_csv(csv_path1)
        
        print("\n[INFO] 加载第二条轨迹...")
        df2 = load_trajectory_csv(csv_path2)
        
        # 绘制双轨迹对比图
        print("\n" + "="*60)
        print("绘制双轨迹 3D 对比图...")
        print("="*60)
        
        error_stats = plot_dual_trajectory_3d(
            df1, 
            df2,
            label1="变换后轨迹",      # 第一条轨迹的标签
            label2="参考轨迹",        # 第二条轨迹的标签
            color1='royalblue',      # 第一条轨迹颜色（蓝色）
            color2='orangered',      # 第二条轨迹颜色（橙红色）
            show_orientation=False,  # 是否显示姿态箭头
            figsize=(18, 14),        # 大图幅
            save_path=None           # 设置路径可保存，如 "dual_trajectory_3d.png"
        )
        
        # 打印误差统计
        if isinstance(error_stats, dict) and 'mean' in error_stats:
            print("\n" + "="*60)
            print("误差统计结果:")
            print("="*60)
            print(f"  平均误差: {error_stats['mean']:.4f} mm")
            print(f"  标准差:   {error_stats['std']:.4f} mm")
            print(f"  RMSE:     {error_stats['rmse']:.4f} mm")
            print(f"  最大误差: {error_stats['max']:.4f} mm")
            print(f"  最小误差: {error_stats['min']:.4f} mm")
        else:
            print(f"\n[WARN] {error_stats.get('message', '无法计算误差')}")
        
    else:
        # ========== 单轨迹分析模式 ==========
        print("\n" + "="*60)
        print("单轨迹分析模式")
        print("="*60)
        
        # CSV 文件路径
        csv_path = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\PlaybackRecord_HighFreq_7_20260204_200642_7_20260204_200919.csv"
        # 加载数据
        df = load_trajectory_csv(csv_path)
        
        # 打印前几行数据预览
        print("\n[INFO] 数据预览:")
        print(df.head())
        
        # ===== 1. 绘制 3D 轨迹图 =====
        print("\n" + "="*60)
        print("绘制 3D 轨迹图...")
        print("="*60)
        
        stats = plot_3d_trajectory(
            df,
            show_orientation=True,      # 显示姿态箭头
            orientation_interval=100,   # 每 100 帧显示一个箭头
            arrow_length=None,          # 自动计算箭头长度
            figsize=(14, 10),
            save_path=None              # 设置路径可保存，如 "trajectory_3d.png"
        )
        
        # ===== 2. 绘制位姿分量图 =====
        print("\n" + "="*60)
        print("绘制位姿分量图...")
        print("="*60)
        
        plot_trajectory_components(
            df,
            figsize=(14, 8),
            save_path=None
        )
        
        # ===== 3. 绘制速度剖面图 =====
        print("\n" + "="*60)
        print("绘制速度剖面图...")
        print("="*60)
        
        plot_velocity_profile(
            df,
            figsize=(12, 6),
            save_path=None
        )
    
    print("\n[INFO] 可视化完成!")

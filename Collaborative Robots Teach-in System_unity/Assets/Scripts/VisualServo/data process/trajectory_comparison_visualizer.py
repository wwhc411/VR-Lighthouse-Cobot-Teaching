# -*- coding: utf-8 -*-
"""
双轨迹对比可视化工具
同时加载并对比两个 CSV 轨迹文件

功能:
1. 双轨迹 3D 对比可视化（不同颜色）
2. 起点/终点标记
3. 姿态方向箭头显示（可选）
4. 轨迹统计对比
5. 多视角投影对比
6. 轨迹差异分析

Author: GitHub Copilot
Date: 2026-01-28
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

from scipy import interpolate
from scipy.ndimage import gaussian_filter1d


# ============================================================================
#                        预处理与对齐函数
# ============================================================================

# ⭐⭐⭐ 预处理配置（与 tunable_registration.py 保持一致）⭐⭐⭐
PREPROCESS_CONFIG = {
    'enable_smoothing': True,
    'gaussian_sigma': 1.0,  # 与配准工具一致
}


def smooth_gaussian(points: np.ndarray, sigma: float = 1.0) -> np.ndarray:
    """
    高斯平滑轨迹（与 tunable_registration.py 一致）
    
    Args:
        points: Nx3 点云数组
        sigma: 高斯核标准差
        
    Returns:
        平滑后的 Nx3 数组
    """
    smoothed = np.zeros_like(points)
    for i in range(3):
        smoothed[:, i] = gaussian_filter1d(points[:, i], sigma=sigma)
    return smoothed


def compute_arc_length(points: np.ndarray) -> np.ndarray:
    """计算累积弧长"""
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    arc_length = np.concatenate([[0], np.cumsum(segment_lengths)])
    return arc_length


def resample_by_arc_length(points: np.ndarray, num_samples: int = 5000) -> tuple:
    """
    按弧长等距离重采样
    
    Args:
        points: Nx3 位置数组
        num_samples: 重采样点数
        
    Returns:
        (重采样后的点, 总弧长)
    """
    arc_length = compute_arc_length(points)
    total_length = arc_length[-1]
    
    # 目标弧长位置
    target_arc = np.linspace(0, total_length, num_samples)
    
    # 插值
    resampled = np.zeros((num_samples, 3))
    for i in range(3):
        f = interpolate.interp1d(arc_length, points[:, i], kind='linear', fill_value='extrapolate')
        resampled[:, i] = f(target_arc)
    
    return resampled, total_length


def align_trajectories_by_arc_length(df1: pd.DataFrame, df2: pd.DataFrame, 
                                      num_samples: int = 5000,
                                      apply_preprocessing: bool = False,
                                      preprocess_traj1: bool = True,
                                      preprocess_traj2: bool = True) -> tuple:
    """
    按弧长对齐两条轨迹（与 tunable_registration.py 流程一致）
    
    Args:
        df1: 第一条轨迹 DataFrame
        df2: 第二条轨迹 DataFrame
        num_samples: 重采样点数
        apply_preprocessing: 是否应用预处理（高斯平滑）
        preprocess_traj1: 是否对轨迹1预处理（如果已经是变换后的轨迹，设为False）
        preprocess_traj2: 是否对轨迹2预处理
        
    Returns:
        (对齐后的点1, 对齐后的点2, 统计信息)
    """
    pos1 = df1[['X_mm', 'Y_mm', 'Z_mm']].values
    pos2 = df2[['X_mm', 'Y_mm', 'Z_mm']].values
    
    # 原始弧长
    original_len1 = compute_arc_length(pos1)[-1]
    original_len2 = compute_arc_length(pos2)[-1]
    
    # 弧长重采样（先重采样再平滑，与配准流程一致）
    aligned1, len1 = resample_by_arc_length(pos1, num_samples)
    aligned2, len2 = resample_by_arc_length(pos2, num_samples)
    
    # ⭐ 预处理（高斯平滑）- 单独控制每条轨迹
    traj1_smoothed = False
    traj2_smoothed = False
    if apply_preprocessing and PREPROCESS_CONFIG['enable_smoothing']:
        sigma = PREPROCESS_CONFIG['gaussian_sigma']
        if preprocess_traj1:
            aligned1 = smooth_gaussian(aligned1, sigma)
            traj1_smoothed = True
        if preprocess_traj2:
            aligned2 = smooth_gaussian(aligned2, sigma)
            traj2_smoothed = True
        # 更新平滑后的弧长
        len1 = compute_arc_length(aligned1)[-1]
        len2 = compute_arc_length(aligned2)[-1]
        len1 = compute_arc_length(aligned1)[-1]
        len2 = compute_arc_length(aligned2)[-1]
    
    info = {
        'original_len1': len(df1),
        'original_len2': len(df2),
        'resampled_len': num_samples,
        'arc_length1': len1,
        'arc_length2': len2,
        'original_arc_length1': original_len1,
        'original_arc_length2': original_len2,
        'length_ratio': len1 / len2 if len2 > 0 else 0,
        'preprocessed': apply_preprocessing,
        'traj1_smoothed': traj1_smoothed,
        'traj2_smoothed': traj2_smoothed
    }
    
    return aligned1, aligned2, info


def load_trajectory_csv(csv_path: str, convert_to_mm: bool = True) -> pd.DataFrame:
    """
    加载 CSV/Excel 轨迹文件
    
    Args:
        csv_path: CSV或Excel文件路径
        convert_to_mm: 是否将数据转换为毫米（检测到米单位时）
        
    Returns:
        pandas DataFrame
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"文件不存在: {csv_path}")
    
    # 自动识别文件类型
    file_ext = os.path.splitext(csv_path)[1].lower()
    if file_ext in ['.xlsx', '.xls']:
        df = pd.read_excel(csv_path)
        print(f"[INFO] 读取Excel文件格式")
    else:
        df = pd.read_csv(csv_path)
    
    # 检测数据单位（如果X/Y/Z范围小于1，可能是米单位）
    x_range = df['X_mm'].max() - df['X_mm'].min()
    y_range = df['Y_mm'].max() - df['Y_mm'].min()
    z_range = df['Z_mm'].max() - df['Z_mm'].min()
    
    is_meter_unit = (x_range < 10 and y_range < 10 and z_range < 10)
    
    if is_meter_unit and convert_to_mm:
        print(f"[WARN] 检测到数据可能是米单位，自动转换为毫米")
        df['X_mm'] = df['X_mm'] * 1000
        df['Y_mm'] = df['Y_mm'] * 1000
        df['Z_mm'] = df['Z_mm'] * 1000
        print(f"[INFO] 转换后范围: X={df['X_mm'].max()-df['X_mm'].min():.1f}mm, "
              f"Y={df['Y_mm'].max()-df['Y_mm'].min():.1f}mm, "
              f"Z={df['Z_mm'].max()-df['Z_mm'].min():.1f}mm")
    
    print(f"[INFO] 加载轨迹文件: {csv_path}")
    print(f"[INFO] 总帧数: {len(df)}")
    
    return df


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


def calculate_trajectory_stats(df: pd.DataFrame, label: str = "") -> dict:
    """
    计算轨迹统计信息
    
    Args:
        df: 轨迹数据
        label: 轨迹标签
        
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
    
    # 计算时间跨度（如果有时间列）
    if 'TimeFromStart_s' in df.columns:
        time_span = df['TimeFromStart_s'].iloc[-1] - df['TimeFromStart_s'].iloc[0]
        avg_speed = total_length / time_span if time_span > 0 else 0
    else:
        time_span = 0
        avg_speed = 0
    
    stats = {
        'label': label,
        'total_frames': len(df),
        'total_length_mm': total_length,
        'time_span_s': time_span,
        'avg_speed_mm_s': avg_speed,
        'min_x': min_pos[0], 'max_x': max_pos[0],
        'min_y': min_pos[1], 'max_y': max_pos[1],
        'min_z': min_pos[2], 'max_z': max_pos[2],
        'range_x': max_pos[0] - min_pos[0],
        'range_y': max_pos[1] - min_pos[1],
        'range_z': max_pos[2] - min_pos[2],
    }
    
    return stats


def calculate_trajectory_difference(df1: pd.DataFrame, df2: pd.DataFrame, 
                                     use_arc_length_align: bool = False,
                                     num_samples: int = 5000,
                                     apply_preprocessing: bool = False,
                                     preprocess_traj1: bool = True,
                                     preprocess_traj2: bool = True) -> dict:
    """
    计算两条轨迹的差异
    
    Args:
        df1: 第一条轨迹
        df2: 第二条轨迹
        use_arc_length_align: 是否使用弧长对齐（推荐！与配准工具一致）
        num_samples: 弧长重采样点数
        apply_preprocessing: ⭐ 是否应用预处理（与配准流程一致）
        preprocess_traj1: 是否对轨迹1预处理（已变换的轨迹设为False避免双重平滑）
        preprocess_traj2: 是否对轨迹2预处理
        
    Returns:
        差异统计信息
    """
    if use_arc_length_align:
        # 使用弧长对齐（与 tunable_registration.py 一致）
        pos1, pos2, align_info = align_trajectories_by_arc_length(
            df1, df2, num_samples, 
            apply_preprocessing=apply_preprocessing,
            preprocess_traj1=preprocess_traj1,
            preprocess_traj2=preprocess_traj2
        )
        print(f"[INFO] 使用弧长对齐: {num_samples}点")
        if apply_preprocessing:
            sigma = PREPROCESS_CONFIG['gaussian_sigma']
            print(f"       ⭐ 高斯平滑 (sigma={sigma})")
            if align_info['traj1_smoothed']:
                print(f"       轨迹1: {align_info['original_arc_length1']:.2f}mm -> 平滑后: {align_info['arc_length1']:.2f}mm")
            else:
                print(f"       轨迹1: {align_info['arc_length1']:.2f}mm (未平滑，已是变换后数据)")
            if align_info['traj2_smoothed']:
                print(f"       轨迹2: {align_info['original_arc_length2']:.2f}mm -> 平滑后: {align_info['arc_length2']:.2f}mm")
            else:
                print(f"       轨迹2: {align_info['arc_length2']:.2f}mm (未平滑)")
        else:
            print(f"       轨迹1弧长: {align_info['arc_length1']:.2f}mm, "
                  f"轨迹2弧长: {align_info['arc_length2']:.2f}mm")
        print(f"       长度比: {align_info['length_ratio']:.4f}")
        compared_frames = num_samples
    else:
        # 直接按索引对应（原始方式）
        min_len = min(len(df1), len(df2))
        pos1 = df1[['X_mm', 'Y_mm', 'Z_mm']].values[:min_len]
        pos2 = df2[['X_mm', 'Y_mm', 'Z_mm']].values[:min_len]
        compared_frames = min_len
    
    # 计算点对点距离
    distances = np.linalg.norm(pos1 - pos2, axis=1)
    
    # 计算RMSE（与tunable_registration.py一致）
    rmse = np.sqrt(np.mean(distances**2))
    
    diff_stats = {
        'compared_frames': compared_frames,
        'mean_distance_mm': np.mean(distances),
        'rmse_mm': rmse,  # 新增RMSE
        'max_distance_mm': np.max(distances),
        'min_distance_mm': np.min(distances),
        'std_distance_mm': np.std(distances),
        'median_distance_mm': np.median(distances),
        'p95_distance_mm': np.percentile(distances, 95),  # 新增P95
        'use_arc_length': use_arc_length_align,  # 标记对齐方式
    }
    
    return diff_stats


def plot_dual_trajectory_3d(df1: pd.DataFrame, 
                            df2: pd.DataFrame,
                            label1: str = "轨迹1",
                            label2: str = "轨迹2",
                            color1: str = 'royalblue',
                            color2: str = 'orangered',
                            show_orientation: bool = True,
                            orientation_interval: int = 100,
                            arrow_length: float = None,
                            figsize: tuple = (16, 10),
                            save_path: str = None,
                            use_arc_length_align: bool = True,  # ⭐ 默认开启弧长对齐
                            num_samples: int = 5000,            # ⭐ 与配准工具一致
                            apply_preprocessing: bool = False,  # ⭐ 预处理（与配准流程一致）
                            preprocess_traj1: bool = True,      # ⭐ 是否对轨迹1预处理
                            preprocess_traj2: bool = True):     # ⭐ 是否对轨迹2预处理
    """
    绘制双轨迹对比 3D 图
    
    Args:
        df1: 第一条轨迹数据
        df2: 第二条轨迹数据
        label1: 第一条轨迹标签
        label2: 第二条轨迹标签
        color1: 第一条轨迹颜色
        color2: 第二条轨迹颜色
        show_orientation: 是否显示姿态箭头
        orientation_interval: 姿态箭头间隔
        arrow_length: 箭头长度
        figsize: 图形尺寸
        save_path: 保存路径
        use_arc_length_align: ⭐ 是否使用弧长对齐计算误差（推荐开启，与配准工具一致）
        num_samples: 弧长重采样点数
        apply_preprocessing: ⭐ 是否应用预处理（高斯平滑，与配准流程一致）
        preprocess_traj1: ⭐ 是否对轨迹1预处理（已变换的轨迹设False避免双重平滑）
        preprocess_traj2: ⭐ 是否对轨迹2预处理
    """
    # 提取位置数据
    x1 = df1['X_mm'].values
    y1 = df1['Y_mm'].values
    z1 = df1['Z_mm'].values
    
    x2 = df2['X_mm'].values
    y2 = df2['Y_mm'].values
    z2 = df2['Z_mm'].values
    
    # 计算统计信息
    stats1 = calculate_trajectory_stats(df1, label1)
    stats2 = calculate_trajectory_stats(df2, label2)
    # ⭐ 使用弧长对齐计算误差（与tunable_registration.py一致）
    diff_stats = calculate_trajectory_difference(df1, df2, 
                                                  use_arc_length_align=use_arc_length_align,
                                                  num_samples=num_samples,
                                                  apply_preprocessing=apply_preprocessing,
                                                  preprocess_traj1=preprocess_traj1,
                                                  preprocess_traj2=preprocess_traj2)
    
    # 计算全局边界
    all_x = np.concatenate([x1, x2])
    all_y = np.concatenate([y1, y2])
    all_z = np.concatenate([z1, z2])
    
    min_x, max_x = all_x.min(), all_x.max()
    min_y, max_y = all_y.min(), all_y.max()
    min_z, max_z = all_z.min(), all_z.max()
    
    # 自动计算箭头长度
    if arrow_length is None:
        max_range = max(max_x - min_x, max_y - min_y, max_z - min_z)
        arrow_length = max_range * 0.05
    
    # 创建图形（使用GridSpec布局）
    from matplotlib.gridspec import GridSpec
    fig = plt.figure(figsize=figsize)
    gs = GridSpec(2, 2, figure=fig, width_ratios=[2, 1], height_ratios=[1, 1], 
                  hspace=0.3, wspace=0.3)
    
    # ===== 主 3D 视图（左侧占据整个高度）=====
    ax_main = fig.add_subplot(gs[:, 0], projection='3d')
    
    # 绘制轨迹1
    sample_interval1 = max(1, len(x1) // 100)
    ax_main.plot(x1, y1, z1, c=color1, linewidth=2.5, alpha=0.8, label=label1)
    ax_main.scatter(x1[::sample_interval1], y1[::sample_interval1], z1[::sample_interval1], 
                   c=color1, s=20, alpha=0.6, edgecolors='none', zorder=3)
    ax_main.scatter(x1[0], y1[0], z1[0], c=color1, s=150, marker='o', 
                    edgecolors='darkgreen', linewidths=2, zorder=5, label=f'{label1} 起点')
    ax_main.scatter(x1[-1], y1[-1], z1[-1], c=color1, s=150, marker='s', 
                    edgecolors='darkred', linewidths=2, zorder=5, label=f'{label1} 终点')
    
    # 绘制轨迹2
    sample_interval2 = max(1, len(x2) // 100)
    ax_main.plot(x2, y2, z2, c=color2, linewidth=2.5, alpha=0.8, label=label2)
    ax_main.scatter(x2[::sample_interval2], y2[::sample_interval2], z2[::sample_interval2], 
                   c=color2, s=20, alpha=0.6, edgecolors='none', zorder=3)
    ax_main.scatter(x2[0], y2[0], z2[0], c=color2, s=150, marker='o', 
                    edgecolors='darkgreen', linewidths=2, zorder=5, label=f'{label2} 起点')
    ax_main.scatter(x2[-1], y2[-1], z2[-1], c=color2, s=150, marker='s', 
                    edgecolors='darkred', linewidths=2, zorder=5, label=f'{label2} 终点')
    
    # 绘制姿态箭头（轨迹1）
    if show_orientation and 'QX' in df1.columns:
        arrow_indices = range(0, len(df1), orientation_interval)
        for i in arrow_indices:
            qx = df1['QX'].iloc[i]
            qy = df1['QY'].iloc[i]
            qz = df1['QZ'].iloc[i]
            qw = df1['QW'].iloc[i]
            
            rot_mat = quaternion_to_rotation_matrix(qx, qy, qz, qw)
            z_axis = rot_mat[:, 2] * arrow_length
            ax_main.quiver(x1[i], y1[i], z1[i], 
                          z_axis[0], z_axis[1], z_axis[2],
                          color=color1, alpha=0.4, arrow_length_ratio=0.3, 
                          linewidth=1.2, zorder=2)
    
    # 绘制姿态箭头（轨迹2）
    if show_orientation and 'QX' in df2.columns:
        arrow_indices = range(0, len(df2), orientation_interval)
        for i in arrow_indices:
            qx = df2['QX'].iloc[i]
            qy = df2['QY'].iloc[i]
            qz = df2['QZ'].iloc[i]
            qw = df2['QW'].iloc[i]
            
            rot_mat = quaternion_to_rotation_matrix(qx, qy, qz, qw)
            z_axis = rot_mat[:, 2] * arrow_length
            ax_main.quiver(x2[i], y2[i], z2[i], 
                          z_axis[0], z_axis[1], z_axis[2],
                          color=color2, alpha=0.4, arrow_length_ratio=0.3, 
                          linewidth=1.2, zorder=2)
    
    ax_main.set_xlabel('X (mm)', fontsize=10)
    ax_main.set_ylabel('Y (mm)', fontsize=10)
    ax_main.set_zlabel('Z (mm)', fontsize=10)
    ax_main.set_title('双轨迹 3D 对比视图', fontsize=12, fontweight='bold')
    ax_main.legend(loc='upper left', fontsize=8)
    
    # 设置等比例坐标轴
    max_range = max(max_x - min_x, max_y - min_y, max_z - min_z) / 2
    mid_x = (min_x + max_x) / 2
    mid_y = (min_y + max_y) / 2
    mid_z = (min_z + max_z) / 2
    ax_main.set_xlim(mid_x - max_range * 1.1, mid_x + max_range * 1.1)
    ax_main.set_ylim(mid_y - max_range * 1.1, mid_y + max_range * 1.1)
    ax_main.set_zlim(mid_z - max_range * 1.1, mid_z + max_range * 1.1)
    
    # 添加滚轮缩放功能
    def on_scroll(event):
        if event.inaxes != ax_main:
            return
        
        xlim = ax_main.get_xlim3d()
        ylim = ax_main.get_ylim3d()
        zlim = ax_main.get_zlim3d()
        
        x_center = (xlim[0] + xlim[1]) / 2
        y_center = (ylim[0] + ylim[1]) / 2
        z_center = (zlim[0] + zlim[1]) / 2
        
        x_range = (xlim[1] - xlim[0]) / 2
        y_range = (ylim[1] - ylim[0]) / 2
        z_range = (zlim[1] - zlim[0]) / 2
        
        scale_factor = 0.9 if event.button == 'up' else 1.1
        
        new_x_range = x_range * scale_factor
        new_y_range = y_range * scale_factor
        new_z_range = z_range * scale_factor
        
        ax_main.set_xlim3d([x_center - new_x_range, x_center + new_x_range])
        ax_main.set_ylim3d([y_center - new_y_range, y_center + new_y_range])
        ax_main.set_zlim3d([z_center - new_z_range, z_center + new_z_range])
        
        fig.canvas.draw_idle()
    
    fig.canvas.mpl_connect('scroll_event', on_scroll)
    
    # ===== 轨迹差异热图（右上）=====
    ax_diff = fig.add_subplot(gs[0, 1])
    
    # 根据对齐方式计算距离
    if use_arc_length_align:
        # 弧长对齐后的距离（与配准工具一致）
        aligned1, aligned2, _ = align_trajectories_by_arc_length(df1, df2, num_samples)
        distances = np.linalg.norm(aligned1 - aligned2, axis=1)
        # 使用归一化弧长作为X轴
        arc_progress = np.linspace(0, 1, num_samples)
        scatter = ax_diff.scatter(arc_progress, distances, c=distances, cmap='hot', s=10, alpha=0.7)
        ax_diff.plot(arc_progress, distances, 'b-', alpha=0.3, linewidth=0.5)
        ax_diff.set_xlabel('归一化弧长位置', fontsize=9)
    else:
        # 原始索引对应
        min_len = min(len(df1), len(df2))
        pos1 = df1[['X_mm', 'Y_mm', 'Z_mm']].values[:min_len]
        pos2 = df2[['X_mm', 'Y_mm', 'Z_mm']].values[:min_len]
        distances = np.linalg.norm(pos1 - pos2, axis=1)
        
        # 使用时间轴（如果有）或索引
        if 'TimeFromStart_s' in df1.columns:
            x_axis = df1['TimeFromStart_s'].values[:min_len]
            x_label = '时间 (s)'
        else:
            x_axis = np.arange(min_len)
            x_label = '点索引'
        
        scatter = ax_diff.scatter(x_axis, distances, c=distances, cmap='hot', s=10, alpha=0.7)
        ax_diff.plot(x_axis, distances, 'b-', alpha=0.3, linewidth=0.5)
        ax_diff.set_xlabel(x_label, fontsize=9)
    
    ax_diff.axhline(y=diff_stats['mean_distance_mm'], color='g', linestyle='--', 
                   label=f"平均: {diff_stats['mean_distance_mm']:.2f} mm")
    ax_diff.set_ylabel('点对点距离 (mm)', fontsize=9)
    
    # 根据对齐方式显示标题
    align_method = "弧长对齐" if use_arc_length_align else "索引对齐"
    ax_diff.set_title(f'轨迹差异分析 ({align_method})', fontsize=11, fontweight='bold')
    ax_diff.legend(fontsize=8)
    ax_diff.grid(True, alpha=0.3)
    cbar = plt.colorbar(scatter, ax=ax_diff)
    cbar.set_label('距离 (mm)', fontsize=8)
    
    # ===== 统计信息面板（右下）=====
    ax_info = fig.add_subplot(gs[1, 1])
    ax_info.axis('off')
    
    # 确定对齐方式文字
    align_method_text = "弧长对齐(Arc Length)" if use_arc_length_align else "索引对齐(Index)"
    
    info_text = f"""
    ========================================
            Trajectory Comparison
         对齐方式: {align_method_text}
    ========================================
    
    【{label1}】
      Frames:    {stats1['total_frames']:>10}
      Length:    {stats1['total_length_mm']:>10.2f} mm
      Duration:  {stats1['time_span_s']:>10.2f} s
      Avg Speed: {stats1['avg_speed_mm_s']:>10.2f} mm/s

    【{label2}】
      Frames:    {stats2['total_frames']:>10}
      Length:    {stats2['total_length_mm']:>10.2f} mm
      Duration:  {stats2['time_span_s']:>10.2f} s
      Avg Speed: {stats2['avg_speed_mm_s']:>10.2f} mm/s
    
    ----------------------------------------
         ⭐ Difference Analysis ⭐
    ----------------------------------------
      Compared:  {diff_stats['compared_frames']:>10} pts
      Mean Dist: {diff_stats['mean_distance_mm']:>10.2f} mm
      RMSE:      {diff_stats['rmse_mm']:>10.2f} mm
      Max Dist:  {diff_stats['max_distance_mm']:>10.2f} mm
      Min Dist:  {diff_stats['min_distance_mm']:>10.2f} mm
      P95:       {diff_stats['p95_distance_mm']:>10.2f} mm
      Std Dev:   {diff_stats['std_distance_mm']:>10.2f} mm
    ========================================
    """
    
    ax_info.text(0.05, 0.5, info_text, transform=ax_info.transAxes,
                 fontsize=9, verticalalignment='center',
                 fontfamily='Consolas',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    
    # 保存图形
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[INFO] 对比图已保存: {save_path}")
    
    plt.show()
    
    return stats1, stats2, diff_stats


def plot_dual_trajectory_components(df1: pd.DataFrame, 
                                    df2: pd.DataFrame,
                                    label1: str = "轨迹1",
                                    label2: str = "轨迹2",
                                    color1: str = 'royalblue',
                                    color2: str = 'orangered',
                                    figsize: tuple = (14, 8), 
                                    save_path: str = None):
    """
    绘制双轨迹各分量对比图
    
    Args:
        df1: 第一条轨迹
        df2: 第二条轨迹
        label1: 第一条轨迹标签
        label2: 第二条轨迹标签
        color1: 第一条轨迹颜色
        color2: 第二条轨迹颜色
        figsize: 图形尺寸
        save_path: 保存路径
    """
    # 检查是否有时间列
    has_time1 = 'TimeFromStart_s' in df1.columns
    has_time2 = 'TimeFromStart_s' in df2.columns
    
    if has_time1:
        time1 = df1['TimeFromStart_s'].values
    else:
        time1 = np.arange(len(df1))
        
    if has_time2:
        time2 = df2['TimeFromStart_s'].values
    else:
        time2 = np.arange(len(df2))
    
    x_label = '时间 (s)' if (has_time1 and has_time2) else '点索引'
    
    fig, axes = plt.subplots(2, 3, figsize=figsize)
    
    # 位置分量
    axes[0, 0].plot(time1, df1['X_mm'], color=color1, linewidth=1.2, alpha=0.8, label=label1)
    axes[0, 0].plot(time2, df2['X_mm'], color=color2, linewidth=1.2, alpha=0.8, label=label2)
    axes[0, 0].set_xlabel(x_label)
    axes[0, 0].set_ylabel('X (mm)')
    axes[0, 0].set_title('X 位置对比')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].plot(time1, df1['Y_mm'], color=color1, linewidth=1.2, alpha=0.8, label=label1)
    axes[0, 1].plot(time2, df2['Y_mm'], color=color2, linewidth=1.2, alpha=0.8, label=label2)
    axes[0, 1].set_xlabel(x_label)
    axes[0, 1].set_ylabel('Y (mm)')
    axes[0, 1].set_title('Y 位置对比')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    axes[0, 2].plot(time1, df1['Z_mm'], color=color1, linewidth=1.2, alpha=0.8, label=label1)
    axes[0, 2].plot(time2, df2['Z_mm'], color=color2, linewidth=1.2, alpha=0.8, label=label2)
    axes[0, 2].set_xlabel(x_label)
    axes[0, 2].set_ylabel('Z (mm)')
    axes[0, 2].set_title('Z 位置对比')
    axes[0, 2].legend()
    axes[0, 2].grid(True, alpha=0.3)
    
    # 姿态分量（旋转向量）
    if 'RX_rad' in df1.columns and 'RX_rad' in df2.columns:
        axes[1, 0].plot(time1, np.rad2deg(df1['RX_rad']), color=color1, linewidth=1.2, alpha=0.8, label=label1)
        axes[1, 0].plot(time2, np.rad2deg(df2['RX_rad']), color=color2, linewidth=1.2, alpha=0.8, label=label2)
        axes[1, 0].set_xlabel(x_label)
        axes[1, 0].set_ylabel('RX (deg)')
        axes[1, 0].set_title('RX 旋转对比')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        axes[1, 1].plot(time1, np.rad2deg(df1['RY_rad']), color=color1, linewidth=1.2, alpha=0.8, label=label1)
        axes[1, 1].plot(time2, np.rad2deg(df2['RY_rad']), color=color2, linewidth=1.2, alpha=0.8, label=label2)
        axes[1, 1].set_xlabel(x_label)
        axes[1, 1].set_ylabel('RY (deg)')
        axes[1, 1].set_title('RY 旋转对比')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        axes[1, 2].plot(time1, np.rad2deg(df1['RZ_rad']), color=color1, linewidth=1.2, alpha=0.8, label=label1)
        axes[1, 2].plot(time2, np.rad2deg(df2['RZ_rad']), color=color2, linewidth=1.2, alpha=0.8, label=label2)
        axes[1, 2].set_xlabel(x_label)
        axes[1, 2].set_ylabel('RZ (deg)')
        axes[1, 2].set_title('RZ 旋转对比')
        axes[1, 2].legend()
        axes[1, 2].grid(True, alpha=0.3)
    
    plt.suptitle('双轨迹位姿分量对比', fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[INFO] 分量对比图已保存: {save_path}")
    
    plt.show()


def plot_dual_velocity_profile(df1: pd.DataFrame, 
                               df2: pd.DataFrame,
                               label1: str = "轨迹1",
                               label2: str = "轨迹2",
                               color1: str = 'royalblue',
                               color2: str = 'orangered',
                               figsize: tuple = (14, 6), 
                               save_path: str = None):
    """
    绘制双轨迹速度对比图
    
    Args:
        df1: 第一条轨迹
        df2: 第二条轨迹
        label1: 第一条轨迹标签
        label2: 第二条轨迹标签
        color1: 第一条轨迹颜色
        color2: 第二条轨迹颜色
        figsize: 图形尺寸
        save_path: 保存路径
    """
    # 检查时间列
    has_time1 = 'TimeFromStart_s' in df1.columns
    has_time2 = 'TimeFromStart_s' in df2.columns
    
    if not has_time1 or not has_time2:
        print("[WARN] 速度分析需要 TimeFromStart_s 列，跳过速度对比图")
        return
    
    # 计算速度 - 轨迹1
    positions1 = df1[['X_mm', 'Y_mm', 'Z_mm']].values
    time1 = df1['TimeFromStart_s'].values
    diffs1 = np.diff(positions1, axis=0)
    dt1 = np.diff(time1)
    dt1[dt1 == 0] = 1e-6
    velocities1 = np.linalg.norm(diffs1, axis=1) / dt1
    
    # 计算速度 - 轨迹2
    positions2 = df2[['X_mm', 'Y_mm', 'Z_mm']].values
    time2 = df2['TimeFromStart_s'].values
    diffs2 = np.diff(positions2, axis=0)
    dt2 = np.diff(time2)
    dt2[dt2 == 0] = 1e-6
    velocities2 = np.linalg.norm(diffs2, axis=1) / dt2
    
    # 计算加速度
    accelerations1 = np.diff(velocities1) / dt1[:-1]
    accelerations2 = np.diff(velocities2) / dt2[:-1]
    
    fig, axes = plt.subplots(1, 2, figsize=figsize)
    
    # 速度曲线
    axes[0].plot(time1[1:], velocities1, color=color1, linewidth=1.2, alpha=0.8, label=label1)
    axes[0].plot(time2[1:], velocities2, color=color2, linewidth=1.2, alpha=0.8, label=label2)
    axes[0].axhline(y=np.mean(velocities1), color=color1, linestyle='--', alpha=0.5,
                   label=f'{label1} 平均: {np.mean(velocities1):.2f} mm/s')
    axes[0].axhline(y=np.mean(velocities2), color=color2, linestyle='--', alpha=0.5,
                   label=f'{label2} 平均: {np.mean(velocities2):.2f} mm/s')
    axes[0].set_xlabel('时间 (s)')
    axes[0].set_ylabel('速度 (mm/s)')
    axes[0].set_title('速度剖面对比')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # 加速度曲线
    axes[1].plot(time1[2:], accelerations1, color=color1, linewidth=1.2, alpha=0.8, label=label1)
    axes[1].plot(time2[2:], accelerations2, color=color2, linewidth=1.2, alpha=0.8, label=label2)
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_ylabel('加速度 (mm/s²)')
    axes[1].set_title('加速度剖面对比')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    plt.suptitle('双轨迹运动学对比', fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[INFO] 速度对比图已保存: {save_path}")
    
    plt.show()


def plot_error_spatial_distribution(df1: pd.DataFrame,
                                    df2: pd.DataFrame,
                                    label1: str = "轨迹1",
                                    label2: str = "轨迹2",
                                    use_arc_length_align: bool = True,
                                    num_samples: int = 5000,
                                    apply_preprocessing: bool = False,
                                    preprocess_traj1: bool = True,
                                    preprocess_traj2: bool = True,
                                    figsize: tuple = (18, 12),
                                    save_path: str = None):
    """
    绘制误差空间分布图（3D散点图，颜色表示误差大小）
    
    Args:
        df1: 第一条轨迹数据
        df2: 第二条轨迹数据
        label1: 第一条轨迹标签
        label2: 第二条轨迹标签
        use_arc_length_align: 是否使用弧长对齐
        num_samples: 弧长重采样点数
        apply_preprocessing: 是否应用预处理
        preprocess_traj1: 是否对轨迹1预处理
        preprocess_traj2: 是否对轨迹2预处理
        figsize: 图形尺寸
        save_path: 保存路径
    """
    # 根据对齐方式获取对应点和误差
    if use_arc_length_align:
        pos1, pos2, align_info = align_trajectories_by_arc_length(
            df1, df2, num_samples,
            apply_preprocessing=apply_preprocessing,
            preprocess_traj1=preprocess_traj1,
            preprocess_traj2=preprocess_traj2
        )
        align_method = "弧长对齐"
    else:
        # 直接按索引对应
        min_len = min(len(df1), len(df2))
        pos1 = df1[['X_mm', 'Y_mm', 'Z_mm']].values[:min_len]
        pos2 = df2[['X_mm', 'Y_mm', 'Z_mm']].values[:min_len]
        align_method = "索引对齐"
    
    # 计算误差
    errors = np.linalg.norm(pos1 - pos2, axis=1)
    
    # 计算误差统计
    mean_error = np.mean(errors)
    rmse = np.sqrt(np.mean(errors**2))
    max_error = np.max(errors)
    min_error = np.min(errors)
    
    print(f"\n[INFO] 误差空间分布统计 ({align_method}):")
    print(f"       平均误差: {mean_error:.4f} mm")
    print(f"       RMSE:     {rmse:.4f} mm")
    print(f"       最大误差: {max_error:.4f} mm")
    print(f"       最小误差: {min_error:.4f} mm")
    
    # 创建图形（2x2布局）
    fig = plt.figure(figsize=figsize)
    
    # ===== 3D误差分布图（左上，占据2/3高度）=====
    ax_3d = fig.add_subplot(2, 2, (1, 3), projection='3d')
    
    # 使用轨迹1的位置作为坐标，用误差作为颜色
    scatter = ax_3d.scatter(pos1[:, 0], pos1[:, 1], pos1[:, 2],
                           c=errors, cmap='jet', s=30, alpha=0.7,
                           vmin=min_error, vmax=max_error)
    
    # 同时绘制轨迹2的路径作为参考（灰色半透明）
    ax_3d.plot(pos2[:, 0], pos2[:, 1], pos2[:, 2],
              color='gray', linewidth=1.5, alpha=0.3, label=f'{label2}（参考）')
    
    # 起点终点标记
    ax_3d.scatter(pos1[0, 0], pos1[0, 1], pos1[0, 2],
                 c='green', s=200, marker='o', edgecolors='darkgreen',
                 linewidths=3, zorder=5, label='起点')
    ax_3d.scatter(pos1[-1, 0], pos1[-1, 1], pos1[-1, 2],
                 c='red', s=200, marker='s', edgecolors='darkred',
                 linewidths=3, zorder=5, label='终点')
    
    ax_3d.set_xlabel('X (mm)', fontsize=12, fontweight='bold')
    ax_3d.set_ylabel('Y (mm)', fontsize=12, fontweight='bold')
    ax_3d.set_zlabel('Z (mm)', fontsize=12, fontweight='bold')
    ax_3d.set_title(f'误差空间分布图 ({align_method})\n平均={mean_error:.2f}mm, RMSE={rmse:.2f}mm',
                   fontsize=14, fontweight='bold')
    ax_3d.legend(loc='upper left', fontsize=10)
    
    # 添加颜色条
    cbar = plt.colorbar(scatter, ax=ax_3d, shrink=0.6, aspect=15)
    cbar.set_label('误差 (mm)', fontsize=11, fontweight='bold')
    
    # 滚轮缩放
    def on_scroll(event):
        if event.inaxes != ax_3d:
            return
        
        xlim = ax_3d.get_xlim3d()
        ylim = ax_3d.get_ylim3d()
        zlim = ax_3d.get_zlim3d()
        
        x_center = (xlim[0] + xlim[1]) / 2
        y_center = (ylim[0] + ylim[1]) / 2
        z_center = (zlim[0] + zlim[1]) / 2
        
        x_range = (xlim[1] - xlim[0]) / 2
        y_range = (ylim[1] - ylim[0]) / 2
        z_range = (zlim[1] - zlim[0]) / 2
        
        scale_factor = 0.85 if event.button == 'up' else 1.15
        
        new_x_range = x_range * scale_factor
        new_y_range = y_range * scale_factor
        new_z_range = z_range * scale_factor
        
        ax_3d.set_xlim3d([x_center - new_x_range, x_center + new_x_range])
        ax_3d.set_ylim3d([y_center - new_y_range, y_center + new_y_range])
        ax_3d.set_zlim3d([z_center - new_z_range, z_center + new_z_range])
        
        fig.canvas.draw_idle()
    
    fig.canvas.mpl_connect('scroll_event', on_scroll)
    
    # ===== 误差直方图（右上）=====
    ax_hist = fig.add_subplot(2, 2, 2)
    
    n, bins, patches = ax_hist.hist(errors, bins=50, color='steelblue', 
                                     edgecolor='black', alpha=0.7)
    
    # 根据bin值给直方图上色
    cm = plt.cm.jet
    bin_centers = 0.5 * (bins[:-1] + bins[1:])
    col = (bin_centers - bin_centers.min()) / (bin_centers.max() - bin_centers.min())
    for c, p in zip(col, patches):
        plt.setp(p, 'facecolor', cm(c))
    
    ax_hist.axvline(mean_error, color='red', linestyle='--', linewidth=2,
                   label=f'平均: {mean_error:.2f} mm')
    ax_hist.axvline(rmse, color='orange', linestyle='--', linewidth=2,
                   label=f'RMSE: {rmse:.2f} mm')
    
    ax_hist.set_xlabel('误差 (mm)', fontsize=11)
    ax_hist.set_ylabel('频数', fontsize=11)
    ax_hist.set_title('误差分布直方图', fontsize=12, fontweight='bold')
    ax_hist.legend(fontsize=10)
    ax_hist.grid(True, alpha=0.3)
    
    # ===== 累积分布图（右下）=====
    ax_cdf = fig.add_subplot(2, 2, 4)
    
    sorted_errors = np.sort(errors)
    cumulative = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors) * 100
    
    ax_cdf.plot(sorted_errors, cumulative, 'b-', linewidth=2)
    ax_cdf.axhline(50, color='green', linestyle='--', alpha=0.5, label='50%')
    ax_cdf.axhline(95, color='orange', linestyle='--', alpha=0.5, label='95%')
    ax_cdf.axhline(99, color='red', linestyle='--', alpha=0.5, label='99%')
    
    # 标注关键点
    p50 = np.percentile(errors, 50)
    p95 = np.percentile(errors, 95)
    p99 = np.percentile(errors, 99)
    
    ax_cdf.axvline(p50, color='green', linestyle=':', alpha=0.5)
    ax_cdf.axvline(p95, color='orange', linestyle=':', alpha=0.5)
    ax_cdf.axvline(p99, color='red', linestyle=':', alpha=0.5)
    
    ax_cdf.text(p50, 50, f' P50={p50:.2f}mm', fontsize=9, va='bottom')
    ax_cdf.text(p95, 95, f' P95={p95:.2f}mm', fontsize=9, va='bottom')
    ax_cdf.text(p99, 99, f' P99={p99:.2f}mm', fontsize=9, va='top')
    
    ax_cdf.set_xlabel('误差 (mm)', fontsize=11)
    ax_cdf.set_ylabel('累积百分比 (%)', fontsize=11)
    ax_cdf.set_title('误差累积分布函数 (CDF)', fontsize=12, fontweight='bold')
    ax_cdf.legend(fontsize=10)
    ax_cdf.grid(True, alpha=0.3)
    ax_cdf.set_ylim([0, 105])
    
    plt.suptitle(f'误差空间分布分析 - {align_method}', fontsize=16, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[INFO] 误差分布图已保存: {save_path}")
    
    plt.show()
    
    # 返回统计信息
    return {
        'mean': mean_error,
        'rmse': rmse,
        'max': max_error,
        'min': min_error,
        'p50': p50,
        'p95': p95,
        'p99': p99,
        'align_method': align_method,
        'num_points': len(errors)
    }


# ==================== 主程序 ====================
if __name__ == "__main__":
    # ========================================================================
    #                   ⭐⭐⭐ 误差对比方案配置说明 ⭐⭐⭐
    # ========================================================================
    # 
    # 【方案1】使用5000点预处理文件（推荐，误差最准确）
    #   - 文件来源: apply_transform.py 的 transform_for_verification() 生成
    #   - 特点: 已经过与配准完全一致的预处理（弧长重采样+高斯平滑）
    #   - 对齐方式: USE_ARC_LENGTH_ALIGN = False（已对齐，直接按索引比较）
    #   - 预处理: APPLY_PREPROCESSING = False（已预处理，无需再处理）
    #   - 适用场景: 验证配准结果的真实误差
    #   - 优点: 与配准报告的误差完全一致
    #
    # 【方案2】使用原始/变换后的文件（需要实时预处理）
    #   - 文件来源: 原始轨迹文件或 apply_transform.py 的 transform_csv() 生成
    #   - 特点: 点数不同或未预处理，需要实时对齐和平滑
    #   - 对齐方式: USE_ARC_LENGTH_ALIGN = True（弧长对齐）
    #   - 预处理: APPLY_PREPROCESSING = True（高斯平滑）
    #   - 适用场景: 快速对比任意两条轨迹
    #   - 注意: 如果轨迹1已经过变换，设置 PREPROCESS_TRAJ1 = False 避免双重平滑
    #
    # ========================================================================
    #                        如何切换配置？
    # ========================================================================
    #
    # 方法1: 切换 USE_5000_POINT_FILES 标志
    #   USE_5000_POINT_FILES = True   -> 使用方案1（推荐）
    #   USE_5000_POINT_FILES = False  -> 使用方案2
    #
    # 方法2: 手动配置（高级用户）
    #   直接设置以下参数：
    #   - USE_ARC_LENGTH_ALIGN: 是否弧长对齐
    #   - APPLY_PREPROCESSING: 是否高斯平滑
    #   - PREPROCESS_TRAJ1: 是否对轨迹1平滑（已变换数据设False）
    #   - PREPROCESS_TRAJ2: 是否对轨迹2平滑
    #   - NUM_SAMPLES: 弧长重采样点数（默认5000）
    #
    # ========================================================================
    #                        误差计算方式
    # ========================================================================
    #
    # 1. 弧长对齐方式 (USE_ARC_LENGTH_ALIGN = True):
    #    - 按照轨迹的实际路径长度对齐
    #    - 重采样到相同点数（NUM_SAMPLES）
    #    - 可选高斯平滑（APPLY_PREPROCESSING）
    #    - 与 tunable_registration.py 配准流程完全一致
    #    - ✅ 推荐：准确反映轨迹形状差异
    #
    # 2. 索引对齐方式 (USE_ARC_LENGTH_ALIGN = False):
    #    - 直接按点的索引号对应
    #    - 要求两条轨迹点数相同
    #    - 适用于已经过预处理的5000点文件
    #    - ✅ 最快：无需重复预处理
    #
    # ========================================================================
    
    # ===== 配置两个 CSV 文件路径 =====
    
    # ========================================================================
    #                      ⭐ 方案1使用说明 ⭐
    # ========================================================================
    # 【需要提前准备5000点预处理文件】
    # 
    # 步骤1: 运行 apply_transform.py 生成5000点文件
    #   找到 apply_transform.py 文件末尾的测试代码：
    #   ```python
    #   transform_for_verification(
    #       source_csv="你的原始轨迹.csv",
    #       transform_json="saved_transform.json",
    #       output_csv="轨迹_transformed_5000.csv"  # 自动带_5000后缀
    #   )
    #   ```
    #
    # 步骤2: 填写生成的5000点文件路径
    #   csv_path1 = "轨迹1_transformed_5000.csv"  # 变换后的5000点文件
    #   csv_path2 = "轨迹2_preprocessed_5000.csv" # 参考轨迹的5000点文件
    #
    # 文件命名规则：
    #   - 带 _5000 后缀表示已预处理（如：trackerre_transformed_5000.csv）
    #   - 不带后缀表示原始文件
    #
    # ========================================================================
    #                      ⭐ 方案2使用说明 ⭐
    # ========================================================================
    # 【直接填写原始文件即可，脚本自动处理】
    # 
    # 直接填写任意轨迹CSV文件路径：
    #   csv_path1 = "trackerre_transformed.csv"  # 变换后的原始文件
    #   csv_path2 = "tcpp2.csv"                  # 参考轨迹原始文件
    #
    # 脚本会自动：
    #   1. 弧长重采样到5000点
    #   2. 高斯平滑（sigma=1.0）
    #   3. 计算误差
    #
    # ========================================================================
    
    # 方案1：使用5000点预处理文件（推荐，误差最准确）
    USE_5000_POINT_FILES = True  # 切换此标志选择方案1或方案2
    
    if USE_5000_POINT_FILES:
        # ⚠️ 注意：这里必须填写带 _5000 后缀的预处理文件！
        # 如果文件不存在，请先运行 apply_transform.py 的 transform_for_verification() 生成
        csv_path1 = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\trackerrc_transformed.csv"
        csv_path2 = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\tcp6.csv"
        label1 = "变换后轨迹(5000点)"
        label2 = "参考轨迹(5000点)"
        # 5000点文件已经预处理过，不需要再预处理
        APPLY_PREPROCESSING = False
        PREPROCESS_TRAJ1 = False
        PREPROCESS_TRAJ2 = False
        USE_ARC_LENGTH_ALIGN = False  # 5000点文件已对齐，直接按索引比较
        NUM_SAMPLES = 5000
    else:
        # 方案2：使用原始/变换后的文件（脚本自动预处理）
        # ✅ 直接填写原始文件路径即可
        csv_path1 = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\trackerrc_transformed.csv"
        csv_path2 = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\tcp6.csv"
        label1 = "变换后轨迹"
        label2 = "参考轨迹"
        APPLY_PREPROCESSING = True   # 自动高斯平滑
        PREPROCESS_TRAJ1 = False     # 轨迹1已变换，避免双重平滑
        PREPROCESS_TRAJ2 = True      # 轨迹2需要平滑
        USE_ARC_LENGTH_ALIGN = True  # 自动弧长对齐
        NUM_SAMPLES = 5000
    
    # 轨迹颜色
    color1 = 'royalblue'      # 蓝色
    color2 = 'orangered'      # 橙红色
    
    # ===== 加载数据 =====
    print("="*60)
    print("加载第一条轨迹...")
    print("="*60)
    df1 = load_trajectory_csv(csv_path1)
    
    print("\n" + "="*60)
    print("加载第二条轨迹...")
    print("="*60)
    df2 = load_trajectory_csv(csv_path2)
    
    # ===== 1. 绘制双轨迹 3D 对比图 =====
    print("\n" + "="*60)
    print("绘制双轨迹 3D 对比图...")
    if USE_ARC_LENGTH_ALIGN:
        print("⭐ 使用弧长对齐（与配准工具一致）")
    else:
        print("⚠️ 使用索引对齐（可能与配准结果不一致）")
    if APPLY_PREPROCESSING:
        print(f"⭐ 启用预处理：高斯平滑 sigma={PREPROCESS_CONFIG['gaussian_sigma']}")
        print(f"   轨迹1预处理: {'是' if PREPROCESS_TRAJ1 else '否（已变换数据）'}")
        print(f"   轨迹2预处理: {'是' if PREPROCESS_TRAJ2 else '否'}")
    print("="*60)
    
    stats1, stats2, diff_stats = plot_dual_trajectory_3d(
        df1, df2,
        label1=label1,
        label2=label2,
        color1=color1,
        color2=color2,
        show_orientation=True,      # 显示姿态箭头
        orientation_interval=100,   # 每 100 帧显示一个箭头
        arrow_length=None,          # 自动计算
        figsize=(16, 10),
        save_path=None,             # 可设置保存路径
        use_arc_length_align=USE_ARC_LENGTH_ALIGN,  # ⭐ 使用弧长对齐
        num_samples=NUM_SAMPLES,                     # ⭐ 与配准工具一致
        apply_preprocessing=APPLY_PREPROCESSING,     # ⭐ 预处理（与配准流程一致）
        preprocess_traj1=PREPROCESS_TRAJ1,           # ⭐ 轨迹1已变换，不再平滑
        preprocess_traj2=PREPROCESS_TRAJ2            # ⭐ 轨迹2需要平滑
    )
    
    # ===== 2. 绘制位姿分量对比图 =====
    print("\n" + "="*60)
    print("绘制位姿分量对比图...")
    print("="*60)
    
    plot_dual_trajectory_components(
        df1, df2,
        label1=label1,
        label2=label2,
        color1=color1,
        color2=color2,
        figsize=(14, 8),
        save_path=None
    )
    
    # ===== 3. 绘制速度剖面对比图 =====
    print("\n" + "="*60)
    print("绘制速度剖面对比图...")
    print("="*60)
    
    plot_dual_velocity_profile(
        df1, df2,
        label1=label1,
        label2=label2,
        color1=color1,
        color2=color2,
        figsize=(14, 6),
        save_path=None
    )
    
    # ===== 4. 绘制误差空间分布图 =====
    print("\n" + "="*60)
    print("绘制误差空间分布图...")
    print("="*60)
    
    error_spatial_stats = plot_error_spatial_distribution(
        df1, df2,
        label1=label1,
        label2=label2,
        use_arc_length_align=USE_ARC_LENGTH_ALIGN,
        num_samples=NUM_SAMPLES,
        apply_preprocessing=APPLY_PREPROCESSING,
        preprocess_traj1=PREPROCESS_TRAJ1,
        preprocess_traj2=PREPROCESS_TRAJ2,
        figsize=(18, 12),
        save_path=None
    )
    
    print("\n" + "="*60)
    print("[INFO] 双轨迹对比可视化完成!")
    print("="*60)
    align_mode = "弧长对齐" if USE_ARC_LENGTH_ALIGN else "索引对齐"
    print(f"\n轨迹差异摘要 ({align_mode}, {diff_stats['compared_frames']}点):")
    print(f"  平均距离: {diff_stats['mean_distance_mm']:.2f} mm")
    print(f"  RMSE:     {diff_stats['rmse_mm']:.2f} mm")
    print(f"  最大距离: {diff_stats['max_distance_mm']:.2f} mm")
    print(f"  P95:      {diff_stats['p95_distance_mm']:.2f} mm")
    print(f"  标准差:   {diff_stats['std_distance_mm']:.2f} mm")
    
    print(f"\n空间分布统计 ({error_spatial_stats['align_method']}, {error_spatial_stats['num_points']}点):")
    print(f"  P50:      {error_spatial_stats['p50']:.2f} mm")
    print(f"  P95:      {error_spatial_stats['p95']:.2f} mm")
    print(f"  P99:      {error_spatial_stats['p99']:.2f} mm")

# -*- coding: utf-8 -*-
"""
机械臂TCP轨迹回放匹配误差计算工具

核心功能:
1. 平均垂直距离 (Average Perpendicular Distance) - 轨迹复现精度
2. 离散Fréchet距离 (Discrete Fréchet Distance) - 最大轨迹偏差
3. 端点误差 (Endpoint Error) - 起止点定位精度
4. Jerk曲率变化率 (Curvature Change Rate) - 轨迹平滑性

适用场景: 机械臂TCP轨迹回放精度验证
输入格式: CSV文件，位置数据在第4,5,6列（X_mm, Y_mm, Z_mm）

Author: GitHub Copilot
Date: 2026-02-04
"""

import pandas as pd
import numpy as np
import os

# ============ 中文字体配置（必须在导入pyplot之前）============
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'KaiTi', 'FangSong']
matplotlib.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation, Slerp
from scipy.spatial import cKDTree

import warnings
warnings.filterwarnings('ignore')


# ============================================================================
#                           配置参数
# ============================================================================

CONFIG = {
    # 数据预处理
    'remove_duplicate_threshold': 0.01,  # mm，去重阈值
    'unit_auto_detect': True,            # 自动检测单位
    
    # 可视化
    'trajectory_colors': {
        'teach': 'royalblue',
        'replay': 'orangered'
    },
    'marker_size': {
        'start': 200,
        'end': 200,
        'worst': 300
    },
    
    # 评级标准 (mm / 度)
    'excellent_threshold': {'apd': 0.5, 'frechet': 1.5, 'endpoint': 1.0, 'orientation': 1.0},
    'good_threshold': {'apd': 2.0, 'frechet': 5.0, 'endpoint': 2.0, 'orientation': 3.0},
    'acceptable_threshold': {'apd': 5.0, 'frechet': 10.0, 'endpoint': 5.0, 'orientation': 5.0},
    
    # 报告
    'report_output_path': 'trajectory_matching_report.txt',
    'figure_output_dir': 'trajectory_figures',
    'figure_save_dpi': 150,

    # ⭐ 可选误差指标（True=计算, False=跳过）
    # 五项都为 False 时：跳过所有误差计算，仅执行可视化
    'metrics': {
        'compute_apd':         True,  # 1. 平均垂直距离 (APD)   - 较慢 O(N×M)
        'compute_frechet':     True,  # 2. 离散Fréchet距离      - 最慢 O(N×M)²
        'compute_endpoint':    True,  # 3. 端点误差             - 瞬时
        'compute_jerk':        False,   # 4. Jerk曲率变化率       - 快速 O(N)
        'compute_orientation': False,   # 5. 姿态角度误差         - 快速 O(N)
    },
}


# ============================================================================
#                        数据加载与预处理
# ============================================================================

def load_tcp_trajectory(csv_path):
    """
    加载TCP轨迹CSV文件
    
    CSV格式:
    FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,...
    
    位置数据从第4,5,6列（索引3,4,5）读取
    
    Args:
        csv_path (str): CSV文件路径
        
    Returns:
        np.ndarray: Nx3位置数组（毫米单位）
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"文件不存在: {csv_path}")
    
    print(f"\n[INFO] 正在加载: {os.path.basename(csv_path)}")
    
    df = pd.read_csv(csv_path)
    
    # 提取位置数据（第4,5,6列，Python索引为3,4,5）
    positions = df.iloc[:, 3:6].values  # 形状: (N, 3)
    
    # 自动检测单位
    if CONFIG['unit_auto_detect']:
        x_range = positions[:, 0].max() - positions[:, 0].min()
        
        if x_range < 10:  # 数值小于10，可能是米单位
            print(f"[WARN] 检测到米单位，自动转换为毫米")
            positions = positions * 1000
    
    # 去除重复点（距离<阈值的连续点）
    threshold = CONFIG['remove_duplicate_threshold']
    clean_positions = [positions[0]]
    removed_count = 0
    
    for i in range(1, len(positions)):
        if np.linalg.norm(positions[i] - positions[i-1]) > threshold:
            clean_positions.append(positions[i])
        else:
            removed_count += 1
    
    positions = np.array(clean_positions)
    
    if removed_count > 0:
        print(f"[INFO] 去除 {removed_count} 个重复点")
    
    print(f"[INFO] 有效点数: {len(positions)}")
    print(f"[INFO] X范围: {positions[:,0].min():.2f} ~ {positions[:,0].max():.2f} mm")
    print(f"[INFO] Y范围: {positions[:,1].min():.2f} ~ {positions[:,1].max():.2f} mm")
    print(f"[INFO] Z范围: {positions[:,2].min():.2f} ~ {positions[:,2].max():.2f} mm")
    
    return positions


def load_tcp_trajectory_with_orientation(csv_path):
    """
    加载TCP位置轨迹与四元数姿态数据（带同步去重）

    CSV格式:
    FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,...

    Returns:
        tuple: (positions_Nx3 mm, quaternions_Nx4 [QX,QY,QZ,QW] 已归一化)
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"文件不存在: {csv_path}")

    df = pd.read_csv(csv_path)

    if df.shape[1] < 10:
        raise ValueError(
            f"CSV列数不足，至少需要10列（含QX,QY,QZ,QW），当前: {df.shape[1]}列\n"
            f"  文件: {csv_path}"
        )

    positions   = df.iloc[:, 3:6].values.astype(float)
    quaternions = df.iloc[:, 6:10].values.astype(float)

    if CONFIG['unit_auto_detect']:
        x_range = positions[:, 0].max() - positions[:, 0].min()
        if x_range < 10:
            positions = positions * 1000

    # 去重（与 load_tcp_trajectory 保持一致，同步剔除姿态帧）
    threshold = CONFIG['remove_duplicate_threshold']
    clean_pos  = [positions[0]]
    clean_quat = [quaternions[0]]
    for i in range(1, len(positions)):
        if np.linalg.norm(positions[i] - positions[i-1]) > threshold:
            clean_pos.append(positions[i])
            clean_quat.append(quaternions[i])

    positions   = np.array(clean_pos)
    quaternions = np.array(clean_quat)

    # 四元数归一化（消除浮点误差）
    norms = np.linalg.norm(quaternions, axis=1, keepdims=True)
    norms = np.where(norms < 1e-9, 1.0, norms)
    quaternions = quaternions / norms

    return positions, quaternions


# ============================================================================
#                        几何计算函数
# ============================================================================

def point_to_segment_distance(point, seg_start, seg_end):
    """
    计算点到线段的最短距离（垂直距离）
    
    几何原理:
           Point P
             *
            /|
           / | perpendicular
          /  | distance
         /   |
        /    ↓
    A *------*------* B
       ←----------→
       segment_vec
    
    Args:
        point (np.ndarray): (3,) 待测点
        seg_start (np.ndarray): (3,) 线段起点
        seg_end (np.ndarray): (3,) 线段终点
    
    Returns:
        float: 最短距离
    """
    # 线段向量
    segment_vec = seg_end - seg_start
    segment_length = np.linalg.norm(segment_vec)
    
    # 退化为点的情况
    if segment_length < 1e-6:
        return np.linalg.norm(point - seg_start)
    
    # 单位方向向量
    segment_dir = segment_vec / segment_length
    
    # 点到线段起点的向量
    point_vec = point - seg_start
    
    # 投影长度（在线段方向上）
    projection = np.dot(point_vec, segment_dir)
    
    # 判断最近点在线段的哪个位置
    if projection < 0:
        # 最近点是线段起点
        return np.linalg.norm(point - seg_start)
    elif projection > segment_length:
        # 最近点是线段终点
        return np.linalg.norm(point - seg_end)
    else:
        # 最近点在线段中间（垂足）
        closest_point = seg_start + projection * segment_dir
        return np.linalg.norm(point - closest_point)


# ============================================================================
#                     核心指标1: 平均垂直距离 (APD)
# ============================================================================

def compute_average_perpendicular_distance(replay_traj, teach_traj):
    """
    计算回放轨迹到示教轨迹的平均垂直距离
    
    对应工业概念: 轨迹复现精度 (Path Repeatability)
    
    Args:
        replay_traj (np.ndarray): (N, 3) 回放轨迹点云
        teach_traj (np.ndarray): (M, 3) 示教轨迹点云
    
    Returns:
        dict: {
            'mean': 平均距离,
            'std': 标准差,
            'max': 最大距离,
            'min': 最小距离,
            'p95': 95百分位数,
            'median': 中位数,
            'distances': 每个点的距离数组
        }
    """
    print(f"\n[计算] 平均垂直距离 (APD)...")
    
    n_replay = len(replay_traj)
    n_teach = len(teach_traj)
    NEIGHBOR_SEGS = 10  # 每侧检查的相邻线段数

    # 用 cKDTree 快速定位最近示教点，再检查其前后各 NEIGHBOR_SEGS 条线段
    tree = cKDTree(teach_traj)
    _, nearest_idx = tree.query(replay_traj)  # O(N log M)

    distances = np.empty(n_replay)
    for i, point in enumerate(replay_traj):
        j = nearest_idx[i]
        j_start = max(0, j - NEIGHBOR_SEGS)
        j_end   = min(n_teach - 1, j + NEIGHBOR_SEGS)
        min_distance = float('inf')
        for jj in range(j_start, j_end):
            d = point_to_segment_distance(point, teach_traj[jj], teach_traj[jj + 1])
            if d < min_distance:
                min_distance = d
        distances[i] = min_distance
    
    result = {
        'mean': np.mean(distances),
        'std': np.std(distances),
        'max': np.max(distances),
        'min': np.min(distances),
        'p95': np.percentile(distances, 95),
        'median': np.median(distances),
        'distances': distances
    }
    
    print(f"  完成! 平均距离: {result['mean']:.4f} mm")
    
    return result


# ============================================================================
#                  核心指标2: 离散Fréchet距离
# ============================================================================

def compute_discrete_frechet_distance(replay_traj, teach_traj):
    """
    计算离散Fréchet距离（动态规划）
    
    对应工业概念: 最大轨迹偏差 / 安全包络 (Worst-case Deviation)
    
    直观理解（狗绳比喻）:
    - 一个人沿着示教轨迹遛狗，狗沿着回放轨迹走
    - 狗绳可以伸缩但不能交叉
    - Fréchet距离 = 需要的最短狗绳长度
    
    Args:
        replay_traj (np.ndarray): (N, 3) 回放轨迹
        teach_traj (np.ndarray): (M, 3) 示教轨迹
    
    Returns:
        dict: {
            'frechet_distance': Fréchet距离值,
            'worst_replay_idx': 最大偏差的回放点索引,
            'worst_teach_idx': 对应的示教点索引,
            'distance_matrix': 距离矩阵（用于热力图）
        }
    """
    print(f"\n[计算] 离散Fréchet距离...")
    
    n = len(replay_traj)
    m = len(teach_traj)
    NEIGHBOR_BAND = 10  # cKDTree 加速：每侧查询邻近点数

    print(f"  构建距离矩阵（cKDTree带状加速，band={NEIGHBOR_BAND}）...")

    # 步骤1: 用 cKDTree 向量化计算距离矩阵（广播替代双层循环）
    # 为保持 Fréchet DP 正确性，仍构建完整 N×M 矩阵，但用向量化代替 Python 双循环
    # 分块计算防止大矩阵 OOM（每次处理 CHUNK 行）
    CHUNK = 200
    dist_matrix = np.empty((n, m), dtype=np.float32)
    for i0 in range(0, n, CHUNK):
        i1 = min(i0 + CHUNK, n)
        # 广播：(chunk,3) vs (m,3) → (chunk,m)
        diff = replay_traj[i0:i1, np.newaxis, :] - teach_traj[np.newaxis, :, :]
        dist_matrix[i0:i1] = np.sqrt((diff * diff).sum(axis=2))

    print(f"  动态规划求解最优路径...")

    # 步骤2: 动态规划（向量化行迭代）
    dp = np.full((n, m), np.inf, dtype=np.float32)
    dp[0, 0] = dist_matrix[0, 0]

    for i in range(1, n):
        dp[i, 0] = max(dp[i-1, 0], dist_matrix[i, 0])
    for j in range(1, m):
        dp[0, j] = max(dp[0, j-1], dist_matrix[0, j])

    for i in range(1, n):
        prev_row = dp[i-1]                          # shape (m,)
        # min(dp[i-1,j], dp[i-1,j-1], dp[i,j-1]) 需逐列递推（有依赖）
        for j in range(1, m):
            min_prev = min(prev_row[j], prev_row[j-1], dp[i, j-1])
            dp[i, j] = max(dist_matrix[i, j], min_prev)

    frechet_distance = float(dp[n-1, m-1])

    # 找到最大偏差点：在 dist_matrix 中找 <= frechet_distance 的最大值点
    mask = dist_matrix <= frechet_distance + 1e-6
    masked = np.where(mask, dist_matrix, -1.0)
    flat_idx = int(np.argmax(masked))
    worst_i, worst_j = divmod(flat_idx, m)
    
    result = {
        'frechet_distance': frechet_distance,
        'worst_replay_idx': worst_i,
        'worst_teach_idx': worst_j,
        'distance_matrix': dist_matrix
    }
    
    print(f"  完成! Fréchet距离: {frechet_distance:.4f} mm")
    print(f"  最大偏差点: 回放#{worst_i} <-> 示教#{worst_j}")
    
    return result


# ============================================================================
#                     核心指标3: 端点误差
# ============================================================================

def compute_endpoint_errors(replay_traj, teach_traj):
    """
    计算起点和终点的定位误差
    
    对应工业概念: 起止点定位精度
    
    Args:
        replay_traj (np.ndarray): (N, 3) 回放轨迹
        teach_traj (np.ndarray): (M, 3) 示教轨迹
    
    Returns:
        dict: {
            'start_error': 起点误差 (mm),
            'end_error': 终点误差 (mm),
            'start_vector': 起点误差向量 (3,),
            'end_vector': 终点误差向量 (3,)
        }
    """
    print(f"\n[计算] 端点误差...")
    
    start_error = np.linalg.norm(replay_traj[0] - teach_traj[0])
    end_error = np.linalg.norm(replay_traj[-1] - teach_traj[-1])
    
    start_vector = replay_traj[0] - teach_traj[0]
    end_vector = replay_traj[-1] - teach_traj[-1]
    
    print(f"  起点误差: {start_error:.4f} mm")
    print(f"  终点误差: {end_error:.4f} mm")
    
    return {
        'start_error': start_error,
        'end_error': end_error,
        'start_vector': start_vector,
        'end_vector': end_vector
    }


# ============================================================================
#                核心指标4: Jerk（曲率变化率，基于弧长）
# ============================================================================

def _build_curvature_jerk_profile(traj):
    """
    基于弧长参数构建曲率与Jerk（曲率变化率）曲线。

    说明:
    - 曲率: kappa = ||r'(s) × r''(s)|| / ||r'(s)||^3
    - Jerk(本脚本定义): |d(kappa)/ds|

    Args:
        traj (np.ndarray): (N, 3) 轨迹点

    Returns:
        dict: {
            's': 弧长坐标,
            's_norm': 归一化弧长[0,1],
            'curvature': 曲率序列,
            'jerk': 曲率变化率序列
        }
    """
    n = len(traj)
    if n < 5:
        raise ValueError(f"轨迹点数过少({n})，无法稳定计算曲率变化率，至少需要5个点")

    seg_lengths = np.linalg.norm(np.diff(traj, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg_lengths)))
    total_length = s[-1]

    if total_length < 1e-9:
        curvature = np.zeros(n)
        jerk = np.zeros(n)
        s_norm = np.linspace(0, 1, n)
        return {
            's': s,
            's_norm': s_norm,
            'curvature': curvature,
            'jerk': jerk,
        }

    # ── 修复1: 重采样到均匀弧长间距 ──────────────────────────────────────────
    # TCP轨迹是等时间间隔采样，机器人变速运动导致弧长间距极不均匀。
    # 直接将非均匀 s 数组传给 np.gradient 会在采样密集/稀疏交界处产生
    # 数值噪声尖峰，两次梯度叠加后噪声进一步放大至数千倍。
    # 重采样后用标量 ds 调用 np.gradient，精度可靠。
    n_uniform = max(n, 500)
    s_uniform = np.linspace(0.0, total_length, n_uniform)
    ds = s_uniform[1] - s_uniform[0]

    x_u = np.interp(s_uniform, s, traj[:, 0])
    y_u = np.interp(s_uniform, s, traj[:, 1])
    z_u = np.interp(s_uniform, s, traj[:, 2])

    dx_ds   = np.gradient(x_u, ds)
    dy_ds   = np.gradient(y_u, ds)
    dz_ds   = np.gradient(z_u, ds)

    d2x_ds2 = np.gradient(dx_ds, ds)
    d2y_ds2 = np.gradient(dy_ds, ds)
    d2z_ds2 = np.gradient(dz_ds, ds)

    r1 = np.column_stack([dx_ds, dy_ds, dz_ds])
    r2 = np.column_stack([d2x_ds2, d2y_ds2, d2z_ds2])

    speed = np.linalg.norm(r1, axis=1)
    cross_term = np.cross(r1, r2)

    curvature = np.zeros(n_uniform)
    valid = speed > 1e-9
    curvature[valid] = np.linalg.norm(cross_term[valid], axis=1) / (speed[valid] ** 3)

    # ── 修复2: 用百分位数 cap 代替 posinf→0 ────────────────────────────────
    # 原代码将无穷大曲率（急转弯处）强制设为 0，相邻点曲率差 dκ/ds 由此产生
    # 数千量级的虚假 Jerk 尖峰，使 mean 变大但 P50 近零，图上几乎看不见曲线。
    finite_mask = np.isfinite(curvature)
    if finite_mask.any():
        cap = np.percentile(curvature[finite_mask], 99.9) * 10.0
        curvature = np.clip(curvature, 0.0, cap)
    curvature = np.nan_to_num(curvature, nan=0.0, posinf=0.0, neginf=0.0)

    # ── 修复3: Savitzky-Golay 平滑曲率后再求导 ──────────────────────────────
    # 轨迹采样噪声经两次数值微分会被放大；先平滑曲率再求 Jerk，
    # 保留真实的曲率变化趋势，同时抑制高频噪声。
    win = min(51, max(5, (n_uniform // 20) * 2 + 1))  # 奇数，合理窗宽
    if win % 2 == 0:
        win += 1
    curvature = savgol_filter(curvature, window_length=win, polyorder=3)
    curvature = np.maximum(curvature, 0.0)

    jerk = np.abs(np.gradient(curvature, ds))
    jerk = np.nan_to_num(jerk, nan=0.0, posinf=0.0, neginf=0.0)

    s_norm = s_uniform / total_length
    return {
        's': s_uniform,
        's_norm': s_norm,
        'curvature': curvature,
        'jerk': jerk,
    }


def _series_stats(values):
    """返回一维序列的基础统计量。"""
    return {
        'mean': float(np.mean(values)),
        'std': float(np.std(values)),
        'max': float(np.max(values)),
        'min': float(np.min(values)),
        'median': float(np.median(values)),
        'p95': float(np.percentile(values, 95)),
    }


def compute_jerk_curvature_change_rate(replay_traj, teach_traj):
    """
    计算Jerk指标（曲率变化率），并比较示教与回放曲线差异。

    Args:
        replay_traj (np.ndarray): (N, 3) 回放轨迹
        teach_traj (np.ndarray): (M, 3) 示教轨迹

    Returns:
        dict: {
            'replay': {...},
            'teach': {...},
            'comparison': {
                'mean_abs_diff': 平均绝对差,
                'rmse': 均方根差,
                'max_abs_diff': 最大绝对差
            },
            'common_s_norm': 对齐用归一化弧长,
            'replay_jerk_interp': 回放插值曲线,
            'teach_jerk_interp': 示教插值曲线
        }
    """
    print(f"\n[计算] Jerk曲率变化率...")

    replay_profile = _build_curvature_jerk_profile(replay_traj)
    teach_profile = _build_curvature_jerk_profile(teach_traj)

    sample_count = max(200, min(800, max(len(replay_traj), len(teach_traj))))
    common_s_norm = np.linspace(0.0, 1.0, sample_count)

    replay_jerk_interp = np.interp(common_s_norm, replay_profile['s_norm'], replay_profile['jerk'])
    teach_jerk_interp = np.interp(common_s_norm, teach_profile['s_norm'], teach_profile['jerk'])

    jerk_diff = replay_jerk_interp - teach_jerk_interp

    result = {
        'replay': {
            **_series_stats(replay_profile['jerk']),
            's_norm': replay_profile['s_norm'],
            'curvature': replay_profile['curvature'],
            'jerk': replay_profile['jerk'],
        },
        'teach': {
            **_series_stats(teach_profile['jerk']),
            's_norm': teach_profile['s_norm'],
            'curvature': teach_profile['curvature'],
            'jerk': teach_profile['jerk'],
        },
        'comparison': {
            'mean_abs_diff': float(np.mean(np.abs(jerk_diff))),
            'rmse': float(np.sqrt(np.mean(jerk_diff ** 2))),
            'max_abs_diff': float(np.max(np.abs(jerk_diff))),
        },
        'common_s_norm': common_s_norm,
        'replay_jerk_interp': replay_jerk_interp,
        'teach_jerk_interp': teach_jerk_interp,
    }

    print(f"  回放Jerk均值: {result['replay']['mean']:.6f} 1/mm^2")
    print(f"  示教Jerk均值: {result['teach']['mean']:.6f} 1/mm^2")
    print(f"  Jerk曲线RMSE: {result['comparison']['rmse']:.6f} 1/mm^2")

    return result


# ============================================================================
#               核心指标5: 姿态误差（基于弧长参数化+SLERP）
# ============================================================================

def compute_orientation_error(replay_traj, replay_quats, teach_traj, teach_quats):
    """
    计算回放轨迹与示教轨迹的姿态角度误差。

    方法:
    1. 由位置轨迹计算各自归一化弧长序列
    2. 在公共弧长采样点上对两段四元数序列分别做SLERP插值
    3. 计算对应点的旋转差角（geodesic distance, 单位: 度）

    Args:
        replay_traj  (np.ndarray): (N, 3) 回放位置轨迹（用于弧长计算）
        replay_quats (np.ndarray): (N, 4) 回放四元数 [QX, QY, QZ, QW]
        teach_traj   (np.ndarray): (M, 3) 示教位置轨迹
        teach_quats  (np.ndarray): (M, 4) 示教四元数 [QX, QY, QZ, QW]

    Returns:
        dict: {
            'mean_deg':   平均角度误差 (°),
            'std_deg':    标准差 (°),
            'max_deg':    最大角度误差 (°),
            'min_deg':    最小角度误差 (°),
            'p95_deg':    95百分位误差 (°),
            'median_deg': 中位数 (°),
            'errors_deg': 各采样点角度误差数组 (°),
            's_norm':     对应归一化弧长
        }
    """
    print(f"\n[计算] 姿态误差（角度）...")

    def _arc_norm_strict(traj):
        """计算归一化弧长，保证严格单调（Slerp 要求）。"""
        seg_lengths = np.linalg.norm(np.diff(traj, axis=0), axis=1)
        s = np.concatenate(([0.0], np.cumsum(seg_lengths)))
        total = s[-1]
        if total < 1e-9:
            return np.linspace(0.0, 1.0, len(traj))
        s_norm = s / total
        # 强制严格递增，消除静止段引起的重复值
        for i in range(1, len(s_norm)):
            if s_norm[i] <= s_norm[i - 1]:
                s_norm[i] = s_norm[i - 1] + 1e-10
        return np.clip(s_norm, 0.0, 1.0)

    s_replay = _arc_norm_strict(replay_traj)
    s_teach  = _arc_norm_strict(teach_traj)

    sample_count = max(200, min(500, min(len(replay_traj), len(teach_traj))))
    s_common = np.linspace(0.0, 1.0, sample_count)

    # scipy Rotation 使用 [x, y, z, w] 格式，与 [QX, QY, QZ, QW] 一致
    r_replay = Rotation.from_quat(replay_quats)
    r_teach  = Rotation.from_quat(teach_quats)

    slerp_replay = Slerp(s_replay, r_replay)
    slerp_teach  = Slerp(s_teach,  r_teach)

    r_replay_sampled = slerp_replay(s_common)
    r_teach_sampled  = slerp_teach(s_common)

    # 旋转差角 = geodesic distance（始终取最短弧，[0°, 180°]）
    r_diff     = r_teach_sampled * r_replay_sampled.inv()
    angles_deg = np.degrees(r_diff.magnitude())

    result = {
        'mean_deg':   float(np.mean(angles_deg)),
        'std_deg':    float(np.std(angles_deg)),
        'max_deg':    float(np.max(angles_deg)),
        'min_deg':    float(np.min(angles_deg)),
        'p95_deg':    float(np.percentile(angles_deg, 95)),
        'median_deg': float(np.median(angles_deg)),
        'errors_deg': angles_deg,
        's_norm':     s_common,
    }

    print(f"  平均角度误差: {result['mean_deg']:.4f}°")
    print(f"  最大角度误差: {result['max_deg']:.4f}°")
    print(f"  P95角度误差:  {result['p95_deg']:.4f}°")

    return result


# ============================================================================
#                         综合评级
# ============================================================================

def determine_overall_grade(apd, frechet, endpoints, orientation=None):
    """
    根据三大位置指标 + 可选姿态指标综合判定等级

    评级规则:
        优秀: APD<0.5mm AND Fréchet<1.5mm AND 端点<1.0mm AND 姿态<1°
        良好: APD<2.0mm AND Fréchet<5.0mm AND 端点<2.0mm AND 姿态<3°
        可接受: APD<5.0mm AND Fréchet<10.0mm AND 端点<5.0mm AND 姿态<5°
        需优化: 其他情况

    Args:
        apd (dict|None): APD计算结果
        frechet (dict|None): Fréchet距离结果
        endpoints (dict|None): 端点误差结果
        orientation (dict|None): 姿态误差结果（可选）

    Returns:
        tuple: (等级字符串, 是否通过)
    """
    apd_mean     = apd['mean']                                           if apd         is not None else None
    frechet_dist = frechet['frechet_distance']                           if frechet     is not None else None
    max_endpoint = max(endpoints['start_error'], endpoints['end_error']) if endpoints   is not None else None
    ori_err      = orientation['mean_deg']                               if orientation is not None else None

    # 全部未计算 → 无法评级
    if apd_mean is None and frechet_dist is None and max_endpoint is None and ori_err is None:
        return "仅可视化 (Visualization Only)", None

    excellent  = CONFIG['excellent_threshold']
    good       = CONFIG['good_threshold']
    acceptable = CONFIG['acceptable_threshold']

    # 未计算的指标视为满足条件（不参与评级惩罚）
    def _ok(val, threshold):
        return val is None or val < threshold

    if (_ok(apd_mean, excellent['apd']) and _ok(frechet_dist, excellent['frechet'])
            and _ok(max_endpoint, excellent['endpoint']) and _ok(ori_err, excellent['orientation'])):
        return "优秀 (Excellent)", True
    elif (_ok(apd_mean, good['apd']) and _ok(frechet_dist, good['frechet'])
            and _ok(max_endpoint, good['endpoint']) and _ok(ori_err, good['orientation'])):
        return "良好 (Good)", True
    elif (_ok(apd_mean, acceptable['apd']) and _ok(frechet_dist, acceptable['frechet'])
            and _ok(max_endpoint, acceptable['endpoint']) and _ok(ori_err, acceptable['orientation'])):
        return "可接受 (Acceptable)", True
    else:
        return "需优化 (Needs Improvement)", False


# ============================================================================
#                        可视化函数
# ============================================================================

def plot_trajectory_matching_analysis(replay_traj, teach_traj, 
                                      apd_result, frechet_result, endpoint_result, jerk_result,
                                      overall_grade, save_path=None,
                                      orientation_result=None):
    """
    生成轨迹匹配误差综合分析图表
    
    包含6个子图:
    1. 3D轨迹对比图（标注最大偏差点）- 左上，占据两列
    2. APD误差随点索引折线图 - 中左
    3. APD垂直距离分布直方图 - 中右
    4. Jerk曲率变化率折线图 - 下左
    5. 姿态角度误差折线图 - 下中
    6. 统计信息面板 - 右侧，占据三行
    
    Args:
        replay_traj: 回放轨迹
        teach_traj: 示教轨迹
        apd_result: APD计算结果
        frechet_result: Fréchet距离结果
        endpoint_result: 端点误差结果
        jerk_result: Jerk计算结果
        overall_grade: 综合评级
        save_path: 保存路径（可选）
        orientation_result: 姿态误差结果（可选）
    """
    fig = plt.figure(figsize=(18, 14))
    
    # 使用GridSpec创建布局：3行3列，右侧统计面板占据整列
    import matplotlib.gridspec as gridspec
    gs = gridspec.GridSpec(3, 3, figure=fig, width_ratios=[1.2, 1.2, 1], height_ratios=[1.0, 1.0, 0.9],
                           hspace=0.35, wspace=0.35)
    
    # ===== 图1: 3D轨迹对比（上方，占据左侧两列）=====
    ax1 = fig.add_subplot(gs[0, :2], projection='3d')  # 占据第一行的前两列
    
    colors = CONFIG['trajectory_colors']
    sizes = CONFIG['marker_size']
    
    # 绘制轨迹
    ax1.plot(teach_traj[:, 0], teach_traj[:, 1], teach_traj[:, 2], 
            color=colors['teach'], linewidth=2.5, label='示教轨迹', alpha=0.8)
    ax1.plot(replay_traj[:, 0], replay_traj[:, 1], replay_traj[:, 2], 
            color=colors['replay'], linewidth=2.5, label='回放轨迹', alpha=0.8)
    
    # 起点终点
    ax1.scatter(*teach_traj[0], c='green', s=sizes['start'], marker='o', 
               edgecolors='darkgreen', linewidths=3, label='起点', zorder=5)
    ax1.scatter(*teach_traj[-1], c='red', s=sizes['end'], marker='s', 
               edgecolors='darkred', linewidths=3, label='终点', zorder=5)
    
    # 标注最大偏差点（仅 Fréchet 已计算时显示）
    worst_i, worst_j, error_vec = 0, 0, None
    if frechet_result is not None:
        worst_i = frechet_result['worst_replay_idx']
        worst_j = frechet_result['worst_teach_idx']
        ax1.scatter(*replay_traj[worst_i], c='purple', s=sizes['worst'], marker='*', 
                   edgecolors='black', linewidths=2, label='最大偏差点', zorder=6)
        error_vec = teach_traj[worst_j] - replay_traj[worst_i]
        ax1.quiver(replay_traj[worst_i][0], replay_traj[worst_i][1], replay_traj[worst_i][2],
                  error_vec[0], error_vec[1], error_vec[2],
                  color='purple', arrow_length_ratio=0.2, linewidth=2, zorder=4)
    
    ax1.set_xlabel('X (mm)', fontsize=11, fontweight='bold')
    ax1.set_ylabel('Y (mm)', fontsize=11, fontweight='bold')
    ax1.set_zlabel('Z (mm)', fontsize=11, fontweight='bold')
    ax1.set_title('TCP轨迹回放对比 - 3D视图', fontsize=13, fontweight='bold', pad=15)
    ax1.legend(fontsize=10, loc='upper left')
    ax1.grid(True, alpha=0.3)
    
    # 设置等比例坐标轴，避免拉伸变形
    all_points = np.vstack([teach_traj, replay_traj])
    max_range = np.array([all_points[:, 0].max() - all_points[:, 0].min(),
                          all_points[:, 1].max() - all_points[:, 1].min(),
                          all_points[:, 2].max() - all_points[:, 2].min()]).max() / 2.0
    
    mid_x = (all_points[:, 0].max() + all_points[:, 0].min()) * 0.5
    mid_y = (all_points[:, 1].max() + all_points[:, 1].min()) * 0.5
    mid_z = (all_points[:, 2].max() + all_points[:, 2].min()) * 0.5
    
    ax1.set_xlim(mid_x - max_range, mid_x + max_range)
    ax1.set_ylim(mid_y - max_range, mid_y + max_range)
    ax1.set_zlim(mid_z - max_range, mid_z + max_range)
    
    # ===== 图2: 误差随点索引折线图（中左）=====
    ax2 = fig.add_subplot(gs[1, 0])
    distances = None
    max_idx = None

    if apd_result is not None:
        distances = apd_result['distances']
        point_indices = np.arange(len(distances))
        ax2.plot(point_indices, distances, color='steelblue', linewidth=1.5, alpha=0.8, label='点对点误差')
        ax2.axhline(apd_result['mean'], color='red', linestyle='--',
                    linewidth=2, label=f"平均: {apd_result['mean']:.3f} mm", alpha=0.8)
        ax2.axhline(apd_result['p95'], color='orange', linestyle='--',
                    linewidth=2, label=f"P95: {apd_result['p95']:.3f} mm", alpha=0.8)
        ax2.axhline(apd_result['max'], color='darkred', linestyle=':',
                    linewidth=2, label=f"最大: {apd_result['max']:.3f} mm", alpha=0.8)
        max_idx = np.argmax(distances)
        ax2.scatter(max_idx, distances[max_idx], color='darkred', s=100,
                    marker='o', edgecolors='black', linewidths=2, zorder=5)
        ax2.annotate(f'最大误差\n#{max_idx}\n{distances[max_idx]:.3f}mm',
                    xy=(max_idx, distances[max_idx]),
                    xytext=(max_idx + len(distances)*0.1, distances[max_idx]),
                    fontsize=9, color='darkred', fontweight='bold',
                    arrowprops=dict(arrowstyle='->', color='darkred', lw=1.5),
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.7))
        ax2.set_xlabel('回放轨迹点索引', fontsize=10, fontweight='bold')
        ax2.set_ylabel('垂直距离 (mm)', fontsize=10, fontweight='bold')
        ax2.legend(fontsize=8.5, loc='upper right')
        ax2.grid(True, alpha=0.3, linestyle='--')
        ax2.set_xlim(0, len(distances))
        ax2.set_ylim(0, max(distances.max() * 1.15, apd_result['mean'] * 2))
    else:
        ax2.text(0.5, 0.5, '平均垂直距离\n(APD 未计算)',
                 transform=ax2.transAxes, ha='center', va='center',
                 fontsize=13, color='gray', style='italic')
    ax2.set_title('误差沿轨迹变化趋势', fontsize=11, fontweight='bold', pad=10)

    # ===== 图3: 垂直距离分布直方图（中右）=====
    ax3 = fig.add_subplot(gs[1, 1])

    if apd_result is not None:
        n, bins, patches = ax3.hist(distances, bins=50, color='steelblue',
                                    edgecolor='black', alpha=0.7)
        cm = plt.cm.RdYlGn_r
        bin_centers = 0.5 * (bins[:-1] + bins[1:])
        col = (bin_centers - bin_centers.min()) / (bin_centers.max() - bin_centers.min())
        for c, p in zip(col, patches):
            plt.setp(p, 'facecolor', cm(c))
        ax3.axvline(apd_result['mean'], color='red', linestyle='--',
                    linewidth=2.5, label=f"平均: {apd_result['mean']:.3f} mm")
        ax3.axvline(apd_result['p95'], color='orange', linestyle='--',
                    linewidth=2.5, label=f"P95: {apd_result['p95']:.3f} mm")
        ax3.axvline(apd_result['max'], color='darkred', linestyle=':',
                    linewidth=2, label=f"最大: {apd_result['max']:.3f} mm")
        ax3.set_xlabel('垂直距离 (mm)', fontsize=10, fontweight='bold')
        ax3.set_ylabel('频数', fontsize=10, fontweight='bold')
        ax3.legend(fontsize=8.5, loc='upper right')
        ax3.grid(True, alpha=0.3, axis='y')
    else:
        ax3.text(0.5, 0.5, '平均垂直距离\n(APD 未计算)',
                 transform=ax3.transAxes, ha='center', va='center',
                 fontsize=13, color='gray', style='italic')
    ax3.set_title('垂直距离分布', fontsize=11, fontweight='bold', pad=10)

    # ===== 图4: Jerk曲率变化率折线图（下左）=====
    ax5 = fig.add_subplot(gs[2, 0])
    if jerk_result is not None:
        s_replay = jerk_result['replay']['s_norm']
        s_teach = jerk_result['teach']['s_norm']
        jerk_replay = jerk_result['replay']['jerk']
        jerk_teach = jerk_result['teach']['jerk']

        ax5.plot(s_teach, jerk_teach, color=colors['teach'], linewidth=1.8,
                 alpha=0.85, label='示教Jerk曲线')
        ax5.plot(s_replay, jerk_replay, color=colors['replay'], linewidth=1.8,
                 alpha=0.85, label='回放Jerk曲线')

        ax5.axhline(jerk_result['teach']['mean'], color=colors['teach'], linestyle='--', linewidth=1.5,
                    alpha=0.7, label=f"示教均值: {jerk_result['teach']['mean']:.4f}")
        ax5.axhline(jerk_result['replay']['mean'], color=colors['replay'], linestyle='--', linewidth=1.5,
                    alpha=0.7, label=f"回放均值: {jerk_result['replay']['mean']:.4f}")

        ax5.set_xlabel('归一化弧长', fontsize=10, fontweight='bold')
        ax5.set_ylabel('Jerk = |dκ/ds| (1/mm²)', fontsize=10, fontweight='bold')
        ax5.legend(fontsize=8.5, loc='upper right', ncol=2)
        ax5.grid(True, alpha=0.3, linestyle='--')
        ax5.set_xlim(0, 1)
        # 截断 Y 轴至 P99，避免极少数尖峰把有效曲线压扁至视觉零附近
        _jv = np.concatenate([jerk_teach, jerk_replay])
        _ymax = np.percentile(_jv[_jv > 0], 99) * 1.5 if (_jv > 0).any() else 1e-6
        ax5.set_ylim(0, max(_ymax, 1e-9))
    else:
        ax5.text(0.5, 0.5, 'Jerk曲率变化率\n(未计算)',
                 transform=ax5.transAxes, ha='center', va='center',
                 fontsize=13, color='gray', style='italic')
    ax5.set_title('Jerk曲率变化率折线图（轨迹平滑性）', fontsize=11, fontweight='bold', pad=10)

    # ===== 图6: 姿态角度误差折线图（下中）=====
    ax6 = fig.add_subplot(gs[2, 1])
    if orientation_result is not None:
        s_n   = orientation_result['s_norm']
        err_d = orientation_result['errors_deg']
        ax6.plot(s_n, err_d, color='teal', linewidth=1.8, alpha=0.85, label='角度误差')
        ax6.axhline(orientation_result['mean_deg'], color='red', linestyle='--', linewidth=1.5,
                    alpha=0.8, label=f"均值: {orientation_result['mean_deg']:.3f}°")
        ax6.axhline(orientation_result['p95_deg'], color='orange', linestyle='--', linewidth=1.5,
                    alpha=0.8, label=f"P95: {orientation_result['p95_deg']:.3f}°")
        ax6.set_xlabel('归一化弧长', fontsize=10, fontweight='bold')
        ax6.set_ylabel('角度误差 (°)', fontsize=10, fontweight='bold')
        ax6.legend(fontsize=8.5, loc='upper right')
        ax6.grid(True, alpha=0.3, linestyle='--')
        ax6.set_xlim(0, 1)
        _ymax_ori = max(orientation_result['p95_deg'] * 1.5, 0.1)
        ax6.set_ylim(0, _ymax_ori)
    else:
        ax6.text(0.5, 0.5, '姿态误差\n(未计算)',
                 transform=ax6.transAxes, ha='center', va='center',
                 fontsize=13, color='gray', style='italic')
    ax6.set_title('姿态角度误差（沿弧长）', fontsize=11, fontweight='bold', pad=10)

    # ===== 图5（原编号）: 统计信息面板（右侧，占据三行）=====
    ax4 = fig.add_subplot(gs[:, 2])
    ax4.axis('off')

    _apd_str = (
        f"  【1. 平均垂直距离 (轨迹复现精度)】\n"
        f"     平均误差:  {apd_result['mean']:>10.4f} mm\n"
        f"     标准差:    {apd_result['std']:>10.4f} mm\n"
        f"     最大误差:  {apd_result['max']:>10.4f} mm\n"
        f"     中位数:    {apd_result['median']:>10.4f} mm\n"
        f"     P95:       {apd_result['p95']:>10.4f} mm\n"
    ) if apd_result is not None else "  【1. 平均垂直距离 (APD)】: 未计算\n"

    _fre_str = (
        f"  【2. Fréchet距离 (最大轨迹偏差)】\n"
        f"     Fréchet距离: {frechet_result['frechet_distance']:>8.4f} mm\n"
        f"     最大偏差: 回放#{frechet_result['worst_replay_idx']} <-> 示教#{frechet_result['worst_teach_idx']}\n"
    ) if frechet_result is not None else "  【2. 离散Fréchet距离】: 未计算\n"

    _ep_str = (
        f"  【3. 端点定位误差】\n"
        f"     起点误差:  {endpoint_result['start_error']:>10.4f} mm\n"
        f"     终点误差:  {endpoint_result['end_error']:>10.4f} mm\n"
    ) if endpoint_result is not None else "  【3. 端点误差】: 未计算\n"

    _jerk_str = (
        f"  【4. Jerk曲率变化率 (轨迹平滑性)】\n"
        f"     示教Jerk均值:  {jerk_result['teach']['mean']:>9.6f} 1/mm²\n"
        f"     回放Jerk均值:  {jerk_result['replay']['mean']:>9.6f} 1/mm²\n"
        f"     曲线差异RMSE:  {jerk_result['comparison']['rmse']:>9.6f} 1/mm²\n"
        f"     曲线差异MAE:   {jerk_result['comparison']['mean_abs_diff']:>9.6f} 1/mm²\n"
    ) if jerk_result is not None else "  【4. Jerk曲率变化率】: 未计算\n"

    _ori_str = (
        f"  【5. 姿态角度误差】\n"
        f"     平均角度误差:  {orientation_result['mean_deg']:>8.4f}°\n"
        f"     标准差:        {orientation_result['std_deg']:>8.4f}°\n"
        f"     最大角度误差:  {orientation_result['max_deg']:>8.4f}°\n"
        f"     中位数:        {orientation_result['median_deg']:>8.4f}°\n"
        f"     P95:           {orientation_result['p95_deg']:>8.4f}°\n"
    ) if orientation_result is not None else "  【5. 姿态角度误差】: 未计算\n"

    info_text = (
        f"\n  {'='*56}\n"
        f"    机械臂TCP轨迹回放匹配误差分析报告\n"
        f"  {'='*56}\n\n"
        f"{_apd_str}\n"
        f"{_fre_str}\n"
        f"{_ep_str}\n"
        f"{_jerk_str}\n"
        f"{_ori_str}\n"
        f"  {'-'*56}\n"
        f"        ⭐ 综合评级: {overall_grade} ⭐\n"
        f"  {'-'*56}\n\n"
        f"  验收标准参考 (ISO 9283):\n"
        f"    优秀: APD<0.5mm, 端点<1.0mm, 姿态<1°\n"
        f"    良好: APD<2.0mm, 端点<2.0mm, 姿态<3°\n"
        f"    可接受: APD<5.0mm, 端点<5.0mm, 姿态<5°\n"
        f"  {'='*56}\n"
    )
    
    ax4.text(0.05, 0.5, info_text, transform=ax4.transAxes,
             fontsize=9, verticalalignment='center',
             fontfamily='sans-serif',  # 改用支持中文的字体
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))
    
    plt.suptitle('机械臂TCP轨迹回放匹配误差综合分析', fontsize=16, fontweight='bold', y=0.98)

    plt.show()

    if save_path:
        base, ext = os.path.splitext(save_path)
        dpi = CONFIG['figure_save_dpi']

        # ── 独立文件1: 3D 轨迹视图 ──
        fig_3d = plt.figure(figsize=(12, 9))
        ax_3d = fig_3d.add_subplot(111, projection='3d')
        ax_3d.plot(teach_traj[:, 0], teach_traj[:, 1], teach_traj[:, 2],
                   color=colors['teach'], linewidth=2.5, label='示教轨迹', alpha=0.8)
        ax_3d.plot(replay_traj[:, 0], replay_traj[:, 1], replay_traj[:, 2],
                   color=colors['replay'], linewidth=2.5, label='回放轨迹', alpha=0.8)
        ax_3d.scatter(*teach_traj[0], c='green', s=sizes['start'], marker='o',
                      edgecolors='darkgreen', linewidths=3, label='起点', zorder=5)
        ax_3d.scatter(*teach_traj[-1], c='red', s=sizes['end'], marker='s',
                      edgecolors='darkred', linewidths=3, label='终点', zorder=5)
        if frechet_result is not None:
            ax_3d.scatter(*replay_traj[worst_i], c='purple', s=sizes['worst'], marker='*',
                          edgecolors='black', linewidths=2, label='最大偏差点', zorder=6)
            ax_3d.quiver(replay_traj[worst_i][0], replay_traj[worst_i][1], replay_traj[worst_i][2],
                         error_vec[0], error_vec[1], error_vec[2],
                         color='purple', arrow_length_ratio=0.2, linewidth=2, zorder=4)
        ax_3d.set_xlabel('X (mm)', fontsize=11, fontweight='bold')
        ax_3d.set_ylabel('Y (mm)', fontsize=11, fontweight='bold')
        ax_3d.set_zlabel('Z (mm)', fontsize=11, fontweight='bold')
        ax_3d.set_title('TCP轨迹回放对比 - 3D视图', fontsize=13, fontweight='bold', pad=15)
        ax_3d.legend(fontsize=10, loc='upper left')
        ax_3d.grid(True, alpha=0.3)
        all_points_3d = np.vstack([teach_traj, replay_traj])
        max_range_3d = np.array([all_points_3d[:, 0].max() - all_points_3d[:, 0].min(),
                                  all_points_3d[:, 1].max() - all_points_3d[:, 1].min(),
                                  all_points_3d[:, 2].max() - all_points_3d[:, 2].min()]).max() / 2.0
        mid_x_3d = (all_points_3d[:, 0].max() + all_points_3d[:, 0].min()) * 0.5
        mid_y_3d = (all_points_3d[:, 1].max() + all_points_3d[:, 1].min()) * 0.5
        mid_z_3d = (all_points_3d[:, 2].max() + all_points_3d[:, 2].min()) * 0.5
        ax_3d.set_xlim(mid_x_3d - max_range_3d, mid_x_3d + max_range_3d)
        ax_3d.set_ylim(mid_y_3d - max_range_3d, mid_y_3d + max_range_3d)
        ax_3d.set_zlim(mid_z_3d - max_range_3d, mid_z_3d + max_range_3d)
        path_3d = base + '_3d' + ext
        fig_3d.savefig(path_3d, dpi=dpi, bbox_inches='tight')
        plt.close(fig_3d)
        print(f"[INFO] 3D视图已保存: {path_3d}")

        if distances is not None:
            # ── 独立文件2: 误差沿轨迹变化趋势 ──
            fig_trend, ax_trend = plt.subplots(figsize=(12, 6))
            point_indices_s = np.arange(len(distances))
            ax_trend.plot(point_indices_s, distances, color='steelblue', linewidth=1.5,
                          alpha=0.8, label='点对点误差')
            ax_trend.axhline(apd_result['mean'], color='red', linestyle='--', linewidth=2,
                             label=f"平均: {apd_result['mean']:.3f} mm", alpha=0.8)
            ax_trend.axhline(apd_result['p95'], color='orange', linestyle='--', linewidth=2,
                             label=f"P95: {apd_result['p95']:.3f} mm", alpha=0.8)
            ax_trend.axhline(apd_result['max'], color='darkred', linestyle=':', linewidth=2,
                             label=f"最大: {apd_result['max']:.3f} mm", alpha=0.8)
            ax_trend.scatter(max_idx, distances[max_idx], color='darkred', s=100,
                             marker='o', edgecolors='black', linewidths=2, zorder=5)
            ax_trend.annotate(f'最大误差\n#{max_idx}\n{distances[max_idx]:.3f}mm',
                              xy=(max_idx, distances[max_idx]),
                              xytext=(max_idx + len(distances) * 0.1, distances[max_idx]),
                              fontsize=9, color='darkred', fontweight='bold',
                              arrowprops=dict(arrowstyle='->', color='darkred', lw=1.5),
                              bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.7))
            ax_trend.set_xlabel('回放轨迹点索引', fontsize=10, fontweight='bold')
            ax_trend.set_ylabel('垂直距离 (mm)', fontsize=10, fontweight='bold')
            ax_trend.set_title('误差沿轨迹变化趋势', fontsize=12, fontweight='bold')
            ax_trend.legend(fontsize=9, loc='upper right')
            ax_trend.grid(True, alpha=0.3, linestyle='--')
            ax_trend.set_xlim(0, len(distances))
            ax_trend.set_ylim(0, max(distances.max() * 1.15, apd_result['mean'] * 2))
            path_trend = base + '_error_trend' + ext
            fig_trend.savefig(path_trend, dpi=dpi, bbox_inches='tight')
            plt.close(fig_trend)
            print(f"[INFO] 误差趋势图已保存: {path_trend}")

            # ── 独立文件3: 垂直距离分布 ──
            fig_dist, ax_dist = plt.subplots(figsize=(10, 7))
            n_d, bins_d, patches_d = ax_dist.hist(distances, bins=50, color='steelblue',
                                                   edgecolor='black', alpha=0.7)
            cm_d = plt.cm.RdYlGn_r
            bin_centers_d = 0.5 * (bins_d[:-1] + bins_d[1:])
            col_d = (bin_centers_d - bin_centers_d.min()) / (bin_centers_d.max() - bin_centers_d.min())
            for c_d, p_d in zip(col_d, patches_d):
                plt.setp(p_d, 'facecolor', cm_d(c_d))
            ax_dist.axvline(apd_result['mean'], color='red', linestyle='--', linewidth=2.5,
                            label=f"平均: {apd_result['mean']:.3f} mm")
            ax_dist.axvline(apd_result['p95'], color='orange', linestyle='--', linewidth=2.5,
                            label=f"P95: {apd_result['p95']:.3f} mm")
            ax_dist.axvline(apd_result['max'], color='darkred', linestyle=':', linewidth=2,
                            label=f"最大: {apd_result['max']:.3f} mm")
            ax_dist.set_xlabel('垂直距离 (mm)', fontsize=10, fontweight='bold')
            ax_dist.set_ylabel('频数', fontsize=10, fontweight='bold')
            ax_dist.set_title('垂直距离分布', fontsize=12, fontweight='bold')
            ax_dist.legend(fontsize=9, loc='upper right')
            ax_dist.grid(True, alpha=0.3, axis='y')
            path_dist = base + '_distribution' + ext
            fig_dist.savefig(path_dist, dpi=dpi, bbox_inches='tight')
            plt.close(fig_dist)
            print(f"[INFO] 分布图已保存: {path_dist}")

        if jerk_result is not None:
            # ── 独立文件4: Jerk曲率变化率折线图 ──
            fig_jerk, ax_jerk = plt.subplots(figsize=(12, 5.8))
            ax_jerk.plot(jerk_result['teach']['s_norm'], jerk_result['teach']['jerk'],
                         color=colors['teach'], linewidth=1.8, alpha=0.85, label='示教Jerk曲线')
            ax_jerk.plot(jerk_result['replay']['s_norm'], jerk_result['replay']['jerk'],
                         color=colors['replay'], linewidth=1.8, alpha=0.85, label='回放Jerk曲线')
            ax_jerk.axhline(jerk_result['teach']['mean'], color=colors['teach'], linestyle='--',
                            linewidth=1.5, alpha=0.7, label=f"示教均值: {jerk_result['teach']['mean']:.4f}")
            ax_jerk.axhline(jerk_result['replay']['mean'], color=colors['replay'], linestyle='--',
                            linewidth=1.5, alpha=0.7, label=f"回放均值: {jerk_result['replay']['mean']:.4f}")
            ax_jerk.set_xlabel('归一化弧长', fontsize=10, fontweight='bold')
            ax_jerk.set_ylabel('Jerk = |dκ/ds| (1/mm²)', fontsize=10, fontweight='bold')
            ax_jerk.set_title('Jerk曲率变化率折线图（轨迹平滑性）', fontsize=12, fontweight='bold')
            ax_jerk.grid(True, alpha=0.3, linestyle='--')
            ax_jerk.legend(fontsize=9, loc='upper right', ncol=2)
            ax_jerk.set_xlim(0, 1)
            # 截断 Y 轴至 P99，避免尖峰压扁有效曲线
            _jv2 = np.concatenate([jerk_result['teach']['jerk'], jerk_result['replay']['jerk']])
            _ymax2 = np.percentile(_jv2[_jv2 > 0], 99) * 1.5 if (_jv2 > 0).any() else 1e-6
            ax_jerk.set_ylim(0, max(_ymax2, 1e-9))
            path_jerk = base + '_jerk_trend' + ext
            fig_jerk.savefig(path_jerk, dpi=dpi, bbox_inches='tight')
            plt.close(fig_jerk)
            print(f"[INFO] Jerk折线图已保存: {path_jerk}")

        if orientation_result is not None:
            # ── 独立文件5: 姿态角度误差折线图 ──
            fig_ori, ax_ori = plt.subplots(figsize=(12, 5.8))
            ax_ori.plot(orientation_result['s_norm'], orientation_result['errors_deg'],
                        color='teal', linewidth=1.8, alpha=0.85, label='角度误差')
            ax_ori.axhline(orientation_result['mean_deg'], color='red', linestyle='--', linewidth=2,
                           label=f"均值: {orientation_result['mean_deg']:.3f}°", alpha=0.8)
            ax_ori.axhline(orientation_result['p95_deg'], color='orange', linestyle='--', linewidth=2,
                           label=f"P95: {orientation_result['p95_deg']:.3f}°", alpha=0.8)
            ax_ori.axhline(orientation_result['max_deg'], color='darkred', linestyle=':', linewidth=1.5,
                           label=f"最大: {orientation_result['max_deg']:.3f}°", alpha=0.8)
            ax_ori.set_xlabel('归一化弧长', fontsize=10, fontweight='bold')
            ax_ori.set_ylabel('角度误差 (°)', fontsize=10, fontweight='bold')
            ax_ori.set_title('姿态角度误差沿轨迹变化', fontsize=12, fontweight='bold')
            ax_ori.legend(fontsize=9, loc='upper right')
            ax_ori.grid(True, alpha=0.3, linestyle='--')
            ax_ori.set_xlim(0, 1)
            ax_ori.set_ylim(0, max(orientation_result['p95_deg'] * 1.5, 0.1))
            path_ori = base + '_orientation_error' + ext
            fig_ori.savefig(path_ori, dpi=dpi, bbox_inches='tight')
            plt.close(fig_ori)
            print(f"[INFO] 姿态误差图已保存: {path_ori}")


def plot_trajectory_matching_3d_page(replay_traj, teach_traj,
                                     apd_result, frechet_result, endpoint_result, jerk_result,
                                     overall_grade, save_path=None,
                                     orientation_result=None):
    """
    单页3D可视化界面：仅显示轨迹3D对比与关键统计信息。

    Args:
        replay_traj: 回放轨迹
        teach_traj: 示教轨迹
        apd_result: APD计算结果
        frechet_result: Fréchet距离结果
        endpoint_result: 端点误差结果
        jerk_result: Jerk计算结果
        overall_grade: 综合评级
        save_path: 保存路径（可选）
        orientation_result: 姿态误差结果（可选）
    """
    fig = plt.figure(figsize=(13, 9))
    ax = fig.add_subplot(111, projection='3d')

    colors = CONFIG['trajectory_colors']
    sizes = CONFIG['marker_size']

    # 轨迹曲线
    ax.plot(teach_traj[:, 0], teach_traj[:, 1], teach_traj[:, 2],
            color=colors['teach'], linewidth=2.5, label='示教轨迹', alpha=0.85)
    ax.plot(replay_traj[:, 0], replay_traj[:, 1], replay_traj[:, 2],
            color=colors['replay'], linewidth=2.5, label='回放轨迹', alpha=0.85)

    # 起终点
    ax.scatter(*teach_traj[0], c='green', s=sizes['start'], marker='o',
               edgecolors='darkgreen', linewidths=2.5, label='起点', zorder=5)
    ax.scatter(*teach_traj[-1], c='red', s=sizes['end'], marker='s',
               edgecolors='darkred', linewidths=2.5, label='终点', zorder=5)
    # 最大偏差点与误差向量（仅 Fréchet 已计算时显示）
    worst_i, worst_j = 0, 0
    if frechet_result is not None:
        worst_i = frechet_result['worst_replay_idx']
        worst_j = frechet_result['worst_teach_idx']
        ax.scatter(*replay_traj[worst_i], c='purple', s=sizes['worst'], marker='*',
                   edgecolors='black', linewidths=1.8, label='最大偏差点', zorder=6)
        error_vec = teach_traj[worst_j] - replay_traj[worst_i]
        ax.quiver(replay_traj[worst_i][0], replay_traj[worst_i][1], replay_traj[worst_i][2],
                  error_vec[0], error_vec[1], error_vec[2],
                  color='purple', arrow_length_ratio=0.2, linewidth=2, zorder=4)

    # 坐标范围等比例
    all_points = np.vstack([teach_traj, replay_traj])
    max_range = np.array([
        all_points[:, 0].max() - all_points[:, 0].min(),
        all_points[:, 1].max() - all_points[:, 1].min(),
        all_points[:, 2].max() - all_points[:, 2].min()
    ]).max() / 2.0
    mid_x = (all_points[:, 0].max() + all_points[:, 0].min()) * 0.5
    mid_y = (all_points[:, 1].max() + all_points[:, 1].min()) * 0.5
    mid_z = (all_points[:, 2].max() + all_points[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    ax.set_xlabel('X (mm)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Y (mm)', fontsize=11, fontweight='bold')
    ax.set_zlabel('Z (mm)', fontsize=11, fontweight='bold')
    ax.set_title('TCP轨迹回放对比 - 3D单页分析界面', fontsize=14, fontweight='bold', pad=14)
    ax.legend(fontsize=9.5, loc='upper right', bbox_to_anchor=(0.98, 0.98),
              framealpha=0.92, borderpad=0.6, labelspacing=0.4)
    ax.grid(True, alpha=0.3)

    # 统计信息框（动态适应已计算的指标）
    _info_lines = []
    if apd_result is not None:
        _info_lines.append(f"APD均值: {apd_result['mean']:.3f} mm")
    else:
        _info_lines.append("APD均值: 未计算")
    if frechet_result is not None:
        _info_lines.append(f"Fréchet: {frechet_result['frechet_distance']:.3f} mm")
    else:
        _info_lines.append("Fréchet: 未计算")
    if endpoint_result is not None:
        _info_lines.append(f"起点误差: {endpoint_result['start_error']:.3f} mm")
        _info_lines.append(f"终点误差: {endpoint_result['end_error']:.3f} mm")
    else:
        _info_lines.append("端点误差: 未计算")
    if jerk_result is not None:
        _info_lines.append(f"回放Jerk均值: {jerk_result['replay']['mean']:.4f} 1/mm²")
        _info_lines.append(f"Jerk差异RMSE: {jerk_result['comparison']['rmse']:.4f} 1/mm²")
    else:
        _info_lines.append("Jerk: 未计算")
    if orientation_result is not None:
        _info_lines.append(f"姿态均值误差: {orientation_result['mean_deg']:.3f}°")
        _info_lines.append(f"姿态P95误差:  {orientation_result['p95_deg']:.3f}°")
    else:
        _info_lines.append("姿态误差: 未计算")
    _info_lines.append(f"评级: {overall_grade}")
    info_text = "\n".join(_info_lines)
    ax.text2D(0.02, 0.98, info_text,
              transform=ax.transAxes,
              ha='left', va='top', fontsize=10,
              bbox=dict(boxstyle='round,pad=0.4', facecolor='white', edgecolor='gray', alpha=0.92))

    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=CONFIG['figure_save_dpi'], bbox_inches='tight')
        print(f"[INFO] 单页3D图已保存: {save_path}")

    plt.show()


# ============================================================================
#                          主函数接口
# ============================================================================

def compute_trajectory_matching_error(replay_csv, teach_csv, 
                                      visualize=True, 
                                      save_report=False,
                                      visualize_mode='full'):
    """
    机械臂TCP轨迹回放匹配误差计算（主函数）
    
    Args:
        replay_csv (str): 回放轨迹CSV文件路径
        teach_csv (str): 示教轨迹CSV文件路径
        visualize (bool): 是否生成可视化图表
        save_report (bool): 是否保存文本报告
        visualize_mode (str): 可视化模式
            - 'full': 综合界面 + 多张拆分图
            - 'single_3d': 仅生成单页3D界面
    
    Returns:
        dict: 完整评估结果
        {
            'apd': {...},           # 平均垂直距离结果
            'frechet': {...},       # Fréchet距离结果
            'endpoints': {...},     # 端点误差结果
            'jerk': {...},          # Jerk曲率变化率结果
            'orientation': {...},   # 姿态角度误差结果
            'overall_grade': str,   # 综合评级
            'pass': bool            # 是否通过验收
        }
    """
    print("\n" + "="*70)
    print("         机械臂TCP轨迹回放匹配误差分析")
    print("="*70)
    
    # 1. 加载数据
    _mc               = CONFIG['metrics']
    compute_apd       = _mc['compute_apd']
    compute_frechet   = _mc['compute_frechet']
    compute_endpoint  = _mc['compute_endpoint']
    compute_jerk      = _mc['compute_jerk']
    compute_orient    = _mc.get('compute_orientation', False)

    print("\n【步骤1/5】加载轨迹数据...")
    print("-" * 70)

    # 如果需要计算姿态误差，使用带四元数的加载函数
    if compute_orient:
        try:
            replay_traj, replay_quats = load_tcp_trajectory_with_orientation(replay_csv)
            teach_traj,  teach_quats  = load_tcp_trajectory_with_orientation(teach_csv)
        except (ValueError, KeyError) as e:
            print(f"[WARN] 姿态数据加载失败（{e}），退回仅加载位置数据，跳过姿态误差计算")
            replay_traj = load_tcp_trajectory(replay_csv)
            teach_traj  = load_tcp_trajectory(teach_csv)
            replay_quats = teach_quats = None
            compute_orient = False
    else:
        replay_traj = load_tcp_trajectory(replay_csv)
        teach_traj  = load_tcp_trajectory(teach_csv)
        replay_quats = teach_quats = None

    # 2. 计算误差指标（按 CONFIG['metrics'] 配置选择）
    print("\n【步骤2/5】计算核心指标...")
    print("-" * 70)
    if not (compute_apd or compute_frechet or compute_endpoint or compute_jerk or compute_orient):
        print("  ℹ  所有误差指标均已关闭，跳过计算，仅执行可视化")

    apd_result        = compute_average_perpendicular_distance(replay_traj, teach_traj) if compute_apd      else None
    frechet_result    = compute_discrete_frechet_distance(replay_traj, teach_traj)      if compute_frechet  else None
    endpoint_result   = compute_endpoint_errors(replay_traj, teach_traj)                if compute_endpoint else None
    jerk_result       = compute_jerk_curvature_change_rate(replay_traj, teach_traj)     if compute_jerk     else None
    orientation_result = (
        compute_orientation_error(replay_traj, replay_quats, teach_traj, teach_quats)
        if compute_orient else None
    )

    if not compute_apd:     print("  [跳过] 平均垂直距离 (APD)")
    if not compute_frechet: print("  [跳过] 离散Fréchet距离")
    if not compute_endpoint:print("  [跳过] 端点误差")
    if not compute_jerk:    print("  [跳过] Jerk曲率变化率")
    if not compute_orient:  print("  [跳过] 姿态角度误差")
    
    # 3. 综合评级
    print("\n【步骤3/5】综合评级...")
    print("-" * 70)
    overall_grade, is_pass = determine_overall_grade(
        apd_result, frechet_result, endpoint_result, orientation_result
    )
    print(f"  综合评级: {overall_grade}")
    if is_pass is None:
        print(f"  验收结果: — (无误差数据，无法评级)")
    else:
        print(f"  验收结果: {'通过 ✓' if is_pass else '不通过 ✗'}")
    
    # 4. 构建结果字典
    result = {
        'apd': apd_result,
        'frechet': frechet_result,
        'endpoints': endpoint_result,
        'jerk': jerk_result,
        'orientation': orientation_result,
        'overall_grade': overall_grade,
        'pass': is_pass
    }
    
    # 5. 可视化
    if visualize:
        print("\n【步骤4/5】生成可视化图表...")
        print("-" * 70)

        fig_dir = CONFIG['figure_output_dir']
        if save_report:
            os.makedirs(fig_dir, exist_ok=True)

        if visualize_mode == 'single_3d':
            if save_report:
                fig_save_path = os.path.join(fig_dir, 'trajectory_matching_analysis_3d.png')
            else:
                fig_save_path = None
            plot_trajectory_matching_3d_page(
                replay_traj, teach_traj,
                apd_result, frechet_result, endpoint_result, jerk_result,
                overall_grade,
                save_path=fig_save_path,
                orientation_result=orientation_result,
            )
        else:
            if save_report:
                fig_save_path = os.path.join(fig_dir, 'trajectory_matching_analysis.png')
            else:
                fig_save_path = None
            plot_trajectory_matching_analysis(
                replay_traj, teach_traj,
                apd_result, frechet_result, endpoint_result, jerk_result,
                overall_grade,
                save_path=fig_save_path,
                orientation_result=orientation_result,
            )
    
    # 6. 保存报告
    if save_report:
        print("\n【步骤5/5】保存评估报告...")
        print("-" * 70)
        save_evaluation_report(result, replay_csv, teach_csv)
    
    print("\n" + "="*70)
    print("                    分析完成!")
    print("="*70)
    
    return result


def save_evaluation_report(result, replay_csv, teach_csv):
    """
    保存文本格式的评估报告
    
    Args:
        result: 评估结果字典
        replay_csv: 回放轨迹文件路径
        teach_csv: 示教轨迹文件路径
    """
    report_path = CONFIG['report_output_path']
    
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write("="*70 + "\n")
        f.write("         机械臂TCP轨迹回放匹配误差分析报告\n")
        f.write("="*70 + "\n\n")
        
        f.write("【文件信息】\n")
        f.write(f"  回放轨迹: {os.path.basename(replay_csv)}\n")
        f.write(f"  示教轨迹: {os.path.basename(teach_csv)}\n\n")
        
        apd      = result['apd']
        frechet  = result['frechet']
        endpoints= result['endpoints']
        jerk     = result['jerk']

        if apd is not None:
            f.write("【1. 平均垂直距离 (轨迹复现精度)】\n")
            f.write(f"   平均误差:  {apd['mean']:.4f} mm\n")
            f.write(f"   标准差:    {apd['std']:.4f} mm\n")
            f.write(f"   最大误差:  {apd['max']:.4f} mm\n")
            f.write(f"   中位数:    {apd['median']:.4f} mm\n")
            f.write(f"   P95:       {apd['p95']:.4f} mm\n\n")
        else:
            f.write("【1. 平均垂直距离 (APD)】: 未计算\n\n")

        if frechet is not None:
            f.write("【2. Fréchet距离 (最大轨迹偏差)】\n")
            f.write(f"   Fréchet距离: {frechet['frechet_distance']:.4f} mm\n")
            f.write(f"   最大偏差位置: 回放点#{frechet['worst_replay_idx']} "
                    f"<-> 示教点#{frechet['worst_teach_idx']}\n\n")
        else:
            f.write("【2. 离散Fréchet距离】: 未计算\n\n")

        if endpoints is not None:
            f.write("【3. 端点定位误差】\n")
            f.write(f"   起点误差:  {endpoints['start_error']:.4f} mm\n")
            f.write(f"   终点误差:  {endpoints['end_error']:.4f} mm\n\n")
        else:
            f.write("【3. 端点误差】: 未计算\n\n")

        if jerk is not None:
            f.write("【4. Jerk曲率变化率 (轨迹平滑性)】\n")
            f.write(f"   示教Jerk均值:  {jerk['teach']['mean']:.6f} 1/mm²\n")
            f.write(f"   回放Jerk均值:  {jerk['replay']['mean']:.6f} 1/mm²\n")
            f.write(f"   Jerk差异RMSE:  {jerk['comparison']['rmse']:.6f} 1/mm²\n")
            f.write(f"   Jerk差异MAE:   {jerk['comparison']['mean_abs_diff']:.6f} 1/mm²\n")
            f.write(f"   Jerk差异MAX:   {jerk['comparison']['max_abs_diff']:.6f} 1/mm²\n\n")
        else:
            f.write("【4. Jerk曲率变化率】: 未计算\n\n")

        orientation = result.get('orientation')
        if orientation is not None:
            f.write("【5. 姿态角度误差】\n")
            f.write(f"   平均角度误差:  {orientation['mean_deg']:.4f}°\n")
            f.write(f"   标准差:        {orientation['std_deg']:.4f}°\n")
            f.write(f"   最大角度误差:  {orientation['max_deg']:.4f}°\n")
            f.write(f"   中位数:        {orientation['median_deg']:.4f}°\n")
            f.write(f"   P95:           {orientation['p95_deg']:.4f}°\n\n")
        else:
            f.write("【5. 姿态角度误差】: 未计算\n\n")

        f.write("-"*70 + "\n")
        f.write(f"【综合评级】: {result['overall_grade']}\n")
        _is_pass = result['pass']
        if _is_pass is None:
            f.write("【验收结果】: — (无误差数据，无法评级)\n")
        else:
            f.write(f"【验收结果】: {'通过 ✓' if _is_pass else '不通过 ✗'}\n")
        f.write("="*70 + "\n")
    
    print(f"  报告已保存: {report_path}")


# ============================================================================
#                            主程序
# ============================================================================

def compute_trajectory_matching_error(replay_csv, teach_csv, 
                                      visualize=True, 
                                      save_report=False,
                                      visualize_mode='full'):
    """
    机械臂TCP轨迹回放匹配误差计算（主函数）
    
    Args:
        replay_csv (str): 回放轨迹CSV文件路径
        teach_csv (str): 示教轨迹CSV文件路径
        visualize (bool): 是否生成可视化图表
        save_report (bool): 是否保存文本报告
        visualize_mode (str): 可视化模式
            - 'full': 综合界面 + 多张拆分图
            - 'single_3d': 仅生成单页3D界面
    
    Returns:
        dict: 完整评估结果
        {
            'apd': {...},           # 平均垂直距离结果
            'frechet': {...},       # Fréchet距离结果
            'endpoints': {...},     # 端点误差结果
            'jerk': {...},          # Jerk曲率变化率结果
            'orientation': {...},   # 姿态角度误差结果
            'overall_grade': str,   # 综合评级
            'pass': bool            # 是否通过验收
        }
    """
    print("\n" + "="*70)
    print("         机械臂TCP轨迹回放匹配误差分析")
    print("="*70)
    
    # 1. 加载数据
    _mc               = CONFIG['metrics']
    compute_apd       = _mc['compute_apd']
    compute_frechet   = _mc['compute_frechet']
    compute_endpoint  = _mc['compute_endpoint']
    compute_jerk      = _mc['compute_jerk']
    compute_orient    = _mc.get('compute_orientation', False)

    print("\n【步骤1/5】加载轨迹数据...")
    print("-" * 70)

    # 如果需要计算姿态误差，使用带四元数的加载函数
    if compute_orient:
        try:
            replay_traj, replay_quats = load_tcp_trajectory_with_orientation(replay_csv)
            teach_traj,  teach_quats  = load_tcp_trajectory_with_orientation(teach_csv)
        except (ValueError, KeyError) as e:
            print(f"[WARN] 姿态数据加载失败（{e}），退回仅加载位置数据，跳过姿态误差计算")
            replay_traj = load_tcp_trajectory(replay_csv)
            teach_traj  = load_tcp_trajectory(teach_csv)
            replay_quats = teach_quats = None
            compute_orient = False
    else:
        replay_traj = load_tcp_trajectory(replay_csv)
        teach_traj  = load_tcp_trajectory(teach_csv)
        replay_quats = teach_quats = None

    # 2. 计算误差指标（按 CONFIG['metrics'] 配置选择）
    print("\n【步骤2/5】计算核心指标...")
    print("-" * 70)
    if not (compute_apd or compute_frechet or compute_endpoint or compute_jerk or compute_orient):
        print("  ℹ  所有误差指标均已关闭，跳过计算，仅执行可视化")

    apd_result        = compute_average_perpendicular_distance(replay_traj, teach_traj) if compute_apd      else None
    frechet_result    = compute_discrete_frechet_distance(replay_traj, teach_traj)      if compute_frechet  else None
    endpoint_result   = compute_endpoint_errors(replay_traj, teach_traj)                if compute_endpoint else None
    jerk_result       = compute_jerk_curvature_change_rate(replay_traj, teach_traj)     if compute_jerk     else None
    orientation_result = (
        compute_orientation_error(replay_traj, replay_quats, teach_traj, teach_quats)
        if compute_orient else None
    )

    if not compute_apd:     print("  [跳过] 平均垂直距离 (APD)")
    if not compute_frechet: print("  [跳过] 离散Fréchet距离")
    if not compute_endpoint:print("  [跳过] 端点误差")
    if not compute_jerk:    print("  [跳过] Jerk曲率变化率")
    if not compute_orient:  print("  [跳过] 姿态角度误差")
    
    # 3. 综合评级
    print("\n【步骤3/5】综合评级...")
    print("-" * 70)
    overall_grade, is_pass = determine_overall_grade(
        apd_result, frechet_result, endpoint_result, orientation_result
    )
    print(f"  综合评级: {overall_grade}")
    if is_pass is None:
        print(f"  验收结果: — (无误差数据，无法评级)")
    else:
        print(f"  验收结果: {'通过 ✓' if is_pass else '不通过 ✗'}")
    
    # 4. 构建结果字典
    result = {
        'apd': apd_result,
        'frechet': frechet_result,
        'endpoints': endpoint_result,
        'jerk': jerk_result,
        'orientation': orientation_result,
        'overall_grade': overall_grade,
        'pass': is_pass
    }
    
    # 5. 可视化
    if visualize:
        print("\n【步骤4/5】生成可视化图表...")
        print("-" * 70)

        fig_dir = CONFIG['figure_output_dir']
        if save_report:
            os.makedirs(fig_dir, exist_ok=True)

        if visualize_mode == 'single_3d':
            if save_report:
                fig_save_path = os.path.join(fig_dir, 'trajectory_matching_analysis_3d.png')
            else:
                fig_save_path = None
            plot_trajectory_matching_3d_page(
                replay_traj, teach_traj,
                apd_result, frechet_result, endpoint_result, jerk_result,
                overall_grade,
                save_path=fig_save_path,
                orientation_result=orientation_result,
            )
        else:
            if save_report:
                fig_save_path = os.path.join(fig_dir, 'trajectory_matching_analysis.png')
            else:
                fig_save_path = None
            plot_trajectory_matching_analysis(
                replay_traj, teach_traj,
                apd_result, frechet_result, endpoint_result, jerk_result,
                overall_grade,
                save_path=fig_save_path,
                orientation_result=orientation_result,
            )
    
    # 6. 保存报告
    if save_report:
        print("\n【步骤5/5】保存评估报告...")
        print("-" * 70)
        save_evaluation_report(result, replay_csv, teach_csv)
    
    print("\n" + "="*70)
    print("                    分析完成!")
    print("="*70)
    
    return result


def save_evaluation_report(result, replay_csv, teach_csv):
    """
    保存文本格式的评估报告
    
    Args:
        result: 评估结果字典
        replay_csv: 回放轨迹文件路径
        teach_csv: 示教轨迹文件路径
    """
    report_path = CONFIG['report_output_path']
    
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write("="*70 + "\n")
        f.write("         机械臂TCP轨迹回放匹配误差分析报告\n")
        f.write("="*70 + "\n\n")
        
        f.write("【文件信息】\n")
        f.write(f"  回放轨迹: {os.path.basename(replay_csv)}\n")
        f.write(f"  示教轨迹: {os.path.basename(teach_csv)}\n\n")
        
        apd      = result['apd']
        frechet  = result['frechet']
        endpoints= result['endpoints']
        jerk     = result['jerk']

        if apd is not None:
            f.write("【1. 平均垂直距离 (轨迹复现精度)】\n")
            f.write(f"   平均误差:  {apd['mean']:.4f} mm\n")
            f.write(f"   标准差:    {apd['std']:.4f} mm\n")
            f.write(f"   最大误差:  {apd['max']:.4f} mm\n")
            f.write(f"   中位数:    {apd['median']:.4f} mm\n")
            f.write(f"   P95:       {apd['p95']:.4f} mm\n\n")
        else:
            f.write("【1. 平均垂直距离 (APD)】: 未计算\n\n")

        if frechet is not None:
            f.write("【2. Fréchet距离 (最大轨迹偏差)】\n")
            f.write(f"   Fréchet距离: {frechet['frechet_distance']:.4f} mm\n")
            f.write(f"   最大偏差位置: 回放点#{frechet['worst_replay_idx']} "
                    f"<-> 示教点#{frechet['worst_teach_idx']}\n\n")
        else:
            f.write("【2. 离散Fréchet距离】: 未计算\n\n")

        if endpoints is not None:
            f.write("【3. 端点定位误差】\n")
            f.write(f"   起点误差:  {endpoints['start_error']:.4f} mm\n")
            f.write(f"   终点误差:  {endpoints['end_error']:.4f} mm\n\n")
        else:
            f.write("【3. 端点误差】: 未计算\n\n")

        if jerk is not None:
            f.write("【4. Jerk曲率变化率 (轨迹平滑性)】\n")
            f.write(f"   示教Jerk均值:  {jerk['teach']['mean']:.6f} 1/mm²\n")
            f.write(f"   回放Jerk均值:  {jerk['replay']['mean']:.6f} 1/mm²\n")
            f.write(f"   Jerk差异RMSE:  {jerk['comparison']['rmse']:.6f} 1/mm²\n")
            f.write(f"   Jerk差异MAE:   {jerk['comparison']['mean_abs_diff']:.6f} 1/mm²\n")
            f.write(f"   Jerk差异MAX:   {jerk['comparison']['max_abs_diff']:.6f} 1/mm²\n\n")
        else:
            f.write("【4. Jerk曲率变化率】: 未计算\n\n")

        orientation = result.get('orientation')
        if orientation is not None:
            f.write("【5. 姿态角度误差】\n")
            f.write(f"   平均角度误差:  {orientation['mean_deg']:.4f}°\n")
            f.write(f"   标准差:        {orientation['std_deg']:.4f}°\n")
            f.write(f"   最大角度误差:  {orientation['max_deg']:.4f}°\n")
            f.write(f"   中位数:        {orientation['median_deg']:.4f}°\n")
            f.write(f"   P95:           {orientation['p95_deg']:.4f}°\n\n")
        else:
            f.write("【5. 姿态角度误差】: 未计算\n\n")

        f.write("-"*70 + "\n")
        f.write(f"【综合评级】: {result['overall_grade']}\n")
        _is_pass = result['pass']
        if _is_pass is None:
            f.write("【验收结果】: — (无误差数据，无法评级)\n")
        else:
            f.write(f"【验收结果】: {'通过 ✓' if _is_pass else '不通过 ✗'}\n")
        f.write("="*70 + "\n")
    
    print(f"  报告已保存: {report_path}")


# ============================================================================
#                            主程序
# ============================================================================

if __name__ == "__main__":
    # ========== 配置文件路径 ==========
    
    # 示教轨迹（参考轨迹）
    teach_csv = r"E:\Unity cangku\lighthouse_3.4（走数据备份）\Assets\Scripts\VisualServo\算法\圆形\圆形_tcp2 - tcp.csv"
    # 回放轨迹（待评估轨迹）
    replay_csv = r"E:\Unity cangku\lighthouse_3.4（走数据备份）\Assets\Scripts\VisualServo\算法\圆形\圆形_tracker2 - tcp_transformed.csv"
    # ========== 执行分析 ==========
    
    result = compute_trajectory_matching_error(
        replay_csv=replay_csv,
        teach_csv=teach_csv,
        visualize=True,      # 生成可视化图表
        save_report=True,    # 保存文本报告
        visualize_mode='full'  # 仅生成 trajectory_matching_analysis_3d.png 单页界面
    )
    
    # ========== 打印摘要 ==========
    
    print("\n" + "="*70)
    print("                      评估结果摘要")
    print("="*70)
    
    if result['apd'] is not None:
        print(f"\n【平均垂直距离】: {result['apd']['mean']:.4f} mm")
    else:
        print(f"\n【平均垂直距离】: 未计算")
    if result['frechet'] is not None:
        print(f"【Fréchet距离】:   {result['frechet']['frechet_distance']:.4f} mm")
    else:
        print(f"【Fréchet距离】:   未计算")
    if result['endpoints'] is not None:
        print(f"【起点误差】:      {result['endpoints']['start_error']:.4f} mm")
        print(f"【终点误差】:      {result['endpoints']['end_error']:.4f} mm")
    else:
        print(f"【起点误差】:      未计算")
        print(f"【终点误差】:      未计算")
    if result['jerk'] is not None:
        print(f"【回放Jerk均值】:  {result['jerk']['replay']['mean']:.6f} 1/mm²")
        print(f"【Jerk差异RMSE】:  {result['jerk']['comparison']['rmse']:.6f} 1/mm²")
    else:
        print(f"【回放Jerk均值】:  未计算")
        print(f"【Jerk差异RMSE】:  未计算")
    if result.get('orientation') is not None:
        print(f"【姿态均值误差】:  {result['orientation']['mean_deg']:.4f}°")
        print(f"【姿态P95误差】:   {result['orientation']['p95_deg']:.4f}°")
        print(f"【姿态最大误差】:  {result['orientation']['max_deg']:.4f}°")
    else:
        print(f"【姿态误差】:      未计算")
    print(f"\n【综合评级】:      {result['overall_grade']}")
    _pass = result['pass']
    if _pass is None:
        print(f"【验收状态】:      — (无误差数据，无法评级)")
    else:
        print(f"【验收状态】:      {'✓ 通过' if _pass else '✗ 不通过'}")
    print("\n" + "="*70)

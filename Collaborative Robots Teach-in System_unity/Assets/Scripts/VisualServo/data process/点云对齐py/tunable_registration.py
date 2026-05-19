"""
可调参数的轨迹配准工具
======================

集成多种配准方法，支持参数调优

方法选择：
1. Kabsch（SVD）- 适合时间对齐的一一对应点云
2. RANSAC+GICP  - 适合无对应关系的点云
3. 纯ICP        - 适合初始位置接近的点云

Author: AI Assistant
Date: 2026-02-03
"""

import numpy as np
import open3d as o3d
from scipy import interpolate
from scipy.ndimage import gaussian_filter1d
import matplotlib.pyplot as plt
import copy
import sys
import os
import json

# 配置matplotlib中文显示
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'STSong', 'KaiTi']  # 中文字体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# ============================================================================
#                           📝 可调参数区域
# ============================================================================

# ========== 文件路径 ==========
SOURCE_CSV = r"E:\Unity cangku\lighthouse_3.4\Assets\StreamingAssets\TrackerRecordings\PlaybackRecord_PlaybackRecord_原始轨迹1_6_20260415_103320_6_20260415_103957 - 副本.csv"
TARGET_CSV = r"E:\Unity cangku\lighthouse_3.4（走数据备份）\Assets\StreamingAssets\工件\TRACKER - tcp.csv"

# ========== 配准方法选择 ==========
# "kabsch"           - Kabsch算法（点对点对应，最简单直接）
# "kabsch_icp"       - Kabsch+ICP混合精配准
# "kabsch_segmented" - ⭐⭐ 分段Kabsch（推荐！）
# "mls"              - ⭐⭐⭐ 移动最小二乘（MLS，连续非刚性变换）
# "icp"              - 标准ICP（需要初始位置接近）
# "ransac"           - RANSAC+GICP（全局配准）
METHOD = "mls"  # ⭐⭐⭐ 使用MLS移动最小二乘配准

# ========== ⭐ 时间对齐参数（关键！） ==========
TIME_ALIGN = {
    "enable": True,                 # 是否启用时间对齐（强烈建议开启）
    "method": "adaptive_spatial",      # ⭐ 重采样方法选择：
                                    #    "chord_spatial"  = ⭐⭐⭐ 真3D弦距重采样（解决海岸线悖论）
                                    #    "adaptive_spatial"= 旧版自适应空间重采样（需配合预平滑）
                                    #    "arc_length"     = 传统弧长重采样
                                    #    "dtw"            = DTW动态时间规整
    "num_samples": 5000,            # 仅在arc_length模式使用（自适应模式自动计算）
    "time_col": 2,                  # 时间列索引（TimeFromStart_s）
}

# ========== ⭐⭐⭐ 序列号对应模式（同一轨迹不同坐标系观测）==========
# 💡 当训练数据(tracker.csv)和复用数据是同一物理轨迹的不同坐标系观测时，
#    序列号一一对应，无需通过弧长猜测对应关系！
SEQUENCE_ALIGN_MODE = {
    "enable": False,              # ⭐ 是否启用序列号对应模式
    "verify_frame_count": True,   # 验证训练和复用数据帧数是否一致
    "pre_smooth_only": True,      # 仅做平滑去噪（不重采样，保持序列号）
}

# ========== ⭐⭐⭐ 空间重采样参数 ==========
# 两种方法共用此配置，通过 TIME_ALIGN["method"] 选择使用哪种
ADAPTIVE_SPATIAL_RESAMPLE = {
    # --- ⭐ 新版弦距重采样参数 (chord_spatial) ---
    "delta_d": 1.3,                    # ⭐ 采样步长门限(mm)，相邻采样点最小弦距
                                     #   0.5 = 高密度（保留更多细节）
                                     #   1.0 = 推荐值（平衡抗噪与细节）
                                     #   2.0 = 低密度（强抗噪，可能丢失细节）
    "densify_factor": 50,            # ⭐ 插值加密倍数（chord_spatial专用）
                                     #   10 = 默认（0.1mm级精度）
                                     #   20 = 高精度（0.05mm级）
    # --- 旧版自适应弧长重采样参数 (adaptive_spatial) ---
    "noise_level": 2.5,              # 测量噪声水平(mm)（adaptive_spatial专用）
    "noise_suppression_factor": 2,   # 噪声抑制因子（1.5推荐）
    # --- 公共参数 ---
    "min_samples": 100,              # 最小采样点数（保证特征分辨率）
    "max_samples": 5000,             # 最大采样点数（控制计算成本）
}

# ========== ⭐⭐ 分段配准参数 ==========
SEGMENTED_PARAMS = {
    "num_segments": 20,             # ⭐ 初始分段数（推荐30-60段）
    "overlap_ratio": 0.35,           # ⭐ 重叠比例（0.3-0.5，越大越平滑）
    "blend_method": "gaussian",     # ⭐ 融合方法："gaussian"（推荐）或"linear"
    "adaptive_refine": True,        # ⭐ 启用自适应细分
    "refine_threshold": 0.5,        # ⭐ 误差阈值(mm)，RMSE超过此值的段会被细分
    "max_refine_depth": 1,          # ⭐ 最大细分深度（每层分2段）
    "min_segment_points": 30,       # ⭐ 最小分段点数（防止过度细分）
}

# ========== ⭐⭐⭐ MLS移动最小二乘参数 ==========
MLS_PARAMS = {
    "bandwidth": None,                # ⭐ 带宽 h（None=自动选择）
    "bandwidth_method": "loocv",      # "empirical"=经验公式, "loocv"=留一交叉验证
    "loocv_candidates": [0.3],  # LOOCV候选带宽
    "grid_size": 200,                 # 预计算网格点数（200~500）
    "endpoint_extend": 0.10,          # 端点镜像延伸比例（两端各延伸10%）
    "min_effective_weight": 0.01,     # 最小有效权重（低于此值不计入邻居）
    "min_effective_neighbors": 5,     # 最小有效邻居数（不足时回退）
    "enable_ransac_weights": True,    # 是否集成RANSAC异常值剔除
}

# ========== ⭐⭐⭐ RANSAC鲁棒配准参数（解决飞点问题）==========
# 💡 在SVD分解前进行异常值剔除，防止飞点主导变换矩阵计算
RANSAC_KABSCH_PARAMS = {
    "enable": True,                 # ⭐ 是否启用RANSAC-Kabsch（强烈推荐！）
    "max_iterations": 200,          # RANSAC采样迭代次数（50-200）
    "inlier_threshold": 10,        # ⭐ 内点判定阈值(mm)，误差小于此值视为内点
    "min_inlier_ratio": 0.5,        # 最小内点比例（低于此值认为配准失败）
    "min_sample_points": 5,         # 最小采样点数（3个点确定刚体变换）
}

# ========== ⭐⭐ 预平滑参数（弧长重采样前，消除噪声对弧长计算的影响）==========
# 💡 adaptive_spatial 方法需要预平滑（建议开启）
#    chord_spatial 方法自身解决海岸线悖论，不强依赖预平滑，但适当平滑仍有助于提升配准稳定性。
PRE_SMOOTH = {
    "enable": True,                # ⭐ 是否启用预平滑（建议开启，尤其是adaptive_spatial）
    "method": "gaussian",         # "gaussian" / "bspline" / "rdp_pchip"
    "gaussian_sigma": 3,          # [Gaussian] 中等平滑（原0.8，针对10%噪声提升到1.5）
    "bspline_smoothing": 3.0,       # [B-Spline] 平滑因子
    "bspline_k": 3,                 # [B-Spline] 次数
    "rdp_epsilon": 2,               # [RDP+PCHIP] RDP简化阈值(mm)，越大越激进地剔除抖动点
    "rdp_median_window": 9,          # [RDP+PCHIP] 中值滤波窗口（奇数3-9），抑制离群点防止RDP误判
}

# ========== 后处理平滑参数（弧长重采样后，进一步去噪） ==========
POST_SMOOTH = {
    "enable_smoothing": True,       # ⭐ 开启平滑去噪
    "smoothing_method": "gaussian", # "gaussian" 或 "bspline"
    "gaussian_sigma": 5,          # ⭐ 高斯核半宽 ★已改为 mm 单位（空间几何平滑）
    "bspline_smoothing": 3.0,       # B-Spline平滑因子（1-100）
    "bspline_k": 3,                 # B-Spline次数（3或5）
    "downsample_ratio": 1.0,        # 下采样比例（1.0=不下采样）
}

# ========== ⭐ 异常值剔除参数 ==========
OUTLIER_REMOVAL = {
    "enable": True,                 # 是否启用异常值剔除
    "method": "iterative",          # "percentile"=百分位, "iterative"=迭代剔除
    "percentile_threshold": 90,     # ⭐ 剔除超过P90的点
    "iterative_rounds": 3,          # 迭代剔除轮数
    "iterative_sigma": 2.0,         # ⭐ 更严格的sigma阈值（剔除>μ+2*σ的点）
}

# ========== ICP/GICP参数（优化） ==========
ICP_PARAMS = {
    "voxel_size": 1.0,              # ⭐ 减小体素（提高精度）
    "max_correspondence_distance": 5.0,   # ⭐ 减小搜索半径
    "max_iteration": 200,           # ⭐ 增加迭代次数
    "relative_fitness": 1e-10,      # ⭐ 更严格的收敛阈值
    "relative_rmse": 1e-10,         # ⭐ 更严格的RMSE收敛阈值
}

# ========== RANSAC参数 ==========
RANSAC_PARAMS = {
    "voxel_size": 2.0,              # 体素大小
    "distance_threshold_multiplier": 1.5,  # 距离阈值 = voxel_size × 此值
    "max_iteration": 4000000,       # 最大迭代次数
    "confidence": 0.9999,           # 置信度
    "ransac_n": 4,                  # 采样点数（3或4）
    "edge_length_threshold": 0.9,   # 边长比阈值（0.8-0.95）
}

# ========== 可视化参数 ==========
VIS_PARAMS = {
    "show_error_color": True,       # 按误差着色显示
    "coord_frame_size": 50,         # 坐标系大小
}

# ========== Matplotlib 可视化配置 ==========
MATLOT_VIS_CONFIG = {
    "enable": True,                  # 是否启用matplotlib可视化
    "use_open3d": False,            # 是否使用Open3D窗口（False时仅用matplotlib）
    "show_stages": {                # 选择需要显示的阶段
        "original": False,           # 原始轨迹
        "pre_smoothed": False,       # 预平滑后
        "resampled": False,          # 重采样后
        "post_smoothed": False,      # 后平滑后
        "aligned": True,            # 配准后
        "comparison": True,         # 原始vs配准后对比
    },
    "figsize": (18, 10),            # 图像大小
    "dpi": 100,                     # 分辨率
    "point_size": 1,                # 点大小
    "line_width": 0.5,              # 连线宽度
    "show_lines": True,             # 是否显示轨迹连线
    "alpha": 0.6,                   # 点透明度
    "save_figures": True,          # 是否保存图片
    "save_path": "./trajectory_stages.png",  # 保存路径
}

# ============================================================================
#                              数据加载
# ============================================================================

def load_csv(filepath, xyz_cols=(3, 4, 5), skip_header=1):
    """从CSV加载点云"""
    print(f"  加载: {os.path.basename(filepath)}")
    data = np.loadtxt(filepath, delimiter=',', skiprows=skip_header)
    points = data[:, xyz_cols]
    print(f"  点数: {len(points)}")
    return points, data


def load_csv_with_time(filepath, xyz_cols=(3, 4, 5), time_col=2, skip_header=1):
    """从CSV加载点云和时间戳"""
    print(f"  加载: {os.path.basename(filepath)}")
    data = np.loadtxt(filepath, delimiter=',', skiprows=skip_header)
    points = data[:, xyz_cols]
    times = data[:, time_col]
    print(f"  点数: {len(points)}, 时间范围: [{times.min():.2f}, {times.max():.2f}]s")
    return points, times


# ============================================================================
#                           ⭐ 时间对齐函数
# ============================================================================

def compute_arc_length(points):
    """计算累积弧长"""
    diffs = np.diff(points, axis=0)
    segment_lengths = np.linalg.norm(diffs, axis=1)
    arc_length = np.concatenate([[0], np.cumsum(segment_lengths)])
    return arc_length


def resample_by_arc_length(points, num_samples):
    """按弧长等距离重采样（传统方法，可能受噪声影响）"""
    arc_length = compute_arc_length(points)
    total_length = arc_length[-1]
    
    # 等距离采样点
    target_arc = np.linspace(0, total_length, num_samples)
    
    # 对每个坐标轴插值
    resampled = np.zeros((num_samples, 3))
    for i in range(3):
        f = interpolate.interp1d(arc_length, points[:, i], kind='linear')
        resampled[:, i] = f(target_arc)
    
    return resampled, total_length


def adaptive_spatial_resample(points, noise_level=0.5, noise_suppression_factor=2.0,
                               min_samples=500, max_samples=5000, verbose=True):
    """
    ⭐⭐ 旧版自适应空间重采样（基于弧长插值，需配合预平滑使用）
    
    根据轨迹长度和噪声水平自动确定最优采样步长，
    然后按固定步长在弧长轴上等距插值。
    
    ⚠️ 注意：此方法先计算累积弧长再插值，弧长本身受噪声影响，
    建议配合 PRE_SMOOTH 预平滑使用。如需消除海岸线悖论，
    请使用 chord_spatial_resample()。
    
    参数:
        points: Nx3 原始点云数组
        noise_level: 测量噪声水平（mm）
        noise_suppression_factor: 噪声抑制因子（1.5~2.0）
        min_samples: 最小采样点数
        max_samples: 最大采样点数
        verbose: 是否打印详细信息
    
    返回:
        resampled_points: Mx3 重采样点云
        sampling_info: dict, 包含采样统计信息
    """
    # 1. 计算累积弧长
    arc_lengths = compute_arc_length(points)
    total_length = arc_lengths[-1]
    
    # 2. 自适应计算最优步长
    optimal_delta_d = noise_suppression_factor * noise_level
    
    # 3. 计算采样点数（带边界约束）
    num_samples_calculated = int(total_length / optimal_delta_d)
    num_samples = max(min_samples, min(num_samples_calculated, max_samples))
    
    # 4. 实际步长（考虑边界约束后，linspace 含终点，故除以 M-1）
    actual_delta_d = total_length / (num_samples - 1) if num_samples > 1 else 0
    
    # 5. 生成等弧长采样位置（linspace：含起点 0 和终点 total_length）
    sample_arc_positions = np.linspace(0, total_length, num_samples)
    
    # 6. 线性插值（np.interp，端点截止，与复用端一致）
    resampled = np.zeros((num_samples, 3))
    for i in range(3):
        resampled[:, i] = np.interp(sample_arc_positions, arc_lengths, points[:, i])
    
    # 7. 统计信息
    sampling_info = {
        'original_points': len(points),
        'resampled_points': num_samples,
        'total_length_mm': total_length,
        'noise_level_mm': noise_level,
        'noise_suppression_factor': noise_suppression_factor,
        'optimal_delta_d_mm': optimal_delta_d,
        'actual_delta_d_mm': actual_delta_d,
        'delta_d_to_noise_ratio': actual_delta_d / noise_level if noise_level > 0 else 0,
        'sampling_efficiency': num_samples / len(points),
        'boundary_constrained': num_samples != num_samples_calculated,
    }
    
    if verbose:
        print(f"\n  【自适应空间重采样（旧版弧长插值）】")
        print(f"    原始点数:     {sampling_info['original_points']}")
        print(f"    轨迹长度:     {total_length:.2f} mm")
        print(f"    测量噪声:     {noise_level:.2f} mm")
        print(f"    抑制因子:     {noise_suppression_factor:.1f}×")
        print(f"    最优步长:     {optimal_delta_d:.3f} mm")

        if sampling_info['boundary_constrained']:
            constraint = "最小" if num_samples == min_samples else "最大"
            print(f"    ⚠️  触发{constraint}点数限制: {num_samples}")
        else:
            print(f"    ✅ 自适应点数:  {num_samples}")
        
        print(f"    实际步长:     {actual_delta_d:.3f} mm")
        print(f"    步长/噪声:    {sampling_info['delta_d_to_noise_ratio']:.2f}×")
        
        if sampling_info['delta_d_to_noise_ratio'] >= 1.5:
            print(f"    ✅ 抗噪性能:   优秀（步长 > 1.5×噪声）")
        elif sampling_info['delta_d_to_noise_ratio'] >= 1.0:
            print(f"    ⚠️  抗噪性能:   一般（步长 ≈ 噪声）")
        else:
            print(f"    ❌ 抗噪性能:   不足（步长 < 噪声，可能过采样）")
    
    return resampled, sampling_info


def dense_linear_interpolate(points, densify_factor=10):
    """
    ⭐ 密集线性插值：将轨迹点序列加密，使其近似连续
    
    在每对相邻点之间插入 densify_factor-1 个等距点，
    使得后续的欧氏距离行走步长更精确。
    
    参数:
        points: Nx3 原始轨迹点
        densify_factor: 加密倍数（每段插入 densify_factor-1 个点）
            - 10: 默认值（0.1mm级精度）
            - 20: 高精度（0.05mm级）
    
    返回:
        dense_points: Mx3 加密后的轨迹点 (M ≈ N × densify_factor)
    """
    N = len(points)
    if N < 2:
        return points.copy()
    
    # 预分配数组：(N-1) 段，每段 densify_factor 个点 + 最后一个端点
    total_dense = (N - 1) * densify_factor + 1
    dense_points = np.zeros((total_dense, 3))
    
    for seg_i in range(N - 1):
        p_start = points[seg_i]
        p_end = points[seg_i + 1]
        # 在 [p_start, p_end) 之间均匀插入
        for k in range(densify_factor):
            t = k / densify_factor
            dense_points[seg_i * densify_factor + k] = p_start + t * (p_end - p_start)
    
    # 最后一个端点
    dense_points[-1] = points[-1]
    
    return dense_points


def chord_spatial_resample(points, delta_d=1.0, densify_factor=10,
                            min_samples=500, max_samples=5000, verbose=True):
    """
    ⭐⭐⭐ 真3D空间弦距重采样（解决海岸线悖论）
    
    核心思路：
    =========
    1. 先对原始轨迹做密集线性插值，使其近似连续曲线
    2. 从第1个采样点出发，计算后续每个点到它的直线欧氏距离（弦距）
    3. 当弦距 ≥ Δd 时，采样该点作为新采样点
    4. 以新采样点为起点，重复步骤2-3
    
    关键区别（弦距 vs 路径距离）：
    =========
    - 路径距离：Σ||p[i]-p[i-1]|| — 沿轨迹逐段累加，噪声抖动全部计入
    - 弦距：||p[i] - p_last_sampled|| — 上一采样点到当前点的直线距离
      → 噪声造成的来回抖动不增加弦距，只有真正"走远了"才触发采样
    
    类比：拿圆规在海岸线上以 Δd 为半径画圆，找轨迹首次穿出圆的位置。
    
    与旧版弧长重采样的本质区别：
    =========
    - 旧版：先计算含噪声的累积弧长 L，再在 L 上等距插值
            → L 本身就被噪声污染，采样位置不准确
    - 新版：不预先计算总弧长，直接以弦距≥Δd为采样条件
            → 噪声造成的局部绕路不影响采样间隔
            → 采样后再计算累积弧长，得到的才是"真实弧长"
    
    参数:
        points: Nx3 原始轨迹点（未经预平滑的原始数据）
        delta_d: 采样步长(mm)（最小采样间隔门限）
        densify_factor: 插值加密倍数（10~20）
        min_samples: 最小采样点数（保证特征分辨率）
        max_samples: 最大采样点数（控制计算成本）
        verbose: 是否打印详细信息
    
    返回:
        resampled_points: Mx3 重采样后的轨迹点
        sampling_info: dict, 包含采样统计信息
    """
    N_original = len(points)
    
    # ========== Step 1: 密集线性插值 ==========
    dense_points = dense_linear_interpolate(points, densify_factor=densify_factor)
    N_dense = len(dense_points)
    
    # ========== Step 2: 弦距离行走采样 ==========
    # 第1个点作为第1个采样点
    sampled_indices = [0]  # 记录在 dense_points 中的索引
    last_sampled_idx = 0
    
    for i in range(1, N_dense):
        # ⭐ 计算当前点到上一个采样点的直线欧氏距离（弦距）
        #    而非沿轨迹累积的路径距离，噪声造成的来回抖动不会被计入
        chord_dist = np.linalg.norm(dense_points[i] - dense_points[last_sampled_idx])
        
        # 弦距 ≥ Δd 时，采样当前点
        if chord_dist >= delta_d:
            sampled_indices.append(i)
            last_sampled_idx = i
    
    # 确保最后一个点被采样（轨迹终点）
    if sampled_indices[-1] != N_dense - 1:
        sampled_indices.append(N_dense - 1)
    
    resampled = dense_points[sampled_indices]
    
    # ========== Step 3: 边界约束检查 ==========
    boundary_constrained = False
    constraint_type = None
    
    # 内部辅助函数：用弦距重新采样
    def _chord_resample(dense_pts, step):
        indices = [0]
        last_idx = 0
        for j in range(1, len(dense_pts)):
            if np.linalg.norm(dense_pts[j] - dense_pts[last_idx]) >= step:
                indices.append(j)
                last_idx = j
        if indices[-1] != len(dense_pts) - 1:
            indices.append(len(dense_pts) - 1)
        return indices
    
    if len(resampled) < min_samples:
        # 点数不足：缩小步长重新采样
        boundary_constrained = True
        constraint_type = "最小"
        rough_length = np.sum(np.linalg.norm(np.diff(resampled, axis=0), axis=1))
        delta_d_adjusted = rough_length / min_samples
        sampled_indices = _chord_resample(dense_points, delta_d_adjusted)
        resampled = dense_points[sampled_indices]
        delta_d = delta_d_adjusted
    
    elif len(resampled) > max_samples:
        # 点数过多：增大步长重新采样
        boundary_constrained = True
        constraint_type = "最大"
        rough_length = np.sum(np.linalg.norm(np.diff(resampled, axis=0), axis=1))
        delta_d_adjusted = rough_length / max_samples
        sampled_indices = _chord_resample(dense_points, delta_d_adjusted)
        resampled = dense_points[sampled_indices]
        delta_d = delta_d_adjusted
    
    # ========== Step 4: 计算真实弧长（基于重采样点）==========
    true_arc_length = np.sum(np.linalg.norm(np.diff(resampled, axis=0), axis=1))
    
    # 对比：原始含噪声弧长
    raw_arc_length = np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1))
    noise_arc_overestimate = (raw_arc_length - true_arc_length) / true_arc_length * 100
    
    # 实际平均步长
    actual_steps = np.linalg.norm(np.diff(resampled, axis=0), axis=1)
    actual_delta_d = np.mean(actual_steps)
    
    # ========== Step 5: 统计信息 ==========
    sampling_info = {
        'original_points': N_original,
        'dense_points': N_dense,
        'resampled_points': len(resampled),
        'total_length_mm': true_arc_length,       # 真实弧长（基于重采样点）
        'raw_arc_length_mm': raw_arc_length,       # 原始含噪声弧长
        'noise_arc_overestimate_pct': noise_arc_overestimate,
        'delta_d_mm': delta_d,
        'actual_delta_d_mm': actual_delta_d,
        'actual_delta_d_std': np.std(actual_steps),
        'densify_factor': densify_factor,
        'sampling_efficiency': len(resampled) / N_original,
        'boundary_constrained': boundary_constrained,
    }
    
    if verbose:
        print(f"\n  【真3D空间欧氏距离重采样】")
        print(f"    原始点数:       {N_original}")
        print(f"    加密倍数:       {densify_factor}× → {N_dense} 个密集点")
        print(f"    采样步长 Δd:    {delta_d:.3f} mm")
        
        if boundary_constrained:
            print(f"    ⚠️  触发{constraint_type}点数限制，调整步长为 {delta_d:.3f} mm")
        
        print(f"    采样点数:       {len(resampled)}")
        print(f"    实际平均步长:   {actual_delta_d:.3f} ± {np.std(actual_steps):.3f} mm")
        print(f"    真实弧长:       {true_arc_length:.2f} mm")
        print(f"    原始含噪弧长:   {raw_arc_length:.2f} mm")
        print(f"    噪声弧长高估:   {noise_arc_overestimate:.2f}%")
        
        if noise_arc_overestimate > 2:
            print(f"    ✅ 海岸线效应消除: 剔除了 {noise_arc_overestimate:.2f}% 的虚假弧长")
        else:
            print(f"    ✅ 噪声影响较小（高估 < 2%）")
    
    return resampled, sampling_info


def dtw_align(source, target):
    """
    DTW动态时间规整对齐
    找到最优的点对应关系
    """
    n, m = len(source), len(target)
    
    # 计算距离矩阵
    dist_matrix = np.zeros((n, m))
    for i in range(n):
        dist_matrix[i] = np.linalg.norm(target - source[i], axis=1)
    
    # DTW累积代价矩阵
    dtw = np.full((n + 1, m + 1), np.inf)
    dtw[0, 0] = 0
    
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            cost = dist_matrix[i-1, j-1]
            dtw[i, j] = cost + min(dtw[i-1, j], dtw[i, j-1], dtw[i-1, j-1])
    
    # 回溯找路径
    path = []
    i, j = n, m
    while i > 0 and j > 0:
        path.append((i-1, j-1))
        candidates = [
            (i-1, j-1, dtw[i-1, j-1]),
            (i-1, j, dtw[i-1, j]),
            (i, j-1, dtw[i, j-1])
        ]
        i, j, _ = min(candidates, key=lambda x: x[2])
    
    path.reverse()
    
    # 根据DTW路径提取对齐的点对
    src_indices = [p[0] for p in path]
    tgt_indices = [p[1] for p in path]
    
    # 去重：每个源点只对应一个目标点
    aligned_src = []
    aligned_tgt = []
    seen_src = set()
    
    for si, ti in zip(src_indices, tgt_indices):
        if si not in seen_src:
            aligned_src.append(source[si])
            aligned_tgt.append(target[ti])
            seen_src.add(si)
    
    return np.array(aligned_src), np.array(aligned_tgt)


def align_trajectories(source, target, params):
    """轨迹时间对齐主函数"""
    if not params["enable"]:
        # 不对齐，直接截断
        min_len = min(len(source), len(target))
        return source[:min_len], target[:min_len]
    
    method = params["method"]
    
    if method == "chord_spatial":
        # ⭐⭐⭐ 新版：真3D弦距重采样（密集插值+弦距采样，解决海岸线悖论）
        print(f"  ⭐ 使用真3D弦距重采样（chord_spatial）...")
        
        # 获取配置参数
        delta_d = ADAPTIVE_SPATIAL_RESAMPLE["delta_d"]
        densify_factor = ADAPTIVE_SPATIAL_RESAMPLE["densify_factor"]
        min_samples = ADAPTIVE_SPATIAL_RESAMPLE["min_samples"]
        max_samples = ADAPTIVE_SPATIAL_RESAMPLE["max_samples"]
        
        # 源轨迹重采样
        print(f"  源轨迹:")
        src_resampled, src_info = chord_spatial_resample(
            source,
            delta_d=delta_d,
            densify_factor=densify_factor,
            min_samples=min_samples,
            max_samples=max_samples,
            verbose=True
        )
        
        # 目标轨迹重采样
        print(f"  目标轨迹:")
        tgt_resampled, tgt_info = chord_spatial_resample(
            target,
            delta_d=delta_d,
            densify_factor=densify_factor,
            min_samples=min_samples,
            max_samples=max_samples,
            verbose=True
        )
        
        # 打印对比信息
        print(f"\n  【采样对比】")
        print(f"    源轨迹: {src_info['original_points']}点 → {src_info['resampled_points']}点")
        print(f"    目标:   {tgt_info['original_points']}点 → {tgt_info['resampled_points']}点")
        print(f"    源真实弧长: {src_info['total_length_mm']:.2f}mm（原始含噪: {src_info['raw_arc_length_mm']:.2f}mm）")
        print(f"    目标真实弧长: {tgt_info['total_length_mm']:.2f}mm（原始含噪: {tgt_info['raw_arc_length_mm']:.2f}mm）")
        print(f"    长度比: {src_info['total_length_mm']/tgt_info['total_length_mm']:.4f}")
        
        # 如果两条轨迹采样点数不同，需要对齐
        if src_info['resampled_points'] != tgt_info['resampled_points']:
            print(f"\n  ⚠️  采样点数不一致，进行弧长归一化对齐...")
            common_samples = min(src_info['resampled_points'], tgt_info['resampled_points'])
            src_resampled, _ = resample_by_arc_length(src_resampled, common_samples)
            tgt_resampled, _ = resample_by_arc_length(tgt_resampled, common_samples)
            print(f"    对齐后点数: {common_samples}")
        
        return src_resampled, tgt_resampled
    
    elif method == "adaptive_spatial":
        # ⭐⭐ 旧版：自适应弧长重采样（需配合预平滑）
        print(f"  使用自适应空间重采样（旧版弧长插值）...")
        
        noise_level = ADAPTIVE_SPATIAL_RESAMPLE["noise_level"]
        noise_factor = ADAPTIVE_SPATIAL_RESAMPLE["noise_suppression_factor"]
        min_samples = ADAPTIVE_SPATIAL_RESAMPLE["min_samples"]
        max_samples = ADAPTIVE_SPATIAL_RESAMPLE["max_samples"]
        
        # 源轨迹自适应重采样
        print(f"  源轨迹:")
        src_resampled, src_info = adaptive_spatial_resample(
            source,
            noise_level=noise_level,
            noise_suppression_factor=noise_factor,
            min_samples=min_samples,
            max_samples=max_samples,
            verbose=True
        )
        
        # 目标轨迹自适应重采样
        print(f"  目标轨迹:")
        tgt_resampled, tgt_info = adaptive_spatial_resample(
            target,
            noise_level=noise_level,
            noise_suppression_factor=noise_factor,
            min_samples=min_samples,
            max_samples=max_samples,
            verbose=True
        )
        
        # 打印对比信息
        print(f"\n  【采样对比】")
        print(f"    源轨迹: {src_info['original_points']}点 → {src_info['resampled_points']}点")
        print(f"    目标:   {tgt_info['original_points']}点 → {tgt_info['resampled_points']}点")
        print(f"    源弧长: {src_info['total_length_mm']:.2f}mm, 步长: {src_info['actual_delta_d_mm']:.3f}mm")
        print(f"    目标弧长: {tgt_info['total_length_mm']:.2f}mm, 步长: {tgt_info['actual_delta_d_mm']:.3f}mm")
        print(f"    长度比: {src_info['total_length_mm']/tgt_info['total_length_mm']:.4f}")
        
        # 如果两条轨迹采样点数不同，需要对齐
        if src_info['resampled_points'] != tgt_info['resampled_points']:
            print(f"\n  ⚠️  采样点数不一致，进行弧长归一化对齐...")
            common_samples = min(src_info['resampled_points'], tgt_info['resampled_points'])
            src_resampled, _ = resample_by_arc_length(src_resampled, common_samples)
            tgt_resampled, _ = resample_by_arc_length(tgt_resampled, common_samples)
            print(f"    对齐后点数: {common_samples}")
        
        return src_resampled, tgt_resampled
    
    elif method == "arc_length":
        # 传统弧长重采样对齐（可能受噪声影响）
        num_samples = params["num_samples"]
        print(f"  传统弧长重采样: {num_samples} 点")
        print(f"  ⚠️  注意：此方法可能受噪声影响，建议使用 chord_spatial")
        
        src_resampled, src_len = resample_by_arc_length(source, num_samples)
        tgt_resampled, tgt_len = resample_by_arc_length(target, num_samples)
        
        print(f"  源轨迹长度: {src_len:.2f}, 目标轨迹长度: {tgt_len:.2f}")
        print(f"  长度比: {src_len/tgt_len:.4f}")
        
        return src_resampled, tgt_resampled
    
    elif method == "dtw":
        # DTW对齐
        print(f"  DTW动态时间规整...")
        aligned_src, aligned_tgt = dtw_align(source, target)
        print(f"  DTW对齐后点数: {len(aligned_src)}")
        return aligned_src, aligned_tgt
    
    else:
        raise ValueError(f"未知对齐方法: {method}，支持: chord_spatial, adaptive_spatial, arc_length, dtw")


# ============================================================================
#                              预处理
# ============================================================================

def smooth_gaussian(points, sigma=3.0):
    """
    空间几何高斯平滑

    以弧长距离（mm）为高斯核参数，对每个点按其与邻居的实际空间距离
    进行加权平均，而非基于点序号索引。

    相比逐轴独立1D卷积的优势：
      - sigma 单位为 mm，与采样率、运动速度无关
      - 轨迹快慢段平滑强度一致（慢速段点密集时不会被过度平滑）
      - XYZ 三轴共享同一套权重，保证几何一致性

    参数:
        points: (N, 3) 轨迹点数组
        sigma:  高斯核半宽（mm），权重衰减到 e^{-0.5} ≈ 60% 时对应的弧长距离
                典型值：预平滑用 3~8mm，后平滑用 2~5mm

    返回:
        smoothed: (N, 3) 平滑后的轨迹点，点数与输入相同
    """
    N = len(points)
    if N < 2:
        return points.copy()

    # 计算累积弧长（mm）
    arc = np.empty(N)
    arc[0] = 0.0
    arc[1:] = np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))

    # 截断半径：3σ 之外权重 < 0.01，直接忽略
    cutoff = 3.0 * sigma
    sigma2_x2 = 2.0 * sigma * sigma

    smoothed = np.empty_like(points)
    for i in range(N):
        s_i = arc[i]
        # 二分查找窗口边界（在有序弧长数组上 O(log N)）
        lo = np.searchsorted(arc, s_i - cutoff, side='left')
        hi = np.searchsorted(arc, s_i + cutoff, side='right')

        ds = arc[lo:hi] - s_i                    # 弧长偏差（mm）
        weights = np.exp(-ds * ds / sigma2_x2)   # 高斯权重
        w_sum = weights.sum()

        # 三轴联合加权平均
        smoothed[i] = (weights[:, np.newaxis] * points[lo:hi]).sum(axis=0) / w_sum

    return smoothed


def smooth_bspline(points, s=5.0, k=3, num_samples=None):
    """B-Spline平滑"""
    N = len(points)
    if num_samples is None:
        num_samples = N
    
    t = np.linspace(0, 1, N)
    
    try:
        tck, _ = interpolate.splprep(points.T, u=t, s=s, k=k)
        u_new = np.linspace(0, 1, num_samples)
        new_points = interpolate.splev(u_new, tck)
        return np.array(new_points).T
    except Exception as e:
        print(f"  B-Spline失败: {e}, 使用高斯平滑代替")
        return smooth_gaussian(points, sigma=3.0)


def _point_line_distance_3d(point, start, end):
    """计算3D空间中点到线段(start→end)的距离"""
    if np.allclose(start, end):
        return np.linalg.norm(point - start)
    line_vec = end - start
    point_vec = point - start
    cross = np.cross(line_vec, point_vec)
    return np.linalg.norm(cross) / np.linalg.norm(line_vec)


def _geometric_median(pts, tol=1e-6, max_iter=100):
    """
    Weiszfeld 算法求空间几何中值（L1-最小化中心）。
    返回使所有输入点到该点欧式距离之和最小的 3D 点。
    """
    if len(pts) == 1:
        return pts[0].copy()
    estimate = np.mean(pts, axis=0)
    for _ in range(max_iter):
        dists = np.linalg.norm(pts - estimate, axis=1)
        dists = np.maximum(dists, 1e-10)   # 避免除以零
        weights = 1.0 / dists
        new_estimate = np.average(pts, weights=weights, axis=0)
        if np.linalg.norm(new_estimate - estimate) < tol:
            break
        estimate = new_estimate
    return estimate


def median_filter_3d(points, window_size=5):
    """
    3D 轨迹空间几何中值滤波，移除离群点（Outliers/Flyers）。
    对每个点取其滑动窗口内的三维邻域点，用 Weiszfeld L1 几何中值代替原始点，
    按空间欧式距离整体滤波，避免各轴独立处理导致的方向偏差。

    参数:
        points      : (N,3) 原始轨迹点
        window_size : 滑动窗口大小（奇数），越大对离群点抑制越强
    """
    if window_size < 3:
        return points

    # 确保窗口大小为奇数
    if window_size % 2 == 0:
        window_size += 1

    N = len(points)
    result = np.zeros_like(points)
    half = window_size // 2

    for i in range(N):
        start = max(0, i - half)
        end   = min(N, i + half + 1)
        result[i] = _geometric_median(points[start:end])

    return result


def douglas_peucker_3d(points, epsilon):
    """
    Douglas-Peucker 3D曲线简化算法（提取骨干点）
    递归地移除距首尾连线距离 < epsilon 的中间点，
    保留轨迹中具有几何意义的关键转折点。
    """
    if len(points) <= 2:
        return points

    start, end = points[0], points[-1]
    distances = np.array([_point_line_distance_3d(p, start, end) for p in points])
    max_idx = np.argmax(distances)
    max_dist = distances[max_idx]

    if max_dist > epsilon:
        left = douglas_peucker_3d(points[:max_idx + 1], epsilon)
        right = douglas_peucker_3d(points[max_idx:], epsilon)
        return np.vstack([left[:-1], right])
    else:
        return np.array([start, end])


def smooth_rdp_pchip(points, epsilon=0.5, median_window=5):
    """
    中值滤波 + RDP骨干提取 + PCHIP保形插值重建
    Step 0: 中值滤波移除离群点，防止RDP设置不必要的关键帧
    Step 1: Douglas-Peucker 剔除高频微抖动，提取轨迹骨干关键点
    Step 2: PCHIP 对骨干点进行保形插值，重建为原始点数的紧致曲线
    
    ⭐ 改进说明：
    - 中值滤波可有效抑制孤立噪声峰值（离群点），防止RDP误判关键帧
    - 原B-Spline在骨干点稀疏区域会产生过冲（龙格现象）
    - PCHIP保形插值确保：通过所有控制点、保持单调性、无额外极值点
    - 生成的曲线更紧贴原始轨迹，不会大幅度偏离

    参数:
        points        : (N,3) 原始轨迹点
        epsilon       : RDP简化阈值(mm)，越大简化越激进
        median_window : 中值滤波窗口大小，越大对离群点抑制越强（建议3-9奇数）
    """
    N = len(points)
    print(f"  RDP+PCHIP: 原始点数={N}, epsilon={epsilon}, 中值窗口={median_window}")

    # Step 0: 中值滤波预处理，移除离群点
    if median_window >= 3:
        filtered = median_filter_3d(points, median_window)
        print(f"  中值滤波: 窗口={median_window}, 离群点抑制完成")
    else:
        filtered = points

    # Step 1: RDP提取骨干
    backbone = douglas_peucker_3d(filtered, epsilon)
    print(f"  RDP骨干提取: {N} -> {len(backbone)} 点 (保留{len(backbone)/N*100:.1f}%)")

    # 骨干点不足时降级为高斯平滑
    if len(backbone) < 2:
        print(f"  ⚠️ 骨干点不足({len(backbone)}<2)，降级为高斯平滑")
        return smooth_gaussian(points, sigma=3.0)

    # Step 2: PCHIP保形插值回原始点数
    # 使用累积弧长（欧式距离）作为插值参数，保证参数与空间位置对应
    backbone_dists = np.linalg.norm(np.diff(backbone, axis=0), axis=1)
    t_backbone = np.concatenate([[0.0], np.cumsum(backbone_dists)])
    total_len = t_backbone[-1]
    if total_len < 1e-10:
        return smooth_gaussian(points, sigma=3.0)
    t_backbone /= total_len  # 归一化到 [0, 1]

    # 原始点的累积弧长作为查询参数（保留原始点空间分布）
    pts_dists = np.linalg.norm(np.diff(points, axis=0), axis=1)
    t_query = np.concatenate([[0.0], np.cumsum(pts_dists)])
    pts_total = t_query[-1]
    if pts_total < 1e-10:
        t_query = np.linspace(0.0, 1.0, N)
    else:
        t_query /= pts_total  # 归一化到 [0, 1]

    try:
        result = np.zeros((N, 3))
        for dim in range(3):
            # 以骨干点弧长为控制参数，原始点弧长为查询参数进行保形插值
            pchip = interpolate.PchipInterpolator(t_backbone, backbone[:, dim])
            result[:, dim] = pchip(t_query)

        print(f"  PCHIP弧长插值: {len(backbone)} -> {N} 点（弧长参数化）")
        return result
    except Exception as e:
        print(f"  PCHIP插值失败: {e}, 使用高斯平滑代替")
        return smooth_gaussian(points, sigma=3.0)


def preprocess(points, params):
    """预处理流程"""
    result = points.copy()
    
    # 下采样
    if params["downsample_ratio"] < 1.0:
        n = int(len(result) * params["downsample_ratio"])
        indices = np.linspace(0, len(result)-1, n, dtype=int)
        result = result[indices]
        print(f"  下采样: {len(points)} -> {len(result)}")
    
    # 平滑
    if params["enable_smoothing"]:
        if params["smoothing_method"] == "gaussian":
            result = smooth_gaussian(result, params["gaussian_sigma"])
            print(f"  高斯平滑: sigma={params['gaussian_sigma']}")
        else:
            result = smooth_bspline(result, params["bspline_smoothing"], params["bspline_k"])
            print(f"  B-Spline平滑: s={params['bspline_smoothing']}, k={params['bspline_k']}")
    
    return result


# ============================================================================
#                              配准算法
# ============================================================================

def kabsch_align(source, target):
    """Kabsch算法（SVD刚性配准）- 基础版本，无异常值处理"""
    assert len(source) == len(target), "点数必须相同！"
    
    src_center = np.mean(source, axis=0)
    tgt_center = np.mean(target, axis=0)
    
    src_centered = source - src_center
    tgt_centered = target - tgt_center
    
    H = src_centered.T @ tgt_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    t = tgt_center - R @ src_center
    
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    
    return T


def kabsch_align_robust(source, target, ransac_params=None, verbose=False):
    """
    ⭐⭐⭐ RANSAC-Kabsch鲁棒配准
    
    核心思想：
    =========
    在SVD分解【之前】通过随机采样投票机制识别并剔除异常值（飞点）
    
    数学原理：
    =========
    传统Kabsch：E = Σ ||Rp_i + t - q_i||²（平方放大异常值影响）
    RANSAC：   多数派投票，少数异常值不参与最终计算
    
    算法流程：
    =========
    1. 随机采样：从N个点中随机选3个点（最小配准集）
    2. 假设模型：用这3个点计算临时变换矩阵 T_temp
    3. 验证模型：用 T_temp 测试所有点，统计内点数量
    4. 循环迭代：重复100次，选出内点最多的模型
    5. 最终优化：只使用内点重新运行精确Kabsch配准
    
    参数:
        source: Nx3 源点云
        target: Nx3 目标点云
        ransac_params: RANSAC参数字典
            - max_iterations: 迭代次数（默认100）
            - inlier_threshold: 内点阈值(mm)（默认0.5）
            - min_inlier_ratio: 最小内点比例（默认0.5）
            - min_sample_points: 最小采样点数（默认3）
        verbose: 是否打印详细信息
    
    返回:
        T: 4x4变换矩阵（只基于内点计算）
        inliers_mask: 布尔数组，标记哪些点是内点
    
    异常处理：
    =========
    如果内点比例 < min_inlier_ratio，退回到传统Kabsch（全点）
    """
    # 默认参数
    if ransac_params is None:
        ransac_params = RANSAC_KABSCH_PARAMS
    
    max_iterations = ransac_params.get("max_iterations", 100)
    inlier_threshold = ransac_params.get("inlier_threshold", 0.5)
    min_inlier_ratio = ransac_params.get("min_inlier_ratio", 0.5)
    min_sample_points = ransac_params.get("min_sample_points", 3)
    
    N = len(source)
    assert N == len(target), "源点云和目标点云点数必须相同！"
    
    # 如果点数太少，直接使用传统Kabsch
    if N < min_sample_points:
        if verbose:
            print(f"    [RANSAC] 点数({N})不足，使用传统Kabsch")
        T = kabsch_align(source, target)
        return T, np.ones(N, dtype=bool)
    
    # RANSAC循环
    best_inliers_mask = np.zeros(N, dtype=bool)
    best_inlier_count = 0
    best_T = np.eye(4)
    
    for iteration in range(max_iterations):
        # Step 1: 随机采样3个点
        sample_indices = np.random.choice(N, min_sample_points, replace=False)
        sample_src = source[sample_indices]
        sample_tgt = target[sample_indices]
        
        # Step 2: 用采样点计算临时变换矩阵
        try:
            T_temp = kabsch_align(sample_src, sample_tgt)
        except:
            continue  # 采样点可能共线，跳过此次迭代
        
        # Step 3: 验证所有点
        src_homo = np.hstack([source, np.ones((N, 1))])
        transformed = (T_temp @ src_homo.T).T[:, :3]
        errors = np.linalg.norm(transformed - target, axis=1)
        
        # Step 4: 统计内点
        inliers_mask = errors < inlier_threshold
        inlier_count = np.sum(inliers_mask)
        
        # Step 5: 更新最佳模型
        if inlier_count > best_inlier_count:
            best_inlier_count = inlier_count
            best_inliers_mask = inliers_mask
            best_T = T_temp
    
    # 计算内点比例
    inlier_ratio = best_inlier_count / N
    
    # 如果内点比例过低，退回到传统Kabsch
    if inlier_ratio < min_inlier_ratio:
        if verbose:
            print(f"    [RANSAC] 内点比例({inlier_ratio:.1%})过低，退回传统Kabsch")
        T = kabsch_align(source, target)
        return T, np.ones(N, dtype=bool)
    
    # Step 6: 用所有内点重新精确计算最终变换
    final_src = source[best_inliers_mask]
    final_tgt = target[best_inliers_mask]
    T_final = kabsch_align(final_src, final_tgt)
    
    if verbose:
        outlier_count = N - best_inlier_count
        print(f"    [RANSAC] 内点:{best_inlier_count}/{N} ({inlier_ratio:.1%}), "
              f"异常值:{outlier_count} ({100*(1-inlier_ratio):.1f}%)")
    
    return T_final, best_inliers_mask


def weighted_kabsch(source, target, weights):
    """
    ⭐⭐⭐ 加权Kabsch算法（MLS核心计算单元）
    
    与标准Kabsch的唯一区别是所有求和都带权重：
    - 加权质心: c = Σ w_i * p_i
    - 加权协方差: H = Σ w_i * (p_i - c_src)^T * (q_i - c_tgt)
    
    参数:
        source:  Nx3 源点云
        target:  Nx3 目标点云
        weights: N 维权重向量（非负）
    
    返回:
        T: 4×4 刚性变换矩阵
    """
    # 归一化权重
    W = weights / (weights.sum() + 1e-10)
    
    # Step 1: 加权质心
    src_center = np.sum(W[:, None] * source, axis=0)
    tgt_center = np.sum(W[:, None] * target, axis=0)
    
    # Step 2: 中心化
    src_centered = source - src_center
    tgt_centered = target - tgt_center
    
    # Step 3: 加权协方差矩阵 H = (W * src_centered)^T @ tgt_centered
    H = (W[:, None] * src_centered).T @ tgt_centered
    
    # Step 4: SVD
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    # 确保正交矩阵（det(R) = +1）
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # Step 5: 平移
    t = tgt_center - R @ src_center
    
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    
    return T


def kabsch_icp_align(source, target, icp_params):
    """
    ⭐ Kabsch + ICP 混合精配准
    先用Kabsch得到粗略对齐，再用ICP精细优化
    """
    print("  第1阶段: Kabsch粗配准...")
    T_kabsch = kabsch_align(source, target)
    
    # 应用Kabsch变换
    N = len(source)
    src_homo = np.hstack([source, np.ones((N, 1))])
    src_transformed = (T_kabsch @ src_homo.T).T[:, :3]
    
    # 计算Kabsch后误差
    kabsch_errors = np.linalg.norm(src_transformed - target, axis=1)
    print(f"    Kabsch后RMSE: {np.sqrt(np.mean(kabsch_errors**2)):.4f}")
    
    print("  第2阶段: ICP精配准...")
    T_icp = icp_align(src_transformed, target, icp_params, init_T=np.eye(4))
    
    # 组合变换: T_final = T_icp @ T_kabsch
    T_final = T_icp @ T_kabsch
    
    return T_final


# ============================================================================
#              ⭐⭐ 分段配准（支持导出和应用到新数据）
# ============================================================================

class SegmentedTransform:
    """
    分段变换类 - 存储和应用分段配准结果（支持自适应细分）
    
    数学表示：
    ============
    分段配准将轨迹分成N段，每段有独立的刚性变换 T_i = [R_i | t_i]
    
    对于弧长位置 s ∈ [0, L] 的点 P：
    1. 找到 s 所在的分段 i 和相邻分段 i+1
    2. 计算权重 w = (s - s_i) / (s_{i+1} - s_i)
    3. 插值变换: P' = (1-w) * T_i(P) + w * T_{i+1}(P)
    
    自适应细分：
    ============
    对于RMSE > refine_threshold的分段，递归细分为更小的子段
    每层细分将段一分为二，直到达到max_refine_depth或误差满足要求
    
    这实现了沿轨迹的**连续非刚性变换**，并在高误差区域提供更高精度
    """
    
    def __init__(self):
        self.segment_transforms = []  # 每段的变换矩阵 T_i
        self.segment_centers = []     # 每段的弧长中心位置
        self.segment_ranges = []      # 每段的弧长范围 [start, end]
        self.segment_rmses = []       # ⭐ 每段的RMSE
        self.segment_depths = []      # ⭐ 每段的细分深度（0=初始层）
        self.total_arc_length = 0     # 总弧长
        self.num_segments = 0
        self.source_arc_lengths = None  # 源点云的弧长参数
        self.refine_stats = {}        # ⭐ 自适应细分统计信息
        # ⭐⭐⭐ 序列号对应模式属性
        self.mode = "arc_length"          # "arc_length" 或 "sequence_aligned"
        self.total_frames = 0             # 总帧数（序列号模式）
        self.segment_center_frames = []   # 每段中心帧号
        self.segment_frame_ranges = []    # 每段帧号范围 [(start, end), ...]
        
    def save(self, filepath, original_arc_length=None, mode=None):
        """
        保存分段变换到JSON（支持弧长模式和序列号模式）
        
        参数:
            filepath: 保存路径
            original_arc_length: 原始数据的弧长（弧长模式使用）
            mode: 保存模式，None=自动检测self.mode
        """
        save_mode = mode if mode else self.mode
        
        if save_mode == "sequence_aligned":
            return self._save_sequence_mode(filepath)
        else:
            return self._save_arc_length_mode(filepath, original_arc_length)
    
    def _save_sequence_mode(self, filepath):
        """⭐⭐⭐ 序列号对应模式的JSON保存"""
        data = {
            "description": "分段Kabsch配准结果 - 序列号对应模式",
            "mode": "sequence_aligned",
            "math_formula": "P'[i] = Σ w_j(i) * T_j * P[i]，其中i为帧号，w_j基于帧号距离的高斯权重",
            "num_segments": self.num_segments,
            "total_frames": self.total_frames,
            "total_arc_length": self.total_arc_length,  # ⭐ 兼容性字段（帧数作为等价弧长）
            "adaptive_refine_stats": self.refine_stats,
            "segments": []
        }
        
        for i, T in enumerate(self.segment_transforms):
            depth = self.segment_depths[i] if i < len(self.segment_depths) else 0
            rmse = self.segment_rmses[i] if i < len(self.segment_rmses) else 0.0
            center_frame = self.segment_center_frames[i] if i < len(self.segment_center_frames) else 0
            frame_range = self.segment_frame_ranges[i] if i < len(self.segment_frame_ranges) else (0, 0)
            
            seg_data = {
                "index": i,
                "center_frame": int(center_frame),
                "frame_range": [int(frame_range[0]), int(frame_range[1])],
                "refine_depth": depth,
                "segment_rmse": rmse,
                "transform_4x4": T.tolist(),
                "rotation_3x3": T[:3, :3].tolist(),
                "translation": T[:3, 3].tolist(),
            }
            data["segments"].append(seg_data)
        
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        
        print(f"  ⭐ 序列号对应模式 - 分段变换已保存: {filepath}")
        print(f"  共 {self.num_segments} 段, 总帧数 {self.total_frames}")
        if self.refine_stats:
            print(f"  细分统计: {self.refine_stats}")
        return filepath
    
    def _save_arc_length_mode(self, filepath, original_arc_length=None):
        data = {
            "description": "分段Kabsch配准结果 - 非刚性变换（支持自适应细分，归一化弧长空间）",
            "math_formula": "P'(s̃) = Σ w_i(s̃) * T_i * P，其中 s̃=s/L ∈[0,1], w_i 是归一化弧长空间的高斯权重",
            "num_segments": self.num_segments,
            "total_arc_length": self.total_arc_length,
            "original_arc_length": original_arc_length if original_arc_length else self.total_arc_length,
            "note": "⭐ 使用归一化弧长空间权重计算，与复用时完全一致",
            "adaptive_refine_stats": self.refine_stats,  # ⭐ 自适应细分统计
            "segments": []
        }
        
        for i, (T, center, (start, end)) in enumerate(zip(
                self.segment_transforms, self.segment_centers, self.segment_ranges)):
            # 获取细分深度和RMSE（如果有）
            depth = self.segment_depths[i] if i < len(self.segment_depths) else 0
            rmse = self.segment_rmses[i] if i < len(self.segment_rmses) else 0.0
            
            # ⭐⭐⭐ 计算并保存归一化弧长（确保与复用时100%一致）
            total_arc = self.total_arc_length
            normalized_center = center / total_arc if total_arc > 0 else 0
            normalized_start = start / total_arc if total_arc > 0 else 0
            normalized_end = end / total_arc if total_arc > 0 else 1
            
            seg_data = {
                "index": i,
                "arc_length_center": center,
                "arc_length_range": [start, end],
                "normalized_arc_center": normalized_center,      # ⭐⭐⭐ 归一化弧长中心
                "normalized_arc_range": [normalized_start, normalized_end],  # ⭐⭐⭐ 归一化弧长范围
                "refine_depth": depth,           # ⭐ 细分深度
                "segment_rmse": rmse,            # ⭐ 分段RMSE
                "transform_4x4": T.tolist(),
                "rotation_3x3": T[:3, :3].tolist(),
                "translation": T[:3, 3].tolist(),
            }
            data["segments"].append(seg_data)
        
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        
        print(f"  分段变换已保存: {filepath}")
        print(f"  共 {self.num_segments} 段（归一化弧长空间权重）")
        if self.refine_stats:
            print(f"  细分统计: {self.refine_stats}")
        if original_arc_length:
            print(f"  原始弧长: {original_arc_length:.2f}")
        return filepath
    
    @classmethod
    def load(cls, filepath):
        """从JSON加载分段变换（支持弧长模式和序列号模式）"""
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        obj = cls()
        obj.num_segments = data["num_segments"]
        obj.refine_stats = data.get("adaptive_refine_stats", {})
        obj.mode = data.get("mode", "arc_length")
        
        if obj.mode == "sequence_aligned":
            # ⭐⭐⭐ 序列号对应模式
            obj.total_frames = data["total_frames"]
            obj.total_arc_length = data.get("total_arc_length", float(obj.total_frames))
            
            for seg in data["segments"]:
                obj.segment_transforms.append(np.array(seg["transform_4x4"]))
                obj.segment_center_frames.append(seg["center_frame"])
                obj.segment_frame_ranges.append(tuple(seg["frame_range"]))
                obj.segment_depths.append(seg.get("refine_depth", 0))
                obj.segment_rmses.append(seg.get("segment_rmse", 0.0))
                # 兼容弧长字段
                obj.segment_centers.append(float(seg["center_frame"]))
                obj.segment_ranges.append((float(seg["frame_range"][0]), float(seg["frame_range"][1])))
            
            print(f"  ⭐ 已加载序列号对应模式变换: {filepath}")
            print(f"  共 {obj.num_segments} 段, 总帧数 {obj.total_frames}")
        else:
            # 弧长对应模式
            obj.total_arc_length = data["total_arc_length"]
            
            for seg in data["segments"]:
                obj.segment_transforms.append(np.array(seg["transform_4x4"]))
                obj.segment_centers.append(seg["arc_length_center"])
                obj.segment_ranges.append(tuple(seg["arc_length_range"]))
                obj.segment_depths.append(seg.get("refine_depth", 0))
                obj.segment_rmses.append(seg.get("segment_rmse", 0.0))
            
            print(f"  已加载分段变换: {filepath}")
            print(f"  共 {obj.num_segments} 段")
        
        if obj.refine_stats:
            print(f"  细分统计: {obj.refine_stats}")
        return obj
    
    def transform_point(self, point, arc_length):
        """
        变换单个点（根据弧长位置插值）
        
        参数:
            point: 3D点坐标
            arc_length: 该点在轨迹上的弧长位置
        
        返回:
            变换后的3D点
        """
        # 归一化弧长到 [0, 1]
        s_norm = arc_length / self.total_arc_length
        s_norm = np.clip(s_norm, 0, 1)
        
        # 计算每个分段的权重（高斯权重）
        weights = []
        for center, (start, end) in zip(self.segment_centers, self.segment_ranges):
            center_norm = center / self.total_arc_length
            sigma = (end - start) / self.total_arc_length / 2
            w = np.exp(-((s_norm - center_norm) ** 2) / (2 * sigma ** 2 + 1e-10))
            weights.append(w)
        
        weights = np.array(weights)
        weights /= (weights.sum() + 1e-10)  # 归一化
        
        # 加权变换
        point_homo = np.append(point, 1)
        result = np.zeros(3)
        
        for w, T in zip(weights, self.segment_transforms):
            transformed = (T @ point_homo)[:3]
            result += w * transformed
        
        return result
    
    def transform_trajectory(self, points):
        """
        变换整条轨迹（自动计算弧长）
        
        参数:
            points: Nx3 点云数组
        
        返回:
            transformed: Nx3 变换后的点云
        """
        # 计算输入轨迹的弧长
        arc_lengths = compute_arc_length(points)
        
        # 缩放弧长到与原始轨迹匹配
        input_total = arc_lengths[-1]
        scale = self.total_arc_length / (input_total + 1e-10)
        scaled_arc_lengths = arc_lengths * scale
        
        # 变换每个点
        transformed = np.zeros_like(points)
        for i, (p, s) in enumerate(zip(points, scaled_arc_lengths)):
            transformed[i] = self.transform_point(p, s)
        
        return transformed


def segmented_kabsch_align(source, target, params):
    """
    ⭐⭐ 分段Kabsch配准（带自适应细分）
    将轨迹分成多段，每段独立配准，然后融合
    可以处理非均匀形变（局部拉伸/压缩）
    
    自适应细分原理：
    ================
    1. 第一轮：将轨迹分成 num_segments 段，每段进行Kabsch配准
    2. 检查每段RMSE：如果 RMSE > refine_threshold，则该段需要细分
    3. 递归细分：将误差大的段一分为二，重新配准
    4. 终止条件：达到 max_refine_depth 或 点数 < min_segment_points
    
    数学表达：
    =========
    最终变换：P'(s) = Σ w_i(s) × T_i × P
    其中分段集合 S = S_initial ∪ S_refined（初始分段 + 细分分段）
    
    返回:
        T_global: 全局近似变换矩阵（用于显示）
        transformed_points: 变换后的点云
        final_errors: 每个点的误差
        seg_transform: SegmentedTransform对象（用于应用到新数据）
    """
    N = len(source)
    num_segments = params["num_segments"]
    overlap_ratio = params["overlap_ratio"]
    adaptive_refine = params.get("adaptive_refine", False)
    refine_threshold = params.get("refine_threshold", 0.5)
    max_refine_depth = params.get("max_refine_depth", 3)
    min_segment_points = params.get("min_segment_points", 20)
    blend_method = params.get("blend_method", "gaussian")
    
    # 计算弧长
    source_arc = compute_arc_length(source)
    total_arc_length = source_arc[-1]
    
    # 计算每段的点数和重叠
    base_segment_size = N // num_segments
    overlap_size = int(base_segment_size * overlap_ratio)
    
    print(f"  分段配准: {num_segments}段, 每段~{base_segment_size}点, 重叠{overlap_size}点")
    print(f"  总弧长: {total_arc_length:.2f}")
    if adaptive_refine:
        print(f"  ⭐ 自适应细分: 阈值={refine_threshold}mm, 最大深度={max_refine_depth}")
    
    # ========== 内部辅助函数 ==========
    def align_segment(start_idx, end_idx, depth=0):
        """
        配准单个分段，返回配准信息
        
        ⭐⭐⭐ 核心改进：使用RANSAC-Kabsch在SVD前剔除异常值
        
        返回:
            dict: {
                'start_idx', 'end_idx', 'depth',
                'T': 变换矩阵,
                'arc_start', 'arc_end', 'arc_center',
                'rmse': 分段RMSE,
                'need_refine': 是否需要细分,
                'inlier_ratio': 内点比例（RANSAC启用时）
            }
        """
        seg_source = source[start_idx:end_idx]
        seg_target = target[start_idx:end_idx]
        seg_len = len(seg_source)
        
        # ⭐⭐⭐ 鲁棒Kabsch配准（RANSAC-Kabsch或传统Kabsch）
        if RANSAC_KABSCH_PARAMS["enable"]:
            T_seg, inliers_mask = kabsch_align_robust(
                seg_source, seg_target, 
                ransac_params=RANSAC_KABSCH_PARAMS,
                verbose=False  # 段级别不打印详细信息，避免刷屏
            )
            inlier_ratio = np.sum(inliers_mask) / len(inliers_mask)
        else:
            T_seg = kabsch_align(seg_source, seg_target)
            inlier_ratio = 1.0  # 传统模式视为所有点都是内点
        
        # 计算变换后的点和RMSE
        seg_homo = np.hstack([seg_source, np.ones((seg_len, 1))])
        seg_transformed = (T_seg @ seg_homo.T).T[:, :3]
        seg_errors = np.linalg.norm(seg_transformed - seg_target, axis=1)
        seg_rmse = np.sqrt(np.mean(seg_errors**2))
        
        # 弧长信息
        arc_start = source_arc[start_idx]
        arc_end = source_arc[min(end_idx - 1, N - 1)]
        arc_center = (arc_start + arc_end) / 2
        
        # 判断是否需要细分
        need_refine = (
            adaptive_refine and
            seg_rmse > refine_threshold and
            depth < max_refine_depth and
            seg_len >= 2 * min_segment_points  # 细分后每段至少有min_segment_points个点
        )
        
        return {
            'start_idx': start_idx,
            'end_idx': end_idx,
            'depth': depth,
            'T': T_seg,
            'arc_start': arc_start,
            'arc_end': arc_end,
            'arc_center': arc_center,
            'rmse': seg_rmse,
            'transformed': seg_transformed,
            'need_refine': need_refine,
            'inlier_ratio': inlier_ratio,  # ⭐ 新增：记录内点比例
        }
    
    def refine_segment(seg_info):
        """
        递归细分一个分段
        
        返回:
            list: 细分后的分段信息列表
        """
        if not seg_info['need_refine']:
            return [seg_info]
        
        start_idx = seg_info['start_idx']
        end_idx = seg_info['end_idx']
        mid_idx = (start_idx + end_idx) // 2
        depth = seg_info['depth'] + 1
        
        # 带重叠的细分
        overlap = int((end_idx - start_idx) * overlap_ratio * 0.5)
        
        # 左半段
        left_start = start_idx
        left_end = min(mid_idx + overlap, end_idx)
        left_info = align_segment(left_start, left_end, depth)
        
        # 右半段
        right_start = max(mid_idx - overlap, start_idx)
        right_end = end_idx
        right_info = align_segment(right_start, right_end, depth)
        
        # 递归细分
        result = []
        result.extend(refine_segment(left_info))
        result.extend(refine_segment(right_info))
        
        return result
    
    # ========== 第一轮：初始分段配准 ==========
    initial_segments = []
    for seg_i in range(num_segments):
        start_idx = max(0, seg_i * base_segment_size - overlap_size)
        end_idx = min(N, (seg_i + 1) * base_segment_size + overlap_size)
        if seg_i == num_segments - 1:
            end_idx = N
        
        seg_info = align_segment(start_idx, end_idx, depth=0)
        initial_segments.append(seg_info)
    
    # ========== 第二轮：自适应细分 ==========
    all_segments = []
    refine_count = 0
    depth_counts = {0: 0}  # 统计每层细分数量
    
    for seg_info in initial_segments:
        if seg_info['need_refine']:
            # 需要细分
            refined = refine_segment(seg_info)
            all_segments.extend(refined)
            refine_count += len(refined) - 1  # 减1是因为原始段被替换了
        else:
            all_segments.append(seg_info)
    
    # 统计细分深度
    for seg in all_segments:
        d = seg['depth']
        depth_counts[d] = depth_counts.get(d, 0) + 1
    
    # ========== 打印细分统计 ==========
    if adaptive_refine:
        print(f"\n  【自适应细分统计】")
        print(f"    初始分段数: {num_segments}")
        print(f"    细分后总段数: {len(all_segments)}")
        print(f"    新增分段数: {len(all_segments) - num_segments}")
        for d, count in sorted(depth_counts.items()):
            print(f"    深度{d}: {count}段")
        
        # 找出RMSE最大的几个段
        sorted_segs = sorted(all_segments, key=lambda x: x['rmse'], reverse=True)
        print(f"    RMSE最大的5段: {[f'{s['rmse']:.3f}' for s in sorted_segs[:5]]}")
    
    # ========== 打印RANSAC统计 ==========
    if RANSAC_KABSCH_PARAMS["enable"]:
        inlier_ratios = [seg['inlier_ratio'] for seg in all_segments]
        mean_inlier_ratio = np.mean(inlier_ratios)
        min_inlier_ratio = np.min(inlier_ratios)
        outlier_segs = sum(1 for r in inlier_ratios if r < 0.95)
        print(f"\n  ⭐⭐⭐ RANSAC异常值检测统计:")
        print(f"    平均内点比例: {mean_inlier_ratio:.1%}")
        print(f"    最小内点比例: {min_inlier_ratio:.1%}")
        print(f"    疑似异常段数: {outlier_segs}/{len(all_segments)} (内点<95%的段)")
        if mean_inlier_ratio < 0.90:
            print(f"    ⚠️  警告：平均内点比例<90%，数据中可能存在较多飞点！")
        if min_inlier_ratio < 0.70:
            print(f"    ⚠️  严重：某些段内点比例<70%，建议检查数据质量！")
    
    # ========== 创建分段变换对象 ==========
    seg_transform = SegmentedTransform()
    seg_transform.total_arc_length = total_arc_length
    seg_transform.source_arc_lengths = source_arc
    
    # 按弧长中心排序，确保变换应用顺序正确
    all_segments.sort(key=lambda x: x['arc_center'])
    
    for seg in all_segments:
        seg_transform.segment_transforms.append(seg['T'])
        seg_transform.segment_centers.append(seg['arc_center'])
        seg_transform.segment_ranges.append((seg['arc_start'], seg['arc_end']))
        seg_transform.segment_rmses.append(seg['rmse'])
        seg_transform.segment_depths.append(seg['depth'])
    
    seg_transform.num_segments = len(all_segments)
    seg_transform.refine_stats = {
        'initial_segments': num_segments,
        'final_segments': len(all_segments),
        'refined_count': len(all_segments) - num_segments,
        'depth_distribution': depth_counts,
        'refine_threshold': refine_threshold,
        'max_refine_depth': max_refine_depth,
    }
    
    # ========== 应用变换并融合（归一化弧长空间，与复用完全一致）==========
    print(f"\n  ⭐⭐⭐ 应用变换：使用归一化弧长空间权重（与复用完全一致）")
    transformed_points = np.zeros_like(source)
    weight_sum = np.zeros(N)
    
    # ⭐⭐⭐ 预计算归一化弧长位置（与 apply_transform.py 完全一致）
    normalized_arc = source_arc / total_arc_length  # ∈ [0, 1]
    print(f"      归一化弧长范围: [0.0, 1.0]，软高斯权重（无硬截断）")
    
    # ⭐⭐⭐ 端点特殊处理参数
    endpoint_ratio = 0.10  # 端点区域占总弧长的比例
    endpoint_weight_boost = 2.0  # 端点段权重提升因子
    
    # 按弧长中心排序后的段索引
    num_segs = len(all_segments)
    
    for seg_idx, seg in enumerate(all_segments):
        start_idx = seg['start_idx']
        end_idx = seg['end_idx']
        T = seg['T']
        arc_center = seg['arc_center']
        depth = seg['depth']
        
        # 判断是否为端点段
        is_start_endpoint_seg = normalized_arc[start_idx] < endpoint_ratio
        is_end_endpoint_seg = normalized_arc[min(end_idx-1, N-1)] > (1.0 - endpoint_ratio)
        
        # ⭐⭐⭐ 归一化弧长空间的权重计算（与 apply_transform.py 完全一致）
        # 段的归一化弧长范围
        seg_arc_start = source_arc[start_idx]
        seg_arc_end = source_arc[min(end_idx - 1, N - 1)]
        norm_center = (seg_arc_start + seg_arc_end) / 2 / total_arc_length
        norm_range = (seg_arc_end - seg_arc_start) / total_arc_length
        sigma_norm = norm_range / 3  # 归一化空间的 sigma
        
        # ⭐⭐⭐ 遍历所有点（软高斯，不硬截断）
        for i in range(N):
            norm_s = normalized_arc[i]  # 当前点的归一化弧长位置
            
            # 1. 基础高斯权重（归一化弧长空间）
            w = np.exp(-((norm_s - norm_center) ** 2) / (2 * sigma_norm ** 2 + 1e-10))
            
            # 2. 深度加权：细分段权重更高
            w *= (1.0 + depth * 0.3)
            
            # 3. 端点权重提升
            if is_start_endpoint_seg or is_end_endpoint_seg:
                # 判断当前点是否在端点区域内
                is_point_in_start_region = norm_s < endpoint_ratio
                is_point_in_end_region = norm_s > (1.0 - endpoint_ratio)
                
                if (is_start_endpoint_seg and is_point_in_start_region) or \
                   (is_end_endpoint_seg and is_point_in_end_region):
                    w *= endpoint_weight_boost
                    
                    # 4. 边界渐变补偿
                    if norm_range > 1e-10:
                        # 计算点在段内的相对位置
                        relative_pos = (norm_s - (seg_arc_start / total_arc_length)) / norm_range
                        relative_pos = np.clip(relative_pos, 0.0, 1.0)
                        
                        if is_start_endpoint_seg:
                            # 起点段：后半部分权重渐增（1.0 → 1.5）
                            boundary_boost = 1.0 + 0.5 * relative_pos
                            w *= boundary_boost
                        elif is_end_endpoint_seg:
                            # 终点段：前半部分权重渐减（1.5 → 1.0）
                            boundary_boost = 1.5 - 0.5 * relative_pos
                            w *= boundary_boost
            
            # 累加权重和变换结果
            weight_sum[i] += w
            transformed_points[i] += w * (T @ np.append(source[i], 1))[:3]
    
    # 归一化
    weight_sum[weight_sum == 0] = 1
    transformed_points /= weight_sum[:, np.newaxis]
    
    # ⭐⭐⭐ 端点区域误差统计（使用归一化弧长）
    endpoint_arc_start_abs = total_arc_length * endpoint_ratio
    endpoint_arc_end_abs = total_arc_length * (1 - endpoint_ratio)
    endpoint_start_mask = source_arc < endpoint_arc_start_abs
    endpoint_end_mask = source_arc > endpoint_arc_end_abs
    
    # 计算最终误差
    final_errors = np.linalg.norm(transformed_points - target, axis=1)
    final_rmse = np.sqrt(np.mean(final_errors**2))
    
    # 端点区域误差
    start_errors = final_errors[endpoint_start_mask] if np.any(endpoint_start_mask) else np.array([0])
    end_errors = final_errors[endpoint_end_mask] if np.any(endpoint_end_mask) else np.array([0])
    
    segment_rmses = [seg['rmse'] for seg in all_segments[:10]]
    print(f"\n  各段RMSE(前10): {[f'{r:.3f}' for r in segment_rmses]}...")
    print(f"  融合后RMSE: {final_rmse:.4f} (归一化弧长空间权重)")
    print(f"  ⭐ 端点区域误差（权重提升{endpoint_weight_boost}x）:")
    print(f"     起点区域(0~{endpoint_arc_start_abs:.0f}mm): 平均{np.mean(start_errors):.3f}mm, 最大{np.max(start_errors):.3f}mm")
    print(f"     终点区域({endpoint_arc_end_abs:.0f}~{total_arc_length:.0f}mm): 平均{np.mean(end_errors):.3f}mm, 最大{np.max(end_errors):.3f}mm")
    
    # 返回全局变换矩阵（用于显示旋转/平移信息）
    T_global = kabsch_align(source, target)
    
    return T_global, transformed_points, final_errors, seg_transform


def segmented_kabsch_align_by_sequence(source, target, params):
    """
    ⭐⭐⭐ 序列号对应模式的分段Kabsch配准（带自适应细分）
    
    假设：
    - source 和 target 的行索引（帧号）一一对应
    - 第i行在两个坐标系中对应同一物理时刻
    - 无需通过弧长猜测对应关系
    
    与 segmented_kabsch_align 的区别：
    - 权重基于帧号距离（而非弧长距离）
    - JSON保存帧号范围（而非弧长范围）
    - 不依赖弧长计算，消除"海岸线悖论"
    
    返回:
        T_global: 全局近似变换矩阵（用于显示）
        transformed_points: 变换后的点云
        final_errors: 每个点的误差
        seg_transform: SegmentedTransform对象（序列号模式）
    """
    N = len(source)
    num_segments = params["num_segments"]
    overlap_ratio = params["overlap_ratio"]
    adaptive_refine = params.get("adaptive_refine", False)
    refine_threshold = params.get("refine_threshold", 0.5)
    max_refine_depth = params.get("max_refine_depth", 3)
    min_segment_points = params.get("min_segment_points", 20)
    
    # 按帧数均分
    base_segment_size = N // num_segments
    overlap_size = int(base_segment_size * overlap_ratio)
    
    print(f"\n  ⭐⭐⭐ 序列号对应模式 - 分段配准")
    print(f"  总帧数: {N}")
    print(f"  分段数: {num_segments}段, 每段~{base_segment_size}帧, 重叠{overlap_size}帧")
    if adaptive_refine:
        print(f"  自适应细分: 阈值={refine_threshold}mm, 最大深度={max_refine_depth}")
    
    # ========== 内部辅助函数 ==========
    def align_segment(start_idx, end_idx, depth=0):
        """配准单个分段（基于帧号）"""
        seg_source = source[start_idx:end_idx]
        seg_target = target[start_idx:end_idx]
        seg_len = len(seg_source)
        
        # ⭐⭐⭐ 鲁棒Kabsch配准（RANSAC-Kabsch或传统Kabsch）
        if RANSAC_KABSCH_PARAMS["enable"]:
            T_seg, inliers_mask = kabsch_align_robust(
                seg_source, seg_target, 
                ransac_params=RANSAC_KABSCH_PARAMS,
                verbose=False  # 段级别不打印详细信息，避免刷屏
            )
            inlier_ratio = np.sum(inliers_mask) / len(inliers_mask)
        else:
            T_seg = kabsch_align(seg_source, seg_target)
            inlier_ratio = 1.0  # 传统模式视为所有点都是内点
        
        seg_homo = np.hstack([seg_source, np.ones((seg_len, 1))])
        seg_transformed = (T_seg @ seg_homo.T).T[:, :3]
        seg_errors = np.linalg.norm(seg_transformed - seg_target, axis=1)
        seg_rmse = np.sqrt(np.mean(seg_errors**2))
        
        center_frame = (start_idx + end_idx) // 2
        
        need_refine = (
            adaptive_refine and
            seg_rmse > refine_threshold and
            depth < max_refine_depth and
            seg_len >= 2 * min_segment_points
        )
        
        return {
            'start_idx': start_idx,
            'end_idx': end_idx,
            'depth': depth,
            'T': T_seg,
            'center_frame': center_frame,
            'frame_range': [start_idx, end_idx],
            'rmse': seg_rmse,
            'transformed': seg_transformed,
            'need_refine': need_refine,
            'inlier_ratio': inlier_ratio,  # ⭐ 新增：记录内点比例
        }
    
    def refine_segment(seg_info):
        """递归细分一个分段"""
        if not seg_info['need_refine']:
            return [seg_info]
        
        start_idx = seg_info['start_idx']
        end_idx = seg_info['end_idx']
        mid_idx = (start_idx + end_idx) // 2
        depth = seg_info['depth'] + 1
        
        overlap = int((end_idx - start_idx) * overlap_ratio * 0.5)
        
        left_start = start_idx
        left_end = min(mid_idx + overlap, end_idx)
        left_info = align_segment(left_start, left_end, depth)
        
        right_start = max(mid_idx - overlap, start_idx)
        right_end = end_idx
        right_info = align_segment(right_start, right_end, depth)
        
        result = []
        result.extend(refine_segment(left_info))
        result.extend(refine_segment(right_info))
        return result
    
    # ========== 第一轮：初始分段配准 ==========
    initial_segments = []
    for seg_i in range(num_segments):
        start_idx = max(0, seg_i * base_segment_size - overlap_size)
        end_idx = min(N, (seg_i + 1) * base_segment_size + overlap_size)
        if seg_i == num_segments - 1:
            end_idx = N
        
        seg_info = align_segment(start_idx, end_idx, depth=0)
        initial_segments.append(seg_info)
    
    # ========== 第二轮：自适应细分 ==========
    all_segments = []
    depth_counts = {0: 0}
    
    for seg_info in initial_segments:
        if seg_info['need_refine']:
            refined = refine_segment(seg_info)
            all_segments.extend(refined)
        else:
            all_segments.append(seg_info)
    
    for seg in all_segments:
        d = seg['depth']
        depth_counts[d] = depth_counts.get(d, 0) + 1
    
    if adaptive_refine:
        print(f"\n  【自适应细分统计】")
        print(f"    初始分段数: {num_segments}")
        print(f"    细分后总段数: {len(all_segments)}")
        for d, count in sorted(depth_counts.items()):
            print(f"    深度{d}: {count}段")
        sorted_segs = sorted(all_segments, key=lambda x: x['rmse'], reverse=True)
        print(f"    RMSE最大的5段: {[f'{s['rmse']:.3f}' for s in sorted_segs[:5]]}")
    
    # ========== 打印RANSAC统计 ==========
    if RANSAC_KABSCH_PARAMS["enable"]:
        inlier_ratios = [seg['inlier_ratio'] for seg in all_segments]
        mean_inlier_ratio = np.mean(inlier_ratios)
        min_inlier_ratio = np.min(inlier_ratios)
        outlier_segs = sum(1 for r in inlier_ratios if r < 0.95)
        print(f"\n  ⭐⭐⭐ RANSAC异常值检测统计:")
        print(f"    平均内点比例: {mean_inlier_ratio:.1%}")
        print(f"    最小内点比例: {min_inlier_ratio:.1%}")
        print(f"    疑似异常段数: {outlier_segs}/{len(all_segments)} (内点<95%的段)")
        if mean_inlier_ratio < 0.90:
            print(f"    ⚠️  警告：平均内点比例<90%，数据中可能存在较多飞点！")
        if min_inlier_ratio < 0.70:
            print(f"    ⚠️  严重：某些段内点比例<70%，建议检查数据质量！")
    
    # ========== 创建分段变换对象（序列号模式）==========
    seg_transform = SegmentedTransform()
    seg_transform.mode = "sequence_aligned"
    seg_transform.total_frames = N
    
    # 按中心帧号排序
    all_segments.sort(key=lambda x: x['center_frame'])
    
    for seg in all_segments:
        seg_transform.segment_transforms.append(seg['T'])
        seg_transform.segment_center_frames.append(seg['center_frame'])
        seg_transform.segment_frame_ranges.append(tuple(seg['frame_range']))
        seg_transform.segment_rmses.append(seg['rmse'])
        seg_transform.segment_depths.append(seg['depth'])
        
        # 兼容弧长模式的字段（设为帧号等价值）
        seg_transform.segment_centers.append(float(seg['center_frame']))
        seg_transform.segment_ranges.append((float(seg['frame_range'][0]), float(seg['frame_range'][1])))
    
    seg_transform.num_segments = len(all_segments)
    seg_transform.total_arc_length = float(N)  # 帧数作为等价弧长
    seg_transform.refine_stats = {
        'initial_segments': num_segments,
        'final_segments': len(all_segments),
        'refined_count': len(all_segments) - num_segments,
        'depth_distribution': depth_counts,
        'refine_threshold': refine_threshold,
        'max_refine_depth': max_refine_depth,
    }
    
    # ========== 应用变换并融合（基于帧号高斯权重）==========
    print(f"\n  ⭐ 应用变换：基于帧号的高斯权重")
    transformed_points = np.zeros_like(source)
    weight_sum = np.zeros(N)
    
    # 端点参数
    endpoint_ratio = 0.10
    endpoint_weight_boost = 2.0
    
    for seg_idx, seg in enumerate(all_segments):
        T = seg['T']
        center_frame = seg['center_frame']
        start_frame, end_frame = seg['frame_range']
        frame_span = end_frame - start_frame
        sigma_frame = frame_span / 3
        depth = seg['depth']
        
        # 判断端点段
        is_start_endpoint_seg = start_frame < N * endpoint_ratio
        is_end_endpoint_seg = end_frame > N * (1 - endpoint_ratio)
        
        for frame_idx in range(N):
            norm_pos = frame_idx / N  # 归一化帧位置
            
            # 1. 基础高斯权重（帧号距离）
            w = np.exp(-((frame_idx - center_frame) ** 2) / (2 * sigma_frame ** 2 + 1e-10))
            
            # 2. 深度加权
            w *= (1.0 + depth * 0.3)
            
            # 3. 端点权重提升
            is_in_start_region = norm_pos < endpoint_ratio
            is_in_end_region = norm_pos > (1.0 - endpoint_ratio)
            
            if (is_start_endpoint_seg and is_in_start_region) or \
               (is_end_endpoint_seg and is_in_end_region):
                w *= endpoint_weight_boost
                
                # 4. 边界渐变补偿
                if frame_span > 0:
                    relative_pos = (frame_idx - start_frame) / frame_span
                    relative_pos = np.clip(relative_pos, 0.0, 1.0)
                    
                    if is_start_endpoint_seg:
                        boundary_boost = 1.0 + 0.5 * relative_pos
                        w *= boundary_boost
                    elif is_end_endpoint_seg:
                        boundary_boost = 1.5 - 0.5 * relative_pos
                        w *= boundary_boost
            
            weight_sum[frame_idx] += w
            transformed_points[frame_idx] += w * (T @ np.append(source[frame_idx], 1))[:3]
    
    # 归一化
    weight_sum[weight_sum == 0] = 1
    transformed_points /= weight_sum[:, np.newaxis]
    
    # 计算最终误差
    final_errors = np.linalg.norm(transformed_points - target, axis=1)
    final_rmse = np.sqrt(np.mean(final_errors**2))
    
    # 端点区域误差统计
    start_mask = np.arange(N) < N * endpoint_ratio
    end_mask = np.arange(N) > N * (1 - endpoint_ratio)
    start_errors = final_errors[start_mask] if np.any(start_mask) else np.array([0])
    end_errors = final_errors[end_mask] if np.any(end_mask) else np.array([0])
    
    segment_rmses = [seg['rmse'] for seg in all_segments[:10]]
    print(f"\n  各段RMSE(前10): {[f'{r:.3f}' for r in segment_rmses]}...")
    print(f"  融合后RMSE: {final_rmse:.4f} (帧号高斯权重)")
    print(f"  ⭐ 端点区域误差:")
    print(f"     起点区域(前{endpoint_ratio*100:.0f}%): 平均{np.mean(start_errors):.3f}mm, 最大{np.max(start_errors):.3f}mm")
    print(f"     终点区域(后{endpoint_ratio*100:.0f}%): 平均{np.mean(end_errors):.3f}mm, 最大{np.max(end_errors):.3f}mm")
    
    # 全局变换矩阵
    T_global = kabsch_align(source, target)
    
    return T_global, transformed_points, final_errors, seg_transform


def remove_outliers(source, target, T, params):
    """
    剔除异常值点对，返回清洗后的点集
    """
    if not params["enable"]:
        return source, target, np.arange(len(source))
    
    N = len(source)
    src_homo = np.hstack([source, np.ones((N, 1))])
    transformed = (T @ src_homo.T).T[:, :3]
    errors = np.linalg.norm(transformed - target, axis=1)
    
    if params["method"] == "percentile":
        threshold = np.percentile(errors, params["percentile_threshold"])
        valid_mask = errors <= threshold
        print(f"  百分位剔除: 阈值={threshold:.4f}, 剔除{np.sum(~valid_mask)}点")
    
    elif params["method"] == "iterative":
        valid_mask = np.ones(N, dtype=bool)
        for round_i in range(params["iterative_rounds"]):
            current_errors = errors[valid_mask]
            if len(current_errors) < 10:
                break
            mean_err = np.mean(current_errors)
            std_err = np.std(current_errors)
            threshold = mean_err + params["iterative_sigma"] * std_err
            
            new_mask = errors <= threshold
            removed = np.sum(valid_mask) - np.sum(new_mask)
            valid_mask = new_mask
            print(f"    迭代{round_i+1}: 阈值={threshold:.4f}, 剔除{removed}点")
            
            if removed == 0:
                break
    else:
        valid_mask = np.ones(N, dtype=bool)
    
    valid_indices = np.where(valid_mask)[0]
    print(f"  剔除后点数: {len(valid_indices)}/{N} ({len(valid_indices)/N*100:.1f}%)")
    
    return source[valid_mask], target[valid_mask], valid_indices


# ============================================================================
#              ⭐⭐⭐ 移动最小二乘(MLS)配准
# ============================================================================

def mirror_extend(train_source, train_target, train_norm_arc, extend_ratio=0.1):
    """
    在两端镜像延伸训练数据，解决端点单侧邻域问题
    
    原理：
    1. 取前 extend_ratio 的训练点，沿起点镜像翻转
    2. 取后 extend_ratio 的训练点，沿终点镜像翻转
    3. 拼接到原始数据两端
    
    参数:
        train_source:   M×3 训练源点云
        train_target:   M×3 训练目标点云
        train_norm_arc: M 训练点归一化弧长
        extend_ratio:   延伸比例（两端各延伸此比例）
    
    返回:
        extended_src, extended_tgt, extended_arc
    """
    N = len(train_source)
    extend_n = max(int(N * extend_ratio), 1)
    
    # 起点镜像延伸（翻转前extend_n个点，弧长取负值）
    start_src = train_source[:extend_n][::-1]
    start_tgt = train_target[:extend_n][::-1]
    start_arc = -train_norm_arc[:extend_n][::-1]
    
    # 终点镜像延伸（翻转后extend_n个点，弧长取>1值）
    end_src = train_source[-extend_n:][::-1]
    end_tgt = train_target[-extend_n:][::-1]
    end_arc = 2.0 - train_norm_arc[-extend_n:][::-1]
    
    # 拼接
    extended_src = np.vstack([start_src, train_source, end_src])
    extended_tgt = np.vstack([start_tgt, train_target, end_tgt])
    extended_arc = np.concatenate([start_arc, train_norm_arc, end_arc])
    
    return extended_src, extended_tgt, extended_arc


def estimate_bandwidth_empirical(num_training_points, noise_level_mm, total_arc_mm):
    """
    基于数据特征的经验带宽估计
    
    原理：
    - 带宽应大于噪声引起的弧长波动
    - 带宽应小于轨迹特征变化的尺度
    
    参数:
        num_training_points: 训练点数
        noise_level_mm:      噪声水平(mm)
        total_arc_mm:        总弧长(mm)
    
    返回:
        h: 归一化弧长空间的带宽
    """
    # 归一化噪声水平
    noise_norm = noise_level_mm / total_arc_mm
    
    # 经验公式：h = 3 × 归一化噪声 + 最小保证带宽
    h = max(3 * noise_norm, 0.02) + 0.03
    
    # 限制范围
    h = np.clip(h, 0.03, 0.3)
    
    return h


def bandwidth_loocv(source, target, norm_arc, h_candidates, verbose=True):
    """
    留一交叉验证选择最优带宽
    
    对每个候选 h：
    1. 依次取出每个训练点 i
    2. 用剩余点对 i 做MLS变换
    3. 计算 i 的预测误差
    4. 选择总误差最小的 h
    
    参数:
        source:       M×3 训练源点云
        target:       M×3 训练目标点云
        norm_arc:     M 训练点归一化弧长
        h_candidates: 候选带宽列表
        verbose:      是否打印详细信息
    
    返回:
        best_h:    最优带宽
        best_rmse: 对应的LOOCV RMSE
    """
    best_h = h_candidates[0]
    best_error = np.inf
    
    N = len(source)
    results = []
    
    if verbose:
        print(f"  LOOCV带宽选择: {len(h_candidates)}个候选, {N}个训练点")
    
    for h in h_candidates:
        total_error = 0
        valid_count = 0
        
        for i in range(N):
            # 留出第 i 个点
            mask = np.ones(N, dtype=bool)
            mask[i] = False
            
            # 用其余点计算权重
            distances = np.abs(norm_arc[i] - norm_arc[mask])
            weights = np.exp(-(distances ** 2) / (h ** 2))
            
            # 最小权重检查
            if weights.sum() < 1e-6:
                total_error += 100  # 惩罚：无有效邻居
                valid_count += 1
                continue
            
            # 加权Kabsch
            T = weighted_kabsch(source[mask], target[mask], weights)
            predicted = (T @ np.append(source[i], 1))[:3]
            error = np.linalg.norm(predicted - target[i])
            total_error += error ** 2
            valid_count += 1
        
        rmse = np.sqrt(total_error / max(valid_count, 1))
        results.append((h, rmse))
        
        if rmse < best_error:
            best_error = rmse
            best_h = h
    
    if verbose:
        print(f"  LOOCV结果:")
        for h, rmse in results:
            marker = " ⭐" if h == best_h else ""
            print(f"    h={h:.3f}: RMSE={rmse:.4f}{marker}")
        print(f"  最优带宽: h={best_h:.3f} (RMSE={best_error:.4f})")
    
    return best_h, best_error


class MLSTransform:
    """
    ⭐⭐⭐ 移动最小二乘变换类
    
    数学公式：
    P'(x) = R(x) · x + t(x)
    
    其中 R(x), t(x) 通过加权Kabsch求解：
    E(x) = Σ w_i(x) · |R·p_i + t - q_i|²
    w_i(x) = exp(-|s(x) - s(p_i)|² / h²)
    
    支持两种复用模式：
    - "full": 完整计算（每个点执行加权Kabsch SVD）
    - "grid": 预计算网格插值（快速，推荐）
    """
    
    def __init__(self):
        # 训练数据
        self.train_source = None       # M×3 训练源点云
        self.train_target = None       # M×3 训练目标点云 
        self.train_norm_arc = None     # M 训练点归一化弧长
        self.total_arc_length = 0      # 训练数据的总弧长
        
        # 带宽参数
        self.bandwidth = 0.05          # 高斯核带宽 h
        
        # 预计算网格（快速模式）
        self.grid_positions = None     # G 个网格位置
        self.grid_transforms = None    # G 个预计算变换矩阵 (4x4)
        
        # 端点延伸数据
        self.extended_source = None    # 含端点延伸的源点
        self.extended_target = None
        self.extended_norm_arc = None
        
        # 统计信息
        self.training_rmse = 0
        self.training_max_error = 0
        self.training_mean_error = 0
        self.mode = "mls"
        self.num_training_points = 0
    
    def fit(self, source, target, bandwidth=None, bandwidth_method="loocv",
            loocv_candidates=None, precompute_grid=True, grid_size=200,
            endpoint_extend=0.10, enable_ransac=True, verbose=True):
        """
        训练MLS变换
        
        参数:
            source:           M×3 预处理后的源点云
            target:           M×3 预处理后的目标点云
            bandwidth:        带宽参数（None=自动选择）
            bandwidth_method: "empirical"=经验公式, "loocv"=交叉验证
            loocv_candidates: LOOCV候选带宽列表
            precompute_grid:  是否预计算网格变换（推荐True）
            grid_size:        网格点数（200~500）
            endpoint_extend:  端点镜像延伸比例
            enable_ransac:    是否在训练前用RANSAC剔除异常值
            verbose:          是否打印详细信息
        """
        M = len(source)
        assert len(source) == len(target), "源点云和目标点云点数必须相同！"
        
        if verbose:
            print(f"\n  ⭐⭐⭐ MLS训练: {M}个训练点")
        
        # 计算弧长
        source_arc = compute_arc_length(source)
        self.total_arc_length = source_arc[-1]
        
        # 归一化弧长
        self.train_norm_arc = source_arc / self.total_arc_length  # ∈ [0, 1]
        self.train_source = source.copy()
        self.train_target = target.copy()
        self.num_training_points = M
        
        if verbose:
            print(f"  训练数据弧长: {self.total_arc_length:.2f}mm")
            print(f"  归一化弧长范围: [{self.train_norm_arc[0]:.4f}, {self.train_norm_arc[-1]:.4f}]")
        
        # ========== 带宽选择 ==========
        if bandwidth is not None:
            self.bandwidth = bandwidth
            if verbose:
                print(f"  使用指定带宽: h={self.bandwidth:.4f}")
        elif bandwidth_method == "loocv":
            if loocv_candidates is None:
                loocv_candidates = [0.02, 0.03, 0.05, 0.08, 0.10, 0.15, 0.20]
            
            if verbose:
                print(f"\n  ▶ LOOCV带宽选择...")
            
            # 对LOOCV使用下采样以加速（如果训练点数太多）
            if M > 1000:
                # 每隔几个点取一个作为LOOCV样本
                step = max(1, M // 1000)
                loocv_idx = np.arange(0, M, step)
                loocv_src = source[loocv_idx]
                loocv_tgt = target[loocv_idx]
                loocv_arc = self.train_norm_arc[loocv_idx]
                if verbose:
                    print(f"  LOOCV下采样: {M} → {len(loocv_idx)}点 (加速)")
            else:
                loocv_src = source
                loocv_tgt = target
                loocv_arc = self.train_norm_arc
            
            self.bandwidth, loocv_rmse = bandwidth_loocv(
                loocv_src, loocv_tgt, loocv_arc, loocv_candidates, verbose)
        else:
            # 经验公式
            noise_level = ADAPTIVE_SPATIAL_RESAMPLE.get("delta_d", 1.0) * 0.5  # 估算噪声水平
            self.bandwidth = estimate_bandwidth_empirical(M, noise_level, self.total_arc_length)
            if verbose:
                print(f"  经验公式带宽: h={self.bandwidth:.4f}")
        
        # ========== 端点镜像延伸 ==========
        if endpoint_extend > 0:
            self.extended_source, self.extended_target, self.extended_norm_arc = mirror_extend(
                self.train_source, self.train_target, self.train_norm_arc, endpoint_extend)
            if verbose:
                print(f"  端点镜像延伸: {M} → {len(self.extended_source)}点 (两端各{endpoint_extend*100:.0f}%)")
        else:
            self.extended_source = self.train_source
            self.extended_target = self.train_target
            self.extended_norm_arc = self.train_norm_arc
        
        # ========== 预计算网格变换 ==========
        if precompute_grid:
            if verbose:
                print(f"\n  ▶ 预计算网格变换: {grid_size}个网格点...")
            
            self.grid_positions = np.linspace(0, 1, grid_size)
            self.grid_transforms = []
            
            for s in self.grid_positions:
                # 使用延伸后数据计算权重
                distances = np.abs(s - self.extended_norm_arc)
                weights = np.exp(-(distances ** 2) / (self.bandwidth ** 2))
                
                # 安全检查
                effective_count = np.sum(weights > MLS_PARAMS.get("min_effective_weight", 0.01))
                if effective_count < MLS_PARAMS.get("min_effective_neighbors", 3) or weights.sum() < 1e-6:
                    # 回退：使用均匀权重
                    weights = np.ones(len(self.extended_source))
                
                T_grid = weighted_kabsch(self.extended_source, self.extended_target, weights)
                self.grid_transforms.append(T_grid)
            
            self.grid_transforms = np.array(self.grid_transforms)  # (G, 4, 4)
            
            if verbose:
                print(f"  网格预计算完成: {grid_size}个变换矩阵")
        
        # ========== 计算训练误差 ==========
        if verbose:
            print(f"\n  ▶ 计算训练误差...")
        
        transformed = self.transform_trajectory(source, mode="grid" if precompute_grid else "full")
        errors = np.linalg.norm(transformed - target, axis=1)
        self.training_rmse = np.sqrt(np.mean(errors ** 2))
        self.training_max_error = np.max(errors)
        self.training_mean_error = np.mean(errors)
        
        if verbose:
            print(f"  训练RMSE: {self.training_rmse:.4f}mm")
            print(f"  训练平均误差: {self.training_mean_error:.4f}mm")
            print(f"  训练最大误差: {self.training_max_error:.4f}mm")
        
        return self
    
    def transform_point(self, point, norm_s, mode="grid"):
        """
        变换单个点
        
        参数:
            point:  3D点坐标
            norm_s: 归一化弧长位置 ∈ [0, 1]
            mode:   "grid"=网格插值(快速), "full"=完整计算
        
        返回:
            变换后的3D点
        """
        if mode == "grid" and self.grid_transforms is not None:
            return self._transform_point_grid(point, norm_s)
        else:
            return self._transform_point_full(point, norm_s)
    
    def _transform_point_grid(self, point, norm_s):
        """
        网格插值快速变换: O(1)每点
        
        在预计算网格上找到最近的两个网格点，线性插值变换结果
        """
        norm_s = np.clip(norm_s, 0, 1)
        
        G = len(self.grid_positions)
        idx = np.searchsorted(self.grid_positions, norm_s) - 1
        idx = np.clip(idx, 0, G - 2)
        
        # 线性插值系数
        s_left = self.grid_positions[idx]
        s_right = self.grid_positions[idx + 1]
        t = (norm_s - s_left) / (s_right - s_left + 1e-10)
        t = np.clip(t, 0, 1)
        
        # 对变换结果插值（非矩阵本身）
        point_homo = np.append(point, 1)
        result_left = (self.grid_transforms[idx] @ point_homo)[:3]
        result_right = (self.grid_transforms[idx + 1] @ point_homo)[:3]
        
        return (1 - t) * result_left + t * result_right
    
    def _transform_point_full(self, point, norm_s):
        """
        完整MLS计算: 每点执行加权Kabsch SVD
        """
        # 使用延伸数据（含端点镜像）
        src = self.extended_source if self.extended_source is not None else self.train_source
        tgt = self.extended_target if self.extended_target is not None else self.train_target
        arc = self.extended_norm_arc if self.extended_norm_arc is not None else self.train_norm_arc
        
        distances = np.abs(norm_s - arc)
        weights = np.exp(-(distances ** 2) / (self.bandwidth ** 2))
        
        # 安全检查
        min_w = MLS_PARAMS.get("min_effective_weight", 0.01)
        min_n = MLS_PARAMS.get("min_effective_neighbors", 3)
        effective_count = np.sum(weights > min_w)
        
        if effective_count < min_n or weights.sum() < 1e-6:
            # 回退：使用最近点的局部变换
            nearest_idx = np.argmin(distances)
            # 使用最近邻周围点的等权重Kabsch
            k = min(20, len(src))
            nearest_indices = np.argsort(distances)[:k]
            weights = np.zeros(len(src))
            weights[nearest_indices] = 1.0
        
        T_local = weighted_kabsch(src, tgt, weights)
        return (T_local @ np.append(point, 1))[:3]
    
    def transform_trajectory(self, points, mode="grid"):
        """
        变换整条轨迹
        
        参数:
            points: Nx3 点云数组
            mode:   "grid"=网格插值(快速), "full"=完整计算
        
        返回:
            Nx3 变换后的点云
        """
        # 计算输入轨迹的弧长
        arc_lengths = compute_arc_length(points)
        input_total = arc_lengths[-1]
        
        # 归一化弧长到 [0, 1]
        normalized_arc = arc_lengths / (input_total + 1e-10)
        
        # 变换每个点
        transformed = np.zeros_like(points)
        for i in range(len(points)):
            transformed[i] = self.transform_point(points[i], normalized_arc[i], mode)
        
        return transformed
    
    def save(self, filepath):
        """
        保存MLS变换到JSON
        
        同时保存训练点对和预计算网格，用于复用
        """
        data = {
            "mode": "mls",
            "description": "移动最小二乘(MLS)配准结果",
            "math_formula": "P'(x) = R(x)·x + t(x), 其中 R(x),t(x) 由加权Kabsch求解, w_i(x) = exp(-|s(x)-s(p_i)|²/h²)",
            "bandwidth": float(self.bandwidth),
            "total_arc_length": float(self.total_arc_length),
            "num_training_points": int(self.num_training_points),
            "training_stats": {
                "rmse": float(self.training_rmse),
                "max_error": float(self.training_max_error),
                "mean_error": float(self.training_mean_error),
            },
            "preprocessing": {
                "pre_smooth_sigma": PRE_SMOOTH.get("gaussian_sigma", 1.5) if PRE_SMOOTH.get("enable", False) else None,
                "post_smooth_sigma": POST_SMOOTH.get("gaussian_sigma", 5),
                "resample_method": TIME_ALIGN.get("method", "chord_spatial"),
                "delta_d": ADAPTIVE_SPATIAL_RESAMPLE.get("delta_d", 1.0) if TIME_ALIGN.get("method") == "chord_spatial" else None,
                "densify_factor": ADAPTIVE_SPATIAL_RESAMPLE.get("densify_factor", 10) if TIME_ALIGN.get("method") == "chord_spatial" else None,
                "noise_level": ADAPTIVE_SPATIAL_RESAMPLE.get("noise_level", 0.5) if TIME_ALIGN.get("method") == "adaptive_spatial" else None,
                "noise_suppression_factor": ADAPTIVE_SPATIAL_RESAMPLE.get("noise_suppression_factor", 2.0) if TIME_ALIGN.get("method") == "adaptive_spatial" else None,
            },
            "source_file": SOURCE_CSV,
            "target_file": TARGET_CSV,
        }
        
        # 保存训练数据（方案A: 完整训练数据，用于高精度全量计算）
        data["training_data"] = {
            "source_points": self.train_source.tolist(),
            "target_points": self.train_target.tolist(),
            "normalized_arc_lengths": self.train_norm_arc.tolist(),
        }
        
        # 保存网格变换（方案B: 预计算网格，用于快速复用）
        if self.grid_transforms is not None:
            data["grid"] = {
                "grid_size": len(self.grid_positions),
                "grid_positions": self.grid_positions.tolist(),
                "grid_transforms": self.grid_transforms.tolist(),  # (G, 4, 4)
            }
            data["recommended_mode"] = "grid"
        else:
            data["recommended_mode"] = "full"
        
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        
        # 文件大小
        file_size_kb = os.path.getsize(filepath) / 1024
        
        print(f"  MLS变换已保存: {filepath}")
        print(f"  文件大小: {file_size_kb:.1f}KB")
        print(f"  训练点数: {self.num_training_points}, 网格点数: {len(self.grid_positions) if self.grid_positions is not None else 0}")
        print(f"  带宽: h={self.bandwidth:.4f}")
        
        return filepath
    
    @classmethod
    def load(cls, filepath):
        """
        从JSON加载MLS变换
        
        支持加载训练数据（全量模式）和网格变换（快速模式）
        """
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        obj = cls()
        obj.mode = data.get("mode", "mls")
        obj.bandwidth = data["bandwidth"]
        obj.total_arc_length = data["total_arc_length"]
        obj.num_training_points = data.get("num_training_points", 0)
        
        # 加载训练统计
        stats = data.get("training_stats", {})
        obj.training_rmse = stats.get("rmse", 0)
        obj.training_max_error = stats.get("max_error", 0)
        obj.training_mean_error = stats.get("mean_error", 0)
        
        # 加载训练数据
        if "training_data" in data:
            td = data["training_data"]
            obj.train_source = np.array(td["source_points"])
            obj.train_target = np.array(td["target_points"])
            obj.train_norm_arc = np.array(td["normalized_arc_lengths"])
            
            # 重建端点延伸数据
            extend_ratio = MLS_PARAMS.get("endpoint_extend", 0.10)
            obj.extended_source, obj.extended_target, obj.extended_norm_arc = mirror_extend(
                obj.train_source, obj.train_target, obj.train_norm_arc, extend_ratio)
        
        # 加载网格变换
        if "grid" in data:
            grid = data["grid"]
            obj.grid_positions = np.array(grid["grid_positions"])
            obj.grid_transforms = np.array(grid["grid_transforms"])
        
        print(f"  ⭐ 已加载MLS变换: {filepath}")
        print(f"  带宽: h={obj.bandwidth:.4f}, 训练点数: {obj.num_training_points}")
        if obj.grid_transforms is not None:
            print(f"  网格点数: {len(obj.grid_positions)} (快速模式可用)")
        print(f"  训练RMSE: {obj.training_rmse:.4f}mm")
        
        return obj


def mls_align(source, target, params):
    """
    ⭐⭐⭐ 移动最小二乘(MLS)配准主函数
    
    流程：
    1. 可选RANSAC异常值预剔除
    2. 带宽选择（LOOCV或经验公式）
    3. 端点镜像延伸
    4. 逐点加权Kabsch计算局部变换
    5. 网格预计算（加速复用）
    6. 误差统计
    
    参数:
        source: Nx3 预处理后的源点云
        target: Nx3 预处理后的目标点云
        params: MLS_PARAMS 参数字典
    
    返回:
        T_global:          全局近似变换矩阵（用于显示旋转/平移信息）
        transformed_points: 变换后的点云
        final_errors:       每个点的误差
        mls_transform:      MLSTransform对象（用于保存和复用）
    """
    N = len(source)
    
    print(f"\n{'='*60}")
    print(f"⭐⭐⭐ 移动最小二乘(MLS)配准")
    print(f"{'='*60}")
    print(f"  训练点数: {N}")
    
    # ========== 可选RANSAC预剔除 ==========
    src_clean = source
    tgt_clean = target
    
    if params.get("enable_ransac_weights", True) and RANSAC_KABSCH_PARAMS.get("enable", True):
        print(f"\n  ▶ RANSAC异常值预剔除...")
        # 先用全局Kabsch + RANSAC识别异常值
        T_pre, inliers_mask = kabsch_align_robust(source, target, 
                                                   ransac_params=RANSAC_KABSCH_PARAMS,
                                                   verbose=True)
        inlier_ratio = np.sum(inliers_mask) / N
        outlier_count = N - np.sum(inliers_mask)
        
        if inlier_ratio < 1.0:
            print(f"  RANSAC剔除: {outlier_count}个异常值 (内点率{inlier_ratio:.1%})")
            src_clean = source[inliers_mask]
            tgt_clean = target[inliers_mask]
        else:
            print(f"  无异常值检出（内点率100%）")
    
    # ========== 创建MLS变换对象并训练 ==========
    mls = MLSTransform()
    mls.fit(
        source=src_clean,
        target=tgt_clean,
        bandwidth=params.get("bandwidth"),
        bandwidth_method=params.get("bandwidth_method", "loocv"),
        loocv_candidates=params.get("loocv_candidates"),
        precompute_grid=True,
        grid_size=params.get("grid_size", 200),
        endpoint_extend=params.get("endpoint_extend", 0.10),
        enable_ransac=params.get("enable_ransac_weights", True),
        verbose=True
    )
    
    # ========== 使用训练好的MLS变换所有源点（含被剔除的异常值点） ==========
    print(f"\n  ▶ 对全部{N}点应用MLS变换...")
    transformed_points = mls.transform_trajectory(source, mode="grid")
    
    # 计算最终误差
    final_errors = np.linalg.norm(transformed_points - target, axis=1)
    final_rmse = np.sqrt(np.mean(final_errors ** 2))
    final_mean = np.mean(final_errors)
    final_max = np.max(final_errors)
    
    print(f"\n  MLS配准结果:")
    print(f"    RMSE:  {final_rmse:.4f}mm")
    print(f"    平均:  {final_mean:.4f}mm")
    print(f"    最大:  {final_max:.4f}mm")
    print(f"    P95:   {np.percentile(final_errors, 95):.4f}mm")
    print(f"    带宽:  h={mls.bandwidth:.4f}")
    
    # 返回全局变换矩阵（用于显示旋转/平移信息）
    T_global = kabsch_align(source, target)
    
    return T_global, transformed_points, final_errors, mls


def iterative_kabsch_with_outlier_removal(source, target, outlier_params, max_iterations=3):
    """
    ⭐ 迭代Kabsch + 异常值剔除
    每轮Kabsch后剔除异常值，再重新配准
    """
    current_source = source.copy()
    current_target = target.copy()
    
    for iter_i in range(max_iterations):
        print(f"\n  --- 迭代 {iter_i+1}/{max_iterations} ---")
        
        # Kabsch配准
        T = kabsch_align(current_source, current_target)
        
        # 剔除异常值
        if outlier_params["enable"] and iter_i < max_iterations - 1:
            current_source, current_target, _ = remove_outliers(
                current_source, current_target, T, outlier_params)
        
        # 计算当前RMSE
        N = len(current_source)
        src_homo = np.hstack([current_source, np.ones((N, 1))])
        transformed = (T @ src_homo.T).T[:, :3]
        rmse = np.sqrt(np.mean(np.linalg.norm(transformed - current_target, axis=1)**2))
        print(f"  当前RMSE: {rmse:.4f}")
    
    # 最终配准
    T_final = kabsch_align(current_source, current_target)
    return T_final, current_source, current_target


def icp_align(source, target, params, init_T=None):
    """标准ICP配准"""
    src_pcd = o3d.geometry.PointCloud()
    src_pcd.points = o3d.utility.Vector3dVector(source)
    
    tgt_pcd = o3d.geometry.PointCloud()
    tgt_pcd.points = o3d.utility.Vector3dVector(target)
    
    # 估计法线
    radius = params["voxel_size"] * 2
    src_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
    tgt_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
    
    if init_T is None:
        init_T = np.eye(4)
    
    # 多轮ICP，逐步收紧搜索半径
    current_T = init_T
    radii = [params["max_correspondence_distance"] * m for m in [4, 2, 1, 0.5]]
    
    for i, radius in enumerate(radii):
        result = o3d.pipelines.registration.registration_icp(
            src_pcd, tgt_pcd,
            radius,
            current_T,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=params["relative_fitness"],
                relative_rmse=params["relative_rmse"],
                max_iteration=params["max_iteration"] // 4
            )
        )
        current_T = result.transformation
        print(f"    ICP轮次{i+1}: 半径={radius:.2f}, Fitness={result.fitness:.4f}")
    
    return result.transformation


def ransac_gicp_align(source, target, params_ransac, params_icp):
    """RANSAC + GICP配准"""
    src_pcd = o3d.geometry.PointCloud()
    src_pcd.points = o3d.utility.Vector3dVector(source)
    
    tgt_pcd = o3d.geometry.PointCloud()
    tgt_pcd.points = o3d.utility.Vector3dVector(target)
    
    voxel_size = params_ransac["voxel_size"]
    
    # 下采样和特征计算
    src_down = src_pcd.voxel_down_sample(voxel_size)
    tgt_down = tgt_pcd.voxel_down_sample(voxel_size)
    
    radius_normal = voxel_size * 2
    src_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    tgt_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    
    radius_feature = voxel_size * 5
    src_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        src_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    tgt_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        tgt_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    
    print(f"    RANSAC: 源点云 {len(source)} -> {len(src_down.points)}")
    print(f"    RANSAC: 目标 {len(target)} -> {len(tgt_down.points)}")
    
    # RANSAC
    dist_thresh = voxel_size * params_ransac["distance_threshold_multiplier"]
    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src_down, tgt_down, src_fpfh, tgt_fpfh,
        mutual_filter=True,
        max_correspondence_distance=dist_thresh,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=params_ransac["ransac_n"],
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                params_ransac["edge_length_threshold"]),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist_thresh)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
            params_ransac["max_iteration"], params_ransac["confidence"])
    )
    
    print(f"    RANSAC: Fitness={result_ransac.fitness:.4f}, RMSE={result_ransac.inlier_rmse:.4f}")
    
    # GICP精配准
    return icp_align(source, target, params_icp, result_ransac.transformation)


# ============================================================================
#                              误差计算
# ============================================================================

def compute_error(source, target, T):
    """计算配准误差"""
    N = len(source)
    src_homo = np.hstack([source, np.ones((N, 1))])
    transformed = (T @ src_homo.T).T[:, :3]
    
    errors = np.linalg.norm(transformed - target, axis=1)
    
    return {
        'mean': np.mean(errors),
        'rmse': np.sqrt(np.mean(errors**2)),
        'max': np.max(errors),
        'min': np.min(errors),
        'std': np.std(errors),
        'median': np.median(errors),
        'p95': np.percentile(errors, 95),
    }, errors


def plot_error_curve(errors, error_dict):
    """绘制误差折线图"""
    plt.figure(figsize=(12, 6))
    
    # 主图：误差折线
    plt.subplot(1, 2, 1)
    plt.plot(errors, 'b-', linewidth=1, alpha=0.7)
    plt.axhline(error_dict['mean'], color='r', linestyle='--', label=f"平均: {error_dict['mean']:.4f}")
    plt.axhline(error_dict['median'], color='g', linestyle='--', label=f"中位数: {error_dict['median']:.4f}")
    plt.fill_between(range(len(errors)), 
                     error_dict['mean'] - error_dict['std'], 
                     error_dict['mean'] + error_dict['std'], 
                     alpha=0.2, color='orange', label=f"±1σ")
    
    plt.xlabel('点索引', fontsize=12)
    plt.ylabel('误差', fontsize=12)
    plt.title('点对点误差分布', fontsize=14, fontweight='bold')
    plt.legend(loc='upper right')
    plt.grid(True, alpha=0.3)
    
    # 子图：误差直方图
    plt.subplot(1, 2, 2)
    plt.hist(errors, bins=50, color='steelblue', alpha=0.7, edgecolor='black')
    plt.axvline(error_dict['mean'], color='r', linestyle='--', linewidth=2, label=f"平均")
    plt.axvline(error_dict['median'], color='g', linestyle='--', linewidth=2, label=f"中位数")
    plt.xlabel('误差值', fontsize=12)
    plt.ylabel('频数', fontsize=12)
    plt.title('误差分布直方图', fontsize=14, fontweight='bold')
    plt.legend()
    plt.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    
    # 保存图像
    save_path = os.path.join(os.path.dirname(SOURCE_CSV), "error_plot.png")
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n误差图已保存: {save_path}")
    
    plt.show()


# ============================================================================
#                              可视化
# ============================================================================

def visualize_stages_matplotlib(stages_data, config=MATLOT_VIS_CONFIG):
    """
    使用matplotlib可视化各阶段轨迹（分两页共六幅图）
    
    参数:
        stages_data: dict, 包含各阶段数据
            {
                'original_source': np.array,
                'original_target': np.array,
                'pre_smoothed_source': np.array,
                'pre_smoothed_target': np.array,
                'resampled_source': np.array,
                'resampled_target': np.array,
                'post_smoothed_source': np.array,
                'post_smoothed_target': np.array,
                'aligned_source': np.array,
                'aligned_target': np.array,
            }
        config: dict, 可视化配置
    """
    from mpl_toolkits.mplot3d import Axes3D

    def _set_equal_3d_scale(ax, points_list):
        """让3D坐标轴使用一致的空间刻度，避免轨迹视觉拉伸。"""
        valid = [p for p in points_list if p is not None and len(p) > 0]
        if not valid:
            return

        all_points = np.vstack(valid)
        mins = all_points.min(axis=0)
        maxs = all_points.max(axis=0)
        center = (mins + maxs) / 2.0
        max_span = float(np.max(maxs - mins))

        # 退化情况：所有点几乎重合时给一个最小显示范围
        if max_span < 1e-6:
            max_span = 1.0

        half = max_span / 2.0
        ax.set_xlim(center[0] - half, center[0] + half)
        ax.set_ylim(center[1] - half, center[1] + half)
        ax.set_zlim(center[2] - half, center[2] + half)

        # 新版Matplotlib可进一步固定立方体比例
        if hasattr(ax, "set_box_aspect"):
            ax.set_box_aspect((1, 1, 1))
    
    show_stages = config["show_stages"]
    point_size = config["point_size"]
    alpha = config["alpha"]
    line_width = config["line_width"]
    show_lines = config["show_lines"]
    
    # 统计需要显示的图数量
    stages_to_show = []
    if show_stages["original"]:
        stages_to_show.append(("原始轨迹", 'original_source', 'original_target'))
    if show_stages["pre_smoothed"]:
        stages_to_show.append(("预平滑后轨迹", 'pre_smoothed_source', 'pre_smoothed_target'))
    if show_stages["resampled"]:
        stages_to_show.append(("重采样后轨迹", 'resampled_source', 'resampled_target'))
    if show_stages["post_smoothed"]:
        stages_to_show.append(("后平滑后轨迹", 'post_smoothed_source', 'post_smoothed_target'))
    if show_stages["aligned"]:
        stages_to_show.append(("MLS配准后轨迹", 'aligned_source', 'aligned_target'))
    if show_stages["comparison"]:
        stages_to_show.append(("配准后对比", 'aligned_source', 'aligned_target'))
    
    if len(stages_to_show) == 0:
        print("  ⚠️ 未选择任何可视化阶段，跳过可视化")
        return
    
    # 分3页显示（每页2个子图）
    n_stages = len(stages_to_show)
    n_page1 = min(2, n_stages)
    n_page2 = min(2, max(0, n_stages - 2))
    n_page3 = max(0, n_stages - 4)
    
    # === 第一页（前2个图）===
    if n_page1 > 0:
        fig1 = plt.figure(figsize=config["figsize"], dpi=config["dpi"])
        fig1.suptitle('轨迹处理流程可视化 - 第1页', fontsize=16, fontweight='bold')
        
        for i in range(n_page1):
            title, *keys = stages_to_show[i]
            ax = fig1.add_subplot(1, 2, i+1, projection='3d')
            src = None
            tgt = None
            
            # 绘制源轨迹（红色）
            if keys[0] in stages_data:
                src = stages_data[keys[0]]
                if show_lines:
                    ax.plot(src[:, 0], src[:, 1], src[:, 2], 
                           'r-', linewidth=line_width, alpha=alpha, label='源轨迹')
                ax.scatter(src[:, 0], src[:, 1], src[:, 2], 
                          c='red', s=point_size, alpha=alpha)
            
            # 绘制目标轨迹（蓝色）
            if keys[1] in stages_data:
                tgt = stages_data[keys[1]]
                if show_lines:
                    ax.plot(tgt[:, 0], tgt[:, 1], tgt[:, 2], 
                           'b-', linewidth=line_width, alpha=alpha, label='目标轨迹')
                ax.scatter(tgt[:, 0], tgt[:, 1], tgt[:, 2], 
                          c='blue', s=point_size, alpha=alpha)
            
            ax.set_xlabel('X (mm)')
            ax.set_ylabel('Y (mm)')
            ax.set_zlabel('Z (mm)')
            ax.set_title(title, fontsize=12, fontweight='bold')
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)
            _set_equal_3d_scale(ax, [src, tgt])
        
        plt.tight_layout()
    
    # === 第二页（中间2个图）===
    if n_page2 > 0:
        fig2 = plt.figure(figsize=config["figsize"], dpi=config["dpi"])
        fig2.suptitle('轨迹处理流程可视化 - 第2页', fontsize=16, fontweight='bold')
        
        for i in range(n_page2):
            title, *keys = stages_to_show[2 + i]
            ax = fig2.add_subplot(1, 2, i+1, projection='3d')
            src = None
            tgt = None
            
            # 绘制源轨迹（红色）
            if keys[0] in stages_data:
                src = stages_data[keys[0]]
                if show_lines:
                    ax.plot(src[:, 0], src[:, 1], src[:, 2], 
                           'r-', linewidth=line_width, alpha=alpha, label='源轨迹')
                ax.scatter(src[:, 0], src[:, 1], src[:, 2], 
                          c='red', s=point_size, alpha=alpha)
            
            # 绘制目标轨迹（蓝色）
            if keys[1] in stages_data:
                tgt = stages_data[keys[1]]
                if show_lines:
                    ax.plot(tgt[:, 0], tgt[:, 1], tgt[:, 2], 
                           'b-', linewidth=line_width, alpha=alpha, label='目标轨迹')
                ax.scatter(tgt[:, 0], tgt[:, 1], tgt[:, 2], 
                          c='blue', s=point_size, alpha=alpha)
            
            ax.set_xlabel('X (mm)')
            ax.set_ylabel('Y (mm)')
            ax.set_zlabel('Z (mm)')
            ax.set_title(title, fontsize=12, fontweight='bold')
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)
            _set_equal_3d_scale(ax, [src, tgt])
        
        plt.tight_layout()
    
    # === 第三页（最后2个图）===
    if n_page3 > 0:
        fig3 = plt.figure(figsize=config["figsize"], dpi=config["dpi"])
        fig3.suptitle('轨迹处理流程可视化 - 第3页', fontsize=16, fontweight='bold')
        
        for i in range(n_page3):
            title, *keys = stages_to_show[4 + i]
            ax = fig3.add_subplot(1, 2, i+1, projection='3d')
            src = None
            tgt = None
            
            # 绘制源轨迹（红色）
            if keys[0] in stages_data:
                src = stages_data[keys[0]]
                if show_lines:
                    ax.plot(src[:, 0], src[:, 1], src[:, 2], 
                           'r-', linewidth=line_width, alpha=alpha, label='源轨迹')
                ax.scatter(src[:, 0], src[:, 1], src[:, 2], 
                          c='red', s=point_size, alpha=alpha)
            
            # 绘制目标轨迹（蓝色）
            if keys[1] in stages_data:
                tgt = stages_data[keys[1]]
                if show_lines:
                    ax.plot(tgt[:, 0], tgt[:, 1], tgt[:, 2], 
                           'b-', linewidth=line_width, alpha=alpha, label='目标轨迹')
                ax.scatter(tgt[:, 0], tgt[:, 1], tgt[:, 2], 
                          c='blue', s=point_size, alpha=alpha)
            
            ax.set_xlabel('X (mm)')
            ax.set_ylabel('Y (mm)')
            ax.set_zlabel('Z (mm)')
            ax.set_title(title, fontsize=12, fontweight='bold')
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)
            _set_equal_3d_scale(ax, [src, tgt])
        
        plt.tight_layout()
    
    # 保存图片（可选）
    if config["save_figures"]:
        if n_page1 > 0:
            save_path_1 = config["save_path"].replace(".png", "_page1.png")
            fig1.savefig(save_path_1, dpi=config["dpi"], bbox_inches='tight')
            print(f"  ✅ 第1页已保存: {save_path_1}")
        if n_page2 > 0:
            save_path_2 = config["save_path"].replace(".png", "_page2.png")
            fig2.savefig(save_path_2, dpi=config["dpi"], bbox_inches='tight')
            print(f"  ✅ 第2页已保存: {save_path_2}")
        if n_page3 > 0:
            save_path_3 = config["save_path"].replace(".png", "_page3.png")
            fig3.savefig(save_path_3, dpi=config["dpi"], bbox_inches='tight')
            print(f"  ✅ 第3页已保存: {save_path_3}")
    
    plt.show()
    print("  ✅ Matplotlib可视化完成")


def visualize(source, target, T, errors=None):
    """可视化配准结果（Open3D方式）"""
    src_pcd = o3d.geometry.PointCloud()
    src_pcd.points = o3d.utility.Vector3dVector(source)
    
    tgt_pcd = o3d.geometry.PointCloud()
    tgt_pcd.points = o3d.utility.Vector3dVector(target)
    
    src_transformed = copy.deepcopy(src_pcd)
    src_transformed.transform(T)
    
    if errors is not None and VIS_PARAMS["show_error_color"]:
        err_norm = (errors - errors.min()) / (errors.max() - errors.min() + 1e-10)
        colors = np.zeros((len(errors), 3))
        colors[:, 0] = err_norm          # Red
        colors[:, 1] = 1 - err_norm      # Green
        src_transformed.colors = o3d.utility.Vector3dVector(colors)
    else:
        src_transformed.paint_uniform_color([1, 0, 0])  # 红色
    
    tgt_pcd.paint_uniform_color([0, 0, 1])  # 蓝色
    
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=VIS_PARAMS["coord_frame_size"], origin=[0, 0, 0])
    
    if errors is not None and VIS_PARAMS["show_error_color"]:
        print("\n可视化: 绿=误差小, 红=误差大, 蓝=目标点云")
    else:
        print("\n可视化: 红=源点云(配准后), 蓝=目标点云")
    o3d.visualization.draw_geometries([src_transformed, tgt_pcd, coord],
                                       window_name="配准结果", width=1280, height=720)


def visualize_segmented(transformed_points, target, errors=None):
    """可视化分段配准结果（非刚性变换结果）"""
    src_pcd = o3d.geometry.PointCloud()
    src_pcd.points = o3d.utility.Vector3dVector(transformed_points)
    
    tgt_pcd = o3d.geometry.PointCloud()
    tgt_pcd.points = o3d.utility.Vector3dVector(target)
    
    if errors is not None and VIS_PARAMS["show_error_color"]:
        err_norm = (errors - errors.min()) / (errors.max() - errors.min() + 1e-10)
        colors = np.zeros((len(errors), 3))
        colors[:, 0] = err_norm          # Red
        colors[:, 1] = 1 - err_norm      # Green
        src_pcd.colors = o3d.utility.Vector3dVector(colors)
    else:
        src_pcd.paint_uniform_color([1, 0, 0])  # 红色
    
    tgt_pcd.paint_uniform_color([0, 0, 1])  # 蓝色
    
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=VIS_PARAMS["coord_frame_size"], origin=[0, 0, 0])
    
    if errors is not None and VIS_PARAMS["show_error_color"]:
        print("\n可视化(分段配准): 绿=误差小, 红=误差大, 蓝=目标点云")
    else:
        print("\n可视化(分段配准): 红=变换后点云, 蓝=目标点云")
    o3d.visualization.draw_geometries([src_pcd, tgt_pcd, coord],
                                       window_name="分段配准结果", width=1280, height=720)


# ============================================================================
#                              主流程
# ============================================================================

def pre_smooth(points, params):
    """
    预平滑处理（在弧长重采样前执行）
    目的：消除噪声对弧长计算的累积影响
    支持方法: gaussian / bspline / rdp_pchip
    """
    if not params["enable"]:
        return points
    
    method = params["method"]
    if method == "gaussian":
        return smooth_gaussian(points, params["gaussian_sigma"])
    elif method == "rdp_pchip":
        return smooth_rdp_pchip(
            points, 
            epsilon=params["rdp_epsilon"],
            median_window=params["rdp_median_window"]
        )
    else:
        return smooth_bspline(points, params["bspline_smoothing"], params["bspline_k"])


def main():
    print("=" * 60)
    print(f"轨迹配准 - 方法: {METHOD.upper()}")
    print("=" * 60)
    
    # 加载数据
    print("\n▶ 加载数据")
    print("-" * 40)
    source, _ = load_csv(SOURCE_CSV)
    target, _ = load_csv(TARGET_CSV)
    
    # ⭐ 保存原始弧长（用于导出变换时，基于原始数据）
    original_source_arc_length = compute_arc_length(source)[-1]
    original_target_arc_length = compute_arc_length(target)[-1]
    print(f"  原始弧长 - Source: {original_source_arc_length:.2f}, Target: {original_target_arc_length:.2f}")
    
    # ⭐⭐ Step 2: 预平滑
    # chord_spatial 方法自身消除噪声弧长累积，建议关闭预平滑
    # adaptive_spatial 方法依赖弧长插值，建议开启预平滑以压制噪声
    resample_method = TIME_ALIGN.get("method", "chord_spatial")
    print(f"\n▶ 预平滑检查（当前重采样方法: {resample_method}）")
    print("-" * 40)
    if PRE_SMOOTH["enable"]:
        source_pre = pre_smooth(source, PRE_SMOOTH)
        target_pre = pre_smooth(target, PRE_SMOOTH)
        print(f"  预平滑方法: {PRE_SMOOTH['method']}, sigma={PRE_SMOOTH['gaussian_sigma']}")
        
        # 对比预平滑前后的弧长差异（用于验证噪声影响）
        pre_smooth_arc = compute_arc_length(source_pre)[-1]
        arc_diff_percent = (original_source_arc_length - pre_smooth_arc) / pre_smooth_arc * 100
        print(f"  预平滑后弧长: {pre_smooth_arc:.2f} (噪声导致的弧长高估: {arc_diff_percent:.2f}%)")
        if resample_method == "chord_spatial":
            print(f"  💡 提示：chord_spatial 方法自身不受噪声弧长影响，预平滑可选")
    else:
        source_pre = source
        target_pre = target
        if resample_method == "adaptive_spatial":
            print("  ⚠️  预平滑已关闭，但 adaptive_spatial 依赖弧长插值，建议开启预平滑")
        else:
            print("  ✅ 预平滑已跳过（chord_spatial 弦距采样自身消除噪声弧长累积，无需预平滑）")
    
    # ⭐ Step 3: 时间/弧长对齐
    # 💡 序列号对应模式下跳过重采样，保持帧号对应关系
    if SEQUENCE_ALIGN_MODE["enable"]:
        print("\n▶ 序列号对应模式 - 跳过重采样（保持帧号对应）")
        print("-" * 40)
        if SEQUENCE_ALIGN_MODE.get("verify_frame_count", True):
            if len(source_pre) != len(target_pre):
                print(f"  ❌ 帧数不一致！Source={len(source_pre)}, Target={len(target_pre)}")
                print(f"     序列号对应模式要求帧数完全一致！")
                print(f"     请检查数据或改用弧长模式。")
                return None, None, None
            else:
                print(f"  ✅ 帧数一致: {len(source_pre)} 帧")
        source_aligned = source_pre
        target_aligned = target_pre
        print(f"  跳过重采样，保持原始 {len(source_aligned)} 帧")
    else:
        print("\n▶ 轨迹对齐（基于预平滑后的数据）")
        print("-" * 40)
        source_aligned, target_aligned = align_trajectories(source_pre, target_pre, TIME_ALIGN)
        print(f"  对齐后点数: {len(source_aligned)}")
    
    # ⭐ 保存重采样后的弧长
    resampled_source_arc_length = compute_arc_length(source_aligned)[-1]
    
    # ⭐ Step 4: 后平滑（可选，进一步去噪）
    # 💡 序列号对应模式：仅平滑，不改变点数
    if SEQUENCE_ALIGN_MODE["enable"] and SEQUENCE_ALIGN_MODE.get("pre_smooth_only", True):
        print("\n▶ 序列号对应模式 - 跳过后平滑（已在预平滑阶段完成去噪）")
        print("-" * 40)
        source_proc = source_aligned
        target_proc = target_aligned
        print(f"  使用预平滑数据，{len(source_proc)} 帧")
    else:
        print("\n▶ 后平滑处理")
        print("-" * 40)
        source_proc = preprocess(source_aligned, POST_SMOOTH)
        target_proc = preprocess(target_aligned, POST_SMOOTH)
    
    # 配准前误差
    pre_errors = np.linalg.norm(source_proc - target_proc, axis=1)
    print(f"\n  配准前误差: 平均={np.mean(pre_errors):.4f}, 最大={np.max(pre_errors):.4f}")
    
    # 配准
    print(f"\n▶ 执行配准 ({METHOD})")
    print("-" * 40)
    
    use_segmented_result = False
    use_mls_result = False
    seg_transform = None  # 分段变换对象
    mls_transform = None  # MLS变换对象
    
    if METHOD == "kabsch":
        T = kabsch_align(source_proc, target_proc)
    elif METHOD == "kabsch_icp":
        # Kabsch + ICP混合精配准
        T = kabsch_icp_align(source_proc, target_proc, ICP_PARAMS)
    elif METHOD == "kabsch_segmented":
        # ⭐⭐ 分段Kabsch配准
        if SEQUENCE_ALIGN_MODE["enable"]:
            # ⭐⭐⭐ 序列号对应模式
            T, segmented_transformed, segmented_errors, seg_transform = segmented_kabsch_align_by_sequence(
                source_proc, target_proc, SEGMENTED_PARAMS)
        else:
            # 弧长对应模式（原有逻辑）
            T, segmented_transformed, segmented_errors, seg_transform = segmented_kabsch_align(
                source_proc, target_proc, SEGMENTED_PARAMS)
        use_segmented_result = True
    elif METHOD == "mls":
        # ⭐⭐⭐ 移动最小二乘(MLS)配准
        T, mls_transformed, mls_errors, mls_transform = mls_align(
            source_proc, target_proc, MLS_PARAMS)
        use_mls_result = True
    elif METHOD == "kabsch_iterative":
        # 迭代Kabsch + 异常值剔除
        T, source_proc, target_proc = iterative_kabsch_with_outlier_removal(
            source_proc, target_proc, OUTLIER_REMOVAL, max_iterations=3)
    elif METHOD == "icp":
        T = icp_align(source_proc, target_proc, ICP_PARAMS)
    elif METHOD == "ransac":
        T = ransac_gicp_align(source_proc, target_proc, RANSAC_PARAMS, ICP_PARAMS)
    else:
        raise ValueError(f"未知方法: {METHOD}")
    
    # 误差计算
    if use_segmented_result or use_mls_result:
        # 使用分段配准或MLS的结果
        errors = segmented_errors if use_segmented_result else mls_errors
        
        # ⭐ 对分段结果也进行异常值剔除统计
        if OUTLIER_REMOVAL["enable"]:
            if OUTLIER_REMOVAL["method"] == "percentile":
                threshold = np.percentile(errors, OUTLIER_REMOVAL["percentile_threshold"])
            else:  # iterative
                threshold = np.mean(errors) + OUTLIER_REMOVAL["iterative_sigma"] * np.std(errors)
            
            outlier_mask = errors > threshold
            num_outliers = np.sum(outlier_mask)
            print(f"\n  异常值检测: {num_outliers}个点超过阈值 {threshold:.2f}")
            
            # ⭐⭐⭐ 先显示真实的全部点误差统计 ⭐⭐⭐
            all_error_dict = {
                'mean': np.mean(errors),
                'rmse': np.sqrt(np.mean(errors**2)),
                'max': np.max(errors),
                'min': np.min(errors),
                'std': np.std(errors),
                'median': np.median(errors),
                'p95': np.percentile(errors, 95),
            }
            print(f"\n  【⚠️ 真实误差（全部{len(errors)}点，未剔除）】")
            print(f"     平均={all_error_dict['mean']:.4f}, RMSE={all_error_dict['rmse']:.4f}, 最大={all_error_dict['max']:.4f}")
            
            # 统计时排除异常值
            clean_errors = errors[~outlier_mask]
            error_dict = {
                'mean': np.mean(clean_errors),
                'rmse': np.sqrt(np.mean(clean_errors**2)),
                'max': np.max(clean_errors),
                'min': np.min(clean_errors),
                'std': np.std(clean_errors),
                'median': np.median(clean_errors),
                'p95': np.percentile(clean_errors, 95),
                'total_points': len(errors),
                'clean_points': len(clean_errors),
                'outliers': num_outliers,
                # ⭐ 保存真实误差用于对比
                'real_mean': all_error_dict['mean'],
                'real_rmse': all_error_dict['rmse'],
                'real_max': all_error_dict['max'],
            }
            print(f"  【剔除{num_outliers}异常值后（{len(clean_errors)}点）】")
            print(f"     平均={error_dict['mean']:.4f}, RMSE={error_dict['rmse']:.4f}, 最大={error_dict['max']:.4f}")
        else:
            error_dict = {
                'mean': np.mean(errors),
                'rmse': np.sqrt(np.mean(errors**2)),
                'max': np.max(errors),
                'min': np.min(errors),
                'std': np.std(errors),
                'median': np.median(errors),
                'p95': np.percentile(errors, 95),
            }
    else:
        error_dict, errors = compute_error(source_proc, target_proc, T)
    
    # 结果汇总
    print("\n" + "=" * 60)
    print(">>> 配准结果 <<<")
    print("=" * 60)
    
    # ⭐ 显示真实误差（如果有）
    if 'real_mean' in error_dict:
        print(f"\n【⚠️ 真实误差统计（全部点）】")
        print(f"  平均: {error_dict['real_mean']:.4f}")
        print(f"  RMSE: {error_dict['real_rmse']:.4f}")
        print(f"  最大: {error_dict['real_max']:.4f}")
        print(f"  ---")
        print(f"  异常值: {error_dict['outliers']}点 ({error_dict['outliers']/error_dict['total_points']*100:.1f}%)")
    
    print(f"\n【误差统计】(剔除异常值后)" if 'outliers' in error_dict else f"\n【误差统计】")
    print(f"  平均: {error_dict['mean']:.4f}")
    print(f"  RMSE: {error_dict['rmse']:.4f}")
    print(f"  最大: {error_dict['max']:.4f}")
    print(f"  最小: {error_dict['min']:.4f}")
    print(f"  中位数: {error_dict['median']:.4f}")
    print(f"  P95: {error_dict['p95']:.4f}")
    
    R = T[:3, :3]
    t = T[:3, 3]
    angle = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    
    print(f"\n【变换】")
    print(f"  旋转: {angle:.4f}°")
    print(f"  平移: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]")
    
    improve = (1 - error_dict['mean'] / np.mean(pre_errors)) * 100
    print(f"\n【改善】{improve:.2f}%")
    
    # ========== ⭐ 导出变换矩阵 ==========
    print("\n▶ 导出变换矩阵")
    print("-" * 40)
    transform_path = os.path.join(os.path.dirname(SOURCE_CSV), "transform_matrix.json")
    export_transform(T, transform_path, error_dict)
    
    # ⭐⭐ 如果是分段配准，额外保存分段变换
    if use_segmented_result and seg_transform is not None:
        seg_transform_path = os.path.join(os.path.dirname(SOURCE_CSV), "segmented_transform.json")
        if SEQUENCE_ALIGN_MODE["enable"]:
            # 序列号模式保存
            seg_transform.save(seg_transform_path, mode="sequence_aligned")
        else:
            # 弧长模式保存（传入原始弧长）
            seg_transform.save(seg_transform_path, original_arc_length=original_source_arc_length)
    
    # ⭐⭐⭐ 如果是MLS配准，保存MLS变换
    # ⭐ 保存到与 SOURCE_CSV 相同的目录（与 convert_segmented_transform.py 默认路径一致！）
    if use_mls_result and mls_transform is not None:
        mls_transform_path = os.path.join(os.path.dirname(SOURCE_CSV), "mls_transform.json")
        mls_transform.save(mls_transform_path)
    
    # 绘制误差折线图
    print("\n▶ 绘制误差分析图")
    print("-" * 40)
    plot_error_curve(errors, error_dict)
    
    # ⭐⭐ 收集各阶段数据用于可视化
    aligned_source = None
    if use_segmented_result:
        aligned_source = segmented_transformed
    elif use_mls_result:
        aligned_source = mls_transformed
    else:
        # 使用单一变换矩阵
        aligned_source = apply_transform(source_proc, T)
    
    stages_data = {
        'original_source': source,
        'original_target': target,
        'pre_smoothed_source': source_pre,
        'pre_smoothed_target': target_pre,
        'resampled_source': source_aligned,
        'resampled_target': target_aligned,
        'post_smoothed_source': source_proc,
        'post_smoothed_target': target_proc,
        'aligned_source': aligned_source,
        'aligned_target': target_proc,  # target不变
    }
    
    # 3D可视化
    print("\n▶ 3D轨迹可视化")
    print("-" * 40)
    
    if MATLOT_VIS_CONFIG["enable"]:
        # 使用matplotlib可视化
        print("  使用Matplotlib多阶段可视化...")
        visualize_stages_matplotlib(stages_data, MATLOT_VIS_CONFIG)
    
    if MATLOT_VIS_CONFIG.get("use_open3d", False):
        # 使用Open3D可视化（原方式）
        print("  使用Open3D可视化...")
        if use_segmented_result:
            visualize_segmented(segmented_transformed, target_proc, errors)
        elif use_mls_result:
            visualize_segmented(mls_transformed, target_proc, errors)
        else:
            visualize(source_proc, target_proc, T, errors)
    
    return T, error_dict, seg_transform if use_segmented_result else mls_transform


# ============================================================================
#                     ⭐⭐ 变换导出与应用（核心功能）
# ============================================================================

def export_transform(T, filepath, error_dict=None):
    """
    导出变换矩阵到JSON文件
    
    数学表示：
    P_target = T @ P_source (齐次坐标)
    P_target = R @ P_source + t (3D坐标)
    
    参数：
        T: 4x4变换矩阵
        filepath: 保存路径
        error_dict: 误差统计信息
    """
    R = T[:3, :3]
    t = T[:3, 3]
    
    # 计算旋转角度和轴
    angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    
    # 将矩阵转为列表以便JSON序列化
    transform_data = {
        "description": "SOURCE点云到TARGET点云的刚性变换",
        "formula": "P_target = R @ P_source + t",
        "transform_matrix_4x4": T.tolist(),
        "rotation_matrix_3x3": R.tolist(),
        "translation_vector": t.tolist(),
        "rotation_angle_deg": np.degrees(angle),
        "source_file": SOURCE_CSV,
        "target_file": TARGET_CSV,
    }
    
    if error_dict:
        transform_data["registration_error"] = {
            "mean": float(error_dict['mean']),
            "rmse": float(error_dict['rmse']),
            "max": float(error_dict['max']),
        }
    
    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(transform_data, f, indent=2, ensure_ascii=False)
    
    print(f"  变换矩阵已保存: {filepath}")
    print(f"\n  【应用公式】")
    print(f"  P_target = R @ P_source + t")
    print(f"  其中:")
    print(f"    R (旋转矩阵 3×3):")
    for row in R:
        print(f"      [{row[0]:12.8f}, {row[1]:12.8f}, {row[2]:12.8f}]")
    print(f"    t (平移向量): [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]")
    
    return transform_data


def load_transform(filepath):
    """从JSON文件加载变换矩阵"""
    with open(filepath, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    T = np.array(data["transform_matrix_4x4"])
    print(f"  已加载变换矩阵: {filepath}")
    return T, data


def apply_transform(points, T):
    """
    将变换应用到点云
    
    数学公式：
    P' = T @ [P; 1] = R @ P + t
    
    参数：
        points: Nx3 点云数组
        T: 4x4 变换矩阵
    
    返回：
        transformed: Nx3 变换后的点云
    """
    N = len(points)
    # 转为齐次坐标 [x, y, z, 1]
    points_homo = np.hstack([points, np.ones((N, 1))])
    # 应用变换
    transformed_homo = (T @ points_homo.T).T
    # 提取3D坐标
    transformed = transformed_homo[:, :3]
    return transformed


def apply_transform_to_csv(input_csv, output_csv, T, xyz_cols=(3, 4, 5)):
    """
    将变换应用到CSV文件中的点云数据
    
    参数：
        input_csv: 输入CSV文件路径
        output_csv: 输出CSV文件路径
        T: 4x4变换矩阵
        xyz_cols: XYZ列索引
    """
    # 读取原始数据
    with open(input_csv, 'r') as f:
        header = f.readline()
    
    data = np.loadtxt(input_csv, delimiter=',', skiprows=1)
    points = data[:, xyz_cols]
    
    # 应用变换
    transformed = apply_transform(points, T)
    
    # 替换原始数据中的XYZ列
    data[:, xyz_cols[0]] = transformed[:, 0]
    data[:, xyz_cols[1]] = transformed[:, 1]
    data[:, xyz_cols[2]] = transformed[:, 2]
    
    # 保存
    with open(output_csv, 'w') as f:
        f.write(header)
        for row in data:
            f.write(','.join([str(v) for v in row]) + '\n')
    
    print(f"  已变换并保存: {output_csv}")
    print(f"  变换点数: {len(points)}")
    return transformed


# ============================================================================
#                     示例：如何应用变换到新数据
# ============================================================================

def example_apply_transform():
    """
    示例：如何将配准结果应用到其他点云数据
    
    使用方法：
    1. 先运行 main() 获取变换矩阵并保存
    2. 然后调用此函数应用到新数据
    """
    print("\n" + "=" * 60)
    print("示例：应用变换到新数据")
    print("=" * 60)
    
    # 1. 加载之前保存的变换矩阵（全局刚性变换）
    transform_path = os.path.join(os.path.dirname(SOURCE_CSV), "transform_matrix.json")
    T, info = load_transform(transform_path)
    
    # 2. 方式一：直接对numpy数组应用变换
    print("\n【方式一】全局刚性变换（简单但精度较低）:")
    new_points = np.array([
        [100.0, 200.0, 300.0],
        [150.0, 250.0, 350.0],
        [200.0, 300.0, 400.0],
    ])
    transformed_points = apply_transform(new_points, T)
    print(f"  原始点:\n{new_points}")
    print(f"  变换后:\n{transformed_points}")
    
    # 3. 方式二：对CSV文件应用变换
    print("\n【方式二】对CSV文件应用刚性变换:")
    print("  调用: apply_transform_to_csv(input.csv, output.csv, T)")
    
    # 4. 方式三：手动应用公式
    print("\n【方式三】手动应用公式 (Python代码):")
    print("  R = T[:3, :3]  # 旋转矩阵")
    print("  t = T[:3, 3]   # 平移向量")
    print("  P_new = R @ P_old + t  # 或 P_new = (R @ P_old.T).T + t")
    
    # 5. C#/Unity中应用
    print("\n【方式四】C#/Unity中应用:")
    R = T[:3, :3]
    t = T[:3, 3]
    print(f"  // 在Unity中创建变换")
    print(f"  Matrix4x4 transform = new Matrix4x4(")
    for i in range(4):
        row = T[i, :]
        print(f"      new Vector4({row[0]:12.8f}f, {row[1]:12.8f}f, {row[2]:12.8f}f, {row[3]:12.8f}f)" + ("," if i < 3 else ""))
    print(f"  );")
    print(f"  // 应用变换: newPos = transform.MultiplyPoint3x4(oldPos);")


def example_apply_segmented_transform():
    """
    ⭐⭐ 示例：应用分段配准到新轨迹（高精度）
    
    数学原理：
    =========
    分段配准将轨迹分成 N 段，每段有独立变换 T_i
    
    对于轨迹上弧长位置为 s 的点 P：
    P'(s) = Σ w_i(s) × T_i × P
    
    其中权重 w_i(s) 是基于弧长的高斯函数：
    w_i(s) = exp(-(s - s_i)² / (2σ_i²))
    
    这实现了沿轨迹的连续、平滑的非刚性变换
    """
    print("\n" + "=" * 60)
    print("示例：应用分段配准到新轨迹（高精度）")
    print("=" * 60)
    
    # 1. 加载分段变换
    seg_path = os.path.join(os.path.dirname(SOURCE_CSV), "segmented_transform.json")
    
    if not os.path.exists(seg_path):
        print(f"  错误：未找到分段变换文件 {seg_path}")
        print("  请先运行 main() 生成分段变换")
        return
    
    seg_transform = SegmentedTransform.load(seg_path)
    
    # 2. 加载要变换的新轨迹
    print("\n▶ 加载新轨迹数据")
    # 这里用SOURCE_CSV作为示例，实际使用时替换为新的轨迹文件
    new_trajectory, _ = load_csv(SOURCE_CSV)
    print(f"  点数: {len(new_trajectory)}")
    
    # 3. 应用分段变换
    print("\n▶ 应用分段变换")
    transformed_trajectory = seg_transform.transform_trajectory(new_trajectory)
    
    # 4. 验证结果
    # 加载目标轨迹进行对比
    target_trajectory, _ = load_csv(TARGET_CSV)
    
    # 重采样到相同点数进行对比
    min_len = min(len(transformed_trajectory), len(target_trajectory))
    trans_sample = transformed_trajectory[:min_len]
    target_sample = target_trajectory[:min_len]
    
    errors = np.linalg.norm(trans_sample - target_sample, axis=1)
    print(f"\n  变换后误差统计:")
    print(f"    平均: {np.mean(errors):.4f}")
    print(f"    RMSE: {np.sqrt(np.mean(errors**2)):.4f}")
    print(f"    最大: {np.max(errors):.4f}")
    
    # 5. 保存变换后的轨迹
    output_path = os.path.join(os.path.dirname(SOURCE_CSV), "transformed_trajectory.csv")
    np.savetxt(output_path, transformed_trajectory, delimiter=',', 
               header='X,Y,Z', comments='')
    print(f"\n  变换后轨迹已保存: {output_path}")
    
    return transformed_trajectory


def apply_segmented_transform_to_csv(input_csv, output_csv, seg_transform_path, xyz_cols=(3, 4, 5)):
    """
    将分段变换应用到CSV文件中的轨迹数据
    
    参数：
        input_csv: 输入CSV文件路径
        output_csv: 输出CSV文件路径  
        seg_transform_path: 分段变换JSON文件路径
        xyz_cols: XYZ列索引
    """
    # 加载分段变换
    seg_transform = SegmentedTransform.load(seg_transform_path)
    
    # 读取原始数据
    with open(input_csv, 'r') as f:
        header = f.readline()
    
    data = np.loadtxt(input_csv, delimiter=',', skiprows=1)
    points = data[:, xyz_cols]
    
    # 应用分段变换
    transformed = seg_transform.transform_trajectory(points)
    
    # 替换原始数据中的XYZ列
    data[:, xyz_cols[0]] = transformed[:, 0]
    data[:, xyz_cols[1]] = transformed[:, 1]
    data[:, xyz_cols[2]] = transformed[:, 2]
    
    # 保存
    with open(output_csv, 'w') as f:
        f.write(header)
        for row in data:
            f.write(','.join([str(v) for v in row]) + '\n')
    
    print(f"  已应用分段变换并保存: {output_csv}")
    print(f"  变换点数: {len(points)}")
    return transformed


if __name__ == "__main__":
    main()
    
    # 取消注释以查看应用变换的示例
    # example_apply_transform()
    # example_apply_segmented_transform()

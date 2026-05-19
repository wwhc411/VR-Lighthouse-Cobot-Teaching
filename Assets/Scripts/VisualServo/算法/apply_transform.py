"""
高精度分段配准变换应用工具
============================

将分段配准结果应用到新的CSV轨迹数据

使用方法：
---------
1. 命令行模式：
   python apply_transform.py input.csv output.csv
   
2. 交互模式（直接运行）：
   python apply_transform.py

3. Python代码调用：
   from apply_transform import transform_csv
   transform_csv("input.csv", "output.csv")

Author: AI Assistant
Date: 2026-02-04
"""

import numpy as np
import json
import os
import sys
from scipy import interpolate
from scipy.ndimage import gaussian_filter1d

# 默认变换文件路径
# ⭐ 注意：MLS变换必须先经过convert_segmented_transform.py坐标系转换！
#    灯塔坐标系: mls_transform.json（训练输出）
#    机械臂坐标系: mls_transform_robot.json（转换后，用于复用）
DEFAULT_TRANSFORM_PATH = r"E:\Unity cangku\lighthouse_3.4（走数据备份）\Assets\Scripts\VisualServo\算法\圆形\mls_transform_robot.json"
# ========== ⭐⭐⭐ 空间重采样参数（与tunable_registration.py保持一致！）==========
ADAPTIVE_SPATIAL_RESAMPLE = {
    # --- 新版 chord_spatial 参数（弦距采样，解决海岸线悖论）---
    "delta_d": 1.3,                  # ⭐ 弦距步长门限(mm)
    "densify_factor": 50,            # ⭐ 插值加密倍数
    # --- 旧版 adaptive_spatial 参数（弧长插值，需配合预平滑）---
    "noise_level": 0.5,              # 噪声水平估计(mm)
    "noise_suppression_factor": 1.8, # 噪声抑制系数
    # --- 共享参数 ---
    "min_samples": 100,              # 最小采样点数
    "max_samples": 5000,             # 最大采样点数
}

# ========== ⭐⭐⭐ 混合变换参数（复用时降低分段过拟合）==========
# 💡 分段变换(75段)在配准数据上RMSE=0.38mm，但复用时易过校正(max=11mm)
# 全局变换泛化性更好(max=3.3mm)，混合后取长补短(α=0.3时max=3.2mm)
BLEND_CONFIG = {
    "enable": False,                 # ⭐ 是否启用混合变换（复用时强烈建议开启）
    "alpha": 0.3,                   # 分段权重: 0.0=纯全局, 1.0=纯分段, 0.3=推荐
    "global_transform_path": None,  # 全局变换文件路径（None=自动从同目录查找transform_matrix.json）
}

# ========== ⭐⭐⭐ MLS变换参数（复用端）==========
# 💡 MLS变换支持两种复用模式：
#    - "grid"（网格插值）: O(1)每点，快速，适用于大批量复用（推荐）
#    - "full"（完整计算）: O(M)每点，每个点执行加权Kabsch SVD，精度最高但较慢
MLS_CONFIG = {
    "use_mode": "full",              # ⭐ "grid"=网格插值(快速推荐) / "full"=完整计算(高精度)
}

# ========== ⭐⭐ 预平滑参数 ==========
# 💡 chord_spatial （弦距采样自身消除噪声弧长累积）
# 💡 adaptive_spatial 时建议开启（弧长插值需预平滑压制噪声）
# 💡 与 tunable_registration.py 保持一致！
PRE_SMOOTH = {
    "enable": True,                # ⭐ 是否启用预平滑（建议开启，尤其是adaptive_spatial）
    "method": "gaussian",          # "gaussian" / "bspline" / "rdp_pchip"
    "gaussian_sigma": 1.5,          # [Gaussian] 高斯核半宽 ★已改为 mm 单位（空间几何平滑）
                                   # 3mm=轻平滑  5mm=中度（推荐）  10mm=强平滑
    "bspline_smoothing": 3.0,       # [B-Spline] 平滑因子
    "bspline_k": 3,                 # [B-Spline] 次数
    "rdp_epsilon": 2,             # [RDP+PCHIP] RDP简化阈值(mm)，越大越激进地剔除抖动点
    "rdp_median_window": 9,          # [RDP+PCHIP] 中值滤波窗口（奇数3-9），抑制离群点防止RDP误判
}

# ============================================================================
#                        ⭐ 后处理参数（与配准工具保持一致！）
# ============================================================================
# 💡 在自适应空间重采样【之后】应用
PREPROCESS_CONFIG = {
    "enable_smoothing": True,       # 是否启用后平滑（配准时用了平滑）
    "gaussian_sigma": 5,          # ⭐ 后平滑高斯核半宽 ★mm单位（与POST_SMOOTH["gaussian_sigma"]一致）
    "num_resample": 5000,           # 弧长重采样点数（仅在传统模式使用）
    "resample_method": "adaptive_spatial",  # ⭐ "chord_spatial"(新) / "adaptive_spatial"(旧) / "arc_length"
}


def compute_arc_length(points):
    """计算累积弧长"""
    diffs = np.diff(points, axis=0)
    segment_lengths = np.linalg.norm(diffs, axis=1)
    arc_length = np.concatenate([[0], np.cumsum(segment_lengths)])
    return arc_length


def smooth_gaussian(points, sigma=1.0):
    """
    空间几何高斯平滑（与配准工具一致）

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
    """B-Spline平滑（与配准工具一致）"""
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


def pre_smooth(points, params):
    """预平滑处理（在重采样前，消除噪声对弧长的影响）
    支持方法: gaussian / bspline / rdp_pchip
    """
    if not params["enable"]:
        return points
    
    method = params["method"]
    if method == "gaussian":
        sigma = params["gaussian_sigma"]
        return smooth_gaussian(points, sigma)
    elif method == "bspline":
        s = params["bspline_smoothing"]
        k = params["bspline_k"]
        return smooth_bspline(points, s, k)
    elif method == "rdp_pchip":
        return smooth_rdp_pchip(
            points, 
            epsilon=params["rdp_epsilon"],
            median_window=params["rdp_median_window"]
        )
    else:
        print(f"  ⚠️ 未知平滑方法: {method}, 跳过预平滑")
        return points


def resample_by_arc_length(points, num_samples):
    """按弧长等距重采样（传统方法，可能受噪声影响）"""
    arc_length = compute_arc_length(points)
    total_length = arc_length[-1]
    
    target_arc = np.linspace(0, total_length, num_samples)
    
    resampled = np.zeros((num_samples, 3))
    for i in range(3):
        f = interpolate.interp1d(arc_length, points[:, i], kind='linear', fill_value='extrapolate')
        resampled[:, i] = f(target_arc)
    
    return resampled, total_length


def adaptive_spatial_resample_legacy(points, noise_level=0.5, noise_suppression_factor=2.0,
                                      min_samples=500, max_samples=5000, verbose=True):
    """
    旧版自适应空间重采样（弧长插值方式）
    
    基于累积弧长做 linspace 等距插值。
    ⚠️ 此方法的弧长计算会受噪声污染（海岸线效应），
    建议配合 PRE_SMOOTH 预平滑使用。如需消除海岸线悖论，
    请改用 chord_spatial_resample()。
    
    参数:
        noise_level: 噪声水平估计(mm)
        noise_suppression_factor: 噪声抑制系数
    """
    N = len(points)
    
    # Step 1: 计算累积弧长
    arc_lengths = compute_arc_length(points)
    total_length = arc_lengths[-1]
    
    # Step 2: 基于噪声水平自适应确定采样间隔
    delta_d = noise_level * noise_suppression_factor
    num_samples = int(total_length / delta_d) if delta_d > 0 else N
    num_samples = np.clip(num_samples, min_samples, max_samples)
    
    # Step 3: 等弧长插值
    target_arc_lengths = np.linspace(0, total_length, num_samples)
    resampled = np.zeros((num_samples, 3))
    for dim in range(3):
        resampled[:, dim] = np.interp(target_arc_lengths, arc_lengths, points[:, dim])
    
    # Step 4: 统计
    actual_delta_d = total_length / (num_samples - 1) if num_samples > 1 else 0
    
    sampling_info = {
        'original_points': N,
        'resampled_points': num_samples,
        'total_length_mm': total_length,
        'actual_delta_d_mm': actual_delta_d,
        'noise_level_mm': noise_level,
        'delta_d_mm': delta_d,
    }
    
    if verbose:
        print(f"  【自适应弧长重采样（旧版）】 {N}点 → {num_samples}点")
        print(f"    弧长: {total_length:.2f}mm, 步长Δd: {actual_delta_d:.3f}mm")
        print(f"    噪声水平: {noise_level:.2f}mm, 抑制系数: {noise_suppression_factor:.1f}")
    
    return resampled, sampling_info


def dense_linear_interpolate(points, densify_factor=10):
    """
    密集线性插值：将轨迹点序列加密，使其近似连续
    """
    N = len(points)
    if N < 2:
        return points.copy()
    
    total_dense = (N - 1) * densify_factor + 1
    dense_points = np.zeros((total_dense, 3))
    
    for seg_i in range(N - 1):
        p_start = points[seg_i]
        p_end = points[seg_i + 1]
        for k in range(densify_factor):
            t = k / densify_factor
            dense_points[seg_i * densify_factor + k] = p_start + t * (p_end - p_start)
    
    dense_points[-1] = points[-1]
    return dense_points


def chord_spatial_resample(points, delta_d=1.0, densify_factor=10,
                            min_samples=500, max_samples=5000, verbose=True):
    """
    ⭐⭐⭐ 真3D弦距重采样（与tunable_registration.py保持一致）
    
    1. 密集线性插值使轨迹近似连续
    2. 弦距采样：||p[i] - p_last_sampled|| ≥ Δd 时采样
    3. 噪声往返不增加弦距，彻底消除海岸线效应
    """
    N_original = len(points)
    
    # Step 1: 密集插值
    dense_points = dense_linear_interpolate(points, densify_factor=densify_factor)
    N_dense = len(dense_points)
    
    # Step 2: 弦距离行走采样（直线距离，非路径累积）
    sampled_indices = [0]
    last_sampled_idx = 0
    
    for i in range(1, N_dense):
        chord_dist = np.linalg.norm(dense_points[i] - dense_points[last_sampled_idx])
        if chord_dist >= delta_d:
            sampled_indices.append(i)
            last_sampled_idx = i
    
    if sampled_indices[-1] != N_dense - 1:
        sampled_indices.append(N_dense - 1)
    
    resampled = dense_points[sampled_indices]
    
    # Step 3: 边界约束
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
        rough_length = np.sum(np.linalg.norm(np.diff(resampled, axis=0), axis=1))
        delta_d_adj = rough_length / min_samples
        sampled_indices = _chord_resample(dense_points, delta_d_adj)
        resampled = dense_points[sampled_indices]
    elif len(resampled) > max_samples:
        rough_length = np.sum(np.linalg.norm(np.diff(resampled, axis=0), axis=1))
        delta_d_adj = rough_length / max_samples
        sampled_indices = _chord_resample(dense_points, delta_d_adj)
        resampled = dense_points[sampled_indices]
    
    # Step 4: 统计
    true_arc_length = np.sum(np.linalg.norm(np.diff(resampled, axis=0), axis=1))
    actual_steps = np.linalg.norm(np.diff(resampled, axis=0), axis=1)
    actual_delta_d = np.mean(actual_steps)
    
    sampling_info = {
        'original_points': N_original,
        'resampled_points': len(resampled),
        'total_length_mm': true_arc_length,
        'actual_delta_d_mm': actual_delta_d,
    }
    
    if verbose:
        print(f"  【弦距重采样(chord_spatial)】 {N_original}点 → {len(resampled)}点")
        print(f"    弦距弧长: {true_arc_length:.2f}mm, 平均步长: {actual_delta_d:.3f}mm")
    
    return resampled, sampling_info


class SegmentedTransform:
    """
    分段变换类 - 加载和应用分段配准结果
    
    数学公式：
    P'(s) = Σ w_i(s) × T_i × P
    
    其中 w_i(s) 是基于弧长的高斯权重
    """
    
    def __init__(self):
        self.segment_transforms = []
        self.segment_centers = []
        self.segment_ranges = []
        # ⭐ 归一化弧长属性（解决不同长度轨迹的弧长对应问题）
        self.normalized_segment_centers = []
        self.normalized_segment_ranges = []
        self.total_arc_length = 0
        self.original_arc_length = 0  # ⭐ 原始数据弧长
        self.num_segments = 0
        # ⭐⭐ 新增：与配准时一致的权重控制参数
        self.segment_depths = []  # 细分深度（用于深度加权）
        self.segment_rmses = []   # 分段 RMSE（可选）
        # ⭐⭐⭐ 序列号对应模式属性
        self.mode = "arc_length"          # "arc_length" 或 "sequence_aligned"
        self.total_frames = 0             # 总帧数（序列号模式）
        self.segment_center_frames = []   # 每段中心帧号
        self.segment_frame_ranges = []    # 每段帧号范围 [(start, end), ...]
        
    @classmethod
    def load(cls, filepath):
        """从JSON加载分段变换（自动识别弧长模式 / 序列号模式）"""
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        obj = cls()
        obj.num_segments = data["num_segments"]
        obj.mode = data.get("mode", "arc_length")
        
        if obj.mode == "sequence_aligned":
            # ⭐⭐⭐ 序列号对应模式
            obj.total_frames = data["total_frames"]
            obj.total_arc_length = data.get("total_arc_length", float(obj.total_frames))
            obj.original_arc_length = obj.total_arc_length
            
            for seg in data["segments"]:
                obj.segment_transforms.append(np.array(seg["transform_4x4"]))
                obj.segment_center_frames.append(seg["center_frame"])
                obj.segment_frame_ranges.append(tuple(seg["frame_range"]))
                obj.segment_depths.append(seg.get("refine_depth", 0))
                obj.segment_rmses.append(seg.get("segment_rmse", 0.0))
                # 兼容弧长字段
                obj.segment_centers.append(float(seg["center_frame"]))
                obj.segment_ranges.append((float(seg["frame_range"][0]), float(seg["frame_range"][1])))
            
            print(f"  ⭐ 序列号对应模式: {filepath}")
            print(f"  共 {obj.num_segments} 段, 总帧数 {obj.total_frames}")
        else:
            # 弧长对应模式（原有逻辑）
            obj.total_arc_length = data["total_arc_length"]
            obj.original_arc_length = data.get("original_arc_length", data["total_arc_length"])
            
            for seg in data["segments"]:
                obj.segment_transforms.append(np.array(seg["transform_4x4"]))
                obj.segment_centers.append(seg["arc_length_center"])
                obj.segment_ranges.append(tuple(seg["arc_length_range"]))
                
                obj.segment_depths.append(seg.get("refine_depth", 0))
                obj.segment_rmses.append(seg.get("segment_rmse", 0.0))
                
                if "normalized_arc_center" in seg:
                    obj.normalized_segment_centers.append(seg["normalized_arc_center"])
                    obj.normalized_segment_ranges.append(tuple(seg["normalized_arc_range"]))
                else:
                    total = obj.total_arc_length
                    obj.normalized_segment_centers.append(seg["arc_length_center"] / total)
                    obj.normalized_segment_ranges.append((
                        seg["arc_length_range"][0] / total,
                        seg["arc_length_range"][1] / total
                    ))
            
            print(f"  弧长对应模式: {filepath}")
            print(f"  共 {obj.num_segments} 段")
        
        return obj
    
    def transform_point(self, point, arc_length):
        """
        变换单个点（根据弧长位置插值）
        
        权重计算与 tunable_registration.py 中的 segmented_kabsch_align 一致：
        - 配准时：weights = exp(-((idx - center)^2) / (2 * sigma^2)), sigma = seg_len / 3
        - 这里：weights = exp(-((s - center)^2) / (2 * sigma^2)), sigma = range / 3
        """
        # 计算每个分段的高斯权重
        weights = []
        for center, (start, end) in zip(self.segment_centers, self.segment_ranges):
            # 与配准时一致：sigma = 段长度 / 3
            seg_range = end - start
            sigma = seg_range / 3
            w = np.exp(-((arc_length - center) ** 2) / (2 * sigma ** 2 + 1e-10))
            weights.append(w)
        
        weights = np.array(weights)
        weights /= (weights.sum() + 1e-10)
        
        # 加权变换
        point_homo = np.append(point, 1)
        result = np.zeros(3)
        
        for w, T in zip(weights, self.segment_transforms):
            transformed = (T @ point_homo)[:3]
            result += w * transformed
        
        return result
    
    def transform_point_normalized(self, point, normalized_arc_position, 
                                    enable_depth_weight=True, 
                                    enable_endpoint_boost=True,
                                    enable_boundary_compensation=True,
                                    endpoint_ratio=0.10,
                                    endpoint_weight_boost=2.0):
        """
        使用归一化弧长位置（0-1）进行变换
        
        ⭐⭐⭐ 与配准时完全一致的权重策略 ⭐⭐⭐
        
        数学公式：
            P'(s̃) = Σ w_i(s̃) × T_i × P
            
            其中：
            - s̃ = s / L ∈ [0, 1] (归一化弧长位置)
            - w_i(s̃) = 基础高斯权重 × 深度加权 × 端点提升 × 边界补偿
        
        权重策略（与 tunable_registration.py 中的 segmented_kabsch_align 一致）：
        1. 基础高斯权重：exp(-(s̃ - c̃_i)² / (2σ̃_i²))
        2. 深度加权：w *= (1 + depth * 0.3) — 细分段权重更高
        3. 端点权重提升：前后 10% 区域 w *= 2.0
        4. 边界渐变补偿：起点段后半/终点段前半额外加权
        
        参数:
            point: 3D点坐标 [x, y, z]
            normalized_arc_position: 归一化弧长位置，0.0=起点, 1.0=终点
            enable_depth_weight: 是否启用深度加权
            enable_endpoint_boost: 是否启用端点权重提升
            enable_boundary_compensation: 是否启用边界渐变补偿
            endpoint_ratio: 端点区域占比（默认10%）
            endpoint_weight_boost: 端点权重提升因子（默认2.0）
        
        返回:
            变换后的3D点坐标
        """
        weights = []
        s_norm = normalized_arc_position
        
        # ⭐ 端点区域判定
        is_in_start_region = s_norm < endpoint_ratio
        is_in_end_region = s_norm > (1.0 - endpoint_ratio)
        
        for i, (norm_center, (norm_start, norm_end), depth) in enumerate(zip(
            self.normalized_segment_centers,
            self.normalized_segment_ranges,
            self.segment_depths
        )):
            # ========== 1. 基础高斯权重 ==========
            norm_range = norm_end - norm_start
            sigma = norm_range / 3  # 与配准时一致：sigma = 段长/3
            
            w = np.exp(-((s_norm - norm_center) ** 2) / (2 * sigma ** 2 + 1e-10))
            
            # ========== 2. 深度加权（与配准时一致）==========
            if enable_depth_weight:
                depth_weight = 1.0 + depth * 0.3  # 细分越深，权重越高
                w *= depth_weight
            
            # ========== 3. 端点权重提升（与配准时一致）==========
            if enable_endpoint_boost:
                # 判断当前段是否为端点段
                seg_is_start_endpoint = norm_center < endpoint_ratio
                seg_is_end_endpoint = norm_center > (1.0 - endpoint_ratio)
                
                if (seg_is_start_endpoint and is_in_start_region) or \
                   (seg_is_end_endpoint and is_in_end_region):
                    w *= endpoint_weight_boost  # 端点段权重 × 2.0
                    
                    # ========== 4. 边界渐变补偿（与配准时一致）==========
                    if enable_boundary_compensation:
                        # 计算点在段内的相对位置 ∈ [0, 1]
                        if norm_range > 1e-10:
                            relative_pos = (s_norm - norm_start) / norm_range
                            relative_pos = np.clip(relative_pos, 0.0, 1.0)
                            
                            if seg_is_start_endpoint:
                                # 起点段：后半部分额外提升（从1.0渐变到1.5）
                                # 原逻辑：boundary_boost = linspace(1.0, 1.5, seg_len)
                                boundary_boost = 1.0 + 0.5 * relative_pos
                                w *= boundary_boost
                            elif seg_is_end_endpoint:
                                # 终点段：前半部分额外提升（从1.5渐变到1.0）
                                # 原逻辑：boundary_boost = linspace(1.5, 1.0, seg_len)
                                boundary_boost = 1.5 - 0.5 * relative_pos
                                w *= boundary_boost
            
            weights.append(w)
        
        # 归一化权重
        weights = np.array(weights)
        weights /= (weights.sum() + 1e-10)
        
        # 加权变换
        point_homo = np.append(point, 1)  # [x, y, z, 1]
        result = np.zeros(3)
        
        for w, T in zip(weights, self.segment_transforms):
            transformed = (T @ point_homo)[:3]
            result += w * transformed
        
        return result
    
    def transform_trajectory(self, points, use_normalized=True):
        """
        变换整条轨迹（自动选择模式）
        
        参数:
            points: Nx3 点云数组
            use_normalized: 
                True  = ⭐ 使用归一化弧长空间（弧长模式推荐）
                False = 使用缩放弧长（旧方法，保留兼容性）
        
        返回:
            变换后的Nx3点云数组
        """
        # ⭐⭐⭐ 序列号对应模式自动分发
        if self.mode == "sequence_aligned":
            return self._transform_by_sequence(points)
        
        arc_lengths = compute_arc_length(points)
        input_total = arc_lengths[-1]
        
        transformed = np.zeros_like(points)
        
        if use_normalized and len(self.normalized_segment_centers) > 0:
            # ⭐⭐⭐ 新方法：归一化弧长空间
            # 将输入轨迹弧长归一化到0-1
            normalized_arc_lengths = arc_lengths / input_total
            
            for i, (p, norm_s) in enumerate(zip(points, normalized_arc_lengths)):
                transformed[i] = self.transform_point_normalized(p, norm_s)
        
        else:
            # 旧方法：缩放弧长（保留用于兼容性）
            scale = self.total_arc_length / (input_total + 1e-10)
            scaled_arc_lengths = arc_lengths * scale
            
            for i, (p, s) in enumerate(zip(points, scaled_arc_lengths)):
                transformed[i] = self.transform_point(p, s)
        
        return transformed
    
    def _transform_by_sequence(self, points):
        """
        ⭐⭐⭐ 序列号对应模式变换
        
        要求：
        - points 的行数必须等于 self.total_frames
        - 第i行对应训练时的第i帧
        
        权重策略与配准时（segmented_kabsch_align_by_sequence）完全一致：
        1. 基础高斯权重（帧号距离）
        2. 深度加权（细分段权重更高）
        3. 端点权重提升
        4. 边界渐变补偿
        """
        N = len(points)
        
        if N != self.total_frames:
            raise ValueError(
                f"帧数不匹配！训练时{self.total_frames}帧，当前{N}帧。\n"
                f"序列号对应模式要求帧数完全一致！"
            )
        
        transformed = np.zeros_like(points)
        weight_sum = np.zeros(N)
        
        # 端点参数（与配准时一致）
        endpoint_ratio = 0.10
        endpoint_weight_boost = 2.0
        
        for seg_idx in range(self.num_segments):
            T = self.segment_transforms[seg_idx]
            center_frame = self.segment_center_frames[seg_idx]
            start_frame, end_frame = self.segment_frame_ranges[seg_idx]
            frame_span = end_frame - start_frame
            sigma_frame = frame_span / 3
            depth = self.segment_depths[seg_idx] if seg_idx < len(self.segment_depths) else 0
            
            # 判断端点段
            is_start_endpoint_seg = start_frame < N * endpoint_ratio
            is_end_endpoint_seg = end_frame > N * (1 - endpoint_ratio)
            
            for frame_idx in range(N):
                norm_pos = frame_idx / N
                
                # 1. 基础高斯权重
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
                transformed[frame_idx] += w * (T @ np.append(points[frame_idx], 1))[:3]
        
        # 归一化
        weight_sum[weight_sum == 0] = 1
        transformed /= weight_sum[:, np.newaxis]
        
        return transformed


class MLSTransform:
    """
    ⭐⭐⭐ MLS变换类（复用端） - 加载和应用MLS配准结果
    
    数学公式：
    P'(x) = R(x)·x + t(x)
    其中 R(x), t(x) 由预计算网格线性插值获得
    
    支持两种模式：
    - "grid": 预计算网格插值（O(1)每点，推荐）
    - "full": 完整加权Kabsch计算（O(M)每点，高精度）
    """
    
    def __init__(self):
        self.mode = "mls"
        self.bandwidth = 0.05
        self.total_arc_length = 0
        self.num_training_points = 0
        self.training_rmse = 0
        self.training_max_error = 0
        self.training_mean_error = 0
        
        # 训练数据（全量模式使用）
        self.train_source = None
        self.train_target = None
        self.train_norm_arc = None
        
        # 预计算网格（快速模式使用）
        self.grid_positions = None
        self.grid_transforms = None
        
        # 推荐模式
        self.recommended_mode = "grid"
    
    @classmethod
    def load(cls, filepath):
        """从JSON加载MLS变换"""
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        obj = cls()
        obj.mode = data.get("mode", "mls")
        obj.bandwidth = data["bandwidth"]
        obj.total_arc_length = data["total_arc_length"]
        obj.num_training_points = data.get("num_training_points", 0)
        obj.recommended_mode = data.get("recommended_mode", "grid")
        
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
    
    def transform_point_grid(self, point, norm_s):
        """
        网格插值快速变换: O(1)每点
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
        
        # 对变换结果插值
        point_homo = np.append(point, 1)
        result_left = (self.grid_transforms[idx] @ point_homo)[:3]
        result_right = (self.grid_transforms[idx + 1] @ point_homo)[:3]
        
        return (1 - t) * result_left + t * result_right
    
    def transform_point_full(self, point, norm_s):
        """
        完整MLS计算: 加权Kabsch SVD
        """
        if self.train_source is None:
            raise ValueError("训练数据未加载，无法使用full模式")
        
        distances = np.abs(norm_s - self.train_norm_arc)
        weights = np.exp(-(distances ** 2) / (self.bandwidth ** 2))
        
        # 安全检查
        if weights.sum() < 1e-6:
            # 回退：使用最近点几何
            k = min(20, len(self.train_source))
            nearest_indices = np.argsort(distances)[:k]
            weights = np.zeros(len(self.train_source))
            weights[nearest_indices] = 1.0
        
        T_local = self._weighted_kabsch(self.train_source, self.train_target, weights)
        return (T_local @ np.append(point, 1))[:3]
    
    @staticmethod
    def _weighted_kabsch(source, target, weights):
        """加权Kabsch（内联版本，避免依赖tunable_registration）"""
        W = weights / (weights.sum() + 1e-10)
        src_center = np.sum(W[:, None] * source, axis=0)
        tgt_center = np.sum(W[:, None] * target, axis=0)
        src_centered = source - src_center
        tgt_centered = target - tgt_center
        H = (W[:, None] * src_centered).T @ tgt_centered
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
    
    def transform_trajectory(self, points, use_mode=None):
        """
        变换整条轨迹
        
        参数:
            points: Nx3 点云数组
            use_mode: "grid"=网格插值(快速), "full"=完整计算, None=自动选择
        
        返回:
            Nx3 变换后的点云
        """
        mode = use_mode or self.recommended_mode
        
        # 计算弧长并归一化
        arc_lengths = compute_arc_length(points)
        input_total = arc_lengths[-1]
        normalized_arc = arc_lengths / (input_total + 1e-10)
        
        transformed = np.zeros_like(points)
        
        if mode == "grid" and self.grid_transforms is not None:
            for i in range(len(points)):
                transformed[i] = self.transform_point_grid(points[i], normalized_arc[i])
        else:
            for i in range(len(points)):
                transformed[i] = self.transform_point_full(points[i], normalized_arc[i])
        
        return transformed


def load_global_transform(filepath):
    """加载全局刚性变换矩阵"""
    with open(filepath, 'r', encoding='utf-8') as f:
        data = json.load(f)
    return np.array(data["transform_matrix_4x4"])


def apply_global_transform(points, T):
    """应用全局刚性变换"""
    N = len(points)
    points_homo = np.hstack([points, np.ones((N, 1))])
    transformed_homo = (T @ points_homo.T).T
    return transformed_homo[:, :3]


def _get_global_transform_for_blend(segmented_json_path, global_path_override=None):
    """
    获取与分段变换同坐标系的全局变换矩阵（用于混合变换）
    
    自动处理坐标系转换：
    - 如果分段变换是机器人坐标系(robot_base)，自动将灯塔坐标系全局变换
      通过 T^R = T_L^R @ T^L @ (T_L^R)^{-1} 转换到机器人坐标系
    - 如果分段变换是灯塔坐标系(lighthouse)，直接使用全局变换
    
    参数:
        segmented_json_path: 分段变换JSON文件路径
        global_path_override: 指定全局变换文件路径（None=自动查找）
    
    返回:
        4x4全局变换矩阵（与分段变换同坐标系），或None（找不到文件时）
    """
    # 确定全局变换文件路径
    if global_path_override:
        global_path = global_path_override
    else:
        dir_path = os.path.dirname(segmented_json_path)
        global_path = os.path.join(dir_path, "transform_matrix.json")
    
    if not os.path.exists(global_path):
        return None
    
    # 加载全局变换（灯塔坐标系）
    T_global_L = load_global_transform(global_path)
    
    # 加载分段变换JSON以获取坐标系信息
    with open(segmented_json_path, 'r', encoding='utf-8') as f:
        seg_data = json.load(f)
    
    coord_frame = seg_data.get("coordinate_frame", "lighthouse")
    
    if coord_frame == "robot_base" and "hand_eye_transform" in seg_data:
        # 分段变换在机器人坐标系 → 全局变换也需要转到机器人坐标系
        T_LR = np.array(seg_data["hand_eye_transform"]["matrix_4x4"])
        T_RL = np.linalg.inv(T_LR)
        T_global_R = T_LR @ T_global_L @ T_RL
        return T_global_R
    else:
        # 都在灯塔坐标系，直接使用
        return T_global_L


def transform_for_verification(source_csv, target_csv, transform_path=None, 
                                output_transformed_csv=None, output_target_csv=None,
                                xyz_cols=(3, 4, 5), verbose=True):
    """
    ⭐⭐⭐ 专门用于验证配准效果的函数 ⭐⭐⭐
    
    在配准时使用的采样空间中进行变换和对比，确保验证误差与配准误差一致。
    
    流程（与配准完全一致）：
    1. 加载source和target原始数据
    2. 自适应空间重采样（或传统弧长重采样）
    3. 高斯平滑
    4. 对source应用分段变换
    5. 输出变换后的source和预处理后的target（用于可视化对比）
    
    参数：
        source_csv: 源轨迹CSV文件路径
        target_csv: 目标轨迹CSV文件路径
        transform_path: 分段变换文件路径
        output_transformed_csv: 变换后源轨迹输出路径
        output_target_csv: 预处理后目标轨迹输出路径
        xyz_cols: XYZ坐标列索引
        verbose: 是否打印详细信息
        
    返回：
        (transformed_source, preprocessed_target, error_stats)
    """
    if transform_path is None:
        transform_path = DEFAULT_TRANSFORM_PATH
    
    if verbose:
        print(f"\n{'='*60}")
        print(f"配准验证工具（自适应空间重采样）")
        print(f"{'='*60}")
    
    # 加载数据
    if verbose:
        print(f"\n▶ 加载数据...")
    
    with open(source_csv, 'r', encoding='utf-8') as f:
        source_header = f.readline()
    source_data = np.loadtxt(source_csv, delimiter=',', skiprows=1)
    source_points = source_data[:, xyz_cols].copy()
    
    with open(target_csv, 'r', encoding='utf-8') as f:
        target_header = f.readline()
    target_data = np.loadtxt(target_csv, delimiter=',', skiprows=1)
    target_points = target_data[:, xyz_cols].copy()
    
    if verbose:
        print(f"  源轨迹: {len(source_points)}点, 弧长={compute_arc_length(source_points)[-1]:.2f}mm")
        print(f"  目标轨迹: {len(target_points)}点, 弧长={compute_arc_length(target_points)[-1]:.2f}mm")
    
    # ⭐ 先加载变换文件以检测模式
    # 检测JSON中的mode字段判断是MLS还是分段Kabsch
    with open(transform_path, 'r', encoding='utf-8') as f:
        _peek_data = json.load(f)
    detected_mode = _peek_data.get("mode", "arc_length")
    
    if detected_mode == "mls":
        mls_transform = MLSTransform.load(transform_path)
        seg_transform = None
    else:
        seg_transform = SegmentedTransform.load(transform_path)
        mls_transform = None
    
    # ⭐⭐⭐ MLS模式验证
    if mls_transform is not None:
        if verbose:
            print(f"\n▶ MLS模式验证")
            print(f"  带宽: h={mls_transform.bandwidth:.4f}")
        
        # Step 1: 预平滑
        if PRE_SMOOTH["enable"]:
            source_points = pre_smooth(source_points, PRE_SMOOTH)
            target_points = pre_smooth(target_points, PRE_SMOOTH)
            if verbose:
                print(f"  预平滑完成: 方法={PRE_SMOOTH['method']}")
        
        # Step 2: 弦距重采样（与训练一致）
        resample_method = PREPROCESS_CONFIG.get("resample_method", "chord_spatial")
        if resample_method == "chord_spatial":
            delta_d = ADAPTIVE_SPATIAL_RESAMPLE["delta_d"]
            densify_factor = ADAPTIVE_SPATIAL_RESAMPLE["densify_factor"]
            min_samples = ADAPTIVE_SPATIAL_RESAMPLE["min_samples"]
            max_samples = ADAPTIVE_SPATIAL_RESAMPLE["max_samples"]
            source_points, _ = chord_spatial_resample(
                source_points, delta_d, densify_factor, min_samples, max_samples, verbose)
            target_points, _ = chord_spatial_resample(
                target_points, delta_d, densify_factor, min_samples, max_samples, verbose)
        elif resample_method == "adaptive_spatial":
            noise_level = ADAPTIVE_SPATIAL_RESAMPLE["noise_level"]
            noise_factor = ADAPTIVE_SPATIAL_RESAMPLE["noise_suppression_factor"]
            min_samples = ADAPTIVE_SPATIAL_RESAMPLE["min_samples"]
            max_samples = ADAPTIVE_SPATIAL_RESAMPLE["max_samples"]
            source_points, _ = adaptive_spatial_resample_legacy(
                source_points, noise_level, noise_factor, min_samples, max_samples, verbose)
            target_points, _ = adaptive_spatial_resample_legacy(
                target_points, noise_level, noise_factor, min_samples, max_samples, verbose)
        
        # 对齐点数
        if len(source_points) != len(target_points):
            common_samples = min(len(source_points), len(target_points))
            source_points, _ = resample_by_arc_length(source_points, common_samples)
            target_points, _ = resample_by_arc_length(target_points, common_samples)
            if verbose:
                print(f"  对齐后点数: {common_samples}")
        
        # Step 3: 后平滑（与训练一致）
        if PREPROCESS_CONFIG.get("enable_smoothing", False):
            post_sigma = PREPROCESS_CONFIG.get("gaussian_sigma", 3.0)
            source_points = smooth_gaussian(source_points, sigma=post_sigma)
            target_points = smooth_gaussian(target_points, sigma=post_sigma)
            if verbose:
                print(f"  后平滑: Gaussian σ={post_sigma}")
        
        # Step 4: 应用MLS变换
        transformed_source = mls_transform.transform_trajectory(
            source_points, 
            use_mode=MLS_CONFIG["use_mode"]
        )
        target_smoothed = target_points
        
        if verbose:
            mode_name = "网格插值" if MLS_CONFIG["use_mode"] == "grid" else "完整计算"
            print(f"  ⭐ MLS变换完成（{len(transformed_source)}点，模式: {mode_name}）")
    
    # ⭐⭐⭐ 序列号对应模式：跳过重采样，直接帧号变换
    elif seg_transform is not None and seg_transform.mode == "sequence_aligned":
        if verbose:
            print(f"\n▶ 序列号对应模式验证")
            print(f"  训练帧数: {seg_transform.total_frames}")
        
        # 预平滑
        if PRE_SMOOTH["enable"]:
            source_points = pre_smooth(source_points, PRE_SMOOTH)
            target_points = pre_smooth(target_points, PRE_SMOOTH)
            if verbose:
                print(f"  预平滑完成")
        
        # 直接应用序列号对应变换（不重采样）
        transformed_source = seg_transform._transform_by_sequence(source_points)
        target_smoothed = target_points  # 无重采样，直接使用预平滑数据
        
        if verbose:
            print(f"  ⭐ 序列号变换完成（{len(transformed_source)} 帧）")
    else:
        # ========== 弧长模式：预平滑 → 重采样 → 后平滑 → 变换 ==========
        # ⭐⭐⭐ 预平滑（在重采样前，与配准完全一致）
        if PRE_SMOOTH["enable"]:
            if verbose:
                print(f"\n▶ 预平滑（消除噪声对弧长的影响）...")
            source_points = pre_smooth(source_points, PRE_SMOOTH)
            target_points = pre_smooth(target_points, PRE_SMOOTH)
            if verbose:
                method = PRE_SMOOTH["method"]
                if method == "gaussian":
                    print(f"  方法: 高斯平滑, sigma={PRE_SMOOTH['gaussian_sigma']}")
                else:
                    print(f"  方法: B-Spline, s={PRE_SMOOTH['bspline_smoothing']}, k={PRE_SMOOTH['bspline_k']}")
                print(f"  预平滑后弧长: 源={compute_arc_length(source_points)[-1]:.2f}mm, 目标={compute_arc_length(target_points)[-1]:.2f}mm")
        
        # 预处理（与配准完全一致）
        if verbose:
            print(f"\n▶ 重采样与后平滑（与配准流程完全一致）...")
        
        sigma = PREPROCESS_CONFIG["gaussian_sigma"]
        resample_method = PREPROCESS_CONFIG.get("resample_method", "chord_spatial")
        
        if resample_method == "chord_spatial":
            # ⭐ 新版：弦距重采样（解决海岸线悖论）
            delta_d = ADAPTIVE_SPATIAL_RESAMPLE["delta_d"]
            densify_factor = ADAPTIVE_SPATIAL_RESAMPLE["densify_factor"]
            min_samples = ADAPTIVE_SPATIAL_RESAMPLE["min_samples"]
            max_samples = ADAPTIVE_SPATIAL_RESAMPLE["max_samples"]
            
            print(f"  使用真3D弦距重采样（chord_spatial）...")
            source_resampled, src_info = chord_spatial_resample(
                source_points, delta_d, densify_factor, min_samples, max_samples, verbose
            )
            target_resampled, tgt_info = chord_spatial_resample(
                target_points, delta_d, densify_factor, min_samples, max_samples, verbose
            )
            
            if src_info['resampled_points'] != tgt_info['resampled_points']:
                common_samples = min(src_info['resampled_points'], tgt_info['resampled_points'])
                source_resampled, _ = resample_by_arc_length(source_resampled, common_samples)
                target_resampled, _ = resample_by_arc_length(target_resampled, common_samples)
                if verbose:
                    print(f"  对齐后点数: {common_samples}")
        
        elif resample_method == "adaptive_spatial":
            # 旧版：弧长插值重采样（需配合预平滑）
            noise_level = ADAPTIVE_SPATIAL_RESAMPLE["noise_level"]
            noise_factor = ADAPTIVE_SPATIAL_RESAMPLE["noise_suppression_factor"]
            min_samples = ADAPTIVE_SPATIAL_RESAMPLE["min_samples"]
            max_samples = ADAPTIVE_SPATIAL_RESAMPLE["max_samples"]
            
            print(f"  使用自适应弧长重采样（旧版）...")
            source_resampled, src_info = adaptive_spatial_resample_legacy(
                source_points, noise_level, noise_factor, min_samples, max_samples, verbose
            )
            target_resampled, tgt_info = adaptive_spatial_resample_legacy(
                target_points, noise_level, noise_factor, min_samples, max_samples, verbose
            )
            
            if src_info['resampled_points'] != tgt_info['resampled_points']:
                common_samples = min(src_info['resampled_points'], tgt_info['resampled_points'])
                source_resampled, _ = resample_by_arc_length(source_resampled, common_samples)
                target_resampled, _ = resample_by_arc_length(target_resampled, common_samples)
                if verbose:
                    print(f"  对齐后点数: {common_samples}")
        else:
            num_resample = PREPROCESS_CONFIG.get("num_resample", 5000)
            print(f"  使用传统弧长重采样: {num_resample}点")
            source_resampled, _ = resample_by_arc_length(source_points, num_resample)
            target_resampled, _ = resample_by_arc_length(target_points, num_resample)
        
        # 高斯平滑
        source_smoothed = smooth_gaussian(source_resampled, sigma)
        target_smoothed = smooth_gaussian(target_resampled, sigma)
        
        if verbose:
            print(f"  高斯平滑: sigma={sigma}")
            print(f"  源轨迹平滑后弧长: {compute_arc_length(source_smoothed)[-1]:.2f}mm")
            print(f"  目标轨迹平滑后弧长: {compute_arc_length(target_smoothed)[-1]:.2f}mm")
        
        # 应用分段变换
        if verbose:
            print(f"\n▶ 应用分段变换...")
        
        transformed_source = seg_transform.transform_trajectory(source_smoothed, use_normalized=True)
        
        if verbose:
            print(f"  分段数: {seg_transform.num_segments}")
            print(f"  变换完成")
    
    # 计算误差
    distances = np.linalg.norm(transformed_source - target_smoothed, axis=1)
    
    error_stats = {
        'mean': np.mean(distances),
        'rmse': np.sqrt(np.mean(distances**2)),
        'max': np.max(distances),
        'min': np.min(distances),
        'std': np.std(distances),
        'median': np.median(distances),
        'p95': np.percentile(distances, 95)
    }
    
    if verbose:
        print(f"\n▶ 误差统计（5000点空间）")
        print(f"  平均: {error_stats['mean']:.4f}mm")
        print(f"  RMSE: {error_stats['rmse']:.4f}mm")
        print(f"  最大: {error_stats['max']:.4f}mm")
        print(f"  P95:  {error_stats['p95']:.4f}mm")
    
    # 保存输出文件（用于可视化）
    if output_transformed_csv:
        if verbose:
            print(f"\n▶ 保存变换后源轨迹...")
        with open(output_transformed_csv, 'w', encoding='utf-8') as f:
            f.write("X_mm,Y_mm,Z_mm\n")
            for p in transformed_source:
                f.write(f"{p[0]:.4f},{p[1]:.4f},{p[2]:.4f}\n")
        if verbose:
            print(f"  已保存: {output_transformed_csv}")
    
    if output_target_csv:
        if verbose:
            print(f"\n▶ 保存预处理后目标轨迹...")
        with open(output_target_csv, 'w', encoding='utf-8') as f:
            f.write("X_mm,Y_mm,Z_mm\n")
            for p in target_smoothed:
                f.write(f"{p[0]:.4f},{p[1]:.4f},{p[2]:.4f}\n")
        if verbose:
            print(f"  已保存: {output_target_csv}")
    
    if verbose:
        print(f"\n{'='*60}")
        print(f"验证完成！")
        print(f"{'='*60}")
    
    return transformed_source, target_smoothed, error_stats


def transform_csv(input_csv, output_csv, transform_path=None, xyz_cols=(3, 4, 5), 
                  method="segmented", verbose=True, apply_preprocessing=True):
    """
    对CSV文件应用高精度分段配准变换
    
    参数：
        input_csv: 输入CSV文件路径
        output_csv: 输出CSV文件路径
        transform_path: 变换文件路径（默认使用分段变换）
        xyz_cols: XYZ坐标列索引，默认(3,4,5)对应第4,5,6列
        method: "segmented"=分段高精度, "global"=全局刚性
        verbose: 是否打印详细信息
        apply_preprocessing: ⭐ 是否应用与配准一致的预处理（平滑+重采样）
    
    返回：
        变换后的点云数组
    """
    if transform_path is None:
        if method == "segmented":
            transform_path = DEFAULT_TRANSFORM_PATH
        else:
            transform_path = DEFAULT_TRANSFORM_PATH.replace("segmented_transform.json", "transform_matrix.json")
    
    if verbose:
        print(f"\n{'='*60}")
        print(f"高精度轨迹变换工具")
        print(f"{'='*60}")
        print(f"\n输入文件: {input_csv}")
        print(f"输出文件: {output_csv}")
        print(f"变换方法: {method}")
        print(f"变换文件: {transform_path}")
        print(f"预处理: {'启用（与配准一致）' if apply_preprocessing else '禁用'}")
    
    # 检查文件是否存在
    if not os.path.exists(input_csv):
        raise FileNotFoundError(f"输入文件不存在: {input_csv}")
    
    if not os.path.exists(transform_path):
        raise FileNotFoundError(f"变换文件不存在: {transform_path}")
    
    # 读取CSV文件
    if verbose:
        print(f"\n▶ 读取CSV文件...")
    
    # 读取表头
    with open(input_csv, 'r', encoding='utf-8') as f:
        header = f.readline()
    
    # 读取数据
    data = np.loadtxt(input_csv, delimiter=',', skiprows=1)
    points_original = data[:, xyz_cols].copy()
    original_count = len(points_original)
    
    if verbose:
        print(f"  原始数据行数: {original_count}")
        print(f"  XYZ列索引: {xyz_cols}")
        print(f"  坐标范围: X[{points_original[:,0].min():.2f}, {points_original[:,0].max():.2f}]")
        print(f"            Y[{points_original[:,1].min():.2f}, {points_original[:,1].max():.2f}]")
        print(f"            Z[{points_original[:,2].min():.2f}, {points_original[:,2].max():.2f}]")
    
    # ⭐⭐⭐ 预处理与变换（完全复制配准流程）⭐⭐⭐
    # 配准流程：原始数据 → 弧长重采样5000点 → 高斯平滑 → 配准变换
    # 应用流程：原始数据 → 弧长重采样5000点 → 高斯平滑 → 应用变换 → 插值回原始点数
    
    # ⭐ 先加载变换文件以检测模式
    mls_transform = None
    seg_transform = None
    if method == "segmented":
        # 检测JSON中的mode字段判断是MLS还是分段Kabsch
        with open(transform_path, 'r', encoding='utf-8') as f:
            _peek = json.load(f)
        detected_mode = _peek.get("mode", "arc_length")
        
        if detected_mode == "mls":
            mls_transform = MLSTransform.load(transform_path)
            # ⭐ 从MLS JSON读取训练时的预处理参数，确保一致性
            mls_preproc = _peek.get("preprocessing", {})
            mls_pre_sigma = mls_preproc.get("pre_smooth_sigma")
            if mls_pre_sigma is not None and mls_pre_sigma != PRE_SMOOTH.get("gaussian_sigma"):
                if verbose:
                    print(f"  ⚠ 检测到MLS训练sigma={mls_pre_sigma}，自动同步（原值={PRE_SMOOTH.get('gaussian_sigma')}）")
                PRE_SMOOTH["gaussian_sigma"] = mls_pre_sigma
        else:
            seg_transform = SegmentedTransform.load(transform_path)
    
    # ⭐⭐⭐ MLS模式：方案A — 预平滑弧长定位 + 直接逐点变换（无重采样、无反插值）
    # 原理：预平滑只用于计算稳定的归一化弧长（弧长定位），变换作用在原始坐标上。
    #       MLS 变换已是定义在 [0,1] 上的连续函数，直接逐点查表即可，无需重采样中间层。
    if method == "segmented" and mls_transform is not None:
        if verbose:
            print(f"\n▶ MLS模式（方案A：直接逐点变换，无重采样）")
            print(f"  带宽: h={mls_transform.bandwidth:.4f}")
            print(f"  训练点数: {mls_transform.num_training_points}")

        # ========== Step 1: 预平滑（仅用于稳定弧长计算，与训练一致）==========
        points_smooth = points_original.copy()
        if PRE_SMOOTH["enable"]:
            points_smooth = pre_smooth(points_smooth, PRE_SMOOTH)
            if verbose:
                print(f"  预平滑: 方法={PRE_SMOOTH['method']}, "
                      f"sigma={PRE_SMOOTH.get('gaussian_sigma')}")

        # ========== Step 2: 基于预平滑数据计算归一化弧长（稳定、抗噪）==========
        # 用平滑后的弧长作为 MLS 查表的位置索引；原始坐标留给 Step 3 变换，保留测量特征。
        arc_smooth = compute_arc_length(points_smooth)
        norm_arc = arc_smooth / (arc_smooth[-1] + 1e-10)   # ∈ [0, 1]，N 个点

        if verbose:
            print(f"  预平滑后弧长: {arc_smooth[-1]:.2f}mm  ({len(points_smooth)} 点)")

        # ========== Step 3: 对每个原始点直接应用 MLS 变换（O(1)/点，共 N 次）==========
        # 弧长定位来自 points_smooth（稳定），变换坐标来自 points_original（原始特征）。
        # 注意：MLSTransform.transform_trajectory() 用传入点自身的弧长同时做定位和变换，
        #       不满足"分离弧长来源"的要求，故此处手动循环。
        use_mode = MLS_CONFIG["use_mode"]
        transformed = np.zeros_like(points_original)

        if use_mode == "grid" and mls_transform.grid_transforms is not None:
            for i in range(len(points_original)):
                transformed[i] = mls_transform.transform_point_grid(
                    points_original[i],   # 变换原始坐标
                    norm_arc[i]           # 用平滑弧长定位
                )
        else:
            for i in range(len(points_original)):
                transformed[i] = mls_transform.transform_point_full(
                    points_original[i],
                    norm_arc[i]
                )

        if verbose:
            mode_name = "网格插值" if use_mode == "grid" else "完整计算"
            print(f"  ⭐ MLS变换完成（{len(transformed)} 点, 模式={mode_name}）")
            print(f"  变换后弧长: {compute_arc_length(transformed)[-1]:.2f}mm")
    
    # ⭐⭐⭐ 序列号对应模式：跳过重采样，直接按帧号变换
    elif method == "segmented" and seg_transform is not None and seg_transform.mode == "sequence_aligned":
        if verbose:
            print(f"\n▶ 序列号对应模式 - 直接帧号变换...")
            print(f"  分段数: {seg_transform.num_segments}")
            print(f"  训练帧数: {seg_transform.total_frames}")
            print(f"  当前帧数: {original_count}")
        
        # 预平滑（与配准时一致）
        points_to_process = points_original.copy()
        if PRE_SMOOTH["enable"]:
            points_to_process = pre_smooth(points_to_process, PRE_SMOOTH)
            if verbose:
                print(f"  预平滑: sigma={PRE_SMOOTH.get('gaussian_sigma', 1.5)}")
        
        # 直接应用序列号对应变换
        transformed = seg_transform._transform_by_sequence(points_to_process)
        
        if verbose:
            print(f"  ⭐ 序列号对应变换完成（{len(transformed)} 帧）")
            print(f"  变换后弧长: {compute_arc_length(transformed)[-1]:.2f}mm")
    
    elif apply_preprocessing and method == "segmented":
        if verbose:
            print(f"\n▶ 预处理与直接变换...")
        
        # ⭐ 步骤1：预平滑（消除噪声对弧长计算的影响）
        points_to_process = points_original.copy()
        if PRE_SMOOTH["enable"]:
            if verbose:
                print(f"  预平滑处理...")
            points_to_process = pre_smooth(points_to_process, PRE_SMOOTH)
            if verbose:
                method_name = PRE_SMOOTH["method"]
                if method_name == "gaussian":
                    print(f"    方法: 高斯平滑, sigma={PRE_SMOOTH['gaussian_sigma']}")
                else:
                    print(f"    方法: B-Spline, s={PRE_SMOOTH['bspline_smoothing']}, k={PRE_SMOOTH['bspline_k']}")
                pre_arc = compute_arc_length(points_to_process)[-1]
                print(f"    预平滑后弧长: {pre_arc:.2f}mm")
        
        # ⭐ 步骤2：计算预平滑后数据的归一化弧长位置
        # 使用预平滑数据计算弧长（稳定、抗噪），但对原始点应用变换（保留原始特征）
        arc_lengths = compute_arc_length(points_to_process)
        total_arc = arc_lengths[-1]
        normalized_arc = arc_lengths / total_arc  # 归一化到 [0, 1]
        
        if verbose:
            print(f"  预平滑后轨迹弧长: {total_arc:.2f}mm")
            print(f"  原始点数: {len(points_original)}")
        
        # ⭐ 步骤3：直接应用分段变换到每个原始点
        # （seg_transform 已在上方加载）
        if verbose:
            print(f"\n▶ 直接应用分段变换（无重采样/逆映射）...")
            print(f"  分段数: {seg_transform.num_segments}")
            print(f"  配准时弧长(预处理后): {seg_transform.total_arc_length:.2f}")
            print(f"  原始弧长: {seg_transform.original_arc_length:.2f}")
        
        # 对每个原始点，按其归一化弧长位置直接应用分段变换
        transformed_seg = np.zeros_like(points_original)
        for i in range(len(points_original)):
            transformed_seg[i] = seg_transform.transform_point_normalized(
                points_original[i], normalized_arc[i]
            )
        
        if verbose:
            print(f"  分段变换完成（{len(points_original)} 点）")
        
        # ⭐ 混合变换：分段 + 全局，降低分段过拟合的风险
        alpha = BLEND_CONFIG.get("alpha", 1.0) if BLEND_CONFIG.get("enable", False) else 1.0
        
        if alpha < 1.0:
            # 加载全局变换（自动转到与分段变换相同的坐标系）
            T_global = _get_global_transform_for_blend(
                transform_path, 
                BLEND_CONFIG.get("global_transform_path")
            )
            
            if T_global is not None:
                transformed_global = apply_global_transform(points_original, T_global)
                # 混合: result = α × 分段 + (1-α) × 全局
                transformed = alpha * transformed_seg + (1 - alpha) * transformed_global
                
                if verbose:
                    print(f"  全局变换加载成功")
                    print(f"  ⭐ 混合变换: α={alpha:.2f} (分段×{alpha:.0%} + 全局×{1-alpha:.0%})")
            else:
                transformed = transformed_seg
                if verbose:
                    print(f"  ⚠️ 未找到全局变换文件，仅使用分段变换")
        else:
            transformed = transformed_seg
            if verbose:
                print(f"  使用纯分段变换（α=1.0）")
        
        if verbose:
            print(f"  变换后弧长: {compute_arc_length(transformed)[-1]:.2f}mm")
    
    elif method == "segmented":
        # 不使用预处理，直接应用变换
        # （seg_transform 已在上方加载）
        if verbose:
            print(f"\n▶ 应用分段变换（无预处理）...")
            print(f"  分段数: {seg_transform.num_segments}")
            print(f"  模式: {seg_transform.mode}")
        
        transformed = seg_transform.transform_trajectory(
            points_original, 
            use_normalized=False
        )
    
    else:
        # 全局变换
        if verbose:
            print(f"\n▶ 应用全局变换...")
        T = load_global_transform(transform_path)
        transformed = apply_global_transform(points_original, T)
    
    if verbose:
        print(f"\n▶ 变换结果")
        print(f"  新坐标范围: X[{transformed[:,0].min():.2f}, {transformed[:,0].max():.2f}]")
        print(f"              Y[{transformed[:,1].min():.2f}, {transformed[:,1].max():.2f}]")
        print(f"              Z[{transformed[:,2].min():.2f}, {transformed[:,2].max():.2f}]")
    
    # 替换原始数据中的XYZ列
    data[:, xyz_cols[0]] = transformed[:, 0]
    data[:, xyz_cols[1]] = transformed[:, 1]
    data[:, xyz_cols[2]] = transformed[:, 2]
    
    # 保存输出文件
    if verbose:
        print(f"\n▶ 保存输出文件...")
    
    # 读取原始文件获取每列的格式
    with open(input_csv, 'r', encoding='utf-8') as f:
        header = f.readline()
        first_line = f.readline().strip()
    
    # 分析原始数据格式
    original_values = first_line.split(',')
    
    with open(output_csv, 'w', encoding='utf-8') as f:
        f.write(header)
        for row_idx, row in enumerate(data):
            formatted_values = []
            for col_idx, v in enumerate(row):
                if col_idx in xyz_cols:
                    # XYZ列保持4位小数（与原始格式一致）
                    formatted_values.append(f"{v:.4f}")
                else:
                    # 其他列尽量保持原始格式
                    if v == int(v):
                        formatted_values.append(str(int(v)))
                    else:
                        # 检查原始精度
                        if row_idx == 0 and col_idx < len(original_values):
                            orig = original_values[col_idx]
                            if '.' in orig:
                                decimals = len(orig.split('.')[1])
                                formatted_values.append(f"{v:.{decimals}f}")
                            else:
                                formatted_values.append(str(int(v)))
                        else:
                            formatted_values.append(f"{v:.6f}" if abs(v) < 100 else f"{v:.4f}")
            f.write(','.join(formatted_values) + '\n')
    
    if verbose:
        print(f"  已保存: {output_csv}")
        print(f"\n{'='*60}")
        print(f"变换完成!")
        print(f"{'='*60}")
    
    return transformed


def verify_transform_result(transformed_csv, target_csv, xyz_cols=(3, 4, 5), 
                            use_arc_length_align=True, num_samples=5000, verbose=True):
    """
    ⭐ 验证变换结果 - 使用与配准一致的方式计算误差
    
    参数：
        transformed_csv: 变换后的CSV文件
        target_csv: 目标CSV文件
        xyz_cols: XYZ坐标列索引
        use_arc_length_align: 是否使用弧长对齐（推荐！与配准一致）
        num_samples: 弧长重采样点数
        verbose: 是否打印详细信息
    
    返回：
        误差统计字典
    """
    if verbose:
        print(f"\n{'='*60}")
        print(f"验证变换结果（与配准方法一致）")
        print(f"{'='*60}")
    
    # 读取数据
    data1 = np.loadtxt(transformed_csv, delimiter=',', skiprows=1)
    data2 = np.loadtxt(target_csv, delimiter=',', skiprows=1)
    
    points1 = data1[:, xyz_cols]
    points2 = data2[:, xyz_cols]
    
    if verbose:
        print(f"\n变换后轨迹: {len(points1)} 点")
        print(f"目标轨迹:   {len(points2)} 点")
    
    if use_arc_length_align:
        # ⭐ 使用弧长对齐（与配准工具一致）
        if verbose:
            print(f"\n▶ 使用弧长对齐（{num_samples}点）...")
        
        # 可选：先平滑
        if PREPROCESS_CONFIG["enable_smoothing"]:
            points1 = smooth_gaussian(points1, PREPROCESS_CONFIG["gaussian_sigma"])
            points2 = smooth_gaussian(points2, PREPROCESS_CONFIG["gaussian_sigma"])
            if verbose:
                print(f"  高斯平滑: sigma={PREPROCESS_CONFIG['gaussian_sigma']}")
        
        # 弧长重采样
        aligned1, len1 = resample_by_arc_length(points1, num_samples)
        aligned2, len2 = resample_by_arc_length(points2, num_samples)
        
        if verbose:
            print(f"  变换后弧长: {len1:.2f}")
            print(f"  目标弧长:   {len2:.2f}")
            print(f"  长度比:     {len1/len2:.4f}")
        
        distances = np.linalg.norm(aligned1 - aligned2, axis=1)
        compared_points = num_samples
    else:
        # 索引对齐
        min_len = min(len(points1), len(points2))
        distances = np.linalg.norm(points1[:min_len] - points2[:min_len], axis=1)
        compared_points = min_len
        if verbose:
            print(f"\n▶ 使用索引对齐（{compared_points}点）...")
    
    # 计算统计量
    stats = {
        'compared_points': compared_points,
        'mean': np.mean(distances),
        'rmse': np.sqrt(np.mean(distances**2)),
        'max': np.max(distances),
        'min': np.min(distances),
        'median': np.median(distances),
        'std': np.std(distances),
        'p95': np.percentile(distances, 95),
    }
    
    if verbose:
        print(f"\n{'='*60}")
        print(f"【验证结果】（对齐方式: {'弧长' if use_arc_length_align else '索引'}）")
        print(f"{'='*60}")
        print(f"  比较点数: {stats['compared_points']}")
        print(f"  平均误差: {stats['mean']:.4f}")
        print(f"  RMSE:     {stats['rmse']:.4f}")
        print(f"  最大误差: {stats['max']:.4f}")
        print(f"  最小误差: {stats['min']:.4f}")
        print(f"  中位数:   {stats['median']:.4f}")
        print(f"  P95:      {stats['p95']:.4f}")
        print(f"{'='*60}")
    
    return stats


def interactive_mode():
    """交互模式"""
    print("\n" + "=" * 60)
    print("高精度轨迹变换工具 - 交互模式")
    print("=" * 60)
    
    # 输入文件
    print("\n请输入要处理的CSV文件路径:")
    print("(直接回车使用默认测试文件)")
    input_csv = input("> ").strip()
    
    if not input_csv:
        input_csv = r"E:\Unity cangku\lighthouse_2.10\Assets\StreamingAssets\TrackerRecordings\tracker1 - tcp.csv"
        print(f"  使用默认: {input_csv}")
    
    if not os.path.exists(input_csv):
        print(f"错误: 文件不存在 - {input_csv}")
        return
    
    # 输出文件
    print("\n请输入输出CSV文件路径:")
    print("(直接回车使用默认: 在原文件名后加 _transformed)")
    output_csv = input("> ").strip()
    
    if not output_csv:
        base, ext = os.path.splitext(input_csv)
        output_csv = f"{base}_transformed{ext}"
        print(f"  使用默认: {output_csv}")
    
    # 变换方法
    print("\n选择变换方法:")
    print("  1. segmented - 分段高精度变换（推荐）")
    print("  2. global    - 全局刚性变换（快速）")
    print("(直接回车使用默认: segmented)")
    method_input = input("> ").strip()
    
    if method_input == "2" or method_input.lower() == "global":
        method = "global"
    else:
        method = "segmented"
    print(f"  使用: {method}")
    
    # XYZ列索引
    print("\n请输入XYZ列索引（从0开始，用逗号分隔）:")
    print("(直接回车使用默认: 3,4,5 即第4,5,6列)")
    cols_input = input("> ").strip()
    
    if cols_input:
        try:
            xyz_cols = tuple(int(x.strip()) for x in cols_input.split(','))
        except:
            print("  输入格式错误，使用默认: (3,4,5)")
            xyz_cols = (3, 4, 5)
    else:
        xyz_cols = (3, 4, 5)
    print(f"  使用: {xyz_cols}")
    
    # 是否启用预处理
    print("\n是否启用预处理（高斯平滑，与配准一致）?")
    print("  1. 是（推荐，与配准流程一致）")
    print("  2. 否（使用原始数据）")
    print("(直接回车使用默认: 是)")
    preprocess_input = input("> ").strip()
    apply_preprocessing = preprocess_input != "2"
    print(f"  预处理: {'启用' if apply_preprocessing else '禁用'}")
    
    # 执行变换
    print("\n" + "-" * 60)
    transform_csv(input_csv, output_csv, method=method, xyz_cols=xyz_cols, 
                  apply_preprocessing=apply_preprocessing)
    
    # 询问是否验证
    print("\n是否验证变换结果?")
    print("(需要提供目标CSV文件路径)")
    print("  1. 是")
    print("  2. 否")
    print("(直接回车跳过)")
    verify_input = input("> ").strip()
    
    if verify_input == "1":
        print("\n请输入目标CSV文件路径:")
        target_csv = input("> ").strip()
        
        if target_csv and os.path.exists(target_csv):
            verify_transform_result(output_csv, target_csv, xyz_cols=xyz_cols, 
                                   use_arc_length_align=True, num_samples=5000)
        else:
            print(f"目标文件不存在: {target_csv}")


def main():
    """主函数"""
    if len(sys.argv) == 1:
        # 无参数，自动使用默认配置运行
        default_input = r"E:\Unity cangku\lighthouse_3.4（走数据备份）\Assets\Scripts\VisualServo\算法\圆形\圆形_tracker2 - tcp.csv"
        default_output = r"E:\Unity cangku\lighthouse_3.4（走数据备份）\Assets\Scripts\VisualServo\算法\圆形\圆形_tracker2 - tcp_transformed.csv"
        
        print("\n使用默认配置自动运行...")
        print(f"  输入: {default_input}")
        print(f"  输出: {default_output}")
        print(f"  方法: segmented (分段高精度)")
        print(f"  XYZ列: (3,4,5)")
        print(f"  预处理: 启用\n")
        
        if not os.path.exists(default_input):
            print(f"\n✗ 默认输入文件不存在: {default_input}")
            print("\n提示: 使用 --interactive 进入交互模式")
            print("      使用 -h 查看完整帮助")
            return
        
        try:
            transform_csv(default_input, default_output, 
                         method="segmented", 
                         xyz_cols=(3, 4, 5),
                         apply_preprocessing=True)
        except Exception as e:
            print(f"\n✗ 转换失败: {e}")
            import traceback
            traceback.print_exc()
    
    elif len(sys.argv) >= 2 and sys.argv[1] == '--interactive':
        # 交互模式
        interactive_mode()
    
    elif len(sys.argv) >= 3:
        # 命令行模式
        input_csv = sys.argv[1]
        output_csv = sys.argv[2]
        
        # 可选参数
        method = "segmented"
        xyz_cols = (3, 4, 5)
        apply_preprocessing = True
        target_csv = None
        
        for i, arg in enumerate(sys.argv[3:], 3):
            if arg == "--global":
                method = "global"
            elif arg.startswith("--cols="):
                cols_str = arg.replace("--cols=", "")
                xyz_cols = tuple(int(x) for x in cols_str.split(','))
            elif arg == "--no-preprocess":
                apply_preprocessing = False
            elif arg.startswith("--verify="):
                target_csv = arg.replace("--verify=", "")
        
        transform_csv(input_csv, output_csv, method=method, xyz_cols=xyz_cols,
                     apply_preprocessing=apply_preprocessing)
        
        # 如果指定了验证目标，执行验证
        if target_csv and os.path.exists(target_csv):
            verify_transform_result(output_csv, target_csv, xyz_cols=xyz_cols,
                                   use_arc_length_align=True)
    
    elif len(sys.argv) == 2 and sys.argv[1] in ["-h", "--help"]:
        print(__doc__)
        print("\n命令行参数:")
        print("  python apply_transform.py")
        print("      → 自动使用默认配置运行")
        print("")
        print("  python apply_transform.py --interactive")
        print("      → 进入交互模式")
        print("")
        print("  python apply_transform.py <input.csv> <output.csv> [选项]")
        print("\n选项:")
        print("  --global         使用全局刚性变换（默认使用分段高精度）")
        print("  --cols=3,4,5     指定XYZ列索引（从0开始）")
        print("  --no-preprocess  禁用预处理（不推荐）")
        print("  --verify=<file>  验证结果，与指定目标文件比较")
        print("\n示例:")
        print("  python apply_transform.py")
        print("  python apply_transform.py data.csv result.csv")
        print("  python apply_transform.py data.csv result.csv --global")
        print("  python apply_transform.py data.csv result.csv --verify=target.csv")
    
    else:
        print("参数错误，使用 -h 查看帮助")


if __name__ == "__main__":
    main()
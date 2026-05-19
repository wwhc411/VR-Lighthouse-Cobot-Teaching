# -*- coding: utf-8 -*-
"""
点到曲线投影距离误差计算工具 (Point-to-Curve Projection)

核心原理:
    不寻找"索引相同"的点，而是寻找"几何位置最近"的点。
    对于测量轨迹中的每一个点 P_A，在参考轨迹的线段上找到距离最近的投影点 P'_B，
    计算这两点间的距离（横向误差 Cross-Track Error）。

优点:
    - 完全忽略点数和采样率的差异
    - 物理意义明确：表示实际偏离参考路径的距离
    - 适用于非同步采集的轨迹对比

加速策略:
    - 使用 KD-Tree 快速定位候选线段
    - 避免全局遍历所有线段

适用场景: 机械臂TCP轨迹回放精度验证、路径跟踪误差分析
输入格式: CSV文件，位置数据在第4,5,6列（X_mm, Y_mm, Z_mm）

Author: GitHub Copilot
Date: 2026-02-05
"""

import pandas as pd
import numpy as np
import os
from scipy.spatial import cKDTree
from typing import Tuple, Dict, List, Optional

# ============ 中文字体配置（必须在导入pyplot之前）============
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'KaiTi', 'FangSong']
matplotlib.rcParams['axes.unicode_minus'] = False

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import warnings
warnings.filterwarnings('ignore')


# ============================================================================
#                           配置参数
# ============================================================================

CONFIG = {
    # 数据预处理
    'remove_duplicate_threshold': 0.01,  # mm，去重阈值
    'unit_auto_detect': True,            # 自动检测单位
    
    # KD-Tree加速参数
    'kdtree_k_neighbors': 5,             # 查找最近的k个顶点，用于确定候选线段
    
    # 可视化
    'trajectory_colors': {
        'reference': 'royalblue',        # 参考轨迹（Ground Truth）
        'measured': 'orangered'          # 测量轨迹
    },
    'marker_size': {
        'start': 200,
        'end': 200,
        'worst': 300
    },
    'error_colormap': 'RdYlGn_r',        # 误差颜色映射
    
    # 评级标准 (mm)
    'excellent_threshold': {'mean': 0.5, 'rmse': 0.8, 'max': 1.5},
    'good_threshold': {'mean': 2.0, 'rmse': 3.0, 'max': 5.0},
    'acceptable_threshold': {'mean': 5.0, 'rmse': 7.0, 'max': 10.0},
    
    # 报告
    'report_output_path': 'trajectory_projection_report.txt',
    'figure_save_dpi': 150
}


# ============================================================================
#                        数据加载与预处理
# ============================================================================

def load_tcp_trajectory(csv_path: str) -> np.ndarray:
    """
    加载TCP轨迹CSV文件
    
    CSV格式:
    FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,...
    
    位置数据从第4,5,6列（索引3,4,5）读取
    
    Args:
        csv_path: CSV文件路径
        
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


# ============================================================================
#                    点到线段距离计算（核心几何函数）
# ============================================================================

def point_to_segment_projection(point: np.ndarray, 
                                 seg_start: np.ndarray, 
                                 seg_end: np.ndarray) -> Tuple[float, np.ndarray, float]:
    """
    计算点到线段的投影距离和投影点
    
    几何原理:
              Point P
                *
               /|
              / | distance (垂直距离/横向误差)
             /  |
            /   ↓
       A *------*------* B
          ←--→  P'
          t (投影参数)
    
    Args:
        point: (3,) 待测点 P
        seg_start: (3,) 线段起点 A
        seg_end: (3,) 线段终点 B
    
    Returns:
        Tuple[float, np.ndarray, float]:
            - distance: 点到线段的最短距离
            - projection: 投影点坐标 P'
            - t: 投影参数 (0=起点, 1=终点, 0~1=中间)
    """
    # 线段向量 AB
    segment_vec = seg_end - seg_start
    segment_length_sq = np.dot(segment_vec, segment_vec)
    
    # 退化为点的情况
    if segment_length_sq < 1e-12:
        return np.linalg.norm(point - seg_start), seg_start.copy(), 0.0
    
    # 点到线段起点的向量 AP
    point_vec = point - seg_start
    
    # 计算投影参数 t = (AP · AB) / |AB|²
    t = np.dot(point_vec, segment_vec) / segment_length_sq
    
    # 将 t 限制在 [0, 1] 范围内
    t_clamped = np.clip(t, 0.0, 1.0)
    
    # 计算投影点 P' = A + t * AB
    projection = seg_start + t_clamped * segment_vec
    
    # 计算距离
    distance = np.linalg.norm(point - projection)
    
    return distance, projection, t_clamped


def point_to_polyline_distance(point: np.ndarray, 
                                polyline: np.ndarray,
                                segment_indices: Optional[List[int]] = None) -> Tuple[float, np.ndarray, int, float]:
    """
    计算点到折线（多段线）的最短距离
    
    Args:
        point: (3,) 待测点
        polyline: (N, 3) 折线顶点数组
        segment_indices: 可选，仅搜索指定的线段索引列表（用于KD-Tree加速）
    
    Returns:
        Tuple[float, np.ndarray, int, float]:
            - min_distance: 最短距离
            - closest_projection: 最近投影点坐标
            - closest_segment_idx: 最近线段的起点索引
            - closest_t: 投影参数
    """
    n_vertices = len(polyline)
    
    if segment_indices is None:
        # 遍历所有线段
        segment_indices = range(n_vertices - 1)
    
    min_distance = float('inf')
    closest_projection = None
    closest_segment_idx = -1
    closest_t = 0.0
    
    for seg_idx in segment_indices:
        if seg_idx >= n_vertices - 1:
            continue
            
        seg_start = polyline[seg_idx]
        seg_end = polyline[seg_idx + 1]
        
        distance, projection, t = point_to_segment_projection(point, seg_start, seg_end)
        
        if distance < min_distance:
            min_distance = distance
            closest_projection = projection
            closest_segment_idx = seg_idx
            closest_t = t
    
    return min_distance, closest_projection, closest_segment_idx, closest_t


# ============================================================================
#                       KD-Tree 加速查找
# ============================================================================

class PolylineProjector:
    """
    使用KD-Tree加速的折线投影器
    
    原理:
        1. 对折线顶点建立KD-Tree
        2. 对于查询点，先找到最近的k个顶点
        3. 只在这些顶点相邻的线段中搜索最近投影
    """
    
    def __init__(self, polyline: np.ndarray, k_neighbors: int = 5):
        """
        初始化投影器
        
        Args:
            polyline: (N, 3) 参考折线顶点
            k_neighbors: 查找最近的k个顶点
        """
        self.polyline = polyline
        self.k_neighbors = min(k_neighbors, len(polyline))
        self.n_vertices = len(polyline)
        
        # 建立KD-Tree
        print(f"[INFO] 构建KD-Tree，顶点数: {self.n_vertices}")
        self.kdtree = cKDTree(polyline)
        
        # 预计算每个顶点相邻的线段索引
        # 顶点i相邻的线段: i-1 (如果存在) 和 i
        self.vertex_to_segments = []
        for i in range(self.n_vertices):
            segments = []
            if i > 0:
                segments.append(i - 1)  # 前一条线段
            if i < self.n_vertices - 1:
                segments.append(i)      # 后一条线段
            self.vertex_to_segments.append(segments)
    
    def project_point(self, point: np.ndarray) -> Tuple[float, np.ndarray, int, float]:
        """
        将点投影到折线上
        
        Args:
            point: (3,) 查询点
        
        Returns:
            Tuple[float, np.ndarray, int, float]:
                - distance: 投影距离
                - projection: 投影点坐标
                - segment_idx: 所在线段索引
                - t: 线段参数
        """
        # 使用KD-Tree找到最近的k个顶点
        _, nearest_indices = self.kdtree.query(point, k=self.k_neighbors)
        
        # 如果只有一个邻居，确保是数组
        if isinstance(nearest_indices, (int, np.integer)):
            nearest_indices = [nearest_indices]
        
        # 收集候选线段（去重）
        candidate_segments = set()
        for vertex_idx in nearest_indices:
            candidate_segments.update(self.vertex_to_segments[vertex_idx])
        
        # 在候选线段中搜索最近投影
        return point_to_polyline_distance(point, self.polyline, list(candidate_segments))
    
    def project_points(self, points: np.ndarray, 
                       verbose: bool = True) -> Dict[str, np.ndarray]:
        """
        批量投影多个点
        
        Args:
            points: (M, 3) 查询点数组
            verbose: 是否显示进度
        
        Returns:
            Dict containing:
                - 'distances': (M,) 距离数组
                - 'projections': (M, 3) 投影点数组
                - 'segment_indices': (M,) 线段索引数组
                - 'parameters': (M,) 投影参数数组
        """
        n_points = len(points)
        
        distances = np.zeros(n_points)
        projections = np.zeros((n_points, 3))
        segment_indices = np.zeros(n_points, dtype=int)
        parameters = np.zeros(n_points)
        
        for i, point in enumerate(points):
            if verbose and (i + 1) % 500 == 0:
                print(f"  投影进度: {i+1}/{n_points} ({100*(i+1)/n_points:.1f}%)")
            
            dist, proj, seg_idx, t = self.project_point(point)
            
            distances[i] = dist
            projections[i] = proj
            segment_indices[i] = seg_idx
            parameters[i] = t
        
        return {
            'distances': distances,
            'projections': projections,
            'segment_indices': segment_indices,
            'parameters': parameters
        }


# ============================================================================
#                    核心误差计算函数
# ============================================================================

def compute_projection_error(measured_traj: np.ndarray, 
                              reference_traj: np.ndarray,
                              use_kdtree: bool = True) -> Dict:
    """
    计算测量轨迹到参考轨迹的投影误差
    
    原理 (Point-to-Curve Projection):
        对于测量轨迹中的每一个点 P，找到它在参考轨迹折线上的最近投影点 P'，
        计算横向误差 (Cross-Track Error) = ||P - P'||
    
    Args:
        measured_traj: (N, 3) 测量轨迹（待评估）
        reference_traj: (M, 3) 参考轨迹（Ground Truth）
        use_kdtree: 是否使用KD-Tree加速
    
    Returns:
        Dict containing:
            - 'distances': (N,) 每个测量点的投影距离
            - 'projections': (N, 3) 每个测量点在参考轨迹上的投影点
            - 'segment_indices': (N,) 每个投影点所在的线段索引
            - 'mean': 平均误差
            - 'std': 标准差
            - 'rmse': 均方根误差
            - 'max': 最大误差
            - 'min': 最小误差
            - 'p95': 95百分位数
            - 'p99': 99百分位数
            - 'median': 中位数
            - 'worst_idx': 最大误差点的索引
    """
    print(f"\n[计算] 点到曲线投影误差...")
    print(f"  测量轨迹点数: {len(measured_traj)}")
    print(f"  参考轨迹点数: {len(reference_traj)}")
    
    if use_kdtree:
        print(f"  使用KD-Tree加速 (k={CONFIG['kdtree_k_neighbors']})")
        projector = PolylineProjector(reference_traj, CONFIG['kdtree_k_neighbors'])
        proj_result = projector.project_points(measured_traj, verbose=True)
    else:
        print(f"  使用暴力搜索（无加速）")
        n_points = len(measured_traj)
        distances = np.zeros(n_points)
        projections = np.zeros((n_points, 3))
        segment_indices = np.zeros(n_points, dtype=int)
        
        for i, point in enumerate(measured_traj):
            if (i + 1) % 500 == 0:
                print(f"  进度: {i+1}/{n_points} ({100*(i+1)/n_points:.1f}%)")
            
            dist, proj, seg_idx, _ = point_to_polyline_distance(point, reference_traj)
            distances[i] = dist
            projections[i] = proj
            segment_indices[i] = seg_idx
        
        proj_result = {
            'distances': distances,
            'projections': projections,
            'segment_indices': segment_indices
        }
    
    distances = proj_result['distances']
    
    # 计算统计指标
    result = {
        'distances': distances,
        'projections': proj_result['projections'],
        'segment_indices': proj_result['segment_indices'],
        'mean': np.mean(distances),
        'std': np.std(distances),
        'rmse': np.sqrt(np.mean(distances ** 2)),
        'max': np.max(distances),
        'min': np.min(distances),
        'p95': np.percentile(distances, 95),
        'p99': np.percentile(distances, 99),
        'median': np.median(distances),
        'worst_idx': np.argmax(distances)
    }
    
    print(f"\n[结果] 投影误差统计:")
    print(f"  平均误差 (Mean):   {result['mean']:.4f} mm")
    print(f"  均方根误差 (RMSE): {result['rmse']:.4f} mm")
    print(f"  最大误差 (Max):    {result['max']:.4f} mm")
    print(f"  中位数 (Median):   {result['median']:.4f} mm")
    print(f"  P95:               {result['p95']:.4f} mm")
    
    return result


def compute_bidirectional_projection_error(traj_a: np.ndarray, 
                                            traj_b: np.ndarray) -> Dict:
    """
    计算双向投影误差（对称误差）
    
    原理:
        1. A → B: 计算轨迹A中每个点到轨迹B的投影距离
        2. B → A: 计算轨迹B中每个点到轨迹A的投影距离
        3. 综合两个方向的误差
    
    优点:
        - 不依赖于哪条轨迹是"参考"
        - 更全面地评估两条轨迹的相似度
    
    Args:
        traj_a: (N, 3) 轨迹A
        traj_b: (M, 3) 轨迹B
    
    Returns:
        Dict containing:
            - 'a_to_b': 轨迹A到B的投影误差结果
            - 'b_to_a': 轨迹B到A的投影误差结果
            - 'symmetric_mean': 对称平均误差
            - 'symmetric_rmse': 对称RMSE
            - 'symmetric_max': 双向最大误差
            - 'hausdorff': Hausdorff距离（双向最大误差的最大值）
    """
    print("\n" + "="*60)
    print("【双向投影误差计算】")
    print("="*60)
    
    print("\n[方向1] 轨迹A → 轨迹B (A投影到B)")
    a_to_b = compute_projection_error(traj_a, traj_b)
    
    print("\n[方向2] 轨迹B → 轨迹A (B投影到A)")
    b_to_a = compute_projection_error(traj_b, traj_a)
    
    # 合并两个方向的距离
    all_distances = np.concatenate([a_to_b['distances'], b_to_a['distances']])
    
    result = {
        'a_to_b': a_to_b,
        'b_to_a': b_to_a,
        'symmetric_mean': np.mean(all_distances),
        'symmetric_rmse': np.sqrt(np.mean(all_distances ** 2)),
        'symmetric_max': np.max(all_distances),
        'hausdorff': max(a_to_b['max'], b_to_a['max'])  # Hausdorff距离
    }
    
    print("\n" + "-"*60)
    print("[综合结果] 双向投影误差:")
    print(f"  对称平均误差: {result['symmetric_mean']:.4f} mm")
    print(f"  对称RMSE:     {result['symmetric_rmse']:.4f} mm")
    print(f"  Hausdorff距离: {result['hausdorff']:.4f} mm")
    
    return result


# ============================================================================
#                          端点误差计算
# ============================================================================

def compute_endpoint_errors(measured_traj: np.ndarray, 
                            reference_traj: np.ndarray) -> Dict:
    """
    计算起点和终点的定位误差
    
    Args:
        measured_traj: (N, 3) 测量轨迹
        reference_traj: (M, 3) 参考轨迹
    
    Returns:
        Dict containing endpoint errors
    """
    print(f"\n[计算] 端点误差...")
    
    start_error = np.linalg.norm(measured_traj[0] - reference_traj[0])
    end_error = np.linalg.norm(measured_traj[-1] - reference_traj[-1])
    
    start_vector = measured_traj[0] - reference_traj[0]
    end_vector = measured_traj[-1] - reference_traj[-1]
    
    print(f"  起点误差: {start_error:.4f} mm")
    print(f"  终点误差: {end_error:.4f} mm")
    
    return {
        'start_error': start_error,
        'end_error': end_error,
        'start_vector': start_vector,
        'end_vector': end_vector,
        'max_endpoint_error': max(start_error, end_error)
    }


# ============================================================================
#                         综合评级
# ============================================================================

def determine_overall_grade(projection_error: Dict, 
                            endpoints: Dict) -> Tuple[str, bool]:
    """
    根据投影误差和端点误差综合判定等级
    
    评级规则:
        优秀: Mean<0.5mm AND RMSE<0.8mm AND Max<1.5mm
        良好: Mean<2.0mm AND RMSE<3.0mm AND Max<5.0mm
        可接受: Mean<5.0mm AND RMSE<7.0mm AND Max<10.0mm
        需优化: 其他情况
    
    Args:
        projection_error: 投影误差结果
        endpoints: 端点误差结果
    
    Returns:
        Tuple[str, bool]: (等级字符串, 是否通过)
    """
    mean_err = projection_error['mean']
    rmse_err = projection_error['rmse']
    max_err = projection_error['max']
    
    excellent = CONFIG['excellent_threshold']
    good = CONFIG['good_threshold']
    acceptable = CONFIG['acceptable_threshold']
    
    if mean_err < excellent['mean'] and rmse_err < excellent['rmse'] and max_err < excellent['max']:
        return "优秀 (Excellent)", True
    elif mean_err < good['mean'] and rmse_err < good['rmse'] and max_err < good['max']:
        return "良好 (Good)", True
    elif mean_err < acceptable['mean'] and rmse_err < acceptable['rmse'] and max_err < acceptable['max']:
        return "可接受 (Acceptable)", True
    else:
        return "需优化 (Needs Improvement)", False


# ============================================================================
#                        可视化函数
# ============================================================================

def plot_projection_error_analysis(measured_traj: np.ndarray,
                                    reference_traj: np.ndarray,
                                    projection_result: Dict,
                                    endpoint_result: Dict,
                                    overall_grade: str,
                                    save_path: Optional[str] = None):
    """
    生成投影误差综合分析图表
    
    包含4个子图:
    1. 3D轨迹对比图（误差着色）- 左上
    2. 误差沿轨迹变化曲线 - 右上
    3. 误差分布直方图 - 左下
    4. 统计信息面板 - 右下
    
    Args:
        measured_traj: 测量轨迹
        reference_traj: 参考轨迹
        projection_result: 投影误差结果
        endpoint_result: 端点误差结果
        overall_grade: 综合评级
        save_path: 保存路径（可选）
    """
    fig = plt.figure(figsize=(16, 12))
    
    colors = CONFIG['trajectory_colors']
    sizes = CONFIG['marker_size']
    distances = projection_result['distances']
    projections = projection_result['projections']
    
    # ===== 图1: 3D轨迹对比（误差着色）=====
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    
    # 绘制参考轨迹
    ax1.plot(reference_traj[:, 0], reference_traj[:, 1], reference_traj[:, 2],
             color=colors['reference'], linewidth=2.5, label='参考轨迹', alpha=0.8)
    
    # 绘制测量轨迹（按误差着色）
    scatter = ax1.scatter(measured_traj[:, 0], measured_traj[:, 1], measured_traj[:, 2],
                          c=distances, cmap=CONFIG['error_colormap'], 
                          s=20, alpha=0.8, label='测量轨迹')
    
    # 添加颜色条
    cbar = plt.colorbar(scatter, ax=ax1, shrink=0.6, pad=0.1)
    cbar.set_label('投影误差 (mm)', fontsize=10)
    
    # 起点终点
    ax1.scatter(*reference_traj[0], c='green', s=sizes['start'], marker='o',
                edgecolors='darkgreen', linewidths=3, label='起点', zorder=5)
    ax1.scatter(*reference_traj[-1], c='red', s=sizes['end'], marker='s',
                edgecolors='darkred', linewidths=3, label='终点', zorder=5)
    
    # 标注最大偏差点
    worst_idx = projection_result['worst_idx']
    ax1.scatter(*measured_traj[worst_idx], c='purple', s=sizes['worst'], marker='*',
                edgecolors='black', linewidths=2, label='最大偏差点', zorder=6)
    
    # 绘制最大误差的投影线
    ax1.plot([measured_traj[worst_idx, 0], projections[worst_idx, 0]],
             [measured_traj[worst_idx, 1], projections[worst_idx, 1]],
             [measured_traj[worst_idx, 2], projections[worst_idx, 2]],
             'purple', linewidth=2, linestyle='--', alpha=0.8)
    
    ax1.set_xlabel('X (mm)', fontsize=10, fontweight='bold')
    ax1.set_ylabel('Y (mm)', fontsize=10, fontweight='bold')
    ax1.set_zlabel('Z (mm)', fontsize=10, fontweight='bold')
    ax1.set_title('3D轨迹对比（按误差着色）', fontsize=12, fontweight='bold', pad=15)
    ax1.legend(fontsize=9, loc='upper left')
    ax1.grid(True, alpha=0.3)
    
    # 设置等比例坐标轴
    all_points = np.vstack([reference_traj, measured_traj])
    max_range = np.array([all_points[:, 0].max() - all_points[:, 0].min(),
                          all_points[:, 1].max() - all_points[:, 1].min(),
                          all_points[:, 2].max() - all_points[:, 2].min()]).max() / 2.0
    
    mid_x = (all_points[:, 0].max() + all_points[:, 0].min()) * 0.5
    mid_y = (all_points[:, 1].max() + all_points[:, 1].min()) * 0.5
    mid_z = (all_points[:, 2].max() + all_points[:, 2].min()) * 0.5
    
    ax1.set_xlim(mid_x - max_range, mid_x + max_range)
    ax1.set_ylim(mid_y - max_range, mid_y + max_range)
    ax1.set_zlim(mid_z - max_range, mid_z + max_range)
    
    # ===== 图2: 误差沿轨迹变化曲线 =====
    ax2 = fig.add_subplot(2, 2, 2)
    
    point_indices = np.arange(len(distances))
    ax2.plot(point_indices, distances, color='steelblue', linewidth=1.5, alpha=0.8)
    ax2.fill_between(point_indices, 0, distances, alpha=0.3, color='steelblue')
    
    # 标注统计线
    ax2.axhline(projection_result['mean'], color='red', linestyle='--',
                linewidth=2, label=f"平均: {projection_result['mean']:.3f} mm")
    ax2.axhline(projection_result['p95'], color='orange', linestyle='--',
                linewidth=2, label=f"P95: {projection_result['p95']:.3f} mm")
    ax2.axhline(projection_result['max'], color='darkred', linestyle=':',
                linewidth=2, label=f"最大: {projection_result['max']:.3f} mm")
    
    # 标注最大误差点
    ax2.scatter(worst_idx, distances[worst_idx], c='purple', s=150, marker='*',
                edgecolors='black', linewidths=2, zorder=5)
    ax2.annotate(f'最大误差\n{distances[worst_idx]:.3f} mm',
                 xy=(worst_idx, distances[worst_idx]),
                 xytext=(worst_idx + len(distances)*0.05, distances[worst_idx] * 0.9),
                 fontsize=9, arrowprops=dict(arrowstyle='->', color='purple'))
    
    ax2.set_xlabel('测量点索引', fontsize=10, fontweight='bold')
    ax2.set_ylabel('投影误差 (mm)', fontsize=10, fontweight='bold')
    ax2.set_title('误差沿轨迹变化曲线', fontsize=12, fontweight='bold', pad=10)
    ax2.legend(fontsize=9, loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(0, len(distances) - 1)
    ax2.set_ylim(0, None)
    
    # ===== 图3: 误差分布直方图 =====
    ax3 = fig.add_subplot(2, 2, 3)
    
    n, bins, patches = ax3.hist(distances, bins=50, color='steelblue',
                                 edgecolor='black', alpha=0.7)
    
    # 根据bin值给直方图上色
    cm = plt.cm.RdYlGn_r
    bin_centers = 0.5 * (bins[:-1] + bins[1:])
    col = (bin_centers - bin_centers.min()) / (bin_centers.max() - bin_centers.min() + 1e-8)
    for c, p in zip(col, patches):
        plt.setp(p, 'facecolor', cm(c))
    
    # 标注统计值
    ax3.axvline(projection_result['mean'], color='red', linestyle='--',
                linewidth=2.5, label=f"平均: {projection_result['mean']:.3f} mm")
    ax3.axvline(projection_result['median'], color='green', linestyle='--',
                linewidth=2.5, label=f"中位数: {projection_result['median']:.3f} mm")
    ax3.axvline(projection_result['p95'], color='orange', linestyle='--',
                linewidth=2.5, label=f"P95: {projection_result['p95']:.3f} mm")
    
    ax3.set_xlabel('投影误差 (mm)', fontsize=10, fontweight='bold')
    ax3.set_ylabel('频数', fontsize=10, fontweight='bold')
    ax3.set_title('投影误差分布直方图', fontsize=12, fontweight='bold', pad=10)
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3, axis='y')
    
    # ===== 图4: 统计信息面板 =====
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.axis('off')
    
    info_text = f"""
    ================================================================
           点到曲线投影误差分析报告 (Point-to-Curve Projection)
    ================================================================
    
    【投影误差统计】
       平均误差 (Mean):     {projection_result['mean']:>10.4f} mm
       均方根误差 (RMSE):   {projection_result['rmse']:>10.4f} mm
       标准差 (Std):        {projection_result['std']:>10.4f} mm
       中位数 (Median):     {projection_result['median']:>10.4f} mm
       最小误差 (Min):      {projection_result['min']:>10.4f} mm
       最大误差 (Max):      {projection_result['max']:>10.4f} mm
       P95:                 {projection_result['p95']:>10.4f} mm
       P99:                 {projection_result['p99']:>10.4f} mm
       
       最大偏差点索引:      #{projection_result['worst_idx']:>6}
    
    【端点定位误差】
       起点误差:            {endpoint_result['start_error']:>10.4f} mm
       终点误差:            {endpoint_result['end_error']:>10.4f} mm
    
    ----------------------------------------------------------------
                    ⭐ 综合评级: {overall_grade} ⭐
    ----------------------------------------------------------------
    
    验收标准参考:
      优秀:   Mean<0.5mm, RMSE<0.8mm, Max<1.5mm
      良好:   Mean<2.0mm, RMSE<3.0mm, Max<5.0mm
      可接受: Mean<5.0mm, RMSE<7.0mm, Max<10.0mm
    
    ================================================================
    """
    
    ax4.text(0.05, 0.5, info_text, transform=ax4.transAxes,
             fontsize=9.5, verticalalignment='center',
             fontfamily='sans-serif',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))
    
    plt.suptitle('机械臂TCP轨迹投影误差分析 (Point-to-Curve Projection)',
                 fontsize=14, fontweight='bold', y=0.98)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    if save_path:
        plt.savefig(save_path, dpi=CONFIG['figure_save_dpi'], bbox_inches='tight')
        print(f"\n[INFO] 可视化图表已保存: {save_path}")
    
    plt.show()


# ============================================================================
#                          报告保存
# ============================================================================

def save_evaluation_report(result: Dict, 
                           measured_csv: str, 
                           reference_csv: str):
    """
    保存文本格式的评估报告
    
    Args:
        result: 评估结果字典
        measured_csv: 测量轨迹文件路径
        reference_csv: 参考轨迹文件路径
    """
    report_path = CONFIG['report_output_path']
    
    proj = result['projection']
    endpoints = result['endpoints']
    
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write("="*70 + "\n")
        f.write("   点到曲线投影误差分析报告 (Point-to-Curve Projection)\n")
        f.write("="*70 + "\n\n")
        
        f.write("【文件信息】\n")
        f.write(f"  测量轨迹: {os.path.basename(measured_csv)}\n")
        f.write(f"  参考轨迹: {os.path.basename(reference_csv)}\n\n")
        
        f.write("【投影误差统计】\n")
        f.write(f"   平均误差 (Mean):     {proj['mean']:.4f} mm\n")
        f.write(f"   均方根误差 (RMSE):   {proj['rmse']:.4f} mm\n")
        f.write(f"   标准差 (Std):        {proj['std']:.4f} mm\n")
        f.write(f"   中位数 (Median):     {proj['median']:.4f} mm\n")
        f.write(f"   最小误差 (Min):      {proj['min']:.4f} mm\n")
        f.write(f"   最大误差 (Max):      {proj['max']:.4f} mm\n")
        f.write(f"   P95:                 {proj['p95']:.4f} mm\n")
        f.write(f"   P99:                 {proj['p99']:.4f} mm\n")
        f.write(f"   最大偏差点索引:      #{proj['worst_idx']}\n\n")
        
        f.write("【端点定位误差】\n")
        f.write(f"   起点误差:  {endpoints['start_error']:.4f} mm\n")
        f.write(f"   终点误差:  {endpoints['end_error']:.4f} mm\n\n")
        
        f.write("-"*70 + "\n")
        f.write(f"【综合评级】: {result['overall_grade']}\n")
        f.write(f"【验收结果】: {'通过 ✓' if result['pass'] else '不通过 ✗'}\n")
        f.write("="*70 + "\n")
    
    print(f"  报告已保存: {report_path}")


# ============================================================================
#                          主函数接口
# ============================================================================

def compute_trajectory_projection_error(measured_csv: str, 
                                         reference_csv: str,
                                         visualize: bool = True,
                                         save_report: bool = False,
                                         use_kdtree: bool = True,
                                         bidirectional: bool = False) -> Dict:
    """
    点到曲线投影误差计算（主函数）
    
    原理:
        对于测量轨迹中的每一个点 P，找到它在参考轨迹折线上的最近投影点 P'，
        计算横向误差 (Cross-Track Error)。
    
    Args:
        measured_csv: 测量轨迹CSV文件路径（待评估轨迹）
        reference_csv: 参考轨迹CSV文件路径（Ground Truth）
        visualize: 是否生成可视化图表
        save_report: 是否保存文本报告
        use_kdtree: 是否使用KD-Tree加速
        bidirectional: 是否计算双向投影误差
    
    Returns:
        Dict: 完整评估结果
        {
            'projection': {...},     # 投影误差结果
            'endpoints': {...},      # 端点误差结果
            'overall_grade': str,    # 综合评级
            'pass': bool,            # 是否通过验收
            'bidirectional': {...}   # 双向误差（如果启用）
        }
    """
    print("\n" + "="*70)
    print("    点到曲线投影误差分析 (Point-to-Curve Projection)")
    print("="*70)
    
    # 1. 加载数据
    print("\n【步骤1/5】加载轨迹数据...")
    print("-" * 70)
    measured_traj = load_tcp_trajectory(measured_csv)
    reference_traj = load_tcp_trajectory(reference_csv)
    
    # 2. 计算投影误差
    print("\n【步骤2/5】计算投影误差...")
    print("-" * 70)
    
    projection_result = compute_projection_error(measured_traj, reference_traj, use_kdtree)
    
    # 3. 计算端点误差
    print("\n【步骤3/5】计算端点误差...")
    print("-" * 70)
    endpoint_result = compute_endpoint_errors(measured_traj, reference_traj)
    
    # 4. 综合评级
    print("\n【步骤4/5】综合评级...")
    print("-" * 70)
    overall_grade, is_pass = determine_overall_grade(projection_result, endpoint_result)
    print(f"  综合评级: {overall_grade}")
    print(f"  验收结果: {'通过 ✓' if is_pass else '不通过 ✗'}")
    
    # 构建结果字典
    result = {
        'projection': projection_result,
        'endpoints': endpoint_result,
        'overall_grade': overall_grade,
        'pass': is_pass
    }
    
    # 可选：双向投影误差
    if bidirectional:
        print("\n【附加】计算双向投影误差...")
        print("-" * 70)
        result['bidirectional'] = compute_bidirectional_projection_error(
            measured_traj, reference_traj)
    
    # 5. 可视化
    if visualize:
        print("\n【步骤5/5】生成可视化图表...")
        print("-" * 70)
        plot_projection_error_analysis(
            measured_traj, reference_traj,
            projection_result, endpoint_result,
            overall_grade,
            save_path='trajectory_projection_analysis.png' if save_report else None
        )
    
    # 保存报告
    if save_report:
        print("\n【保存报告】...")
        print("-" * 70)
        save_evaluation_report(result, measured_csv, reference_csv)
    
    print("\n" + "="*70)
    print("                    分析完成!")
    print("="*70)
    
    return result


# ============================================================================
#                            主程序
# ============================================================================

if __name__ == "__main__":
    # ========== 配置文件路径 ==========
    
    # 参考轨迹（Ground Truth）
    reference_csv = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\trackerb.csv"
    
    # 测量轨迹（待评估轨迹）
    measured_csv = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\trackera.csv"
    
    # ========== 执行分析 ==========
    
    result = compute_trajectory_projection_error(
        measured_csv=measured_csv,
        reference_csv=reference_csv,
        visualize=True,          # 生成可视化图表
        save_report=True,        # 保存文本报告
        use_kdtree=True,         # 使用KD-Tree加速
        bidirectional=False      # 是否计算双向误差
    )
    
    # ========== 打印摘要 ==========
    
    print("\n" + "="*70)
    print("                      评估结果摘要")
    print("="*70)
    
    proj = result['projection']
    endpoints = result['endpoints']
    
    print(f"\n【投影误差】")
    print(f"   平均误差 (Mean):     {proj['mean']:.4f} mm")
    print(f"   均方根误差 (RMSE):   {proj['rmse']:.4f} mm")
    print(f"   最大误差 (Max):      {proj['max']:.4f} mm")
    print(f"   P95:                 {proj['p95']:.4f} mm")
    
    print(f"\n【端点误差】")
    print(f"   起点误差:            {endpoints['start_error']:.4f} mm")
    print(f"   终点误差:            {endpoints['end_error']:.4f} mm")
    
    print(f"\n【综合评级】:           {result['overall_grade']}")
    print(f"【验收状态】:           {'✓ 通过' if result['pass'] else '✗ 不通过'}")
    
    print("\n" + "="*70)

# -*- coding: utf-8 -*-
"""
机械臂TCP轨迹回放匹配误差计算工具

核心功能:
1. 平均垂直距离 (Average Perpendicular Distance) - 轨迹复现精度
2. 离散Fréchet距离 (Discrete Fréchet Distance) - 最大轨迹偏差
3. 端点误差 (Endpoint Error) - 起止点定位精度

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
    
    # 评级标准 (mm)
    'excellent_threshold': {'apd': 0.5, 'frechet': 1.5, 'endpoint': 1.0},
    'good_threshold': {'apd': 2.0, 'frechet': 5.0, 'endpoint': 2.0},
    'acceptable_threshold': {'apd': 5.0, 'frechet': 10.0, 'endpoint': 5.0},
    
    # 报告
    'report_output_path': 'trajectory_matching_report.txt',
    'figure_save_dpi': 150
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
    
    distances = []
    n_replay = len(replay_traj)
    n_teach = len(teach_traj)
    
    # 遍历回放轨迹的每个点
    for i, point in enumerate(replay_traj):
        if (i + 1) % 500 == 0:
            print(f"  进度: {i+1}/{n_replay} ({100*(i+1)/n_replay:.1f}%)")
        
        min_distance = float('inf')
        
        # 遍历示教轨迹的每个线段
        for j in range(n_teach - 1):
            seg_start = teach_traj[j]
            seg_end = teach_traj[j + 1]
            
            distance = point_to_segment_distance(point, seg_start, seg_end)
            
            if distance < min_distance:
                min_distance = distance
        
        distances.append(min_distance)
    
    distances = np.array(distances)
    
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
    
    print(f"  构建 {n}x{m} 距离矩阵...")
    
    # 步骤1: 计算距离矩阵
    dist_matrix = np.zeros((n, m))
    for i in range(n):
        if (i + 1) % 200 == 0:
            print(f"  进度: {i+1}/{n} ({100*(i+1)/n:.1f}%)")
        for j in range(m):
            dist_matrix[i, j] = np.linalg.norm(replay_traj[i] - teach_traj[j])
    
    print(f"  动态规划求解最优路径...")
    
    # 步骤2: 动态规划
    dp = np.full((n, m), np.inf)
    dp[0, 0] = dist_matrix[0, 0]
    
    # 初始化第一行和第一列
    for i in range(1, n):
        dp[i, 0] = max(dp[i-1, 0], dist_matrix[i, 0])
    
    for j in range(1, m):
        dp[0, j] = max(dp[0, j-1], dist_matrix[0, j])
    
    # 填充DP表
    for i in range(1, n):
        for j in range(1, m):
            candidates = [
                dp[i-1, j],      # 回放前进
                dp[i, j-1],      # 示教前进
                dp[i-1, j-1]     # 同时前进
            ]
            min_prev = min(candidates)
            dp[i, j] = max(dist_matrix[i, j], min_prev)
    
    frechet_distance = dp[n-1, m-1]
    
    # 找到最大偏差点
    max_dist = 0
    worst_i, worst_j = 0, 0
    for i in range(n):
        for j in range(m):
            if dist_matrix[i, j] > max_dist and dist_matrix[i, j] <= frechet_distance:
                max_dist = dist_matrix[i, j]
                worst_i, worst_j = i, j
    
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
#                         综合评级
# ============================================================================

def determine_overall_grade(apd, frechet, endpoints):
    """
    根据三大指标综合判定等级
    
    评级规则:
        优秀: APD<0.5mm AND Fréchet<1.5mm AND 端点<1.0mm
        良好: APD<2.0mm AND Fréchet<5.0mm AND 端点<2.0mm
        可接受: APD<5.0mm AND Fréchet<10.0mm AND 端点<5.0mm
        需优化: 其他情况
    
    Args:
        apd (dict): APD计算结果
        frechet (dict): Fréchet距离结果
        endpoints (dict): 端点误差结果
    
    Returns:
        tuple: (等级字符串, 是否通过)
    """
    apd_mean = apd['mean']
    frechet_dist = frechet['frechet_distance']
    max_endpoint = max(endpoints['start_error'], endpoints['end_error'])
    
    excellent = CONFIG['excellent_threshold']
    good = CONFIG['good_threshold']
    acceptable = CONFIG['acceptable_threshold']
    
    if apd_mean < excellent['apd'] and frechet_dist < excellent['frechet'] and max_endpoint < excellent['endpoint']:
        return "优秀 (Excellent)", True
    elif apd_mean < good['apd'] and frechet_dist < good['frechet'] and max_endpoint < good['endpoint']:
        return "良好 (Good)", True
    elif apd_mean < acceptable['apd'] and frechet_dist < acceptable['frechet'] and max_endpoint < acceptable['endpoint']:
        return "可接受 (Acceptable)", True
    else:
        return "需优化 (Needs Improvement)", False


# ============================================================================
#                        可视化函数
# ============================================================================

def plot_trajectory_matching_analysis(replay_traj, teach_traj, 
                                      apd_result, frechet_result, endpoint_result,
                                      overall_grade, save_path=None):
    """
    生成轨迹匹配误差综合分析图表
    
    包含4个子图:
    1. 3D轨迹对比图（标注最大偏差点）- 左上，占据两列
    2. 误差随点索引折线图 - 左下
    3. 垂直距离分布直方图 - 中下
    4. 统计信息面板 - 右侧，占据两行
    
    Args:
        replay_traj: 回放轨迹
        teach_traj: 示教轨迹
        apd_result: APD计算结果
        frechet_result: Fréchet距离结果
        endpoint_result: 端点误差结果
        overall_grade: 综合评级
        save_path: 保存路径（可选）
    """
    fig = plt.figure(figsize=(18, 12))
    
    # 使用GridSpec创建布局：2行3列，右侧统计面板占据整列
    import matplotlib.gridspec as gridspec
    gs = gridspec.GridSpec(2, 3, figure=fig, width_ratios=[1.2, 1.2, 1], height_ratios=[1, 1],
                          hspace=0.3, wspace=0.35)
    
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
    
    # 标注最大偏差点
    worst_i = frechet_result['worst_replay_idx']
    worst_j = frechet_result['worst_teach_idx']
    ax1.scatter(*replay_traj[worst_i], c='purple', s=sizes['worst'], marker='*', 
               edgecolors='black', linewidths=2, label='最大偏差点', zorder=6)
    
    # 绘制误差向量
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
    
    # ===== 图2: 误差随点索引折线图（左下）=====
    ax2 = fig.add_subplot(gs[1, 0])
    
    distances = apd_result['distances']
    point_indices = np.arange(len(distances))
    
    # 绘制折线图
    ax2.plot(point_indices, distances, color='steelblue', linewidth=1.5, alpha=0.8, label='点对点误差')
    
    # 添加统计参考线
    ax2.axhline(apd_result['mean'], color='red', linestyle='--', 
                linewidth=2, label=f"平均: {apd_result['mean']:.3f} mm", alpha=0.8)
    ax2.axhline(apd_result['p95'], color='orange', linestyle='--', 
                linewidth=2, label=f"P95: {apd_result['p95']:.3f} mm", alpha=0.8)
    ax2.axhline(apd_result['max'], color='darkred', linestyle=':', 
                linewidth=2, label=f"最大: {apd_result['max']:.3f} mm", alpha=0.8)
    
    # 标注最大误差点
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
    ax2.set_title('误差沿轨迹变化趋势', fontsize=11, fontweight='bold', pad=10)
    ax2.legend(fontsize=8.5, loc='upper right')
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.set_xlim(0, len(distances))
    ax2.set_ylim(0, max(distances.max() * 1.15, apd_result['mean'] * 2))
    
    # ===== 图3: 垂直距离分布直方图（中下）=====
    ax3 = fig.add_subplot(gs[1, 1])
    
    n, bins, patches = ax3.hist(distances, bins=50, color='steelblue', 
                                edgecolor='black', alpha=0.7)
    
    # 根据bin值给直方图上色
    cm = plt.cm.RdYlGn_r
    bin_centers = 0.5 * (bins[:-1] + bins[1:])
    col = (bin_centers - bin_centers.min()) / (bin_centers.max() - bin_centers.min())
    for c, p in zip(col, patches):
        plt.setp(p, 'facecolor', cm(c))
    
    # 标注统计值
    ax3.axvline(apd_result['mean'], color='red', linestyle='--', 
                linewidth=2.5, label=f"平均: {apd_result['mean']:.3f} mm")
    ax3.axvline(apd_result['p95'], color='orange', linestyle='--', 
                linewidth=2.5, label=f"P95: {apd_result['p95']:.3f} mm")
    ax3.axvline(apd_result['max'], color='darkred', linestyle=':', 
                linewidth=2, label=f"最大: {apd_result['max']:.3f} mm")
    
    ax3.set_xlabel('垂直距离 (mm)', fontsize=10, fontweight='bold')
    ax3.set_ylabel('频数', fontsize=10, fontweight='bold')
    ax3.set_title('垂直距离分布', fontsize=11, fontweight='bold', pad=10)
    ax3.legend(fontsize=8.5, loc='upper right')
    ax3.grid(True, alpha=0.3, axis='y')
    
    # ===== 图4: 统计信息面板（右侧，占据两行）=====
    ax4 = fig.add_subplot(gs[:, 2])
    ax4.axis('off')
    
    info_text = f"""
    ================================================================
                机械臂TCP轨迹回放匹配误差分析报告
    ================================================================
    
    【1. 平均垂直距离 (轨迹复现精度)】
       平均误差:  {apd_result['mean']:>10.4f} mm
       标准差:    {apd_result['std']:>10.4f} mm
       最大误差:  {apd_result['max']:>10.4f} mm
       中位数:    {apd_result['median']:>10.4f} mm
       P95:       {apd_result['p95']:>10.4f} mm
    
    【2. Fréchet距离 (最大轨迹偏差)】
       Fréchet距离: {frechet_result['frechet_distance']:>8.4f} mm
       最大偏差位置: 回放点 #{frechet_result['worst_replay_idx']:>5}
                      <-> 示教点 #{frechet_result['worst_teach_idx']:>5}
    
    【3. 端点定位误差】
       起点误差:  {endpoint_result['start_error']:>10.4f} mm
       终点误差:  {endpoint_result['end_error']:>10.4f} mm
    
    ----------------------------------------------------------------
                    ⭐ 综合评级: {overall_grade} ⭐
    ----------------------------------------------------------------
    
    验收标准参考 (ISO 9283):
      优秀: APD<0.5mm, Fréchet<1.5mm, 端点<1.0mm
      良好: APD<2.0mm, Fréchet<5.0mm, 端点<2.0mm
      可接受: APD<5.0mm, Fréchet<10.0mm, 端点<5.0mm
    
    ================================================================
    """
    
    ax4.text(0.05, 0.5, info_text, transform=ax4.transAxes,
             fontsize=9, verticalalignment='center',
             fontfamily='sans-serif',  # 改用支持中文的字体
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))
    
    plt.suptitle('机械臂TCP轨迹回放匹配误差综合分析', fontsize=16, fontweight='bold', y=0.98)
    
    if save_path:
        plt.savefig(save_path, dpi=CONFIG['figure_save_dpi'], bbox_inches='tight')
        print(f"\n[INFO] 可视化图表已保存: {save_path}")
    
    plt.show()


# ============================================================================
#                          主函数接口
# ============================================================================

def compute_trajectory_matching_error(replay_csv, teach_csv, 
                                      visualize=True, 
                                      save_report=False):
    """
    机械臂TCP轨迹回放匹配误差计算（主函数）
    
    Args:
        replay_csv (str): 回放轨迹CSV文件路径
        teach_csv (str): 示教轨迹CSV文件路径
        visualize (bool): 是否生成可视化图表
        save_report (bool): 是否保存文本报告
    
    Returns:
        dict: 完整评估结果
        {
            'apd': {...},           # 平均垂直距离结果
            'frechet': {...},       # Fréchet距离结果
            'endpoints': {...},     # 端点误差结果
            'overall_grade': str,   # 综合评级
            'pass': bool            # 是否通过验收
        }
    """
    print("\n" + "="*70)
    print("         机械臂TCP轨迹回放匹配误差分析")
    print("="*70)
    
    # 1. 加载数据
    print("\n【步骤1/5】加载轨迹数据...")
    print("-" * 70)
    replay_traj = load_tcp_trajectory(replay_csv)
    teach_traj = load_tcp_trajectory(teach_csv)
    
    # 2. 计算三大指标
    print("\n【步骤2/5】计算核心指标...")
    print("-" * 70)
    
    apd_result = compute_average_perpendicular_distance(replay_traj, teach_traj)
    frechet_result = compute_discrete_frechet_distance(replay_traj, teach_traj)
    endpoint_result = compute_endpoint_errors(replay_traj, teach_traj)
    
    # 3. 综合评级
    print("\n【步骤3/5】综合评级...")
    print("-" * 70)
    overall_grade, is_pass = determine_overall_grade(apd_result, frechet_result, endpoint_result)
    print(f"  综合评级: {overall_grade}")
    print(f"  验收结果: {'通过 ✓' if is_pass else '不通过 ✗'}")
    
    # 4. 构建结果字典
    result = {
        'apd': apd_result,
        'frechet': frechet_result,
        'endpoints': endpoint_result,
        'overall_grade': overall_grade,
        'pass': is_pass
    }
    
    # 5. 可视化
    if visualize:
        print("\n【步骤4/5】生成可视化图表...")
        print("-" * 70)
        plot_trajectory_matching_analysis(
            replay_traj, teach_traj,
            apd_result, frechet_result, endpoint_result,
            overall_grade,
            save_path='trajectory_matching_analysis.png' if save_report else None
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
        
        apd = result['apd']
        frechet = result['frechet']
        endpoints = result['endpoints']
        
        f.write("【1. 平均垂直距离 (轨迹复现精度)】\n")
        f.write(f"   平均误差:  {apd['mean']:.4f} mm\n")
        f.write(f"   标准差:    {apd['std']:.4f} mm\n")
        f.write(f"   最大误差:  {apd['max']:.4f} mm\n")
        f.write(f"   中位数:    {apd['median']:.4f} mm\n")
        f.write(f"   P95:       {apd['p95']:.4f} mm\n\n")
        
        f.write("【2. Fréchet距离 (最大轨迹偏差)】\n")
        f.write(f"   Fréchet距离: {frechet['frechet_distance']:.4f} mm\n")
        f.write(f"   最大偏差位置: 回放点#{frechet['worst_replay_idx']} "
                f"<-> 示教点#{frechet['worst_teach_idx']}\n\n")
        
        f.write("【3. 端点定位误差】\n")
        f.write(f"   起点误差:  {endpoints['start_error']:.4f} mm\n")
        f.write(f"   终点误差:  {endpoints['end_error']:.4f} mm\n\n")
        
        f.write("-"*70 + "\n")
        f.write(f"【综合评级】: {result['overall_grade']}\n")
        f.write(f"【验收结果】: {'通过 ✓' if result['pass'] else '不通过 ✗'}\n")
        f.write("="*70 + "\n")
    
    print(f"  报告已保存: {report_path}")


# ============================================================================
#                            主程序
# ============================================================================

if __name__ == "__main__":
    # ========== 配置文件路径 ==========
    
    # 示教轨迹（参考轨迹）
    teach_csv = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\tracker6 - tcp_transformed.csv"
    # 回放轨迹（待评估轨迹）
    replay_csv = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\tcp6 - tcp.csv"  # 修改为实际回放文件
    
    # ========== 执行分析 ==========
    
    result = compute_trajectory_matching_error(
        replay_csv=replay_csv,
        teach_csv=teach_csv,
        visualize=True,      # 生成可视化图表
        save_report=True     # 保存文本报告
    )
    
    # ========== 打印摘要 ==========
    
    print("\n" + "="*70)
    print("                      评估结果摘要")
    print("="*70)
    
    print(f"\n【平均垂直距离】: {result['apd']['mean']:.4f} mm")
    print(f"【Fréchet距离】:   {result['frechet']['frechet_distance']:.4f} mm")
    print(f"【起点误差】:      {result['endpoints']['start_error']:.4f} mm")
    print(f"【终点误差】:      {result['endpoints']['end_error']:.4f} mm")
    print(f"\n【综合评级】:      {result['overall_grade']}")
    print(f"【验收状态】:      {'✓ 通过' if result['pass'] else '✗ 不通过'}")
    print("\n" + "="*70)

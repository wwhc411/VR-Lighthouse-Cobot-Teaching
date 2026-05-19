"""
CSV点云配准演示脚本
==================

从两个CSV文件加载点云数据（第4、5、6列），进行配准并可视化

使用方法:
    python csv_registration_demo.py <source.csv> <target.csv>
    
    或在代码中直接修改文件路径
"""

import numpy as np
import open3d as o3d
from scipy import interpolate
import copy
import time
import sys
import os

# ============================================================================
#                        📝 可调参数配置区域
# ============================================================================

# ===== B-Spline 平滑参数 =====
BSPLINE_SMOOTHING = 10.0      # 平滑因子（越大越平滑，建议1-50）
BSPLINE_NUM_SAMPLES = 3000    # 重采样点数（不要太大，防止过密）

# ===== 体素/特征参数 =====
VOXEL_SIZE_MIN = 1.0          # 最小体素大小(mm)
VOXEL_SIZE_MAX = 3.0          # 最大体素大小(mm)
VOXEL_SIZE_DIVISOR = 300      # 对角线除数（越大voxel_size越小）

# ===== RANSAC 参数 =====
RANSAC_DISTANCE_MULTIPLIER = 3.0   # 距离阈值 = voxel_size × 此值（增大以允许更多对应）
RANSAC_MAX_ITERATION = 4000000     # 最大迭代次数
RANSAC_CONFIDENCE = 0.9999         # 置信度
RANSAC_N = 3                       # 采样点数(3点更鲁棒)
RANSAC_EDGE_LENGTH = 0.85          # 边长比阈值(放宽以增加容错)
RANSAC_MUTUAL_FILTER = True        # 互惠过滤

# ===== GICP 参数 =====
GICP_RADIUS_MULTIPLIERS = [100, 80, 60, 40, 30, 20, 15, 10, 7, 5, 3, 2, 1]  # 更多轮次+更大初始范围
GICP_MAX_ITERATION_PER_ROUND = 150               # 每轮最大迭代次数
GICP_FITNESS_THRESHOLD = 1e-12                   # 收敛阈值(更严格)

# ===== 是否启用质心对齐预处理 =====
ENABLE_CENTER_ALIGN = True

# ===== 配准模式 =====
# 'ransac_gicp' - 完整流程：RANSAC全局配准 + GICP精配准
# 'gicp_only'   - 仅GICP：直接使用质心对齐后的恒等变换作为GICP初始解（适用于已粗对齐的数据）
REGISTRATION_MODE = 'gicp_only'

# ===== 变换合理性检查 =====
MAX_ROTATION_ANGLE = 45.0   # 最大允许旋转角度（度），超过则认为RANSAC失败


# ============================================================================
#                              数据加载
# ============================================================================

def load_csv_trajectory(filepath, xyz_cols=(3, 4, 5), time_col=2, skip_header=1):
    """
    从CSV文件加载轨迹数据
    
    参数:
        filepath: CSV文件路径
        xyz_cols: XYZ坐标的列索引（默认第4,5,6列，即索引3,4,5）
        time_col: 时间戳列索引（默认第3列，即TimeFromStart_s）
        skip_header: 跳过的表头行数
    
    返回:
        points: (N, 3) 点云
        timestamps: (N,) 时间戳
    """
    print(f"  加载文件: {os.path.basename(filepath)}")
    data = np.loadtxt(filepath, delimiter=',', skiprows=skip_header)
    
    points = data[:, xyz_cols]
    timestamps = data[:, time_col] if time_col is not None else None
    
    print(f"  读取到 {len(points)} 个点")
    return points, timestamps


# ============================================================================
#                          B-Spline 轨迹平滑
# ============================================================================

def fit_bspline_trajectory(points, timestamps=None, smoothing=None, num_samples=None):
    """
    使用三次B-Spline对轨迹进行平滑拟合
    """
    # 使用全局配置或传入参数
    if smoothing is None:
        smoothing = BSPLINE_SMOOTHING
    if num_samples is None:
        num_samples = BSPLINE_NUM_SAMPLES
    
    N = len(points)
    if N < 4:
        return points
    
    if timestamps is None:
        t_normalized = np.linspace(0, 1, N)
    else:
        t_normalized = (timestamps - timestamps[0]) / (timestamps[-1] - timestamps[0] + 1e-10)
    
    try:
        tck, u = interpolate.splprep(
            points.T,
            u=t_normalized,
            s=smoothing,
            k=3
        )
        u_new = np.linspace(0, 1, num_samples)
        new_points = interpolate.splev(u_new, tck)
        return np.array(new_points).T
    except Exception as e:
        print(f"  B-Spline拟合失败: {e}")
        indices = np.linspace(0, N-1, num_samples, dtype=int)
        return points[indices]


# ============================================================================
#                     FPFH + RANSAC (High-Conf) 全局配准
# ============================================================================

def compute_fpfh_features(pcd, voxel_size):
    """计算FPFH特征"""
    pcd_down = pcd.voxel_down_sample(voxel_size)
    
    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )
    
    radius_feature = voxel_size * 5
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
    )
    
    return pcd_down, fpfh


def execute_ransac_global(source_pcd, target_pcd, voxel_size):
    """FPFH + RANSAC (High-Conf) 全局配准"""
    print("  [RANSAC] 计算FPFH特征...")
    source_down, source_fpfh = compute_fpfh_features(source_pcd, voxel_size)
    target_down, target_fpfh = compute_fpfh_features(target_pcd, voxel_size)
    
    print(f"  [RANSAC] 源点云: {len(source_pcd.points)} -> {len(source_down.points)}")
    print(f"  [RANSAC] 目标点云: {len(target_pcd.points)} -> {len(target_down.points)}")
    
    # 使用配置参数
    distance_threshold = voxel_size * RANSAC_DISTANCE_MULTIPLIER
    print(f"  [RANSAC] 距离阈值: {distance_threshold:.2f}")
    
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down,
        source_fpfh, target_fpfh,
        mutual_filter=RANSAC_MUTUAL_FILTER,
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=RANSAC_N,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(RANSAC_EDGE_LENGTH),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
            max_iteration=RANSAC_MAX_ITERATION,
            confidence=RANSAC_CONFIDENCE
        )
    )
    
    print(f"  [RANSAC] ✓ Fitness: {result.fitness:.4f}, RMSE: {result.inlier_rmse:.6f}")
    
    # 如果RANSAC结果不理想，尝试FGR
    if result.fitness < 0.5:
        print("  [RANSAC] 尝试FGR...")
        result_fgr = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            source_down, target_down,
            source_fpfh, target_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=distance_threshold
            )
        )
        if result_fgr.fitness > result.fitness:
            print(f"  [FGR] ✓ 更优: Fitness: {result_fgr.fitness:.4f}")
            return result_fgr.transformation
    
    return result.transformation


# ============================================================================
#                              GICP 精配准
# ============================================================================

def execute_gicp(source_pcd, target_pcd, T_init, voxel_size):
    """GICP精配准 - 多轮迭代逐步收紧搜索半径"""
    source = copy.deepcopy(source_pcd)
    target = copy.deepcopy(target_pcd)
    
    radius_normal = voxel_size * 3
    source.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )
    target.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )
    
    # 多轮ICP，使用配置的半径倍数
    current_T = T_init
    search_radii = [voxel_size * m for m in GICP_RADIUS_MULTIPLIERS]
    
    for i, radius in enumerate(search_radii):
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=GICP_FITNESS_THRESHOLD,
            relative_rmse=GICP_FITNESS_THRESHOLD,
            max_iteration=GICP_MAX_ITERATION_PER_ROUND
        )
        
        result = o3d.pipelines.registration.registration_generalized_icp(
            source, target,
            radius,
            current_T,
            o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
            criteria
        )
        
        current_T = result.transformation
        print(f"    轮次 {i+1}/{len(search_radii)}: 搜索半径={radius:.2f}, Fitness={result.fitness:.4f}, RMSE={result.inlier_rmse:.4f}")
    
    return result


# ============================================================================
#                              误差计算
# ============================================================================

def compute_trajectory_error(source_points, target_points, transformation):
    """
    计算配准后两条轨迹的空间误差
    
    参数:
        source_points: (N, 3) 源轨迹点
        target_points: (M, 3) 目标轨迹点
        transformation: (4, 4) 变换矩阵
    
    返回:
        dict: 包含各种误差指标
    """
    # 将源点云变换到目标坐标系
    N = len(source_points)
    source_homo = np.hstack([source_points, np.ones((N, 1))])  # [N, 4]
    transformed = (transformation @ source_homo.T).T[:, :3]    # [N, 3]
    
    # 对于每个变换后的点，找目标点云中的最近点
    target_pcd = o3d.geometry.PointCloud()
    target_pcd.points = o3d.utility.Vector3dVector(target_points)
    tree = o3d.geometry.KDTreeFlann(target_pcd)
    
    distances = []
    for pt in transformed:
        [_, idx, dist] = tree.search_knn_vector_3d(pt, 1)
        distances.append(np.sqrt(dist[0]))
    
    distances = np.array(distances)
    
    # 计算各种误差指标
    error_dict = {
        'mean_error': np.mean(distances),
        'max_error': np.max(distances),
        'min_error': np.min(distances),
        'std_error': np.std(distances),
        'rmse': np.sqrt(np.mean(distances**2)),
        'median_error': np.median(distances),
        'percentile_95': np.percentile(distances, 95),
        'percentile_99': np.percentile(distances, 99),
    }
    
    return error_dict, distances


# ============================================================================
#                              可视化
# ============================================================================

def visualize_registration(source_pcd, target_pcd, transformation, title="配准结果"):
    """可视化配准后的两个点云"""
    source_temp = copy.deepcopy(source_pcd)
    target_temp = copy.deepcopy(target_pcd)
    
    # 着色：源点云橙色，目标点云蓝色
    source_temp.paint_uniform_color([1, 0.706, 0])      # 橙色
    target_temp.paint_uniform_color([0, 0.651, 0.929])  # 蓝色
    
    # 应用变换
    source_temp.transform(transformation)
    
    # 创建坐标系
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.05, origin=[0, 0, 0]
    )
    
    print("\n可视化说明:")
    print("  橙色：源点云（已变换）")
    print("  蓝色：目标点云")
    print("  按 Q 关闭窗口")
    
    o3d.visualization.draw_geometries(
        [source_temp, target_temp, coord_frame],
        window_name=title,
        width=1280,
        height=720
    )


# ============================================================================
#                              主流程
# ============================================================================

def registration_from_csv(source_csv, target_csv, visualize=True):
    """
    从两个CSV文件加载点云并进行配准
    
    参数:
        source_csv: 源轨迹CSV文件路径
        target_csv: 目标轨迹CSV文件路径
        visualize: 是否显示可视化
    
    返回:
        result: 配准结果字典
    """
    print("=" * 60)
    print("B-Spline + RANSAC (High-Conf) + GICP 点云配准")
    print("=" * 60)
    
    total_start = time.time()
    
    # ==========================================
    # 加载数据
    # ==========================================
    print("\n▶ 加载CSV数据")
    print("-" * 40)
    
    source_points, source_ts = load_csv_trajectory(source_csv)
    target_points, target_ts = load_csv_trajectory(target_csv)
    
    # 质心对齐预处理
    if ENABLE_CENTER_ALIGN:
        source_center = np.mean(source_points, axis=0)
        target_center = np.mean(target_points, axis=0)
        initial_offset = target_center - source_center
        
        print(f"  源点云质心: [{source_center[0]:.2f}, {source_center[1]:.2f}, {source_center[2]:.2f}]")
        print(f"  目标点云质心: [{target_center[0]:.2f}, {target_center[1]:.2f}, {target_center[2]:.2f}]")
        print(f"  初始质心偏移: [{initial_offset[0]:.2f}, {initial_offset[1]:.2f}, {initial_offset[2]:.2f}]")
        
        source_points_aligned = source_points + initial_offset
    else:
        source_points_aligned = source_points
        initial_offset = np.zeros(3)
    
    # 自动估计voxel_size - 使用配置参数
    all_points = np.vstack([source_points_aligned, target_points])
    min_bound = np.min(all_points, axis=0)
    max_bound = np.max(all_points, axis=0)
    diagonal = np.linalg.norm(max_bound - min_bound)
    
    voxel_size = diagonal / VOXEL_SIZE_DIVISOR
    voxel_size = max(voxel_size, VOXEL_SIZE_MIN)
    voxel_size = min(voxel_size, VOXEL_SIZE_MAX)
    print(f"  自动voxel_size: {voxel_size:.4f}mm")
    
    # ==========================================
    # 阶段 1: B-Spline 平滑
    # ==========================================
    print("\n▶ 阶段 1: B-Spline 轨迹平滑")
    print("-" * 40)
    print(f"  平滑因子: {BSPLINE_SMOOTHING}")
    print(f"  重采样点数: {BSPLINE_NUM_SAMPLES}")
    
    stage1_start = time.time()
    
    # 使用质心对齐后的源点云
    smooth_source = fit_bspline_trajectory(source_points_aligned, source_ts)
    smooth_target = fit_bspline_trajectory(target_points, target_ts)
    
    stage1_time = time.time() - stage1_start
    print(f"  ✓ 平滑完成，耗时: {stage1_time:.2f}s")
    
    # 构建Open3D点云
    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(smooth_source)
    
    target_pcd = o3d.geometry.PointCloud()
    target_pcd.points = o3d.utility.Vector3dVector(smooth_target)
    
    # ==========================================
    # 阶段 2: RANSAC 全局配准
    # ==========================================
    print("\n▶ 阶段 2: FPFH + RANSAC (High-Conf) 全局配准")
    print("-" * 40)
    
    stage2_start = time.time()
    
    if REGISTRATION_MODE == 'gicp_only':
        print(f"  [模式] gicp_only - 跳过RANSAC，使用恒等变换")
        T_init = np.eye(4)
    else:
        T_ransac = execute_ransac_global(source_pcd, target_pcd, voxel_size)
        
        # 检查变换合理性
        rotation_matrix = T_ransac[:3, :3]
        rotation_angle = np.degrees(np.arccos((np.trace(rotation_matrix) - 1) / 2))
        
        print(f"  [检查] RANSAC旋转角度: {rotation_angle:.2f}°")
        
        if rotation_angle > MAX_ROTATION_ANGLE:
            print(f"  [警告] 旋转角度超过阈值{MAX_ROTATION_ANGLE}°，可能是错误解！")
            print(f"  [回退] 使用恒等变换作为GICP初始解")
            T_init = np.eye(4)
        else:
            T_init = T_ransac
    
    stage2_time = time.time() - stage2_start
    print(f"  ✓ 阶段2完成，耗时: {stage2_time:.2f}s")
    
    # ==========================================
    # 阶段 3: GICP 精配准
    # ==========================================
    print("\n▶ 阶段 3: GICP 精配准")
    print("-" * 40)
    
    stage3_start = time.time()
    result_gicp = execute_gicp(source_pcd, target_pcd, T_init, voxel_size)
    T_final = result_gicp.transformation
    stage3_time = time.time() - stage3_start
    print(f"  ✓ GICP完成，耗时: {stage3_time:.2f}s")
    print(f"  Fitness: {result_gicp.fitness:.6f}, RMSE: {result_gicp.inlier_rmse:.6f}")
    
    # ==========================================
    # 计算空间误差（使用原始数据验证）
    # ==========================================
    print("\n▶ 计算轨迹空间误差")
    print("-" * 40)
    
    # 先计算平滑后轨迹的误差
    error_dict, distances = compute_trajectory_error(smooth_source, smooth_target, T_final)
    
    # 同时计算原始轨迹的误差（使用质心偏移后的原始数据）
    error_dict_raw, _ = compute_trajectory_error(source_points_aligned, target_points, T_final)
    
    total_time = time.time() - total_start
    
    # 构建完整变换矩阵（包含质心预对齐）
    T_center_align = np.eye(4)
    T_center_align[:3, 3] = initial_offset
    T_complete = T_final @ T_center_align  # 完整变换 = GICP变换 × 质心对齐
    
    # ==========================================
    # 结果汇总
    # ==========================================
    print("\n" + "=" * 60)
    print(">>> 配准结果汇总 <<<")
    print("=" * 60)
    
    print(f"\n【平滑轨迹误差】(B-Spline平滑后，单位mm)")
    print(f"  平均误差 (Mean):     {error_dict['mean_error']:.3f}")
    print(f"  均方根误差 (RMSE):   {error_dict['rmse']:.3f}")
    print(f"  最大误差 (Max):      {error_dict['max_error']:.3f}")
    print(f"  中位数 (Median):     {error_dict['median_error']:.3f}")
    
    print(f"\n【原始轨迹误差】(未平滑，单位mm)")
    print(f"  平均误差 (Mean):     {error_dict_raw['mean_error']:.3f}")
    print(f"  均方根误差 (RMSE):   {error_dict_raw['rmse']:.3f}")
    print(f"  最大误差 (Max):      {error_dict_raw['max_error']:.3f}")
    print(f"  中位数 (Median):     {error_dict_raw['median_error']:.3f}")
    
    print(f"\n【耗时统计】")
    print(f"  阶段1 (B-Spline):     {stage1_time:.2f}s")
    print(f"  阶段2 (RANSAC):       {stage2_time:.2f}s")
    print(f"  阶段3 (GICP):         {stage3_time:.2f}s")
    print(f"  总计:                 {total_time:.2f}s")
    
    print(f"\n【GICP变换矩阵】(基于质心对齐后的数据)")
    print(T_final)
    
    # 提取旋转和平移
    R_final = T_final[:3, :3]
    t_final = T_final[:3, 3]
    angle = np.arccos(np.clip((np.trace(R_final) - 1) / 2, -1, 1))
    angle_deg = np.degrees(angle)
    
    print(f"\n【变换分解】")
    print(f"  旋转角度:   {angle_deg:.4f}°")
    print(f"  平移向量:   [{t_final[0]:.6f}, {t_final[1]:.6f}, {t_final[2]:.6f}]")
    print(f"  平移距离:   {np.linalg.norm(t_final):.6f}")
    
    # 可视化
    if visualize:
        print("\n正在显示配准结果...")
        visualize_registration(source_pcd, target_pcd, T_final, "CSV点云配准结果")
    
    return {
        'T_final': T_final,
        'fitness': result_gicp.fitness,
        'rmse': result_gicp.inlier_rmse,
        'error': error_dict,
        'distances': distances,
        'source_smooth': smooth_source,
        'target_smooth': smooth_target,
    }


# ============================================================================
#                              主入口
# ============================================================================

if __name__ == "__main__":
    # ========================================================================
    # 📝 在此处修改你的两个CSV文件路径
    # ========================================================================
    
    # 源轨迹CSV文件（橙色显示）
    SOURCE_CSV = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\PlaybackRecord_HighFreq_7_20260202_145911_7_20260202_150007.csv"
    
    # 目标轨迹CSV文件（蓝色显示）
    TARGET_CSV = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\PlaybackRecord_HighFreq_7_20260202_145911_7_20260202_150124.csv"
    
    # ========================================================================
    
    # 检查命令行参数（优先级更高）
    if len(sys.argv) >= 3:
        source_csv = sys.argv[1]
        target_csv = sys.argv[2]
    else:
        source_csv = SOURCE_CSV
        target_csv = TARGET_CSV
    
    # 检查文件是否存在
    if not os.path.exists(source_csv):
        print(f"错误：源文件不存在: {source_csv}")
        sys.exit(1)
    if not os.path.exists(target_csv):
        print(f"错误：目标文件不存在: {target_csv}")
        sys.exit(1)
    
    # 执行配准
    result = registration_from_csv(source_csv, target_csv, visualize=True)

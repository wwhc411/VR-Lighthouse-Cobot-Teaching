"""
B-Spline + RANSAC (High-Conf) + GICP 高精度点云配准方案
====================================================

功能：对高频抖动传感器录制的两段相似轨迹点云进行高精度刚性配准

流程：
    阶段1：B-Spline 轨迹平滑（去除高频抖动噪声）
    阶段2：FPFH特征 + RANSAC全局粗配准（Open3D高置信度参数）
    阶段3：GICP 精配准（亚厘米级收敛）

依赖库：
    pip install numpy scipy open3d

作者：自动生成
日期：2026-02-02
"""

import numpy as np
import open3d as o3d
from scipy import interpolate
import copy
import time

print("✓ 使用 FPFH + RANSAC (High-Conf) 全局配准方案")


# ============================================================================
#                          阶段 1: B-Spline 轨迹平滑
# ============================================================================

def fit_bspline_trajectory(points, timestamps=None, smoothing=5.0, num_samples=5000):
    """
    使用三次B-Spline对轨迹进行平滑拟合
    
    参数:
        points: (N, 3) numpy array - 原始带噪点云
        timestamps: (N,) numpy array - 时间戳（可选，如果为None则自动生成）
        smoothing: float - 平滑因子，值越大越平滑
                          建议：s ≈ N × σ²，其中N是点数，σ是噪声标准差
                          高频抖动：1.0 ~ 5.0
        num_samples: int - 重采样点数（输出点云密度）
    
    返回:
        (num_samples, 3) numpy array - 平滑后的轨迹点云
    """
    N = len(points)
    
    # 如果没有时间戳，自动生成均匀时间参数
    if timestamps is None:
        t_normalized = np.linspace(0, 1, N)
    else:
        # 归一化时间戳到 [0, 1]
        t_normalized = (timestamps - timestamps[0]) / (timestamps[-1] - timestamps[0])
    
    # 确保时间单调递增
    if not np.all(np.diff(t_normalized) > 0):
        print("警告：时间戳非单调递增，尝试修复...")
        # 对于重复时间戳，添加微小偏移
        for i in range(1, len(t_normalized)):
            if t_normalized[i] <= t_normalized[i-1]:
                t_normalized[i] = t_normalized[i-1] + 1e-10
    
    try:
        # B样条拟合：k=3表示三次样条（C²连续）
        tck, u = interpolate.splprep(
            points.T,           # 转置为 [3, N]
            u=t_normalized,     # 参数化变量
            s=smoothing,        # 平滑因子
            k=3                 # 三次B样条
        )
        
        # 重采样生成平滑轨迹
        u_new = np.linspace(0, 1, num_samples)
        new_points = interpolate.splev(u_new, tck)
        
        return np.array(new_points).T  # 转置回 [N, 3]
        
    except Exception as e:
        print(f"B-Spline拟合失败: {e}")
        print("回退：使用简单下采样...")
        indices = np.linspace(0, N-1, num_samples, dtype=int)
        return points[indices]


# ============================================================================
#                  阶段 2: FPFH + RANSAC (High-Conf) 全局配准
# ============================================================================

def compute_fpfh_features(pcd, voxel_size):
    """
    计算点云的FPFH特征
    
    参数:
        pcd: Open3D点云对象
        voxel_size: float - 体素大小，影响特征计算半径
    
    返回:
        pcd_down: 下采样后的点云
        fpfh: FPFH特征
    """
    # 下采样
    pcd_down = pcd.voxel_down_sample(voxel_size)
    
    # 法线估计
    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )
    
    # FPFH特征计算
    radius_feature = voxel_size * 5
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
    )
    
    return pcd_down, fpfh


def execute_ransac_global(source_pcd, target_pcd, voxel_size):
    """
    使用 FPFH特征 + RANSAC (High-Conf) 进行鲁棒全局配准
    
    采用Open3D官方推荐的高置信度参数配置:
    - 严格的边长比检验 (0.9)
    - 严格的距离检验
    - 高迭代次数确保收敛
    
    参数:
        source_pcd: 源点云 (Open3D格式)
        target_pcd: 目标点云 (Open3D格式)
        voxel_size: float - 体素大小
    
    返回:
        T_ransac: (4, 4) 变换矩阵
    """
    print("  [RANSAC] 计算FPFH特征...")
    source_down, source_fpfh = compute_fpfh_features(source_pcd, voxel_size)
    target_down, target_fpfh = compute_fpfh_features(target_pcd, voxel_size)
    
    print(f"  [RANSAC] 源点云下采样: {len(source_pcd.points)} -> {len(source_down.points)}")
    print(f"  [RANSAC] 目标点云下采样: {len(target_pcd.points)} -> {len(target_down.points)}")
    
    # ===== High-Confidence RANSAC 参数 =====
    # 距离阈值：体素大小的1.5倍（适中，不太宽松）
    distance_threshold = voxel_size * 1.5
    
    print(f"  [RANSAC] 距离阈值: {distance_threshold:.4f}")
    print("  [RANSAC] 执行 High-Confidence 全局配准...")
    
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down,
        source_fpfh, target_fpfh,
        mutual_filter=True,   # 开启互惠过滤，提高匹配质量
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,  # 使用4点采样，比3点更鲁棒
        checkers=[
            # 边长比检验：拒绝边长比例不一致的匹配对
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            # 距离检验：拒绝距离过大的匹配对
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
            max_iteration=4000000,      # 高迭代次数
            confidence=0.9999           # 99.99% 置信度
        )
    )
    
    print(f"  [RANSAC] ✓ 完成，Fitness: {result.fitness:.4f}, RMSE: {result.inlier_rmse:.6f}")
    
    # 如果RANSAC结果不理想，尝试FGR作为补充
    if result.fitness < 0.3:
        print("  [RANSAC] Fitness较低，尝试FGR补充...")
        result_fgr = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            source_down, target_down,
            source_fpfh, target_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=distance_threshold
            )
        )
        if result_fgr.fitness > result.fitness:
            print(f"  [FGR] ✓ FGR更优，Fitness: {result_fgr.fitness:.4f}")
            return result_fgr.transformation
    
    return result.transformation


# ============================================================================
#                          阶段 3: GICP 精配准
# ============================================================================

def execute_gicp(source_pcd, target_pcd, T_init, voxel_size):
    """
    使用GICP进行精细配准
    
    参数:
        source_pcd: 源点云 (Open3D格式)
        target_pcd: 目标点云 (Open3D格式)
        T_init: (4, 4) 初始变换矩阵（来自RANSAC全局配准）
        voxel_size: float - 体素大小，用于设置搜索半径
    
    返回:
        result: Open3D配准结果对象
    """
    # 复制点云避免修改原始数据
    source = copy.deepcopy(source_pcd)
    target = copy.deepcopy(target_pcd)
    
    # 估计法线（GICP核心依赖）
    radius_normal = voxel_size * 2
    source.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )
    target.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )
    
    # 配置收敛标准
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=1e-8,   # 重合度变化阈值
        relative_rmse=1e-8,      # RMSE变化阈值
        max_iteration=100        # 最大迭代次数
    )
    
    # GICP配准
    # 由于RANSAC已经比较准，搜索半径可以设小
    max_correspondence_distance = voxel_size * 0.5
    
    print(f"  [GICP] 搜索半径: {max_correspondence_distance:.4f}")
    print(f"  [GICP] 开始迭代配准...")
    
    result = o3d.pipelines.registration.registration_generalized_icp(
        source, target,
        max_correspondence_distance,
        T_init,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        criteria
    )
    
    return result


# ============================================================================
#                              可视化工具
# ============================================================================

def visualize_registration(source, target, transformation=None, window_name="配准结果"):
    """
    可视化配准结果
    
    参数:
        source: 源点云
        target: 目标点云
        transformation: 变换矩阵（如果提供，将应用到源点云）
        window_name: 窗口标题
    """
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    
    # 着色
    source_temp.paint_uniform_color([1, 0.706, 0])      # 橙色：源点云
    target_temp.paint_uniform_color([0, 0.651, 0.929])  # 蓝色：目标点云
    
    # 应用变换
    if transformation is not None:
        source_temp.transform(transformation)
    
    # 可视化
    o3d.visualization.draw_geometries(
        [source_temp, target_temp],
        window_name=window_name,
        width=1280,
        height=720
    )


def visualize_comparison(source, target, T_init, T_final):
    """
    并排对比粗配准和精配准结果
    
    参数:
        source: 源点云
        target: 目标点云
        T_init: RANSAC初始变换
        T_final: GICP最终变换
    """
    # 创建三个版本的源点云
    source_original = copy.deepcopy(source)
    source_ransac = copy.deepcopy(source)
    source_gicp = copy.deepcopy(source)
    target_copy = copy.deepcopy(target)
    
    # 着色
    source_original.paint_uniform_color([1, 0, 0])      # 红色：原始
    source_ransac.paint_uniform_color([0, 1, 0])        # 绿色：RANSAC
    source_gicp.paint_uniform_color([0, 0, 1])          # 蓝色：GICP
    target_copy.paint_uniform_color([0.7, 0.7, 0.7])    # 灰色：目标
    
    # 应用变换
    source_ransac.transform(T_init)
    source_gicp.transform(T_final)
    
    # 平移显示
    offset = np.max(np.asarray(target_copy.points)[:, 0]) - np.min(np.asarray(target_copy.points)[:, 0])
    source_original.translate([-offset * 1.5, 0, 0])
    source_ransac.translate([0, 0, 0])
    source_gicp.translate([offset * 1.5, 0, 0])
    
    target_1 = copy.deepcopy(target_copy)
    target_2 = copy.deepcopy(target_copy)
    target_3 = copy.deepcopy(target_copy)
    target_1.translate([-offset * 1.5, 0, 0])
    target_3.translate([offset * 1.5, 0, 0])
    
    print("\n可视化说明：")
    print("  左侧（红色）：原始未配准")
    print("  中间（绿色）：RANSAC粗配准")
    print("  右侧（蓝色）：GICP精配准")
    print("  灰色：目标点云")
    
    o3d.visualization.draw_geometries(
        [source_original, source_ransac, source_gicp, target_1, target_2, target_3],
        window_name="配准对比：原始 | RANSAC | GICP",
        width=1920,
        height=720
    )


# ============================================================================
#                              主工作流程
# ============================================================================

def registration_pipeline(
    source_points, 
    target_points,
    source_timestamps=None,
    target_timestamps=None,
    smoothing=5.0,
    num_samples=5000,
    voxel_size=0.02,
    visualize=True
):
    """
    完整的点云配准流程
    
    参数:
        source_points: (N, 3) numpy array - 源轨迹点云
        target_points: (M, 3) numpy array - 目标轨迹点云
        source_timestamps: (N,) numpy array - 源轨迹时间戳（可选）
        target_timestamps: (M,) numpy array - 目标轨迹时间戳（可选）
        smoothing: float - B-Spline平滑因子
        num_samples: int - 重采样点数
        voxel_size: float - 体素大小（单位与点云一致，如米）
        visualize: bool - 是否显示可视化
    
    返回:
        dict: 包含变换矩阵、精度指标等结果
    """
    print("=" * 60)
    print("B-Spline + RANSAC (High-Conf) + GICP 高精度点云配准")
    print("=" * 60)
    
    total_start = time.time()
    
    # ==========================================
    # 阶段 1: B-Spline 轨迹平滑
    # ==========================================
    print("\n▶ 阶段 1: B-Spline 轨迹平滑")
    print("-" * 40)
    
    stage1_start = time.time()
    
    print(f"  源点云原始点数: {len(source_points)}")
    print(f"  目标点云原始点数: {len(target_points)}")
    print(f"  平滑因子 s = {smoothing}")
    print(f"  重采样点数 = {num_samples}")
    
    smooth_source = fit_bspline_trajectory(
        source_points, source_timestamps, smoothing, num_samples
    )
    smooth_target = fit_bspline_trajectory(
        target_points, target_timestamps, smoothing, num_samples
    )
    
    stage1_time = time.time() - stage1_start
    print(f"  ✓ 平滑完成，耗时: {stage1_time:.2f}s")
    
    # 构建Open3D点云对象
    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(smooth_source)
    
    target_pcd = o3d.geometry.PointCloud()
    target_pcd.points = o3d.utility.Vector3dVector(smooth_target)
    
    # ==========================================
    # 阶段 2: RANSAC (High-Conf) 全局配准
    # ==========================================
    print("\n▶ 阶段 2: FPFH + RANSAC (High-Conf) 全局配准")
    print("-" * 40)
    
    stage2_start = time.time()
    
    T_init = execute_ransac_global(source_pcd, target_pcd, voxel_size)
    
    stage2_time = time.time() - stage2_start
    print(f"  ✓ RANSAC完成，耗时: {stage2_time:.2f}s")
    
    # ==========================================
    # 阶段 3: GICP 精配准
    # ==========================================
    print("\n▶ 阶段 3: GICP 精细配准")
    print("-" * 40)
    
    stage3_start = time.time()
    
    result_gicp = execute_gicp(source_pcd, target_pcd, T_init, voxel_size)
    T_final = result_gicp.transformation
    
    stage3_time = time.time() - stage3_start
    print(f"  ✓ GICP完成，耗时: {stage3_time:.2f}s")
    
    # ==========================================
    # 结果汇总
    # ==========================================
    total_time = time.time() - total_start
    
    print("\n" + "=" * 60)
    print(">>> 配准结果汇总 <<<")
    print("=" * 60)
    
    print(f"\n【精度指标】")
    print(f"  Fitness (重合度):     {result_gicp.fitness:.6f}")
    print(f"  Inlier RMSE (误差):   {result_gicp.inlier_rmse:.6f}")
    
    print(f"\n【耗时统计】")
    print(f"  阶段1 (B-Spline):     {stage1_time:.2f}s")
    print(f"  阶段2 (RANSAC):       {stage2_time:.2f}s")
    print(f"  阶段3 (GICP):         {stage3_time:.2f}s")
    print(f"  总计:                 {total_time:.2f}s")
    
    print(f"\n【RANSAC 初始变换矩阵】")
    print(T_init)
    
    print(f"\n【GICP 最终变换矩阵】")
    print(T_final)
    
    # 提取旋转和平移
    R_final = T_final[:3, :3]
    t_final = T_final[:3, 3]
    
    # 计算旋转角度（绕轴-角度表示）
    angle = np.arccos(np.clip((np.trace(R_final) - 1) / 2, -1, 1))
    angle_deg = np.degrees(angle)
    
    print(f"\n【变换分解】")
    print(f"  旋转角度:   {angle_deg:.4f}°")
    print(f"  平移向量:   [{t_final[0]:.6f}, {t_final[1]:.6f}, {t_final[2]:.6f}]")
    print(f"  平移距离:   {np.linalg.norm(t_final):.6f}")
    
    # 可视化
    if visualize:
        print("\n正在显示配准结果...")
        visualize_registration(source_pcd, target_pcd, T_final, "GICP最终配准结果")
    
    # 返回结果字典
    return {
        'T_init': T_init,
        'T_final': T_final,
        'fitness': result_gicp.fitness,
        'rmse': result_gicp.inlier_rmse,
        'rotation': R_final,
        'translation': t_final,
        'rotation_angle_deg': angle_deg,
        'source_smooth': smooth_source,
        'target_smooth': smooth_target,
        'time_total': total_time,
        'time_bspline': stage1_time,
        'time_ransac': stage2_time,
        'time_gicp': stage3_time,
    }


# ============================================================================
#                              数据加载工具
# ============================================================================

def load_trajectory_csv(filepath, xyz_cols=(1, 2, 3), time_col=0, delimiter=',', skip_header=1):
    """
    从CSV文件加载轨迹数据
    
    参数:
        filepath: CSV文件路径
        xyz_cols: XYZ坐标的列索引（默认第1,2,3列）
        time_col: 时间戳列索引（默认第0列），设为None则不读取时间戳
        delimiter: 分隔符
        skip_header: 跳过的表头行数
    
    返回:
        points: (N, 3) 点云
        timestamps: (N,) 时间戳（如果time_col为None则返回None）
    """
    data = np.loadtxt(filepath, delimiter=delimiter, skiprows=skip_header)
    
    points = data[:, xyz_cols]
    timestamps = data[:, time_col] if time_col is not None else None
    
    return points, timestamps


def load_point_cloud_file(filepath):
    """
    从点云文件加载数据（支持PCD、PLY、XYZ等格式）
    
    参数:
        filepath: 点云文件路径
    
    返回:
        points: (N, 3) numpy array
    """
    pcd = o3d.io.read_point_cloud(filepath)
    return np.asarray(pcd.points)


# ============================================================================
#                              演示与测试
# ============================================================================

def generate_test_data(noise_level=0.01, n_points=1000):
    """
    生成带噪声的测试轨迹数据
    
    参数:
        noise_level: 噪声标准差
        n_points: 点数
    
    返回:
        source_points, target_points, timestamps, T_ground_truth
    """
    # 生成螺旋线轨迹
    t = np.linspace(0, 4 * np.pi, n_points)
    
    # 真实轨迹（螺旋线）- 使用更大的尺度
    x = t * np.cos(t)
    y = t * np.sin(t)
    z = 0.5 * t
    gt_trajectory = np.stack([x, y, z], axis=1)
    
    # 源点云：添加噪声
    source_points = gt_trajectory + np.random.normal(0, noise_level, gt_trajectory.shape)
    
    # 定义真实变换
    angle = np.radians(15)  # 旋转15度
    R_gt = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])
    t_gt = np.array([1.0, 0.5, 0.2])  # 更大的平移量
    
    T_gt = np.eye(4)
    T_gt[:3, :3] = R_gt
    T_gt[:3, 3] = t_gt
    
    # 目标点云：变换 + 噪声
    target_points = (R_gt @ gt_trajectory.T).T + t_gt
    target_points += np.random.normal(0, noise_level, target_points.shape)
    
    # 时间戳
    timestamps = np.linspace(0, 10, n_points)
    
    return source_points, target_points, timestamps, T_gt


def estimate_voxel_size(points):
    """
    根据点云自动估计合适的voxel_size
    
    参数:
        points: (N, 3) 点云
    
    返回:
        voxel_size: 推荐的体素大小
    """
    # 计算点云的包围盒对角线长度
    min_bound = np.min(points, axis=0)
    max_bound = np.max(points, axis=0)
    diagonal = np.linalg.norm(max_bound - min_bound)
    
    # voxel_size 约为对角线的 1/100 到 1/50
    voxel_size = diagonal / 80
    
    return voxel_size


def demo():
    """
    演示完整配准流程
    """
    print("\n" + "=" * 60)
    print("生成测试数据...")
    print("=" * 60)
    
    # 生成测试数据
    source_points, target_points, timestamps, T_gt = generate_test_data(
        noise_level=0.05,  # 5cm噪声
        n_points=1000
    )
    
    print(f"真实变换矩阵:\n{T_gt}")
    
    # 自动估计voxel_size
    voxel_size = estimate_voxel_size(source_points)
    print(f"自动估计的voxel_size: {voxel_size:.4f}")
    
    # 执行配准
    result = registration_pipeline(
        source_points=source_points,
        target_points=target_points,
        source_timestamps=timestamps,
        target_timestamps=timestamps,
        smoothing=5.0,      # 平滑因子
        num_samples=2000,   # 重采样点数
        voxel_size=voxel_size,
        visualize=True
    )
    
    # 验证精度
    T_error = np.linalg.inv(T_gt) @ result['T_final']
    R_error = T_error[:3, :3]
    t_error = T_error[:3, 3]
    
    angle_error = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1, 1))
    angle_error_deg = np.degrees(angle_error)
    translation_error = np.linalg.norm(t_error)
    
    print("\n" + "=" * 60)
    print(">>> 与真实值对比 <<<")
    print("=" * 60)
    print(f"  旋转误差:   {angle_error_deg:.4f}°")
    print(f"  平移误差:   {translation_error:.6f} (单位与点云一致)")
    
    return result


# ============================================================================
#                              主入口
# ============================================================================

if __name__ == "__main__":
    # 运行演示
    demo()
    
    # 如果要使用自己的数据，可以这样：
    # 
    # # 方式1：从CSV加载
    # source_pts, source_ts = load_trajectory_csv("source_trajectory.csv")
    # target_pts, target_ts = load_trajectory_csv("target_trajectory.csv")
    #
    # # 方式2：从点云文件加载
    # source_pts = load_point_cloud_file("source.pcd")
    # target_pts = load_point_cloud_file("target.pcd")
    #
    # # 执行配准
    # result = registration_pipeline(
    #     source_points=source_pts,
    #     target_points=target_pts,
    #     source_timestamps=source_ts,  # 可选
    #     target_timestamps=target_ts,  # 可选
    #     smoothing=5.0,
    #     num_samples=5000,
    #     voxel_size=0.02,  # 根据点云尺度调整
    #     visualize=True
    # )

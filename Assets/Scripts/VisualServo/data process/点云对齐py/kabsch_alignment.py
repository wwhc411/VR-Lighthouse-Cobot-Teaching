"""
时间对齐轨迹的Kabsch配准与可视化
================================

适用场景：
- 两条轨迹点数相同，已时间对齐（一一对应）
- 存在刚性变换（旋转+平移）关系
- CSV第4、5、6列为XYZ坐标

使用方法：
    修改 SOURCE_CSV 和 TARGET_CSV 路径后运行
"""

import numpy as np
import open3d as o3d
import copy
import sys
import os


def load_csv_trajectory(filepath, xyz_cols=(3, 4, 5), skip_header=1):
    """从CSV加载点云"""
    print(f"  加载: {os.path.basename(filepath)}")
    data = np.loadtxt(filepath, delimiter=',', skiprows=skip_header)
    points = data[:, xyz_cols]
    print(f"  点数: {len(points)}")
    return points


def kabsch_align(source_points, target_points):
    """
    Kabsch算法：计算source到target的最优刚性变换
    
    参数:
        source_points: (N, 3) 源点云
        target_points: (N, 3) 目标点云（必须一一对应）
    
    返回:
        R: (3, 3) 旋转矩阵
        t: (3,) 平移向量
        T: (4, 4) 完整变换矩阵
    """
    assert len(source_points) == len(target_points), "点数必须相同！"
    
    # 计算质心
    source_center = np.mean(source_points, axis=0)
    target_center = np.mean(target_points, axis=0)
    
    # 去质心化
    source_centered = source_points - source_center
    target_centered = target_points - target_center
    
    # 计算协方差矩阵
    H = source_centered.T @ target_centered
    
    # SVD分解
    U, S, Vt = np.linalg.svd(H)
    
    # 计算旋转矩阵
    R = Vt.T @ U.T
    
    # 处理反射情况（确保det(R)=1）
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # 计算平移
    t = target_center - R @ source_center
    
    # 构建4x4变换矩阵
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    
    return R, t, T


def compute_alignment_error(source_points, target_points, T):
    """
    计算配准后的误差
    """
    # 变换源点云
    N = len(source_points)
    source_homo = np.hstack([source_points, np.ones((N, 1))])
    transformed = (T @ source_homo.T).T[:, :3]
    
    # 计算点对点误差
    errors = np.linalg.norm(transformed - target_points, axis=1)
    
    return {
        'mean': np.mean(errors),
        'rmse': np.sqrt(np.mean(errors**2)),
        'max': np.max(errors),
        'min': np.min(errors),
        'std': np.std(errors),
        'median': np.median(errors),
        'p95': np.percentile(errors, 95),
        'p99': np.percentile(errors, 99),
    }, errors


def visualize_alignment(source_points, target_points, T, errors=None, title="Kabsch配准结果"):
    """
    可视化配准结果
    
    参数:
        source_points: 原始源点云
        target_points: 目标点云
        T: 变换矩阵
        errors: 每个点的误差（用于颜色映射）
    """
    # 创建点云对象
    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(source_points)
    
    target_pcd = o3d.geometry.PointCloud()
    target_pcd.points = o3d.utility.Vector3dVector(target_points)
    
    # 复制用于显示
    source_transformed = copy.deepcopy(source_pcd)
    source_transformed.transform(T)
    
    # 着色
    if errors is not None:
        # 根据误差着色（误差小=绿色，误差大=红色）
        errors_normalized = (errors - errors.min()) / (errors.max() - errors.min() + 1e-10)
        colors = np.zeros((len(errors), 3))
        colors[:, 0] = errors_normalized        # R: 误差越大越红
        colors[:, 1] = 1 - errors_normalized    # G: 误差越小越绿
        source_transformed.colors = o3d.utility.Vector3dVector(colors)
    else:
        source_transformed.paint_uniform_color([1, 0.706, 0])  # 橙色
    
    target_pcd.paint_uniform_color([0, 0.651, 0.929])  # 蓝色
    
    # 坐标系
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=50, origin=[0, 0, 0]
    )
    
    print("\n可视化说明:")
    if errors is not None:
        print("  源点云：按误差着色（绿色=误差小，红色=误差大）")
    else:
        print("  橙色：源点云（已变换）")
    print("  蓝色：目标点云")
    print("  按 Q 关闭窗口")
    
    o3d.visualization.draw_geometries(
        [source_transformed, target_pcd, coord_frame],
        window_name=title,
        width=1280,
        height=720
    )


def main(source_csv, target_csv):
    """主流程"""
    print("=" * 60)
    print("Kabsch刚性配准 - 时间对齐轨迹")
    print("=" * 60)
    
    # 加载数据
    print("\n▶ 加载CSV数据")
    print("-" * 40)
    source_points = load_csv_trajectory(source_csv)
    target_points = load_csv_trajectory(target_csv)
    
    if len(source_points) != len(target_points):
        print(f"\n⚠ 警告：点数不匹配！源={len(source_points)}, 目标={len(target_points)}")
        print("将使用较小的点数进行配准...")
        min_len = min(len(source_points), len(target_points))
        source_points = source_points[:min_len]
        target_points = target_points[:min_len]
    
    # 配准前误差
    print("\n▶ 配准前分析")
    print("-" * 40)
    pre_errors = np.linalg.norm(source_points - target_points, axis=1)
    print(f"  点对点距离 (配准前):")
    print(f"    平均: {np.mean(pre_errors):.4f}")
    print(f"    最大: {np.max(pre_errors):.4f}")
    print(f"    最小: {np.min(pre_errors):.4f}")
    
    # Kabsch配准
    print("\n▶ Kabsch刚性配准")
    print("-" * 40)
    R, t, T = kabsch_align(source_points, target_points)
    
    # 计算配准后误差
    error_dict, errors = compute_alignment_error(source_points, target_points, T)
    
    # 提取旋转角度
    angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    angle_deg = np.degrees(angle)
    
    # 结果汇总
    print("\n" + "=" * 60)
    print(">>> 配准结果汇总 <<<")
    print("=" * 60)
    
    print(f"\n【配准后误差】(单位与CSV坐标一致)")
    print(f"  平均误差 (Mean):     {error_dict['mean']:.6f}")
    print(f"  均方根误差 (RMSE):   {error_dict['rmse']:.6f}")
    print(f"  最大误差 (Max):      {error_dict['max']:.6f}")
    print(f"  最小误差 (Min):      {error_dict['min']:.6f}")
    print(f"  标准差 (Std):        {error_dict['std']:.6f}")
    print(f"  中位数 (Median):     {error_dict['median']:.6f}")
    print(f"  95%分位 (P95):       {error_dict['p95']:.6f}")
    print(f"  99%分位 (P99):       {error_dict['p99']:.6f}")
    
    print(f"\n【变换矩阵】")
    print(T)
    
    print(f"\n【变换分解】")
    print(f"  旋转角度:   {angle_deg:.4f}°")
    print(f"  平移向量:   [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]")
    print(f"  平移距离:   {np.linalg.norm(t):.6f}")
    
    print(f"\n【误差改善】")
    print(f"  配准前平均误差: {np.mean(pre_errors):.4f}")
    print(f"  配准后平均误差: {error_dict['mean']:.4f}")
    print(f"  改善比例: {(1 - error_dict['mean']/np.mean(pre_errors))*100:.2f}%")
    
    # 可视化
    print("\n正在显示配准结果...")
    visualize_alignment(source_points, target_points, T, errors, "Kabsch配准结果（按误差着色）")
    
    return {
        'T': T,
        'R': R,
        't': t,
        'error': error_dict,
        'errors': errors,
    }


# ============================================================================
#                              主入口
# ============================================================================

if __name__ == "__main__":
    # ========================================================================
    # 📝 在此处修改你的两个CSV文件路径
    # ========================================================================
    
    # 源轨迹CSV文件
    SOURCE_CSV = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\PlaybackRecord_HighFreq_7_20260202_145911_7_20260202_150007.csv"
    # 目标轨迹CSV文件
    TARGET_CSV = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\PlaybackRecord_HighFreq_7_20260202_145911_7_20260202_150124.csv"
    
    # ========================================================================
    
    # 命令行参数优先
    if len(sys.argv) >= 3:
        SOURCE_CSV = sys.argv[1]
        TARGET_CSV = sys.argv[2]
    
    # 检查文件
    if not os.path.exists(SOURCE_CSV):
        print(f"错误：文件不存在: {SOURCE_CSV}")
        sys.exit(1)
    if not os.path.exists(TARGET_CSV):
        print(f"错误：文件不存在: {TARGET_CSV}")
        sys.exit(1)
    
    # 执行配准
    result = main(SOURCE_CSV, TARGET_CSV)

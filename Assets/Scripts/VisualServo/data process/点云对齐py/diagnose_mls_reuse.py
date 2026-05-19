# -*- coding: utf-8 -*-
"""
MLS配准复用误差诊断脚本
========================
逐步拆解 训练→坐标转换→复用 全流程，定位误差来源

诊断项目：
1. 训练阶段回验（基线）
2. 坐标系转换数学验证
3. 复用流程各环节误差分离
4. 预处理差异对比
5. 弧长归一化差异分析

Author: Diagnostic Script
"""

import numpy as np
import json
import os
import sys
from scipy.ndimage import gaussian_filter1d

# ============================================================================
# 路径配置
# ============================================================================
BASE_DIR = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings"

MLS_LIGHTHOUSE = os.path.join(BASE_DIR, "mls_transform.json")
MLS_ROBOT = os.path.join(BASE_DIR, "mls_transform_robot.json")

# 复用输入（机器人坐标系下的tracker数据）
REUSE_INPUT = os.path.join(BASE_DIR, "tracker6 - tcp.csv")
# 复用目标（机器人坐标系下的tcp数据）
REUSE_TARGET = os.path.join(BASE_DIR, "tcp6 - tcp.csv")

# 训练文件
TRAIN_SOURCE = os.path.join(BASE_DIR, "tracker6.csv")
TRAIN_TARGET = os.path.join(BASE_DIR, "tcp6.csv")


def compute_arc_length(points):
    diffs = np.diff(points, axis=0)
    return np.concatenate([[0], np.cumsum(np.linalg.norm(diffs, axis=1))])


def smooth_gaussian(points, sigma=1.5):
    smoothed = np.zeros_like(points)
    for i in range(3):
        smoothed[:, i] = gaussian_filter1d(points[:, i], sigma=sigma)
    return smoothed


def load_csv_xyz(filepath, xyz_cols=(3, 4, 5)):
    data = np.loadtxt(filepath, delimiter=',', skiprows=1)
    return data[:, list(xyz_cols)]


def weighted_kabsch(source, target, weights):
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


def transform_point_grid(point, norm_s, grid_positions, grid_transforms):
    norm_s = np.clip(norm_s, 0, 1)
    G = len(grid_positions)
    idx = np.searchsorted(grid_positions, norm_s) - 1
    idx = np.clip(idx, 0, G - 2)
    s_left = grid_positions[idx]
    s_right = grid_positions[idx + 1]
    t = (norm_s - s_left) / (s_right - s_left + 1e-10)
    t = np.clip(t, 0, 1)
    point_homo = np.append(point, 1)
    result_left = (grid_transforms[idx] @ point_homo)[:3]
    result_right = (grid_transforms[idx + 1] @ point_homo)[:3]
    return (1 - t) * result_left + t * result_right


# ============================================================================
# 收集结果
# ============================================================================
results = {}

print("=" * 70)
print("MLS配准复用全流程误差诊断")
print("=" * 70)

# ============================================================================
# 1. 加载MLS变换文件
# ============================================================================
print("\n▶ 1. 加载MLS变换文件")
print("-" * 50)

with open(MLS_LIGHTHOUSE, 'r', encoding='utf-8') as f:
    mls_L = json.load(f)

with open(MLS_ROBOT, 'r', encoding='utf-8') as f:
    mls_R = json.load(f)

src_L = np.array(mls_L["training_data"]["source_points"])
tgt_L = np.array(mls_L["training_data"]["target_points"])
arc_L = np.array(mls_L["training_data"]["normalized_arc_lengths"])
grid_pos_L = np.array(mls_L["grid"]["grid_positions"])
grid_T_L = np.array(mls_L["grid"]["grid_transforms"])

src_R = np.array(mls_R["training_data"]["source_points"])
tgt_R = np.array(mls_R["training_data"]["target_points"])
arc_R = np.array(mls_R["training_data"]["normalized_arc_lengths"])
grid_pos_R = np.array(mls_R["grid"]["grid_positions"])
grid_T_R = np.array(mls_R["grid"]["grid_transforms"])

bandwidth = mls_L["bandwidth"]
total_arc = mls_L["total_arc_length"]

print(f"  灯塔坐标系: {len(src_L)} 训练点, {len(grid_T_L)} 网格变换")
print(f"  机器人坐标系: {len(src_R)} 训练点, {len(grid_T_R)} 网格变换")
print(f"  带宽: h={bandwidth}")
print(f"  训练弧长: {total_arc:.2f}mm")

results["带宽"] = bandwidth
results["训练点数"] = len(src_L)
results["网格点数"] = len(grid_T_L)

# ============================================================================
# 2. 训练回验（基线误差）- 灯塔坐标系
# ============================================================================
print("\n▶ 2. 训练回验 - 灯塔坐标系（基线）")
print("-" * 50)

errors_train_L = []
for i in range(len(src_L)):
    result = transform_point_grid(src_L[i], arc_L[i], grid_pos_L, grid_T_L)
    errors_train_L.append(np.linalg.norm(result - tgt_L[i]))

errors_train_L = np.array(errors_train_L)
print(f"  平均误差: {np.mean(errors_train_L):.4f}mm")
print(f"  RMSE:    {np.sqrt(np.mean(errors_train_L**2)):.4f}mm")
print(f"  最大误差: {np.max(errors_train_L):.4f}mm")
print(f"  P95:     {np.percentile(errors_train_L, 95):.4f}mm")

results["灯塔_训练回验_mean"] = float(np.mean(errors_train_L))
results["灯塔_训练回验_rmse"] = float(np.sqrt(np.mean(errors_train_L**2)))
results["灯塔_训练回验_max"] = float(np.max(errors_train_L))

# ============================================================================
# 3. 坐标转换数学验证
# ============================================================================
print("\n▶ 3. 坐标转换数学验证")
print("-" * 50)

# 提取手眼矩阵
T_LR = np.array(mls_R["hand_eye_transform"]["matrix_4x4"])
T_RL = np.linalg.inv(T_LR)
R_he = T_LR[:3, :3]
t_he = T_LR[:3, 3]

print(f"  手眼矩阵旋转角: {mls_R['hand_eye_transform']['rotation_angle_deg']:.4f}°")
print(f"  手眼矩阵平移: {t_he}")

# 3a. 验证训练点转换一致性
src_L_to_R = (R_he @ src_L.T).T + t_he
tgt_L_to_R = (R_he @ tgt_L.T).T + t_he

src_conv_error = np.max(np.linalg.norm(src_L_to_R - src_R, axis=1))
tgt_conv_error = np.max(np.linalg.norm(tgt_L_to_R - tgt_R, axis=1))
print(f"  训练源点转换误差(max): {src_conv_error:.2e}")
print(f"  训练目标点转换误差(max): {tgt_conv_error:.2e}")

results["训练点转换误差"] = float(max(src_conv_error, tgt_conv_error))

# 3b. 验证网格变换转换一致性: T_R = T_LR @ T_L @ T_RL
grid_conv_errors = []
for i in range(len(grid_T_L)):
    T_R_expected = T_LR @ grid_T_L[i] @ T_RL
    error = np.max(np.abs(T_R_expected - grid_T_R[i]))
    grid_conv_errors.append(error)

grid_conv_errors = np.array(grid_conv_errors)
print(f"  网格变换转换误差(max): {np.max(grid_conv_errors):.2e}")
print(f"  网格变换转换误差(mean): {np.mean(grid_conv_errors):.2e}")

results["网格变换转换误差"] = float(np.max(grid_conv_errors))

# 3c. 训练回验 - 机器人坐标系（坐标转换后）
print("\n  ▶ 3c. 转换后训练数据回验 - 机器人坐标系")
errors_train_R = []
for i in range(len(src_R)):
    result = transform_point_grid(src_R[i], arc_R[i], grid_pos_R, grid_T_R)
    errors_train_R.append(np.linalg.norm(result - tgt_R[i]))

errors_train_R = np.array(errors_train_R)
print(f"  平均误差: {np.mean(errors_train_R):.4f}mm")
print(f"  RMSE:    {np.sqrt(np.mean(errors_train_R**2)):.4f}mm")
print(f"  最大误差: {np.max(errors_train_R):.4f}mm")
print(f"  P95:     {np.percentile(errors_train_R, 95):.4f}mm")

results["机器人_训练回验_mean"] = float(np.mean(errors_train_R))
results["机器人_训练回验_rmse"] = float(np.sqrt(np.mean(errors_train_R**2)))
results["机器人_训练回验_max"] = float(np.max(errors_train_R))

# 灯塔→机器人回验误差对比
delta_max = abs(np.max(errors_train_R) - np.max(errors_train_L))
print(f"\n  ⭐ 坐标转换引入的最大误差增量: {delta_max:.6f}mm")
results["坐标转换引入误差增量"] = float(delta_max)

# ============================================================================
# 4. 复用测试 - 使用训练数据的原始CSV（模拟完全一致的情况）
# ============================================================================
print("\n▶ 4. 复用流程模拟（训练数据原始CSV）")
print("-" * 50)

# 加载训练用的原始CSV（灯塔坐标系）
train_src_raw = load_csv_xyz(TRAIN_SOURCE)
train_tgt_raw = load_csv_xyz(TRAIN_TARGET)
print(f"  训练源CSV: {len(train_src_raw)} 点")
print(f"  训练目标CSV: {len(train_tgt_raw)} 点")

# 预平滑
train_src_smoothed = smooth_gaussian(train_src_raw, sigma=1.5)
train_src_arc = compute_arc_length(train_src_smoothed)
train_src_total = train_src_arc[-1]
train_src_narc = train_src_arc / train_src_total

print(f"  预平滑后弧长: {train_src_total:.2f}mm")
print(f"  训练时总弧长: {total_arc:.2f}mm")
print(f"  弧长差异: {abs(train_src_total - total_arc):.2f}mm ({abs(train_src_total - total_arc)/total_arc*100:.2f}%)")

results["训练CSV弧长"] = float(train_src_total)
results["训练存储弧长"] = float(total_arc)
results["训练弧长差异比(%)"] = float(abs(train_src_total - total_arc)/total_arc*100)

# 使用灯塔坐标系网格对原始CSV做变换
errors_csv_L = []
for i in range(len(train_src_raw)):
    ns = train_src_narc[i]
    result = transform_point_grid(train_src_raw[i], ns, grid_pos_L, grid_T_L)
    # 找对应的目标点 (简单用最近弧长)
    if i < len(train_tgt_raw):
        errors_csv_L.append(np.linalg.norm(result - train_tgt_raw[i]))

errors_csv_L = np.array(errors_csv_L[:min(len(train_src_raw), len(train_tgt_raw))])
print(f"\n  原始CSV + 灯塔网格 (无重采样):")
print(f"    平均误差: {np.mean(errors_csv_L):.4f}mm")
print(f"    RMSE:    {np.sqrt(np.mean(errors_csv_L**2)):.4f}mm") 
print(f"    最大误差: {np.max(errors_csv_L):.4f}mm")

results["原始CSV_灯塔直接变换_mean"] = float(np.mean(errors_csv_L))
results["原始CSV_灯塔直接变换_max"] = float(np.max(errors_csv_L))

# ============================================================================
# 5. 复用输入数据分析（机器人坐标系 tracker6-tcp）
# ============================================================================
print("\n▶ 5. 复用输入数据分析")
print("-" * 50)

reuse_input = load_csv_xyz(REUSE_INPUT)
reuse_target = load_csv_xyz(REUSE_TARGET)
print(f"  复用输入: {len(reuse_input)} 点")
print(f"  复用目标: {len(reuse_target)} 点")

print(f"\n  复用输入坐标范围:")
print(f"    X: [{reuse_input[:,0].min():.1f}, {reuse_input[:,0].max():.1f}]")
print(f"    Y: [{reuse_input[:,1].min():.1f}, {reuse_input[:,1].max():.1f}]")
print(f"    Z: [{reuse_input[:,2].min():.1f}, {reuse_input[:,2].max():.1f}]")

print(f"\n  机器人坐标系训练源点范围:")
print(f"    X: [{src_R[:,0].min():.1f}, {src_R[:,0].max():.1f}]")
print(f"    Y: [{src_R[:,1].min():.1f}, {src_R[:,1].max():.1f}]")
print(f"    Z: [{src_R[:,2].min():.1f}, {src_R[:,2].max():.1f}]")

print(f"\n  复用目标坐标范围:")
print(f"    X: [{reuse_target[:,0].min():.1f}, {reuse_target[:,0].max():.1f}]")
print(f"    Y: [{reuse_target[:,1].min():.1f}, {reuse_target[:,1].max():.1f}]")
print(f"    Z: [{reuse_target[:,2].min():.1f}, {reuse_target[:,2].max():.1f}]")

print(f"\n  机器人坐标系训练目标点范围:")
print(f"    X: [{tgt_R[:,0].min():.1f}, {tgt_R[:,0].max():.1f}]")
print(f"    Y: [{tgt_R[:,1].min():.1f}, {tgt_R[:,1].max():.1f}]")
print(f"    Z: [{tgt_R[:,2].min():.1f}, {tgt_R[:,2].max():.1f}]")

# ============================================================================
# 6. 模拟完整的 apply_transform 复用流程
# ============================================================================
print("\n▶ 6. 模拟完整复用流程 (apply_transform逻辑)")
print("-" * 50)

# 6a. 预平滑
reuse_smoothed = smooth_gaussian(reuse_input, sigma=1.5)
reuse_arc = compute_arc_length(reuse_smoothed)
reuse_total_arc = reuse_arc[-1]
reuse_narc = reuse_arc / reuse_total_arc

print(f"  预平滑后弧长: {reuse_total_arc:.2f}mm")
print(f"  训练存储弧长: {total_arc:.2f}mm")
print(f"  弧长比: {reuse_total_arc/total_arc:.4f}")

results["复用弧长"] = float(reuse_total_arc)
results["复用/训练弧长比"] = float(reuse_total_arc / total_arc)

# 6b. 使用原始点坐标 + 预平滑弧长做变换（与apply_transform.py一致）
errors_reuse = []
transformed_reuse = np.zeros_like(reuse_input)
for i in range(len(reuse_input)):
    result = transform_point_grid(reuse_input[i], reuse_narc[i], grid_pos_R, grid_T_R)
    transformed_reuse[i] = result

# 需要找到对应关系来计算误差
# 由于复用输入和目标点数可能不同，使用弧长对齐
target_smoothed = smooth_gaussian(reuse_target, sigma=1.5)
target_arc = compute_arc_length(target_smoothed)
target_total_arc = target_arc[-1]
target_narc = target_arc / target_total_arc

# 使用简单的逐点对应（假设帧号一致）
N_compare = min(len(transformed_reuse), len(reuse_target))
errors_reuse = np.linalg.norm(transformed_reuse[:N_compare] - reuse_target[:N_compare], axis=1)

print(f"\n  复用结果(原始点+预平滑弧长, grid模式):")
print(f"    对比点数: {N_compare}")
print(f"    平均误差: {np.mean(errors_reuse):.4f}mm")
print(f"    RMSE:    {np.sqrt(np.mean(errors_reuse**2)):.4f}mm")
print(f"    最大误差: {np.max(errors_reuse):.4f}mm")
print(f"    P95:     {np.percentile(errors_reuse, 95):.4f}mm")

results["复用_原始点_mean"] = float(np.mean(errors_reuse))
results["复用_原始点_rmse"] = float(np.sqrt(np.mean(errors_reuse**2)))
results["复用_原始点_max"] = float(np.max(errors_reuse))

# 6c. 找到误差最大的位置
worst_idx = np.argmax(errors_reuse)
print(f"\n  最大误差位置: 索引{worst_idx}")
print(f"    弧长位置(归一化): {reuse_narc[worst_idx]:.4f}")
print(f"    输入点: {reuse_input[worst_idx]}")
print(f"    变换结果: {transformed_reuse[worst_idx]}")
print(f"    期望目标: {reuse_target[worst_idx]}")
print(f"    误差: {errors_reuse[worst_idx]:.4f}mm")

results["最大误差位置_弧长"] = float(reuse_narc[worst_idx])
results["最大误差位置_索引"] = int(worst_idx)

# ============================================================================
# 7. 分离误差来源：预处理差异 vs 变换本身
# ============================================================================
print("\n▶ 7. 分离误差来源")
print("-" * 50)

# 7a. 测试：用预平滑数据（而非原始数据）做变换
errors_smoothed = []
transformed_smoothed = np.zeros_like(reuse_smoothed)
for i in range(len(reuse_smoothed)):
    result = transform_point_grid(reuse_smoothed[i], reuse_narc[i], grid_pos_R, grid_T_R)
    transformed_smoothed[i] = result

errors_smoothed = np.linalg.norm(transformed_smoothed[:N_compare] - reuse_target[:N_compare], axis=1)

print(f"\n  7a. 预平滑数据+预平滑弧长:")
print(f"    平均误差: {np.mean(errors_smoothed):.4f}mm")
print(f"    最大误差: {np.max(errors_smoothed):.4f}mm")

results["复用_平滑点_mean"] = float(np.mean(errors_smoothed))
results["复用_平滑点_max"] = float(np.max(errors_smoothed))

# 7b. 训练数据原始坐标 vs 训练数据存储坐标 - 确认训练时预处理了什么
# MLS训练时源数据是经过完整预处理流程的（预平滑→重采样→后平滑）
# 但复用时只做了预平滑，没有重采样和后平滑
print(f"\n  7b. 训练流程 vs 复用流程差异:")
print(f"    训练流程: 加载CSV → 预平滑(σ=1.5) → 自适应重采样 → 后平滑(σ=5) → MLS训练")
print(f"    复用流程: 加载CSV → 预平滑(σ=1.5) → 直接MLS变换（无重采样/后平滑）")
print(f"    ⚠ 差异1: 复用时没有自适应空间重采样（步骤不同）")
print(f"    ⚠ 差异2: 复用时没有后平滑σ=5（训练数据更平滑）")

results["流程差异"] = "复用缺少: 自适应空间重采样 + 后平滑σ=5"

# 7c. 训练数据点数 vs 原始CSV点数对比
print(f"\n  7c. 数据密度对比:")
print(f"    训练后点数: {len(src_L)} (经过重采样)")
print(f"    原始CSV点数: {len(train_src_raw)}")
print(f"    复用输入点数: {len(reuse_input)}")

# ============================================================================
# 8. 端点区域分析（MLS端点退化）
# ============================================================================
print("\n▶ 8. 误差分布分析 - 端点 vs 中段")
print("-" * 50)

# 把轨迹分成5个区间看误差分布
n_bins = 10
for bin_idx in range(n_bins):
    lo = bin_idx / n_bins
    hi = (bin_idx + 1) / n_bins
    mask = (reuse_narc[:N_compare] >= lo) & (reuse_narc[:N_compare] < hi)
    if mask.sum() > 0:
        bin_errors = errors_reuse[mask]
        print(f"  弧长 [{lo:.1f}, {hi:.1f}): {mask.sum():4d}点, "
              f"平均={np.mean(bin_errors):.2f}mm, 最大={np.max(bin_errors):.2f}mm")

# ============================================================================
# 9. 网格变换分析 - 检查robot坐标系下变换合理性
# ============================================================================
print("\n▶ 9. 网格变换特性分析")
print("-" * 50)

# 灯塔坐标系
rotation_angles_L = []
translation_mags_L = []
for i in range(len(grid_T_L)):
    R = grid_T_L[i][:3, :3]
    t = grid_T_L[i][:3, 3]
    angle = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    rotation_angles_L.append(angle)
    translation_mags_L.append(np.linalg.norm(t))

# 机器人坐标系
rotation_angles_R = []
translation_mags_R = []
for i in range(len(grid_T_R)):
    R = grid_T_R[i][:3, :3]
    t = grid_T_R[i][:3, 3]
    angle = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    rotation_angles_R.append(angle)
    translation_mags_R.append(np.linalg.norm(t))

rotation_angles_L = np.array(rotation_angles_L)
translation_mags_L = np.array(translation_mags_L)
rotation_angles_R = np.array(rotation_angles_R)
translation_mags_R = np.array(translation_mags_R)

print(f"  灯塔坐标系网格变换:")
print(f"    旋转角: [{rotation_angles_L.min():.2f}°, {rotation_angles_L.max():.2f}°], 均值={rotation_angles_L.mean():.2f}°")
print(f"    平移量: [{translation_mags_L.min():.1f}, {translation_mags_L.max():.1f}]mm, 均值={translation_mags_L.mean():.1f}mm")

print(f"  机器人坐标系网格变换:")
print(f"    旋转角: [{rotation_angles_R.min():.2f}°, {rotation_angles_R.max():.2f}°], 均值={rotation_angles_R.mean():.2f}°")
print(f"    平移量: [{translation_mags_R.min():.1f}, {translation_mags_R.max():.1f}]mm, 均值={translation_mags_R.mean():.1f}mm")

results["灯塔_旋转角_max"] = float(rotation_angles_L.max())
results["灯塔_平移量_max"] = float(translation_mags_L.max())
results["机器人_旋转角_max"] = float(rotation_angles_R.max())
results["机器人_平移量_max"] = float(translation_mags_R.max())

# ============================================================================
# 10. 关键测试：如果输入点不在训练点附近，变换是否可靠
# ============================================================================
print("\n▶ 10. 输入偏移敏感度测试")
print("-" * 50)

# 取训练数据中段的一个点
mid = len(src_R) // 2
test_point = src_R[mid]
test_target = tgt_R[mid]
test_norm_s = arc_R[mid]

# 正常变换
result_exact = transform_point_grid(test_point, test_norm_s, grid_pos_R, grid_T_R)
error_exact = np.linalg.norm(result_exact - test_target)

print(f"  训练点 [{mid}] 精确变换误差: {error_exact:.4f}mm")

# 施加不同大小的空间偏移
for offset_mm in [0.5, 1.0, 2.0, 5.0, 10.0, 20.0]:
    offset_vec = np.array([offset_mm, 0, 0])  # 仅X方向偏移
    result_offset = transform_point_grid(test_point + offset_vec, test_norm_s, grid_pos_R, grid_T_R)
    # 期望输出也应该偏移相同量
    expected_offset = test_target + offset_vec  # 理想情况
    error_offset = np.linalg.norm(result_offset - expected_offset)
    
    # 实际变换引入的额外误差
    result_offset_error = np.linalg.norm(result_offset - (test_target + offset_vec))
    print(f"  偏移 {offset_mm:5.1f}mm → 额外误差: {result_offset_error:.4f}mm")

# ============================================================================
# 11. 关键测试：复用输入与训练源点之间的空间距离
# ============================================================================
print("\n▶ 11. 复用输入 vs 训练数据 空间距离分析")
print("-" * 50)

# 对复用输入的每个点，找到最近的训练源点
from scipy.spatial import KDTree
tree = KDTree(src_R)
distances, _ = tree.query(reuse_input)

print(f"  复用输入到最近训练点的距离:")
print(f"    平均: {np.mean(distances):.2f}mm")
print(f"    最大: {np.max(distances):.2f}mm")
print(f"    P95:  {np.percentile(distances, 95):.2f}mm")
print(f"    P99:  {np.percentile(distances, 99):.2f}mm")

results["输入_到训练点距离_mean"] = float(np.mean(distances))
results["输入_到训练点距离_max"] = float(np.max(distances))

# 误差与空间距离的相关性
corr = np.corrcoef(distances[:N_compare], errors_reuse[:N_compare])[0, 1]
print(f"\n  误差 vs 空间距离 相关系数: {corr:.4f}")
results["误差_空间距离_相关系数"] = float(corr)

# ============================================================================
# 12. transform_trajectory方法测试 - 使用full模式对比
# ============================================================================
print("\n▶ 12. full模式对比（完整加权Kabsch, 使用robot坐标系训练数据）")
print("-" * 50)

# 使用少量采样点测试full模式
n_test = 50
test_indices = np.linspace(0, len(reuse_input) - 1, n_test, dtype=int)
errors_full = []

for i in test_indices:
    ns = reuse_narc[i]
    distances_w = np.abs(ns - arc_R)
    weights = np.exp(-(distances_w ** 2) / (bandwidth ** 2))
    
    if weights.sum() < 1e-6:
        k = min(20, len(src_R))
        nearest_indices = np.argsort(distances_w)[:k]
        weights = np.zeros(len(src_R))
        weights[nearest_indices] = 1.0
    
    T_local = weighted_kabsch(src_R, tgt_R, weights)
    result = (T_local @ np.append(reuse_input[i], 1))[:3]
    
    if i < len(reuse_target):
        errors_full.append(np.linalg.norm(result - reuse_target[i]))

errors_full = np.array(errors_full)
print(f"  full模式（{n_test}个采样点）:")
print(f"    平均误差: {np.mean(errors_full):.4f}mm")
print(f"    最大误差: {np.max(errors_full):.4f}mm")

results["full模式_mean"] = float(np.mean(errors_full))
results["full模式_max"] = float(np.max(errors_full))

# 对比grid模式
errors_grid_sample = errors_reuse[test_indices[:len(errors_full)]]
print(f"  grid模式（相同点）:")
print(f"    平均误差: {np.mean(errors_grid_sample):.4f}mm")
print(f"    最大误差: {np.max(errors_grid_sample):.4f}mm")

results["grid模式采样_mean"] = float(np.mean(errors_grid_sample))
results["grid模式采样_max"] = float(np.max(errors_grid_sample))

# ============================================================================
# 13. 预处理一致性测试 - 完全模拟训练流程后再变换
# ============================================================================
print("\n▶ 13. 完全模拟训练流程后变换 (预平滑+重采样+后平滑)")
print("-" * 50)

# 导入自适应重采样
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from tunable_registration import adaptive_spatial_resample
    
    # 对复用数据做完整训练预处理
    reuse_pre_smooth = smooth_gaussian(reuse_input, sigma=1.5)
    reuse_resampled, _ = adaptive_spatial_resample(
        reuse_pre_smooth, noise_level=0.5, noise_suppression_factor=2,
        min_samples=500, max_samples=5000, verbose=False)
    reuse_post_smooth = smooth_gaussian(reuse_resampled, sigma=5)
    
    # 计算弧长
    reuse_full_arc = compute_arc_length(reuse_post_smooth)
    reuse_full_narc = reuse_full_arc / reuse_full_arc[-1]
    
    print(f"  完全预处理后点数: {len(reuse_post_smooth)}")
    print(f"  完全预处理后弧长: {reuse_full_arc[-1]:.2f}mm")
    
    # 使用原始点做变换 (与apply_transform一致)  
    # 但这里没有原始点到重采样点的对应关系
    # 所以用重采样后的点做变换
    errors_full_preprocess = []
    transformed_full = np.zeros_like(reuse_post_smooth)
    for i in range(len(reuse_post_smooth)):
        result = transform_point_grid(reuse_post_smooth[i], reuse_full_narc[i], grid_pos_R, grid_T_R)
        transformed_full[i] = result
    
    # 目标数据也做相同处理
    target_pre_smooth = smooth_gaussian(reuse_target, sigma=1.5)
    target_resampled, _ = adaptive_spatial_resample(
        target_pre_smooth, noise_level=0.5, noise_suppression_factor=2,
        min_samples=500, max_samples=5000, verbose=False)
    target_post_smooth = smooth_gaussian(target_resampled, sigma=5)
    
    N_full = min(len(transformed_full), len(target_post_smooth))
    errors_full_preprocess = np.linalg.norm(
        transformed_full[:N_full] - target_post_smooth[:N_full], axis=1)
    
    print(f"\n  完全预处理后变换误差:")
    print(f"    对比点数: {N_full}")
    print(f"    平均误差: {np.mean(errors_full_preprocess):.4f}mm")
    print(f"    RMSE:    {np.sqrt(np.mean(errors_full_preprocess**2)):.4f}mm")
    print(f"    最大误差: {np.max(errors_full_preprocess):.4f}mm")
    
    results["完全预处理_mean"] = float(np.mean(errors_full_preprocess))
    results["完全预处理_max"] = float(np.max(errors_full_preprocess))
    
except ImportError as e:
    print(f"  ⚠ 无法导入tunable_registration: {e}")
    results["完全预处理_mean"] = "N/A"

# ============================================================================
# 14. 总结
# ============================================================================
print("\n" + "=" * 70)
print("诊断总结")
print("=" * 70)

print(f"\n  1. 训练回验(灯塔):     mean={results.get('灯塔_训练回验_mean', 0):.4f}, max={results.get('灯塔_训练回验_max', 0):.4f}mm")
print(f"  2. 训练回验(机器人):   mean={results.get('机器人_训练回验_mean', 0):.4f}, max={results.get('机器人_训练回验_max', 0):.4f}mm")
print(f"  3. 坐标转换引入误差:   {results.get('坐标转换引入误差增量', 0):.6f}mm")
print(f"  4. 复用(原始点+grid):  mean={results.get('复用_原始点_mean', 0):.4f}, max={results.get('复用_原始点_max', 0):.4f}mm")
print(f"  5. 复用(平滑点+grid):  mean={results.get('复用_平滑点_mean', 0):.4f}, max={results.get('复用_平滑点_max', 0):.4f}mm")
fp_mean = results.get('完全预处理_mean', 'N/A')
fp_max = results.get('完全预处理_max', 'N/A')
if isinstance(fp_mean, float):
    print(f"  6. 完全预处理后变换:   mean={fp_mean:.4f}, max={fp_max:.4f}mm")
else:
    print(f"  6. 完全预处理后变换:   {fp_mean}")
print(f"  7. full vs grid对比:   full_max={results.get('full模式_max', 0):.4f}, grid_max={results.get('grid模式采样_max', 0):.4f}mm")

# 保存结果到JSON
results_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mls_diagnosis_results.json")
with open(results_path, 'w', encoding='utf-8') as f:
    json.dump(results, f, indent=2, ensure_ascii=False)
print(f"\n  结果已保存: {results_path}")

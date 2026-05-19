"""
手眼坐标系分段配准变换转换工具
================================

将灯塔坐标系下的分段配准变换转换到机械臂坐标系

核心公式: T_i^R = T_L^R @ T_i^L @ (T_L^R)^{-1}

使用方法:
---------
1. 命令行模式：
   python convert_segmented_transform.py
   
2. Python代码调用：
   from convert_segmented_transform import convert_segmented_transform
   convert_segmented_transform(hand_eye_matrix, input_json, output_json)

Author: AI Assistant
Date: 2026-02-04
"""

import numpy as np
import json
import os
import sys
import re

# ============================================================================
#                           配置区域
# ============================================================================

# 默认文件路径（分段配准）
DEFAULT_INPUT_JSON = r"E:\Unity cangku\lighthouse_3.4\Assets\StreamingAssets\曲线\segmented_transform.json"
DEFAULT_OUTPUT_JSON = r"E:\Unity cangku\lighthouse_3.4\Assets\StreamingAssets\曲线\segmented_transform_robot.json"

# 默认文件路径（MLS配准）
DEFAULT_MLS_INPUT_JSON = r"E:\Unity cangku\lighthouse_3.4（走数据备份）\Assets\Scripts\VisualServo\算法\圆形\mls_transform.json"
DEFAULT_MLS_OUTPUT_JSON = r"E:\Unity cangku\lighthouse_3.4（走数据备份）\Assets\Scripts\VisualServo\算法\圆形\mls_transform_robot.json"

# 默认手眼变换矩阵（从用户提供的标定结果）
# Camera to Robot Base Transform (Cam → Base) = T_L^R (灯塔→机械臂)
DEFAULT_HAND_EYE_MATRIX = np.array([
  [0.820190, -0.005288, -0.572067, -0.178015],
  [-0.572053, 0.004087, -0.820207, -0.581931],
  [0.006675, 0.999978, 0.000327, 0.311207],
  [0.000000, 0.000000, 0.000000, 1.000000]
])

# 单位转换：手眼矩阵中的平移是否为米（True）或毫米（False）
# 根据用户输入 "Position (mm): (25.591, -454.959, -15.691)"，矩阵中是米
HAND_EYE_TRANSLATION_IN_METERS = True


# ============================================================================
#                     手眼变换矩阵解析函数
# ============================================================================

def parse_hand_eye_from_text(text: str) -> np.ndarray:
    """
    从手眼标定输出文本解析4x4变换矩阵
    
    支持的格式：
    1. 4x4矩阵格式（每行4个数字）
    2. 位置+四元数格式
    
    参数:
        text: 手眼标定输出文本
        
    返回:
        4x4变换矩阵 (numpy array)
    """
    
    # 尝试解析4x4矩阵格式
    # 查找类似 [0.939349, -0.003974, -0.342940, 0.025591] 的行
    matrix_pattern = r'\[([^\]]+)\]'
    matches = re.findall(matrix_pattern, text)
    
    if len(matches) >= 4:
        # 找到了矩阵行
        matrix_rows = []
        for match in matches[-4:]:  # 取最后4行（可能前面有其他括号内容）
            try:
                row = [float(x.strip()) for x in match.split(',')]
                if len(row) == 4:
                    matrix_rows.append(row)
            except ValueError:
                continue
        
        if len(matrix_rows) == 4:
            matrix = np.array(matrix_rows)
            print(f"  ✓ 成功解析4x4矩阵格式")
            return matrix
    
    # 尝试解析位置+四元数格式
    # Position (mm): (25.591, -454.959, -15.691)
    # Rotation (Quaternion): (w:0.6945, x:0.6981, y:-0.1242, z:-0.1220)
    
    pos_pattern = r'Position\s*\(mm\)\s*:\s*\(([^)]+)\)'
    quat_pattern = r'Rotation\s*\(Quaternion\)\s*:\s*\(w:([^,]+),\s*x:([^,]+),\s*y:([^,]+),\s*z:([^)]+)\)'
    
    pos_match = re.search(pos_pattern, text)
    quat_match = re.search(quat_pattern, text)
    
    if pos_match and quat_match:
        # 解析位置 (mm)
        pos_str = pos_match.group(1)
        pos = np.array([float(x.strip()) for x in pos_str.split(',')])
        pos_m = pos / 1000.0  # 转换为米
        
        # 解析四元数 (w, x, y, z)
        w = float(quat_match.group(1))
        x = float(quat_match.group(2))
        y = float(quat_match.group(3))
        z = float(quat_match.group(4))
        
        # 四元数转旋转矩阵
        R = quaternion_to_rotation_matrix(w, x, y, z)
        
        # 构建4x4矩阵
        matrix = np.eye(4)
        matrix[:3, :3] = R
        matrix[:3, 3] = pos_m
        
        print(f"  ✓ 成功解析位置+四元数格式")
        print(f"    位置(m): [{pos_m[0]:.6f}, {pos_m[1]:.6f}, {pos_m[2]:.6f}]")
        return matrix
    
    raise ValueError("无法解析手眼变换矩阵，请检查输入格式")


def quaternion_to_rotation_matrix(w, x, y, z):
    """
    四元数转旋转矩阵
    
    参数:
        w, x, y, z: 四元数分量
        
    返回:
        3x3旋转矩阵
    """
    # 归一化四元数
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    
    # 构建旋转矩阵
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])
    
    return R


def load_hand_eye_from_file(filepath: str) -> np.ndarray:
    """
    从文件加载手眼变换矩阵
    
    支持格式:
    1. .json 文件（包含 transform_matrix_4x4 或 hand_eye_matrix 字段）
    2. .txt 文件（手眼标定输出文本）
    3. .npy 文件（numpy数组）
    """
    ext = os.path.splitext(filepath)[1].lower()
    
    if ext == '.json':
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        if "transform_matrix_4x4" in data:
            return np.array(data["transform_matrix_4x4"])
        elif "hand_eye_matrix" in data:
            return np.array(data["hand_eye_matrix"])
        elif "matrix_4x4" in data:
            return np.array(data["matrix_4x4"])
        else:
            raise ValueError("JSON文件中未找到变换矩阵字段")
    
    elif ext == '.txt':
        with open(filepath, 'r', encoding='utf-8') as f:
            text = f.read()
        return parse_hand_eye_from_text(text)
    
    elif ext == '.npy':
        return np.load(filepath)
    
    else:
        # 尝试作为文本解析
        with open(filepath, 'r', encoding='utf-8') as f:
            text = f.read()
        return parse_hand_eye_from_text(text)


# ============================================================================
#                     核心转换函数
# ============================================================================

def convert_segmented_transform(
    hand_eye_matrix: np.ndarray,
    input_json_path: str,
    output_json_path: str,
    convert_translation_to_mm: bool = True,
    verbose: bool = True
) -> dict:
    """
    将灯塔坐标系下的分段配准变换转换到机械臂坐标系
    
    核心公式: T_i^R = T_L^R @ T_i^L @ (T_L^R)^{-1}
    
    参数:
        hand_eye_matrix: 手眼变换矩阵 T_L^R (4×4)，灯塔→机械臂
        input_json_path: 灯塔坐标系下分段变换JSON文件路径
        output_json_path: 输出的机械臂坐标系分段变换JSON文件路径
        convert_translation_to_mm: 如果手眼矩阵平移为米，是否转换为毫米
        verbose: 是否打印详细信息
    
    返回:
        机械臂坐标系下的分段变换字典
    """
    
    if verbose:
        print(f"\n{'='*70}")
        print("分段配准变换坐标系转换工具")
        print(f"{'='*70}")
        print(f"\n核心公式: T_i^R = T_L^R @ T_i^L @ (T_L^R)^{-1}")
    
    # ========== 1. 处理手眼变换矩阵 ==========
    if verbose:
        print(f"\n▶ 步骤1: 处理手眼变换矩阵 T_L^R (灯塔→机械臂)")
        print(f"-" * 50)
    
    T_L_to_R = hand_eye_matrix.copy()
    
    # 检查平移单位（如果平移值很小，可能是米）
    translation = T_L_to_R[:3, 3]
    if convert_translation_to_mm and np.max(np.abs(translation)) < 10:
        # 平移值很小，可能是米，转换为毫米
        T_L_to_R[:3, 3] = translation * 1000.0
        if verbose:
            print(f"  ⚠ 检测到平移值较小，已从米转换为毫米")
            print(f"    原始: [{translation[0]:.6f}, {translation[1]:.6f}, {translation[2]:.6f}] m")
            print(f"    转换后: [{T_L_to_R[0,3]:.3f}, {T_L_to_R[1,3]:.3f}, {T_L_to_R[2,3]:.3f}] mm")
    
    # 计算逆矩阵
    T_R_to_L = np.linalg.inv(T_L_to_R)
    
    # 验证逆矩阵
    identity_check = T_L_to_R @ T_R_to_L
    inv_error = np.max(np.abs(identity_check - np.eye(4)))
    
    # 提取旋转角度和平移量
    R_he = T_L_to_R[:3, :3]
    t_he = T_L_to_R[:3, 3]
    angle_he = np.degrees(np.arccos(np.clip((np.trace(R_he) - 1) / 2, -1, 1)))
    
    if verbose:
        print(f"\n  手眼变换矩阵 T_L^R:")
        for row in T_L_to_R:
            print(f"    [{row[0]:12.6f}, {row[1]:12.6f}, {row[2]:12.6f}, {row[3]:12.6f}]")
        print(f"\n  旋转角度: {angle_he:.4f}°")
        print(f"  平移向量: [{t_he[0]:.3f}, {t_he[1]:.3f}, {t_he[2]:.3f}] mm")
        print(f"  逆矩阵验证误差: {inv_error:.2e}")
        
        if inv_error > 1e-10:
            print("  ⚠️ 警告: 手眼变换矩阵可能存在数值问题")
        else:
            print("  ✓ 逆矩阵验证通过")
    
    # ========== 2. 加载灯塔坐标系下的分段变换 ==========
    if verbose:
        print(f"\n▶ 步骤2: 加载灯塔坐标系分段变换")
        print(f"-" * 50)
        print(f"  输入文件: {input_json_path}")
    
    if not os.path.exists(input_json_path):
        raise FileNotFoundError(f"找不到输入文件: {input_json_path}")
    
    with open(input_json_path, 'r', encoding='utf-8') as f:
        seg_data_L = json.load(f)
    
    num_segments = seg_data_L["num_segments"]
    mode = seg_data_L.get("mode", "arc_length")
    
    # ⭐⭐⭐ 兼容序列号模式和弧长模式
    if mode == "sequence_aligned":
        total_arc_length = seg_data_L.get("total_arc_length", seg_data_L["total_frames"])
        if verbose:
            print(f"  模式: 序列号对应模式")
            print(f"  分段数: {num_segments}")
            print(f"  总帧数: {seg_data_L['total_frames']}")
    else:
        total_arc_length = seg_data_L["total_arc_length"]
        if verbose:
            print(f"  模式: 弧长对应模式")
            print(f"  分段数: {num_segments}")
            print(f"  总弧长: {total_arc_length:.2f} mm")
            if "original_arc_length" in seg_data_L:
                print(f"  原始弧长: {seg_data_L['original_arc_length']:.2f} mm")
    
    # ========== 3. 创建机械臂坐标系下的分段变换 ==========
    if verbose:
        print(f"\n▶ 步骤3: 转换分段变换矩阵")
        print(f"-" * 50)
        print(f"  应用公式: T_i^R = T_L^R @ T_i^L @ (T_L^R)^{-1}")
    
    seg_data_R = {
        "description": "机械臂基座坐标系下的分段配准变换（从灯塔坐标系转换）",
        "coordinate_frame": "robot_base",
        "original_coordinate_frame": "lighthouse",
        "conversion_formula": "T_i^R = T_L^R @ T_i^L @ (T_L^R)^{-1}",
        "hand_eye_transform": {
            "direction": "lighthouse_to_robot (T_L^R)",
            "matrix_4x4": T_L_to_R.tolist(),
            "rotation_angle_deg": float(angle_he),
            "translation_mm": t_he.tolist()
        },
        "num_segments": num_segments,
        "total_arc_length": total_arc_length,
        "segments": []
    }
    
    # 可选字段（含序列号模式字段）
    if "original_arc_length" in seg_data_L:
        seg_data_R["original_arc_length"] = seg_data_L["original_arc_length"]
    if "mode" in seg_data_L:
        seg_data_R["mode"] = seg_data_L["mode"]
    if "total_frames" in seg_data_L:
        seg_data_R["total_frames"] = seg_data_L["total_frames"]
    if "math_formula" in seg_data_L:
        seg_data_R["original_math_formula"] = seg_data_L["math_formula"]
    
    # ========== 4. 遍历每个分段，应用坐标变换 ==========
    rotation_angle_diffs = []
    translation_diffs = []
    
    for seg_L in seg_data_L["segments"]:
        idx = seg_L["index"]
        T_i_L = np.array(seg_L["transform_4x4"])
        
        # ⭐⭐⭐ 核心公式：相似变换 ⭐⭐⭐
        T_i_R = T_L_to_R @ T_i_L @ T_R_to_L
        
        # 提取旋转和平移
        R_i_R = T_i_R[:3, :3]
        t_i_R = T_i_R[:3, 3]
        
        # 计算变换统计（用于验证）
        R_i_L = np.array(seg_L["rotation_3x3"]) if "rotation_3x3" in seg_L else T_i_L[:3, :3]
        t_i_L = np.array(seg_L["translation"]) if "translation" in seg_L else T_i_L[:3, 3]
        
        angle_L = np.degrees(np.arccos(np.clip((np.trace(R_i_L) - 1) / 2, -1, 1)))
        angle_R = np.degrees(np.arccos(np.clip((np.trace(R_i_R) - 1) / 2, -1, 1)))
        
        rotation_angle_diffs.append(abs(angle_R - angle_L))
        translation_diffs.append(np.linalg.norm(t_i_R - t_i_L))
        
        # 构建分段数据
        seg_R = {
            "index": idx,
            "transform_4x4": T_i_R.tolist(),
            "rotation_3x3": R_i_R.tolist(),
            "translation": t_i_R.tolist(),
        }
        
        # 根据模式传递对应字段
        if "center_frame" in seg_L:
            # 序列号对应模式
            seg_R["center_frame"] = seg_L["center_frame"]
            seg_R["frame_range"] = seg_L["frame_range"]
        if "arc_length_center" in seg_L:
            # 弧长对应模式
            seg_R["arc_length_center"] = seg_L["arc_length_center"]
            seg_R["arc_length_range"] = seg_L["arc_length_range"]
        # 通用可选字段
        if "normalized_arc_center" in seg_L:
            seg_R["normalized_arc_center"] = seg_L["normalized_arc_center"]
            seg_R["normalized_arc_range"] = seg_L["normalized_arc_range"]
        if "refine_depth" in seg_L:
            seg_R["refine_depth"] = seg_L["refine_depth"]
        if "segment_rmse" in seg_L:
            seg_R["segment_rmse"] = seg_L["segment_rmse"]
        
        seg_data_R["segments"].append(seg_R)
    
    if verbose:
        print(f"\n  转换完成: {num_segments} 段")
        print(f"  旋转角度变化统计:")
        print(f"    平均: {np.mean(rotation_angle_diffs):.6f}°")
        print(f"    最大: {np.max(rotation_angle_diffs):.6f}°")
        print(f"  平移向量变化统计:")
        print(f"    平均: {np.mean(translation_diffs):.4f} mm")
        print(f"    最大: {np.max(translation_diffs):.4f} mm")
    
    # ========== 5. 保存结果 ==========
    if verbose:
        print(f"\n▶ 步骤4: 保存结果")
        print(f"-" * 50)
    
    # 确保输出目录存在
    output_dir = os.path.dirname(output_json_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    with open(output_json_path, 'w', encoding='utf-8') as f:
        json.dump(seg_data_R, f, indent=2, ensure_ascii=False)
    
    if verbose:
        print(f"  输出文件: {output_json_path}")
        print(f"\n{'='*70}")
        print("✓ 转换完成！")
        print(f"{'='*70}")
        print(f"\n【使用说明】")
        print(f"  转换后的分段变换可用于机械臂坐标系下的轨迹配准。")
        print(f"  使用 apply_transform.py 应用变换:")
        print(f"    python apply_transform.py <tcp_trajectory.csv> <output.csv>")
        print(f"    (需要修改 DEFAULT_TRANSFORM_PATH 指向新文件)")
    
    # 返回结果和实际使用的手眼矩阵
    return seg_data_R, T_L_to_R


# ============================================================================
#              ⭐⭐⭐ MLS变换坐标系转换
# ============================================================================

def convert_mls_transform(
    hand_eye_matrix: np.ndarray,
    input_json_path: str,
    output_json_path: str,
    convert_translation_to_mm: bool = True,
    verbose: bool = True
) -> tuple:
    """
    将灯塔坐标系下的MLS配准变换转换到机械臂坐标系
    
    核心公式:
        网格变换: T_grid_R = T_L^R @ T_grid_L @ (T_L^R)^{-1}
        训练点:   p_R = T_L^R @ p_L
    
    参数:
        hand_eye_matrix: 手眼变换矩阵 T_L^R (4×4)，灯塔→机械臂
        input_json_path: 灯塔坐标系下MLS变换JSON文件路径
        output_json_path: 输出的机械臂坐标系MLS变换JSON文件路径
        convert_translation_to_mm: 如果手眼矩阵平移为米，是否转换为毫米
        verbose: 是否打印详细信息
    
    返回:
        (转换后的MLS数据字典, 实际使用的手眼矩阵)
    """
    import copy
    
    if verbose:
        print(f"\n{'='*70}")
        print("MLS配准变换坐标系转换工具")
        print(f"{'='*70}")
        print(f"\n核心公式:")
        print(f"  网格变换: T_grid_R = T_L^R @ T_grid_L @ (T_L^R)^{{-1}}")
        print(f"  训练点:   p_R = T_L^R @ p_L")
    
    # ========== 1. 处理手眼变换矩阵 ==========
    if verbose:
        print(f"\n▶ 步骤1: 处理手眼变换矩阵 T_L^R (灯塔→机械臂)")
        print(f"-" * 50)
    
    T_L_to_R = hand_eye_matrix.copy()
    
    # 检查平移单位
    translation = T_L_to_R[:3, 3]
    if convert_translation_to_mm and np.max(np.abs(translation)) < 10:
        T_L_to_R[:3, 3] = translation * 1000.0
        if verbose:
            print(f"  ⚠ 检测到平移值较小，已从米转换为毫米")
            print(f"    原始: [{translation[0]:.6f}, {translation[1]:.6f}, {translation[2]:.6f}] m")
            print(f"    转换后: [{T_L_to_R[0,3]:.3f}, {T_L_to_R[1,3]:.3f}, {T_L_to_R[2,3]:.3f}] mm")
    
    T_R_to_L = np.linalg.inv(T_L_to_R)
    
    R_he = T_L_to_R[:3, :3]
    t_he = T_L_to_R[:3, 3]
    angle_he = np.degrees(np.arccos(np.clip((np.trace(R_he) - 1) / 2, -1, 1)))
    
    if verbose:
        print(f"\n  手眼变换矩阵 T_L^R:")
        for row in T_L_to_R:
            print(f"    [{row[0]:12.6f}, {row[1]:12.6f}, {row[2]:12.6f}, {row[3]:12.6f}]")
        print(f"  旋转角度: {angle_he:.4f}°")
        print(f"  平移向量: [{t_he[0]:.3f}, {t_he[1]:.3f}, {t_he[2]:.3f}] mm")
    
    # ========== 2. 加载灯塔坐标系下的MLS变换 ==========
    if verbose:
        print(f"\n▶ 步骤2: 加载灯塔坐标系MLS变换")
        print(f"-" * 50)
        print(f"  输入文件: {input_json_path}")
    
    if not os.path.exists(input_json_path):
        raise FileNotFoundError(f"找不到MLS输入文件: {input_json_path}")
    
    with open(input_json_path, 'r', encoding='utf-8') as f:
        mls_data_L = json.load(f)
    
    bandwidth = mls_data_L["bandwidth"]
    total_arc_length = mls_data_L["total_arc_length"]
    num_training = mls_data_L.get("num_training_points", 0)
    
    if verbose:
        print(f"  模式: MLS移动最小二乘")
        print(f"  带宽: h={bandwidth:.4f}")
        print(f"  总弧长: {total_arc_length:.2f} mm")
        print(f"  训练点数: {num_training}")
    
    # ========== 3. 转换网格变换矩阵 + 训练数据 ==========
    if verbose:
        print(f"\n▶ 步骤3: 转换网格变换矩阵 + 训练数据")
        print(f"-" * 50)
        print(f"  应用公式: T_grid_R = T_L^R @ T_grid_L @ (T_L^R)^{{-1}}")
    
    mls_data_R = copy.deepcopy(mls_data_L)
    
    # 添加坐标系信息
    mls_data_R["description"] = "机械臂基座坐标系下的MLS配准变换（从灯塔坐标系转换）"
    mls_data_R["coordinate_frame"] = "robot_base"
    mls_data_R["original_coordinate_frame"] = "lighthouse"
    mls_data_R["conversion_formula"] = "T_grid_R = T_L^R @ T_grid_L @ (T_L^R)^{-1}"
    mls_data_R["hand_eye_transform"] = {
        "direction": "lighthouse_to_robot (T_L^R)",
        "matrix_4x4": T_L_to_R.tolist(),
        "rotation_angle_deg": float(angle_he),
        "translation_mm": t_he.tolist()
    }
    
    # 3a. 转换网格变换矩阵（相似变换）
    if "grid" in mls_data_L:
        grid_transforms_L = np.array(mls_data_L["grid"]["grid_transforms"])
        grid_size = len(grid_transforms_L)
        
        grid_transforms_R = np.zeros_like(grid_transforms_L)
        for i in range(grid_size):
            grid_transforms_R[i] = T_L_to_R @ grid_transforms_L[i] @ T_R_to_L
        
        mls_data_R["grid"]["grid_transforms"] = grid_transforms_R.tolist()
        
        if verbose:
            t_diffs = []
            for i in range(grid_size):
                t_L = grid_transforms_L[i][:3, 3]
                t_R = grid_transforms_R[i][:3, 3]
                t_diffs.append(np.linalg.norm(t_R - t_L))
            print(f"\n  网格变换转换完成: {grid_size} 个")
            print(f"  平移向量变化统计:")
            print(f"    平均: {np.mean(t_diffs):.4f} mm")
            print(f"    最大: {np.max(t_diffs):.4f} mm")
    
    # 3b. 转换训练源点和目标点: p_R = R_he @ p_L + t_he
    if "training_data" in mls_data_L:
        source_L = np.array(mls_data_L["training_data"]["source_points"])
        target_L = np.array(mls_data_L["training_data"]["target_points"])
        
        source_R = (R_he @ source_L.T).T + t_he
        target_R = (R_he @ target_L.T).T + t_he
        
        mls_data_R["training_data"]["source_points"] = source_R.tolist()
        mls_data_R["training_data"]["target_points"] = target_R.tolist()
        # 归一化弧长不变（刚性变换保距）
        
        if verbose:
            print(f"\n  训练数据转换完成: {len(source_L)} 点")
            print(f"  灯塔坐标系源点范围:")
            print(f"    X: [{source_L[:,0].min():.1f}, {source_L[:,0].max():.1f}]")
            print(f"    Y: [{source_L[:,1].min():.1f}, {source_L[:,1].max():.1f}]")
            print(f"    Z: [{source_L[:,2].min():.1f}, {source_L[:,2].max():.1f}]")
            print(f"  机械臂坐标系源点范围:")
            print(f"    X: [{source_R[:,0].min():.1f}, {source_R[:,0].max():.1f}]")
            print(f"    Y: [{source_R[:,1].min():.1f}, {source_R[:,1].max():.1f}]")
            print(f"    Z: [{source_R[:,2].min():.1f}, {source_R[:,2].max():.1f}]")
    
    # ========== 4. 保存结果 ==========
    if verbose:
        print(f"\n▶ 步骤4: 保存结果")
        print(f"-" * 50)
    
    output_dir = os.path.dirname(output_json_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    with open(output_json_path, 'w', encoding='utf-8') as f:
        json.dump(mls_data_R, f, indent=2, ensure_ascii=False)
    
    file_size_kb = os.path.getsize(output_json_path) / 1024
    
    if verbose:
        print(f"  输出文件: {output_json_path}")
        print(f"  文件大小: {file_size_kb:.1f}KB")
        print(f"\n{'='*70}")
        print("✓ MLS变换坐标系转换完成！")
        print(f"{'='*70}")
        print(f"\n【使用说明】")
        print(f"  转换后的MLS变换可用于机械臂坐标系下的轨迹配准。")
        print(f"  请将 apply_transform.py 中的 DEFAULT_TRANSFORM_PATH 指向:")
        print(f"    {output_json_path}")
    
    return mls_data_R, T_L_to_R


# ============================================================================
#                     验证函数
# ============================================================================

def verify_transform_conversion(
    hand_eye_matrix_used: np.ndarray,
    seg_data_L: dict,
    seg_data_R: dict,
    verbose: bool = True
) -> bool:
    """
    验证分段变换转换的正确性
    
    参数:
        hand_eye_matrix_used: 实际用于转换的手眼矩阵（已做单位转换）
        seg_data_L: 灯塔坐标系下的分段变换
        seg_data_R: 机械臂坐标系下的分段变换
    
    数学原理: T_i^R = T_L^R @ T_i^L @ (T_L^R)^{-1}
    """
    if verbose:
        print(f"\n{'='*70}")
        print("验证转换正确性")
        print(f"{'='*70}")
    
    T_L_to_R = hand_eye_matrix_used  # 使用实际转换时用的矩阵
    T_R_to_L = np.linalg.inv(T_L_to_R)
    
    # 验证每个分段的相似变换关系
    errors = []
    for seg_L, seg_R in zip(seg_data_L["segments"], seg_data_R["segments"]):
        T_i_L = np.array(seg_L["transform_4x4"])
        T_i_R = np.array(seg_R["transform_4x4"])
        
        # 重新计算应该得到的 T_i_R
        T_i_R_expected = T_L_to_R @ T_i_L @ T_R_to_L
        
        # 计算误差
        error = np.max(np.abs(T_i_R - T_i_R_expected))
        errors.append(error)
    
    max_error = np.max(errors)
    mean_error = np.mean(errors)
    
    if verbose:
        print(f"\n  相似变换验证:")
        print(f"    最大误差: {max_error:.2e}")
        print(f"    平均误差: {mean_error:.2e}")
        
        if max_error < 1e-10:
            print(f"\n  ✓ 验证通过: 所有分段变换转换正确")
        else:
            print(f"\n  ⚠️ 警告: 存在数值误差，请检查")
    
    return max_error < 1e-10


# ============================================================================
#                     交互模式
# ============================================================================

def interactive_mode():
    """交互模式"""
    print(f"\n{'='*70}")
    print("分段配准变换坐标系转换工具 - 交互模式")
    print(f"{'='*70}")
    print("\n核心公式: T_i^R = T_L^R @ T_i^L @ (T_L^R)^{-1}")
    print("  T_L^R: 手眼变换矩阵（灯塔→机械臂）")
    print("  T_i^L: 灯塔坐标系下第i段的配准变换")
    print("  T_i^R: 机械臂坐标系下第i段的配准变换")
    
    # ===== 输入1: 手眼变换矩阵 =====
    print(f"\n{'─'*70}")
    print("【输入1】手眼变换矩阵 T_L^R (灯塔→机械臂)")
    print("─"*70)
    print("\n选择输入方式:")
    print("  1. 使用默认矩阵（从标定结果）")
    print("  2. 从文件加载（.json/.txt/.npy）")
    print("  3. 手动输入4x4矩阵")
    print("  4. 粘贴手眼标定输出文本")
    
    choice = input("\n请选择 [1-4]，直接回车使用默认: ").strip()
    
    if choice == "" or choice == "1":
        hand_eye_matrix = DEFAULT_HAND_EYE_MATRIX.copy()
        print(f"\n  使用默认手眼变换矩阵:")
        for row in hand_eye_matrix:
            print(f"    [{row[0]:12.6f}, {row[1]:12.6f}, {row[2]:12.6f}, {row[3]:12.6f}]")
    
    elif choice == "2":
        filepath = input("请输入文件路径: ").strip()
        if not filepath:
            print("  路径为空，使用默认矩阵")
            hand_eye_matrix = DEFAULT_HAND_EYE_MATRIX.copy()
        else:
            try:
                hand_eye_matrix = load_hand_eye_from_file(filepath)
                print(f"  ✓ 成功从文件加载")
            except Exception as e:
                print(f"  ✗ 加载失败: {e}")
                print("  使用默认矩阵")
                hand_eye_matrix = DEFAULT_HAND_EYE_MATRIX.copy()
    
    elif choice == "3":
        print("请输入4x4矩阵（每行4个数字，用空格或逗号分隔）:")
        matrix_rows = []
        for i in range(4):
            row_str = input(f"  第{i+1}行: ").strip()
            row = [float(x) for x in re.split(r'[,\s]+', row_str) if x]
            if len(row) != 4:
                print(f"    ⚠ 需要4个数字，得到{len(row)}个，使用默认矩阵")
                hand_eye_matrix = DEFAULT_HAND_EYE_MATRIX.copy()
                break
            matrix_rows.append(row)
        else:
            hand_eye_matrix = np.array(matrix_rows)
    
    elif choice == "4":
        print("请粘贴手眼标定输出文本（输入空行结束）:")
        lines = []
        while True:
            line = input()
            if line.strip() == "":
                break
            lines.append(line)
        text = "\n".join(lines)
        
        try:
            hand_eye_matrix = parse_hand_eye_from_text(text)
        except Exception as e:
            print(f"  ✗ 解析失败: {e}")
            print("  使用默认矩阵")
            hand_eye_matrix = DEFAULT_HAND_EYE_MATRIX.copy()
    
    else:
        print("  无效选择，使用默认矩阵")
        hand_eye_matrix = DEFAULT_HAND_EYE_MATRIX.copy()
    
    # ===== 输入2: 灯塔坐标系分段变换文件 =====
    print(f"\n{'─'*70}")
    print("【输入2】灯塔坐标系下分段配准变换JSON文件")
    print("─"*70)
    print(f"\n默认文件: {DEFAULT_INPUT_JSON}")
    
    input_json = input("请输入文件路径（直接回车使用默认）: ").strip()
    if not input_json:
        input_json = DEFAULT_INPUT_JSON
        print(f"  使用默认: {input_json}")
    
    if not os.path.exists(input_json):
        print(f"  ✗ 文件不存在: {input_json}")
        return
    
    # ===== 输出文件 =====
    print(f"\n{'─'*70}")
    print("【输出】机械臂坐标系下分段配准变换JSON文件")
    print("─"*70)
    
    # 自动生成默认输出路径
    input_dir = os.path.dirname(input_json)
    input_name = os.path.basename(input_json)
    default_output = os.path.join(input_dir, input_name.replace('.json', '_robot.json'))
    
    print(f"\n默认输出: {default_output}")
    output_json = input("请输入输出路径（直接回车使用默认）: ").strip()
    if not output_json:
        output_json = default_output
        print(f"  使用默认: {output_json}")
    
    # ===== 执行转换 =====
    try:
        result, T_L_to_R_used = convert_segmented_transform(
            hand_eye_matrix=hand_eye_matrix,
            input_json_path=input_json,
            output_json_path=output_json,
            convert_translation_to_mm=True,
            verbose=True
        )
        
        # 验证（使用实际转换时用的手眼矩阵）
        with open(input_json, 'r', encoding='utf-8') as f:
            seg_data_L = json.load(f)
        
        verify_transform_conversion(T_L_to_R_used, seg_data_L, result, verbose=True)
        
    except Exception as e:
        print(f"\n✗ 转换失败: {e}")
        import traceback
        traceback.print_exc()


# ============================================================================
#                     命令行接口
# ============================================================================

def main():
    """主函数 - 自动检测分段Kabsch和MLS两种变换模式"""
    
    if len(sys.argv) == 1:
        # 无参数，自动检测模式运行
        
        # ⭐ 优先检测MLS变换文件
        if os.path.exists(DEFAULT_MLS_INPUT_JSON):
            with open(DEFAULT_MLS_INPUT_JSON, 'r', encoding='utf-8') as f:
                peek_data = json.load(f)
            
            if peek_data.get("mode") == "mls":
                print("\n⭐ 检测到MLS变换文件，自动使用MLS模式...")
                print(f"  输入: {DEFAULT_MLS_INPUT_JSON}")
                print(f"  输出: {DEFAULT_MLS_OUTPUT_JSON}")
                print(f"  手眼矩阵: 使用内置默认矩阵\n")
                
                try:
                    result, T_L_to_R_used = convert_mls_transform(
                        hand_eye_matrix=DEFAULT_HAND_EYE_MATRIX.copy(),
                        input_json_path=DEFAULT_MLS_INPUT_JSON,
                        output_json_path=DEFAULT_MLS_OUTPUT_JSON,
                        convert_translation_to_mm=True,
                        verbose=True
                    )
                except Exception as e:
                    print(f"\n✗ MLS转换失败: {e}")
                    import traceback
                    traceback.print_exc()
                return
        
        # 分段Kabsch模式
        print("\n使用默认配置自动运行（分段Kabsch模式）...")
        print(f"  输入: {DEFAULT_INPUT_JSON}")
        print(f"  输出: {DEFAULT_OUTPUT_JSON}")
        print(f"  手眼矩阵: 使用内置默认矩阵\n")
        
        try:
            result, T_L_to_R_used = convert_segmented_transform(
                hand_eye_matrix=DEFAULT_HAND_EYE_MATRIX.copy(),
                input_json_path=DEFAULT_INPUT_JSON,
                output_json_path=DEFAULT_OUTPUT_JSON,
                convert_translation_to_mm=True,
                verbose=True
            )
            
            # 验证
            with open(DEFAULT_INPUT_JSON, 'r', encoding='utf-8') as f:
                seg_data_L = json.load(f)
            verify_transform_conversion(T_L_to_R_used, seg_data_L, result, verbose=True)
            
        except Exception as e:
            print(f"\n✗ 转换失败: {e}")
            print("\n提示: 使用 --interactive 进入交互模式")
            print("      使用 -h 查看完整帮助")
            import traceback
            traceback.print_exc()
    
    elif len(sys.argv) >= 2 and sys.argv[1] == '--interactive':
        # 交互模式
        interactive_mode()
    
    elif len(sys.argv) >= 3:
        # 命令行模式: python script.py <input.json> <output.json> [hand_eye_file]
        input_json = sys.argv[1]
        output_json = sys.argv[2]
        
        if len(sys.argv) >= 4:
            hand_eye_file = sys.argv[3]
            hand_eye_matrix = load_hand_eye_from_file(hand_eye_file)
        else:
            hand_eye_matrix = DEFAULT_HAND_EYE_MATRIX.copy()
        
        # ⭐ 自动检测MLS模式
        with open(input_json, 'r', encoding='utf-8') as f:
            peek_data = json.load(f)
        
        if peek_data.get("mode") == "mls":
            convert_mls_transform(
                hand_eye_matrix=hand_eye_matrix,
                input_json_path=input_json,
                output_json_path=output_json,
                verbose=True
            )
        else:
            convert_segmented_transform(
                hand_eye_matrix=hand_eye_matrix,
                input_json_path=input_json,
                output_json_path=output_json,
                verbose=True
            )
    
    elif sys.argv[1] in ['-h', '--help']:
        print(__doc__)
        print("\n命令行用法:")
        print("  python convert_segmented_transform.py")
        print("      → 自动检测模式运行（优先MLS，其次分段Kabsch）")
        print("")
        print("  python convert_segmented_transform.py --interactive")
        print("      → 进入交互模式")
        print("")
        print("  python convert_segmented_transform.py <input.json> <output.json>")
        print("      → 自动检测输入模式（MLS/分段Kabsch）转换")
        print("")
        print("  python convert_segmented_transform.py <input.json> <output.json> <hand_eye.json>")
        print("      → 使用指定手眼矩阵文件转换")
        print("")
        print("示例:")
        print("  python convert_segmented_transform.py")
        print("  python convert_segmented_transform.py mls_transform.json mls_transform_robot.json")
        print("  python convert_segmented_transform.py segmented_transform.json segmented_transform_robot.json")
    
    else:
        print("参数错误，使用 -h 查看帮助")


if __name__ == "__main__":
    main()

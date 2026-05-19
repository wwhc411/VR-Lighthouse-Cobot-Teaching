"""
RDP 关键帧提取器
================

基于 Ramer-Douglas-Peucker 算法提取轨迹关键帧，
并使用线性插值生成简化后的线段轨迹。

功能：
1. 从CSV读取原始轨迹数据
2. 使用RDP算法（位置优先 + 姿态校验）提取关键帧
3. 输出关键帧CSV
4. 使用线性插值生成完整的线段轨迹CSV

作者: GitHub Copilot
日期: 2025-12-26
"""

import numpy as np
import pandas as pd
from typing import List, Tuple
from pathlib import Path
from scipy.spatial.transform import Rotation, Slerp
import time


class RDPKeyframeExtractor:
    """RDP 关键帧提取器"""
    
    def __init__(
        self,
        position_epsilon: float = 0.3,      # 位置阈值 (mm)
        rotation_epsilon: float = 5.0,      # 姿态阈值 (度)
        min_keyframe_interval: int = 2,     # 最小关键帧间隔
        max_keyframe_interval: int = 50     # 最大关键帧间隔
    ):
        self.position_epsilon = position_epsilon
        self.rotation_epsilon = rotation_epsilon
        self.min_keyframe_interval = min_keyframe_interval
        self.max_keyframe_interval = max_keyframe_interval
        
        # 统计信息
        self.original_frame_count = 0
        self.keyframe_count = 0
        self.compression_ratio = 0.0
        self.extraction_time_ms = 0.0
    
    def load_csv_data(self, csv_path: str) -> pd.DataFrame:
        """加载CSV轨迹数据"""
        df = pd.read_csv(csv_path)
        self.original_frame_count = len(df)
        print(f"[RDP] 加载CSV: {csv_path}")
        print(f"[RDP] 原始帧数: {self.original_frame_count}")
        return df
    
    def point_to_line_distance(
        self, 
        point: np.ndarray, 
        line_start: np.ndarray, 
        line_end: np.ndarray
    ) -> float:
        """
        计算点到线段的垂直距离
        使用叉乘公式: d = |AB × AP| / |AB|
        """
        line_vec = line_end - line_start
        point_vec = point - line_start
        
        line_length = np.linalg.norm(line_vec)
        
        if line_length < 1e-6:
            # 线段退化为点
            return np.linalg.norm(point_vec)
        
        # 三维叉乘
        cross = np.cross(line_vec, point_vec)
        return np.linalg.norm(cross) / line_length
    
    def rdp_recursive(
        self, 
        positions: np.ndarray, 
        start_idx: int, 
        end_idx: int, 
        epsilon: float
    ) -> List[int]:
        """
        RDP 递归算法核心
        
        Args:
            positions: 位置数组 (N, 3)
            start_idx: 起始索引
            end_idx: 结束索引
            epsilon: 距离阈值
            
        Returns:
            关键帧索引列表
        """
        # 基础情况：点数太少，无法简化
        if end_idx - start_idx < 2:
            result = [start_idx]
            if end_idx != start_idx:
                result.append(end_idx)
            return result
        
        # 找到距离首尾连线最远的点
        max_distance = 0.0
        max_index = start_idx
        
        line_start = positions[start_idx]
        line_end = positions[end_idx]
        
        for i in range(start_idx + 1, end_idx):
            distance = self.point_to_line_distance(positions[i], line_start, line_end)
            if distance > max_distance:
                max_distance = distance
                max_index = i
        
        # 判断是否需要分割
        if max_distance > epsilon:
            # 递归处理左右两段
            left_result = self.rdp_recursive(positions, start_idx, max_index, epsilon)
            right_result = self.rdp_recursive(positions, max_index, end_idx, epsilon)
            
            # 合并结果（去除重复的分割点）
            left_result.pop()  # 移除左段的最后一个点（与右段第一个点重复）
            left_result.extend(right_result)
            return left_result
        else:
            # 整段简化为直线，只保留首尾
            return [start_idx, end_idx]
    
    def quaternion_angle(self, q1: np.ndarray, q2: np.ndarray) -> float:
        """
        计算两个四元数之间的角度差（度）
        
        四元数格式: [qx, qy, qz, qw]
        """
        # 归一化
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # 四元数点积
        dot = np.abs(np.dot(q1, q2))
        dot = np.clip(dot, -1.0, 1.0)
        
        # 角度 = 2 * arccos(|dot|)
        angle_rad = 2.0 * np.arccos(dot)
        return np.degrees(angle_rad)
    
    def validate_rotation_gaps(
        self, 
        keyframes: List[int], 
        quaternions: np.ndarray,
        rotation_epsilon_deg: float
    ) -> List[int]:
        """
        检查关键帧间的姿态变化，必要时插入额外关键帧
        
        使用滚动参考帧策略：
        - 从当前关键帧开始检查
        - 超过阈值时插入新关键帧并更新参考点
        """
        result = []
        
        for i in range(len(keyframes) - 1):
            current = keyframes[i]
            next_kf = keyframes[i + 1]
            
            result.append(current)
            
            # 参考姿态（滚动更新）
            ref_rotation = quaternions[current]
            
            for j in range(current + 1, next_kf):
                # 计算相对于参考点的累积姿态变化
                angle_from_ref = self.quaternion_angle(ref_rotation, quaternions[j])
                
                if angle_from_ref > rotation_epsilon_deg:
                    # 插入额外关键帧
                    result.append(j)
                    # 更新参考点
                    ref_rotation = quaternions[j]
        
        # 添加最后一个关键帧
        result.append(keyframes[-1])
        
        # 去重并排序
        result = sorted(list(set(result)))
        return result
    
    def apply_interval_constraints(
        self, 
        keyframes: List[int], 
        min_interval: int, 
        max_interval: int,
        total_frames: int
    ) -> List[int]:
        """
        应用最小/最大间隔约束
        """
        result = [keyframes[0]]
        
        for i in range(1, len(keyframes)):
            last_added = result[-1]
            current = keyframes[i]
            gap = current - last_added
            
            # 最小间隔检查
            if gap < min_interval and i != len(keyframes) - 1:
                # 间隔太小，跳过（除非是最后一帧）
                continue
            
            # 最大间隔检查
            if gap > max_interval:
                # 间隔太大，需要插入中间点
                insert_count = (gap - 1) // max_interval
                insert_interval = gap // (insert_count + 1)
                
                for j in range(1, insert_count + 1):
                    result.append(last_added + j * insert_interval)
            
            result.append(current)
        
        # 去重并排序
        result = sorted(list(set(result)))
        return result
    
    def extract_keyframes(self, df: pd.DataFrame) -> Tuple[List[int], pd.DataFrame]:
        """
        从DataFrame提取关键帧
        
        Returns:
            (关键帧索引列表, 关键帧DataFrame)
        """
        start_time = time.time()
        
        # 提取位置和四元数
        positions = df[['X_mm', 'Y_mm', 'Z_mm']].values
        quaternions = df[['QX', 'QY', 'QZ', 'QW']].values
        
        frame_count = len(df)
        
        # Step 1: 位置RDP
        print(f"\n[RDP] Step 1: 位置RDP (ε_pos = {self.position_epsilon} mm)")
        position_keyframes = self.rdp_recursive(
            positions, 0, frame_count - 1, self.position_epsilon
        )
        print(f"  位置RDP后关键帧数: {len(position_keyframes)}")
        
        # Step 2: 姿态校验
        print(f"\n[RDP] Step 2: 姿态校验 (ε_rot = {self.rotation_epsilon}°)")
        rotation_checked_keyframes = self.validate_rotation_gaps(
            position_keyframes, quaternions, self.rotation_epsilon
        )
        print(f"  姿态校验后关键帧数: {len(rotation_checked_keyframes)}")
        
        # Step 3: 间隔约束
        print(f"\n[RDP] Step 3: 间隔约束 (min={self.min_keyframe_interval}, max={self.max_keyframe_interval})")
        final_keyframes = self.apply_interval_constraints(
            rotation_checked_keyframes,
            self.min_keyframe_interval,
            self.max_keyframe_interval,
            frame_count
        )
        print(f"  间隔约束后关键帧数: {len(final_keyframes)}")
        
        # 更新统计信息
        self.keyframe_count = len(final_keyframes)
        self.compression_ratio = 1.0 - self.keyframe_count / self.original_frame_count
        self.extraction_time_ms = (time.time() - start_time) * 1000
        
        # 提取关键帧数据
        keyframe_df = df.iloc[final_keyframes].copy()
        keyframe_df['KeyframeIndex'] = range(len(final_keyframes))
        keyframe_df['OriginalFrameIndex'] = final_keyframes
        
        # 添加插入原因标记
        reasons = []
        pos_kf_set = set(position_keyframes)
        rot_kf_set = set(rotation_checked_keyframes) - pos_kf_set
        
        for idx in final_keyframes:
            if idx == 0:
                reasons.append('FirstFrame')
            elif idx == frame_count - 1:
                reasons.append('LastFrame')
            elif idx in pos_kf_set:
                reasons.append('PositionRDP')
            elif idx in rot_kf_set:
                reasons.append('RotationCheck')
            else:
                reasons.append('IntervalConstraint')
        
        keyframe_df['InsertReason'] = reasons
        
        print(f"\n[RDP] ===== 提取完成 =====")
        print(f"  原始帧数: {self.original_frame_count}")
        print(f"  关键帧数: {self.keyframe_count}")
        print(f"  压缩率: {self.compression_ratio:.1%}")
        print(f"  耗时: {self.extraction_time_ms:.1f} ms")
        
        return final_keyframes, keyframe_df
    
    def interpolate_trajectory(
        self, 
        df: pd.DataFrame, 
        keyframe_indices: List[int]
    ) -> pd.DataFrame:
        """
        使用线性插值在关键帧之间生成完整的线段轨迹
        
        Args:
            df: 原始轨迹DataFrame
            keyframe_indices: 关键帧索引列表
            
        Returns:
            插值后的完整轨迹DataFrame
        """
        print(f"\n[RDP] 生成线段轨迹 (线性插值)...")
        
        total_frames = len(df)
        
        # 准备关键帧数据
        kf_positions = df.iloc[keyframe_indices][['X_mm', 'Y_mm', 'Z_mm']].values
        kf_quaternions = df.iloc[keyframe_indices][['QX', 'QY', 'QZ', 'QW']].values
        kf_rotations_rv = df.iloc[keyframe_indices][['RX_rad', 'RY_rad', 'RZ_rad']].values
        kf_tcp_pos = df.iloc[keyframe_indices][['TCP_X', 'TCP_Y', 'TCP_Z']].values
        kf_tcp_rot = df.iloc[keyframe_indices][['TCP_RX', 'TCP_RY', 'TCP_RZ']].values
        kf_times = df.iloc[keyframe_indices]['TimeFromStart_s'].values
        
        # 创建结果数组
        interpolated_positions = np.zeros((total_frames, 3))
        interpolated_quaternions = np.zeros((total_frames, 4))
        interpolated_rotations_rv = np.zeros((total_frames, 3))
        interpolated_tcp_pos = np.zeros((total_frames, 3))
        interpolated_tcp_rot = np.zeros((total_frames, 3))
        
        # 逐段线性插值
        for k in range(len(keyframe_indices) - 1):
            start_idx = keyframe_indices[k]
            end_idx = keyframe_indices[k + 1]
            
            # 获取起点和终点数据
            pos_start = kf_positions[k]
            pos_end = kf_positions[k + 1]
            
            quat_start = kf_quaternions[k]
            quat_end = kf_quaternions[k + 1]
            
            rv_start = kf_rotations_rv[k]
            rv_end = kf_rotations_rv[k + 1]
            
            tcp_pos_start = kf_tcp_pos[k]
            tcp_pos_end = kf_tcp_pos[k + 1]
            
            tcp_rot_start = kf_tcp_rot[k]
            tcp_rot_end = kf_tcp_rot[k + 1]
            
            # 对每个中间帧进行插值
            for i in range(start_idx, end_idx + 1):
                if end_idx == start_idx:
                    t = 0.0
                else:
                    t = (i - start_idx) / (end_idx - start_idx)
                
                # 位置线性插值
                interpolated_positions[i] = pos_start + t * (pos_end - pos_start)
                
                # 旋转向量线性插值
                interpolated_rotations_rv[i] = rv_start + t * (rv_end - rv_start)
                
                # TCP位置线性插值
                interpolated_tcp_pos[i] = tcp_pos_start + t * (tcp_pos_end - tcp_pos_start)
                
                # TCP旋转线性插值
                interpolated_tcp_rot[i] = tcp_rot_start + t * (tcp_rot_end - tcp_rot_start)
                
                # 四元数球面线性插值 (Slerp)
                interpolated_quaternions[i] = self.slerp_quaternion(quat_start, quat_end, t)
        
        # 创建插值后的DataFrame
        result_df = df.copy()
        
        result_df['X_mm'] = interpolated_positions[:, 0]
        result_df['Y_mm'] = interpolated_positions[:, 1]
        result_df['Z_mm'] = interpolated_positions[:, 2]
        
        result_df['QX'] = interpolated_quaternions[:, 0]
        result_df['QY'] = interpolated_quaternions[:, 1]
        result_df['QZ'] = interpolated_quaternions[:, 2]
        result_df['QW'] = interpolated_quaternions[:, 3]
        
        result_df['RX_rad'] = interpolated_rotations_rv[:, 0]
        result_df['RY_rad'] = interpolated_rotations_rv[:, 1]
        result_df['RZ_rad'] = interpolated_rotations_rv[:, 2]
        
        result_df['TCP_X'] = interpolated_tcp_pos[:, 0]
        result_df['TCP_Y'] = interpolated_tcp_pos[:, 1]
        result_df['TCP_Z'] = interpolated_tcp_pos[:, 2]
        
        result_df['TCP_RX'] = interpolated_tcp_rot[:, 0]
        result_df['TCP_RY'] = interpolated_tcp_rot[:, 1]
        result_df['TCP_RZ'] = interpolated_tcp_rot[:, 2]
        
        # 添加是否为关键帧的标记
        is_keyframe = np.zeros(total_frames, dtype=int)
        for idx in keyframe_indices:
            is_keyframe[idx] = 1
        result_df['IsKeyframe'] = is_keyframe
        
        print(f"  线段轨迹生成完成: {total_frames} 帧")
        
        return result_df
    
    def slerp_quaternion(self, q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """
        四元数球面线性插值 (Slerp)
        
        Args:
            q1: 起始四元数 [qx, qy, qz, qw]
            q2: 终止四元数 [qx, qy, qz, qw]
            t: 插值参数 [0, 1]
            
        Returns:
            插值后的四元数
        """
        # 归一化
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # 计算点积
        dot = np.dot(q1, q2)
        
        # 如果点积为负，取反q2以选择最短路径
        if dot < 0:
            q2 = -q2
            dot = -dot
        
        # 如果接近，使用线性插值避免数值问题
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        # Slerp
        theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
        theta = theta_0 * t
        
        q2_perp = q2 - q1 * dot
        q2_perp = q2_perp / np.linalg.norm(q2_perp)
        
        return q1 * np.cos(theta) + q2_perp * np.sin(theta)


def main():
    """主函数"""
    # 配置参数
    input_csv = r"C:\Users\15421\Desktop\lighthouse_12.2\Assets\StreamingAssets\TrackerRecordings\HighFreq_7_20251218_223754_filtered_Combined.csv"
    
    # 输出文件路径
    output_dir = Path(input_csv).parent
    base_name = Path(input_csv).stem
    
    keyframe_csv = output_dir / f"{base_name}_RDP_keyframes.csv"
    segment_csv = output_dir / f"{base_name}_RDP_segments.csv"
    
    # RDP参数（高精度模式）
    position_epsilon = 0.3    # mm
    rotation_epsilon = 5.0    # 度
    min_interval = 2
    max_interval = 50
    
    print("=" * 60)
    print("RDP 关键帧提取器")
    print("=" * 60)
    print(f"\n输入文件: {input_csv}")
    print(f"\n参数配置:")
    print(f"  位置阈值 ε_pos: {position_epsilon} mm")
    print(f"  姿态阈值 ε_rot: {rotation_epsilon}°")
    print(f"  最小间隔: {min_interval} 帧")
    print(f"  最大间隔: {max_interval} 帧")
    
    # 创建提取器
    extractor = RDPKeyframeExtractor(
        position_epsilon=position_epsilon,
        rotation_epsilon=rotation_epsilon,
        min_keyframe_interval=min_interval,
        max_keyframe_interval=max_interval
    )
    
    # 加载数据
    df = extractor.load_csv_data(input_csv)
    
    # 提取关键帧
    keyframe_indices, keyframe_df = extractor.extract_keyframes(df)
    
    # 生成线段轨迹
    segment_df = extractor.interpolate_trajectory(df, keyframe_indices)
    
    # 保存关键帧CSV
    # 重新排列列顺序
    keyframe_columns = [
        'KeyframeIndex', 'OriginalFrameIndex', 'InsertReason',
        'FrameNumber', 'TimeStamp_ms', 'TimeFromStart_s',
        'X_mm', 'Y_mm', 'Z_mm',
        'QX', 'QY', 'QZ', 'QW',
        'RX_rad', 'RY_rad', 'RZ_rad',
        'TCP_X', 'TCP_Y', 'TCP_Z',
        'TCP_RX', 'TCP_RY', 'TCP_RZ'
    ]
    keyframe_df = keyframe_df[keyframe_columns]
    keyframe_df.to_csv(keyframe_csv, index=False, float_format='%.6f')
    print(f"\n[输出] 关键帧CSV: {keyframe_csv}")
    
    # 保存线段轨迹CSV
    segment_df.to_csv(segment_csv, index=False, float_format='%.6f')
    print(f"[输出] 线段轨迹CSV: {segment_csv}")
    
    # 打印关键帧统计
    print("\n" + "=" * 60)
    print("关键帧分布统计")
    print("=" * 60)
    reason_counts = keyframe_df['InsertReason'].value_counts()
    for reason, count in reason_counts.items():
        print(f"  {reason}: {count}")
    
    # 打印关键帧间隔统计
    intervals = np.diff(keyframe_indices)
    print(f"\n关键帧间隔统计:")
    print(f"  最小间隔: {intervals.min()} 帧")
    print(f"  最大间隔: {intervals.max()} 帧")
    print(f"  平均间隔: {intervals.mean():.1f} 帧")
    
    # 计算轨迹误差（原始曲线点到简化线段的距离）
    print("\n" + "=" * 60)
    print("轨迹误差分析")
    print("=" * 60)
    
    original_positions = df[['X_mm', 'Y_mm', 'Z_mm']].values
    segment_positions = segment_df[['X_mm', 'Y_mm', 'Z_mm']].values
    
    # 方法1: 原始点与插值点的差异（这不是RDP保证的误差）
    point_diff_errors = np.linalg.norm(original_positions - segment_positions, axis=1)
    print(f"  原始点与插值点差异:")
    print(f"    最大: {point_diff_errors.max():.4f} mm")
    print(f"    平均: {point_diff_errors.mean():.4f} mm")
    print(f"    RMS:  {np.sqrt(np.mean(point_diff_errors**2)):.4f} mm")
    
    # 方法2: 原始点到简化线段的垂直距离（RDP保证的误差）
    print(f"\n  原始点到简化线段的垂直距离（RDP误差）:")
    max_perpendicular_error = 0.0
    total_perpendicular_error = 0.0
    count = 0
    
    for k in range(len(keyframe_indices) - 1):
        start_idx = keyframe_indices[k]
        end_idx = keyframe_indices[k + 1]
        line_start = original_positions[start_idx]
        line_end = original_positions[end_idx]
        
        for i in range(start_idx + 1, end_idx):
            point = original_positions[i]
            perp_dist = extractor.point_to_line_distance(point, line_start, line_end)
            max_perpendicular_error = max(max_perpendicular_error, perp_dist)
            total_perpendicular_error += perp_dist
            count += 1
    
    avg_perpendicular_error = total_perpendicular_error / count if count > 0 else 0
    print(f"    最大: {max_perpendicular_error:.4f} mm")
    print(f"    平均: {avg_perpendicular_error:.4f} mm")
    
    # RDP保证：最大垂直距离应该 <= epsilon
    print(f"\n  RDP保证: 最大垂直距离 <= ε_pos = {position_epsilon} mm")
    if max_perpendicular_error <= position_epsilon:
        print(f"  ✓ 验证通过")
    else:
        print(f"  ✗ 警告: 误差超出阈值（间隔约束可能引入误差）")
    
    print("\n" + "=" * 60)
    print("完成!")
    print("=" * 60)


if __name__ == "__main__":
    main()

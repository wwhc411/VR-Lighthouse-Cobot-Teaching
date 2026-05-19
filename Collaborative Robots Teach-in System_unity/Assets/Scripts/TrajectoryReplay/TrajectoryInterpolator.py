"""
轨迹插值脚本
功能：对CSV轨迹数据进行线性插值，确保每1cm欧氏距离至少有一个采样点

使用方法：
    python TrajectoryInterpolator.py arc_1.csv --distance 10 --output arc_1_interpolated.csv

参数说明：
    input_file: 输入CSV文件路径
    --distance: 插值距离阈值(mm)，默认10mm (1cm)
    --output: 输出文件路径，默认为 <input>_interpolated.csv
"""

import csv
import numpy as np
import argparse
from pathlib import Path


def quaternion_slerp(q1, q2, t):
    """
    四元数球面线性插值 (Spherical Linear Interpolation)
    
    Args:
        q1: 起始四元数 [x, y, z, w]
        q2: 目标四元数 [x, y, z, w]
        t: 插值参数 [0, 1]
    
    Returns:
        插值后的四元数
    """
    # 归一化
    q1 = np.array(q1) / np.linalg.norm(q1)
    q2 = np.array(q2) / np.linalg.norm(q2)
    
    # 计算点积
    dot = np.dot(q1, q2)
    
    # 如果点积为负，反转其中一个四元数以取最短路径
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    
    # 如果四元数非常接近，使用线性插值避免数值问题
    if dot > 0.9995:
        result = q1 + t * (q2 - q1)
        return result / np.linalg.norm(result)
    
    # 计算夹角
    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    theta = theta_0 * t
    
    # 计算垂直分量
    q3 = q2 - q1 * dot
    q3 = q3 / np.linalg.norm(q3)
    
    # 球面插值
    return q1 * np.cos(theta) + q3 * np.sin(theta)


def euler_from_quaternion(qx, qy, qz, qw):
    """
    从四元数计算欧拉角（旋转矢量）
    注意：这里使用与Unity/CSV中相同的转换方式
    
    Returns:
        (rx, ry, rz) in radians
    """
    # 转换为旋转矢量（轴角表示）
    # angle = 2 * arccos(qw)
    # axis = (qx, qy, qz) / sin(angle/2)
    # rotation_vector = axis * angle
    
    angle = 2.0 * np.arccos(np.clip(qw, -1.0, 1.0))
    
    if abs(angle) < 1e-10:
        return 0.0, 0.0, 0.0
    
    sin_half_angle = np.sin(angle / 2.0)
    
    if abs(sin_half_angle) < 1e-10:
        return 0.0, 0.0, 0.0
    
    axis_x = qx / sin_half_angle
    axis_y = qy / sin_half_angle
    axis_z = qz / sin_half_angle
    
    # 归一化轴
    axis_length = np.sqrt(axis_x**2 + axis_y**2 + axis_z**2)
    if axis_length > 0:
        axis_x /= axis_length
        axis_y /= axis_length
        axis_z /= axis_length
    
    # 旋转矢量 = 轴 * 角度
    rx = axis_x * angle
    ry = axis_y * angle
    rz = axis_z * angle
    
    return rx, ry, rz


def interpolate_trajectory(input_csv, output_csv, distance_threshold_mm=10.0):
    """
    对轨迹进行插值
    
    Args:
        input_csv: 输入CSV文件路径
        output_csv: 输出CSV文件路径
        distance_threshold_mm: 距离阈值(mm)，默认10mm (1cm)
    """
    # 读取CSV数据
    rows = []
    with open(input_csv, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        for row in reader:
            rows.append(row)
    
    if len(rows) < 2:
        print("错误：CSV文件至少需要2个数据点")
        return
    
    print(f"输入文件: {input_csv}")
    print(f"原始点数: {len(rows)}")
    print(f"距离阈值: {distance_threshold_mm} mm")
    
    # 插值后的数据
    interpolated_rows = []
    
    # 统计信息
    total_distance = 0.0
    max_segment_distance = 0.0
    interpolated_count = 0
    
    for i in range(len(rows)):
        current_row = rows[i]
        
        # 添加当前点
        interpolated_rows.append(current_row)
        
        # 如果不是最后一个点，检查是否需要插值
        if i < len(rows) - 1:
            next_row = rows[i + 1]
            
            # 提取位置 (mm)
            p1 = np.array([
                float(current_row['X_mm']),
                float(current_row['Y_mm']),
                float(current_row['Z_mm'])
            ])
            
            p2 = np.array([
                float(next_row['X_mm']),
                float(next_row['Y_mm']),
                float(next_row['Z_mm'])
            ])
            
            # 计算欧氏距离
            distance = np.linalg.norm(p2 - p1)
            total_distance += distance
            max_segment_distance = max(max_segment_distance, distance)
            
            # 如果距离大于阈值，需要插值
            if distance > distance_threshold_mm:
                # 计算需要插入的点数
                num_segments = int(np.ceil(distance / distance_threshold_mm))
                num_interpolated = num_segments - 1
                
                if num_interpolated > 0:
                    interpolated_count += num_interpolated
                    
                    # 提取四元数
                    q1 = np.array([
                        float(current_row['QX']),
                        float(current_row['QY']),
                        float(current_row['QZ']),
                        float(current_row['QW'])
                    ])
                    
                    q2 = np.array([
                        float(next_row['QX']),
                        float(next_row['QY']),
                        float(next_row['QZ']),
                        float(next_row['QW'])
                    ])
                    
                    # 提取TCP位姿（如果存在）
                    has_tcp = 'TCP_X_m' in current_row and current_row['TCP_X_m']
                    if has_tcp:
                        tcp_p1 = np.array([
                            float(current_row['TCP_X_m']),
                            float(current_row['TCP_Y_m']),
                            float(current_row['TCP_Z_m'])
                        ])
                        tcp_p2 = np.array([
                            float(next_row['TCP_X_m']),
                            float(next_row['TCP_Y_m']),
                            float(next_row['TCP_Z_m'])
                        ])
                        tcp_r1 = np.array([
                            float(current_row['TCP_RX_rad']),
                            float(current_row['TCP_RY_rad']),
                            float(current_row['TCP_RZ_rad'])
                        ])
                        tcp_r2 = np.array([
                            float(next_row['TCP_RX_rad']),
                            float(next_row['TCP_RY_rad']),
                            float(next_row['TCP_RZ_rad'])
                        ])
                    
                    # 插值时间戳
                    time_start = float(current_row['TimeStamp_ms'])
                    time_end = float(next_row['TimeStamp_ms'])
                    
                    time_from_start_begin = float(current_row['TimeFromStart_s'])
                    time_from_start_end = float(next_row['TimeFromStart_s'])
                    
                    # 逐个插值点
                    for j in range(1, num_segments):
                        t = j / num_segments  # 插值参数 [0, 1]
                        
                        # 位置线性插值
                        p_interp = p1 + t * (p2 - p1)
                        
                        # 四元数球面插值
                        q_interp = quaternion_slerp(q1, q2, t)
                        
                        # 计算旋转矢量
                        rx, ry, rz = euler_from_quaternion(
                            q_interp[0], q_interp[1], q_interp[2], q_interp[3]
                        )
                        
                        # 创建插值行
                        interp_row = {
                            'FrameNumber': '',  # 稍后重新编号
                            'TimeStamp_ms': f"{time_start + t * (time_end - time_start):.3f}",
                            'TimeFromStart_s': f"{time_from_start_begin + t * (time_from_start_end - time_from_start_begin):.6f}",
                            'X_mm': f"{p_interp[0]:.3f}",
                            'Y_mm': f"{p_interp[1]:.3f}",
                            'Z_mm': f"{p_interp[2]:.3f}",
                            'QX': f"{q_interp[0]:.6f}",
                            'QY': f"{q_interp[1]:.6f}",
                            'QZ': f"{q_interp[2]:.6f}",
                            'QW': f"{q_interp[3]:.6f}",
                            'RX_rad': f"{rx:.6f}",
                            'RY_rad': f"{ry:.6f}",
                            'RZ_rad': f"{rz:.6f}"
                        }
                        
                        # TCP插值（如果存在）
                        if has_tcp:
                            tcp_p_interp = tcp_p1 + t * (tcp_p2 - tcp_p1)
                            tcp_r_interp = tcp_r1 + t * (tcp_r2 - tcp_r1)
                            
                            interp_row['TCP_X_m'] = f"{tcp_p_interp[0]:.5f}"
                            interp_row['TCP_Y_m'] = f"{tcp_p_interp[1]:.5f}"
                            interp_row['TCP_Z_m'] = f"{tcp_p_interp[2]:.5f}"
                            interp_row['TCP_RX_rad'] = f"{tcp_r_interp[0]:.4f}"
                            interp_row['TCP_RY_rad'] = f"{tcp_r_interp[1]:.4f}"
                            interp_row['TCP_RZ_rad'] = f"{tcp_r_interp[2]:.4f}"
                        
                        interpolated_rows.append(interp_row)
    
    # 重新编号帧
    for idx, row in enumerate(interpolated_rows):
        row['FrameNumber'] = str(idx)
    
    # 写入输出CSV
    with open(output_csv, 'w', encoding='utf-8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(interpolated_rows)
    
    # 输出统计信息
    print(f"\n插值完成！")
    print(f"输出文件: {output_csv}")
    print(f"插值后点数: {len(interpolated_rows)}")
    print(f"新增点数: {interpolated_count}")
    print(f"总轨迹长度: {total_distance:.2f} mm ({total_distance/10:.2f} cm)")
    print(f"最大段距离: {max_segment_distance:.2f} mm")
    print(f"平均点间距: {total_distance / (len(interpolated_rows) - 1):.2f} mm")


def main():
    parser = argparse.ArgumentParser(description='轨迹插值工具 - 确保每1cm至少有一个采样点')
    parser.add_argument('input_file', help='输入CSV文件路径')
    parser.add_argument('--distance', type=float, default=10.0, 
                        help='插值距离阈值(mm)，默认10mm (1cm)')
    parser.add_argument('--output', default=None, 
                        help='输出文件路径，默认为 <input>_interpolated.csv')
    
    args = parser.parse_args()
    
    # 构建输出文件名
    if args.output is None:
        input_path = Path(args.input_file)
        output_path = input_path.parent / f"{input_path.stem}_interpolated{input_path.suffix}"
        args.output = str(output_path)
    
    # 执行插值
    interpolate_trajectory(args.input_file, args.output, args.distance)


if __name__ == '__main__':
    main()

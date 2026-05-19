"""
简化版轨迹插值脚本
功能：仅对CSV第14、15、16列（TCP_X_m, TCP_Y_m, TCP_Z_m）进行线性插值

使用方法：
    1. 修改下面的 input_file 和 output_file 路径
    2. 运行: python SimpleInterpolator.py
"""

import csv
import numpy as np

# ==================== 在这里修改文件路径 ====================
input_file = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\PlaybackRecord_666_7_20260203_210446_7_20260203_210805.csv"
output_file = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\PlaybackRecord_666_7_20260203_210446_7_20260203_210932.csv"

# 插值距离阈值(m)，相邻点距离大于此值时进行插值
distance_threshold_m = 0.05  # 1cm = 0.01m
# ============================================================


def interpolate_position_only(input_csv, output_csv, threshold_m):
    """
    只对TCP位置列（TCP_X_m, TCP_Y_m, TCP_Z_m）进行插值，其他列复制或简单线性插值
    """
    # 读取CSV
    with open(input_csv, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        # 清理fieldnames，去除空白和换行
        fieldnames = [name.strip() for name in reader.fieldnames if name and name.strip()]
        rows = list(reader)
    
    if len(rows) < 2:
        print("错误：CSV至少需要2个数据点")
        return
    
    print(f"输入文件: {input_csv}")
    print(f"原始点数: {len(rows)}")
    print(f"距离阈值: {threshold_m * 1000:.1f} mm ({threshold_m * 100:.1f} cm)")
    
    # 插值结果
    result_rows = []
    total_distance = 0.0
    added_points = 0
    
    for i in range(len(rows)):
        # 添加原始点
        result_rows.append(rows[i].copy())
        
        # 检查是否需要插值
        if i < len(rows) - 1:
            # 读取第14、15、16列（TCP_X_m, TCP_Y_m, TCP_Z_m）
            p1 = np.array([
                float(rows[i]['TCP_X_m']),
                float(rows[i]['TCP_Y_m']),
                float(rows[i]['TCP_Z_m'])
            ])
            
            p2 = np.array([
                float(rows[i + 1]['TCP_X_m']),
                float(rows[i + 1]['TCP_Y_m']),
                float(rows[i + 1]['TCP_Z_m'])
            ])
            
            # 计算距离（单位：米）
            distance = np.linalg.norm(p2 - p1)
            total_distance += distance
            
            # 每两个样本点之间都需要插值到间隔1cm为止
            if distance > threshold_m:
                num_segments = int(np.ceil(distance / threshold_m))
                num_new_points = num_segments - 1
                
                if num_new_points > 0:
                    added_points += num_new_points
                    
                    # 对其他数值列也进行线性插值（可选）
                    numeric_fields = []
                    for field in fieldnames:
                        if field not in ['FrameNumber', 'TCP_X_m', 'TCP_Y_m', 'TCP_Z_m']:
                            try:
                                float(rows[i][field])
                                numeric_fields.append(field)
                            except:
                                pass
                    
                    # 插值点
                    for j in range(1, num_segments):
                        t = j / num_segments
                        
                        # TCP位置插值
                        p_interp = p1 + t * (p2 - p1)
                        
                        # 创建新行（复制第一个点的所有数据）
                        new_row = rows[i].copy()
                        
                        # 更新TCP位置列（第14、15、16列）
                        new_row['TCP_X_m'] = f"{p_interp[0]:.5f}"
                        new_row['TCP_Y_m'] = f"{p_interp[1]:.5f}"
                        new_row['TCP_Z_m'] = f"{p_interp[2]:.5f}"
                        
                        # 其他数值列线性插值
                        for field in numeric_fields:
                            v1 = float(rows[i][field])
                            v2 = float(rows[i + 1][field])
                            v_interp = v1 + t * (v2 - v1)
                            
                            # 保持原有精度格式
                            if '.' in rows[i][field]:
                                decimals = len(rows[i][field].split('.')[-1])
                                new_row[field] = f"{v_interp:.{decimals}f}"
                            else:
                                new_row[field] = f"{v_interp:.3f}"
                        
                        result_rows.append(new_row)
    
    # 重新编号（仅在FrameNumber字段存在时）
    if 'FrameNumber' in fieldnames:
        for idx, row in enumerate(result_rows):
            row['FrameNumber'] = str(idx)
    
    # 写入CSV（确保只写入fieldnames中的字段）
    with open(output_csv, 'w', encoding='utf-8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        # 只写入fieldnames中的字段
        clean_rows = []
        for row in result_rows:
            clean_row = {k: row.get(k, '') for k in fieldnames}
            clean_rows.append(clean_row)
        writer.writerows(clean_rows)
    
    # 统计信息
    print(f"\n✓ 插值完成！")
    print(f"输出文件: {output_csv}")
    print(f"插值后点数: {len(result_rows)}")
    print(f"新增点数: {added_points}")
    print(f"总轨迹长度: {total_distance * 1000:.1f} mm ({total_distance * 100:.1f} cm)")
    print(f"平均点间距: {total_distance * 1000 / (len(result_rows) - 1):.2f} mm")


if __name__ == '__main__':
    interpolate_position_only(input_file, output_file, distance_threshold_m)

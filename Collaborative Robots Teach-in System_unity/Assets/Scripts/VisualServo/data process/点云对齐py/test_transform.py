# -*- coding: utf-8 -*-
"""测试apply_transform.py的新流程"""

import sys
import os

# 添加当前目录到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 确保输出编码正确
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')

print("=" * 60)
print("测试配准验证功能")
print("=" * 60)

try:
    from apply_transform import transform_for_verification
    
    source_csv = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\trackerre.csv"
    target_csv = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\tcpp2.csv"
    
    # 输出文件路径（用于可视化）
    output_transformed = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\trackerre_transformed_5000.csv"
    output_target = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\tcpp2_preprocessed_5000.csv"
    
    transformed, target_proc, errors = transform_for_verification(
        source_csv, 
        target_csv,
        output_transformed_csv=output_transformed,
        output_target_csv=output_target,
        verbose=True
    )
    
    print("\n" + "=" * 60)
    print("验证完成！")
    print("=" * 60)
    print(f"\n对比配准时的误差：")
    print(f"  配准报告 - 真实误差: 平均=1.6573, RMSE=2.7058")
    print(f"  验证结果 - 误差:     平均={errors['mean']:.4f}, RMSE={errors['rmse']:.4f}")
    
except Exception as e:
    import traceback
    print(f"\n错误: {e}")
    traceback.print_exc()

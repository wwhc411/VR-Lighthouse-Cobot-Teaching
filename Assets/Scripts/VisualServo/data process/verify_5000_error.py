"""
验证5000点预处理文件的误差
"""
import pandas as pd
import numpy as np

# 读取5000点文件
csv1 = r'C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\trackerre_transformed_5000.csv'
csv2 = r'C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\tcpp2_preprocessed_5000.csv'

df1 = pd.read_csv(csv1)
df2 = pd.read_csv(csv2)

print('=== 5000点文件验证 ===')
print(f'变换后轨迹: {len(df1)} 点')
print(f'参考轨迹: {len(df2)} 点')
print(f'变换后列: {list(df1.columns)}')
print(f'参考列: {list(df2.columns)}')

# 计算误差
pos1 = df1[['X_mm', 'Y_mm', 'Z_mm']].values
pos2 = df2[['X_mm', 'Y_mm', 'Z_mm']].values
errors = np.linalg.norm(pos1 - pos2, axis=1)

print('')
print('=== 误差统计 ===')
print(f'平均误差: {np.mean(errors):.4f} mm')
print(f'标准差: {np.std(errors):.4f} mm')
print(f'RMSE: {np.sqrt(np.mean(errors**2)):.4f} mm')
print(f'最大误差: {np.max(errors):.4f} mm')
print(f'最小误差: {np.min(errors):.4f} mm')

# 与配准报告对比
print('')
print('=== 与配准报告对比 ===')
print('配准报告: mean=1.6573mm, RMSE=2.7058mm')
print(f'验证结果: mean={np.mean(errors):.4f}mm, RMSE={np.sqrt(np.mean(errors**2)):.4f}mm')
print(f'差异: mean={abs(np.mean(errors)-1.6573):.4f}mm, RMSE={abs(np.sqrt(np.mean(errors**2))-2.7058):.4f}mm')

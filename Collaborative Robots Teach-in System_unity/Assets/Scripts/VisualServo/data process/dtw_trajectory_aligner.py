"""
DTW 轨迹对齐工具

功能：
    使用经典 DTW 算法对齐两条采样密度不同的 3D 轨迹
    输出对应同一位置的点对到 CSV 文件

输入：
    - 轨迹1 CSV（例如：快速、稀疏轨迹）
    - 轨迹2 CSV（例如：慢速、稠密轨迹）
    - 读取第 4-6 列作为位置数据 (X_mm, Y_mm, Z_mm)

输出：
    - aligned_trajectories.csv - 对齐后的点对数据
    - dtw_alignment_visualization.png - 可视化结果

使用方法：
    python dtw_trajectory_aligner.py
    
作者: Copilot
日期: 2026-01-30
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Tuple, Dict
import os


class DTWTrajectoryAligner:
    """
    经典 DTW 轨迹对齐器（优先保证精度）
    
    时间复杂度: O(M×N)
    空间复杂度: O(M×N)
    
    适用于: M×N < 1,000,000 的中小规模轨迹
    """
    
    def __init__(self):
        self.dtw_matrix = None
        self.distance_matrix = None
        self.alignment_path = None
    
    def align(self, 
              traj1: np.ndarray, 
              traj2: np.ndarray,
              distance_metric: str = 'euclidean') -> Dict:
        """
        对齐两条轨迹
        
        参数:
            traj1: 轨迹1 (M × 3)，每行 [X, Y, Z]
            traj2: 轨迹2 (N × 3)，每行 [X, Y, Z]
            distance_metric: 距离度量 ('euclidean' 或 'manhattan')
        
        返回:
            {
                'path': [(i0,j0), (i1,j1), ...],     # 对齐路径
                'cost': float,                        # 总代价
                'normalized_cost': float,             # 归一化代价
                'traj1_to_traj2': {i: j},            # 轨迹1→轨迹2映射
                'traj2_to_traj1': {j: i}             # 轨迹2→轨迹1映射
            }
        """
        M = len(traj1)
        N = len(traj2)
        
        print(f"\n{'='*60}")
        print(f"[DTW 经典算法] 开始对齐")
        print(f"  轨迹1: {M} 个点")
        print(f"  轨迹2: {N} 个点")
        print(f"  矩阵规模: {M} × {N} = {M*N:,} 个元素")
        print(f"  距离度量: {distance_metric}")
        print(f"{'='*60}")
        
        # 步骤 1: 计算距离矩阵
        print("\n[步骤 1/4] 计算点对点距离矩阵...")
        self.distance_matrix = self._compute_distance_matrix(
            traj1, traj2, distance_metric
        )
        print(f"  完成 - 距离范围: [{self.distance_matrix.min():.2f}, {self.distance_matrix.max():.2f}] mm")
        
        # 步骤 2: 计算 DTW 累积距离矩阵
        print("\n[步骤 2/4] 计算 DTW 累积距离矩阵...")
        self.dtw_matrix = self._compute_dtw_matrix(self.distance_matrix)
        print(f"  完成 - 总代价: {self.dtw_matrix[M-1, N-1]:.2f}")
        
        # 步骤 3: 回溯最优路径
        print("\n[步骤 3/4] 回溯最优对齐路径...")
        self.alignment_path = self._backtrack_path(self.dtw_matrix)
        print(f"  完成 - 路径长度: {len(self.alignment_path)}")
        
        # 步骤 4: 构建索引映射
        print("\n[步骤 4/4] 构建索引映射...")
        traj1_to_traj2, traj2_to_traj1 = self._build_index_mapping(
            self.alignment_path, M, N
        )
        print(f"  完成 - 映射关系已建立")
        
        total_cost = self.dtw_matrix[M-1, N-1]
        normalized_cost = total_cost / len(self.alignment_path)
        
        print(f"\n{'='*60}")
        print(f"[DTW 对齐完成]")
        print(f"  路径长度: {len(self.alignment_path)}")
        print(f"  总代价: {total_cost:.2f}")
        print(f"  归一化代价: {normalized_cost:.4f} mm/点对")
        print(f"{'='*60}\n")
        
        return {
            'path': self.alignment_path,
            'cost': total_cost,
            'normalized_cost': normalized_cost,
            'traj1_to_traj2': traj1_to_traj2,
            'traj2_to_traj1': traj2_to_traj1
        }
    
    def _compute_distance_matrix(self, 
                                  traj_a: np.ndarray, 
                                  traj_b: np.ndarray,
                                  metric: str) -> np.ndarray:
        """
        计算点对点距离矩阵 D[i,j] = distance(a_i, b_j)
        """
        M, N = len(traj_a), len(traj_b)
        D = np.zeros((M, N))
        
        for i in range(M):
            for j in range(N):
                if metric == 'euclidean':
                    D[i, j] = np.linalg.norm(traj_a[i] - traj_b[j])
                elif metric == 'manhattan':
                    D[i, j] = np.sum(np.abs(traj_a[i] - traj_b[j]))
                else:
                    raise ValueError(f"未知的距离度量: {metric}")
        
        return D
    
    def _compute_dtw_matrix(self, distance_matrix: np.ndarray) -> np.ndarray:
        """
        计算 DTW 累积距离矩阵
        
        递推公式:
            DTW[i,j] = D[i,j] + min(
                DTW[i-1, j],      # 从上方来
                DTW[i, j-1],      # 从左方来
                DTW[i-1, j-1]     # 从对角来
            )
        """
        M, N = distance_matrix.shape
        DTW = np.full((M, N), np.inf)
        
        # 初始化起点
        DTW[0, 0] = distance_matrix[0, 0]
        
        # 初始化第一行（只能从左边来）
        for j in range(1, N):
            DTW[0, j] = DTW[0, j-1] + distance_matrix[0, j]
        
        # 初始化第一列（只能从上面来）
        for i in range(1, M):
            DTW[i, 0] = DTW[i-1, 0] + distance_matrix[i, 0]
        
        # 填充其余格子（动态规划）
        for i in range(1, M):
            for j in range(1, N):
                DTW[i, j] = distance_matrix[i, j] + min(
                    DTW[i-1, j],      # 上方
                    DTW[i, j-1],      # 左方
                    DTW[i-1, j-1]     # 对角
                )
        
        return DTW
    
    def _backtrack_path(self, dtw_matrix: np.ndarray) -> List[Tuple[int, int]]:
        """
        从终点回溯到起点，找到最优对齐路径
        
        策略: 每步选择累积代价最小的方向
        """
        M, N = dtw_matrix.shape
        path = []
        
        i, j = M - 1, N - 1
        path.append((i, j))
        
        while i > 0 or j > 0:
            # 收集候选来源
            candidates = []
            
            if i > 0 and j > 0:
                candidates.append((i-1, j-1, dtw_matrix[i-1, j-1]))  # 对角
            if i > 0:
                candidates.append((i-1, j, dtw_matrix[i-1, j]))      # 上方
            if j > 0:
                candidates.append((i, j-1, dtw_matrix[i, j-1]))      # 左方
            
            # 选择代价最小的方向
            i, j, _ = min(candidates, key=lambda x: x[2])
            path.append((i, j))
        
        path.reverse()
        return path
    
    def _build_index_mapping(self, 
                             path: List[Tuple[int, int]], 
                             M: int, 
                             N: int) -> Tuple[Dict[int, int], Dict[int, int]]:
        """
        从对齐路径构建索引映射
        
        策略: 
        - 一个点可能对应多个点，取最后出现的对应关系
        - 确保所有索引都有映射（填补空缺）
        """
        traj1_to_traj2 = {}
        traj2_to_traj1 = {}
        
        for i, j in path:
            traj1_to_traj2[i] = j
            traj2_to_traj1[j] = i
        
        # 填补空缺（使用最近的已有映射）
        for i in range(M):
            if i not in traj1_to_traj2:
                if i > 0:
                    traj1_to_traj2[i] = traj1_to_traj2[i-1]
                else:
                    traj1_to_traj2[i] = 0
        
        for j in range(N):
            if j not in traj2_to_traj1:
                if j > 0:
                    traj2_to_traj1[j] = traj2_to_traj1[j-1]
                else:
                    traj2_to_traj1[j] = 0
        
        return traj1_to_traj2, traj2_to_traj1


def load_trajectory_csv(csv_path: str, 
                        position_columns: List[str] = ['X_mm', 'Y_mm', 'Z_mm']) -> Tuple[pd.DataFrame, np.ndarray]:
    """
    从 CSV 文件加载轨迹数据
    
    参数:
        csv_path: CSV 文件路径
        position_columns: 位置列名（默认第4-6列）
    
    返回:
        (完整的DataFrame, 位置数组 N×3)
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"文件不存在: {csv_path}")
    
    df = pd.read_csv(csv_path)
    
    # 检查列是否存在
    for col in position_columns:
        if col not in df.columns:
            raise ValueError(f"列 '{col}' 不存在于文件 {csv_path} 中")
    
    positions = df[position_columns].values
    
    print(f"[加载] {os.path.basename(csv_path)}")
    print(f"  - 点数: {len(positions)}")
    print(f"  - 列名: {list(df.columns)}")
    print(f"  - 位置列: {position_columns}")
    
    return df, positions


def export_aligned_csv(traj1_df: pd.DataFrame,
                       traj2_df: pd.DataFrame,
                       traj1_positions: np.ndarray,
                       traj2_positions: np.ndarray,
                       alignment_result: Dict,
                       output_path: str = 'aligned_trajectories.csv'):
    """
    导出对齐后的轨迹数据到 CSV
    
    输出格式:
        - Traj1_Index: 轨迹1的索引
        - Traj2_Index: 轨迹2的索引
        - Traj1_X/Y/Z: 轨迹1的位置
        - Traj2_X/Y/Z: 轨迹2的位置
        - Distance_mm: 点对之间的距离
        - 以及两条轨迹的所有原始列（带前缀）
    """
    path = alignment_result['path']
    traj1_to_traj2 = alignment_result['traj1_to_traj2']
    
    aligned_data = []
    
    # 遍历轨迹1的每个点
    for i in range(len(traj1_positions)):
        j = traj1_to_traj2[i]
        
        # 计算距离
        distance = np.linalg.norm(traj1_positions[i] - traj2_positions[j])
        
        # 构建记录
        record = {
            'Traj1_Index': i,
            'Traj2_Index': j,
            'Traj1_X_mm': traj1_positions[i, 0],
            'Traj1_Y_mm': traj1_positions[i, 1],
            'Traj1_Z_mm': traj1_positions[i, 2],
            'Traj2_X_mm': traj2_positions[j, 0],
            'Traj2_Y_mm': traj2_positions[j, 1],
            'Traj2_Z_mm': traj2_positions[j, 2],
            'Distance_mm': distance
        }
        
        # 添加轨迹1的所有原始列（带前缀 Traj1_）
        for col in traj1_df.columns:
            if col not in ['X_mm', 'Y_mm', 'Z_mm']:
                record[f'Traj1_{col}'] = traj1_df.iloc[i][col]
        
        # 添加轨迹2的所有原始列（带前缀 Traj2_）
        for col in traj2_df.columns:
            if col not in ['X_mm', 'Y_mm', 'Z_mm']:
                record[f'Traj2_{col}'] = traj2_df.iloc[j][col]
        
        aligned_data.append(record)
    
    # 创建 DataFrame 并保存
    df_aligned = pd.DataFrame(aligned_data)
    df_aligned.to_csv(output_path, index=False)
    
    # 统计信息
    distances = df_aligned['Distance_mm'].values
    
    print(f"\n{'='*60}")
    print(f"[导出对齐数据] {output_path}")
    print(f"  - 点对数量: {len(df_aligned)}")
    print(f"  - 平均距离: {distances.mean():.2f} mm")
    print(f"  - 中位数距离: {np.median(distances):.2f} mm")
    print(f"  - 最大距离: {distances.max():.2f} mm")
    print(f"  - 距离标准差: {distances.std():.2f} mm")
    print(f"{'='*60}\n")
    
    return df_aligned


def export_uniform_sampled_pairs(traj1_df: pd.DataFrame,
                                  traj2_df: pd.DataFrame,
                                  alignment_result: Dict,
                                  num_samples: int = 100,
                                  output_traj1: str = 'sampled_traj1.csv',
                                  output_traj2: str = 'sampled_traj2.csv'):
    """
    从对齐后的轨迹中均匀采样N组点对，并分别导出到两个CSV文件
    
    参数:
        traj1_df: 轨迹1的完整DataFrame
        traj2_df: 轨迹2的完整DataFrame
        alignment_result: DTW对齐结果
        num_samples: 采样点对数量
        output_traj1: 轨迹1采样数据输出路径
        output_traj2: 轨迹2采样数据输出路径
    
    返回:
        (sampled_df1, sampled_df2): 采样后的两个DataFrame
    """
    traj1_to_traj2 = alignment_result['traj1_to_traj2']
    M = len(traj1_df)
    
    print(f"\n{'='*60}")
    print(f"[均匀采样点对]")
    print(f"  轨迹1总点数: {M}")
    print(f"  轨迹2总点数: {len(traj2_df)}")
    print(f"  采样点对数: {num_samples}")
    print(f"{'='*60}\n")
    
    # 在轨迹1上均匀采样索引
    if num_samples > M:
        print(f"⚠️ 警告: 采样数 {num_samples} 大于轨迹1点数 {M}，将使用全部点")
        num_samples = M
    
    # 均匀采样（确保包含起点和终点）
    sample_indices_traj1 = np.linspace(0, M-1, num_samples, dtype=int)
    
    # 构建采样数据
    sampled_data1 = []
    sampled_data2 = []
    
    for pair_idx, i in enumerate(sample_indices_traj1):
        j = traj1_to_traj2[i]
        
        # 从原始DataFrame提取完整行数据
        row1 = traj1_df.iloc[i].to_dict()
        row2 = traj2_df.iloc[j].to_dict()
        
        # 添加点对序列号（放在第一列）
        row1_with_idx = {'PairIndex': pair_idx}
        row1_with_idx.update(row1)
        
        row2_with_idx = {'PairIndex': pair_idx}
        row2_with_idx.update(row2)
        
        sampled_data1.append(row1_with_idx)
        sampled_data2.append(row2_with_idx)
    
    # 创建DataFrame
    sampled_df1 = pd.DataFrame(sampled_data1)
    sampled_df2 = pd.DataFrame(sampled_data2)
    
    # 保存到CSV
    sampled_df1.to_csv(output_traj1, index=False)
    sampled_df2.to_csv(output_traj2, index=False)
    
    # 统计信息
    print(f"[导出采样数据]")
    print(f"  轨迹1采样文件: {output_traj1}")
    print(f"    - 采样点数: {len(sampled_df1)}")
    print(f"    - 列数: {len(sampled_df1.columns)}")
    print(f"    - 索引范围: [{sample_indices_traj1[0]}, {sample_indices_traj1[-1]}]")
    
    print(f"\n  轨迹2采样文件: {output_traj2}")
    print(f"    - 采样点数: {len(sampled_df2)}")
    print(f"    - 列数: {len(sampled_df2.columns)}")
    
    # 显示采样索引对应关系
    print(f"\n  点对对应关系示例（前5个）:")
    print(f"  {'PairIndex':<12} {'Traj1索引':<12} {'Traj2索引':<12}")
    print(f"  {'-'*36}")
    for pair_idx in range(min(5, num_samples)):
        i = sample_indices_traj1[pair_idx]
        j = traj1_to_traj2[i]
        print(f"  {pair_idx:<12} {i:<12} {j:<12}")
    
    if num_samples > 5:
        print(f"  ...")
        pair_idx = num_samples - 1
        i = sample_indices_traj1[pair_idx]
        j = traj1_to_traj2[i]
        print(f"  {pair_idx:<12} {i:<12} {j:<12}")
    
    print(f"\n{'='*60}\n")
    
    return sampled_df1, sampled_df2


def visualize_alignment(traj1_pos: np.ndarray,
                       traj2_pos: np.ndarray,
                       alignment_result: Dict,
                       traj1_name: str = "Trajectory 1",
                       traj2_name: str = "Trajectory 2"):
    """
    可视化 DTW 对齐结果（4子图布局）
    直接显示图像窗口，不保存文件
    """
    # 设置中文字体支持
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
    
    fig = plt.figure(figsize=(20, 5))
    
    path = alignment_result['path']
    traj1_to_traj2 = alignment_result['traj1_to_traj2']
    
    # ========== 子图 1: DTW 矩阵热力图 ==========
    ax1 = fig.add_subplot(141)
    
    aligner = DTWTrajectoryAligner()
    dtw_matrix = aligner._compute_dtw_matrix(
        aligner._compute_distance_matrix(traj1_pos, traj2_pos, 'euclidean')
    )
    
    im = ax1.imshow(dtw_matrix.T, origin='lower', cmap='viridis', aspect='auto')
    ax1.set_xlabel(f'{traj1_name} 索引')
    ax1.set_ylabel(f'{traj2_name} 索引')
    ax1.set_title('DTW 累积距离矩阵')
    plt.colorbar(im, ax=ax1, label='累积距离 (mm)')
    
    # 叠加最优路径
    path_i = [p[0] for p in path]
    path_j = [p[1] for p in path]
    ax1.plot(path_i, path_j, 'r-', linewidth=2, label='最优路径', alpha=0.8)
    ax1.legend()
    
    # ========== 子图 2: 索引映射关系 ==========
    ax2 = fig.add_subplot(142)
    ax2.plot(path_i, 'b-', label=f'{traj1_name} 索引', linewidth=2)
    ax2.plot(path_j, 'r-', label=f'{traj2_name} 索引', linewidth=2)
    ax2.set_xlabel('路径步数')
    ax2.set_ylabel('轨迹索引')
    ax2.set_title('对齐路径索引变化')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # ========== 子图 3: 3D 轨迹对比 + 连线 ==========
    ax3 = fig.add_subplot(143, projection='3d')
    
    # 绘制完整轨迹
    ax3.plot(traj1_pos[:, 0], traj1_pos[:, 1], traj1_pos[:, 2], 
            'b-o', label=traj1_name, alpha=0.6, markersize=4)
    ax3.plot(traj2_pos[:, 0], traj2_pos[:, 1], traj2_pos[:, 2], 
            'r-', label=traj2_name, alpha=0.4, linewidth=1)
    
    # 绘制对齐点对的连线（采样显示）
    sample_step = max(1, len(path) // 20)
    for idx in range(0, len(path), sample_step):
        i, j = path[idx]
        ax3.plot([traj1_pos[i, 0], traj2_pos[j, 0]],
                [traj1_pos[i, 1], traj2_pos[j, 1]],
                [traj1_pos[i, 2], traj2_pos[j, 2]],
                'g-', alpha=0.3, linewidth=1)
    
    ax3.set_xlabel('X (mm)')
    ax3.set_ylabel('Y (mm)')
    ax3.set_zlabel('Z (mm)')
    ax3.set_title('3D 轨迹对齐可视化')
    ax3.legend()
    
    # ========== 子图 4: 点对距离分布 ==========
    ax4 = fig.add_subplot(144)
    
    # 计算所有点对的距离
    pair_distances = []
    for i in range(len(traj1_pos)):
        j = traj1_to_traj2[i]
        dist = np.linalg.norm(traj1_pos[i] - traj2_pos[j])
        pair_distances.append(dist)
    
    pair_distances = np.array(pair_distances)
    
    ax4.hist(pair_distances, bins=50, color='green', alpha=0.7, edgecolor='black')
    ax4.axvline(pair_distances.mean(), color='red', linestyle='--', linewidth=2,
               label=f'平均: {pair_distances.mean():.2f} mm')
    ax4.axvline(np.median(pair_distances), color='orange', linestyle='--', linewidth=2,
               label=f'中位数: {np.median(pair_distances):.2f} mm')
    ax4.set_xlabel('点对距离 (mm)')
    ax4.set_ylabel('频数')
    ax4.set_title('对齐点对距离分布')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    print(f"[可视化] 显示对齐结果图表...\n")
    plt.show()  # 直接显示图像，不保存文件


def main():
    """
    主函数 - DTW 轨迹对齐完整流程
    """
    print("\n" + "="*60)
    print("DTW 轨迹对齐工具")
    print("="*60 + "\n")
    
    # ============ 配置参数 ============
    # 修改这里的文件路径
    csv1_path = "Assets/StreamingAssets/TrackerRecordings/trackerrr1.csv"
    csv2_path = "Assets/StreamingAssets/TrackerRecordings/tcpppp1.csv"
    
    output_csv = "aligned_trajectories.csv"
    output_sampled_traj1 = "Assets/StreamingAssets/TrackerRecordings/sampled_traj1.csv"
    output_sampled_traj2 = "Assets/StreamingAssets/TrackerRecordings/sampled_traj2.csv"
    num_samples = 10  # 均匀采样点对数量
    
    position_columns = ['X_mm', 'Y_mm', 'Z_mm']  # 第4-6列
    
    # ============ 步骤 1: 加载数据 ============
    print("[步骤 1/4] 加载轨迹数据\n")
    
    traj1_df, traj1_pos = load_trajectory_csv(csv1_path, position_columns)
    traj2_df, traj2_pos = load_trajectory_csv(csv2_path, position_columns)
    
    # ============ 步骤 2: DTW 对齐 ============
    print("\n[步骤 2/4] 执行 DTW 对齐\n")
    
    aligner = DTWTrajectoryAligner()
    result = aligner.align(traj1_pos, traj2_pos, distance_metric='euclidean')
    
    # ============ 步骤 3: 导出完整对齐数据 ============
    print("\n[步骤 3/5] 导出完整对齐数据\n")
    
    df_aligned = export_aligned_csv(
        traj1_df, traj2_df,
        traj1_pos, traj2_pos,
        result,
        output_csv
    )
    
    # ============ 步骤 4: 均匀采样并导出点对 ============
    print("[步骤 4/5] 均匀采样点对并导出\n")
    
    sampled_df1, sampled_df2 = export_uniform_sampled_pairs(
        traj1_df, traj2_df,
        result,
        num_samples=num_samples,
        output_traj1=output_sampled_traj1,
        output_traj2=output_sampled_traj2
    )
    
    # ============ 步骤 5: 可视化 ============
    print("[步骤 5/5] 生成可视化\n")
    
    visualize_alignment(
        traj1_pos, traj2_pos, result,
        traj1_name="Tracker轨迹",
        traj2_name="TCP轨迹"
    )
    
    # ============ 完成 ============
    print("="*60)
    print("✅ DTW 对齐完成!")
    print("="*60)
    print(f"\n输出文件:")
    print(f"  1. {output_csv} - 完整对齐数据（所有点对）")
    print(f"  2. {output_sampled_traj1} - 轨迹1均匀采样数据（{num_samples}个点）")
    print(f"  3. {output_sampled_traj2} - 轨迹2均匀采样数据（{num_samples}个点）")
    print(f"  4. 可视化图表已显示在窗口中\n")
    
    # 显示前几个对应关系示例
    print("对应关系示例（前5个点）:")
    print("-" * 80)
    for idx in range(min(5, len(df_aligned))):
        row = df_aligned.iloc[idx]
        print(f"  Traj1[{int(row['Traj1_Index'])}] ({row['Traj1_X_mm']:.2f}, {row['Traj1_Y_mm']:.2f}, {row['Traj1_Z_mm']:.2f})")
        print(f"    ↕ 距离: {row['Distance_mm']:.2f} mm")
        print(f"  Traj2[{int(row['Traj2_Index'])}] ({row['Traj2_X_mm']:.2f}, {row['Traj2_Y_mm']:.2f}, {row['Traj2_Z_mm']:.2f})")
        print("-" * 80)


if __name__ == "__main__":
    main()

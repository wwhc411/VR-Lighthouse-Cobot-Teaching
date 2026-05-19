# MLS 复用误差来源深度诊断与改进方案

> 对应文件：`apply_transform.py` → `transform_csv()` MLS 分支

---

## 1. 现有流程的完整路径（带问题标记）

```
原始 N 个点
     │
     ▼
Step 1: 预平滑  →  points_to_process (N点，预平滑坐标)
     │
     ▼
Step 2: 重采样  →  points_resampled (M点，均匀弧长间距)      ←【插值误差①】
     │
     ▼
Step 3: 后平滑  →  points_processed (M点，处理完成坐标)
     │
     ▼
Step 4: 计算归一化弧长  →  norm_arc[j] = arc[j] / arc[-1]，基于 points_processed
     │
     ▼
Step 5: MLS 网格变换 M 次  →  transformed_resampled (M点，变换后坐标)
     │
     ▼
Step 6: 弧长线性反插值  →  transformed (N点，写入 CSV)       ←【插值误差②+假设谬误③+坐标空间混用④】
```

---

## 2. 四个具体问题的诊断

### 问题① — 重采样带来的第一次插值误差

重采样将原始 N 个点（例如 3000 点）插值成 M 个均匀弧长采样点（例如 2000 点）。每个重采样点的坐标是由相邻原始点的线性插值得到的，**已经不再等于原始测量值**。

```
原始点 p[i]（测量值）—→ 插值近似 → points_resampled[j]（中间值）
```

后续 MLS 变换作用在这批**已经被误差污染**的中间点上，学到的映射关系本身就受到插值误差的影响。

---

### 问题② — 反插值带来的第二次插值误差

Step 6 的目标是把 M 个变换后的点扩展回 N 个原始点对应的输出，用的是弧长插值：

```python
# 来自 apply_transform.py  Step 6
arc_transformed = compute_arc_length(transformed_resampled)       # M 个变换后点的弧长
target_arc_positions = arc_original / total_arc_original * total_arc_transformed
for dim in range(3):
    transformed[:, dim] = np.interp(target_arc_positions, arc_transformed, transformed_resampled[:, dim])
```

这是**第二次线性插值**，引入第二次近似误差。两次插值的误差方向不同，不能相消，只会叠加：

$$\text{总误差} \geq \varepsilon_{\text{重采样}} + \varepsilon_{\text{反插值}}$$

---

### 问题③ — 非刚性变换后"弧长比例不变"的假设失效（最关键）

Step 6 的反插值隐含了一个假设：

> 原始点在弧长轴上的相对位置（比例）在经过 MLS 变换后保持不变。

Step 6 的映射公式是：

$$\text{查询弧长} = \frac{s_{\text{原始}}}{L_{\text{原始}}} \times L_{\text{变换后}}$$

这等价于**全局均匀缩放**：把所有原始点的弧长按同一比例缩放到变换后弧长空间。

但 **MLS 是非刚性变换**，轨迹的不同位置段会有不同的局部旋转和平移，必然导致局部弧长伸缩不均匀：

```
训练轨迹示意（重采样空间，M 点均匀弧长分布）：

变换前：  ──────────┬──────────┬──────────┬──────────
弧长：     0%       25%        50%       75%      100%

变换后（非刚性）：
         ──────┬──────────────┬──────┬──────────────
               ↑ 该段旋转小      ↑ 该段旋转大，弧长拉伸
弧长：    0%   20%             55%   65%          100%
```

Step 6 用全局线性映射把原始点的 50% 位置映射到变换后弧长的 50% 处，**实际上应该映射到约 55%**。这种位置偏移在高曲率/大旋转段误差尤为显著。

---

### 问题④ — Step 6 使用了不同坐标空间的弧长

Step 6 反插值中：

| 变量 | 来源 |
|------|------|
| `arc_transformed` | 从 **M 个重采样+变换后**的点计算的弧长 |
| `arc_original` | 从 **N 个预平滑（未重采样）**的点计算的弧长 |

这两组弧长基于的坐标集不同：重采样改变了点的坐标（不只是数量），导致弧长参数化空间存在系统性偏移。用"预平滑弧长"查询"重采样+变换后的弧长轴"，本质上是在两个**不对齐的参数化坐标系**之间做插值。

```
坐标空间不匹配图示：

预平滑 N 点弧长 arc_smooth:    0 ──── 500 ──── 1000 ──── 2350 mm
                                       ↑
                                这里 arc_smooth[j] 对应原始点 j 的弧长

重采样 M 点弧长（变换前）:     0 ──── 470 ──── 940  ──── 2300 mm
（重采样改变了点的坐标 → 弧长总量也变了，各点的弧长位置也变了）

Step 6 用 arc_smooth[j] / 2350 * 2300 = 499 mm 作为查询值
真正应该查询的弧长却是约 490 mm（因为重采样坐标偏移）
→ 每点存在一个系统性的"弧长查询偏移"
```

---

## 3. 为什么分段 Kabsch 没有这些问题

`apply_transform.py` 中的分段 Kabsch 复用分支做的是：

```python
# 预平滑 → 计算稳定弧长 → 直接对每个原始点逐点变换
points_smooth = pre_smooth(points_original, PRE_SMOOTH)
arc = compute_arc_length(points_smooth)
norm_arc = arc / arc[-1]

for i in range(len(points_original)):
    transformed[i] = seg_transform.transform_point_normalized(
        points_original[i], norm_arc[i]    ← 原始坐标 + 平滑弧长定位
    )
```

**没有重采样，没有反插值，不存在上述四个问题。**

讽刺的是，`MLSTransform` 类本身已经实现了同样正确的 `transform_trajectory()` 方法：

```python
# tunable_registration.py  MLSTransform.transform_trajectory()
def transform_trajectory(self, points, mode="grid"):
    arc_lengths = compute_arc_length(points)
    normalized_arc = arc_lengths / (arc_lengths[-1] + 1e-10)
    transformed = np.zeros_like(points)
    for i in range(len(points)):
        transformed[i] = self.transform_point(points[i], normalized_arc[i], mode)
    return transformed
```

但 `transform_csv()` 的 MLS 分支没有调用它，而是用了一套带有上述四个缺陷的独立逻辑。

---

## 4. 改进方案

### 方案 A（推荐）：预平滑弧长定位 + 直接逐点变换

**核心思想**：复制分段 Kabsch 的正确做法，用预平滑数据计算稳定的归一化弧长，然后对每个原始点直接应用 MLS 网格变换，完全跳过重采样和反插值。

```python
# ===== 替换 transform_csv() MLS 分支中的 Step 2~6 =====

# Step A: 预平滑（稳定弧长计算，与训练一致）
points_smooth = pre_smooth(points_original.copy(), PRE_SMOOTH)

# Step B: 基于预平滑数据计算归一化弧长（稳定、抗噪）
arc = compute_arc_length(points_smooth)
norm_arc = arc / arc[-1]                   # ∈ [0, 1]，N 个点各自的轨迹比例位置

# Step C: 直接对每个原始点应用 MLS 网格变换（O(1)/点，共 N 次）
use_mode = MLS_CONFIG["use_mode"]
transformed = np.zeros_like(points_original)

if use_mode == "grid" and mls_transform.grid_transforms is not None:
    for i in range(len(points_original)):
        transformed[i] = mls_transform.transform_point_grid(
            points_original[i],   ← 变换原始坐标（保留测量特征）
            norm_arc[i]           ← 用平滑弧长精确定位
        )
else:
    for i in range(len(points_original)):
        transformed[i] = mls_transform.transform_point_full(
            points_original[i], norm_arc[i]
        )
```

**优点**：
- 消除两次插值误差
- 消除非刚性弧长比例假设
- 消除坐标空间混用问题
- 与分段 Kabsch 逻辑完全对称，维护成本低
- grid 模式 O(1)/点，N=3000 点仅需约 3ms，计算代价可忽略

**缺点 / 注意**：
- MLS 变换是在**重采样+平滑后的点对**上训练的，直接作用于原始坐标相当于向经过预处理的训练域之外稍微外推。在预平滑质量良好的前提下，这个差异通常在亚毫米级，远小于当前的两次插值误差。

---

### 方案 B：预平滑坐标 + 直接逐点变换（最干净，消除坐标域差异）

与方案 A 类似，但变换的是预平滑坐标而非原始坐标，与训练时的输入空间完全一致：

```python
points_smooth = pre_smooth(points_original.copy(), PRE_SMOOTH)
arc = compute_arc_length(points_smooth)
norm_arc = arc / arc[-1]

transformed = np.zeros_like(points_smooth)   ← 注意：基于 points_smooth

for i in range(len(points_smooth)):
    transformed[i] = mls_transform.transform_point_grid(
        points_smooth[i],   ← 训练一致的坐标空间
        norm_arc[i]
    )
```

**与方案 A 的区别**：输出的是预平滑后再变换的轨迹，噪声更少，但高频细节被平滑掉。
对于机器人控制轨迹回放，这通常是期望的效果。

---

### 方案 C：对称重采样（保留重采样但修复反插值逻辑）

若必须保留重采样步骤（例如为保证与训练时完全相同的坐标分布），改进反插值逻辑：

```
关键修复：Step 6 的反插值查询弧长应来自"重采样空间"而非"预平滑空间"

变换前 M 点弧长  →  等比例映射  →  在变换后 M 点弧长上插值
（两者均基于同一坐标集，不存在坐标空间混用）

具体：
  arc_pre  = compute_arc_length(points_resampled)     # 变换前 M 点弧长
  arc_post = compute_arc_length(transformed_resampled) # 变换后 M 点弧长
  
  # 原始点在"重采样前空间"的弧长位置  →  映射到"重采样空间弧长"
  # 通过 arc_smooth 与 arc_pre 的比例关系得到查询位置
  query = np.interp(arc_smooth / arc_smooth[-1], 
                    arc_smooth_normalized_to_resampled,  # 需额外建立映射
                    arc_pre)
```

此方案修复了问题④，但问题①②③依然存在，**不推荐**，仅作为最小改动的备用方案。

---

## 5. 三种方案对比

| | 现有逻辑 | 方案 A（推荐）| 方案 B | 方案 C |
|---|---|---|---|---|
| 插值次数 | **2 次** | **0 次** | **0 次** | 1 次（仅重采样）|
| 非刚性比例假设 | **有** | 无 | 无 | **有**（部分）|
| 坐标空间一致性 | **混用** | 一致 | **完全一致** | 部分修复 |
| 与训练坐标域差异 | 小（均为预处理后）| 有（原始vs预处理）| **无** | **无** |
| 输出点数 = 输入点数 | ✅ | ✅ | ✅  | ✅ |
| 实现复杂度 | 高 | **低（30行→10行）** | **低** | 中 |
| 预期误差增量 | 基准 | **显著降低** | **最低** | 轻微改善 |

---

## 6. 建议的修改位置

修改 `apply_transform.py` `transform_csv()` 函数中约第 **1445~1540 行**的 MLS 分支：

```python
# 当前：Step 1 → 重采样(M) → 后平滑(M) → MLS变换(M) → 反插值(N)
# 改为：Step 1(预平滑N) → 计算弧长(N) → MLS变换(N) → 直接输出(N)
```

同时，`transform_for_verification()` 函数（约第 1071 行）用于**在训练空间中评估配准误差**（计算 RMSE 等），它需要保持重采样流程以和训练时对齐，**不应修改**。

两个函数的用途：

| 函数 | 用途 | 是否保留重采样 |
|------|------|--------------|
| `transform_for_verification()` | 评估配准精度（RMSE计算） | ✅ 保留（与训练空间对齐）|
| `transform_csv()` | **实际复用输出**（回放轨迹）| ❌ 去掉，改用直接逐点变换 |

---

## 7. 问题根本原因小结

现有 MLS 复用逻辑的根本错误是：**将"训练时需要重采样"的前提错误地延伸到了"复用时也必须重采样"**。两者的目标完全不同：

- 训练时重采样：让 source 和 target 的对应点数相同，才能做一一对应的加权 Kabsch
- 复用时：MLS 变换已经是一个定义在 $[0,1]$ 归一化弧长空间上的连续函数，**任意点只要知道自己的归一化弧长位置，就可以直接查询对应的局部变换**，与训练时的点如何采样无关

分段 Kabsch 从一开始就理解了这一点，所以它的复用是正确的。MLS 复用时强行引入了不必要的重采样中间层，并试图用弧长反插值抹平这一层带来的误差，结果适得其反。

# Tracker 位姿录制与回放 - 配置及使用指南

**版本**: v1.0  
**更新日期**: 2025年12月2日

---

## 📋 目录

1. [功能概述](#功能概述)
2. [前置条件](#前置条件)
3. [场景配置](#场景配置)
4. [录制功能配置与使用](#录制功能配置与使用)
5. [回放功能配置与使用](#回放功能配置与使用)
6. [快捷键一览](#快捷键一览)
7. [常见问题解答](#常见问题解答)

---

## 功能概述

本系统提供 **SteamVR Tracker 位姿录制** 和 **UR 机器人轨迹回放** 两大核心功能：

| 功能 | 说明 |
|------|------|
| **录制** | 实时采集 Tracker 的位置(mm)和姿态(四元数+旋转矢量)，保存为 CSV 文件 |
| **回放** | 加载 CSV 文件，按指定频率发送 servoj 命令控制 UR 机器人 |

### 典型使用场景

1. **示教回放**: 手持 Tracker 演示轨迹 → 录制 → 机器人回放
2. **轨迹复现**: 录制动作 → 多次精确回放
3. **调试测试**: 录制位姿数据用于离线分析

---

## 前置条件

### 硬件要求

- ✅ HTC Vive / Valve Index 基站 (Lighthouse)
- ✅ Vive Tracker 设备 (已配对)
- ✅ UR 机器人 (已连接网络)
- ✅ 运行 SteamVR 的 PC

### 软件要求

- ✅ Unity 2020.3 或更高版本
- ✅ SteamVR 已安装并运行
- ✅ 项目已包含 SteamVR Plugin
- ✅ 项目已包含 `ViveTrackerPoseLogger` 组件

### 验证环境

1. 启动 SteamVR，确认基站和 Tracker 显示为绿色（已连接）
2. 在 SteamVR 设置中确认 Tracker 已被识别
3. 确认 UR 机器人控制面板显示已连接

---

## 场景配置

### 步骤 1: 添加录制器组件

1. 在 Unity Hierarchy 中创建空 GameObject，命名为 `TrackerRecorder`
2. 添加 `TrackerPoseRecorderCSV` 组件
3. 组件会自动查找场景中的 `ViveTrackerPoseLogger`

```
Hierarchy:
├── [Main Camera]
├── [SteamVR]
│   └── ViveTrackerPoseLogger  ← 必须存在
├── TrackerRecorder            ← 新建
│   └── TrackerPoseRecorderCSV (Script)
└── ...
```

### 步骤 2: 添加回放控制器组件

1. 创建另一个空 GameObject，命名为 `TrajectoryPlayer`
2. 添加 `RigidBodyServojController` 组件

```
Hierarchy:
├── TrackerRecorder
│   └── TrackerPoseRecorderCSV (Script)
├── TrajectoryPlayer           ← 新建
│   └── RigidBodyServojController (Script)
└── ...
```

### 步骤 3: 确认文件夹结构

确保 `StreamingAssets` 目录下存在录制文件夹：

```
Assets/
└── StreamingAssets/
    └── TrackerRecordings/     ← 如不存在会自动创建
        └── (CSV 文件保存位置)
```

---

## 录制功能配置与使用

### Inspector 参数配置

在 `TrackerPoseRecorderCSV` 组件的 Inspector 面板中配置以下参数：

#### Tracker 配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| **Tracker Device Id** | `2` | Tracker 设备 ID，使用右键菜单"列出所有 Tracker"确认 |

#### 录制参数

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| **Record Frequency Hz** | `125` | 10-500 | 录制频率，建议与回放频率一致 |
| **Verbose Logging** | `true` | - | 是否每100帧输出日志 |
| **Max Frames** | `50000` | - | 最大录制帧数（0=无限制） |

#### 组件引用

| 参数 | 默认值 | 说明 |
|------|--------|------|
| **Pose Logger** | (自动) | ViveTrackerPoseLogger 引用 |
| **Auto Find Components** | `true` | 自动查找组件 |

#### 文件保存

| 参数 | 默认值 | 说明 |
|------|--------|------|
| **Save Directory** | `TrackerRecordings` | 保存子目录（相对于 StreamingAssets） |
| **File Name Prefix** | `TrackerRecord` | 文件名前缀 |

### 录制操作步骤

#### 1. 确认 Tracker ID

首次使用前，需要确认正确的 Tracker ID：

1. 在 Inspector 中右键点击 `TrackerPoseRecorderCSV` 组件
2. 选择 **"列出所有 Tracker"**
3. 在 Console 中查看输出，找到你要录制的 Tracker ID

```
[TrackerRecorderCSV] ========== 发现 2 个 Tracker 设备 ==========
  ● Tracker ID: 2
  ● Tracker ID: 3
```

4. 将正确的 ID 填入 `Tracker Device Id` 字段

#### 2. 开始录制

1. 进入 Unity Play 模式
2. 将 Tracker 移动到起始位置
3. 按下 **`S` 键** 开始录制

Console 输出：
```
==================== 开始录制 ====================
  设备 ID: 2
  录制频率: 125 Hz (8.00 ms/帧)
  最大帧数: 50000
  按 E 键结束录制并保存 CSV
```

#### 3. 执行轨迹

- 平稳移动 Tracker，执行想要录制的轨迹
- 避免过快移动导致数据跳变
- 如开启详细日志，每100帧会输出当前位姿

#### 4. 结束录制

1. 按下 **`E` 键** 结束录制
2. 系统自动保存 CSV 文件

Console 输出：
```
==================== 录制完成 ====================
  录制时长: 10.50 秒
  录制帧数: 1312 帧
  实际帧率: 124.95 Hz (目标: 125 Hz)
  数据大小: ~128.00 KB (估算)

==================== CSV 保存成功 ====================
  文件路径: C:/项目路径/Assets/StreamingAssets/TrackerRecordings/TrackerRecord_2_20251202_143052.csv
  文件大小: 156.23 KB
  数据行数: 1312 行
```

### 录制数据验证

录制完成后，可以验证数据质量：

1. 在 Inspector 中右键点击组件
2. 选择 **"验证录制数据"**

输出示例：
```
==================== 数据验证 ====================
  位置范围:
    X: [-520.50, -480.25] mm (跨度: 40.25 mm)
    Y: [1180.00, 1220.50] mm (跨度: 40.50 mm)
    Z: [780.00, 820.75] mm (跨度: 40.75 mm)
  旋转角度范围: [0.00°, 15.50°]
  时间戳连续性: 通过 ✓
  总帧数: 1312
  录制时长: 10.50 秒
```

---

## 回放功能配置与使用

### Inspector 参数配置

在 `RigidBodyServojController` 组件的 Inspector 面板中配置以下参数：

#### CSV 文件配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| **Csv File Path** | (需填写) | CSV 文件路径，相对于 StreamingAssets |

示例路径：
```
TrackerRecordings/TrackerRecord_2_20251202_143052.csv
```

#### Servoj 控制参数

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| **Send Frequency Hz** | `125` | 10-500 | 发送频率，应与录制频率一致 |
| **Servoj Acceleration** | `0.001` | 0-10 | 关节加速度 (rad/s²) |
| **Servoj Velocity** | `0.01` | 0-3.14 | 关节速度 (rad/s) |
| **Servoj Look Ahead Time** | `0.1` | 0.03-0.2 | 前瞻时间 (s) |
| **Servoj Gain** | `300` | 100-2000 | 控制增益 |

#### UI 绑定（可选）

| 参数 | 说明 |
|------|------|
| **Load Button** | 加载按钮 (可选) |
| **Play Button** | 播放按钮 (可选) |
| **Pause Button** | 暂停按钮 (可选) |
| **Stop Button** | 停止按钮 (可选) |
| **Status Text** | 状态文本显示 (可选) |
| **Progress Text** | 进度文本显示 (可选) |
| **Progress Slider** | 进度条 (可选) |

### 回放操作步骤

#### 1. 配置 CSV 文件路径

在 Inspector 中填写要回放的 CSV 文件路径：

```
Csv File Path: TrackerRecordings/TrackerRecord_2_20251202_143052.csv
```

#### 2. 确认机器人连接

确保：
- UR 机器人已开机并连接
- Unity 中机器人连接状态为 `is_alive = true`
- 机器人处于远程控制模式

#### 3. 加载并回放

1. 进入 Unity Play 模式
2. 按下 **`L` 键** 加载 CSV 并开始回放

Console 输出：
```
[CSVCaptureReader] 成功加载CSV数据:
  文件路径: C:/项目路径/Assets/StreamingAssets/TrackerRecordings/TrackerRecord_2_20251202_143052.csv
  数据名称: TrackerRecord_2_20251202_143052
  有效帧数: 1312

[RigidBodyServojController] 开始播放 - 频率125Hz
```

#### 4. 播放控制

| 操作 | 快捷键 | 说明 |
|------|--------|------|
| 暂停/继续 | `P` | 切换暂停状态，暂停时机器人停止 |
| 停止 | `X` | 完全停止回放，重置到开头 |

#### 5. 回放完成

回放完成后，系统自动发送停止命令：

```
[RigidBodyServojController] 播放完成
[RigidBodyServojController] 已发送停止命令
```

---

## 快捷键一览

### 录制快捷键 (TrackerPoseRecorderCSV)

| 快捷键 | 功能 | 状态要求 |
|--------|------|----------|
| **S** | 开始录制 | 未在录制中 |
| **E** | 结束录制并保存 CSV | 正在录制中 |

### 回放快捷键 (RigidBodyServojController)

| 快捷键 | 功能 | 状态要求 |
|--------|------|----------|
| **L** | 加载 CSV 并开始回放 | 任意状态 |
| **P** | 暂停/继续回放 | 正在回放中 |
| **X** | 停止回放 | 正在回放中 |

### 快捷键记忆口诀

```
录制: S(Start) 开始, E(End) 结束
回放: L(Load) 加载, P(Pause) 暂停, X(Stop) 停止
```

---

## 常见问题解答

### Q1: 录制时提示"无法获取 Tracker 位姿"

**原因**: Tracker 设备未连接或 ID 错误

**解决方案**:
1. 确认 SteamVR 已启动且 Tracker 显示为绿色
2. 使用"列出所有 Tracker"功能确认正确的设备 ID
3. 检查 Tracker 电量是否充足

---

### Q2: 录制帧率不稳定

**原因**: Unity 性能不足或其他程序占用资源

**解决方案**:
1. 关闭不必要的后台程序
2. 降低录制频率（如 100Hz）
3. 在 Quality Settings 中降低画质
4. 确保 Unity 不在后台运行

---

### Q3: CSV 文件路径找不到

**原因**: 路径格式错误或文件不存在

**解决方案**:
1. 使用相对路径（相对于 StreamingAssets）
2. 确认文件名完全正确（包括时间戳）
3. 检查文件是否在 `StreamingAssets/TrackerRecordings/` 目录下

正确路径示例：
```
TrackerRecordings/TrackerRecord_2_20251202_143052.csv
```

---

### Q4: 回放时机器人不动

**原因**: 机器人未连接或命令发送失败

**解决方案**:
1. 确认 `UR_Control_Data.is_alive = true`
2. 确认机器人处于远程控制模式（非本地模式）
3. 检查网络连接是否正常
4. 查看 Console 是否有错误信息

---

### Q5: 回放运动不平滑

**原因**: 参数配置不当或频率不匹配

**解决方案**:
1. 确保录制频率和回放频率一致
2. 增大 `Look Ahead Time`（如 0.15）
3. 增大 `Gain`（如 500）
4. 降低运动速度

推荐平滑参数：
```
Send Frequency Hz: 125
Servoj Look Ahead Time: 0.15
Servoj Gain: 500
```

---

### Q6: 录制的轨迹与原始运动有偏差

**原因**: 坐标系未校准或手眼标定误差

**解决方案**:
1. 重新进行手眼标定
2. 确认 Tracker 安装位置稳固
3. 检查基站是否有移动或遮挡
4. 录制时避免 Tracker 遮挡

---

### Q7: 如何查看录制的 CSV 文件内容？

**方法**:
1. 使用 Excel 或 WPS 打开 CSV 文件
2. 使用文本编辑器（如 VS Code）查看
3. 使用 Python pandas 进行数据分析

CSV 列说明：
| 列名 | 说明 |
|------|------|
| FrameNumber | 帧序号 |
| TimeStamp_ms | Unix 时间戳 (毫秒) |
| TimeFromStart_s | 相对开始时间 (秒) |
| X_mm, Y_mm, Z_mm | 位置 (毫米) |
| QX, QY, QZ, QW | 四元数 |
| RX_rad, RY_rad, RZ_rad | 旋转矢量 (弧度) |

---

### Q8: 如何从录制器直接回放（不保存文件）？

**方法**: 使用 `LoadFromRecords` 方法

```csharp
// 获取录制器和回放控制器
TrackerPoseRecorderCSV recorder = GetComponent<TrackerPoseRecorderCSV>();
RigidBodyServojController player = GetComponent<RigidBodyServojController>();

// 停止录制
recorder.StopRecording();

// 获取录制数据（不保存文件）
List<TrackerPoseRecord> records = recorder.GetRecordedPoses();

// 加载到回放器
player.LoadFromRecords(records, "DirectPlayback");

// 开始回放
player.StartPlayback();
```

---

## 附录: 参数推荐配置

### 场景 1: 慢速精确轨迹

```
录制频率: 125 Hz
回放频率: 125 Hz
Acceleration: 0.001
Velocity: 0.01
Look Ahead Time: 0.15
Gain: 500
```

### 场景 2: 快速运动轨迹

```
录制频率: 250 Hz
回放频率: 250 Hz
Acceleration: 0.01
Velocity: 0.1
Look Ahead Time: 0.08
Gain: 300
```

### 场景 3: 调试测试

```
录制频率: 50 Hz
回放频率: 50 Hz
Acceleration: 0.001
Velocity: 0.005
Look Ahead Time: 0.2
Gain: 800
```

---

**文档结束**

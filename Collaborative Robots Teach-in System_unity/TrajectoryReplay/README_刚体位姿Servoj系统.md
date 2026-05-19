# Tracker 位姿录制与回放系统 - 完整架构文档

**版本**: v3.0  
**更新日期**: 2025年12月2日  
**系统类型**: 基于 SteamVR Tracker 的 UR 机器人 servoj 控制系统（CSV 格式）

---

## 📋 系统概述

### 功能定位
基于 **SteamVR Tracker** 实时录制位姿数据，通过逆运动学转换为 UR 机器人 servoj 控制命令，实现高精度位姿跟踪与轨迹回放。

### 核心能力
1. **实时录制**: 从 SteamVR Tracker 采集位置(mm)、四元数、旋转矢量(rad)
2. **CSV 存储**: 录制数据保存为 CSV 文件，便于查看和编辑
3. **逆运动学控制**: 使用 `get_inverse_kin(p[x,y,z,rx,ry,rz], qnear=[...])` 转换位姿
4. **精确回放**: 10-500Hz 可调频率发送 servoj 命令
5. **键盘控制**: 简洁的快捷键操作（S/E 录制，L/P/X 回放）

### 应用场景
- ✅ Tracker 轨迹录制与回放
- ✅ 视觉伺服/动捕跟踪
- ✅ 示教回放（手持 Tracker 演示轨迹）
- ✅ 笛卡尔空间轨迹复现

---

## ⌨️ 快捷键汇总

| 功能 | 快捷键 | 说明 |
|------|--------|------|
| **开始录制** | `S` | 开始采集 Tracker 位姿数据 |
| **结束录制** | `E` | 停止录制并保存 CSV 文件 |
| **加载并回放** | `L` | 加载 CSV 文件并立即开始回放 |
| **暂停/继续** | `P` | 切换暂停/继续状态 |
| **停止回放** | `X` | 停止回放并重置进度 |

---

## 🏗️ 系统架构

### 架构层次

```
┌─────────────────────────────────────────────────────────────────┐
│                        录制模块                                  │
│  TrackerPoseRecorderCSV (S键开始, E键结束保存CSV)                │
└─────────────────────────────┬───────────────────────────────────┘
                              │ CSV文件
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     StreamingAssets/TrackerRecordings/          │
│                     TrackerRecord_{ID}_{日期时间}.csv            │
└─────────────────────────────┬───────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                        回放模块                                  │
│  RigidBodyServojController (L键加载回放, P暂停, X停止)           │
│         ↓                                                        │
│  CSVCaptureReader → RigidBodyCaptureData → servoj命令            │
└─────────────────────────────┬───────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      UR机器人硬件层                              │
│  ur_data_processing.UR_Control_Data (发送servoj命令)            │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📦 模块详细说明

### 1️⃣ TrackerPoseRecord.cs (录制数据结构)

**功能**: 定义单帧位姿数据结构

```csharp
[Serializable]
public struct TrackerPoseRecord
{
    // 基本信息
    public int frameNumber;           // 帧序号（从0开始）
    public long timeStampMs;          // Unix时间戳（毫秒）
    public double timeFromStartSec;   // 相对录制开始时间（秒）
    
    // 位置 (单位: mm)
    public double x_mm, y_mm, z_mm;
    
    // 四元数 (原始姿态)
    public double qx, qy, qz, qw;
    
    // 旋转矢量 (单位: rad，由四元数转换)
    public double rx_rad, ry_rad, rz_rad;
}
```

**辅助方法**:
- `GetPositionMm()` → Vector3 (mm)
- `GetPositionM()` → Vector3 (m)
- `GetQuaternion()` → Quaternion
- `GetRotationVectorRad()` → Vector3 (rad)
- `GetRotationAngleDeg()` → float (度)

---

### 2️⃣ TrackerPoseRecorderCSV.cs (录制器)

**功能**: 实时录制 Tracker 位姿并保存为 CSV

**Inspector 配置**:
```csharp
[Header("Tracker 配置")]
public uint trackerDeviceId = 2;              // Tracker 设备 ID

[Header("录制参数")]
public float recordFrequencyHz = 125f;        // 录制频率 (10-500Hz)
public bool verboseLogging = true;            // 详细日志
public int maxFrames = 50000;                 // 最大帧数限制

[Header("文件保存")]
public string saveDirectory = "TrackerRecordings";  // 保存目录
public string fileNamePrefix = "TrackerRecord";     // 文件名前缀
```

**核心方法**:
| 方法 | 说明 |
|------|------|
| `StartRecording()` | 开始录制 (S键) |
| `StopRecording()` | 停止录制并保存 (E键) |
| `RecordCurrentPose()` | 采集当前帧位姿 |
| `QuaternionToRotationVector()` | 四元数转旋转矢量 |
| `SaveToCSV()` | 保存数据到 CSV 文件 |

**数据来源**:
```csharp
// 使用 ViveTrackerPoseLogger 获取 Tracker 位姿
poseLogger.GetTrackerPoseForCalibration(trackerDeviceId, 
    out Vector3 positionMm,    // 位置 (mm)
    out Quaternion rotation);  // 姿态 (四元数)
```

---

### 3️⃣ CSVCaptureReader.cs (CSV 读取器)

**功能**: 从 CSV 文件加载轨迹数据

**核心方法**:
```csharp
// 从CSV文件加载数据
public static RigidBodyCaptureData LoadFromCSV(string filePath)

// 验证CSV文件格式
public static bool ValidateCSVFile(string filePath, out string errorMessage)

// 从录制器直接转换（无需保存文件）
public static RigidBodyCaptureData ConvertFromRecords(
    List<TrackerPoseRecord> records, string trackerName)
```

**CSV 格式验证**:
- 检查必需列: `FrameNumber`, `X_mm`, `Y_mm`, `Z_mm`, `QX`, `QY`, `QZ`, `QW`
- 至少需要 10 列数据
- 支持 13 列完整格式（含旋转矢量）

---

### 4️⃣ RigidBodyCaptureData.cs (通用数据结构)

**功能**: 录制与回放共用的数据结构

```csharp
public class RigidBodyCaptureData {
    public Metadata Metadata;           // 元数据
    public List<FrameData> FrameData;   // 帧数据数组
}

public class FrameData {
    public int FrameNumber;             // 帧序号
    public string TimeStamp;            // 时间戳字符串
    public long UnixTimeStamp;          // Unix时间戳 (ms)
    
    public PositionData Position;       // 位置 (mm)
    public QuaternionData Quaternion;   // 四元数
    public VelocityData Velocity;       // 速度 (可选)
    public AccelerationData Acceleration; // 加速度 (可选)
}
```

---

### 5️⃣ RigidBodyServojController.cs (回放控制器)

**功能**: 从 CSV 加载数据并发送 servoj 命令

**Inspector 配置**:
```csharp
[Header("CSV文件配置")]
public string csvFilePath = "TrackerRecordings/TrackerRecord_2_20251202_120000.csv";

[Header("Servoj控制参数")]
public float sendFrequencyHz = 125f;        // 发送频率 (10-500Hz)
public float servojAcceleration = 0.001f;   // 加速度 (rad/s²)
public float servojVelocity = 0.01f;        // 速度 (rad/s)
public float servojLookAheadTime = 0.1f;    // 前瞻时间 (s)
public float servojGain = 300f;             // 控制增益
```

**核心方法**:
| 方法 | 快捷键 | 说明 |
|------|--------|------|
| `LoadAndPlay()` | L | 加载 CSV 并开始回放 |
| `PausePlayback()` | P | 暂停/继续回放 |
| `StopPlayback()` | X | 停止回放 |

---

### 6️⃣ RigidBodyServojCommandGenerator.cs (指令生成器)

**功能**: 生成 URScript servoj 命令

**参数结构**:
```csharp
public struct ServojParameters {
    public float Acceleration;     // 加速度 (rad/s²) [0.001-10]
    public float Velocity;         // 速度 (rad/s) [0.01-3.14]
    public float TimeStep;         // 时间步长 (s) = 1/频率
    public float LookAheadTime;    // 前瞻时间 (s) [0.03-0.2]
    public float Gain;             // 控制增益 [100-2000]
}
```

**生成的命令格式**:
```urscript
servoj(get_inverse_kin(p[x,y,z,rx,ry,rz], qnear=[j0,j1,j2,j3,j4,j5]), a, v, t, lookahead, gain)
```

---

## 📄 CSV 文件格式

### 表头（13列）

```csv
FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,RX_rad,RY_rad,RZ_rad
```

### 字段说明

| 列号 | 字段名 | 类型 | 单位 | 说明 |
|------|--------|------|------|------|
| 1 | `FrameNumber` | int | - | 帧序号（从0开始） |
| 2 | `TimeStamp_ms` | long | ms | Unix 时间戳 |
| 3 | `TimeFromStart_s` | double | s | 相对录制开始时间 |
| 4-6 | `X_mm`, `Y_mm`, `Z_mm` | double | mm | 位置（SteamVR坐标系） |
| 7-10 | `QX`, `QY`, `QZ`, `QW` | double | - | 四元数姿态 |
| 11-13 | `RX_rad`, `RY_rad`, `RZ_rad` | double | rad | 旋转矢量 |

### 示例数据

```csv
FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,RX_rad,RY_rad,RZ_rad
0,1733126400000,0.000000,-500.2500,1200.5000,800.7500,0.000000,0.000000,0.000000,1.000000,0.000000,0.000000,0.000000
1,1733126400008,0.008000,-500.3000,1200.5500,800.8000,0.000100,0.000200,0.000300,0.999999,0.000200,0.000400,0.000600
```

### 文件保存路径

```
Assets/StreamingAssets/TrackerRecordings/
    └── TrackerRecord_{TrackerID}_{yyyyMMdd_HHmmss}.csv
```

---

## 🔄 完整工作流程

### 录制流程

```
1. [准备阶段]
   - 确保 SteamVR 已启动
   - 确保 Tracker 设备已连接
   - 在 Inspector 中设置 trackerDeviceId

2. [开始录制] 按 S 键
   StartRecording()
   → 清空数据列表
   → 设置 FixedUpdate 频率 = recordFrequencyHz
   → 启动计时器
   → isRecording = true

3. [录制中] FixedUpdate 循环 (125Hz)
   RecordCurrentPose()
   → GetTrackerPoseForCalibration() 获取位姿
   → QuaternionToRotationVector() 转换姿态
   → 创建 TrackerPoseRecord 并添加到列表

4. [结束录制] 按 E 键
   StopRecording()
   → isRecording = false
   → 恢复原始 FixedUpdate 频率
   → SaveToCSV() 保存文件
   → 输出统计信息
```

### 回放流程

```
1. [加载并回放] 按 L 键
   LoadAndPlay()
   → CSVCaptureReader.LoadFromCSV() 加载 CSV
   → 验证数据有效性
   → StartPlayback() 开始播放

2. [播放循环] 协程
   PlaybackCoroutine()
   → 构建 ServojParameters (TimeStep = 1/频率)
   → while (currentFrameIndex < totalFrames):
       - 检查暂停状态
       - GenerateServojCommand() 生成命令
       - SendCommandToUR() 发送到 UR
       - 等待 sendInterval
       - currentFrameIndex++
   → 播放完成，发送停止命令

3. [控制操作]
   - 按 P 键: 暂停/继续
   - 按 X 键: 停止回放
```

---

## 🔍 数据流追踪

### 录制数据流

```
[SteamVR Tracker]
    │
    ▼
ViveTrackerPoseLogger.GetTrackerPoseForCalibration()
    │
    ├─ positionMm: Vector3 (mm)
    └─ rotation: Quaternion
           │
           ▼
    QuaternionToRotationVector(rotation)
           │
           └─ rotationVec: Vector3 (rad)
                  │
                  ▼
    TrackerPoseRecord {
        frameNumber, timeStampMs, timeFromStartSec,
        x_mm, y_mm, z_mm,
        qx, qy, qz, qw,
        rx_rad, ry_rad, rz_rad
    }
           │
           ▼
    SaveToCSV() → TrackerRecord_2_20251202_120000.csv
```

### 回放数据流

```
[CSV文件] TrackerRecord_2_20251202_120000.csv
    │
    ▼
CSVCaptureReader.LoadFromCSV()
    │
    ▼
RigidBodyCaptureData {
    Metadata: { TotalFrames, RigidBodyName, ... }
    FrameData[]: [
        { Position: {X,Y,Z}, Quaternion: {X,Y,Z,W}, ... }
    ]
}
    │
    ▼
RigidBodyServojCommandGenerator.GenerateServojCommand()
    │
    ├─ 位置: mm → m (÷1000)
    ├─ 旋转: 四元数 → 旋转矢量 (rad)
    └─ qnear: 从 UR_Stream_Data 读取
           │
           ▼
"servoj(get_inverse_kin(p[0.5,0.3,0.4,1.57,0,0], qnear=[...]), 0.001, 0.01, 0.008, 0.1, 300)\n"
           │
           ▼
SendCommandToUR() → UR_Control_Data.command
           │
           ▼
[UR机器人执行 servoj]
```

---

## ⚙️ 关键技术细节

### 1. 四元数转旋转矢量

```csharp
private Vector3 QuaternionToRotationVector(Quaternion q)
{
    // 1. 归一化四元数
    q = NormalizeQuaternion(q);
    
    // 2. 符号规范化 (强制 q.w >= 0)
    if (q.w < 0f) { q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w; }
    
    // 3. 计算旋转角度
    float angle = 2f * Mathf.Acos(Mathf.Clamp(q.w, 0f, 1f));
    
    // 4. 处理特殊情况
    if (angle < 1e-6f) return Vector3.zero;           // 接近 0°
    if (angle > Mathf.PI - 1e-4f) { ... }             // 接近 180°
    
    // 5. 一般情况
    float sinHalfAngle = Mathf.Sin(angle * 0.5f);
    float scale = angle / sinHalfAngle;
    return new Vector3(q.x * scale, q.y * scale, q.z * scale);
}
```

### 2. Servoj 参数说明

| 参数 | 范围 | 默认值 | 说明 |
|------|------|--------|------|
| **Acceleration** | 0.001-10 rad/s² | 0.001 | 关节加速度 |
| **Velocity** | 0.01-3.14 rad/s | 0.01 | 关节速度限制 |
| **TimeStep** | 0.002-0.2 s | 1/频率 | ⚠️ **必须等于发送周期** |
| **LookAheadTime** | 0.03-0.2 s | 0.1 | 轨迹前瞻时间 |
| **Gain** | 100-2000 | 300 | 控制增益 |

### 3. 频率控制

```csharp
// 录制频率控制 (FixedUpdate)
Time.fixedDeltaTime = 1.0f / recordFrequencyHz;  // 125Hz = 0.008s

// 回放频率控制 (协程)
float sendInterval = 1.0f / sendFrequencyHz;     // 125Hz = 0.008s
yield return new WaitForSeconds(sendInterval);
```

---

## 📊 性能指标

### 内存占用

```
单帧数据 (TrackerPoseRecord):
  - 基本信息: int + long + double = 20字节
  - 位置: 3 × double = 24字节
  - 四元数: 4 × double = 32字节
  - 旋转矢量: 3 × double = 24字节
  - 总计: ~100字节/帧

录制 1000 帧 (125Hz, 8秒): ~100KB
录制 10000 帧 (125Hz, 80秒): ~1MB
```

### CSV 文件大小

```
单行数据: ~120字节 (含逗号和换行)
1000 帧: ~120KB
10000 帧: ~1.2MB
```

---

## ⚠️ 注意事项

### 录制注意事项
1. **Tracker ID**: 使用 `ListAllTrackers()` 确认设备 ID
2. **录制频率**: 建议与回放频率一致 (默认 125Hz)
3. **最大帧数**: 默认限制 50000 帧，防止内存溢出
4. **坐标系**: 保存的是 SteamVR 原始坐标系数据

### 回放注意事项
1. **机器人连接**: 必须检查 `UR_Control_Data.is_alive`
2. **文件路径**: CSV 文件应放在 `StreamingAssets/TrackerRecordings/`
3. **单位转换**: 回放时自动将 mm 转换为 m
4. **停止命令**: 暂停/停止时自动发送 `speedl([0,0,0,0,0,0])`

---

## 🔧 故障排查

### 问题1: 录制无数据
**检查项**:
1. SteamVR 是否已启动
2. Tracker 设备是否已连接
3. trackerDeviceId 是否正确
4. ViveTrackerPoseLogger 组件是否存在

### 问题2: CSV 加载失败
**检查项**:
1. 文件路径是否正确
2. CSV 表头格式是否正确
3. 是否有足够的数据列 (至少10列)
4. 数值格式是否使用英文小数点

### 问题3: 机器人不动
**检查项**:
1. `UR_Control_Data.is_alive` 是否为 true
2. `manual_send_active` 是否被正确设置
3. servoj 命令格式是否正确
4. qnear 参数是否有效

### 问题4: 运动不平滑
**检查项**:
1. TimeStep 是否等于 1/发送频率
2. LookAheadTime 是否太小 (<0.03)
3. 录制频率与回放频率是否匹配
4. Unity 帧率是否稳定

---

## 📁 文件结构

```
Assets/Scripts/TrajectoryReplay/
├── TrackerPoseRecord.cs              # 录制数据结构
├── TrackerPoseRecorderCSV.cs         # 录制器 (S/E键控制)
├── CSVCaptureReader.cs               # CSV读取器
├── RigidBodyCaptureData.cs           # 通用数据结构
├── RigidBodyServojController.cs      # 回放控制器 (L/P/X键控制)
├── RigidBodyServojCommandGenerator.cs # servoj命令生成器
└── README_刚体位姿Servoj系统.md       # 本文档

Assets/StreamingAssets/TrackerRecordings/
└── TrackerRecord_{ID}_{日期时间}.csv  # 录制的CSV文件
```

---

## 📝 版本历史

**v3.0** (2025-12-02)
- ✅ 新增 Tracker 位姿录制功能 (TrackerPoseRecorderCSV)
- ✅ 数据格式从 JSON 改为 CSV
- ✅ 新增键盘快捷键控制 (S/E/L/P/X)
- ✅ 新增四元数转旋转矢量功能
- ✅ 移除所有 JSON 相关代码
- ✅ 录制与回放数据格式完全兼容

**v2.0** (2025-11-14)
- 基于 JSON 格式的回放系统（已废弃）

---

**文档结束**

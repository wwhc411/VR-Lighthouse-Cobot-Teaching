# Tracker 位姿录制功能（CSV版）- 代码实现框架

## 功能概述

**目标**: 实时录制 Tracker 设备的位姿数据，并保存为 CSV 文件

**核心需求**:
- 在 Unity Inspector 中配置 `trackerID`
- 读取 Tracker 原始位置信息 (x, y, z，单位 mm)
- 读取 Tracker 原始姿态四元数 (qx, qy, qz, qw)
- 将四元数转换为旋转矢量 (rx, ry, rz，单位弧度)
- 按 `S` 键开始录制，按 `E` 键结束录制
- 录制结束后保存为 CSV 文件

---

## 数据格式定义

### CSV 文件格式

```csv
FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,RX_rad,RY_rad,RZ_rad
0,1701500000000,0.000000,-500.25,1200.50,800.75,0.0000,0.0000,0.0000,1.0000,0.0000,0.0000,0.0000
1,1701500000008,0.008000,-500.30,1200.55,800.80,0.0001,0.0002,0.0003,0.9999,0.0002,0.0004,0.0006
...
```

### 字段说明

| 字段名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| `FrameNumber` | int | - | 帧序号（从0开始） |
| `TimeStamp_ms` | long | ms | Unix 时间戳（毫秒） |
| `TimeFromStart_s` | double | s | 相对录制开始时间 |
| `X_mm`, `Y_mm`, `Z_mm` | double | mm | 原始位置（SteamVR坐标系） |
| `QX`, `QY`, `QZ`, `QW` | double | - | 原始四元数姿态 |
| `RX_rad`, `RY_rad`, `RZ_rad` | double | rad | 旋转矢量（由四元数转换） |

---

## 接口分析

### ✅ 已实现的接口（可直接使用）

#### 1. Tracker 位姿获取接口

**文件**: `Assets/Scripts/VR/ViveTrackerPoseLogger.cs`

```csharp
/// <summary>
/// 获取指定Tracker的位姿（用于手眼标定数据采集）
/// 返回原始SteamVR坐标系下的位姿（毫米 + 四元数）
/// </summary>
/// <param name="deviceId">设备ID</param>
/// <param name="positionMm">位置（毫米）</param>
/// <param name="rotation">旋转（四元数）</param>
/// <returns>是否成功获取数据</returns>
public bool GetTrackerPoseForCalibration(uint deviceId, out Vector3 positionMm, out Quaternion rotation)
```

**调用示例**:
```csharp
ViveTrackerPoseLogger poseLogger = FindObjectOfType<ViveTrackerPoseLogger>();
if (poseLogger.GetTrackerPoseForCalibration(trackerDeviceId, out Vector3 positionMm, out Quaternion rotation))
{
    // 成功获取 positionMm (mm) 和 rotation (四元数)
}
```

---

#### 2. 四元数转旋转矢量接口

**文件**: `Assets/Scripts/VR/ViveTrackerPoseLogger.cs` (私有方法)

```csharp
static Vector3 QuaternionToRotationVector(Quaternion q)
{
    // 步骤1: 归一化四元数
    q = NormalizeQuaternion(q);
    
    // 步骤2: 四元数符号规范化 (强制 q.w >= 0)
    if (q.w < 0f)
    {
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
        q.w = -q.w;
    }
    
    // 步骤3: 计算旋转角度
    float wClamped = Mathf.Clamp(q.w, 0f, 1f);
    float angle = 2f * Mathf.Acos(wClamped);
    
    // 步骤4: 处理特殊情况 (接近180°、接近0°)
    // ... 完整实现见源代码
    
    // 一般情况: rotationVector = [q.x, q.y, q.z] * (angle / sin(angle/2))
    float sinHalfAngle = Mathf.Sin(angle * 0.5f);
    float scale = angle / sinHalfAngle;
    return new Vector3(q.x * scale, q.y * scale, q.z * scale);
}
```

**文件**: `Assets/Scripts/VR/TrackerPoseCapture.cs` (公开可用版本)

```csharp
/// <summary>
/// 四元数转旋转矢量 (修复版本)
/// </summary>
private Vector3 QuaternionToRotationVector(Quaternion q)
```

**文件**: `Assets/Scripts/Calibration/SteamVrUrCoordinateConverter.cs` (静态方法)

```csharp
/// <summary>
/// 四元数转换为轴角表示
/// </summary>
private static Vector3 QuaternionToRotationVector(Quaternion q)
```

> ⚠️ **注意**: 上述方法均为 `private`，需要复制实现或改为 `public`。

---

#### 3. 四元数归一化接口

**文件**: `Assets/Scripts/VR/ViveTrackerPoseLogger.cs`

```csharp
static Quaternion NormalizeQuaternion(Quaternion q)
{
    float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (mag > Mathf.Epsilon)
    {
        float invMag = 1f / mag;
        q.x *= invMag;
        q.y *= invMag;
        q.z *= invMag;
        q.w *= invMag;
    }
    else
    {
        q = Quaternion.identity;
    }
    return q;
}
```

---

### ❌ 需要新增的接口

#### 1. 录制数据结构 `TrackerPoseRecord`

**作用**: 存储单帧位姿数据（包含原始四元数和转换后的旋转矢量）

```csharp
/// <summary>
/// Tracker 单帧位姿记录
/// 包含原始四元数和转换后的旋转矢量
/// </summary>
[System.Serializable]
public struct TrackerPoseRecord
{
    public int frameNumber;           // 帧序号
    public long timeStampMs;          // Unix时间戳 (毫秒)
    public double timeFromStartSec;   // 相对开始时间 (秒)
    
    // 位置 (mm)
    public double x_mm;
    public double y_mm;
    public double z_mm;
    
    // 四元数 (原始)
    public double qx;
    public double qy;
    public double qz;
    public double qw;
    
    // 旋转矢量 (rad，由四元数转换)
    public double rx_rad;
    public double ry_rad;
    public double rz_rad;
}
```

---

#### 2. 主控制器 `TrackerPoseRecorderCSV`

**作用**: 录制控制逻辑 + CSV 文件保存

**需实现的方法**:

| 方法名 | 访问级别 | 功能说明 |
|--------|----------|----------|
| `Start()` | private | 初始化：查找 PoseLogger 组件、初始化列表 |
| `Update()` | private | 按键检测：S 开始、E 结束 |
| `FixedUpdate()` | private | 固定频率采集位姿数据 |
| `StartRecording()` | public | 开始录制：清空数据、启动计时器 |
| `StopRecording()` | public | 停止录制：保存 CSV 文件 |
| `RecordCurrentPose()` | private | 采集当前位姿并添加到列表 |
| `QuaternionToRotationVector()` | private | 四元数转旋转矢量（复制现有实现） |
| `SaveToCSV()` | private | 保存数据到 CSV 文件 |
| `GenerateCSVLine()` | private | 生成单行 CSV 数据 |

---

#### 3. CSV 文件保存方法 `SaveToCSV()`

**作用**: 将录制数据导出为 CSV 文件

```csharp
/// <summary>
/// 保存录制数据到 CSV 文件
/// </summary>
private void SaveToCSV()
{
    // 1. 生成文件名: TrackerRecord_{ID}_{日期时间}.csv
    // 2. 构建 CSV 表头
    // 3. 逐行写入数据
    // 4. 保存到 StreamingAssets 目录
}
```

**保存路径**: `Assets/StreamingAssets/TrackerRecordings/`

---

## 类设计概要

### TrackerPoseRecorderCSV.cs

```
┌─────────────────────────────────────────────────────────────┐
│                   TrackerPoseRecorderCSV                    │
├─────────────────────────────────────────────────────────────┤
│ [Inspector 配置]                                             │
│   + trackerDeviceId : uint = 2                              │
│   + recordFrequencyHz : float = 125                         │
│   + verboseLogging : bool = true                            │
│   + poseLogger : ViveTrackerPoseLogger                      │
│   + autoFindComponents : bool = true                        │
│   + saveDirectory : string = "TrackerRecordings"            │
├─────────────────────────────────────────────────────────────┤
│ [内部状态]                                                   │
│   - isRecording : bool                                      │
│   - recordedPoses : List<TrackerPoseRecord>                 │
│   - recordStopwatch : Stopwatch                             │
│   - frameCounter : int                                      │
├─────────────────────────────────────────────────────────────┤
│ [公开方法]                                                   │
│   + StartRecording() : void                                 │
│   + StopRecording() : void                                  │
│   + GetRecordedPoses() : List<TrackerPoseRecord>            │
│   + GetRecordedFrameCount() : int                           │
│   + ClearRecordedData() : void                              │
├─────────────────────────────────────────────────────────────┤
│ [私有方法]                                                   │
│   - Start() : void                                          │
│   - Update() : void           // 按键检测                   │
│   - FixedUpdate() : void      // 数据采集                   │
│   - RecordCurrentPose() : void                              │
│   - QuaternionToRotationVector(Quaternion) : Vector3        │
│   - NormalizeQuaternion(Quaternion) : Quaternion            │
│   - SaveToCSV() : void                                      │
│   - GenerateCSVLine(TrackerPoseRecord) : string             │
│   - EnsureDirectoryExists(string) : void                    │
└─────────────────────────────────────────────────────────────┘
```

---

## 数据流图

```
┌──────────────────────────────────────────────────────────────────────────┐
│                           录制数据流                                      │
└──────────────────────────────────────────────────────────────────────────┘

    按下 S 键                                                  按下 E 键
        │                                                          │
        ▼                                                          ▼
┌───────────────┐                                         ┌───────────────┐
│ StartRecording│                                         │ StopRecording │
│   - 清空列表   │                                         │  - 停止计时   │
│   - 启动计时   │                                         │  - 调用保存   │
│   - 设置标志   │                                         │  - 输出统计   │
└───────┬───────┘                                         └───────┬───────┘
        │                                                          │
        ▼                                                          ▼
┌───────────────────────────────────────────────────┐     ┌───────────────┐
│            FixedUpdate() 循环 (125Hz)              │     │   SaveToCSV   │
│                                                   │     │  - 生成文件名  │
│   ┌─────────────────────────────────────────────┐ │     │  - 写入表头   │
│   │ ViveTrackerPoseLogger                       │ │     │  - 写入数据   │
│   │ .GetTrackerPoseForCalibration()             │ │     │  - 保存文件   │
│   │                                             │ │     └───────┬───────┘
│   │   输出: positionMm (Vector3, mm)            │ │             │
│   │   输出: rotation (Quaternion)               │ │             ▼
│   └─────────────────┬───────────────────────────┘ │     ┌───────────────┐
│                     │                             │     │   CSV 文件    │
│                     ▼                             │     │ (StreamingAs- │
│   ┌─────────────────────────────────────────────┐ │     │  sets目录)    │
│   │ QuaternionToRotationVector()                │ │     └───────────────┘
│   │                                             │ │
│   │   输入: Quaternion (qx, qy, qz, qw)         │ │
│   │   输出: Vector3 (rx, ry, rz, 弧度)          │ │
│   └─────────────────┬───────────────────────────┘ │
│                     │                             │
│                     ▼                             │
│   ┌─────────────────────────────────────────────┐ │
│   │ 创建 TrackerPoseRecord                      │ │
│   │   - frameNumber                             │ │
│   │   - timeStampMs, timeFromStartSec           │ │
│   │   - x_mm, y_mm, z_mm                        │ │
│   │   - qx, qy, qz, qw                          │ │
│   │   - rx_rad, ry_rad, rz_rad                  │ │
│   └─────────────────┬───────────────────────────┘ │
│                     │                             │
│                     ▼                             │
│   ┌─────────────────────────────────────────────┐ │
│   │ recordedPoses.Add(record)                   │ │
│   └─────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────┘
```

---

## 接口依赖关系

```
TrackerPoseRecorderCSV
    │
    ├── [已实现] ViveTrackerPoseLogger.GetTrackerPoseForCalibration()
    │       │
    │       └── 返回: Vector3 positionMm (mm), Quaternion rotation
    │
    ├── [需复制] QuaternionToRotationVector(Quaternion)
    │       │
    │       ├── 来源: ViveTrackerPoseLogger.cs (私有静态方法)
    │       │         TrackerPoseCapture.cs (私有方法)
    │       │         SteamVrUrCoordinateConverter.cs (私有静态方法)
    │       │
    │       └── 输出: Vector3 (rx, ry, rz, 单位弧度)
    │
    ├── [需复制] NormalizeQuaternion(Quaternion)
    │       │
    │       └── 来源: ViveTrackerPoseLogger.cs (私有静态方法)
    │
    └── [需新增] SaveToCSV()
            │
            └── 使用: System.IO.File.WriteAllText()
                      System.Text.StringBuilder
                      System.Globalization.CultureInfo.InvariantCulture
```

---

## 关键代码片段（伪代码）

### 1. 位姿采集

```csharp
private void RecordCurrentPose()
{
    // 1. 调用现有接口获取位姿
    if (!poseLogger.GetTrackerPoseForCalibration(trackerDeviceId, 
        out Vector3 positionMm, out Quaternion rotation))
    {
        Debug.LogWarning("无法获取 Tracker 位姿");
        return;
    }
    
    // 2. 四元数转旋转矢量
    Vector3 rotationVec = QuaternionToRotationVector(rotation);
    
    // 3. 创建记录
    TrackerPoseRecord record = new TrackerPoseRecord
    {
        frameNumber = frameCounter,
        timeStampMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
        timeFromStartSec = recordStopwatch.Elapsed.TotalSeconds,
        
        x_mm = positionMm.x,
        y_mm = positionMm.y,
        z_mm = positionMm.z,
        
        qx = rotation.x,
        qy = rotation.y,
        qz = rotation.z,
        qw = rotation.w,
        
        rx_rad = rotationVec.x,
        ry_rad = rotationVec.y,
        rz_rad = rotationVec.z
    };
    
    // 4. 添加到列表
    recordedPoses.Add(record);
    frameCounter++;
}
```

### 2. CSV 保存

```csharp
private void SaveToCSV()
{
    // 1. 确保目录存在
    string dirPath = Path.Combine(Application.streamingAssetsPath, saveDirectory);
    EnsureDirectoryExists(dirPath);
    
    // 2. 生成文件名
    string fileName = $"TrackerRecord_{trackerDeviceId}_{DateTime.Now:yyyyMMdd_HHmmss}.csv";
    string filePath = Path.Combine(dirPath, fileName);
    
    // 3. 构建 CSV 内容
    StringBuilder sb = new StringBuilder();
    
    // 表头
    sb.AppendLine("FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,RX_rad,RY_rad,RZ_rad");
    
    // 数据行
    foreach (var record in recordedPoses)
    {
        sb.AppendLine(GenerateCSVLine(record));
    }
    
    // 4. 写入文件
    File.WriteAllText(filePath, sb.ToString());
    
    Debug.Log($"[TrackerRecorder] CSV 已保存: {filePath}");
}

private string GenerateCSVLine(TrackerPoseRecord record)
{
    return string.Format(CultureInfo.InvariantCulture,
        "{0},{1},{2:F6},{3:F4},{4:F4},{5:F4},{6:F6},{7:F6},{8:F6},{9:F6},{10:F6},{11:F6},{12:F6}",
        record.frameNumber,
        record.timeStampMs,
        record.timeFromStartSec,
        record.x_mm, record.y_mm, record.z_mm,
        record.qx, record.qy, record.qz, record.qw,
        record.rx_rad, record.ry_rad, record.rz_rad
    );
}
```

---

## 文件结构

```
Assets/Scripts/TrajectoryReplay/
    ├── TrackerPoseRecorderCSV.cs       # [新增] 主控制器脚本
    ├── TrackerPoseRecord.cs            # [新增] 数据结构定义
    │
    ├── RigidBodyCaptureData.cs         # [现有] 数据结构（可参考）
    ├── ViveTrackerPoseLogger.cs        # [现有] 位姿获取接口
    └── TrackerPoseCapture.cs           # [现有] 四元数转换参考

Assets/StreamingAssets/TrackerRecordings/
    └── TrackerRecord_{ID}_{日期时间}.csv   # [输出] 录制的CSV文件
```

---

## 使用方法

### 1. 场景配置

1. 在场景中创建空 GameObject，命名为 `TrackerRecorder`
2. 添加 `TrackerPoseRecorderCSV` 组件
3. 在 Inspector 中设置:
   - `Tracker Device Id`: 你的 Tracker ID（通常是 2 或 3）
   - `Record Frequency Hz`: 录制频率（默认 125Hz）
   - `Verbose Logging`: 是否显示详细日志
   - 勾选 `Auto Find Components` 自动查找 PoseLogger

### 2. 操作流程

```
1. 运行 Unity Play 模式
2. 确保 Tracker 设备已连接并被 SteamVR 识别
3. 按 S 键 → 开始录制（Console 显示 "开始录制"）
4. 移动 Tracker 完成轨迹
5. 按 E 键 → 结束录制并保存（Console 显示保存路径）
6. 在 StreamingAssets/TrackerRecordings/ 目录下找到 CSV 文件
```

### 3. CSV 文件验证

```
1. 用 Excel 或文本编辑器打开 CSV 文件
2. 检查列数是否为 13 列
3. 检查数据行数是否与控制台输出的帧数一致
4. 检查旋转矢量范围是否合理（一般 |rx|, |ry|, |rz| < π ≈ 3.14）
```

---

## 注意事项

1. **Tracker ID 确认**: 使用 `ViveTrackerPoseLogger` 的 "列出所有 Tracker" 功能确认设备 ID
2. **坐标系**: 保存的是 SteamVR 原始坐标系数据（右手系，X右 Y上 Z后）
3. **旋转矢量**: 使用轴角表示法，单位为弧度，范围 [-π, π]
4. **文件编码**: CSV 使用 UTF-8 编码，数字使用英文格式（小数点为 `.`）
5. **录制频率**: 默认 125Hz，可根据需要调整（建议与回放频率一致）

---

## 扩展建议

1. **添加 UI 界面**: 显示录制状态、帧数、时长
2. **实时预览**: 在 Scene 视图中绘制录制轨迹
3. **多 Tracker 支持**: 同时录制多个 Tracker 到同一文件
4. **数据验证**: 检测位姿跳变、丢帧等异常
5. **自动命名**: 支持自定义文件名前缀

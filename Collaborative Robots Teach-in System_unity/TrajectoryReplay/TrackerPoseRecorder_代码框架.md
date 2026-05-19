# Tracker 位姿录制功能 - 代码框架

## 功能概述

**目标**: 实时录制 Tracker 设备的位姿数据，用于后续轨迹回放

**控制方式**:
- 按 `S` 键: 开始录制
- 按 `E` 键: 结束录制并保存

**数据格式**: 位置(mm) + 四元数姿态

---

## 核心类设计

### 1. TrackerPoseRecorder.cs (主控制器)

```csharp
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Tracker 位姿录制器
/// 功能: 按 S 键开始录制, 按 E 键结束录制
/// 数据: 位置(mm) + 四元数姿态
/// </summary>
public class TrackerPoseRecorder : MonoBehaviour
{
    // ==================== Inspector 配置 ====================
    
    [Header("Tracker 配置")]
    [Tooltip("要录制的 Tracker 设备 ID")]
    public uint trackerDeviceId = 2;
    
    [Header("录制参数")]
    [Tooltip("录制频率 (Hz)，与回放频率一致")]
    [Range(10f, 500f)]
    public float recordFrequencyHz = 125f;
    
    [Tooltip("是否启用详细日志")]
    public bool verboseLogging = true;
    
    [Header("组件引用")]
    [Tooltip("ViveTrackerPoseLogger 组件引用")]
    public ViveTrackerPoseLogger poseLogger;
    
    [Tooltip("是否自动查找组件")]
    public bool autoFindComponents = true;
    
    // ==================== 内部状态 ====================
    
    private bool isRecording = false;                        // 录制状态
    private List<TrackerPoseData> recordedPoses;             // 录制的位姿数据数组
    private System.Diagnostics.Stopwatch recordStopwatch;    // 录制计时器
    private int frameCounter = 0;                            // 帧计数器
    
    // ==================== 生命周期 ====================
    
    void Start()
    {
        // 自动查找 PoseLogger 组件
        if (autoFindComponents && poseLogger == null)
        {
            poseLogger = FindObjectOfType<ViveTrackerPoseLogger>();
            if (poseLogger == null)
            {
                Debug.LogError("[TrackerRecorder] 未找到 ViveTrackerPoseLogger 组件!");
                enabled = false;
                return;
            }
        }
        
        // 初始化数据容器
        recordedPoses = new List<TrackerPoseData>();
        recordStopwatch = new System.Diagnostics.Stopwatch();
        
        Debug.Log($"<color=green>[TrackerRecorder] 录制器初始化完成</color>");
        Debug.Log($"  Tracker ID: {trackerDeviceId}");
        Debug.Log($"  录制频率: {recordFrequencyHz} Hz");
        Debug.Log($"  快捷键: S=开始录制, E=结束录制");
    }
    
    void Update()
    {
        // 检测按键
        if (Input.GetKeyDown(KeyCode.S))
        {
            StartRecording();
        }
        else if (Input.GetKeyDown(KeyCode.E))
        {
            StopRecording();
        }
    }
    
    void FixedUpdate()
    {
        // 在录制状态下按固定频率采集数据
        if (isRecording)
        {
            RecordCurrentPose();
        }
    }
    
    void OnDisable()
    {
        // 停止录制，防止退出时丢失数据
        if (isRecording)
        {
            StopRecording();
        }
    }
    
    // ==================== 录制控制 ====================
    
    /// <summary>
    /// 开始录制
    /// </summary>
    [ContextMenu("开始录制 (S键)")]
    public void StartRecording()
    {
        if (isRecording)
        {
            Debug.LogWarning("[TrackerRecorder] 已经在录制中，请先停止");
            return;
        }
        
        if (poseLogger == null)
        {
            Debug.LogError("[TrackerRecorder] PoseLogger 未设置!");
            return;
        }
        
        // 清空之前的数据
        recordedPoses.Clear();
        frameCounter = 0;
        
        // 设置 FixedUpdate 频率
        Time.fixedDeltaTime = 1.0f / recordFrequencyHz;
        
        // 启动录制
        isRecording = true;
        recordStopwatch.Restart();
        
        Debug.Log($"<color=yellow>========== 开始录制 ==========</color>");
        Debug.Log($"  设备 ID: {trackerDeviceId}");
        Debug.Log($"  录制频率: {recordFrequencyHz} Hz ({Time.fixedDeltaTime * 1000f:F2} ms)");
        Debug.Log($"  按 E 键结束录制");
    }
    
    /// <summary>
    /// 停止录制并保存数据
    /// </summary>
    [ContextMenu("停止录制 (E键)")]
    public void StopRecording()
    {
        if (!isRecording)
        {
            Debug.LogWarning("[TrackerRecorder] 当前未在录制");
            return;
        }
        
        // 停止录制
        isRecording = false;
        recordStopwatch.Stop();
        
        // 统计信息
        double recordDuration = recordStopwatch.Elapsed.TotalSeconds;
        int totalFrames = recordedPoses.Count;
        double actualFrameRate = totalFrames / recordDuration;
        
        Debug.Log($"<color=green>========== 录制完成 ==========</color>");
        Debug.Log($"  录制时长: {recordDuration:F2} 秒");
        Debug.Log($"  录制帧数: {totalFrames} 帧");
        Debug.Log($"  实际帧率: {actualFrameRate:F2} Hz (目标: {recordFrequencyHz} Hz)");
        Debug.Log($"  数据大小: ~{totalFrames * 56 / 1024:F2} KB");
        
        // 【可选】保存到 JSON 文件
        // SaveToJSON();
        
        // 【可选】验证数据
        // ValidateRecordedData();
    }
    
    // ==================== 位姿采集 ====================
    
    /// <summary>
    /// 记录当前 Tracker 位姿
    /// </summary>
    private void RecordCurrentPose()
    {
        // 获取 Tracker 当前位姿 (使用现有 API)
        if (!poseLogger.GetTrackerPoseForCalibration(
            trackerDeviceId, 
            out Vector3 positionMm,      // 位置 (mm)
            out Quaternion rotation))    // 姿态 (四元数)
        {
            Debug.LogWarning($"[TrackerRecorder] 帧 {frameCounter}: 无法获取 Tracker 位姿");
            return;
        }
        
        // 创建位姿数据
        TrackerPoseData poseData = new TrackerPoseData
        {
            frameNumber = frameCounter,
            timeStamp = recordStopwatch.ElapsedMilliseconds,
            timeFromStart = recordStopwatch.Elapsed.TotalSeconds,
            
            // 位置 (保持 mm 单位)
            positionX = positionMm.x,
            positionY = positionMm.y,
            positionZ = positionMm.z,
            
            // 姿态 (四元数)
            quaternionX = rotation.x,
            quaternionY = rotation.y,
            quaternionZ = rotation.z,
            quaternionW = rotation.w
        };
        
        // 添加到数组
        recordedPoses.Add(poseData);
        
        // 详细日志 (可选)
        if (verboseLogging && frameCounter % 100 == 0)  // 每 100 帧输出一次
        {
            Debug.Log($"[TrackerRecorder] 帧 {frameCounter}: " +
                     $"Pos=({positionMm.x:F2}, {positionMm.y:F2}, {positionMm.z:F2}) mm, " +
                     $"Quat=({rotation.x:F4}, {rotation.y:F4}, {rotation.z:F4}, {rotation.w:F4})");
        }
        
        frameCounter++;
    }
    
    // ==================== 数据访问 ====================
    
    /// <summary>
    /// 获取录制的位姿数据
    /// </summary>
    public List<TrackerPoseData> GetRecordedPoses()
    {
        return recordedPoses;
    }
    
    /// <summary>
    /// 获取录制帧数
    /// </summary>
    public int GetRecordedFrameCount()
    {
        return recordedPoses.Count;
    }
    
    /// <summary>
    /// 清空录制数据
    /// </summary>
    [ContextMenu("清空录制数据")]
    public void ClearRecordedData()
    {
        recordedPoses.Clear();
        frameCounter = 0;
        Debug.Log("[TrackerRecorder] 录制数据已清空");
    }
}
```

---

### 2. TrackerPoseData.cs (数据结构)

```csharp
using System;
using UnityEngine;

/// <summary>
/// Tracker 单帧位姿数据
/// 存储格式: 位置(mm) + 姿态(四元数)
/// </summary>
[Serializable]
public class TrackerPoseData
{
    // -------------------- 基本信息 -------------------- //
    public int frameNumber;           // 帧序号
    public long timeStamp;            // 时间戳 (毫秒)
    public double timeFromStart;      // 相对录制开始时间 (秒)
    
    // -------------------- 位置 (单位: mm) -------------------- //
    public double positionX;
    public double positionY;
    public double positionZ;
    
    // -------------------- 姿态 (四元数) -------------------- //
    public double quaternionX;
    public double quaternionY;
    public double quaternionZ;
    public double quaternionW;
    
    // ==================== 辅助方法 ====================
    
    /// <summary>
    /// 获取位置向量 (Unity Vector3, mm)
    /// </summary>
    public Vector3 GetPositionMm()
    {
        return new Vector3(
            (float)positionX,
            (float)positionY,
            (float)positionZ
        );
    }
    
    /// <summary>
    /// 获取位置向量 (Unity Vector3, m)
    /// </summary>
    public Vector3 GetPositionM()
    {
        return new Vector3(
            (float)(positionX / 1000.0),
            (float)(positionY / 1000.0),
            (float)(positionZ / 1000.0)
        );
    }
    
    /// <summary>
    /// 获取姿态四元数 (Unity Quaternion)
    /// </summary>
    public Quaternion GetQuaternion()
    {
        return new Quaternion(
            (float)quaternionX,
            (float)quaternionY,
            (float)quaternionZ,
            (float)quaternionW
        );
    }
    
    /// <summary>
    /// 转换为 FrameData (兼容现有回放系统)
    /// </summary>
    public FrameData ToFrameData()
    {
        FrameData frame = new FrameData
        {
            FrameNumber = frameNumber,
            TimeStamp = $"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff}",  // ISO 格式
            UnixTimeStamp = timeStamp,
            
            // 位置数据 (mm)
            Position = new PositionData
            {
                X = positionX,
                Y = positionY,
                Z = positionZ
            },
            
            // 四元数数据
            Quaternion = new QuaternionData
            {
                X = quaternionX,
                Y = quaternionY,
                Z = quaternionZ,
                W = quaternionW
            }
        };
        
        return frame;
    }
}
```

---

### 3. 【可选】JSON 保存功能扩展

```csharp
// 在 TrackerPoseRecorder.cs 中添加

/// <summary>
/// 保存录制数据到 JSON 文件
/// </summary>
private void SaveToJSON()
{
    if (recordedPoses.Count == 0)
    {
        Debug.LogWarning("[TrackerRecorder] 没有数据可保存");
        return;
    }
    
    // 构建完整的捕获数据结构 (兼容现有回放系统)
    RigidBodyCaptureData captureData = new RigidBodyCaptureData
    {
        Metadata = new Metadata
        {
            CollectionTime = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"),
            TotalFrames = recordedPoses.Count,
            RigidBodyId = (int)trackerDeviceId,
            RigidBodyName = $"Tracker_{trackerDeviceId}",
            Description = "手动录制的 Tracker 轨迹",
            Units = new Units
            {
                Position = "mm",
                Velocity = "mm/s",
                Acceleration = "mm/s²",
                Time = "UTC"
            }
        },
        FrameData = new List<FrameData>()
    };
    
    // 转换数据
    foreach (var pose in recordedPoses)
    {
        captureData.FrameData.Add(pose.ToFrameData());
    }
    
    // 序列化为 JSON
    string json = JsonUtility.ToJson(captureData, true);
    
    // 生成文件名
    string fileName = $"TrackerRecording_{trackerDeviceId}_{DateTime.Now:yyyyMMdd_HHmmss}.json";
    string filePath = System.IO.Path.Combine(Application.streamingAssetsPath, fileName);
    
    // 保存文件
    System.IO.File.WriteAllText(filePath, json);
    
    Debug.Log($"<color=cyan>[TrackerRecorder] 数据已保存</color>");
    Debug.Log($"  文件路径: {filePath}");
    Debug.Log($"  文件大小: {new System.IO.FileInfo(filePath).Length / 1024:F2} KB");
}

/// <summary>
/// 验证录制数据
/// </summary>
private void ValidateRecordedData()
{
    if (recordedPoses.Count == 0)
    {
        Debug.LogWarning("[TrackerRecorder] 没有数据可验证");
        return;
    }
    
    Debug.Log($"<color=cyan>[TrackerRecorder] 数据验证</color>");
    
    // 统计位置范围
    Vector3 minPos = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
    Vector3 maxPos = new Vector3(float.MinValue, float.MinValue, float.MinValue);
    
    foreach (var pose in recordedPoses)
    {
        Vector3 pos = pose.GetPositionMm();
        minPos = Vector3.Min(minPos, pos);
        maxPos = Vector3.Max(maxPos, pos);
    }
    
    Debug.Log($"  位置范围:");
    Debug.Log($"    X: [{minPos.x:F2}, {maxPos.x:F2}] mm (跨度: {maxPos.x - minPos.x:F2} mm)");
    Debug.Log($"    Y: [{minPos.y:F2}, {maxPos.y:F2}] mm (跨度: {maxPos.y - minPos.y:F2} mm)");
    Debug.Log($"    Z: [{minPos.z:F2}, {maxPos.z:F2}] mm (跨度: {maxPos.z - minPos.z:F2} mm)");
    
    // 检查时间戳连续性
    bool timeStampConsistent = true;
    for (int i = 1; i < recordedPoses.Count; i++)
    {
        double deltaTime = recordedPoses[i].timeFromStart - recordedPoses[i - 1].timeFromStart;
        double expectedDelta = 1.0 / recordFrequencyHz;
        
        if (Mathf.Abs((float)(deltaTime - expectedDelta)) > expectedDelta * 0.1f)  // 10% 容差
        {
            Debug.LogWarning($"  帧 {i}: 时间间隔异常 ({deltaTime * 1000:F2}ms, 期望 {expectedDelta * 1000:F2}ms)");
            timeStampConsistent = false;
        }
    }
    
    if (timeStampConsistent)
    {
        Debug.Log($"  <color=green>时间戳连续性: 通过</color>");
    }
    else
    {
        Debug.LogWarning($"  <color=yellow>时间戳连续性: 存在异常</color>");
    }
}
```

---

## 使用流程

### 1. 场景设置

```csharp
// 1. 在 Unity 场景中创建空物体
GameObject recorder = new GameObject("TrackerPoseRecorder");

// 2. 添加 TrackerPoseRecorder 组件
TrackerPoseRecorder recorderScript = recorder.AddComponent<TrackerPoseRecorder>();

// 3. 配置参数 (在 Inspector 中)
//    - Tracker Device Id: 2 (你的 Tracker ID)
//    - Record Frequency Hz: 125 (与回放频率一致)
//    - Auto Find Components: ✓ (勾选)
```

### 2. 录制操作

```
1. 运行 Unity Play 模式
2. 将 Tracker 移动到起始位置
3. 按 S 键 → 开始录制
4. 移动 Tracker 完成轨迹
5. 按 E 键 → 结束录制
6. 查看 Console 日志确认录制成功
```

### 3. 数据使用

```csharp
// 方法1: 直接获取数组数据
List<TrackerPoseData> poses = recorderScript.GetRecordedPoses();

// 方法2: 保存到 JSON 文件 (需实现 SaveToJSON)
// 文件自动保存到 StreamingAssets 目录

// 方法3: 转换为 RigidBodyCaptureData 供回放系统使用
RigidBodyCaptureData captureData = ConvertToRigidBodyCaptureData(poses);
```

---

## 关键技术点

### 1. 使用现有 API

```csharp
// ✅ 使用 ViveTrackerPoseLogger.GetTrackerPoseForCalibration()
// 优点: 
//   - 已验证的稳定接口
//   - 单位统一 (位置 mm, 姿态四元数)
//   - 无需重复实现 SteamVR 接口

bool success = poseLogger.GetTrackerPoseForCalibration(
    trackerDeviceId,       // 输入: Tracker ID
    out Vector3 positionMm,   // 输出: 位置 (mm)
    out Quaternion rotation   // 输出: 姿态 (四元数)
);
```

### 2. 频率控制

```csharp
// ✅ 使用 FixedUpdate + Time.fixedDeltaTime 精确控制采集频率
void Start()
{
    Time.fixedDeltaTime = 1.0f / recordFrequencyHz;  // 125Hz = 0.008s
}

void FixedUpdate()
{
    if (isRecording)
    {
        RecordCurrentPose();  // 每 8ms 执行一次
    }
}
```

### 3. 数据存储

```csharp
// ✅ 使用 List<TrackerPoseData> 动态数组
// 优点:
//   - 自动扩容, 无需预定义大小
//   - 内存占用: 56字节/帧 (7个double + 1个int + 1个long)
//   - 1000帧 ≈ 56KB

private List<TrackerPoseData> recordedPoses = new List<TrackerPoseData>();
```

### 4. 兼容性设计

```csharp
// ✅ 可转换为 FrameData 格式, 兼容现有回放系统
public FrameData ToFrameData()
{
    // 保持相同的数据结构和单位
    // 可直接用于 RigidBodyServojController 回放
}
```

---

## 内存与性能估算

### 内存占用

```
单帧数据:
  - int frameNumber: 4 字节
  - long timeStamp: 8 字节
  - double timeFromStart: 8 字节
  - double positionX/Y/Z: 24 字节 (3×8)
  - double quaternionX/Y/Z/W: 32 字节 (4×8)
  - 对象开销: ~8 字节
  总计: ~84 字节/帧

录制 1000 帧 (125Hz, 8秒):
  - 数据: 84 KB
  - List 开销: ~4 KB
  - 总计: ~88 KB

录制 10000 帧 (125Hz, 80秒):
  - 数据: 840 KB
  - List 开销: ~40 KB
  - 总计: ~880 KB (< 1 MB)
```

### CPU 占用

```
- RecordCurrentPose(): <0.5% (125Hz 下单核)
- GetTrackerPoseForCalibration(): <0.1% (SteamVR API)
- List.Add(): <0.01% (动态扩容偶尔发生)
```

---

## 待确认事项

### 请检查以下设计决策:

1. **录制频率**: 默认 125Hz 是否合适? (需与回放频率一致)
2. **Tracker ID**: 默认 ID=2 是否正确?
3. **JSON 保存**: 是否需要自动保存功能? (目前可选)
4. **数据验证**: 是否需要实时验证位姿数据有效性?
5. **UI 界面**: 是否需要 Unity UI 界面? (目前仅键盘控制)
6. **数据格式**: 是否需要同时保存速度/加速度? (目前仅位置+姿态)

---

## 扩展建议

### 可选功能

1. **UI 界面**: 添加录制按钮、进度条、状态显示
2. **实时预览**: 在 Scene 视图中可视化录制轨迹
3. **数据分析**: 统计速度、加速度、旋转角速度
4. **数据编辑**: 录制后可删除/修剪帧
5. **多 Tracker 支持**: 同时录制多个 Tracker
6. **自动保存**: 录制结束后自动保存 JSON

### 与回放系统集成

```csharp
// 录制完成后直接回放
TrackerPoseRecorder recorder = GetComponent<TrackerPoseRecorder>();
RigidBodyServojController player = GetComponent<RigidBodyServojController>();

// 停止录制
recorder.StopRecording();

// 转换数据
RigidBodyCaptureData data = ConvertToRigidBodyCaptureData(
    recorder.GetRecordedPoses()
);

// 加载到回放系统
player.LoadFromData(data);

// 开始回放
player.StartPlayback();
```

---

## 问题与注意事项

### ⚠️ 潜在问题

1. **Tracker 丢失**: 如何处理录制过程中 Tracker 信号丢失?
   - 建议: 跳过无效帧, 记录警告日志

2. **录制时长**: 长时间录制内存占用?
   - 建议: 添加最大帧数限制 (如 50000 帧 ≈ 400 秒 @ 125Hz)

3. **数据同步**: 如何确保时间戳准确?
   - 建议: 使用 Stopwatch 高精度计时器 (已实现)

4. **文件保存失败**: StreamingAssets 目录不存在?
   - 建议: 保存前检查并创建目录

---

**请检查此框架是否符合您的需求，确认后我可以生成完整的 .cs 代码文件！**

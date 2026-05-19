# Servoj 批量脚本发送方案设计

**版本**: v1.1  
**日期**: 2026-01-29  
**状态**: ✅ 已实现  
**目标**: 将 servoj 轨迹回放从**单帧实时发送**改为**一次性脚本批量发送**

---

## 📋 方案对比

| 特性 | 单帧实时发送（当前） | 批量脚本发送（目标） |
|------|---------------------|---------------------|
| **发送方式** | 协程循环，每帧发送一条命令 | 构建完整脚本，一次性发送 |
| **网络开销** | 高（每帧一次TCP通信） | 低（仅一次TCP通信） |
| **时序精度** | 受Unity帧率和网络延迟影响 | 机器人内部精确执行 |
| **实时控制** | 可暂停/停止/调速 | 发送后无法中断（需发stopj） |
| **脚本大小限制** | 无 | UR控制器缓冲区限制 |
| **适用场景** | 实时跟踪、视觉伺服 | 离线轨迹回放、精确复现 |

---

## 🏗️ 系统架构变更

### 当前架构（单帧发送）

```
┌─────────────────────────────────────────────────────────────────┐
│  RigidBodyServojController.PlaybackCoroutine()                  │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  while (currentFrameIndex < frameCount)                   │  │
│  │  {                                                        │  │
│  │      command = GenerateServojCommand(frame[i]);           │  │
│  │      SendCommandToUR(command);      ← 每帧发送一次        │  │
│  │      yield return WaitForSeconds(1/125Hz);                │  │
│  │  }                                                        │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### 目标架构（批量脚本发送）

```
┌─────────────────────────────────────────────────────────────────┐
│  RigidBodyServojScriptGenerator (新增)                          │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  Step 1: 遍历所有帧，生成位姿数组                          │  │
│  │  Step 2: 构建完整URScript脚本                             │  │
│  │  Step 3: 一次性发送到UR控制器    ← 仅发送一次             │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📝 URScript 脚本结构设计

### 目标脚本格式

```python
def trajectory_replay():
  # 初始化：获取当前关节角度作为逆运动学参考
  qnear = get_actual_joint_positions()
  
  # ========== 第一部分：定义所有目标位姿 ==========
  # P{index} = p[x(m), y(m), z(m), rx(rad), ry(rad), rz(rad)]
  P0 = p[0.3521, -0.1234, 0.4567, 1.2345, -0.5678, 2.1234]
  P1 = p[0.3525, -0.1230, 0.4570, 1.2348, -0.5675, 2.1237]
  P2 = p[0.3530, -0.1225, 0.4575, 1.2352, -0.5670, 2.1240]
  ...
  P999 = p[0.4521, -0.0234, 0.5567, 1.3345, -0.4678, 2.2234]
  
  # ========== 第二部分：执行servoj运动序列 ==========
  # servoj(关节角度, t=时间步长, lookahead_time=前瞻时间, gain=增益)
  servoj(get_inverse_kin(P0, qnear=get_actual_joint_positions()), t=0.008, lookahead_time=0.1, gain=300)
  servoj(get_inverse_kin(P1, qnear=get_actual_joint_positions()), t=0.008, lookahead_time=0.1, gain=300)
  servoj(get_inverse_kin(P2, qnear=get_actual_joint_positions()), t=0.008, lookahead_time=0.1, gain=300)
  ...
  servoj(get_inverse_kin(P999, qnear=get_actual_joint_positions()), t=0.008, lookahead_time=0.1, gain=300)
  
  # 结束：平滑停止
  stopl(5)
end

# 调用函数执行轨迹
trajectory_replay()
```

### 关键参数说明

| 参数 | 含义 | 推荐值 | 说明 |
|------|------|--------|------|
| `t` | 时间步长 | `1/频率` | 125Hz → 0.008s |
| `lookahead_time` | 前瞻时间 | 0.06~0.1 | 轨迹平滑度 |
| `gain` | 伺服增益 | 300~500 | 响应速度 |
| `qnear` | IK参考 | 当前关节角 | 确保解连续 |

---

## 🔄 数据处理流程

### 流程图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         CSV 轨迹文件                                     │
│              (X_mm, Y_mm, Z_mm, QX, QY, QZ, QW, ...)                    │
└─────────────────────────────┬───────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  Step 1: 加载CSV数据                                                    │
│  CSVCaptureReader.LoadFromCSV() → List<FrameData>                       │
└─────────────────────────────┬───────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  Step 2: 坐标转换与校正（遍历每帧）                                       │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │  for each frame:                                                   │ │
│  │    2.1 提取位置(mm) + 四元数                                        │ │
│  │    2.2 应用Tracker本地偏移（可选）                                   │ │
│  │    2.3 应用Kabsch点云对齐（可选，仅位置）                            │ │
│  │    2.4 手眼标定坐标变换 SteamVR → UR Base                          │ │
│  │    2.5 旋转矢量连续性校正                                           │ │
│  │    → 输出: (x_m, y_m, z_m, rx_rad, ry_rad, rz_rad)                 │ │
│  └────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────┬───────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  Step 3: 构建URScript脚本                                               │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │  StringBuilder scriptBuilder                                       │ │
│  │  3.1 写入函数头: "def trajectory_replay():"                        │ │
│  │  3.2 写入初始化: "qnear = get_actual_joint_positions()"           │ │
│  │  3.3 写入位姿定义: "P{i} = p[x, y, z, rx, ry, rz]"                │ │
│  │  3.4 写入servoj指令: "servoj(get_inverse_kin(P{i}, ...), ...)"    │ │
│  │  3.5 写入结束: "stopl(5)\nend\ntrajectory_replay()"               │ │
│  └────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────┬───────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  Step 4: 发送脚本                                                       │
│  byte[] scriptBytes = Encoding.UTF8.GetBytes(script)                   │
│  UR_Control_Data.command = scriptBytes                                  │
│  UR_Control_Data.manual_send_active = true                             │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 🔧 核心代码逻辑设计

### 3.1 新增类：RigidBodyServojScriptGenerator

```csharp
/// <summary>
/// Servoj批量脚本生成器
/// 功能：将完整轨迹转换为URScript脚本，一次性发送执行
/// </summary>
public class RigidBodyServojScriptGenerator
{
    /// <summary>
    /// 从CSV轨迹数据生成完整的URScript脚本
    /// </summary>
    public static string GenerateTrajectoryScript(
        RigidBodyCaptureData captureData,      // CSV加载的轨迹数据
        ServojScriptParameters parameters,      // 脚本参数
        bool useTcpMode = false                 // 是否使用TCP直接回放
    );
    
    /// <summary>
    /// 发送脚本到UR控制器
    /// </summary>
    public static void SendScriptToUR(string script);
}
```

### 3.2 脚本参数结构

```csharp
[Serializable]
public struct ServojScriptParameters
{
    public float SendFrequencyHz;      // 发送频率 → 计算时间步长t
    public float LookAheadTime;        // 前瞻时间
    public float Gain;                 // 控制增益
    public int PointStep;              // 点采样步长（1=全部，2=隔点）
    public bool EnableCoordinateTransform;  // 启用坐标转换
    public bool EnableKabschAlignment;      // 启用Kabsch校正
    public bool EnableRotationContinuity;   // 启用旋转连续性校正
    
    public static ServojScriptParameters Default => new ServojScriptParameters
    {
        SendFrequencyHz = 125f,
        LookAheadTime = 0.1f,
        Gain = 300f,
        PointStep = 1,
        EnableCoordinateTransform = true,
        EnableKabschAlignment = false,
        EnableRotationContinuity = true
    };
}
```

### 3.3 脚本生成核心逻辑

```csharp
public static string GenerateTrajectoryScript(
    RigidBodyCaptureData captureData,
    ServojScriptParameters parameters,
    bool useTcpMode)
{
    StringBuilder sb = new StringBuilder();
    List<FrameData> frames = captureData.FrameData;
    int frameCount = frames.Count;
    
    // 计算时间步长
    double timeStep = 1.0 / parameters.SendFrequencyHz;
    
    // ========== 脚本头部 ==========
    sb.AppendLine("def trajectory_replay():");
    sb.AppendLine("  qnear = get_actual_joint_positions()");
    sb.AppendLine("");
    
    // ========== 重置旋转连续性状态 ==========
    RigidBodyServojCommandGenerator.ResetRotationContinuityState();
    
    // ========== 第一遍：生成位姿定义 ==========
    List<(Vector3 pos, Vector3 rot)> processedPoses = new List<(Vector3, Vector3)>();
    
    for (int i = 0; i < frameCount; i += parameters.PointStep)
    {
        FrameData frame = frames[i];
        Vector3 posUr_m;
        Vector3 rotUr_rad;
        
        if (useTcpMode && frame.HasTcpData)
        {
            // TCP直接模式
            posUr_m = new Vector3(
                (float)frame.TcpPose.X,
                (float)frame.TcpPose.Y,
                (float)frame.TcpPose.Z);
            rotUr_rad = new Vector3(
                (float)frame.TcpPose.RX,
                (float)frame.TcpPose.RY,
                (float)frame.TcpPose.RZ);
        }
        else
        {
            // Tracker + 坐标转换模式
            // 复用现有的 RigidBodyServojCommandGenerator 中的转换逻辑
            ProcessFramePose(frame, parameters, out posUr_m, out rotUr_rad);
        }
        
        // 旋转连续性校正
        if (parameters.EnableRotationContinuity)
        {
            rotUr_rad = EnsureRotationVectorContinuity(rotUr_rad);
        }
        
        processedPoses.Add((posUr_m, rotUr_rad));
        
        // 写入位姿定义
        sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
            "  P{0} = p[{1:F6}, {2:F6}, {3:F6}, {4:F6}, {5:F6}, {6:F6}]",
            i, posUr_m.x, posUr_m.y, posUr_m.z,
            rotUr_rad.x, rotUr_rad.y, rotUr_rad.z));
    }
    
    sb.AppendLine("");
    
    // ========== 第二遍：生成servoj指令 ==========
    for (int i = 0; i < frameCount; i += parameters.PointStep)
    {
        sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
            "  servoj(get_inverse_kin(P{0}, qnear=get_actual_joint_positions()), " +
            "t={1:F6}, lookahead_time={2:F4}, gain={3:F0})",
            i, timeStep, parameters.LookAheadTime, parameters.Gain));
    }
    
    // ========== 脚本尾部 ==========
    sb.AppendLine("");
    sb.AppendLine("  stopl(5)");  // 平滑停止
    sb.AppendLine("end");
    sb.AppendLine("");
    sb.AppendLine("trajectory_replay()");  // 调用函数执行
    
    return sb.ToString();
}
```

---

## ⚠️ 注意事项与限制

### 4.1 UR控制器脚本大小限制

| UR型号 | 脚本缓冲区大小 | 约可容纳点数 |
|--------|---------------|-------------|
| UR3/5/10 | ~2MB | ~10,000-20,000点 |
| UR3e/5e/10e | ~4MB | ~20,000-40,000点 |

**计算公式**：
- 每个位姿定义：约 70 字节
- 每个servoj指令：约 100 字节
- 总计每点：约 170 字节
- 10,000点 ≈ 1.7MB

### 4.2 长轨迹分段发送策略

```csharp
const int MAX_POINTS_PER_SCRIPT = 8000;  // 安全阈值

if (frameCount > MAX_POINTS_PER_SCRIPT)
{
    // 分段发送
    int segments = (frameCount + MAX_POINTS_PER_SCRIPT - 1) / MAX_POINTS_PER_SCRIPT;
    for (int seg = 0; seg < segments; seg++)
    {
        int start = seg * MAX_POINTS_PER_SCRIPT;
        int end = Math.Min(start + MAX_POINTS_PER_SCRIPT, frameCount);
        string segmentScript = GenerateSegmentScript(frames, start, end, parameters);
        SendScriptToUR(segmentScript);
        // 等待当前段执行完成...
    }
}
```

### 4.3 脚本发送后的控制

| 操作 | 方法 |
|------|------|
| **紧急停止** | 发送 `stopj(2)\n` 或 `stopl(5)\n` |
| **暂停** | 发送 `pause program\n`（仅PolyScope程序） |
| **无法暂停** | 脚本执行中无法暂停，只能停止 |

### 4.4 与单帧发送的兼容

保留现有的 `RigidBodyServojController` 用于需要实时控制的场景，新增 `RigidBodyServojScriptController` 用于批量发送场景。两者共用：
- `CSVCaptureReader` - 数据加载
- `RigidBodyServojCommandGenerator` - 坐标转换逻辑（提取为静态方法复用）

---

## 📊 Inspector 配置设计

```csharp
[Header("=== 脚本生成模式 ===")]
[Tooltip("使用批量脚本发送（一次性发送完整轨迹）\n" +
         "✓ 勾选：生成URScript脚本一次性发送\n" +
         "✗ 不勾选：使用原有单帧实时发送")]
public bool useBatchScriptMode = true;

[Header("=== 批量脚本参数 ===")]
[Tooltip("点采样步长\n1=使用所有点\n2=隔点采样\n...")]
[Range(1, 10)]
public int pointSamplingStep = 1;

[Tooltip("每个脚本最大点数（超过则分段发送）")]
[Range(1000, 10000)]
public int maxPointsPerScript = 8000;

[Tooltip("是否在发送前保存脚本到文件（用于调试）")]
public bool saveScriptToFile = false;
```

---

## 🚀 执行流程

### 用户操作流程

```
1. 配置参数（Inspector）
   - 设置CSV路径
   - 选择回放模式（Tracker/TCP）
   - 启用/禁用Kabsch校正
   - 设置servoj参数（频率、增益等）

2. 按 L 键 或 点击"加载并执行"按钮
   ↓
3. 系统执行：
   a) 加载CSV数据
   b) 遍历所有帧，进行坐标转换
   c) 构建URScript脚本
   d) （可选）保存脚本到文件
   e) 一次性发送到UR

4. UR机器人执行轨迹
   ↓
5. 执行完成（或按紧急停止）
```

### 日志输出

```
[批量脚本] 开始生成轨迹脚本...
[批量脚本] 加载CSV: 3918帧
[批量脚本] 坐标转换模式: Tracker + 手眼标定
[批量脚本] Kabsch校正: 已启用 (RMSE=0.0023)
[批量脚本] 处理完成: 3918个位姿点
[批量脚本] 脚本大小: 665,060 字节
[批量脚本] 发送到UR控制器...
[批量脚本] ✓ 发送成功！预计执行时间: 31.3秒
```

---

## 📁 文件结构

```
Assets/Scripts/TrajectoryReplay/
├── RigidBodyServojController.cs          # 回放控制器（已修改：支持两种模式切换）
├── RigidBodyServojCommandGenerator.cs    # 单帧指令生成（保留，供单帧模式和视觉伺服使用）
├── RigidBodyServojScriptGenerator.cs     # ✅ 新增：批量脚本生成器
└── CSVCaptureReader.cs                   # CSV读取（复用）
```

---

## ✅ 实现检查清单

- [x] 创建 `RigidBodyServojScriptGenerator.cs` - 脚本生成核心逻辑
- [x] 修改 `RigidBodyServojController.cs` - 添加批量模式支持
- [x] 复用坐标转换逻辑（独立实现，不影响单帧发送）
- [x] 实现分段发送API（GenerateSegmentedScripts）
- [x] 添加脚本保存到文件功能（ContextMenu: 仅生成脚本保存到文件）
- [x] 添加紧急停止功能（X键 / ContextMenu: 紧急停止）
- [ ] 测试不同轨迹长度的执行效果
- [ ] 验证与现有单帧发送模式的兼容性

---

## 🔑 使用说明

### Inspector 配置

1. **发送模式选择**
   - `useBatchScriptMode = true` → 批量脚本发送
   - `useBatchScriptMode = false` → 单帧实时发送（默认）

2. **批量脚本参数**
   - `pointSamplingStep`: 采样步长（1=全部点）
   - `maxPointsPerScript`: 最大点数限制
   - `saveScriptToFile`: 保存脚本用于调试
   - `scriptSaveDirectory`: 脚本保存目录

### 快捷键

| 按键 | 功能 |
|------|------|
| L | 加载CSV并开始回放 |
| P | 暂停/继续（仅单帧模式） |
| X | 停止/紧急停止 |

### ContextMenu 命令

- **加载CSV并回放 (L键)** - 标准回放入口
- **仅生成脚本保存到文件** - 调试用，不发送到机器人
- **紧急停止 (X键)** - 发送 stopj(2) 命令

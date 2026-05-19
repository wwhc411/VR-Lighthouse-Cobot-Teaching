# Nokov官方Unity插件功能分析文档

> **文档版本**: 1.0  
> **生成日期**: 2026年1月21日  
> **插件路径**: `Assets/Nokov/`

---

## 📋 目录

1. [插件概述](#插件概述)
2. [插件结构](#插件结构)
3. [核心组件详解](#核心组件详解)
   - [StreamingClient](#1-streamingclient数据流客户端)
   - [NokovRigidBody](#2-nokovrigidbody刚体跟踪器)
   - [NokovSkeletonAnimator](#3-nokovskeletonanimator骨骼动画器)
   - [NokovSDKClient](#4-nokovsdkclientsdk封装层)
4. [数据结构定义](#数据结构定义)
5. [枚举类型](#枚举类型)
6. [工具类](#工具类)
7. [预制体说明](#预制体说明)
8. [使用场景与代码示例](#使用场景与代码示例)
9. [坐标系转换](#坐标系转换)
10. [与自定义NokovSDKManager对比](#与自定义nokovsdkmanager对比)
11. [最佳实践](#最佳实践)

---

## 插件概述

### 功能简介

这是**Nokov官方**提供的Unity动作捕捉SDK插件，用于接收和处理Nokov光学动捕系统（Mars/XING/XINGYING系列）的实时数据流。

### 核心功能

| 功能 | 描述 |
|------|------|
| ✅ **刚体跟踪** | 实时获取刚体的位置和旋转数据 |
| ✅ **骨骼动画** | 支持骨骼数据重定向到Unity Humanoid Avatar |
| ✅ **Marker可视化** | 在场景中绘制已标记和未标记的Marker点 |
| ✅ **多平台支持** | Windows x64、Android、macOS |
| ✅ **坐标系转换** | 自动将Nokov坐标系转换为Unity坐标系 |
| ✅ **单位转换** | 支持毫米/厘米/米单位自动转换 |

### 支持平台

| 平台 | 库文件 | 路径 |
|------|--------|------|
| Windows x64 | `CSNokovSDK.dll`, `nokov_sdk.dll` | `Plugins/x86_64/` |
| Android ARM64 | `libnokov_sdk.so` | `Plugins/Android/arm64-v8a/` |
| Android ARMv7 | `libnokov_sdk.so` | `Plugins/Android/armeabi-v7a/` |
| macOS | `libnokov_sdk.dylib` | `Plugins/macOS/` |

---

## 插件结构

```
Assets/Nokov/
│
├── Editor/                           # Unity编辑器资源
│   └── Materials/                    # Marker可视化材质
│       ├── MarkerMaterial.mat
│       └── MarkerShader.shader
│   └── Meshes/                       # Marker可视化Mesh
│       └── MarkerMesh.fbx
│
├── Materials/                        # 运行时材质
│   ├── Nokov.mat                    # Logo材质
│   ├── rigibody-1.mat               # 刚体材质1
│   ├── rigidbody-2.mat              # 刚体材质2
│   ├── rigidbody-3.mat              # 刚体材质3
│   └── TransparentPlane.mat         # 透明平面材质
│
├── Plugins/                          # SDK核心库
│   ├── x86_64/                      # Windows 64位
│   │   ├── CSNokovSDK.dll           # C# SDK封装
│   │   ├── CSNokovSDK.xml           # XML文档
│   │   └── nokov_sdk.dll            # 原生SDK
│   ├── Android/                     # Android库
│   │   ├── arm64-v8a/libnokov_sdk.so
│   │   └── armeabi-v7a/libnokov_sdk.so
│   ├── macOS/                       # macOS库
│   │   └── libnokov_sdk.dylib
│   └── Managed/NokovSDKLib/         # C#托管代码
│       ├── Client.cs                # SDK客户端封装
│       ├── PluginVersion.cs         # 版本信息
│       └── Utility.cs               # 工具类（平滑滤波）
│
├── Prefabs/                          # 预制体
│   ├── Client - Nokov.prefab       # 数据流客户端预制体
│   ├── RigidBody - Nokov.prefab    # 刚体预制体
│   └── Retargeted Skeleton - Nokov.prefab  # 骨骼重定向预制体
│
├── Scenes/                           # 示例场景
│   └── NokovExample.unity           # 使用示例
│
└── Scripts/                          # 核心脚本
    ├── StreamingClient.cs           # 数据流客户端（1090行）
    ├── NokovRigidBody.cs            # 刚体跟踪组件（140行）
    └── NokovSkeletonAnimator.cs     # 骨骼动画组件（464行）
```

---

## 核心组件详解

### 1. StreamingClient（数据流客户端）

**文件位置**: `Scripts/StreamingClient.cs`  
**代码行数**: 1090行  
**功能**: 插件的核心管理器，负责连接Nokov服务器、接收数据流、管理所有数据状态

#### Inspector可配置属性

| 属性 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `ServerIp` | `string` | `10.1.1.198` | Nokov服务器IP地址 |
| `Csys` | `SkeletonCoordinate` | `World` | 骨骼坐标系模式 |
| `LenthUnit` | `E_LengthUnit` | `M` | 长度单位 |
| `UpAxis` | `E_UpAxis` | `Y` | 上轴方向 |
| `DrawMarkers` | `bool` | `false` | 是否在场景中绘制已标记Marker |
| `DrawUnlabeledMarkers` | `bool` | `false` | 是否在场景中绘制未标记Marker |
| `BoneNamingConvention` | `NokovBoneNameConvention` | `XingYing` | 骨骼命名规范 |
| `fpsDisplay` | `Text` | `null` | 帧率显示UI组件 |

#### 公共方法

##### 静态方法

```csharp
/// <summary>
/// 查找场景中的默认StreamingClient实例
/// </summary>
/// <returns>找到的第一个StreamingClient，未找到返回null</returns>
public static StreamingClient FindDefaultClient()
```

```csharp
/// <summary>
/// 检查IP地址格式是否有效
/// </summary>
/// <param name="strJudgeString">待检查的IP字符串</param>
/// <returns>IP格式是否有效</returns>
public static bool CheckIPFormat(string strJudgeString)
```

```csharp
/// <summary>
/// 将Nokov四元数转换为Unity四元数
/// </summary>
/// <param name="x">四元数X分量</param>
/// <param name="y">四元数Y分量</param>
/// <param name="z">四元数Z分量</param>
/// <param name="w">四元数W分量</param>
/// <param name="upAxis">上轴方向</param>
/// <returns>Unity四元数</returns>
public static Quaternion NokovToUnityQuaternion(float x, float y, float z, float w, E_UpAxis upAxis)
```

##### 刚体相关方法

```csharp
/// <summary>
/// 通过刚体名称获取最新的刚体状态
/// </summary>
/// <param name="rigidBodyName">刚体名称（与Nokov软件中一致）</param>
/// <returns>刚体状态，未找到返回null</returns>
public NokovRigidBodyState GetLatestRigidBodyState(String rigidBodyName)
```

```csharp
/// <summary>
/// 通过ID获取刚体定义
/// </summary>
/// <param name="rigidBodyId">刚体ID</param>
/// <returns>刚体定义，未找到返回null</returns>
public NokovRigidBodyDefinition GetRigidBodyDefinitionById(Int32 rigidBodyId)
```

```csharp
/// <summary>
/// 通过名称获取刚体定义
/// </summary>
/// <param name="rigidBodyName">刚体名称</param>
/// <returns>刚体定义，未找到返回null</returns>
public NokovRigidBodyDefinition GetRigidBodyDefinitionByName(String rigidBodyName)
```

##### 骨骼相关方法

```csharp
/// <summary>
/// 通过骨骼ID获取最新的骨骼状态
/// </summary>
/// <param name="skeletonId">骨骼ID</param>
/// <returns>骨骼状态，未找到返回null</returns>
public NokovSkeletonState GetLatestSkeletonState(Int32 skeletonId)
```

```csharp
/// <summary>
/// 通过ID获取骨骼定义
/// </summary>
/// <param name="skeletonId">骨骼ID</param>
/// <returns>骨骼定义，未找到返回null</returns>
public NokovSkeletonDefinition GetSkeletonDefinitionById(Int32 skeletonId)
```

```csharp
/// <summary>
/// 通过名称获取骨骼定义
/// </summary>
/// <param name="skeletonAssetName">骨骼资产名称</param>
/// <returns>骨骼定义，未找到返回null</returns>
public NokovSkeletonDefinition GetSkeletonDefinitionByName(string skeletonAssetName)
```

##### Marker相关方法

```csharp
/// <summary>
/// 获取所有已标记Marker的状态列表
/// </summary>
/// <returns>Marker状态列表</returns>
public List<NokovMarkerState> GetLatestMarkerStates()
```

```csharp
/// <summary>
/// 获取所有未标记Marker的状态列表
/// </summary>
/// <returns>未标记Marker状态列表</returns>
public List<NokovMarkerState> GetLatestUnlabeledMarkerStates()
```

##### 坐标转换方法

```csharp
/// <summary>
/// 将Nokov位置坐标转换为Unity坐标
/// 自动处理坐标系翻转和单位转换
/// </summary>
/// <param name="xPos">X坐标（毫米）</param>
/// <param name="yPos">Y坐标（毫米）</param>
/// <param name="zPos">Z坐标（毫米）</param>
/// <returns>Unity Vector3坐标</returns>
public Vector3 NokovToUnityTranslation(float xPos, float yPos, float zPos)
```

##### 数据刷新方法

```csharp
/// <summary>
/// 刷新数据定义（刚体、骨骼等）
/// </summary>
public void UpdateDefinitions()
```

```csharp
/// <summary>
/// 检查是否需要刷新刚体数据
/// </summary>
public bool GetNeedRefreshRigid()

/// <summary>
/// 重置刚体刷新标志
/// </summary>
public void SetNeedRefreshRigid()

/// <summary>
/// 检查是否需要刷新骨骼数据
/// </summary>
public bool GetNeedRefreshSkeleton()

/// <summary>
/// 重置骨骼刷新标志
/// </summary>
public void SetNeedRefreshSkeleton()
```

##### 线程安全方法

```csharp
/// <summary>
/// 进入帧数据更新锁
/// </summary>
public void _EnterFrameDataUpdateLock()

/// <summary>
/// 退出帧数据更新锁
/// </summary>
public void _ExitFrameDataUpdateLock()
```

---

### 2. NokovRigidBody（刚体跟踪器）

**文件位置**: `Scripts/NokovRigidBody.cs`  
**代码行数**: 140行  
**功能**: 将Nokov刚体的位姿数据实时应用到Unity GameObject

#### Inspector可配置属性

| 属性 | 类型 | 说明 |
|------|------|------|
| `StreamingClient` | `StreamingClient` | 数据源客户端（可自动查找） |
| `RigidBodyName` | `string` | 刚体名称（必须与Nokov软件中一致） |
| `trackingType` | `TrackingType` | 跟踪模式 |
| `DataMix` | `bool` | 是否启用数据混合模式 |

#### 跟踪模式枚举

```csharp
public enum TrackingType
{
    /// <summary>
    /// 同时应用位置和旋转
    /// </summary>
    RotationAndPosition,
    
    /// <summary>
    /// 仅应用旋转
    /// </summary>
    RotationOnly,
    
    /// <summary>
    /// 仅应用位置
    /// </summary>
    PositionOnly
}
```

#### 公共方法

```csharp
/// <summary>
/// 尝试获取基础四元数（首帧捕获的旋转值）
/// </summary>
/// <param name="q">输出的四元数</param>
/// <returns>是否成功获取（首帧之后返回true）</returns>
public bool TryGetBaseQuat(ref Quaternion q)
```

#### 工作流程

1. `Start()` - 自动查找或使用指定的StreamingClient
2. `Update()` - 每帧调用UpdatePose()更新位姿
3. `OnBeforeRender()` - VR模式下在渲染前更新（降低延迟）

#### 数据混合模式（DataMix）

当`DataMix=true`时：
- 首帧记录初始旋转作为基准
- 之后仅跟踪位置变化
- 适用于需要保持初始朝向的场景

---

### 3. NokovSkeletonAnimator（骨骼动画器）

**文件位置**: `Scripts/NokovSkeletonAnimator.cs`  
**代码行数**: 464行  
**功能**: 将Nokov骨骼数据重定向到Unity Humanoid Avatar，驱动角色动画

#### Inspector可配置属性

| 属性 | 类型 | 说明 |
|------|------|------|
| `StreamingClient` | `StreamingClient` | 数据源客户端 |
| `SkeletonAssetName` | `string` | 骨骼资产名称（与Nokov软件中一致） |
| `DestinationAvatar` | `Avatar` | 目标角色的Humanoid Avatar |

#### 工作原理

1. **初始化阶段**（Start）:
   - 获取骨骼定义
   - 创建临时骨骼层级结构
   - 建立Mecanim骨骼映射
   - 创建源Avatar和目标Avatar的PoseHandler

2. **运行时阶段**（Update）:
   - 获取最新骨骼状态
   - 更新临时骨骼对象的位姿
   - 通过HumanPoseHandler进行骨骼重定向
   - 将重定向后的Pose应用到目标角色

#### 支持的骨骼命名规范

| 规范 | 说明 |
|------|------|
| `XingYing` | 星影标准骨骼命名 |
| `XingYingWithHand` | 星影+手部骨骼 |
| `MARKERLESS` | 无标记动捕骨骼 |
| `VRBody` | VR身体追踪骨骼 |
| `NoConvention` | 自定义命名 |

---

### 4. NokovSDKClient（SDK封装层）

**文件位置**: `Plugins/Managed/NokovSDKLib/Client.cs`  
**代码行数**: 363行  
**功能**: C# SDK底层封装，处理与原生DLL的P/Invoke交互

#### 类层级结构

```
SDKClient (抽象基类)
    └── NokovSDKClient (具体实现)
```

#### SDKClient抽象基类

```csharp
public abstract class SDKClient : IDisposable
{
    /// <summary>
    /// 连接状态
    /// </summary>
    public bool Connected { get; protected set; }
    
    /// <summary>
    /// 连接到服务器
    /// </summary>
    public abstract void Connect(string serverAddress);
    
    /// <summary>
    /// 断开连接
    /// </summary>
    public abstract void DisConnect();
    
    /// <summary>
    /// 销毁客户端
    /// </summary>
    public abstract void DestroyClient();
    
    /// <summary>
    /// 释放资源
    /// </summary>
    public void Dispose();
}
```

#### NokovSDKClient公共属性

```csharp
/// <summary>
/// SDK库版本
/// </summary>
public Version NokovSDKLibVersion { get; }

/// <summary>
/// 刷新标志（数据描述变化时设置）
/// </summary>
public static bool Refresh { get; set; }
```

#### NokovSDKClient公共方法

```csharp
/// <summary>
/// 获取数据描述信息（MarkerSet、刚体、骨骼定义）
/// </summary>
/// <returns>数据描述对象</returns>
public DataDescriptions GetDataDescriptions()

/// <summary>
/// 获取最新一帧动捕数据
/// </summary>
/// <returns>帧数据结构</returns>
public sFrameOfMocapData GetLastFrame()
```

#### NokovSDKClient静态事件

```csharp
/// <summary>
/// 原生帧数据接收事件
/// 在SDK回调线程中触发
/// </summary>
public static event EventHandler<NativeFrameReceivedEventArgs> NativeFrameReceived;
```

#### NativeFrameReceivedEventArgs事件参数

```csharp
public class NativeFrameReceivedEventArgs : EventArgs
{
    /// <summary>
    /// 客户端句柄
    /// </summary>
    public IntPtr ClientHandle { get; set; }
    
    /// <summary>
    /// 原生帧数据指针
    /// </summary>
    public IntPtr NativeFramePointer { get; set; }
    
    /// <summary>
    /// 已解析的帧数据（惰性求值）
    /// </summary>
    public sFrameOfMocapData MarshaledFrame { get; }
}
```

#### DataDescriptions数据描述类

```csharp
public class DataDescriptions
{
    /// <summary>
    /// MarkerSet描述列表
    /// </summary>
    public List<sMarkerSetDescription> MarkerSetDescriptions;
    
    /// <summary>
    /// 刚体描述列表
    /// </summary>
    public List<sRigidBodyDescription> RigidBodyDescriptions;
    
    /// <summary>
    /// 骨骼描述列表
    /// </summary>
    public List<sSkeletonDescription> SkeletonDescriptions;
}
```

---

## 数据结构定义

### NokovPose（位姿数据）

```csharp
/// <summary>
/// 位姿数据，包含位置和旋转
/// </summary>
public class NokovPose
{
    /// <summary>
    /// 位置（Unity坐标系，单位由LengthUnit决定）
    /// </summary>
    public Vector3 Position;
    
    /// <summary>
    /// 旋转（Unity坐标系四元数）
    /// </summary>
    public Quaternion Orientation;
}
```

### NokovRigidBodyState（刚体状态）

```csharp
/// <summary>
/// 刚体实时状态
/// </summary>
public class NokovRigidBodyState
{
    /// <summary>
    /// 数据送达时间戳（高精度）
    /// </summary>
    public NokovHiResTimer.Timestamp DeliveryTimestamp;
    
    /// <summary>
    /// 刚体位姿
    /// </summary>
    public NokovPose Pose;
}
```

### NokovSkeletonState（骨骼状态）

```csharp
/// <summary>
/// 骨骼实时状态
/// </summary>
public class NokovSkeletonState
{
    /// <summary>
    /// 世界坐标系下的骨骼位姿（Key=骨骼ID）
    /// </summary>
    public Dictionary<Int32, NokovPose> BonePoses;
    
    /// <summary>
    /// 本地坐标系下的骨骼位姿（Key=骨骼ID）
    /// </summary>
    public Dictionary<Int32, NokovPose> LocalBonePoses;
}
```

### NokovMarkerState（Marker状态）

```csharp
/// <summary>
/// Marker点实时状态
/// </summary>
public class NokovMarkerState
{
    /// <summary>
    /// 位置（Unity坐标系）
    /// </summary>
    public Vector3 Position;
    
    /// <summary>
    /// Marker大小
    /// </summary>
    public float Size;
    
    /// <summary>
    /// 是否已标记
    /// </summary>
    public bool Labeled;
    
    /// <summary>
    /// Marker ID
    /// </summary>
    public Int32 Id;
}
```

### NokovRigidBodyDefinition（刚体定义）

```csharp
/// <summary>
/// 刚体定义信息
/// </summary>
public class NokovRigidBodyDefinition
{
    /// <summary>
    /// Marker定义（刚体内部Marker）
    /// </summary>
    public class MarkerDefinition
    {
        public Vector3 Position;
        public Int32 RequiredLabel;
    }
    
    /// <summary>
    /// 刚体ID
    /// </summary>
    public Int32 Id;
    
    /// <summary>
    /// 刚体名称
    /// </summary>
    public string Name;
}
```

### NokovSkeletonDefinition（骨骼定义）

```csharp
/// <summary>
/// 骨骼定义信息
/// </summary>
public class NokovSkeletonDefinition
{
    /// <summary>
    /// 单个骨骼定义
    /// </summary>
    public class BoneDefinition
    {
        /// <summary>骨骼ID</summary>
        public Int32 Id;
        
        /// <summary>父骨骼ID（根骨骼为0）</summary>
        public Int32 ParentId;
        
        /// <summary>骨骼名称</summary>
        public string Name;
        
        /// <summary>相对父骨骼的偏移</summary>
        public Vector3 Offset;
        
        /// <summary>骨骼朝向</summary>
        public Quaternion orientation;
        
        /// <summary>骨骼长度</summary>
        public double length;
    }
    
    /// <summary>骨骼ID</summary>
    public Int32 Id;
    
    /// <summary>骨骼名称</summary>
    public string Name;
    
    /// <summary>骨骼列表</summary>
    public List<BoneDefinition> Bones;
    
    /// <summary>骨骼ID到父骨骼ID的映射</summary>
    public Dictionary<Int32, Int32> BoneIdToParentIdMap;
}
```

---

## 枚举类型

### SkeletonCoordinate（骨骼坐标系）

```csharp
public enum SkeletonCoordinate
{
    /// <summary>世界坐标系</summary>
    World,
    
    /// <summary>本地坐标系</summary>
    Local
}
```

### E_LengthUnit（长度单位）

```csharp
public enum E_LengthUnit
{
    /// <summary>米</summary>
    M,
    
    /// <summary>厘米</summary>
    CM,
    
    /// <summary>毫米</summary>
    MM
}
```

### E_UpAxis（上轴方向）

```csharp
public enum E_UpAxis
{
    /// <summary>Y轴向上（Unity默认）</summary>
    Y,
    
    /// <summary>Z轴向上</summary>
    Z
}
```

### NokovBoneNameConvention（骨骼命名规范）

```csharp
public enum NokovBoneNameConvention
{
    /// <summary>星影标准</summary>
    XingYing,
    
    /// <summary>星影+手部</summary>
    XingYingWithHand,
    
    /// <summary>无标记动捕</summary>
    MARKERLESS,
    
    /// <summary>VR身体</summary>
    VRBody,
    
    /// <summary>无规范</summary>
    NoConvention
}
```

### TrackingType（跟踪类型）

```csharp
public enum TrackingType
{
    /// <summary>旋转+位置</summary>
    RotationAndPosition,
    
    /// <summary>仅旋转</summary>
    RotationOnly,
    
    /// <summary>仅位置</summary>
    PositionOnly
}
```

---

## 工具类

### NokovHiResTimer（高精度计时器）

```csharp
public static class NokovHiResTimer
{
    /// <summary>
    /// 时间戳结构
    /// </summary>
    public struct Timestamp
    {
        internal Int64 m_ticks;
        
        /// <summary>
        /// 获取该时间戳距当前的秒数
        /// </summary>
        public float AgeSeconds { get; }
        
        /// <summary>
        /// 计算与参考时间戳的时间差（秒）
        /// </summary>
        public float SecondsSince(Timestamp reference);
    }
    
    /// <summary>
    /// 获取当前时间戳
    /// </summary>
    public static Timestamp Now();
}
```

### LocalNicIpHelper（本地网卡IP助手）

```csharp
public class LocalNicIpHelper
{
    public enum ADDRESSFAM
    {
        IPv4,
        IPv6
    }
    
    /// <summary>
    /// 获取本机IP地址
    /// </summary>
    /// <param name="Addfam">地址族（IPv4/IPv6）</param>
    /// <returns>IP地址字符串</returns>
    public static string GetIP(ADDRESSFAM Addfam = ADDRESSFAM.IPv4);
}
```

### SmoothFrameArray（平滑滤波器）

**文件位置**: `Plugins/Managed/NokovSDKLib/Utility.cs`

```csharp
/// <summary>
/// 7点平均平滑滤波器
/// 用于平滑位置或旋转数据
/// </summary>
internal class SmoothFrameArray
{
    /// <summary>默认窗口大小</summary>
    public const int DefaultSize = 7;
    
    /// <summary>缓存数据点</summary>
    public int cache(double data);
    
    /// <summary>尝试平滑数据</summary>
    public bool tryToSmooth(ref float data);
    
    /// <summary>清空缓存</summary>
    public void clear();
}

/// <summary>
/// 三维点平滑滤波器
/// </summary>
internal class SmoothFramePointArray
{
    /// <summary>缓存三维点</summary>
    public void Cache(double x, double y, double z);
    
    /// <summary>平滑三维点</summary>
    public void Smooth(ref float x, ref float y, ref float z);
    
    /// <summary>重置</summary>
    public void Reset();
}
```

---

## 预制体说明

### Client - Nokov.prefab

**用途**: 数据流客户端预制体

**包含组件**:
- `StreamingClient` - 核心数据流客户端

**使用方法**:
1. 将预制体拖入场景
2. 配置ServerIp为Nokov服务器地址
3. 根据需要调整坐标系和单位设置

---

### RigidBody - Nokov.prefab

**用途**: 刚体跟踪预制体

**包含组件**:
- `NokovRigidBody` - 刚体跟踪组件

**使用方法**:
1. 将预制体拖入场景
2. 设置RigidBodyName与Nokov软件中刚体名称一致
3. 选择合适的trackingType

---

### Retargeted Skeleton - Nokov.prefab

**用途**: 骨骼重定向预制体

**包含组件**:
- `NokovSkeletonAnimator` - 骨骼动画组件

**使用方法**:
1. 将预制体拖入场景
2. 设置SkeletonAssetName与Nokov软件中骨骼名称一致
3. 设置DestinationAvatar为目标角色Avatar

---

## 使用场景与代码示例

### 场景1：基础刚体跟踪

**方法A：使用预制体（推荐）**

1. 拖入`Client - Nokov`预制体
2. 拖入`RigidBody - Nokov`预制体
3. 配置参数后运行

**方法B：手动配置**

1. 创建空GameObject，添加`StreamingClient`组件
2. 创建需要跟踪的GameObject，添加`NokovRigidBody`组件
3. 设置`RigidBodyName`

---

### 场景2：代码获取刚体数据

```csharp
using UnityEngine;

public class RigidBodyDataReceiver : MonoBehaviour
{
    [Header("配置")]
    public string rigidBodyName = "tracker";
    
    private StreamingClient client;
    
    void Start()
    {
        // 获取数据流客户端
        client = StreamingClient.FindDefaultClient();
        
        if (client == null)
        {
            Debug.LogError("未找到StreamingClient!");
            enabled = false;
        }
    }
    
    void Update()
    {
        // 获取刚体状态
        NokovRigidBodyState rbState = client.GetLatestRigidBodyState(rigidBodyName);
        
        if (rbState != null)
        {
            // 获取位置和旋转
            Vector3 position = rbState.Pose.Position;
            Quaternion rotation = rbState.Pose.Orientation;
            
            // 应用到当前GameObject
            transform.position = position;
            transform.rotation = rotation;
            
            // 获取时间戳信息
            float age = rbState.DeliveryTimestamp.AgeSeconds;
            Debug.Log($"刚体[{rigidBodyName}] 位置={position}, 数据年龄={age:F3}s");
        }
    }
}
```

---

### 场景3：获取Marker点数据

```csharp
using UnityEngine;
using System.Collections.Generic;

public class MarkerDataReceiver : MonoBehaviour
{
    private StreamingClient client;
    
    void Start()
    {
        client = StreamingClient.FindDefaultClient();
    }
    
    void Update()
    {
        if (client == null) return;
        
        // 获取已标记的Marker
        List<NokovMarkerState> labeledMarkers = client.GetLatestMarkerStates();
        Debug.Log($"已标记Marker数量: {labeledMarkers.Count}");
        
        foreach (var marker in labeledMarkers)
        {
            Debug.Log($"Marker[{marker.Id}] 位置={marker.Position}, 大小={marker.Size}");
        }
        
        // 获取未标记的Marker
        List<NokovMarkerState> unlabeledMarkers = client.GetLatestUnlabeledMarkerStates();
        Debug.Log($"未标记Marker数量: {unlabeledMarkers.Count}");
    }
}
```

---

### 场景4：获取刚体定义信息

```csharp
using UnityEngine;

public class RigidBodyDefinitionReader : MonoBehaviour
{
    void Start()
    {
        StreamingClient client = StreamingClient.FindDefaultClient();
        if (client == null) return;
        
        // 通过名称获取刚体定义
        NokovRigidBodyDefinition rbDef = client.GetRigidBodyDefinitionByName("tracker");
        
        if (rbDef != null)
        {
            Debug.Log($"刚体名称: {rbDef.Name}");
            Debug.Log($"刚体ID: {rbDef.Id}");
        }
        
        // 通过ID获取刚体定义
        NokovRigidBodyDefinition rbDefById = client.GetRigidBodyDefinitionById(0);
    }
}
```

---

### 场景5：骨骼动画重定向

```csharp
// 1. 准备工作：
//    - 导入Humanoid角色模型
//    - 确保角色使用Humanoid Rig
//    - 获取角色的Avatar

// 2. 场景配置：
//    - 拖入 Client - Nokov 预制体
//    - 创建空GameObject，添加NokovSkeletonAnimator组件

// 3. 组件配置：
//    - StreamingClient: 指向场景中的StreamingClient
//    - SkeletonAssetName: 与Nokov软件中骨骼名称一致（如"Skeleton1"）
//    - DestinationAvatar: 拖入角色的Avatar

// NokovSkeletonAnimator会自动：
//    - 创建临时骨骼层级
//    - 建立Mecanim骨骼映射
//    - 实时重定向骨骼动画到目标角色
```

---

### 场景6：多刚体管理

```csharp
using UnityEngine;
using System.Collections.Generic;

public class MultiRigidBodyManager : MonoBehaviour
{
    [System.Serializable]
    public class RigidBodyMapping
    {
        public string rigidBodyName;
        public GameObject targetObject;
    }
    
    public List<RigidBodyMapping> mappings = new List<RigidBodyMapping>();
    
    private StreamingClient client;
    
    void Start()
    {
        client = StreamingClient.FindDefaultClient();
    }
    
    void Update()
    {
        if (client == null) return;
        
        foreach (var mapping in mappings)
        {
            if (mapping.targetObject == null) continue;
            
            NokovRigidBodyState state = client.GetLatestRigidBodyState(mapping.rigidBodyName);
            
            if (state != null)
            {
                mapping.targetObject.transform.position = state.Pose.Position;
                mapping.targetObject.transform.rotation = state.Pose.Orientation;
            }
        }
    }
}
```

---

### 场景7：线程安全数据访问

```csharp
using UnityEngine;
using System.Collections.Generic;

public class ThreadSafeDataAccess : MonoBehaviour
{
    private StreamingClient client;
    
    void Start()
    {
        client = StreamingClient.FindDefaultClient();
    }
    
    void Update()
    {
        if (client == null) return;
        
        // 使用锁保护批量数据访问
        client._EnterFrameDataUpdateLock();
        try
        {
            // 在锁内访问多个数据，保证数据一致性
            NokovRigidBodyState rb1 = client.GetLatestRigidBodyState("rigid1");
            NokovRigidBodyState rb2 = client.GetLatestRigidBodyState("rigid2");
            List<NokovMarkerState> markers = client.GetLatestMarkerStates();
            
            // 处理数据...
        }
        finally
        {
            client._ExitFrameDataUpdateLock();
        }
    }
}
```

---

## 坐标系转换

### Nokov坐标系 vs Unity坐标系

| 属性 | Nokov | Unity |
|------|-------|-------|
| 手性 | 右手系 | 左手系 |
| 单位 | 毫米 | 米 |
| 上轴 | 可配置(Y/Z) | Y |

### 转换公式

#### 位置转换（Y轴向上）

```csharp
// Nokov → Unity
// X轴翻转，单位从毫米转换
Vector3 unityPos = new Vector3(-nokovX, nokovY, nokovZ) / 1000f;
```

#### 位置转换（Z轴向上）

```csharp
// Nokov → Unity  
// 轴映射：Nokov(X,Y,Z) → Unity(-Y,Z,X)
Vector3 unityPos = new Vector3(-nokovY, nokovZ, nokovX) / 1000f;
```

#### 四元数转换（Y轴向上）

```csharp
// Nokov → Unity
// X和W分量取反
Quaternion unityQuat = new Quaternion(-nokovQX, nokovQY, nokovQZ, -nokovQW);
```

#### 四元数转换（Z轴向上）

```csharp
// Nokov → Unity
Quaternion unityQuat = new Quaternion(-nokovQY, nokovQZ, nokovQX, -nokovQW);
```

### 使用内置转换方法

```csharp
StreamingClient client = StreamingClient.FindDefaultClient();

// 位置转换（自动处理单位和坐标系）
Vector3 unityPosition = client.NokovToUnityTranslation(nokovX, nokovY, nokovZ);

// 四元数转换
Quaternion unityRotation = StreamingClient.NokovToUnityQuaternion(
    nokovQX, nokovQY, nokovQZ, nokovQW, client.UpAxis);
```

---

## 与自定义NokovSDKManager对比

### 功能对比表

| 功能 | 官方Nokov插件 | 自定义NokovSDKManager |
|------|--------------|----------------------|
| **连接管理** | ✅ 基础连接/断开 | ✅ 自动连接+断线重连 |
| **刚体跟踪** | ✅ 支持 | ✅ 支持 |
| **骨骼动画** | ✅ 支持Mecanim重定向 | ❌ 不支持 |
| **Marker可视化** | ✅ 内置绘制 | ❌ 不支持 |
| **事件系统** | ⚠️ 原生事件（需手动订阅） | ✅ 友好的C#事件 |
| **日志系统** | ❌ 无 | ✅ NokovDataLogger |
| **诊断工具** | ❌ 无 | ✅ NokovSDKDiagnostics |
| **单例模式** | ❌ 需调用FindDefaultClient | ✅ Instance属性 |
| **线程安全** | ✅ Monitor锁 | ✅ 帧队列+主线程处理 |
| **坐标转换** | ✅ 内置 | ✅ 内置 |
| **数据有效性检查** | ❌ 无 | ✅ IsValidPosition() |
| **帧率统计** | ✅ 支持 | ✅ 支持 |

### 使用建议

| 场景 | 推荐方案 |
|------|----------|
| 仅刚体跟踪 | 可使用任一方案，自定义版本更精简 |
| 需要骨骼动画 | 必须使用官方插件 |
| 需要Marker可视化 | 使用官方插件（开箱即用） |
| 需要详细日志/诊断 | 使用自定义NokovSDKManager |
| 混合使用 | 可以共存，但注意不要创建多个SDK客户端 |

---

## 最佳实践

### 1. 连接管理

```csharp
// ✅ 推荐：使用预制体或让插件自动管理连接
// StreamingClient在OnEnable时自动连接，OnDisable时断开

// ❌ 避免：同时创建多个StreamingClient实例
```

### 2. 刚体名称

```csharp
// ✅ 推荐：使用与Nokov软件中完全一致的刚体名称
public string rigidBodyName = "tracker";  // 区分大小写

// ❌ 避免：硬编码ID，因为ID可能变化
```

### 3. 数据访问

```csharp
// ✅ 推荐：始终检查返回值是否为null
NokovRigidBodyState state = client.GetLatestRigidBodyState("tracker");
if (state != null)
{
    // 使用数据
}

// ❌ 避免：直接使用未检查的返回值
```

### 4. 性能优化

```csharp
// ✅ 推荐：缓存客户端引用
private StreamingClient client;
void Start() { client = StreamingClient.FindDefaultClient(); }

// ❌ 避免：每帧调用FindDefaultClient
void Update() { StreamingClient.FindDefaultClient().GetLatestRigidBodyState("tracker"); }
```

### 5. 坐标系配置

```csharp
// ✅ 推荐：在Inspector中正确配置坐标系
// LengthUnit: M（米，Unity标准）
// UpAxis: Y（Unity标准）

// ⚠️ 注意：修改UpAxis会影响所有数据的转换
```

### 6. 骨骼动画

```csharp
// ✅ 推荐：确保目标Avatar是Humanoid类型
// 在角色模型导入设置中配置Animation Type = Humanoid

// ❌ 避免：使用Generic Avatar（不支持重定向）
```

---

## 附录

### A. 相关文件列表

| 文件 | 行数 | 功能 |
|------|------|------|
| `StreamingClient.cs` | 1090 | 核心数据流客户端 |
| `NokovRigidBody.cs` | 140 | 刚体跟踪组件 |
| `NokovSkeletonAnimator.cs` | 464 | 骨骼动画组件 |
| `Client.cs` | 363 | SDK封装层 |
| `Utility.cs` | 179 | 平滑滤波工具 |
| `PluginVersion.cs` | - | 版本信息 |

### B. 依赖项

- Unity 2017.1 或更高版本
- .NET Framework 4.x
- Nokov动捕系统（Mars/XING/XINGYING）
- Nokov服务器软件（运行中）

### C. 版本历史

| 版本 | 日期 | 更新内容 |
|------|------|----------|
| 2.4.0 | - | 初始版本 |
| 2.5.53 | - | 当前版本 |

---

*文档结束*

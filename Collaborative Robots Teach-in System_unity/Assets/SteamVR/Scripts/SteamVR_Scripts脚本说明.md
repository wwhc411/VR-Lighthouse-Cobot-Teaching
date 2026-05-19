# SteamVR Scripts 脚本文件夹说明

本文档介绍 SteamVR Unity 插件核心脚本文件夹中各个文件的功能与用途。

---

## 📌 目录

- [核心系统类](#核心系统类)
- [相机与渲染](#相机与渲染)
- [设备追踪](#设备追踪)
- [事件与工具](#事件与工具)
- [用户界面](#用户界面)
- [进阶功能](#进阶功能)
- [已弃用组件](#已弃用组件)

---

## 核心系统类

### SteamVR.cs
**核心入口类** - SteamVR 系统的主要接口。

| 属性 | 说明 |
|------|------|
| `instance` | 单例访问 SteamVR 系统 |
| `enabled` | 启用/禁用 SteamVR |
| `active` | 检查 SteamVR 是否已激活 |

**主要功能**：
- 封装 OpenVR API 的初始化与销毁
- 提供 HMD（头显）和 Compositor（合成器）接口访问
- 管理 SteamVR 系统生命周期

---

### SteamVR_Behaviour.cs
**行为管理器** - SteamVR 运行时的 MonoBehaviour 入口。

**主要功能**：
- 自动创建 `[SteamVR]` 游戏对象
- 管理 SteamVR_Render 组件的生命周期
- 处理 VR 初始化流程
- 支持 `DontDestroyOnLoad` 持久化

**使用场景**：
```csharp
// 获取实例
SteamVR_Behaviour.instance.initializeSteamVROnAwake = true;
```

---

### SteamVR_Settings.cs
**全局配置** - SteamVR 插件的可配置参数。

| 配置项 | 说明 |
|--------|------|
| `pauseGameWhenDashboardVisible` | 仪表盘显示时暂停游戏 |
| `lockPhysicsUpdateRateToRenderFrequency` | 物理更新锁定到渲染频率 |
| `trackingSpace` | 追踪空间原点 (Standing/Seated) |
| `actionsFilePath` | Input System 动作配置文件路径 |
| `inputUpdateMode` | 输入更新模式 |
| `poseUpdateMode` | 位姿更新模式 |

---

## 相机与渲染

### SteamVR_Camera.cs
**VR 相机** - 为现有相机添加 SteamVR 渲染支持。

**主要功能**：
- 管理 VR 相机层级结构 (`head`, `origin`)
- 提供场景分辨率缩放控制
- 处理相机与 SteamVR_Render 的注册/注销

**层级结构**：
```
[CameraRig] (origin)
  └── Camera (head)
       └── SteamVR_Camera
```

---

### SteamVR_Render.cs
**渲染管理器** - 处理所有 SteamVR 相机的渲染。

**主要功能**：
- 管理多个 VR 相机的渲染顺序
- 处理外部相机 (MR 混合现实)
- 协调左右眼渲染 (`EVREye`)
- 应用程序退出时安全清理

---

### SteamVR_ExternalCamera.cs
**外部相机** - 用于混合现实 (MR) 拍摄。

**主要功能**：
- 渲染第三人称视角（分离前景/背景）
- 从配置文件读取相机位姿和参数
- 支持色度键（绿幕抠像）
- 用于 VR 直播和录制

**配置参数**：
- 位置 (x, y, z)、旋转 (rx, ry, rz)
- FOV、近/远裁剪面
- 色度键颜色 (r, g, b, a)

---

### SteamVR_Skybox.cs
**VR 天空盒** - 设置合成器使用的立方体贴图。

**主要功能**：
- 支持六面天空盒纹理 (front, back, left, right, top, bottom)
- 立体渲染支持（IPD 设置）
- 在合成器层面设置环境背景

---

### SteamVR_Fade.cs
**画面淡入淡出** - 场景过渡效果。

**使用示例**：
```csharp
// 从黑色淡入
SteamVR_Fade.Start(Color.black, 0);    // 立即设为黑色
SteamVR_Fade.Start(Color.clear, 1);     // 1秒淡入

// 合成器层面淡出（影响整个VR视图）
SteamVR_Fade.View(Color.black, 1);
```

---

### SteamVR_SphericalProjection.cs
**球面投影** - 应用球面变形到渲染输出。

**用途**：特殊投影效果、全景渲染等。

---

## 设备追踪

### SteamVR_TrackedObject.cs ⭐
**设备追踪核心** - 将 GameObject 与追踪设备绑定。

**设备索引枚举**：
```csharp
public enum EIndex {
    None = -1,
    Hmd = 0,      // 头显
    Device1,      // 通常是左手柄
    Device2,      // 通常是右手柄
    Device3~16    // Tracker 等其他设备
}
```

**关键属性**：
| 属性 | 说明 |
|------|------|
| `index` | 设备索引 |
| `origin` | 坐标系原点 Transform |
| `isValid` | 位姿是否有效 |
| `enablePoseMonitoring` | 启用位姿变化监控 |
| `positionChangeThreshold` | 位移阈值（米）|
| `rotationChangeThreshold` | 旋转阈值（度）|

**使用场景**：
- 控制器追踪
- Tracker 定位器追踪
- 任何需要跟随追踪设备的物体

---

### SteamVR_TrackingReferenceManager.cs
**基站管理器** - 自动检测和管理 Lighthouse 基站。

**主要功能**：
- 监听 `NewPoses` 事件
- 自动为检测到的基站创建 GameObject
- 添加 SteamVR_TrackedObject 和 SteamVR_RenderModel 组件
- 基站位姿变化时输出日志

**适用场景**：调试基站位置、可视化追踪参考点。

---

### SteamVR_RenderModel.cs
**设备模型渲染** - 显示追踪设备的 3D 模型。

**主要功能**：
- 从 SteamVR 加载设备官方 3D 模型
- 支持组件化加载（按钮、触控板等分开）
- 运行时动态更新组件状态（如按钮按下动画）

**关键属性**：
| 属性 | 说明 |
|------|------|
| `index` | 设备索引 |
| `createComponents` | 是否拆分为独立组件 |
| `updateDynamically` | 运行时更新组件 |
| `shader` | 应用到模型的着色器 |

---

### SteamVR_TrackedCamera.cs
**头显相机** - 访问 Vive 等设备的前置摄像头。

**主要功能**：
- 获取摄像头视频流纹理
- 支持畸变/去畸变两种模式
- 提供摄像头位姿信息

**使用示例**：
```csharp
var source = SteamVR_TrackedCamera.Undistorted();
source.Acquire();
Texture2D tex = source.texture;
```

---

## 事件与工具

### SteamVR_Events.cs
**事件系统** - SteamVR 内部事件分发。

**核心事件**：
| 事件 | 触发时机 |
|------|----------|
| `DeviceConnected` | 设备连接/断开 |
| `NewPoses` | 新位姿数据到达 |
| `InputFocus` | 输入焦点变化 |
| `Fade` | 淡入淡出触发 |
| `System` | OpenVR 系统事件 |

**使用示例**：
```csharp
void OnEnable() {
    SteamVR_Events.DeviceConnected.Listen(OnDeviceConnected);
}

void OnDisable() {
    SteamVR_Events.DeviceConnected.Remove(OnDeviceConnected);
}

void OnDeviceConnected(int index, bool connected) {
    Debug.Log($"Device {index}: {(connected ? "Connected" : "Disconnected")}");
}
```

---

### SteamVR_Utils.cs
**工具类** - 通用工具函数集合。

**主要功能**：
| 方法/类 | 说明 |
|---------|------|
| `RigidTransform` | 位置+旋转的刚体变换结构 |
| `IsValid(Vector3)` | 检查向量是否有效（非 NaN）|
| `IsValid(Quaternion)` | 检查四元数是否有效 |
| `Slerp` | 不限范围的四元数球面插值 |
| `Event` | 简单事件系统（字符串消息） |

---

### SteamVR_RingBuffer.cs
**环形缓冲区** - 用于存储历史数据的数据结构。

**用途**：
- 位姿历史记录
- 输入事件缓存
- 平滑/预测算法的数据存储

---

### SteamVR_EnumEqualityComparer.cs
**枚举比较器** - 优化枚举作为字典键时的性能。

**技术细节**：避免枚举装箱（Boxing），提高 Dictionary 查找效率。

---

## 用户界面

### SteamVR_PlayArea.cs
**游戏区域可视化** - 显示房间规模的边界。

**主要功能**：
- 可视化 Chaperone 边界
- 支持预设尺寸或校准尺寸
- 编辑器和运行时均可使用

**尺寸选项**：
- `Calibrated` - 用户实际校准的房间
- `_400x300` - 4m × 3m
- `_300x225` - 3m × 2.25m
- `_200x150` - 2m × 1.5m

---

### SteamVR_Overlay.cs
**VR 叠加层** - 在 VR 视图中显示 2D 内容。

**主要功能**：
- 创建浮动的 2D 界面
- 支持鼠标输入
- 可调节尺寸、距离、透明度

**应用场景**：VR 菜单、信息面板、调试界面。

---

### SteamVR_Menu.cs
**示例菜单** - 使用 OnGUI 的 VR 菜单示例。

**功能演示**：
- 结合 SteamVR_Overlay 使用
- 光标显示
- 缩放控制

---

### SteamVR_LoadLevel.cs
**场景加载** - 平滑的 VR 场景过渡。

**主要功能**：
- 异步加载支持
- 加载画面显示
- 进度条显示
- 天空盒过渡
- 支持启动外部进程

---

## 进阶功能

### SteamVR_IK.cs
**逆运动学** - 简单的两骨骼 IK 解算器。

**用途**：
- 手臂/腿部 IK
- VR 化身手部跟随

**参数**：
- `target` - IK 目标位置
- `start`, `joint`, `end` - 骨骼链
- `poleVector` - 极向量（控制弯曲方向）

---

### SteamVR_Frustum.cs
**视锥体可视化** - 基于 FOV 生成网格。

**用途**：
- 可视化设备视场范围
- 调试追踪区域
- 展示 Lighthouse 覆盖范围

---

### SteamVR_Ears.cs
**音频定位** - 使用扬声器时的音频监听器对齐。

**功能**：当用户使用外部扬声器而非耳机时，正确对齐 AudioListener 方向。

---

### SteamVR_ExternalCamera_LegacyManager.cs
**旧版外部相机管理器** - 兼容旧版外部相机配置。

---

## 已弃用组件

以下组件在 Unity 5.4+ 中已弃用，会在 Awake 时自动销毁：

| 组件 | 原功能 |
|------|--------|
| `SteamVR_CameraFlip.cs` | D3D 相机输出翻转 |
| `SteamVR_CameraMask.cs` | 遮罩不可见像素 |

---

## 📊 脚本分类图

```
SteamVR Scripts
├── 核心系统
│   ├── SteamVR.cs              [系统入口]
│   ├── SteamVR_Behaviour.cs    [行为管理]
│   └── SteamVR_Settings.cs     [全局配置]
│
├── 渲染相关
│   ├── SteamVR_Camera.cs       [VR相机]
│   ├── SteamVR_Render.cs       [渲染管理]
│   ├── SteamVR_Fade.cs         [淡入淡出]
│   ├── SteamVR_Skybox.cs       [天空盒]
│   └── SteamVR_ExternalCamera.cs [MR相机]
│
├── 设备追踪
│   ├── SteamVR_TrackedObject.cs     [设备追踪]
│   ├── SteamVR_TrackingReferenceManager.cs [基站管理]
│   ├── SteamVR_RenderModel.cs       [设备模型]
│   └── SteamVR_TrackedCamera.cs     [头显相机]
│
├── 用户界面
│   ├── SteamVR_PlayArea.cs     [游戏区域]
│   ├── SteamVR_Overlay.cs      [VR叠加层]
│   ├── SteamVR_Menu.cs         [菜单示例]
│   └── SteamVR_LoadLevel.cs    [场景加载]
│
└── 工具与辅助
    ├── SteamVR_Events.cs       [事件系统]
    ├── SteamVR_Utils.cs        [工具函数]
    ├── SteamVR_IK.cs           [逆运动学]
    ├── SteamVR_Frustum.cs      [视锥体]
    └── SteamVR_RingBuffer.cs   [环形缓冲]
```

---

## 🔗 常用组合

### 1. 基础 VR 场景
```
[CameraRig]
├── SteamVR_Behaviour
├── SteamVR_PlayArea
└── Camera
    └── SteamVR_Camera
```

### 2. 手柄追踪
```
Controller (Left)
├── SteamVR_TrackedObject (index = Device1)
└── SteamVR_RenderModel
```

### 3. Tracker 定位器
```
TrackerObject
└── SteamVR_TrackedObject (index = Device3)
```

### 4. MR 直播
```
[CameraRig]
├── SteamVR_Behaviour
├── SteamVR_Render
│   └── externalCamera → SteamVR_ExternalCamera
└── SteamVR_ExternalCamera
```

---

*文档版本: 1.0*
*生成日期: 2025年12月13日*
*适用于: SteamVR Unity Plugin*

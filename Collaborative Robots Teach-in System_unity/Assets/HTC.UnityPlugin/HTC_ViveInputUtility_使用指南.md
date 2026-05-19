# HTC Vive Input Utility (VIU) 插件使用指南

## 📋 概述

**VIVE Input Utility (VIU)** 是 HTC 官方开发的 Unity VR 开发工具包，版本 1.20.2。它提供了统一的 VR 输入和交互接口，支持多种 VR 平台：

- HTC VIVE / VIVE Pro / VIVE Cosmos
- Oculus Rift / Rift S / Quest / Go
- Google Daydream
- VIVE Wave SDK（如 VIVE Focus）
- Windows Mixed Reality
- Valve Index
- 以及更多 Unity 支持的 VR 平台

---

## 📁 模块结构

插件包含以下核心模块：

| 模块 | 说明 |
|------|------|
| **ColliderEvent** | 基于碰撞体的 3D 交互事件系统 |
| **Pointer3D** | 3D 指针和射线交互系统 |
| **PoseTracker** | 位姿追踪器基础组件 |
| **LiteCoroutine** | 轻量级协程系统 |
| **Utility** | 通用工具类 |
| **VRModule** | VR 设备抽象层 |
| **ViveInputUtility** | Vive 专用输入工具和追踪器 |

---

## 🎮 模块详解

### 1. ColliderEvent（碰撞事件模块）

用于处理基于物理碰撞体的 3D 交互事件，适合实现抓取、触碰等近距离交互。

#### 核心脚本

| 脚本 | 作用 |
|------|------|
| `ColliderEventCaster.cs` | 碰撞事件发射器，挂载在控制器上检测碰撞 |
| `ColliderEventData.cs` | 碰撞事件数据类，包含悬停、按钮、轴向事件数据 |
| `ColliderEventInterfaces.cs` | 事件接口定义，用于接收碰撞交互事件 |
| `ExecuteColliderEvents.cs` | 事件执行器，负责分发事件到处理器 |

#### 可用事件接口

```csharp
// 悬停事件
IColliderEventHoverEnterHandler    // 悬停进入
IColliderEventHoverExitHandler     // 悬停离开
IColliderEventLastHoverEnterHandler // 最后悬停进入
IColliderEventLastHoverExitHandler  // 最后悬停离开

// 按钮事件
IColliderEventPressDownHandler     // 按下
IColliderEventPressUpHandler       // 抬起
IColliderEventPressEnterHandler    // 按下状态进入
IColliderEventPressExitHandler     // 按下状态离开
IColliderEventClickHandler         // 点击

// 拖拽事件
IColliderEventDragStartHandler     // 拖拽开始
IColliderEventDragUpdateHandler    // 拖拽更新
IColliderEventDragFixedUpdateHandler // 拖拽物理更新
IColliderEventDragEndHandler       // 拖拽结束
IColliderEventDropHandler          // 放置

// 轴向事件
IColliderEventAxisChangedHandler   // 轴向值变化
```

#### 使用示例

```csharp
using HTC.UnityPlugin.ColliderEvent;
using UnityEngine;

public class Grabbable : MonoBehaviour, 
    IColliderEventHoverEnterHandler, 
    IColliderEventDragStartHandler,
    IColliderEventDragEndHandler
{
    public void OnColliderEventHoverEnter(ColliderHoverEventData eventData)
    {
        // 控制器悬停在物体上
        GetComponent<Renderer>().material.color = Color.yellow;
    }

    public void OnColliderEventDragStart(ColliderButtonEventData eventData)
    {
        // 开始抓取
        transform.SetParent(eventData.eventCaster.transform);
    }

    public void OnColliderEventDragEnd(ColliderButtonEventData eventData)
    {
        // 释放物体
        transform.SetParent(null);
    }
}
```

---

### 2. Pointer3D（3D 指针模块）

用于实现基于射线的 UI 和 3D 对象交互，类似于鼠标指针的 VR 版本。

#### 核心脚本

| 脚本 | 作用 |
|------|------|
| `Pointer3DInputModule.cs` | 3D 指针输入模块，替代/配合 StandaloneInputModule |
| `Pointer3DRaycaster.cs` | 3D 射线检测器基类 |
| `Pointer3DEventData.cs` | 3D 指针事件数据 |
| `Pointer3DEventInterfaces.cs` | 3D 指针事件接口 |

#### 射线模式

```csharp
public enum RaycastMode
{
    DefaultRaycast,  // 直线射线
    Projection,      // 投影（抛物线）
    Projectile,      // 弹道
}
```

#### 可用事件接口

```csharp
IPointer3DPressEnterHandler  // 按下状态进入
IPointer3DPressExitHandler   // 按下状态离开
```

#### 使用说明

1. 确保场景中有 `EventSystem`
2. `Pointer3DInputModule` 会自动添加或与现有 InputModule 共存
3. 在控制器上添加 `ViveRaycaster` 组件

---

### 3. PoseTracker（位姿追踪模块）

用于追踪 VR 设备位姿并应用到 GameObject。

#### 核心脚本

| 脚本 | 作用 |
|------|------|
| `BasePoseTracker.cs` | 位姿追踪器基类 |
| `PoseTracker.cs` | 简单位姿追踪器，跟随目标 Transform |
| `IPoseTracker.cs` | 位姿追踪器接口 |
| `IPoseModifier.cs` | 位姿修改器接口 |
| `BasePoseModifier.cs` | 位姿修改器基类 |

#### 位姿修改器系统

位姿修改器可以在追踪数据应用前进行修改（如添加偏移、平滑等）：

```csharp
public interface IPoseModifier
{
    bool enabled { get; }
    int priority { get; set; }  // 优先级，数值越小越先执行
    void ModifyPose(ref RigidPose pose, bool useLocal);
}
```

#### 使用示例

```csharp
// 添加位姿修改器
var tracker = GetComponent<BasePoseTracker>();
tracker.AddModifier(myModifier);

// 移除位姿修改器
tracker.RemoveModifier(myModifier);
```

---

### 4. LiteCoroutine（轻量协程模块）

提供不依赖 MonoBehaviour 的协程系统，支持后台线程执行。

#### 核心脚本

| 脚本 | 作用 |
|------|------|
| `LiteCoroutine.cs` | 轻量协程主类 |
| `LiteCoroutineManager.cs` | 协程管理器 |
| `LiteTask.cs` | 支持前后台切换的异步任务 |

#### 协程使用

```csharp
using HTC.UnityPlugin.LiteCoroutineSystem;
using System.Collections;

public class Example : MonoBehaviour
{
    private LiteCoroutine handle;

    void Start()
    {
        // 启动协程
        handle = LiteCoroutine.StartCoroutine(MyRoutine());
    }

    void OnDestroy()
    {
        // 停止协程
        LiteCoroutine.StopCoroutine(handle);
    }

    IEnumerator MyRoutine()
    {
        yield return new WaitForSeconds(1f);
        Debug.Log("1秒后执行");
    }
}
```

#### LiteTask（异步任务）

支持在主线程和后台线程之间切换：

```csharp
using HTC.UnityPlugin.LiteCoroutineSystem;
using System.Collections;

IEnumerator MyTask()
{
    // 切换到后台线程
    yield return LiteTask.ToBackground;
    
    // 在后台执行耗时操作
    var result = HeavyCalculation();
    
    // 切换回主线程
    yield return LiteTask.ToForeground;
    
    // 更新 UI
    UpdateUI(result);
}
```

#### 任务状态

```csharp
public enum LiteTaskState
{
    Init,       // 初始化
    Running,    // 运行中
    Done,       // 完成
    Cancelled,  // 已取消
    Exception,  // 异常
}
```

---

### 5. Utility（工具模块）

提供各种通用工具类。

#### 核心脚本

| 脚本 | 作用 |
|------|------|
| `RigidPose.cs` | 刚体位姿结构（位置+旋转） |
| `SingletonBehaviour.cs` | 单例 MonoBehaviour 基类 |
| `ChangeProp.cs` | 属性变化检测工具 |
| `Bool3.cs` | 三维布尔结构 |
| `EnumUtils.cs` | 枚举工具类 |

#### RigidPose（刚体位姿）

```csharp
using HTC.UnityPlugin.Utility;

// 创建位姿
var pose = new RigidPose(position, rotation);

// 从 Transform 创建
var pose = new RigidPose(transform);

// 位姿变换
var combinedPose = poseA * poseB;  // 组合变换
var inversePose = pose.GetInverse();  // 求逆

// 插值
var lerpPose = RigidPose.Lerp(poseA, poseB, t);

// 应用到刚体
RigidPose.SetRigidbodyVelocity(rigidbody, fromPos, toPos, duration);
RigidPose.SetRigidbodyAngularVelocity(rigidbody, fromRot, toRot, duration);
```

#### SingletonBehaviour（单例基类）

```csharp
using HTC.UnityPlugin.Utility;

public class GameManager : SingletonBehaviour<GameManager>
{
    protected override void OnSingletonBehaviourInitialized()
    {
        // 单例初始化时调用
    }
}

// 使用
GameManager.Instance.DoSomething();
if (GameManager.Active) { /* 单例存在 */ }
```

#### ChangeProp（属性变化检测）

```csharp
using HTC.UnityPlugin.Utility;

private int currentValue;

void Update()
{
    int newValue = GetNewValue();
    if (ChangeProp.Set(ref currentValue, newValue))
    {
        // 值发生变化时执行
        OnValueChanged();
    }
}
```

---

### 6. VRModule（VR 模块管理）

提供跨平台的 VR 设备抽象层。

#### 核心脚本

| 脚本 | 作用 |
|------|------|
| `VRModule.cs` | VR 模块主类，提供设备状态访问 |
| `VRModuleManager.cs` | 模块管理器，处理模块切换 |
| `VRModuleDeviceState.cs` | 设备状态定义 |
| `VRModuleSettings.cs` | 模块设置 |

#### 支持的 VR 模块

```csharp
public enum VRModuleSelectEnum
{
    Auto = -1,       // 自动选择
    None = 0,        // 无
    Simulator = 1,   // 模拟器
    UnityNativeVR = 2,
    SteamVR = 3,
    OculusVR = 4,
    DayDream = 5,
    WaveVR = 6,
    UnityXR = 7,
}
```

#### 设备类型

```csharp
public enum VRModuleDeviceClass
{
    Invalid,
    HMD,              // 头显
    Controller,       // 控制器
    GenericTracker,   // 通用追踪器
    TrackingReference, // 追踪参考点（基站）
    TrackedHand,      // 追踪手势
}
```

#### 设备状态接口

```csharp
// 获取设备状态
IVRModuleDeviceState state = VRModule.GetCurrentDeviceState(deviceIndex);

// 常用属性
bool isConnected = state.isConnected;
bool isPoseValid = state.isPoseValid;
Vector3 position = state.position;
Quaternion rotation = state.rotation;
RigidPose pose = state.pose;
string serialNumber = state.serialNumber;
VRModuleDeviceClass deviceClass = state.deviceClass;
VRModuleDeviceModel deviceModel = state.deviceModel;
```

#### 事件

```csharp
VRModule.onNewPoses += OnNewPoses;              // 新位姿数据
VRModule.onNewInput += OnNewInput;              // 新输入数据
VRModule.onDeviceConnected += OnDeviceConnected; // 设备连接/断开
VRModule.onActiveModuleChanged += OnModuleChanged; // 模块切换
```

---

### 7. ViveInputUtility（Vive 输入工具）

Vive 控制器专用的输入和位姿工具。

#### ViveInput（输入管理）

```csharp
using HTC.UnityPlugin.Vive;

// 按钮检测
bool pressed = ViveInput.GetPress(HandRole.RightHand, ControllerButton.Trigger);
bool pressDown = ViveInput.GetPressDown(HandRole.RightHand, ControllerButton.Trigger);
bool pressUp = ViveInput.GetPressUp(HandRole.RightHand, ControllerButton.Grip);

// 轴向值
float triggerValue = ViveInput.GetAxis(HandRole.RightHand, ControllerAxis.Trigger);
Vector2 padAxis = new Vector2(
    ViveInput.GetAxis(HandRole.RightHand, ControllerAxis.PadX),
    ViveInput.GetAxis(HandRole.RightHand, ControllerAxis.PadY)
);
```

#### 控制器按钮

```csharp
public enum ControllerButton
{
    Trigger,       // 扳机
    TriggerTouch,  // 扳机触摸
    Pad,           // 触摸板按下
    PadTouch,      // 触摸板触摸
    Grip,          // 握把
    Menu,          // 菜单键
    System,        // 系统键
    AKey,          // A键（Knuckles/Quest）
    BKey,          // B键
    Joystick,      // 摇杆按下
    JoystickTouch, // 摇杆触摸
    // ...更多
}
```

#### VivePose（位姿管理）

```csharp
using HTC.UnityPlugin.Vive;

// 获取设备位姿
RigidPose pose = VivePose.GetPose(HandRole.RightHand);
bool isValid = VivePose.IsValid(HandRole.RightHand);
bool isConnected = VivePose.IsConnected(HandRole.RightHand);

// 获取设备索引
uint deviceIndex = VivePose.GetDeviceIndex(HandRole.RightHand);
```

#### ViveRole（设备角色系统）

```csharp
using HTC.UnityPlugin.Vive;

// 获取设备索引
uint deviceIndex = ViveRole.GetDeviceIndex(HandRole.RightHand);
uint deviceIndex = ViveRole.GetDeviceIndexEx(TrackerRole.Tracker1);

// 验证设备索引有效性
bool isValid = VRModule.IsValidDeviceIndex(deviceIndex);
```

#### 预定义角色枚举

```csharp
// 手部角色
public enum HandRole { RightHand, LeftHand }

// 设备角色
public enum DeviceRole { Hmd, RightHand, LeftHand, ... }

// 追踪器角色
public enum TrackerRole { Tracker1, Tracker2, ... }

// 身体角色
public enum BodyRole { Head, Hip, Chest, ... }
```

#### VivePoseTracker（位姿追踪器）

将 VR 设备位姿应用到 GameObject：

```csharp
// 在 Inspector 中配置：
// - Vive Role: 选择要追踪的设备角色
// - Origin: 坐标原点（可选）
// - Pos Offset: 位置偏移
// - Rot Offset: 旋转偏移
```

#### ViveRaycaster（Vive 射线检测器）

用于 UI 交互的射线检测：

```csharp
// 在 Inspector 中配置：
// - Vive Role: 控制器角色
// - Mouse Button Left/Middle/Right: 对应的控制器按钮
// - Scroll Type: 滚动类型（Auto/Trackpad/Thumbstick）
// - Drag Threshold: 拖拽阈值
// - Click Interval: 点击间隔
```

#### ViveColliderEventCaster（Vive 碰撞事件发射器）

用于 3D 物体抓取交互：

```csharp
// 在 Inspector 中配置：
// - Vive Role: 控制器角色
// - Button Trigger: 扳机对应按钮
// - Button Pad Or Stick: 触摸板对应按钮
// - Button Grip Or Hand Trigger: 握把对应按钮
```

---

## 🛠️ 快速入门

### 基础场景设置

1. **添加 VR 相机**
   - 创建一个空 GameObject 作为 CameraRig
   - 添加 Main Camera 作为子对象

2. **添加 EventSystem**
   - 创建 EventSystem（如果没有会自动创建）
   - `Pointer3DInputModule` 会自动添加

3. **添加控制器**
   ```
   RightHand (空对象)
   ├── VivePoseTracker (设置 Role = RightHand)
   ├── ViveRaycaster (UI 交互用)
   └── ViveColliderEventCaster (3D 物体交互用)
   ```

### UI 交互设置

1. 创建 Canvas，设置 Render Mode 为 **World Space**
2. 添加 `GraphicRaycaster` 组件
3. 在控制器上添加 `ViveRaycaster`
4. UI 元素正常使用 Button、Toggle 等组件

### 3D 物体抓取设置

1. 在控制器上添加：
   - `ViveColliderEventCaster`
   - `Collider`（设为 Trigger）
   - `Rigidbody`（设为 Kinematic）

2. 在可抓取物体上添加：
   - `Collider`
   - `Rigidbody`
   - 实现 `IColliderEventDragStartHandler` 等接口

---

## 📝 常用代码示例

### 检测控制器按钮

```csharp
using HTC.UnityPlugin.Vive;

void Update()
{
    // 扳机按下瞬间
    if (ViveInput.GetPressDown(HandRole.RightHand, ControllerButton.Trigger))
    {
        Fire();
    }
    
    // 扳机持续按住
    if (ViveInput.GetPress(HandRole.RightHand, ControllerButton.Trigger))
    {
        Charge();
    }
    
    // 获取扳机力度
    float triggerForce = ViveInput.GetAxis(HandRole.RightHand, ControllerAxis.Trigger);
}
```

### 获取控制器位姿

```csharp
using HTC.UnityPlugin.Vive;
using HTC.UnityPlugin.Utility;

void Update()
{
    if (VivePose.IsValid(HandRole.RightHand))
    {
        RigidPose pose = VivePose.GetPose(HandRole.RightHand);
        transform.position = pose.pos;
        transform.rotation = pose.rot;
    }
}
```

### 实现可抓取物体

```csharp
using HTC.UnityPlugin.ColliderEvent;
using UnityEngine;

public class GrabbableObject : MonoBehaviour,
    IColliderEventDragStartHandler,
    IColliderEventDragUpdateHandler,
    IColliderEventDragEndHandler
{
    private Transform grabber;
    private RigidPose grabOffset;

    public void OnColliderEventDragStart(ColliderButtonEventData eventData)
    {
        grabber = eventData.eventCaster.transform;
        
        // 计算抓取偏移
        var grabberPose = new RigidPose(grabber);
        var objectPose = new RigidPose(transform);
        grabOffset = grabberPose.GetInverse() * objectPose;
    }

    public void OnColliderEventDragUpdate(ColliderButtonEventData eventData)
    {
        if (grabber != null)
        {
            var grabberPose = new RigidPose(grabber);
            var targetPose = grabberPose * grabOffset;
            
            transform.position = targetPose.pos;
            transform.rotation = targetPose.rot;
        }
    }

    public void OnColliderEventDragEnd(ColliderButtonEventData eventData)
    {
        grabber = null;
    }
}
```

### 传送（Teleport）

```csharp
using HTC.UnityPlugin.Vive;
using UnityEngine;

public class Teleporter : MonoBehaviour
{
    public Transform cameraRig;
    public LineRenderer lineRenderer;
    public LayerMask teleportLayer;

    void Update()
    {
        if (ViveInput.GetPress(HandRole.RightHand, ControllerButton.Pad))
        {
            // 显示传送射线
            ShowTeleportRay();
        }
        
        if (ViveInput.GetPressUp(HandRole.RightHand, ControllerButton.Pad))
        {
            // 执行传送
            TryTeleport();
        }
    }

    void ShowTeleportRay()
    {
        RigidPose pose = VivePose.GetPose(HandRole.RightHand);
        Ray ray = new Ray(pose.pos, pose.forward);
        
        if (Physics.Raycast(ray, out RaycastHit hit, 100f, teleportLayer))
        {
            lineRenderer.SetPosition(0, pose.pos);
            lineRenderer.SetPosition(1, hit.point);
        }
    }

    void TryTeleport()
    {
        RigidPose pose = VivePose.GetPose(HandRole.RightHand);
        Ray ray = new Ray(pose.pos, pose.forward);
        
        if (Physics.Raycast(ray, out RaycastHit hit, 100f, teleportLayer))
        {
            Vector3 offset = cameraRig.position - Camera.main.transform.position;
            offset.y = 0;
            cameraRig.position = hit.point + offset;
        }
    }
}
```

---

## ⚙️ 配置选项

### VRModule 设置

在 `VRModule` 组件或通过代码配置：

```csharp
// 选择 VR 模块
VRModule.selectModule = VRModuleSelectEnum.Auto;

// 锁定物理更新率到渲染帧率
VRModule.lockPhysicsUpdateRateToRenderFrequency = true;
```

### ViveInput 设置

```csharp
// 设置点击间隔
ViveInput.clickInterval = 0.3f;
```

### Pointer3DRaycaster 设置

```csharp
raycaster.dragThreshold = 0.02f;  // 拖拽阈值
raycaster.clickInterval = 0.3f;   // 点击间隔
raycaster.showDebugRay = true;    // 显示调试射线
```

---

## 🔧 调试技巧

1. **启用调试射线**：在 `Pointer3DRaycaster` 中设置 `showDebugRay = true`

2. **检查设备连接状态**：
   ```csharp
   for (uint i = 0; i < VRModule.MAX_DEVICE_COUNT; i++)
   {
       var state = VRModule.GetCurrentDeviceState(i);
       if (state.isConnected)
       {
           Debug.Log($"Device {i}: {state.deviceModel} - {state.serialNumber}");
       }
   }
   ```

3. **使用模拟器**：当没有 VR 设备时，可以设置 `VRModule.selectModule = VRModuleSelectEnum.Simulator`

---

## 📚 示例场景

插件提供了多个示例场景：

| 示例 | 说明 |
|------|------|
| 0. Tutorial | 基础环境和 UI 交互 |
| 1. UGUI | 更多 UI 组件示例 |
| 2. 2D DragDrop | 2D 拖放操作 |
| 3. 3D Drag | 3D 物体拖拽 |
| 4. Teleport | 传送移动 |
| 5. Collider Event | 碰撞事件和物理交互 |
| 6. Controller Manager Sample | 控制器交互示例 |
| 7. Role Binding Example | 设备角色绑定 |
| 8. Near-Field Hand Interaction | 近场手部交互 |
| 9. Tracked Hand UGUI Interaction | 手势追踪 UI 交互 |
| 10. Controller Tooltips | 控制器提示 |

---

## 📋 依赖要求

- Unity 2019.3 或更高版本
- com.unity.ugui 1.0.0 或更高版本
- 兼容 SteamVR 2.4.0+ 和 Oculus Integration 16.0+

---

## 🔗 相关资源

- [GitHub 仓库](https://github.com/ViveSoftware/ViveInputUtility-Unity)
- [官方文档](https://github.com/ViveSoftware/ViveInputUtility-Unity/wiki)
- [HTC 官网](https://www.htc.com)

---

*文档生成时间：2024年12月*
*HTC Vive Input Utility 版本：1.20.2*

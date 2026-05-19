# BasicGrabbable 3D 物体抓取完整指南

## 📋 概述

本指南详细介绍如何使用 HTC Vive Input Utility 插件内置的 `BasicGrabbable` 组件实现 VR 中的 3D 物体抓取功能。这是最简单、最可靠的实现方式，无需编写任何代码。

---

## 🎯 实现原理

### 工作流程

```
┌──────────────────┐     碰撞检测      ┌──────────────────┐
│   VR 控制器       │ ───────────────→ │   可抓取物体      │
│                  │                  │                  │
│ ViveColliderEvent│     按钮事件      │  BasicGrabbable  │
│     Caster       │ ←───────────────→│                  │
│                  │                  │                  │
│  Collider(触发器) │     拖拽事件      │  Collider+刚体   │
│  Rigidbody(运动学)│ ───────────────→ │                  │
└──────────────────┘                  └──────────────────┘
```

### 核心组件说明

| 组件 | 角色 | 作用 |
|------|------|------|
| `ViveColliderEventCaster` | 事件发送者 | 检测碰撞并发送抓取事件 |
| `BasicGrabbable` | 事件接收者 | 响应抓取事件，处理物体跟随 |
| `Collider` | 碰撞检测 | 定义可交互区域 |
| `Rigidbody` | 物理行为 | 控制运动方式和抛出效果 |

---

## 🛠️ 详细设置步骤

### 第一步：设置 VR 相机和控制器追踪

#### 1.1 创建相机装置

1. 在 Hierarchy 中创建空 GameObject，命名为 `[ViveRig]`
2. 将 `Main Camera` 作为其子对象
3. 在 `Main Camera` 上添加 `VivePoseTracker` 组件：
   - **Vive Role** → 选择 `DeviceRole.Hmd`

#### 1.2 创建右手控制器

1. 在 `[ViveRig]` 下创建空 GameObject，命名为 `RightController`
2. 添加 `VivePoseTracker` 组件：
   - **Vive Role** → 选择 `HandRole.RightHand`

#### 1.3 创建左手控制器

1. 在 `[ViveRig]` 下创建空 GameObject，命名为 `LeftController`
2. 添加 `VivePoseTracker` 组件：
   - **Vive Role** → 选择 `HandRole.LeftHand`

---

### 第二步：配置控制器为抓取器

#### 2.1 添加 ViveColliderEventCaster

在 `RightController` 上添加 `ViveColliderEventCaster` 组件：

**菜单路径**：`Add Component` → `VIU` → `Object Grabber` → `Vive Collider Event Caster (Grabber)`

**Inspector 配置**：

| 参数 | 设置值 | 说明 |
|------|--------|------|
| **Vive Role** | HandRole.RightHand | 绑定到右手控制器 |
| **Button Trigger** | Trigger | 扳机键作为主抓取键 |
| **Button Pad Or Stick** | Pad | 触摸板/摇杆 |
| **Button Grip Or Hand Trigger** | Grip | 握把键 |
| **Button Function Key** | Menu | 菜单键 |
| **Scroll Type** | Auto | 自动检测滚动类型 |

#### 2.2 添加 Rigidbody（必须）

在控制器上添加 `Rigidbody` 组件：

| 参数 | 设置值 | 说明 |
|------|--------|------|
| **Mass** | 1 | 质量（任意值即可） |
| **Use Gravity** | ❌ 取消勾选 | 控制器不受重力 |
| **Is Kinematic** | ✅ 勾选 | **必须启用**，由追踪数据控制位置 |

> ⚠️ **重要**：`Is Kinematic` 必须勾选！否则控制器会受物理引擎影响而掉落。

#### 2.3 添加 Collider（触发器模式）

在控制器上添加 Collider 组件（推荐 `SphereCollider`）：

| 参数 | 设置值 | 说明 |
|------|--------|------|
| **Is Trigger** | ✅ 勾选 | **必须启用**，使用触发器检测 |
| **Center** | (0, 0, 0.05) | 可根据控制器模型调整 |
| **Radius** | 0.05 | 抓取检测范围（5厘米） |

> 💡 **提示**：Radius 太小会导致难以触碰到物体，太大会导致误触发。建议 0.03-0.08 之间。

#### 2.4 左手控制器相同设置

对 `LeftController` 重复以上步骤，将 `Vive Role` 改为 `HandRole.LeftHand`。

---

### 第三步：配置可抓取物体

#### 3.1 创建可抓取物体

1. 创建一个 3D 物体（如 `Cube`、`Sphere` 或导入的模型）
2. 确保物体有 `Collider` 组件（MeshCollider、BoxCollider 等）
3. 确保 Collider 的 `Is Trigger` 为 **取消勾选**（非触发器）

#### 3.2 添加 BasicGrabbable 组件

**菜单路径**：`Add Component` → `VIU` → `Object Grabber` → `Basic Grabbable`

#### 3.3 BasicGrabbable 参数详解

##### 基础设置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| **Following Duration** | 0.04 | 物体跟随控制器的平滑时间（秒）。值越小跟随越紧，但可能抖动；值越大越平滑，但有延迟。推荐范围：0.02-0.06 |
| **Override Max Angular Velocity** | ✅ | 覆盖刚体最大角速度限制，使快速旋转更流畅 |
| **Unblockable Grab** | ✅ | 抓取时物体是否可穿透其他碰撞体。✅=可穿透（推荐），❌=会被墙壁阻挡 |

##### 抓取按钮设置

| 参数 | 说明 |
|------|------|
| **Primary Grab Button** | 主抓取按钮（Vive 控制器按钮）。可多选 |
| **Secondary Grab Button** | 次抓取按钮（通用按钮类型）。可多选，默认 Trigger |

> 💡 如果 Primary Grab Button 有任何选中，则优先使用；否则使用 Secondary Grab Button。

##### 抓取行为设置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| **Allow Multiple Grabbers** | ✅ | 是否允许多个控制器同时抓取（双手抓取）|
| **Grab On Last Entered** | ✅ | 只抓取最后进入碰撞范围的物体。✅=精确抓取，❌=可能抓到附近其他物体 |

##### 对齐设置（可选）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| **Align Position** | ❌ | 抓取时物体位置是否吸附到控制器 |
| **Align Rotation** | ❌ | 抓取时物体旋转是否对齐到控制器 |
| **Align Position Offset** | (0,0,0) | 位置吸附偏移量 |
| **Align Rotation Offset** | (0,0,0) | 旋转吸附偏移量（欧拉角） |

##### 缩放设置（双手拉伸）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| **Min Stretch Scale** | 1.0 | 双手拉伸时最小缩放比例 |
| **Max Stretch Scale** | 1.0 | 双手拉伸时最大缩放比例 |

> 💡 设置相同值（如都是 1.0）可禁用缩放功能。设置不同值（如 0.5 和 2.0）可启用双手拉伸缩放。

##### 事件回调

| 事件 | 触发时机 |
|------|---------|
| **After Grabbed** | 物体被抓取后触发 |
| **Before Release** | 物体释放前触发 |
| **On Drop** | 物体被抛出时触发（可在此修改抛出速度） |

#### 3.4 添加 Rigidbody（可选但推荐）

如果希望物体有物理行为（重力、碰撞、抛出）：

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| **Mass** | 1 | 物体质量 |
| **Use Gravity** | ✅ | 释放后受重力影响 |
| **Is Kinematic** | ❌ | 保持为非运动学，允许物理模拟 |
| **Interpolate** | Interpolate | 平滑渲染（可选） |
| **Collision Detection** | Continuous Dynamic | 快速移动时防止穿透（可选） |

---

## 📁 完整场景层级结构

```
Scene
│
├── [ViveRig]
│   │
│   ├── Main Camera
│   │   └── VivePoseTracker
│   │       └── Vive Role: DeviceRole.Hmd
│   │
│   ├── RightController
│   │   ├── VivePoseTracker
│   │   │   └── Vive Role: HandRole.RightHand
│   │   ├── ViveColliderEventCaster
│   │   │   ├── Vive Role: HandRole.RightHand
│   │   │   └── Button Trigger: Trigger
│   │   ├── SphereCollider
│   │   │   ├── Is Trigger: ✅
│   │   │   └── Radius: 0.05
│   │   ├── Rigidbody
│   │   │   ├── Use Gravity: ❌
│   │   │   └── Is Kinematic: ✅
│   │   └── (可选) ControllerModel
│   │
│   └── LeftController
│       ├── VivePoseTracker
│       │   └── Vive Role: HandRole.LeftHand
│       ├── ViveColliderEventCaster
│       │   ├── Vive Role: HandRole.LeftHand
│       │   └── Button Trigger: Trigger
│       ├── SphereCollider
│       │   ├── Is Trigger: ✅
│       │   └── Radius: 0.05
│       ├── Rigidbody
│       │   ├── Use Gravity: ❌
│       │   └── Is Kinematic: ✅
│       └── (可选) ControllerModel
│
├── GrabbableObjects
│   │
│   ├── Cube
│   │   ├── MeshRenderer
│   │   ├── BoxCollider (Is Trigger: ❌)
│   │   ├── Rigidbody (Use Gravity: ✅, Is Kinematic: ❌)
│   │   └── BasicGrabbable
│   │
│   ├── Sphere
│   │   ├── MeshRenderer
│   │   ├── SphereCollider (Is Trigger: ❌)
│   │   ├── Rigidbody (Use Gravity: ✅, Is Kinematic: ❌)
│   │   └── BasicGrabbable
│   │
│   └── CustomModel
│       ├── MeshRenderer
│       ├── MeshCollider (Is Trigger: ❌)
│       ├── Rigidbody (Use Gravity: ✅, Is Kinematic: ❌)
│       └── BasicGrabbable
│
├── Environment
│   ├── Ground (带 Collider)
│   └── Walls (带 Collider)
│
└── EventSystem (自动创建)
```

---

## ⚙️ 常用配置场景

### 场景 1：简单抓取（保持相对位置）

适用于：大多数可抓取物品

```
BasicGrabbable 配置：
├── Align Position: ❌
├── Align Rotation: ❌
├── Following Duration: 0.04
├── Unblockable Grab: ✅
├── Grab On Last Entered: ✅
└── Secondary Grab Button: Trigger ✅
```

**效果**：抓取时物体保持与控制器的相对位置关系。

---

### 场景 2：吸附抓取（工具类物品）

适用于：枪械、工具、手柄类物品

```
BasicGrabbable 配置：
├── Align Position: ✅
├── Align Rotation: ✅
├── Align Position Offset: (0, 0, 0.1)  // 向前偏移10厘米
├── Align Rotation Offset: (0, 0, 0)
├── Following Duration: 0.02
├── Unblockable Grab: ✅
└── Secondary Grab Button: Trigger ✅
```

**效果**：抓取时物体自动吸附到控制器的固定位置和朝向。

---

### 场景 3：双手缩放（可变形物体）

适用于：需要双手拉伸缩放的物体

```
BasicGrabbable 配置：
├── Allow Multiple Grabbers: ✅
├── Min Stretch Scale: 0.5
├── Max Stretch Scale: 3.0
├── Align Position: ❌
├── Align Rotation: ❌
└── Secondary Grab Button: Trigger ✅, Grip ✅
```

**效果**：单手抓取移动，双手抓取时可拉伸缩放物体。

---

### 场景 4：握把键抓取（区分交互）

适用于：扳机用于其他交互（如射击），握把用于抓取

```
BasicGrabbable 配置：
├── Primary Grab Button: Grip ✅ (其他都取消)
├── Secondary Grab Button: (全部取消)
├── Unblockable Grab: ✅
└── Grab On Last Entered: ✅

ViveColliderEventCaster 配置：
├── Button Trigger: Trigger
└── Button Grip Or Hand Trigger: Grip
```

**效果**：只有握把键才能抓取物体，扳机键可用于其他功能。

---

## 🔧 常见问题排查

### 问题 1：物体无法被抓取

**检查清单**：
- [ ] 控制器是否有 `ViveColliderEventCaster` 组件？
- [ ] 控制器是否有 `Rigidbody` 且 `Is Kinematic = true`？
- [ ] 控制器是否有 `Collider` 且 `Is Trigger = true`？
- [ ] 物体是否有 `BasicGrabbable` 组件？
- [ ] 物体是否有 `Collider`（非 Trigger）？
- [ ] Layer 碰撞矩阵是否允许两者碰撞？（Edit → Project Settings → Physics）

### 问题 2：物体抓取后位置跳动

**可能原因**：
- `Following Duration` 值太小
- 物体有多个 Collider 且设置不正确

**解决方案**：
```
将 Following Duration 调大到 0.04-0.06
确保只有一个主 Collider 用于抓取检测
```

### 问题 3：物体释放后穿透地面

**可能原因**：
- 物体移动太快，穿透检测失效
- Rigidbody 碰撞检测模式不正确

**解决方案**：
```
Rigidbody 设置：
├── Collision Detection: Continuous Dynamic
└── Interpolate: Interpolate
```

### 问题 4：双手抓取不生效

**可能原因**：
- `Allow Multiple Grabbers` 未勾选
- 两个控制器抓取按钮设置不同

**解决方案**：
```
BasicGrabbable 设置：
└── Allow Multiple Grabbers: ✅

确保两个控制器的 ViveColliderEventCaster 配置相同
```

### 问题 5：抓取时物体被墙挡住

**解决方案**：
```
BasicGrabbable 设置：
└── Unblockable Grab: ✅
```

---

## 📝 代码扩展：监听抓取事件

如果需要在抓取时执行自定义逻辑，可以通过 Inspector 事件或代码监听：

### 方式 1：Inspector 绑定（无需代码）

1. 在 `BasicGrabbable` 组件的 `After Grabbed` 事件中点击 `+`
2. 拖入需要响应的 GameObject
3. 选择要调用的方法

### 方式 2：代码监听

```csharp
using HTC.UnityPlugin.Vive;
using UnityEngine;

public class GrabEventListener : MonoBehaviour
{
    private BasicGrabbable grabbable;

    void Start()
    {
        grabbable = GetComponent<BasicGrabbable>();
        
        // 注册事件
        grabbable.afterGrabbed.AddListener(OnGrabbed);
        grabbable.beforeRelease.AddListener(OnReleased);
        grabbable.onDrop.AddListener(OnDropped);
    }

    void OnGrabbed(BasicGrabbable obj)
    {
        Debug.Log($"{obj.name} 被抓取了！");
        
        // 获取抓取者信息
        var eventData = obj.grabbedEvent;
        var grabberTransform = eventData.eventCaster.transform;
        Debug.Log($"抓取者位置: {grabberTransform.position}");
    }

    void OnReleased(BasicGrabbable obj)
    {
        Debug.Log($"{obj.name} 即将被释放");
    }

    void OnDropped(BasicGrabbable obj)
    {
        Debug.Log($"{obj.name} 被抛出");
        
        // 可以在这里修改抛出速度
        var rb = obj.GetComponent<Rigidbody>();
        if (rb != null)
        {
            // 例如：增加向上的力
            rb.velocity += Vector3.up * 2f;
        }
    }

    void OnDestroy()
    {
        // 取消注册
        if (grabbable != null)
        {
            grabbable.afterGrabbed.RemoveListener(OnGrabbed);
            grabbable.beforeRelease.RemoveListener(OnReleased);
            grabbable.onDrop.RemoveListener(OnDropped);
        }
    }
}
```

---

## ✅ 快速检查清单

### 控制器配置检查

| 检查项 | 状态 |
|--------|------|
| VivePoseTracker 组件已添加 | ☐ |
| ViveColliderEventCaster 组件已添加 | ☐ |
| Rigidbody 组件已添加 | ☐ |
| Rigidbody.Is Kinematic = true | ☐ |
| Collider 组件已添加 | ☐ |
| Collider.Is Trigger = true | ☐ |
| ViveRole 设置正确（Left/Right Hand） | ☐ |

### 可抓取物体配置检查

| 检查项 | 状态 |
|--------|------|
| BasicGrabbable 组件已添加 | ☐ |
| Collider 组件已添加 | ☐ |
| Collider.Is Trigger = false | ☐ |
| Rigidbody 组件已添加（如需物理） | ☐ |
| Rigidbody.Is Kinematic = false（如需物理） | ☐ |
| 抓取按钮已正确配置 | ☐ |

---

## 📚 相关示例场景

插件包含的相关示例：
- `ViveInputUtility/Examples/3.3DDrag` - 3D 拖拽示例
- `ViveInputUtility/Examples/5.ColliderEvent` - 碰撞事件示例

可以参考这些示例场景了解更多用法。

---

*文档版本：1.0*
*适用插件版本：HTC Vive Input Utility 1.20.2+*
*最后更新：2024年12月*

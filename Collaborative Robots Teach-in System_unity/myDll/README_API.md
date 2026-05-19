# myDll DLL API 使用说明

## 概述

`myDll` 是一个用于机器人视觉与标定的 Windows 动态链接库（DLL）。
它实现了两个主要功能：

- `calculateNeedleTip`：探针尖端位置计算
- `calculateHandAndEye`：手眼标定（眼在手外 / Eye-to-Hand 场景）

此外提供了简单的测试导出函数 `Sum` 与 `Multiplication`，用于验证 DLL 导出和调用。

目标读者：需要在 C/C++/C#/Python 等语言中调用本 DLL 的工程师。

---

## 构建与平台

- 平台：Windows
- 编译器：Visual Studio (建议 2019 或更高)
- 构建配置：x64 Release/Debug
- 依赖：Eigen、OpenCV（请在项目属性中配置 include/lib 路径）

---

## 导出接口 (C 接口，extern "C")

头文件：`myDll.h`

下面列出所有导出的函数、参数与返回值：

1. int __stdcall calculateNeedleTip(Point_Unity* input, Vector3D_Unity* output)
   - 功能：计算探针尖端在探针坐标系下的位置
   - 输入：`Point_Unity* input`
     - `MarkNum`：探针上标记（Mark）数量
     - `PointNum`：每个 Mark 的测量次数
     - `Points[1024]`：位姿数组，按 `Points[j + PointNum * i]` 索引（第 i 个 Mark，第 j 次测量）
   - 输出：`Vector3D_Unity* output`
     - `x,y,z`：探针尖端在探针坐标系中的坐标
   - 返回值：0 表示成功
   - 注意：需要至少 2 次不同姿态的测量（更推荐 5~10 组）

2. int __stdcall calculateHandAndEye(Point_Unity* gripper2base, Point_Unity* target2cam, Pose_Unity* cam2gripper)
   - 功能：手眼标定（眼在手外 / Eye-to-Hand 场景），计算相机相对于机器人基座的位姿（cam2base）
   - 输入：
     - `gripper2base`：机械臂末端相对于机器人基座的位姿序列（来自机器人控制器）
       - `PointNum`：位姿对数量
       - `Points[i]`：每组位姿（Position + Quaternion）
     - `target2cam`：标定板相对于相机的位姿序列（来自视觉算法，如 solvePnP）
       - 需与 `gripper2base` 一一对应，采样同时刻
   - 输出（参数名：`cam2gripper`，但实际含义如下）
     - `cam2gripper` 实际表示 `cam2base`（相机相对于机器人基座的变换）
       - `Position`：相机原点在机器人基座坐标系中的位置
       - `Quaternion`：相机到基座的旋转四元数
   - 返回值：0 成功，1 输入数据长度不匹配
   - 备注：OpenCV 的 `calibrateHandEye` 使用 Tsai 算法。为了在“眼在手外”场景下使用，函数内部对输入/输出变量有映射，见下表。

   映射关系（眼在手外场景）：

   - 本函数输入 `R_gripper2base` / `t_gripper2base` 对应机械臂末端在基座下的位姿
   - 本函数输入 `R_target2cam` / `t_target2cam` 对应标定板在相机下的位姿
   - 本函数输出 `R_cam2gripper` / `t_cam2gripper` 实际等同于 `R_cam2base` / `T_cam2base`

3. int __stdcall Sum(int value1, int value2)
   - 功能：测试函数，返回两数之和

4. int __stdcall Multiplication(int value1, int value2)
   - 功能：测试函数，返回两数之积

---

## 导出数据结构说明

以下结构体定义在 `myDll.h`。

- Vector3D_Unity
  - double x, y, z;

- Quaternion_Unity
  - double w, x, y, z; // 四元数，建议归一化

- Pose_Unity
  - Vector3D_Unity Position;
  - Quaternion_Unity Quaternion;

- Point_Unity
  - int MarkNum;  // 标记数量
  - int PointNum; // 每个标记的测量次数
  - Pose_Unity Points[1024];

---

## 使用示例

下面示例展示如何从不同语言调用 DLL。请按需调整结构体定义与内存布局，确保与C端保持一致。

### C/C++ 示例（动态加载）

```cpp
#include <windows.h>
#include "myDll.h"

int main() {
    HMODULE h = LoadLibrary(TEXT("myDll.dll"));
    if (!h) return -1;

    auto calc = (int (__stdcall *)(Point_Unity*, Vector3D_Unity*))GetProcAddress(h, "calculateNeedleTip");
    Point_Unity in{};
    Vector3D_Unity out{};
    // 填充 in 数据
    calc(&in, &out);
    FreeLibrary(h);
    return 0;
}
```

### C# 示例（P/Invoke）

```csharp
[StructLayout(LayoutKind.Sequential)]
public struct Vector3D_Unity { public double x,y,z; }
[StructLayout(LayoutKind.Sequential)]
public struct Quaternion_Unity { public double w,x,y,z; }
[StructLayout(LayoutKind.Sequential)]
public struct Pose_Unity { public Vector3D_Unity Position; public Quaternion_Unity Quaternion; }
[StructLayout(LayoutKind.Sequential)]
public struct Point_Unity { public int MarkNum; public int PointNum; [MarshalAs(UnmanagedType.ByValArray, SizeConst=1024)] public Pose_Unity[] Points; }

[DllImport("myDll.dll", CallingConvention=CallingConvention.StdCall)]
public static extern int calculateNeedleTip(ref Point_Unity input, ref Vector3D_Unity output);

// 使用相同方式导入 calculateHandAndEye
```

### Python 示例（ctypes）

```python
import ctypes

class Vector3D_Unity(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double),("y", ctypes.c_double),("z", ctypes.c_double)]

class Quaternion_Unity(ctypes.Structure):
    _fields_ = [("w", ctypes.c_double),("x", ctypes.c_double),("y", ctypes.c_double),("z", ctypes.c_double)]

class Pose_Unity(ctypes.Structure):
    _fields_ = [("Position", Vector3D_Unity),("Quaternion", Quaternion_Unity)]

class Point_Unity(ctypes.Structure):
    _fields_ = [("MarkNum", ctypes.c_int),("PointNum", ctypes.c_int),("Points", Pose_Unity*1024)]

dll = ctypes.CDLL("myDll.dll")
dll.calculateNeedleTip.argtypes = [ctypes.POINTER(Point_Unity), ctypes.POINTER(Vector3D_Unity)]
dll.calculateNeedleTip.restype = ctypes.c_int

p = Point_Unity()
# 填充 p
out = Vector3D_Unity()
ret = dll.calculateNeedleTip(ctypes.byref(p), ctypes.byref(out))
```

---

## 注意事项与建议

- 四元数应归一化以避免数值问题
- 输入两组序列 `gripper2base` 与 `target2cam` 必须长度一致且一一对应
- 在眼在手外场景中，相机应固定不动，标定板跟随机械臂一同移动
- 输出参数 `cam2gripper` 实际为 `cam2base`，请在上层代码中按需重命名以避免混淆

---

## 常见问题

Q: 为什么输出参数名是 `cam2gripper` 但表示的是 `cam2base`？

A: 这是为了最大限度复用 OpenCV 的 `calibrateHandEye` 接口和原有代码结构。内部做了输入/输出的坐标含义映射，使得在眼在手外场景下也能正确得到相机相对于基座的变换。文档已在多个位置强调这一点，请务必注意。

---

## 联系与维护

如需协助或扩展（例如添加眼在手上场景的支持），请在项目仓库提交Issue或联系维护者。

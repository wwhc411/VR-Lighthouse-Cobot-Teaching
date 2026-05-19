# myDll 项目说明文档

## 📋 项目概述

这是一个用于机器人视觉系统的动态链接库(DLL)项目，提供了探针标定和手眼标定两个核心功能。该项目使用C++开发，基于Eigen线性代数库和OpenCV计算机视觉库实现。

### 主要功能

1. **探针尖端位置计算** (`calculateNeedleTip`)
   - 通过多次测量不同姿态下的探针Mark位置
   - 使用最小二乘法计算固定不动的探针尖端坐标
   - 输出探针尖端在探针坐标系中的位置

2. **手眼标定 - 眼在手外场景** (`calculateHandAndEye`)
   - 计算相机相对于机器人基座的位姿关系
   - 适用于相机固定在机器人外部，观察机械臂末端抓持标定板的场景
   - 使用OpenCV的Tsai算法求解AX=XB方程
   - 用于机器人视觉伺服和坐标系转换

3. **测试函数** (`Sum`, `Multiplication`)
   - 简单的加法和乘法函数
   - 用于验证DLL导出和调用功能是否正常

---

## 📁 项目文件结构

```
myDll/
├── myDll.h              # 头文件：定义导出接口和数据结构
├── myDll.cpp            # 实现文件：核心算法实现
├── dllmain.cpp          # DLL入口点：处理加载/卸载事件
├── framework.h          # Windows框架头文件
├── pch.h                # 预编译头文件定义
├── pch.cpp              # 预编译头实现文件
├── myDll.sln            # Visual Studio解决方案文件
├── myDll.vcxproj        # Visual Studio项目文件
├── myDll.vcxproj.filters# 项目过滤器配置
├── myDll.vcxproj.user   # 用户特定配置
├── eigen5/              # Eigen数学库（第三方依赖）
├── opencv/              # OpenCV计算机视觉库（第三方依赖）
└── x64/Release/         # 编译输出目录
```

---

## 🔧 核心数据结构

### 1. Vector3D_Unity
三维向量结构，用于表示空间位置或方向。
```cpp
typedef struct {
    double x;  // X轴坐标
    double y;  // Y轴坐标
    double z;  // Z轴坐标
} Vector3D_Unity;
```

### 2. Quaternion_Unity
四元数结构，用于表示三维旋转（无万向锁问题）。
```cpp
typedef struct {
    double w;  // 实部
    double x;  // i分量
    double y;  // j分量
    double z;  // k分量
} Quaternion_Unity;
```
满足归一化条件：w² + x² + y² + z² = 1

### 3. Pose_Unity
位姿结构，完整描述刚体的位置和姿态。
```cpp
typedef struct {
    Vector3D_Unity Position;      // 位置（平移）
    Quaternion_Unity Quaternion;  // 姿态（旋转）
} Pose_Unity;
```

### 4. Point_Unity
点云数据结构，存储多组测量数据。
```cpp
typedef struct {
    int MarkNum;           // Mark标记数量
    int PointNum;          // 每个Mark的测量次数
    Pose_Unity Points[1024]; // 位姿数组（最多1024个）
} Point_Unity;
```
数据索引：`Points[j + PointNum * i]` 表示第i个Mark的第j次测量

---

## 🎯 核心算法详解

### 算法1：探针尖端位置计算

#### 原理
探针尖端在空间中是固定点，通过多次不同姿态的测量建立超定方程组：
```
设探针尖端坐标为 (px, py, pz)
对于每次测量j相对于参考测量1：
2(x_j - x_1)·px + 2(y_j - y_1)·py + 2(z_j - z_1)·pz = ||Mark_j||² - ||Mark_1||²
```

#### 步骤
1. 提取输入数据维度（MarkNum, PointNum）
2. 构建系数矩阵A（(PointNum-1)×MarkNum 行，3列）
3. 构建常数向量B（(PointNum-1)×MarkNum 个元素）
4. 使用QR分解求解最小二乘问题：Ax = B
5. 将相机坐标系下的结果转换到探针坐标系
6. 对所有测量取平均，输出最终结果

#### 坐标系转换
```cpp
// 相机坐标系 -> 探针坐标系
P_probe = R_inv × (P_camera - T)
其中：
R_inv = R_Probe2Camera^T  // 旋转矩阵的逆等于转置
T = T_Probe2Camera         // 平移向量
```

### 算法2：手眼标定（眼在手外场景）

#### 应用场景
**眼在手外(Eye-to-Hand)**：相机固定在机器人外部（如工作台上方），机械臂末端抓持标定板，移动到不同位置，相机观察标定板的位姿变化。

#### 原理
求解方程 **AX = XB**，其中：
- **A**: 机械臂末端在基座坐标系中的变换（gripper2base）
- **B**: 标定板在相机坐标系中的变换（target2cam）
- **X**: 相机相对于基座的变换（**cam2base**，待求解）

⚠️ **重要说明**：虽然函数输出参数名为`cam2gripper`，但在眼在手外场景下，实际求解并输出的是**相机相对于机器人基座的变换关系(cam2base)**。

#### Tsai算法输入输出映射（眼在手外场景）

| 本函数参数 | Tsai算法符号 | 实际物理含义 |
|-----------|-------------|-------------|
| `R_gripper2base` (输入) | R_base2gripper⁻¹ | 末端到基座的旋转 |
| `T_gripper2base` (输入) | T_base2gripper⁻¹ | 末端到基座的平移 |
| `R_target2cam` (输入) | R_target2cam | 标定板到相机的旋转 |
| `T_target2cam` (输入) | T_target2cam | 标定板到相机的平移 |
| `R_cam2gripper` (输出) | **R_cam2base** | 相机到基座的旋转 |
| `T_cam2gripper` (输出) | **T_cam2base** | 相机到基座的平移 |

#### 步骤
1. 验证输入数据长度一致性
2. 遍历所有位姿对：
   - 将Eigen四元数转换为旋转矩阵
   - 将Eigen格式转换为OpenCV Mat格式
3. 调用 `cv::calibrateHandEye()` 使用Tsai算法求解
4. 将结果从OpenCV Mat转换回Eigen格式
5. 将旋转矩阵转换为四元数输出

#### 数据要求
- 至少需要3组不同位姿的测量数据
- 建议采集5-10组以获得稳定结果
- 机械臂姿态变化应足够大，覆盖工作空间，避免接近奇异位置
- 相机应固定不动，标定板跟随机械臂移动

---

## 🚀 使用方法

### C/C++调用示例

```cpp
// 1. 加载DLL
HMODULE hDll = LoadLibrary(TEXT("myDll.dll"));

// 2. 获取函数指针
typedef int (_stdcall *CalculateNeedleTipFunc)(Point_Unity*, Vector3D_Unity*);
CalculateNeedleTipFunc calcNeedleTip = 
    (CalculateNeedleTipFunc)GetProcAddress(hDll, "calculateNeedleTip");

// 3. 准备输入数据
Point_Unity input;
input.MarkNum = 2;
input.PointNum = 10;
// ... 填充Points数据 ...

Vector3D_Unity output;

// 4. 调用函数
int result = calcNeedleTip(&input, &output);

// 5. 使用结果
if (result == 0) {
    printf("探针尖端: (%f, %f, %f)\n", output.x, output.y, output.z);
}

// 6. 释放DLL
FreeLibrary(hDll);
```

### C#调用示例

```csharp
// 1. 定义结构体
[StructLayout(LayoutKind.Sequential)]
public struct Vector3D_Unity {
    public double x, y, z;
}

[StructLayout(LayoutKind.Sequential)]
public struct Quaternion_Unity {
    public double w, x, y, z;
}

[StructLayout(LayoutKind.Sequential)]
public struct Pose_Unity {
    public Vector3D_Unity Position;
    public Quaternion_Unity Quaternion;
}

[StructLayout(LayoutKind.Sequential)]
public struct Point_Unity {
    public int MarkNum;
    public int PointNum;
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 1024)]
    public Pose_Unity[] Points;
}

// 2. 导入DLL函数
[DllImport("myDll.dll", CallingConvention = CallingConvention.StdCall)]
public static extern int calculateNeedleTip(
    ref Point_Unity input, 
    ref Vector3D_Unity output
);

[DllImport("myDll.dll", CallingConvention = CallingConvention.StdCall)]
public static extern int calculateHandAndEye(
    ref Point_Unity gripper2base,
    ref Point_Unity target2cam,
    ref Pose_Unity cam2gripper
);

// 3. 使用
Point_Unity input = new Point_Unity();
input.MarkNum = 2;
input.PointNum = 10;
input.Points = new Pose_Unity[1024];
// ... 填充数据 ...

Vector3D_Unity output = new Vector3D_Unity();
int result = calculateNeedleTip(ref input, ref output);
```

### Python调用示例

```python
import ctypes
import numpy as np

# 1. 定义结构体
class Vector3D_Unity(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double),
                ("y", ctypes.c_double),
                ("z", ctypes.c_double)]

class Quaternion_Unity(ctypes.Structure):
    _fields_ = [("w", ctypes.c_double),
                ("x", ctypes.c_double),
                ("y", ctypes.c_double),
                ("z", ctypes.c_double)]

class Pose_Unity(ctypes.Structure):
    _fields_ = [("Position", Vector3D_Unity),
                ("Quaternion", Quaternion_Unity)]

class Point_Unity(ctypes.Structure):
    _fields_ = [("MarkNum", ctypes.c_int),
                ("PointNum", ctypes.c_int),
                ("Points", Pose_Unity * 1024)]

# 2. 加载DLL
dll = ctypes.CDLL("myDll.dll")

# 3. 设置函数签名
dll.calculateNeedleTip.argtypes = [
    ctypes.POINTER(Point_Unity),
    ctypes.POINTER(Vector3D_Unity)
]
dll.calculateNeedleTip.restype = ctypes.c_int

# 4. 使用
input_data = Point_Unity()
input_data.MarkNum = 2
input_data.PointNum = 10
# ... 填充数据 ...

output_data = Vector3D_Unity()
result = dll.calculateNeedleTip(
    ctypes.byref(input_data),
    ctypes.byref(output_data)
)

print(f"结果: ({output_data.x}, {output_data.y}, {output_data.z})")
```

---

## 🔨 编译说明

### 环境要求
- **操作系统**: Windows 10/11
- **编译器**: Visual Studio 2019或更高版本
- **C++标准**: C++14或更高
- **依赖库**:
  - Eigen 3.x（线性代数库）
  - OpenCV 4.x（计算机视觉库）

### 编译步骤

1. **打开项目**
   ```
   双击 myDll.sln 打开Visual Studio
   ```

2. **配置依赖库路径**
   - 右键项目 → 属性 → C/C++ → 常规 → 附加包含目录
   - 添加Eigen和OpenCV的include路径

3. **配置链接器**
   - 属性 → 链接器 → 常规 → 附加库目录
   - 添加OpenCV的lib路径
   - 属性 → 链接器 → 输入 → 附加依赖项
   - 添加OpenCV库文件（如opencv_world453.lib）

4. **选择配置**
   - 平台：x64
   - 配置：Release（推荐）或Debug

5. **编译**
   ```
   菜单：生成 → 生成解决方案
   或按 Ctrl+Shift+B
   ```

6. **输出文件**
   ```
   x64\Release\myDll.dll  (动态链接库)
   x64\Release\myDll.lib  (导入库)
   ```

### 编译选项说明
- `/std:c++14` - C++14标准
- `/O2` - 优化（Release模式）
- `/Yc` - 创建预编译头（pch.cpp）
- `/Yu` - 使用预编译头（其他.cpp）

---

## 📊 性能优化

### 预编译头技术
本项目使用预编译头（PCH）技术加速编译：
- **pch.h**: 包含稳定的头文件（Windows API等）
- **pch.cpp**: 生成预编译头二进制缓存
- 可将编译时间缩短50%以上

### 算法复杂度
- **探针标定**: O(n·m)，n为Mark数量，m为测量次数
- **手眼标定**: O(k)，k为位姿对数量
- 两者均为线性复杂度，实时性良好

---

## ⚠️ 注意事项

### 1. 数据输入要求
- 所有四元数必须归一化（模长为1）
- 坐标系遵循右手定则
- 位姿数据必须准确，噪声会影响标定精度

### 2. 手眼标定建议（眼在手外场景）
- **相机安装**：相机应牢固固定在机器人外部（如支架、工作台上方），保持静止
- **标定板安装**：标定板固定在机械臂末端执行器上，随机械臂一起运动
- **数据采集**：至少采集5组不同位姿的数据，建议8-10组以上
- **运动范围**：机械臂移动范围应覆盖实际工作空间，姿态变化要充分
- **避免奇异**：避免姿态变化过小或接近奇异位置
- **视觉质量**：标定板应始终在相机视野内，清晰可见，特征点检测准确
- **输出理解**：函数输出的`cam2gripper`参数实际代表`cam2base`（相机到基座的变换）

### 3. 探针标定建议
- 每个Mark至少测量2次不同姿态
- 姿态变化应足够大以提供良好的约束
- 探针尖端应保持固定不动

### 4. 线程安全
- 当前实现未考虑线程安全
- 多线程调用需自行添加互斥锁

---

## 🐛 错误码说明

| 返回值 | 含义 | 处理建议 |
|--------|------|----------|
| 0 | 成功 | - |
| 1 | 输入数据长度不匹配 | 检查gripper2base和target2cam的PointNum是否相等 |

---

## 📚 参考文献

1. **探针标定算法**
   - [opencv+Eigen 手眼标定探针尖端标注](https://blog.csdn.net/2301_76925998/article/details/145118871)

2. **手眼标定算法**
   - [OpenCV手眼标定详解](https://blog.csdn.net/qq_19319481/article/details/150462358)
   - [OpenCV官方文档 - calibrateHandEye](https://docs.opencv.org/4.5.3/d9/d0c/group__calib3d.html)
   - [手眼标定视频教程](https://www.bilibili.com/video/BV1By4y1b7Q7)

3. **Tsai算法原论文**
   - R.Y. Tsai and R.K. Lenz, "A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration"

---

## 📧 技术支持

如有问题或建议，欢迎通过以下方式联系：
- 提交Issue到项目仓库
- 邮件联系项目维护者

---

## 📝 版本历史

- **v1.0** (2025) - 初始版本
  - 实现探针尖端位置计算
  - 实现手眼标定功能
  - 添加测试函数

---

## 📄 许可证

本项目依赖以下开源库：
- **Eigen**: MPL2 License
- **OpenCV**: Apache 2.0 License

请遵守相关开源协议。

---

**最后更新时间**: 2025年10月22日

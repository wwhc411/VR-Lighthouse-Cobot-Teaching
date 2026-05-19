# Plugins文件夹说明

## 📁 DLL部署指南

本文件夹用于存放手眼标定系统所需的动态链接库（DLL）。

---

## ⚠️ 重要提示

**必须**将您的`myDll.dll`放置在此文件夹中！

```
Unity项目/
└── Assets/
    └── Plugins/
        └── myDll.dll  ← 放这里！
```

如果DLL不在此位置,系统将无法加载，会报错：
```
DllNotFoundException: Unable to load DLL 'myDll'
```

---

## 📋 DLL信息

### myDll.dll

- **版本**: 1.0
- **架构**: x64 (64位)
- **依赖**: 
  - Eigen 3.x (线性代数库)
  - OpenCV 4.x (计算机视觉库)
- **功能**:
  - 手眼标定（Tsai算法）
  - 探针尖端位置计算
  - 测试函数（Sum, Multiplication）

### 文件大小

约 **几百KB 到 几MB**（取决于是否包含调试符号）

---

## 🔧 Unity Editor设置

将DLL拖入此文件夹后，在Unity中选中`myDll.dll`，在Inspector窗口配置：

### Platform Settings（平台设置）

```
✅ Editor
   - CPU: x86_64
   
✅ Standalone
   - CPU: x86_64
   
❌ Android
❌ iOS
❌ WebGL
```

### Load Settings（加载设置）

```
✅ Load on startup (启动时加载)
❌ Lazy load (延迟加载)
```

---

## 🧪 验证DLL

### 方法1: 在Unity Console查看

运行游戏后，查看Console输出：

```
[DLL测试] DLL加载成功！Sum(3, 5) = 8
```

### 方法2: 使用代码测试

```csharp
using HandEyeCalibration.DLL;

void Start()
{
    if (DllInterface.TestDllConnection())
    {
        Debug.Log("DLL正常工作！");
    }
    else
    {
        Debug.LogError("DLL加载失败！");
    }
}
```

---

## ❌ 常见错误

### 错误1: "找不到指定的模块"

**完整错误**:
```
DllNotFoundException: Unable to load DLL 'myDll': 
The specified module could not be found.
```

**可能原因**:
1. DLL不在Plugins文件夹
2. DLL架构不匹配（32位/64位）
3. 缺少DLL依赖项

**解决方案**:
1. 确认DLL位置正确
2. 检查Unity Editor是x64还是x86
3. 使用Dependency Walker检查DLL依赖

### 错误2: "入口点未找到"

**完整错误**:
```
EntryPointNotFoundException: Unable to find an entry point named 
'calculateHandAndEye' in DLL 'myDll'.
```

**可能原因**:
1. DLL函数名不匹配
2. 调用约定错误（StdCall vs Cdecl）
3. DLL版本不正确

**解决方案**:
1. 使用DLL Export Viewer检查导出函数名
2. 确认C#中的`CallingConvention.StdCall`
3. 重新编译DLL

---

## 📦 打包发布

### Windows Standalone

DLL会自动复制到输出目录：

```
YourGame_Data/
└── Plugins/
    └── myDll.dll
```

### 其他平台

当前DLL为Windows x64专用，如需支持其他平台：

- **macOS**: 需要编译`.dylib`文件
- **Linux**: 需要编译`.so`文件
- **Android**: 需要编译`.so`（ARM架构）
- **iOS**: 不支持动态库，需使用静态链接

---

## 🔍 DLL依赖检查

### 使用Dependency Walker

1. 下载 [Dependency Walker](http://www.dependencywalker.com/)
2. 打开`myDll.dll`
3. 查看依赖项列表

**常见依赖**:
- `KERNEL32.DLL` (Windows系统库)
- `MSVCR**.DLL` (Visual C++运行时)
- `opencv_world***.dll` (如果OpenCV动态链接)

### 如果缺少运行时

安装 **Visual C++ Redistributable**:
- [下载地址](https://support.microsoft.com/en-us/help/2977003/the-latest-supported-visual-c-downloads)
- 选择x64版本
- 安装后重启Unity

---

## 📚 相关文档

- **DLL API详情**: 查看`Documentation/DLL集成说明.md`
- **DLL源码说明**: 查看`README_项目说明.md`
- **故障排除**: 查看`Documentation/快速入门.md`

---

## 📝 检查清单

在开始使用前，请确认：

- [ ] `myDll.dll`已放入此文件夹
- [ ] DLL架构为x64（与Unity Editor匹配）
- [ ] Unity中DLL的Platform Settings已正确配置
- [ ] 运行测试代码，Console显示"DLL加载成功"
- [ ] 没有DllNotFoundException错误

---

**最后更新**: 2025年1月

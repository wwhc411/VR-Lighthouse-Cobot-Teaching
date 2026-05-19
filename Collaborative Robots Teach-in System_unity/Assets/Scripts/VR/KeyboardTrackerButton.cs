using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR;
using System.Runtime.InteropServices; // 与 C++ DLL 交互时需要的命名空间（封送/非托管内存）

/// <summary>
/// KeyboardTrackerButton: 用于响应绑定到键盘位置的 Tracker 的 SteamVR 按钮事件
/// 主要功能：
/// - 注册 SteamVR 的布尔输入（Grip/Trigger/Trackpad/Menu/Power）事件
/// - 使用 SteamVR_Input_Sources.Keyboard 作为输入源
/// - 支持探针标定数据采集和手眼标定数据采集
/// 注意：与 TriggerTester.cs 功能相同，但绑定到不同的输入源（Keyboard 而非 Camera）
/// </summary>
public class KeyboardTrackerButton : MonoBehaviour
{
    // 从 SteamVR Input 系统定义的布尔动作（在 Inspector 中绑定）
    public SteamVR_Action_Boolean booleanGrip;
    public SteamVR_Action_Boolean booleanPower;
    public SteamVR_Action_Boolean booleanTrigger;
    public SteamVR_Action_Boolean booleanTrackpad;
    public SteamVR_Action_Boolean booleanMenu;

    [Header("探针标定设置")]
    [Tooltip("ViveTrackerPoseLogger 组件引用（用于获取 Tracker 位姿数据）")]
    public ViveTrackerPoseLogger trackerPoseLogger;

    [Tooltip("键盘位置 Tracker 的设备序列号（用于数据采集，默认为 device2）")]
    public uint keyboardTrackerDeviceId = 2;

    [Tooltip("是否使用实时采集模式（true: 实时捕获 Tracker 数据 | false: 使用预设数据）")]
    public bool useRealTimeCapture = false;

    [Tooltip("实时采集时需要的测量次数")]
    public int captureCount = 10;

    // 存储实时采集的位姿数据
    private List<Pose_Unity> capturedPoses = new List<Pose_Unity>();


    // --------------------------------------------------
    // 数据结构声明（与本地 DLL 的结构体保持二进制兼容）
    // 使用 [StructLayout(LayoutKind.Sequential)] 明确字段顺序，避免 CLR 优化导致内存布局变化
    // --------------------------------------------------

    [StructLayout(LayoutKind.Sequential)]
    struct Vector3D_Unity
    {
        // 使用 double 类型以匹配可能的本地实现（注意：Unity 自身通常使用 float）
        public double x;
        public double y;
        public double z;
    };

    [StructLayout(LayoutKind.Sequential)]
    struct Quaternion_Unity
    {
        // 四元数按 w,x,y,z 顺序封送（与本地代码约定一致）
        public double w;
        public double x;
        public double y;
        public double z;
    };

    [StructLayout(LayoutKind.Sequential)]
    struct Pose_Unity
    {
        // 一个位姿由位置（Vector3D_Unity）和四元数（Quaternion_Unity）组成
        public Vector3D_Unity Position;
        public Quaternion_Unity Quaternion;
    };

    [StructLayout(LayoutKind.Sequential)]
    struct Point_Unity
    {
        // 标记数量（例如不同的 marker）
        public int MarkNum; // 标记数量
        // 每个标记的点数（每个 marker 下的观测点数量）
        public int PointNum; // 每个标记的点数
        // Points 数组预留 1024 个 Pose_Unity，大小固定以便于直接内存拷贝
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 1024)]
        public Pose_Unity[] Points;
    };


    // --------------------------------------------------
    // 本地 DLL 导入（示例）
    // EntryPoint：DLL 中导出函数名；CallingConvention：调用约定需与 DLL 一致
    // --------------------------------------------------

    [DllImport("myDll.dll", EntryPoint = "calculateNeedleTip", CallingConvention = CallingConvention.StdCall)]
    private static extern int calculateNeedleTip(IntPtr pv1, IntPtr pv2);

    [DllImport("myDll.dll", EntryPoint = "calculateHandAndEye", CallingConvention = CallingConvention.StdCall)]
    private static extern int calculateHandAndEye(IntPtr pv1, IntPtr pv2, IntPtr pv3);


    [DllImport("myDll.dll", EntryPoint = "Sum", CallingConvention = CallingConvention.StdCall)]
    static extern int Sum(int i1, int i2);

    [DllImport("myDll.dll", EntryPoint = "Multiplication", CallingConvention = CallingConvention.StdCall)]
    static extern int Multiplication(int i1, int i2);

    // --------------------------------------------------
    // Unity 生命周期：Start() - 注册事件；OnDestroy() - 注销事件
    // --------------------------------------------------
    void Start()
    {
        // 将各个 boolean 动作的按下事件绑定到回调函数
        // 这里使用 SteamVR_Input_Sources.Keyboard 作为输入源（绑定到键盘位置的 Tracker）
        // 【修复】添加空检查，防止 SteamVR 未初始化时崩溃
        if (booleanGrip != null && booleanGrip[SteamVR_Input_Sources.Keyboard] != null)
            booleanGrip[SteamVR_Input_Sources.Keyboard].onStateDown += OnStateDownGrip;
        if (booleanPower != null && booleanPower[SteamVR_Input_Sources.Keyboard] != null)
            booleanPower[SteamVR_Input_Sources.Keyboard].onStateDown += OnStateDownPower;
        if (booleanTrigger != null && booleanTrigger[SteamVR_Input_Sources.Keyboard] != null)
            booleanTrigger[SteamVR_Input_Sources.Keyboard].onStateDown += OnStateDownTrigger;
        if (booleanTrackpad != null && booleanTrackpad[SteamVR_Input_Sources.Keyboard] != null)
            booleanTrackpad[SteamVR_Input_Sources.Keyboard].onStateDown += OnStateDownTrackpad;
        if (booleanMenu != null && booleanMenu[SteamVR_Input_Sources.Keyboard] != null)
            booleanMenu[SteamVR_Input_Sources.Keyboard].onStateDown += OnStateDownMenu;
        
        // 检查 ViveTrackerPoseLogger 组件
        if (trackerPoseLogger == null)
        {
            trackerPoseLogger = FindObjectOfType<ViveTrackerPoseLogger>();
            if (trackerPoseLogger == null)
            {
                Debug.LogWarning("[KeyboardTrackerButton] 未找到 ViveTrackerPoseLogger 组件，实时数据采集功能将不可用");
            }
        }
        
        Debug.Log("<color=green>[KeyboardTrackerButton] 已注册 Keyboard Tracker 按钮事件</color>");
    }

    private void OnDestroy()
    {
        // 注销事件，防止对象销毁后回调空引用
        // 增加了空引用检查和异常保护，防止退出 Play 模式时 SteamVR 系统已销毁导致的崩溃
        try
        {
            if (booleanGrip != null && booleanGrip[SteamVR_Input_Sources.Keyboard] != null)
                booleanGrip[SteamVR_Input_Sources.Keyboard].onStateDown -= OnStateDownGrip;
            
            if (booleanPower != null && booleanPower[SteamVR_Input_Sources.Keyboard] != null)
                booleanPower[SteamVR_Input_Sources.Keyboard].onStateDown -= OnStateDownPower;
            
            if (booleanTrigger != null && booleanTrigger[SteamVR_Input_Sources.Keyboard] != null)
                booleanTrigger[SteamVR_Input_Sources.Keyboard].onStateDown -= OnStateDownTrigger;
            
            if (booleanTrackpad != null && booleanTrackpad[SteamVR_Input_Sources.Keyboard] != null)
                booleanTrackpad[SteamVR_Input_Sources.Keyboard].onStateDown -= OnStateDownTrackpad;
            
            if (booleanMenu != null && booleanMenu[SteamVR_Input_Sources.Keyboard] != null)
                booleanMenu[SteamVR_Input_Sources.Keyboard].onStateDown -= OnStateDownMenu;
        }
        catch (System.Exception ex)
        {
            // 捕获退出时可能出现的任何异常，防止崩溃
            Debug.LogWarning($"[KeyboardTrackerButton] 注销事件时发生异常（可忽略）: {ex.Message}");
        }
    }

    // --------------------------------------------------
    // 回调：Grip 按下时触发
    // --------------------------------------------------
    private void OnStateDownGrip(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        Debug.Log("<color=cyan>[KeyboardTracker] Grip 按下</color>");
    }

    // --------------------------------------------------
    // 回调：Power 按下时触发（用于触发探针标定数据采集）
    // --------------------------------------------------
    private void OnStateDownPower(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        // Power 按钮固定触发探针标定数据采集
        ExecutePowerAction();
    }

    /// <summary>
    /// 公共方法：供UI按钮调用，执行探针标定数据采集
    /// </summary>
    public void TriggerProbeCalibration()
    {
        ExecutePowerAction();
    }

    /// <summary>
    /// Power按钮的核心执行逻辑
    /// 用于触发探针标定数据采集
    /// </summary>
    private void ExecutePowerAction()
    {
        Debug.Log("<color=magenta>[KeyboardTracker] 按下 Power - 开始数据采集</color>");

        // ========== 构造输入数据 Point_Unity ==========
        // 使用单个 Tracker 进行多次测量
        Point_Unity pIn = new Point_Unity();
        pIn.MarkNum = 1;                  // 只使用 1 个 Tracker（替代原来的 Mark 点）
        pIn.PointNum = captureCount;      // 对该 Tracker 进行多次不同姿态的测量
        pIn.Points = new Pose_Unity[1024]; // 预分配 1024 个位置槽位以匹配本地结构体

        if (useRealTimeCapture && trackerPoseLogger != null)
        {
            // ========== 实时采集模式：从 ViveTrackerPoseLogger 捕获指定设备ID的 Tracker 位姿 ==========
            if (capturedPoses.Count < captureCount)
            {
                // 从 ViveTrackerPoseLogger 获取指定设备ID的 Tracker 位姿
                Vector3 trackerPositionMm;
                UnityEngine.Quaternion trackerRotation;
                
                if (!trackerPoseLogger.GetTrackerPoseForCalibration(keyboardTrackerDeviceId, out trackerPositionMm, out trackerRotation))
                {
                    Debug.LogError($"<color=red>[KeyboardTracker-探针标定] 无法获取 Tracker[ID:{keyboardTrackerDeviceId}] 数据，请检查设备连接</color>");
                    return;
                }
                
                // 捕获当前帧的 Tracker 位姿
                Pose_Unity currentPose = new Pose_Unity();
                
                // 捕获位置（转换为米：ViveTrackerPoseLogger 返回毫米，DLL 期望米）
                currentPose.Position = new Vector3D_Unity { 
                    x = trackerPositionMm.x / 1000.0, 
                    y = trackerPositionMm.y / 1000.0, 
                    z = trackerPositionMm.z / 1000.0 
                };
                
                // 捕获旋转（四元数）
                currentPose.Quaternion = new Quaternion_Unity { 
                    w = trackerRotation.w, 
                    x = trackerRotation.x, 
                    y = trackerRotation.y, 
                    z = trackerRotation.z 
                };
                
                capturedPoses.Add(currentPose);
                
                Debug.Log($"<color=cyan>[KeyboardTracker-探针标定] 已采集第 {capturedPoses.Count}/{captureCount} 个位姿 (设备ID: {keyboardTrackerDeviceId}) | " +
                          $"位置(m): ({trackerPositionMm.x / 1000.0:F4}, {trackerPositionMm.y / 1000.0:F4}, {trackerPositionMm.z / 1000.0:F4}) | " +
                          $"旋转(quat): ({trackerRotation.x:F4}, {trackerRotation.y:F4}, {trackerRotation.z:F4}, {trackerRotation.w:F4})</color>");
                
                // 如果尚未采集完成，提示用户继续
                if (capturedPoses.Count < captureCount)
                {
                    Debug.Log($"<color=yellow>[KeyboardTracker-探针标定] 请改变 Tracker 姿态，然后再次按下 Power 继续采集 ({capturedPoses.Count}/{captureCount})</color>");
                    return; // 尚未采集完成，不执行标定计算
                }
                else
                {
                    Debug.Log($"<color=green>[KeyboardTracker-探针标定] 数据采集完成 ({captureCount}/{captureCount})，开始计算探针尖端位置...</color>");
                }
            }
            
            // 将采集的数据填充到 pIn
            for (int i = 0; i < capturedPoses.Count && i < 1024; i++)
            {
                pIn.Points[i] = capturedPoses[i];
            }
            
            // 计算完成后清空缓存，准备下一次标定
            capturedPoses.Clear();
        }
        else
        {
            // ========== 预设数据模式：使用示例数据（用于测试） ==========
            Debug.Log("<color=orange>[KeyboardTracker-探针标定] 使用预设示例数据进行标定</color>");
            
            pIn.PointNum = 10; // 使用 10 个预设数据点
            
            // 填充 Tracker 的 10 次测量数据（位置 + 旋转四元数）
            // 每次测量都包含真实的位置和旋转状态
            
            // 第 1 次测量
            pIn.Points[0].Position = new Vector3D_Unity { x = -0.0449617, y = 0.0759295, z = 1.1244 };
            pIn.Points[0].Quaternion = new Quaternion_Unity { w = 0.9239, x = 0.0872, y = 0.3746, z = 0.0045 };
            
            // 第 2 次测量
            pIn.Points[1].Position = new Vector3D_Unity { x = -0.0770059, y = 0.116078, z = 1.09885 };
            pIn.Points[1].Quaternion = new Quaternion_Unity { w = 0.8910, x = 0.1234, y = 0.4012, z = -0.0123 };
            
            // 第 3 次测量
            pIn.Points[2].Position = new Vector3D_Unity { x = 0.00946268, y = 0.112199, z = 1.1598 };
            pIn.Points[2].Quaternion = new Quaternion_Unity { w = 0.9456, x = -0.0567, y = 0.3201, z = 0.0234 };
            
            // 第 4 次测量
            pIn.Points[3].Position = new Vector3D_Unity { x = -0.00235814, y = 0.088978, z = 1.09221 };
            pIn.Points[3].Quaternion = new Quaternion_Unity { w = 0.9123, x = 0.0987, y = 0.3890, z = -0.0345 };
            
            // 第 5 次测量
            pIn.Points[4].Position = new Vector3D_Unity { x = -0.0450156, y = 0.126164, z = 1.0738 };
            pIn.Points[4].Quaternion = new Quaternion_Unity { w = 0.8765, x = 0.1456, y = 0.4234, z = 0.0156 };
            
            // 第 6 次测量
            pIn.Points[5].Position = new Vector3D_Unity { x = 0.0493589, y = 0.127036, z = 1.13147 };
            pIn.Points[5].Quaternion = new Quaternion_Unity { w = 0.9345, x = -0.0789, y = 0.3567, z = -0.0267 };
            
            // 第 7 次测量
            pIn.Points[6].Position = new Vector3D_Unity { x = 0.0550588, y = 0.0870724, z = 1.09486 };
            pIn.Points[6].Quaternion = new Quaternion_Unity { w = 0.9567, x = 0.0234, y = 0.2890, z = 0.0178 };
            
            // 第 8 次测量
            pIn.Points[7].Position = new Vector3D_Unity { x = 0.00139575, y = 0.110565, z = 1.08852 };
            pIn.Points[7].Quaternion = new Quaternion_Unity { w = 0.9012, x = 0.1123, y = 0.4123, z = -0.0089 };
            
            // 第 9 次测量
            pIn.Points[8].Position = new Vector3D_Unity { x = 0.0985311, y = 0.14043, z = 1.12368 };
            pIn.Points[8].Quaternion = new Quaternion_Unity { w = 0.8890, x = 0.1567, y = 0.4289, z = 0.0234 };
            
            // 第 10 次测量
            pIn.Points[9].Position = new Vector3D_Unity { x = 0.0453421, y = 0.0702325, z = 1.17881 };
            pIn.Points[9].Quaternion = new Quaternion_Unity { w = 0.9234, x = -0.0456, y = 0.3678, z = -0.0145 };
        }

        // ========== 将托管结构体封送到非托管内存（IntPtr） ==========
        // 1) 计算输入结构体大小并分配非托管内存
        int sizeIn = Marshal.SizeOf(typeof(Point_Unity));
        IntPtr pBuffIn = Marshal.AllocHGlobal(sizeIn);
        // 2) 将托管结构体复制到非托管内存中
        Marshal.StructureToPtr(pIn, pBuffIn, true);

        // 为输出分配非托管内存（例如 Vector3D_Unity）
        int sizeOut = Marshal.SizeOf(typeof(Vector3D_Unity));
        IntPtr pBuffOut = Marshal.AllocHGlobal(sizeOut);

        // ========== 调用本地 DLL 函数（计算针尖位置示例） ==========
        int result2 = calculateNeedleTip(pBuffIn, pBuffOut);

        // 将输出从非托管内存转换回托管结构体
        Vector3D_Unity pOut = (Vector3D_Unity)Marshal.PtrToStructure(pBuffOut, typeof(Vector3D_Unity)); // 获取输出

        // 输出 DLL 调用结果（功能日志）
        Debug.Log($"<color=lime>[KeyboardTracker-DLL 调用结果] calculateNeedleTip 返回码: {result2} | 输出坐标: ({pOut.x:F4}, {pOut.y:F4}, {pOut.z:F4})</color>");

        // ===== 注意 =====
        // 当前示例中为演示方便没有释放非托管内存：
        // Marshal.FreeHGlobal(pBuffIn);
        // Marshal.FreeHGlobal(pBuffOut);
        // 在生产代码中务必释放分配的非托管内存以避免内存泄漏。
    }

    // --------------------------------------------------
    // 其他按键事件回调（示例：触发相应功能）
    // --------------------------------------------------
    private void OnStateDownTrigger(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        Debug.Log("<color=cyan>[KeyboardTracker] Trigger 按下</color>");
    }

    private void OnStateDownTrackpad(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        Debug.Log("<color=cyan>[KeyboardTracker] Trackpad 按下</color>");
    }

    private void OnStateDownMenu(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        Debug.Log("<color=cyan>[KeyboardTracker] Menu 按下</color>");
    }

    // Update is called once per frame
    void Update()
    {

    }
}

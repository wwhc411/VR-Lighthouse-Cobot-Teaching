using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR;
using System.Runtime.InteropServices; // 与 C++ DLL 交互时需要的命名空间（封送/非托管内存）

/// <summary>
/// TriggerTester: 用于响应 SteamVR 按钮事件并演示与本地 C++ DLL 的交互。
/// 主要功能：
/// - 注册 SteamVR 的布尔输入（Grip/Trigger/Trackpad/Menu/Power）事件
/// - 在 Grip 按下时构造数据结构并通过封送调用本地 DLL 函数（calculateNeedleTip）
/// - 演示如何分配非托管内存、封送结构体、调用 DLL 并读取返回值
/// 注意：本脚本用于演示/测试，真实项目中应增加错误处理和内存释放（Marshal.FreeHGlobal）
/// </summary>
public class TriggerTester : MonoBehaviour
{
    // 从 SteamVR Input 系统定义的布尔动作（在 Inspector 中绑定）
    public SteamVR_Action_Boolean booleanGrip;
    public SteamVR_Action_Boolean booleanPower;
    public SteamVR_Action_Boolean booleanTrigger;
    public SteamVR_Action_Boolean booleanTrackpad;
    public SteamVR_Action_Boolean booleanMenu;

    [Header("手眼标定设置")]
    [Tooltip("手眼标定 UI 管理器（拖入场景中的 HandEyeCalibrationUI 组件）")]
    public HandEyeCalibration.HandEyeCalibrationUI handEyeCalibrationUI;

    [Tooltip("Tracker数据采集模式")]
    public TrackerDataMode trackerDataMode = TrackerDataMode.AbsolutePose;

    [Header("绝对位姿模式设置")]
    [Tooltip("相机位置 Tracker 的设备序列号（用于数据采集）")]
    public uint cameraTrackerDeviceId = 1;

    [Header("相对位姿模式设置")]
    [Tooltip("相对位姿捕获器（用于采集相对位姿数据）")]
    public RelativePoseCapturer relativePoseCapturer;

    /// <summary>
    /// Tracker数据采集模式枚举
    /// </summary>
    public enum TrackerDataMode
    {
        AbsolutePose,   // 绝对位姿模式：直接捕获指定设备的世界坐标位姿
        RelativePose    // 相对位姿模式：捕获相对于参考设备的相对位姿
    }

    // 防重复触发：上次触发时间和最小间隔
    private float lastPowerPressTime = -1f;
    private const float DEBOUNCE_INTERVAL = 0.3f; // 防抖动间隔（秒）
    
    // 防重复注册标记
    private bool eventsRegistered = false;

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
        RegisterEvents();
    }

    /// <summary>
    /// 注册 SteamVR 事件（带防重复注册检查）
    /// </summary>
    private void RegisterEvents()
    {
        // 防止重复注册
        if (eventsRegistered)
        {
            Debug.LogWarning($"<color=yellow>[TriggerTester] 事件已注册，跳过重复注册 (InstanceID: {GetInstanceID()})</color>");
            return;
        }

        // 将各个 boolean 动作的按下事件绑定到回调函数
        // 这里使用 SteamVR_Input_Sources.Camera 作为输入源（可根据需要改为左手/右手）
        // 【修复】添加空检查，防止 SteamVR 未初始化时崩溃
        if (booleanGrip != null && booleanGrip[SteamVR_Input_Sources.Camera] != null)
            booleanGrip[SteamVR_Input_Sources.Camera].onStateDown += OnStateDownGrip;
        if (booleanPower != null && booleanPower[SteamVR_Input_Sources.Camera] != null)
            booleanPower[SteamVR_Input_Sources.Camera].onStateDown += OnStateDownPower;
        if (booleanTrigger != null && booleanTrigger[SteamVR_Input_Sources.Camera] != null)
            booleanTrigger[SteamVR_Input_Sources.Camera].onStateDown += OnStateDownTrigger;
        if (booleanTrackpad != null && booleanTrackpad[SteamVR_Input_Sources.Camera] != null)
            booleanTrackpad[SteamVR_Input_Sources.Camera].onStateDown += OnStateDownTrackpad;
        if (booleanMenu != null && booleanMenu[SteamVR_Input_Sources.Camera] != null)
            booleanMenu[SteamVR_Input_Sources.Camera].onStateDown += OnStateDownMenu;
        
        eventsRegistered = true;
        Debug.Log($"<color=green>[TriggerTester] 事件注册完成 (InstanceID: {GetInstanceID()})</color>");
    }

    private void OnDestroy()
    {
        // 注销事件，防止对象销毁后回调空引用
        // 增加了空引用检查和异常保护，防止退出 Play 模式时 SteamVR 系统已销毁导致的崩溃
        try
        {
            if (booleanGrip != null && booleanGrip[SteamVR_Input_Sources.Camera] != null)
                booleanGrip[SteamVR_Input_Sources.Camera].onStateDown -= OnStateDownGrip;
            
            if (booleanPower != null && booleanPower[SteamVR_Input_Sources.Camera] != null)
                booleanPower[SteamVR_Input_Sources.Camera].onStateDown -= OnStateDownPower;
            
            if (booleanTrigger != null && booleanTrigger[SteamVR_Input_Sources.Camera] != null)
                booleanTrigger[SteamVR_Input_Sources.Camera].onStateDown -= OnStateDownTrigger;
            
            if (booleanTrackpad != null && booleanTrackpad[SteamVR_Input_Sources.Camera] != null)
                booleanTrackpad[SteamVR_Input_Sources.Camera].onStateDown -= OnStateDownTrackpad;
            
            if (booleanMenu != null && booleanMenu[SteamVR_Input_Sources.Camera] != null)
                booleanMenu[SteamVR_Input_Sources.Camera].onStateDown -= OnStateDownMenu;
            
            eventsRegistered = false;
        }
        catch (System.Exception ex)
        {
            // 捕获退出时可能出现的任何异常，防止崩溃
            Debug.LogWarning($"[TriggerTester] 注销事件时发生异常（可忽略）: {ex.Message}");
        }
    }

    // --------------------------------------------------
    // 回调：Grip 按下时触发
    // --------------------------------------------------
    private void OnStateDownGrip(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        print("Grip");
    }

    // --------------------------------------------------
    // 回调：Power 按下时触发（用于触发手眼标定数据采集）
    // --------------------------------------------------
    private void OnStateDownPower(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        // 防抖动检查：忽略短时间内的重复触发
        float currentTime = Time.time;
        if (currentTime - lastPowerPressTime < DEBOUNCE_INTERVAL)
        {
            Debug.LogWarning($"<color=yellow>[TriggerTester] 检测到重复触发，已忽略 (间隔: {(currentTime - lastPowerPressTime) * 1000:F0}ms)</color>");
            return;
        }
        lastPowerPressTime = currentTime;

        if (handEyeCalibrationUI == null)
        {
            Debug.LogError("[TriggerTester] HandEyeCalibrationUI 未设置，无法采集手眼标定数据！");
            return;
        }

        // 根据模式选择不同的数据采集方式
        switch (trackerDataMode)
        {
            case TrackerDataMode.AbsolutePose:
                // 绝对位姿模式：捕获指定设备的世界坐标位姿
                CaptureAbsolutePoseData();
                break;

            case TrackerDataMode.RelativePose:
                // 相对位姿模式：捕获相对于参考设备的相对位姿
                CaptureRelativePoseData();
                break;

            default:
                Debug.LogError($"[TriggerTester] 未知的Tracker数据模式: {trackerDataMode}");
                break;
        }
    }

    /// <summary>
    /// 捕获绝对位姿数据（传统模式）
    /// </summary>
    private void CaptureAbsolutePoseData()
    {
        handEyeCalibrationUI.CaptureCalibrationData(cameraTrackerDeviceId);
        Debug.Log($"<color=cyan>[TriggerTester] 绝对位姿模式 - Power按下 - 已触发手眼标定数据采集 (设备ID: {cameraTrackerDeviceId})</color>");
    }

    /// <summary>
    /// 捕获相对位姿数据（相对位姿模式）
    /// 使用独立的 RelativePoseCapturer 组件处理
    /// </summary>
    private void CaptureRelativePoseData()
    {
        if (relativePoseCapturer == null)
        {
            Debug.LogError("[TriggerTester] 相对位姿模式下未设置 RelativePoseCapturer，请在 Inspector 中拖入 RelativePoseCapturer 组件！");
            return;
        }

        // 调用相对位姿捕获器进行数据采集
        if (relativePoseCapturer.CaptureRelativePose())
        {
            Debug.Log($"<color=green>[TriggerTester] 相对位姿模式 - Power按下 - 已触发手眼标定数据采集</color>");
        }
    }

    // --------------------------------------------------
    // 其他按键事件回调（示例：触发相应功能）
    // --------------------------------------------------
    private void OnStateDownTrigger(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        print("Trigger");
    }

    private void OnStateDownTrackpad(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        print("Trackpad");
    }

    private void OnStateDownMenu(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        print("Menu");
    }

    // Update is called once per frame
    void Update()
    {

    }
}

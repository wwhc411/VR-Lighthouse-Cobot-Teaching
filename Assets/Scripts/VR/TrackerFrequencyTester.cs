using System.Collections.Generic;
using UnityEngine;
using Valve.VR;

/// <summary>
/// VIVE Tracker 位姿更新频率测试工具
/// 
/// 功能：
/// - 实时监测 Tracker 位姿的真实更新频率
/// - 统计帧间时间差和更新率
/// - 检测重复帧和丢帧情况
/// - 支持多种测试模式：Update、FixedUpdate、SteamVR_Events.NewPoses
/// 
/// 使用方法：
/// 1. 将此脚本挂载到场景中的任意 GameObject
/// 2. 在 Inspector 中设置 Tracker 设备 ID
/// 3. 选择测试模式
/// 4. 进入 Play 模式，观察 Console 输出的统计报告
/// </summary>
public class TrackerFrequencyTester : MonoBehaviour
{
    [Header("测试配置")]
    [Tooltip("要测试的 Tracker 设备 ID")]
    public uint trackerDeviceId = 2;

    [Tooltip("测试模式")]
    public TestMode testMode = TestMode.Update;

    [Tooltip("统计报告间隔（秒）")]
    public float reportInterval = 5f;

    [Tooltip("是否检测位姿变化（用于识别重复帧）")]
    public bool detectPoseChanges = true;

    [Tooltip("位置变化阈值（mm），低于此值视为重复帧")]
    public float positionChangeThreshold = 0.01f;

    [Tooltip("旋转变化阈值（度），低于此值视为重复帧")]
    public float rotationChangeThreshold = 0.01f;

    [Header("测试控制")]
    [Tooltip("是否启用测试")]
    public bool enableTesting = true;

    [Tooltip("是否输出详细日志")]
    public bool verboseLogging = false;

    public enum TestMode
    {
        Update,              // 每个 Unity Update 采样
        FixedUpdate,        // 每个 FixedUpdate 采样
        NewPosesEvent,      // 订阅 SteamVR_Events.NewPoses
        HighFrequencyCoroutine  // 协程高频采样（目标 125Hz）
    }

    // 统计数据
    private int totalSamples = 0;
    private int duplicateFrames = 0;
    private float nextReportTime = 0f;
    private List<float> frameTimes = new List<float>();
    private Vector3 lastPosition = Vector3.zero;
    private Quaternion lastRotation = Quaternion.identity;
    private bool hasLastPose = false;

    // SteamVR 相关
    private CVRSystem vrSystem;
    private TrackedDevicePose_t[] trackedDevicePoses = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];

    // 事件相关
    private SteamVR_Events.Action newPosesAction;

    // 高频协程相关
    private bool isRunningCoroutine = false;

    // 记录当前激活的测试模式
    private TestMode activeMode = TestMode.Update;

    void Start()
    {
        // 初始化 SteamVR
        if (OpenVR.System == null)
        {
            Debug.LogWarning("[TrackerFreqTest] SteamVR 未初始化，尝试初始化...");
            EVRInitError vrError = EVRInitError.None;
            vrSystem = OpenVR.Init(ref vrError, EVRApplicationType.VRApplication_Scene);
            
            if (vrError != EVRInitError.None)
            {
                Debug.LogError($"<color=red>[TrackerFreqTest] SteamVR 初始化失败: {vrError}</color>");
                Debug.LogError("  <color=yellow>可能原因:</color>");
                Debug.LogError("    1. SteamVR 未运行");
                Debug.LogError("    2. VR 设备未连接");
                Debug.LogError("    3. OpenVR 驱动问题");
                enabled = false;
                return;
            }
            else
            {
                Debug.Log("<color=green>[TrackerFreqTest] SteamVR 初始化成功</color>");
            }
        }
        else
        {
            vrSystem = OpenVR.System;
            Debug.Log("<color=green>[TrackerFreqTest] 使用现有 SteamVR 实例</color>");
        }

        // 验证设备
        Debug.Log($"<color=cyan>[TrackerFreqTest] 验证设备 ID: {trackerDeviceId}</color>");
        
        if (!vrSystem.IsTrackedDeviceConnected(trackerDeviceId))
        {
            Debug.LogError($"<color=red>[TrackerFreqTest] Tracker[ID:{trackerDeviceId}] 未连接！</color>");
            Debug.LogError("  <color=yellow>请检查:</color>");
            Debug.LogError("    1. Tracker 是否开机");
            Debug.LogError("    2. 设备 ID 是否正确（常见：2 或 3）");
            Debug.LogError("    3. SteamVR 是否识别到设备");
            
            // 列出所有已连接的设备
            ListAllConnectedDevices();
            
            enabled = false;
            return;
        }
        else
        {
            Debug.Log($"<color=green>[TrackerFreqTest] ✓ 设备[ID:{trackerDeviceId}] 已连接</color>");
        }

        var deviceClass = vrSystem.GetTrackedDeviceClass(trackerDeviceId);
        if (deviceClass != ETrackedDeviceClass.GenericTracker)
        {
            Debug.LogWarning($"<color=yellow>[TrackerFreqTest] ⚠ 设备[ID:{trackerDeviceId}] 类型为 {deviceClass}，不是 Tracker</color>");
            Debug.LogWarning("  提示: 仍会尝试测试，但可能不是预期设备");
        }
        else
        {
            Debug.Log($"<color=green>[TrackerFreqTest] ✓ 设备类型确认: Generic Tracker</color>");
        }

        nextReportTime = Time.time + reportInterval;

        Debug.Log("========== Tracker 频率测试开始 ==========");
        Debug.Log($"<color=cyan>测试配置:</color>");
        Debug.Log($"  Tracker ID: {trackerDeviceId}");
        Debug.Log($"  测试模式: {testMode}");
        Debug.Log($"  报告间隔: {reportInterval}s");
        Debug.Log($"  位置变化阈值: {positionChangeThreshold} mm");
        Debug.Log($"  旋转变化阈值: {rotationChangeThreshold}°");
        Debug.Log($"  Unity 目标帧率: {Application.targetFrameRate} (0=VSync)");
        Debug.Log($"  FixedUpdate 频率: {1f / Time.fixedDeltaTime:F1} Hz");
        Debug.Log("==========================================\n");

        // 初始化测试模式
        activeMode = testMode;
        InitializeTestMode(testMode);
    }

    /// <summary>
    /// 初始化指定的测试模式
    /// </summary>
    void InitializeTestMode(TestMode mode)
    {
        Debug.Log($"<color=cyan>[TrackerFreqTest] 开始初始化测试模式: {mode}</color>");
        
        // 先清理所有模式
        CleanupAllModes();

        // ⚠️ 关键：在启动协程之前就设置 activeMode，确保协程的 while 条件能通过
        activeMode = mode;
        Debug.Log($"<color=cyan>[TrackerFreqTest] activeMode 已设置为: {activeMode}</color>");

        // 根据模式初始化
        if (mode == TestMode.NewPosesEvent)
        {
            newPosesAction = SteamVR_Events.NewPosesAction(OnNewPoses);
            newPosesAction.enabled = true;
            Debug.Log("<color=green>[TrackerFreqTest] 已订阅 SteamVR_Events.NewPoses</color>");
        }
        else if (mode == TestMode.HighFrequencyCoroutine)
        {
            Debug.Log("<color=cyan>[TrackerFreqTest] 准备启动高频协程...</color>");
            Debug.Log($"  - vrSystem 状态: {(vrSystem != null ? "已初始化" : "NULL")}");
            Debug.Log($"  - enableTesting: {enableTesting}");
            Debug.Log($"  - isRunningCoroutine: {isRunningCoroutine}");
            Debug.Log($"  - activeMode: {activeMode}");
            
            StartCoroutine(HighFrequencySamplingCoroutine());
            
            Debug.Log("<color=green>[TrackerFreqTest] StartCoroutine 已调用</color>");
        }
        else if (mode == TestMode.Update)
        {
            Debug.Log("<color=green>[TrackerFreqTest] 使用 Update 模式采样</color>");
        }
        else if (mode == TestMode.FixedUpdate)
        {
            Debug.Log("<color=green>[TrackerFreqTest] 使用 FixedUpdate 模式采样</color>");
        }

        Debug.Log($"<color=green>[TrackerFreqTest] 测试模式初始化完成: {activeMode}</color>");
    }

    /// <summary>
    /// 清理所有测试模式的订阅和协程
    /// </summary>
    void CleanupAllModes()
    {
        // 清理事件订阅
        if (newPosesAction != null)
        {
            newPosesAction.enabled = false;
            newPosesAction = null;
        }

        // 停止协程
        if (isRunningCoroutine)
        {
            isRunningCoroutine = false;
            StopAllCoroutines();
        }
    }

    /// <summary>
    /// Inspector 中切换测试模式时的回调
    /// </summary>
    void OnValidate()
    {
        // 仅在运行时切换模式
        if (Application.isPlaying && activeMode != testMode)
        {
            Debug.Log($"<color=yellow>[TrackerFreqTest] 测试模式切换: {activeMode} → {testMode}</color>");
            
            // 重置统计数据
            ResetStatistics();
            
            // 切换模式
            InitializeTestMode(testMode);
        }
    }

    void Update()
    {
        if (!enableTesting) return;

        // 只在激活的模式下采样（防止多个模式同时运行）
        if (activeMode == TestMode.Update && testMode == TestMode.Update)
        {
            SampleTrackerPose();
        }

        // 定期输出统计报告
        if (Time.time >= nextReportTime)
        {
            PrintStatisticsReport();
            nextReportTime = Time.time + reportInterval;
        }
    }

    void FixedUpdate()
    {
        if (!enableTesting) return;

        // 只在激活的模式下采样（防止多个模式同时运行）
        if (activeMode == TestMode.FixedUpdate && testMode == TestMode.FixedUpdate)
        {
            SampleTrackerPose();
        }
    }

    /// <summary>
    /// 采样 Tracker 位姿
    /// </summary>
    void SampleTrackerPose()
    {
        if (vrSystem == null)
        {
            if (verboseLogging)
                Debug.LogWarning("[TrackerFreqTest] vrSystem 为 null，跳过采样");
            return;
        }

        vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, 0f, trackedDevicePoses);

        var pose = trackedDevicePoses[trackerDeviceId];
        if (!pose.bPoseIsValid || !pose.bDeviceIsConnected)
        {
            if (verboseLogging)
                Debug.LogWarning($"[TrackerFreqTest] Pose 无效: bPoseIsValid={pose.bPoseIsValid}, bDeviceIsConnected={pose.bDeviceIsConnected}");
            return;
        }

        var matrix = pose.mDeviceToAbsoluteTracking;
        Vector3 position = new Vector3(matrix.m3 * 1000f, matrix.m7 * 1000f, matrix.m11 * 1000f);
        Quaternion rotation = QuaternionFromMatrix(matrix);

        RecordSample(position, rotation);
        
        if (verboseLogging && totalSamples % 30 == 0)
        {
            Debug.Log($"<color=cyan>[TrackerFreqTest] 采样进度: {totalSamples} 个样本</color>");
        }
    }

    /// <summary>
    /// SteamVR NewPoses 事件回调
    /// </summary>
    void OnNewPoses(TrackedDevicePose_t[] poses)
    {
        // 只在激活的模式下处理（防止模式切换后仍接收事件）
        if (!enableTesting || activeMode != TestMode.NewPosesEvent || testMode != TestMode.NewPosesEvent) 
            return;

        if (trackerDeviceId >= poses.Length) return;

        var pose = poses[trackerDeviceId];
        if (!pose.bPoseIsValid || !pose.bDeviceIsConnected)
        {
            if (verboseLogging)
                Debug.LogWarning($"[TrackerFreqTest] Pose 无效或设备断开");
            return;
        }

        var matrix = pose.mDeviceToAbsoluteTracking;
        Vector3 position = new Vector3(matrix.m3 * 1000f, matrix.m7 * 1000f, matrix.m11 * 1000f);
        Quaternion rotation = QuaternionFromMatrix(matrix);

        RecordSample(position, rotation);
    }

    /// <summary>
    /// 高频采样协程（目标 125Hz）
    /// </summary>
    System.Collections.IEnumerator HighFrequencySamplingCoroutine()
    {
        Debug.Log("<color=yellow>[TrackerFreqTest] ========== 协程已启动 ==========</color>");
        
        // 检查初始条件
        if (vrSystem == null)
        {
            Debug.LogError("[TrackerFreqTest] 协程启动失败: vrSystem 为 NULL");
            yield break;
        }
        
        isRunningCoroutine = true;
        float targetInterval = 1f / 125f; // 0.008s

        Debug.Log($"<color=cyan>[TrackerFreqTest] 启动高频采样协程，目标频率: 125 Hz ({targetInterval * 1000f:F2} ms)</color>");
        Debug.Log($"  - vrSystem: OK");
        Debug.Log($"  - enableTesting: {enableTesting}");
        Debug.Log($"  - isRunningCoroutine: {isRunningCoroutine}");
        Debug.Log($"  - activeMode: {activeMode}");
        Debug.Log($"  - testMode: {testMode}");

        int loopCount = 0;
        
        // 只在激活的模式下继续运行（防止模式切换后协程仍在运行）
        while (enableTesting && isRunningCoroutine && 
               activeMode == TestMode.HighFrequencyCoroutine && 
               testMode == TestMode.HighFrequencyCoroutine)
        {
            loopCount++;
            
            // 前3次循环输出详细信息
            if (loopCount <= 3)
            {
                Debug.Log($"<color=cyan>[TrackerFreqTest] 循环 #{loopCount} - 调用 SampleTrackerPose</color>");
            }
            
            SampleTrackerPose();
            
            // 每125次循环（约1秒）报告一次
            if (loopCount % 125 == 0)
            {
                Debug.Log($"[TrackerFreqTest] 高频协程运行中 - 循环次数: {loopCount}, 已采样: {totalSamples}");
            }
            
            yield return new WaitForSeconds(targetInterval);
        }

        isRunningCoroutine = false;
        
        Debug.LogWarning($"<color=orange>[TrackerFreqTest] ========== 协程已退出 ==========</color>");
        Debug.LogWarning($"  - enableTesting: {enableTesting}");
        Debug.LogWarning($"  - isRunningCoroutine: {isRunningCoroutine}");
        Debug.LogWarning($"  - activeMode: {activeMode}");
        Debug.LogWarning($"  - testMode: {testMode}");
        Debug.LogWarning($"  - 总循环次数: {loopCount}");
    }

    /// <summary>
    /// 记录采样数据
    /// </summary>
    void RecordSample(Vector3 position, Quaternion rotation)
    {
        totalSamples++;
        frameTimes.Add(Time.time);

        // 检测位姿变化（识别重复帧）
        if (detectPoseChanges && hasLastPose)
        {
            float positionDelta = Vector3.Distance(position, lastPosition);
            float rotationDelta = Quaternion.Angle(rotation, lastRotation);

            if (positionDelta < positionChangeThreshold && rotationDelta < rotationChangeThreshold)
            {
                duplicateFrames++;
                if (verboseLogging)
                {
                    Debug.Log($"<color=yellow>[重复帧] ΔPos={positionDelta:F4}mm, ΔRot={rotationDelta:F4}°</color>");
                }
            }
            else if (verboseLogging)
            {
                Debug.Log($"[新帧] ΔPos={positionDelta:F4}mm, ΔRot={rotationDelta:F4}°");
            }
        }

        lastPosition = position;
        lastRotation = rotation;
        hasLastPose = true;
    }

    /// <summary>
    /// 输出统计报告
    /// </summary>
    void PrintStatisticsReport()
    {
        if (totalSamples < 2)
        {
            Debug.LogWarning($"<color=yellow>[TrackerFreqTest] 样本数不足（当前: {totalSamples}），无法生成报告</color>");
            Debug.LogWarning("  <color=cyan>可能原因:</color>");
            Debug.LogWarning("    1. Tracker 位姿数据无效（bPoseIsValid=false）");
            Debug.LogWarning("    2. 测试时间过短（建议等待至少 5 秒）");
            Debug.LogWarning("    3. 测试模式未正确启动");
            Debug.LogWarning($"  <color=cyan>当前配置:</color>");
            Debug.LogWarning($"    - 测试模式: {testMode}");
            Debug.LogWarning($"    - 激活模式: {activeMode}");
            Debug.LogWarning($"    - 启用测试: {enableTesting}");
            Debug.LogWarning($"    - 详细日志: {verboseLogging}");
            Debug.LogWarning("  <color=lime>建议: 启用 Verbose Logging 查看详细采样信息</color>");
            return;
        }

        // 计算统计数据
        float elapsedTime = frameTimes[frameTimes.Count - 1] - frameTimes[0];
        float averageFrequency = (totalSamples - 1) / elapsedTime;

        // 计算帧间时间差
        List<float> deltaTimesMs = new List<float>();
        for (int i = 1; i < frameTimes.Count; i++)
        {
            float deltaMs = (frameTimes[i] - frameTimes[i - 1]) * 1000f;
            deltaTimesMs.Add(deltaMs);
        }

        float minDeltaMs = float.MaxValue;
        float maxDeltaMs = float.MinValue;
        float sumDeltaMs = 0f;

        foreach (float dt in deltaTimesMs)
        {
            if (dt < minDeltaMs) minDeltaMs = dt;
            if (dt > maxDeltaMs) maxDeltaMs = dt;
            sumDeltaMs += dt;
        }

        float avgDeltaMs = sumDeltaMs / deltaTimesMs.Count;

        // 计算标准差
        float varianceSum = 0f;
        foreach (float dt in deltaTimesMs)
        {
            float diff = dt - avgDeltaMs;
            varianceSum += diff * diff;
        }
        float stdDevMs = Mathf.Sqrt(varianceSum / deltaTimesMs.Count);

        // 输出报告
        Debug.Log("\n========== Tracker 频率测试报告 ==========");
        Debug.Log($"<color=cyan>【测试模式】</color> {testMode}");
        Debug.Log($"<color=cyan>【测试时长】</color> {elapsedTime:F2} 秒");
        Debug.Log($"<color=cyan>【总采样数】</color> {totalSamples}");
        Debug.Log($"<color=cyan>【平均频率】</color> <color=yellow>{averageFrequency:F2} Hz</color>");
        
        if (detectPoseChanges)
        {
            int uniqueFrames = totalSamples - duplicateFrames;
            float uniqueFrequency = (uniqueFrames - 1) / elapsedTime;
            float duplicateRatio = (duplicateFrames / (float)totalSamples) * 100f;
            
            Debug.Log($"<color=cyan>【重复帧数】</color> {duplicateFrames} ({duplicateRatio:F1}%)");
            Debug.Log($"<color=cyan>【唯一帧数】</color> {uniqueFrames}");
            Debug.Log($"<color=cyan>【实际更新频率】</color> <color=lime>{uniqueFrequency:F2} Hz</color>");
        }

        Debug.Log($"\n<color=cyan>【帧间时间统计】</color>");
        Debug.Log($"  平均: {avgDeltaMs:F3} ms ({1000f / avgDeltaMs:F1} Hz)");
        Debug.Log($"  最小: {minDeltaMs:F3} ms ({1000f / minDeltaMs:F1} Hz)");
        Debug.Log($"  最大: {maxDeltaMs:F3} ms ({1000f / maxDeltaMs:F1} Hz)");
        Debug.Log($"  标准差: {stdDevMs:F3} ms");
        Debug.Log($"  稳定性: {(stdDevMs < 1f ? "<color=green>优秀</color>" : stdDevMs < 3f ? "<color=yellow>良好</color>" : "<color=red>较差</color>")}");

        Debug.Log($"\n<color=cyan>【频率分析】</color>");
        if (averageFrequency >= 120f)
            Debug.Log($"  → 检测到 <color=lime>120+ Hz</color> 模式 (高刷新率 VR 设备)");
        else if (averageFrequency >= 85f && averageFrequency <= 95f)
            Debug.Log($"  → 检测到 <color=lime>90 Hz</color> 模式 (标准 VR 设备)");
        else if (averageFrequency >= 55f && averageFrequency <= 65f)
            Debug.Log($"  → 检测到 <color=yellow>60 Hz</color> 模式 (可能受 VSync 限制)");
        else
            Debug.Log($"  → <color=yellow>非标准频率</color>，可能受性能或配置影响");

        Debug.Log("==========================================\n");
    }

    /// <summary>
    /// 手动触发统计报告
    /// </summary>
    [ContextMenu("立即输出报告")]
    public void GenerateReportNow()
    {
        PrintStatisticsReport();
    }

    /// <summary>
    /// 重置统计数据
    /// </summary>
    [ContextMenu("重置统计数据")]
    public void ResetStatistics()
    {
        totalSamples = 0;
        duplicateFrames = 0;
        frameTimes.Clear();
        hasLastPose = false;
        nextReportTime = Time.time + reportInterval;
        
        Debug.Log("<color=yellow>[TrackerFreqTest] 统计数据已重置</color>");
    }

    /// <summary>
    /// 切换测试模式（运行时调用）
    /// </summary>
    [ContextMenu("切换到 Update 模式")]
    public void SwitchToUpdateMode()
    {
        testMode = TestMode.Update;
        OnValidate();
    }

    [ContextMenu("切换到 FixedUpdate 模式")]
    public void SwitchToFixedUpdateMode()
    {
        testMode = TestMode.FixedUpdate;
        OnValidate();
    }

    [ContextMenu("切换到 NewPosesEvent 模式")]
    public void SwitchToNewPosesEventMode()
    {
        testMode = TestMode.NewPosesEvent;
        OnValidate();
    }

    [ContextMenu("切换到 125Hz 协程模式")]
    public void SwitchToHighFrequencyMode()
    {
        testMode = TestMode.HighFrequencyCoroutine;
        OnValidate();
    }

    /// <summary>
    /// 列出所有已连接的设备（诊断工具）
    /// </summary>
    [ContextMenu("列出所有已连接设备")]
    public void ListAllConnectedDevices()
    {
        if (vrSystem == null)
        {
            Debug.LogError("[TrackerFreqTest] vrSystem 未初始化，无法列出设备");
            return;
        }

        Debug.Log("========== 已连接的 VR 设备列表 ==========");
        
        int deviceCount = 0;
        for (uint i = 0; i < OpenVR.k_unMaxTrackedDeviceCount; i++)
        {
            if (vrSystem.IsTrackedDeviceConnected(i))
            {
                var deviceClass = vrSystem.GetTrackedDeviceClass(i);
                string deviceName = GetDeviceClassString(deviceClass);
                
                deviceCount++;
                Debug.Log($"  <color=cyan>设备 ID {i}</color>: {deviceName}");
                
                if (deviceClass == ETrackedDeviceClass.GenericTracker)
                {
                    Debug.Log($"    → <color=lime>这是 Tracker 设备！</color>");
                }
            }
        }
        
        if (deviceCount == 0)
        {
            Debug.LogWarning("  <color=yellow>未找到任何已连接的设备</color>");
            Debug.LogWarning("  请确保 SteamVR 正在运行且设备已配对");
        }
        else
        {
            Debug.Log($"  <color=green>共找到 {deviceCount} 个设备</color>");
        }
        
        Debug.Log("==========================================");
    }

    /// <summary>
    /// 获取设备类型的可读字符串
    /// </summary>
    string GetDeviceClassString(ETrackedDeviceClass deviceClass)
    {
        switch (deviceClass)
        {
            case ETrackedDeviceClass.HMD:
                return "头戴显示器 (HMD)";
            case ETrackedDeviceClass.Controller:
                return "控制器 (Controller)";
            case ETrackedDeviceClass.GenericTracker:
                return "通用追踪器 (Tracker) ★";
            case ETrackedDeviceClass.TrackingReference:
                return "基站 (Base Station)";
            case ETrackedDeviceClass.DisplayRedirect:
                return "显示重定向";
            default:
                return $"未知设备类型 ({deviceClass})";
        }
    }

    void OnDestroy()
    {
        // 清理所有模式
        CleanupAllModes();

        // 输出最终报告
        if (totalSamples > 1)
        {
            Debug.Log("\n<color=magenta>========== 最终测试报告 ==========</color>");
            PrintStatisticsReport();
        }
    }

    void OnDisable()
    {
        // 立即停止所有测试活动
        enableTesting = false;
        
        // 清理所有模式
        CleanupAllModes();
    }

    // 辅助方法：从 HmdMatrix34_t 提取四元数
    static Quaternion QuaternionFromMatrix(HmdMatrix34_t matrix)
    {
        float m00 = matrix.m0;
        float m01 = matrix.m1;
        float m02 = matrix.m2;
        float m10 = matrix.m4;
        float m11 = matrix.m5;
        float m12 = matrix.m6;
        float m20 = matrix.m8;
        float m21 = matrix.m9;
        float m22 = matrix.m10;

        float trace = m00 + m11 + m22;
        Quaternion q = new Quaternion();

        if (trace > 0f)
        {
            float s = Mathf.Sqrt(trace + 1f) * 2f;
            q.w = 0.25f * s;
            q.x = (m21 - m12) / s;
            q.y = (m02 - m20) / s;
            q.z = (m10 - m01) / s;
        }
        else if ((m00 > m11) && (m00 > m22))
        {
            float s = Mathf.Sqrt(1f + m00 - m11 - m22) * 2f;
            q.w = (m21 - m12) / s;
            q.x = 0.25f * s;
            q.y = (m01 + m10) / s;
            q.z = (m02 + m20) / s;
        }
        else if (m11 > m22)
        {
            float s = Mathf.Sqrt(1f + m11 - m00 - m22) * 2f;
            q.w = (m02 - m20) / s;
            q.x = (m01 + m10) / s;
            q.y = 0.25f * s;
            q.z = (m12 + m21) / s;
        }
        else
        {
            float s = Mathf.Sqrt(1f + m22 - m00 - m11) * 2f;
            q.w = (m10 - m01) / s;
            q.x = (m02 + m20) / s;
            q.y = (m12 + m21) / s;
            q.z = 0.25f * s;
        }

        return NormalizeQuaternion(q);
    }

    static Quaternion NormalizeQuaternion(Quaternion q)
    {
        float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (mag > Mathf.Epsilon)
        {
            float invMag = 1f / mag;
            q.x *= invMag;
            q.y *= invMag;
            q.z *= invMag;
            q.w *= invMag;
        }
        else
        {
            q = Quaternion.identity;
        }
        return q;
    }
}

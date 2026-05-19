using System.Globalization;
using System.Text;
using UnityEngine;
using Valve.VR;

/// <summary>
/// HTC Vive Tracker 3 位姿记录器
/// 
/// 功能:
/// - 通过 SteamVR/OpenVR API 采集 Tracker 设备的空间位姿
/// - 输出位置(米)和姿态(四元数/轴角)到 Unity 控制台
/// - 支持同时输出 UR 机械臂 TCP 实时位姿用于对比
/// 
/// 坐标系: SteamVR 世界坐标系 (右手系)
///   - X轴: 右
///   - Y轴: 上
///   - Z轴: 后(朝向 Tracker 背面)
/// 
/// 数据流: OpenVR API → TrackedDevicePose_t → Vector3 + Quaternion → 日志输出
/// 
/// 相关文档: 完整流程说明.md - 阶段1: Tracker位姿采集
/// </summary>
public class ViveTrackerPoseLogger : MonoBehaviour
{
    [Header("日志设置")]
    [Tooltip("是否启用位姿日志记录")]
    public bool enableLogging = true;
    
    [Tooltip("日志更新频率(秒)")]
    public float logUpdateInterval = 1.0f;
    
    [Tooltip("是否记录Tracker设备")]
    public bool logTrackers = true;
    
    [Tooltip("将Tracker原始位姿以轴角(rad)输出（SteamVR坐标系，位置mm/姿态rad）")]
    public bool logTrackerPoseInUrAxisAngle = true;
    
    [Header("头显与控制器")]
    [Tooltip("是否记录头显(HMD)")]
    public bool logHMD = true;
    
    [Tooltip("是否记录左手控制器")]
    public bool logLeftController = true;
    
    [Tooltip("是否记录右手控制器")]
    public bool logRightController = true;
    
    [Tooltip("是否输出头显/控制器的速度信息")]
    public bool logVelocity = false;
    
    [Header("扩展日志")]
    [Tooltip("是否输出UR机器人TCP末端位姿（右手坐标系，米/弧度）")]
    public bool logRobotTcpPose = true;

    [Tooltip("是否在日志中包含时间戳和分隔线")]
    public bool verboseHeader = true;

    [Header("位姿预测配置")]
    [Tooltip("启用SteamVR位姿预测，可获得更平滑的位姿数据")]
    public bool enablePosePrediction = false;

    [Tooltip("位姿预测时间（秒），建议0.01-0.03，越大越平滑但延迟越高")]
    [Range(0f, 0.1f)]
    public float predictionTimeSec = 0.011f;  // 默认11ms，约1帧

    [Header("位置滤波器")]
    [Tooltip("位置滤波器组件引用（可选，场景中需挂载 TrackerPositionFilter 脚本）")]
    public TrackerPositionFilter positionFilter;

    [Tooltip("如果未手动指定滤波器引用，是否自动查找场景中的 TrackerPositionFilter")]
    public bool autoFindFilter = true;
    
    [Tooltip("日志中输出滤波后的位姿数据（需启用滤波器）")]
    public bool logFilteredPose = true;

    private float nextLogTime = 0f;
    private CVRSystem vrSystem;
    private TrackedDevicePose_t[] trackedDevicePoses = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
    private readonly StringBuilder logBuilder = new StringBuilder(1024);
    private static readonly CultureInfo Culture = CultureInfo.InvariantCulture;
    
    // ========== 关闭保护机制 ==========
    // 用于防止在 Unity 编辑器退出 Play 模式时访问已失效的 OpenVR 运行时导致崩溃
    private volatile bool isShuttingDown = false;
    
    /// <summary>
    /// 检查 VR 系统是否可用（用于外部调用者检查）
    /// 在 Play 模式退出时，这个属性会立即返回 false
    /// 同时检查 OpenVR.System 以确保 OpenVR 运行时仍然有效
    /// </summary>
    public bool IsVRSystemAvailable
    {
        get
        {
            if (isShuttingDown || vrSystem == null)
                return false;
            
            // 检查 OpenVR 运行时是否仍然有效
            // 在 Play 模式退出时，OpenVR.System 可能会变为 null
            try
            {
                // 检查 Unity 是否仍在播放模式
                if (!Application.isPlaying)
                    return false;
                
                // 检查 OpenVR 运行时是否仍然存在
                if (OpenVR.System == null)
                    return false;
                    
                return true;
            }
            catch
            {
                return false;
            }
        }
    }
    
    // 缓存的设备索引（头显和控制器）
    private uint hmdIndex = OpenVR.k_unTrackedDeviceIndexInvalid;
    private uint leftControllerIndex = OpenVR.k_unTrackedDeviceIndexInvalid;
    private uint rightControllerIndex = OpenVR.k_unTrackedDeviceIndexInvalid;
    
    // SteamVR(右手, X右 Y上 Z后) -> UR基坐标(右手, X前 Y左 Z上) 的固定基变换
    // 位置：x_u = -z_s, y_u = -x_s, z_u = y_s
    // 旋转：q_u = Q * q_s * Q^{-1}，Q为上述轴系映射的四元数
    private static readonly Quaternion SteamVrToUrBasisQuat = ComputeSteamVrToUrBasisQuaternion();

    void Awake()
    {
        // 订阅应用退出事件，这会比 OnDisable/OnDestroy 更早触发
        Application.quitting += OnApplicationQuitting_Early;
        
        // 在 Awake 中初始化 VR 系统，确保在其他组件的 Start() 之前完成
        // 这样其他组件可以在 Start() 中安全地调用 GetTrackerPoseForCalibration
        InitializeVRSystem();
    }
    
    /// <summary>
    /// 初始化 VR 系统
    /// </summary>
    private void InitializeVRSystem()
    {
        if (vrSystem != null) return; // 已初始化
        
        if (OpenVR.System == null)
        {
            Debug.LogWarning("[ViveTrackerPoseLogger] SteamVR系统未初始化，尝试初始化...");
            
            EVRInitError vrError = EVRInitError.None;
            vrSystem = OpenVR.Init(ref vrError, EVRApplicationType.VRApplication_Scene);
            
            if (vrError != EVRInitError.None)
            {
                Debug.LogError($"[ViveTrackerPoseLogger] SteamVR初始化失败: {vrError}");
                return;
            }
            Debug.Log("[ViveTrackerPoseLogger] SteamVR 在 Awake 中初始化成功");
        }
        else
        {
            vrSystem = OpenVR.System;
            Debug.Log("[ViveTrackerPoseLogger] 从 OpenVR.System 获取 vrSystem 成功");
        }
    }
    
    /// <summary>
    /// 应用程序即将退出时的早期回调（在 OnDisable 之前）
    /// </summary>
    private void OnApplicationQuitting_Early()
    {
        isShuttingDown = true;
    }

    void Start()
    {
        // vrSystem 已在 Awake 中初始化，这里只做后续设置
        if (vrSystem == null)
        {
            // 如果 Awake 中初始化失败，再尝试一次
            InitializeVRSystem();
        }
        
        if (vrSystem != null)
        {
            UpdateHMDAndControllerIndices();
            LogConnectedTargetDevices();
        }
        
        // 自动查找滤波器组件
        if (positionFilter == null && autoFindFilter)
        {
            positionFilter = FindObjectOfType<TrackerPositionFilter>();
            if (positionFilter != null)
            {
                Debug.Log("[ViveTrackerPoseLogger] 自动找到 TrackerPositionFilter 组件");
                if (logFilteredPose && positionFilter.IsFilterEnabled)
                {
                    Debug.Log("[ViveTrackerPoseLogger] 日志将输出滤波后的位姿数据");
                }
            }
        }
        else if (positionFilter != null && logFilteredPose && positionFilter.IsFilterEnabled)
        {
            Debug.Log("[ViveTrackerPoseLogger] 日志将输出滤波后的位姿数据");
        }
    }

    /// <summary>
    /// 更新头显和控制器设备索引（处理热插拔）
    /// </summary>
    void UpdateHMDAndControllerIndices()
    {
        // 使用 IsVRSystemAvailable 进行完整检查
        if (!IsVRSystemAvailable) return;

        // HMD 始终是索引 0
        hmdIndex = OpenVR.k_unTrackedDeviceIndex_Hmd;

        // 查找左右手控制器
        leftControllerIndex = vrSystem.GetTrackedDeviceIndexForControllerRole(ETrackedControllerRole.LeftHand);
        rightControllerIndex = vrSystem.GetTrackedDeviceIndexForControllerRole(ETrackedControllerRole.RightHand);
    }

    void Update()
    {
        // 关闭保护：使用 IsVRSystemAvailable 进行全面检查
        // 这会同时检查 isShuttingDown、vrSystem 和 SteamVR_Behaviour.isPlaying
        if (!enableLogging || !IsVRSystemAvailable)
            return;

        // 按指定间隔记录位姿
        if (Time.time >= nextLogTime)
        {
            try
            {
                // 再次检查关闭状态（可能在上面的检查后发生变化）
                if (!IsVRSystemAvailable) return;
                
                // 刷新设备索引（处理控制器热插拔）
                UpdateHMDAndControllerIndices();
                LogTargetDevicePoses();
                nextLogTime = Time.time + logUpdateInterval;
            }
            catch (System.Exception ex)
            {
                if (!isShuttingDown)
                {
                    Debug.LogWarning($"[ViveTrackerPoseLogger] Update 异常: {ex.Message}");
                }
            }
        }
    }

    /// <summary>
    /// 记录目标设备的连接信息
    /// </summary>
    void LogConnectedTargetDevices()
    {
        // 静默初始化，不输出设备列表信息
    }

    /// <summary>
    /// 记录目标设备的位姿信息
    /// </summary>
    void LogTargetDevicePoses()
    {
        // 安全检查：确保VR系统仍然可用
        if (!IsVRSystemAvailable) return;
        
        logBuilder.Clear();

        bool hasAnyEntry = false;

        if (verboseHeader)
        {
            logBuilder.AppendLine($"=== Pose Snapshot [t={Time.time.ToString("F2", Culture)}s] ===");
        }

        if (logRobotTcpPose && TryGetRobotTcpPose(out Vector3 tcpPositionMm, out Vector3 tcpRotationRad))
        {
            logBuilder.AppendLine(FormatRobotPoseLine("TCP", tcpPositionMm, tcpRotationRad));
            hasAnyEntry = true;
        }
        else if (logRobotTcpPose)
        {
            logBuilder.AppendLine("TCP: 数据不可用 (未连接或暂未更新)");
            hasAnyEntry = true;
        }

        // 再次检查VR系统是否可用（在调用原生API之前）
        if (!IsVRSystemAvailable) return;

        // 获取当前帧的所有设备位姿
        vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, 0f, trackedDevicePoses);

        // ===== 头显位姿 =====
        if (logHMD && hmdIndex != OpenVR.k_unTrackedDeviceIndexInvalid)
        {
            if (TryGetDevicePoseWithVelocity(hmdIndex, out Vector3 hmdPosMm, out Quaternion hmdRot, out Vector3 hmdVel, out Vector3 hmdAngVel))
            {
                LogHMDOrControllerData("HMD(头显)", hmdIndex, hmdPosMm, hmdRot, hmdVel, hmdAngVel);
                hasAnyEntry = true;
            }
            else
            {
                logBuilder.AppendLine("HMD(头显): 位姿无效或未连接");
            }
        }

        // ===== 左手控制器位姿 =====
        if (logLeftController)
        {
            if (leftControllerIndex != OpenVR.k_unTrackedDeviceIndexInvalid)
            {
                if (TryGetDevicePoseWithVelocity(leftControllerIndex, out Vector3 lcPosMm, out Quaternion lcRot, out Vector3 lcVel, out Vector3 lcAngVel))
                {
                    LogHMDOrControllerData("左手控制器", leftControllerIndex, lcPosMm, lcRot, lcVel, lcAngVel);
                    hasAnyEntry = true;
                }
                else
                {
                    logBuilder.AppendLine($"左手控制器[ID:{leftControllerIndex}]: 位姿无效");
                }
            }
            else
            {
                logBuilder.AppendLine("左手控制器: 未连接");
            }
        }

        // ===== 右手控制器位姿 =====
        if (logRightController)
        {
            if (rightControllerIndex != OpenVR.k_unTrackedDeviceIndexInvalid)
            {
                if (TryGetDevicePoseWithVelocity(rightControllerIndex, out Vector3 rcPosMm, out Quaternion rcRot, out Vector3 rcVel, out Vector3 rcAngVel))
                {
                    LogHMDOrControllerData("右手控制器", rightControllerIndex, rcPosMm, rcRot, rcVel, rcAngVel);
                    hasAnyEntry = true;
                }
                else
                {
                    logBuilder.AppendLine($"右手控制器[ID:{rightControllerIndex}]: 位姿无效");
                }
            }
            else
            {
                logBuilder.AppendLine("右手控制器: 未连接");
            }
        }

        for (uint deviceId = 0; deviceId < OpenVR.k_unMaxTrackedDeviceCount; deviceId++)
        {
            if (!vrSystem.IsTrackedDeviceConnected(deviceId))
                continue;

            var deviceClass = vrSystem.GetTrackedDeviceClass(deviceId);

        // 只处理 Tracker 设备
            if (!ShouldLogDevice(deviceClass))
                continue;

            if (TryGetRightHandedPose(deviceId, out Vector3 positionMm, out Quaternion rotationQuat))
            {
                string deviceType = GetDeviceTypeString(deviceClass);
                string label = $"{deviceType}[ID:{deviceId}]";
                // 原始SteamVR坐标+四元数
                logBuilder.AppendLine(FormatTrackerPoseLine(label, positionMm, rotationQuat));

                // 可选：输出原始SteamVR坐标系下的轴角（与TCP格式相同的 [x,y,z,rx,ry,rz] 结构，单位mm/rad）
                if (logTrackerPoseInUrAxisAngle)
                {
                    Vector3 posRawMm = positionMm; // 原始SteamVR坐标（mm）
                    Vector3 rotRawAxisAngle = QuaternionToRotationVector(rotationQuat); // (rad)
                    string labelAa = $"{deviceType}[ID:{deviceId}]-AxisAngle";
                    logBuilder.AppendLine(FormatRobotPoseLine(labelAa, posRawMm, rotRawAxisAngle));
                }
                hasAnyEntry = true;
            }
        }

        if (hasAnyEntry)
        {
            if (verboseHeader)
            {
                logBuilder.AppendLine("================================");
            }
            Debug.Log(logBuilder.ToString().TrimEnd());
        }
    }

    /// <summary>
    /// 根据设置判断是否应该记录此设备类型
    /// </summary>
    bool ShouldLogDevice(ETrackedDeviceClass deviceClass)
    {
        return deviceClass == ETrackedDeviceClass.GenericTracker && logTrackers;
    }

    /// <summary>
    /// 记录头显或控制器设备数据
    /// </summary>
    void LogHMDOrControllerData(string deviceName, uint deviceId, Vector3 positionMm, Quaternion rotationQuat, Vector3 velocity, Vector3 angularVelocity)
    {
        string label = $"{deviceName}[ID:{deviceId}]";
        
        // 输出四元数格式
        logBuilder.AppendLine(FormatTrackerPoseLine(label, positionMm, rotationQuat));

        // 可选：输出轴角格式（与Tracker保持一致）
        if (logTrackerPoseInUrAxisAngle)
        {
            Vector3 rotAxisAngle = QuaternionToRotationVector(rotationQuat);
            string labelAa = $"{deviceName}[ID:{deviceId}]-AxisAngle";
            logBuilder.AppendLine(FormatRobotPoseLine(labelAa, positionMm, rotAxisAngle));
        }

        // 可选：输出速度信息
        if (logVelocity)
        {
            logBuilder.AppendLine(string.Format(Culture,
                "  速度: [{0:F3}, {1:F3}, {2:F3}] m/s | 角速度: [{3:F3}, {4:F3}, {5:F3}] rad/s",
                velocity.x, velocity.y, velocity.z,
                angularVelocity.x, angularVelocity.y, angularVelocity.z));
        }
    }

    /// <summary>
    /// 获取设备位姿（带速度信息）
    /// </summary>
    bool TryGetDevicePoseWithVelocity(uint deviceId, out Vector3 positionMm, out Quaternion rotationQuat, out Vector3 velocity, out Vector3 angularVelocity)
    {
        positionMm = Vector3.zero;
        rotationQuat = Quaternion.identity;
        velocity = Vector3.zero;
        angularVelocity = Vector3.zero;

        if (deviceId >= OpenVR.k_unMaxTrackedDeviceCount)
            return false;

        var pose = trackedDevicePoses[deviceId];
        if (!pose.bPoseIsValid || !pose.bDeviceIsConnected)
        {
            return false;
        }

        var matrix = pose.mDeviceToAbsoluteTracking;

        // 位置（米 → 毫米）
        positionMm = new Vector3(
            matrix.m3 * 1000f,
            matrix.m7 * 1000f,
            matrix.m11 * 1000f);

        // 旋转
        rotationQuat = QuaternionFromMatrix(matrix);

        // 速度
        velocity = new Vector3(pose.vVelocity.v0, pose.vVelocity.v1, pose.vVelocity.v2);
        angularVelocity = new Vector3(pose.vAngularVelocity.v0, pose.vAngularVelocity.v1, pose.vAngularVelocity.v2);

        return true;
    }



    /// <summary>
    /// 从SteamVR矩阵获取Unity位置
    /// 如果启用了位置滤波器，会自动应用 1€ 滤波
    /// </summary>
    bool TryGetRightHandedPose(uint deviceId, out Vector3 positionMm, out Quaternion rotationQuat)
    {
        positionMm = Vector3.zero;
        rotationQuat = Quaternion.identity;

        var pose = trackedDevicePoses[deviceId];
        if (!pose.bPoseIsValid || !pose.bDeviceIsConnected)
        {
            return false;
        }

        var matrix = pose.mDeviceToAbsoluteTracking;

        // 获取原始位置（mm）
        Vector3 rawPositionMm = new Vector3(
            matrix.m3 * 1000f,
            matrix.m7 * 1000f,
            matrix.m11 * 1000f);

        // 获取旋转
        rotationQuat = QuaternionFromMatrix(matrix);

        // 应用位置滤波（如果可用）
        if (logFilteredPose && positionFilter != null && positionFilter.IsFilterEnabled)
        {
            // 获取速度（m/s）
            Vector3 velocityMs = new Vector3(pose.vVelocity.v0, pose.vVelocity.v1, pose.vVelocity.v2);
            
            // 滤波器内部会将速度转换为 mm/s
            positionMm = positionFilter.FilterPosition(deviceId, rawPositionMm, velocityMs);
        }
        else
        {
            positionMm = rawPositionMm;
        }

        return true;
    }



    /// <summary>
    /// 获取设备类型的中文描述
    /// </summary>
    string GetDeviceTypeString(ETrackedDeviceClass deviceClass)
    {
        switch (deviceClass)
        {
            case ETrackedDeviceClass.GenericTracker:
                return "定位器(Tracker)";
            default:
                return $"未知设备类型({deviceClass})";
        }
    }

    /// <summary>
    /// 手动触发一次位姿记录(供外部调用)
    /// </summary>
    [ContextMenu("立即记录位姿")]
    public void LogPosesNow()
    {
        if (vrSystem != null)
        {
            LogTargetDevicePoses();
        }
    }

    /// <summary>
    /// 刷新设备列表
    /// </summary>
    [ContextMenu("刷新设备列表")]
    public void RefreshDeviceList()
    {
        // 静默刷新，不输出额外信息
    }

    /// <summary>
    /// 获取指定设备ID的当前位姿
    /// </summary>
    /// <param name="deviceId">设备ID</param>
    /// <returns>位姿信息，如果无效返回null</returns>
    public (Vector3 position, Quaternion rotation)? GetDevicePose(uint deviceId)
    {
        if (vrSystem == null || !vrSystem.IsTrackedDeviceConnected(deviceId))
            return null;

        float predictTime = enablePosePrediction ? predictionTimeSec : 0f;
        vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, predictTime, trackedDevicePoses);
        
        var pose = trackedDevicePoses[deviceId];
        if (pose.bPoseIsValid && pose.bDeviceIsConnected)
        {
            var matrix = pose.mDeviceToAbsoluteTracking;
            Vector3 position = GetPositionFromMatrix(matrix);
            Quaternion rotation = GetRotationFromMatrix(matrix);
            return (position, rotation);
        }

        return null;
    }

    /// <summary>
    /// 获取所有Tracker设备的位姿
    /// </summary>
    public System.Collections.Generic.Dictionary<uint, (Vector3 position, Quaternion rotation)> GetAllTrackerPoses()
    {
        var result = new System.Collections.Generic.Dictionary<uint, (Vector3 position, Quaternion rotation)>();
        
        if (vrSystem == null) return result;

        float predictTime = enablePosePrediction ? predictionTimeSec : 0f;
        vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, predictTime, trackedDevicePoses);
        
        for (uint deviceId = 0; deviceId < OpenVR.k_unMaxTrackedDeviceCount; deviceId++)
        {
            if (vrSystem.IsTrackedDeviceConnected(deviceId))
            {
                var deviceClass = vrSystem.GetTrackedDeviceClass(deviceId);
                if (deviceClass == ETrackedDeviceClass.GenericTracker)
                {
                    var pose = trackedDevicePoses[deviceId];
                    if (pose.bPoseIsValid && pose.bDeviceIsConnected)
                    {
                        var matrix = pose.mDeviceToAbsoluteTracking;
                        Vector3 position = GetPositionFromMatrix(matrix);
                        Quaternion rotation = GetRotationFromMatrix(matrix);
                        result[deviceId] = (position, rotation);
                    }
                }
            }
        }
        
        return result;
    }

    /// <summary>
    /// 获取机器人TCP位姿（用于手眼标定数据采集）
    /// </summary>
    /// <param name="positionMm">TCP位置（毫米）</param>
    /// <param name="rotationRad">TCP旋转（弧度，轴角表示）</param>
    /// <returns>是否成功获取数据</returns>
    public bool GetRobotTcpPoseForCalibration(out Vector3 positionMm, out Vector3 rotationRad)
    {
        return TryGetRobotTcpPose(out positionMm, out rotationRad);
    }

    #region 头显与控制器公开接口

    /// <summary>
    /// 获取头显位姿
    /// </summary>
    /// <param name="positionMm">位置（毫米）</param>
    /// <param name="rotation">旋转（四元数）</param>
    /// <returns>是否成功获取</returns>
    public bool GetHMDPose(out Vector3 positionMm, out Quaternion rotation)
    {
        positionMm = Vector3.zero;
        rotation = Quaternion.identity;

        if (vrSystem == null) return false;

        float predictTime = enablePosePrediction ? predictionTimeSec : 0f;
        vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, predictTime, trackedDevicePoses);
        
        return TryGetDevicePoseWithVelocity(hmdIndex, out positionMm, out rotation, out _, out _);
    }

    /// <summary>
    /// 获取左手控制器位姿
    /// </summary>
    public bool GetLeftControllerPose(out Vector3 positionMm, out Quaternion rotation)
    {
        positionMm = Vector3.zero;
        rotation = Quaternion.identity;

        if (vrSystem == null) return false;

        UpdateHMDAndControllerIndices();
        if (leftControllerIndex == OpenVR.k_unTrackedDeviceIndexInvalid) return false;

        float predictTime = enablePosePrediction ? predictionTimeSec : 0f;
        vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, predictTime, trackedDevicePoses);
        
        return TryGetDevicePoseWithVelocity(leftControllerIndex, out positionMm, out rotation, out _, out _);
    }

    /// <summary>
    /// 获取右手控制器位姿
    /// </summary>
    public bool GetRightControllerPose(out Vector3 positionMm, out Quaternion rotation)
    {
        positionMm = Vector3.zero;
        rotation = Quaternion.identity;

        if (vrSystem == null) return false;

        UpdateHMDAndControllerIndices();
        if (rightControllerIndex == OpenVR.k_unTrackedDeviceIndexInvalid) return false;

        float predictTime = enablePosePrediction ? predictionTimeSec : 0f;
        vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, predictTime, trackedDevicePoses);
        
        return TryGetDevicePoseWithVelocity(rightControllerIndex, out positionMm, out rotation, out _, out _);
    }

    /// <summary>
    /// 获取控制器速度（用于抛出物体等）
    /// </summary>
    public bool GetControllerVelocity(bool isLeftHand, out Vector3 velocity, out Vector3 angularVelocity)
    {
        velocity = Vector3.zero;
        angularVelocity = Vector3.zero;

        if (vrSystem == null) return false;

        UpdateHMDAndControllerIndices();
        uint index = isLeftHand ? leftControllerIndex : rightControllerIndex;
        if (index == OpenVR.k_unTrackedDeviceIndexInvalid) return false;

        float predictTime = enablePosePrediction ? predictionTimeSec : 0f;
        vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, predictTime, trackedDevicePoses);
        
        return TryGetDevicePoseWithVelocity(index, out _, out _, out velocity, out angularVelocity);
    }

    /// <summary>
    /// 打印当前连接的所有VR设备
    /// </summary>
    [ContextMenu("打印所有连接设备")]
    public void PrintAllConnectedDevices()
    {
        if (vrSystem == null)
        {
            Debug.LogWarning("[ViveTrackerPoseLogger] VR 系统未初始化");
            return;
        }

        StringBuilder sb = new StringBuilder();
        sb.AppendLine("=== 已连接的 VR 设备 ===");

        for (uint i = 0; i < OpenVR.k_unMaxTrackedDeviceCount; i++)
        {
            if (!vrSystem.IsTrackedDeviceConnected(i))
                continue;

            var deviceClass = vrSystem.GetTrackedDeviceClass(i);
            string deviceType = GetDeviceClassName(deviceClass);
            string role = "";

            if (deviceClass == ETrackedDeviceClass.Controller)
            {
                var controllerRole = vrSystem.GetControllerRoleForTrackedDeviceIndex(i);
                role = controllerRole == ETrackedControllerRole.LeftHand ? " (左手)" :
                       controllerRole == ETrackedControllerRole.RightHand ? " (右手)" : "";
            }

            sb.AppendLine($"  [{i}] {deviceType}{role}");
        }

        sb.AppendLine("========================");
        Debug.Log(sb.ToString());
    }

    string GetDeviceClassName(ETrackedDeviceClass deviceClass)
    {
        switch (deviceClass)
        {
            case ETrackedDeviceClass.HMD: return "头显(HMD)";
            case ETrackedDeviceClass.Controller: return "控制器";
            case ETrackedDeviceClass.GenericTracker: return "定位器(Tracker)";
            case ETrackedDeviceClass.TrackingReference: return "基站";
            case ETrackedDeviceClass.DisplayRedirect: return "显示重定向";
            default: return $"未知({deviceClass})";
        }
    }

    #endregion

    /// <summary>
    /// 获取指定Tracker的位姿（用于手眼标定数据采集）
    /// 返回原始SteamVR坐标系下的位姿（毫米 + 四元数）
    /// 
    /// 安全说明：此方法包含完整的关闭保护，可以在 Play 模式退出期间安全调用
    /// </summary>
    /// <param name="deviceId">设备ID</param>
    /// <param name="positionMm">位置（毫米）</param>
    /// <param name="rotation">旋转（四元数）</param>
    /// <returns>是否成功获取数据</returns>
    public bool GetTrackerPoseForCalibration(uint deviceId, out Vector3 positionMm, out Quaternion rotation)
    {
        positionMm = Vector3.zero;
        rotation = Quaternion.identity;

        // ========== 关闭保护检查 ==========
        // 使用 IsVRSystemAvailable 进行全面检查
        // 这会同时检查 isShuttingDown、vrSystem 和 SteamVR_Behaviour.isPlaying
        if (!IsVRSystemAvailable)
        {
            // 详细诊断：找出具体是哪个条件导致失败
            if (isShuttingDown)
            {
                Debug.LogWarning($"[ViveTrackerPoseLogger] GetTrackerPoseForCalibration(ID={deviceId}): 失败原因 - isShuttingDown=true");
            }
            else if (vrSystem == null)
            {
                Debug.LogWarning($"[ViveTrackerPoseLogger] GetTrackerPoseForCalibration(ID={deviceId}): 失败原因 - vrSystem==null");
            }
            else if (!Application.isPlaying)
            {
                Debug.LogWarning($"[ViveTrackerPoseLogger] GetTrackerPoseForCalibration(ID={deviceId}): 失败原因 - Application.isPlaying=false");
            }
            else if (OpenVR.System == null)
            {
                Debug.LogWarning($"[ViveTrackerPoseLogger] GetTrackerPoseForCalibration(ID={deviceId}): 失败原因 - OpenVR.System==null");
            }
            else
            {
                Debug.LogWarning($"[ViveTrackerPoseLogger] GetTrackerPoseForCalibration(ID={deviceId}): 失败原因 - 未知(IsVRSystemAvailable返回false)");
            }
            return false;
        }

        // 使用 try-catch 包装所有 OpenVR 原生调用
        // 即使检查了 IsVRSystemAvailable，也可能存在竞态条件
        // 在极端情况下（如 SteamVR 插件先关闭了 OpenVR 运行时）仍需要保护
        try
        {
            // 延迟初始化：如果 vrSystem 为 null，尝试初始化
            if (vrSystem == null)
            {
                // 再次检查是否可用（可能在上面的检查后发生变化）
                if (!IsVRSystemAvailable) return false;
                
                EnsureVRSystemInitialized();
            }

            if (vrSystem == null)
            {
                // 只在非关闭状态下记录警告
                if (!isShuttingDown)
                {
                    Debug.LogWarning($"[ViveTrackerPoseLogger] GetTrackerPoseForCalibration: vrSystem 初始化失败");
                }
                return false;
            }
            
            // ========== 原生 OpenVR API 调用（需要 try-catch 保护）==========
            // 这些调用可能在 VR 运行时已关闭时抛出异常或导致崩溃
            
            if (!IsVRSystemAvailable) return false;  // 每次原生调用前检查
            
            if (!vrSystem.IsTrackedDeviceConnected(deviceId))
            {
                if (!isShuttingDown)
                {
                    Debug.LogWarning($"[ViveTrackerPoseLogger] GetTrackerPoseForCalibration: 设备 ID={deviceId} 未连接");
                }
                return false;
            }

            if (!IsVRSystemAvailable) return false;  // 每次原生调用前检查

            // 更新位姿数据（使用可配置的预测时间）
            // predictionTimeSec > 0 时启用 SteamVR 运动预测，可获得更平滑的位姿
            float predictTime = enablePosePrediction ? predictionTimeSec : 0f;
            vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, predictTime, trackedDevicePoses);

            if (!IsVRSystemAvailable) return false;

            bool result = TryGetRightHandedPose(deviceId, out positionMm, out rotation);
            if (!result && !isShuttingDown)
            {
                var pose = trackedDevicePoses[deviceId];
                Debug.LogWarning($"[ViveTrackerPoseLogger] GetTrackerPoseForCalibration(ID={deviceId}): TryGetRightHandedPose失败 - bPoseIsValid={pose.bPoseIsValid}, bDeviceIsConnected={pose.bDeviceIsConnected}");
            }
            return result;
        }
        catch (System.Exception ex)
        {
            // 捕获所有异常，防止崩溃
            // 这可能包括：
            // - NullReferenceException（vrSystem 在调用过程中变为 null）
            // - AccessViolationException（OpenVR 运行时已关闭）
            // - 其他原生代码异常
            if (!isShuttingDown)
            {
                Debug.LogWarning($"[ViveTrackerPoseLogger] GetTrackerPoseForCalibration 异常（可能是 VR 运行时正在关闭）: {ex.Message}");
            }
            return false;
        }
    }
    
    /// <summary>
    /// 确保 VR 系统已初始化（延迟初始化）
    /// </summary>
    private void EnsureVRSystemInitialized()
    {
        if (vrSystem != null) return;
        
        if (OpenVR.System != null)
        {
            vrSystem = OpenVR.System;
            Debug.Log("[ViveTrackerPoseLogger] 延迟初始化: 从 OpenVR.System 获取 vrSystem");
            UpdateHMDAndControllerIndices();
        }
        else
        {
            Debug.LogWarning("[ViveTrackerPoseLogger] 延迟初始化: OpenVR.System 也为 null，尝试初始化 OpenVR...");
            EVRInitError vrError = EVRInitError.None;
            vrSystem = OpenVR.Init(ref vrError, EVRApplicationType.VRApplication_Scene);
            
            if (vrError != EVRInitError.None)
            {
                Debug.LogError($"[ViveTrackerPoseLogger] OpenVR 初始化失败: {vrError}");
                vrSystem = null;
            }
            else
            {
                Debug.Log("[ViveTrackerPoseLogger] 延迟初始化: OpenVR 初始化成功");
                UpdateHMDAndControllerIndices();
            }
        }
    }

    bool TryGetRobotTcpPose(out Vector3 positionMm, out Vector3 rotationRad)
    {
        positionMm = Vector3.zero;
        rotationRad = Vector3.zero;

        if (!ur_data_processing.UR_Stream_Data.is_alive)
        {
            return false;
        }

        var tcpPos = ur_data_processing.UR_Stream_Data.C_Position;
        var tcpRot = ur_data_processing.UR_Stream_Data.C_Orientation;

        if (tcpPos == null || tcpRot == null || tcpPos.Length < 3 || tcpRot.Length < 3)
        {
            return false;
        }

        positionMm = new Vector3(
            (float)(tcpPos[0] * 1000.0),
            (float)(tcpPos[1] * 1000.0),
            (float)(tcpPos[2] * 1000.0));

        rotationRad = new Vector3(
            (float)tcpRot[0],
            (float)tcpRot[1],
            (float)tcpRot[2]);

        return true;
    }

    // 辅助：从 HmdMatrix34_t 提取位置（米）并返回 Unity Vector3
    Vector3 GetPositionFromMatrix(HmdMatrix34_t matrix)
    {
        return new Vector3(matrix.m3, matrix.m7, matrix.m11);
    }

    // 辅助：从 HmdMatrix34_t 提取旋转并返回四元数
    Quaternion GetRotationFromMatrix(HmdMatrix34_t matrix)
    {
        return QuaternionFromMatrix(matrix);
    }

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

        q = NormalizeQuaternion(q);
        
        // 四元数符号规范化: 强制 q.w >= 0
        // 确保四元数的唯一性表示
        if (q.w < 0f)
        {
            q.x = -q.x;
            q.y = -q.y;
            q.z = -q.z;
            q.w = -q.w;
        }
        
        return q;
    }

    // 计算SteamVR到UR基坐标的固定四元数Q（用于q_u = Q * q_s * Q^{-1}）
    static Quaternion ComputeSteamVrToUrBasisQuaternion()
    {
        // R = [[0,0,-1],[-1,0,0],[0,1,0]]
        HmdMatrix34_t m = new HmdMatrix34_t();
        m.m0 = 0f;  m.m1 = 0f;  m.m2 = -1f; m.m3 = 0f;
        m.m4 = -1f; m.m5 = 0f;  m.m6 = 0f;  m.m7 = 0f;
        m.m8 = 0f;  m.m9 = 1f;  m.m10 = 0f; m.m11 = 0f;
        return QuaternionFromMatrix(m);
    }

    // 位置基变换（mm）
    static Vector3 SteamVrToUr_Position(Vector3 posMm)
    {
        // x_u = -z_s, y_u = -x_s, z_u = y_s
        return new Vector3(-posMm.z, -posMm.x, posMm.y);
    }

    // 旋转基变换
    static Quaternion SteamVrToUr_Rotation(Quaternion qSteamVr)
    {
        return NormalizeQuaternion(SteamVrToUrBasisQuat * qSteamVr * Quaternion.Inverse(SteamVrToUrBasisQuat));
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

    static Vector3 QuaternionToRotationVector(Quaternion q)
    {
        // 步骤1: 归一化四元数
        q = NormalizeQuaternion(q);
        
        // 步骤2: 四元数符号规范化 (强制 q.w >= 0)
        // 因为 q 和 -q 表示相同的旋转，统一使用 w >= 0 的表示
        if (q.w < 0f)
        {
            q.x = -q.x;
            q.y = -q.y;
            q.z = -q.z;
            q.w = -q.w;
        }
        
        // 步骤3: 计算旋转角度 (现在 q.w 一定在 [0, 1] 范围内)
        float wClamped = Mathf.Clamp(q.w, 0f, 1f);
        float angle = 2f * Mathf.Acos(wClamped);
        
        // 步骤4: 处理特殊情况
        
        // 情况A: 接近 180° (q.w ≈ 0)
        // 使用公式: axis = normalize([q.x, q.y, q.z]) * π
        if (angle > Mathf.PI - 1e-4f)
        {
            Vector3 axis180 = new Vector3(q.x, q.y, q.z);
            float axisMag = axis180.magnitude;
            if (axisMag > 1e-8f)
            {
                axis180 = axis180 / axisMag;
            }
            else
            {
                // 极端情况: 无法确定轴向，使用默认轴
                axis180 = new Vector3(1f, 0f, 0f);
            }
            return axis180 * angle;
        }
        
        // 情况B: 接近 0° (q.w ≈ 1)
        if (angle < 1e-6f)
        {
            return Vector3.zero;
        }
        
        // 情况C: 一般情况 (0° < angle < 180°)
        // 使用公式: rotationVector = [q.x, q.y, q.z] * (angle / sin(angle/2))
        float sinHalfAngle = Mathf.Sin(angle * 0.5f);
        float scale = angle / sinHalfAngle;
        return new Vector3(q.x * scale, q.y * scale, q.z * scale);
    }

    string FormatRobotPoseLine(string label, Vector3 positionMm, Vector3 rotationRad)
    {
        return string.Format(Culture,
            "{0}: [{1:F2}, {2:F2}, {3:F2}, {4:F4}, {5:F4}, {6:F4}] (mm, mm, mm, rad, rad, rad)",
            label,
            positionMm.x, positionMm.y, positionMm.z,
            rotationRad.x, rotationRad.y, rotationRad.z);
    }

    string FormatTrackerPoseLine(string label, Vector3 positionMm, Quaternion rotationQuat)
    {
        // 检查是否使用了滤波器
        bool isFiltered = logFilteredPose && positionFilter != null && positionFilter.IsFilterEnabled;
        string filterTag = isFiltered ? "[滤波]" : "";
        return string.Format(Culture,
            "{0}{1}: [{2:F2}, {3:F2}, {4:F2}, {5:F4}, {6:F4}, {7:F4}, {8:F4}] (mm, mm, mm, qx, qy, qz, qw)",
            label,
            filterTag,
            positionMm.x, positionMm.y, positionMm.z,
            rotationQuat.x, rotationQuat.y, rotationQuat.z, rotationQuat.w);
    }

    /// <summary>
    /// 组件禁用时立即设置关闭标志
    /// OnDisable 比 OnDestroy 更早触发，可以更早地阻止对 OpenVR 的访问
    /// </summary>
    void OnDisable()
    {
        // 立即设置关闭标志，阻止任何外部调用访问 OpenVR API
        isShuttingDown = true;
    }
    
    /// <summary>
    /// 组件启用时重置关闭标志
    /// </summary>
    void OnEnable()
    {
        isShuttingDown = false;
    }

    void OnDestroy()
    {
        // 取消订阅事件，防止内存泄漏
        Application.quitting -= OnApplicationQuitting_Early;
        
        // 安全清理：只清空本地引用，不调用 OpenVR.Shutdown()
        // 
        // 重要说明：
        // OpenVR.Shutdown() 是一个全局操作，会关闭整个 OpenVR 运行时。
        // 在 Unity 编辑器中退出 Play 模式时调用它会导致：
        // 1. 其他仍在运行的组件尝试访问已关闭的 VR 系统时崩溃
        // 2. Unity 编辑器闪退
        // 
        // 正确做法：
        // - 在编辑器中：不调用 Shutdown，让 Unity 自然清理
        // - 在构建版本中：由 SteamVR 插件或应用退出时自动处理
        // 
        // 如果确实需要手动关闭 OpenVR（比如在构建版本的特定场景），
        // 应该使用 OnApplicationQuit 并确保是最后一个调用者。
        
        isShuttingDown = true;
        vrSystem = null;
    }
    
    /// <summary>
    /// 应用程序退出时的清理（仅在构建版本中有效）
    /// </summary>
    void OnApplicationQuit()
    {
        // 在应用程序真正退出时才调用 Shutdown
        // 注意：在 Unity 编辑器中停止 Play 模式时，OnApplicationQuit 可能不会被调用
        // 或者调用时机不可预测，所以这里也要谨慎
        #if !UNITY_EDITOR
        if (vrSystem != null)
        {
            try
            {
                OpenVR.Shutdown();
            }
            catch
            {
                // 忽略退出时的异常
            }
            vrSystem = null;
        }
        #endif
    }
}
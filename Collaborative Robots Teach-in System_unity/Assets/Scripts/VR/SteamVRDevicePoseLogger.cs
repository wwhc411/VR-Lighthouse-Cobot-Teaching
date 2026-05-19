using System.Globalization;
using System.Text;
using UnityEngine;
using Valve.VR;

/// <summary>
/// SteamVR 头显和控制器位姿记录器
/// 
/// 功能:
/// - 通过 SteamVR/OpenVR API 采集头显(HMD)和左右手控制器的空间位姿
/// - 输出位置(毫米)和姿态(四元数/轴角)到 Unity 控制台
/// - 输出格式与 ViveTrackerPoseLogger 保持一致
/// 
/// 坐标系: SteamVR 世界坐标系 (右手系)
///   - X轴: 右
///   - Y轴: 上
///   - Z轴: 后
/// 
/// 支持设备:
///   - HMD (头显)
///   - Controller (左/右手控制器)
/// </summary>
public class SteamVRDevicePoseLogger : MonoBehaviour
{
    [Header("日志设置")]
    [Tooltip("是否启用位姿日志记录")]
    public bool enableLogging = true;
    
    [Tooltip("日志更新频率(秒)")]
    public float logUpdateInterval = 1.0f;

    [Header("设备选择")]
    [Tooltip("是否记录头显(HMD)")]
    public bool logHMD = true;
    
    [Tooltip("是否记录左手控制器")]
    public bool logLeftController = true;
    
    [Tooltip("是否记录右手控制器")]
    public bool logRightController = true;

    [Header("输出格式")]
    [Tooltip("是否输出轴角表示(rad)，否则只输出四元数")]
    public bool logAxisAngle = true;
    
    [Tooltip("是否在日志中包含时间戳和分隔线")]
    public bool verboseHeader = true;
    
    [Tooltip("是否输出设备速度信息")]
    public bool logVelocity = false;

    // 私有变量
    private float nextLogTime = 0f;
    private CVRSystem vrSystem;
    private TrackedDevicePose_t[] trackedDevicePoses = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
    private readonly StringBuilder logBuilder = new StringBuilder(1024);
    private static readonly CultureInfo Culture = CultureInfo.InvariantCulture;

    // 缓存的设备索引
    private uint hmdIndex = OpenVR.k_unTrackedDeviceIndexInvalid;
    private uint leftControllerIndex = OpenVR.k_unTrackedDeviceIndexInvalid;
    private uint rightControllerIndex = OpenVR.k_unTrackedDeviceIndexInvalid;

    void Start()
    {
        InitializeVRSystem();
        UpdateDeviceIndices();
    }

    void Update()
    {
        if (!enableLogging || vrSystem == null)
            return;

        // 按指定间隔记录位姿
        if (Time.time >= nextLogTime)
        {
            // 刷新设备索引（处理控制器热插拔）
            UpdateDeviceIndices();
            LogDevicePoses();
            nextLogTime = Time.time + logUpdateInterval;
        }
    }

    /// <summary>
    /// 初始化 SteamVR 系统
    /// </summary>
    void InitializeVRSystem()
    {
        if (OpenVR.System == null)
        {
            Debug.LogWarning("[SteamVRDevicePoseLogger] SteamVR系统未初始化，尝试初始化...");
            
            EVRInitError vrError = EVRInitError.None;
            vrSystem = OpenVR.Init(ref vrError, EVRApplicationType.VRApplication_Scene);
            
            if (vrError != EVRInitError.None)
            {
                Debug.LogError($"[SteamVRDevicePoseLogger] SteamVR初始化失败: {vrError}");
                return;
            }
        }
        else
        {
            vrSystem = OpenVR.System;
        }
        
        Debug.Log("[SteamVRDevicePoseLogger] SteamVR 系统初始化成功");
    }

    /// <summary>
    /// 更新设备索引（处理热插拔）
    /// </summary>
    void UpdateDeviceIndices()
    {
        if (vrSystem == null) return;

        // HMD 始终是索引 0
        hmdIndex = OpenVR.k_unTrackedDeviceIndex_Hmd;

        // 查找左右手控制器
        leftControllerIndex = vrSystem.GetTrackedDeviceIndexForControllerRole(ETrackedControllerRole.LeftHand);
        rightControllerIndex = vrSystem.GetTrackedDeviceIndexForControllerRole(ETrackedControllerRole.RightHand);
    }

    /// <summary>
    /// 记录所有设备的位姿信息
    /// </summary>
    void LogDevicePoses()
    {
        logBuilder.Clear();

        bool hasAnyEntry = false;

        if (verboseHeader)
        {
            logBuilder.AppendLine($"=== VR Device Pose Snapshot [t={Time.time.ToString("F2", Culture)}s] ===");
        }

        // 获取当前帧的所有设备位姿
        vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, 0f, trackedDevicePoses);

        // 记录 HMD
        if (logHMD && hmdIndex != OpenVR.k_unTrackedDeviceIndexInvalid)
        {
            if (TryGetDevicePose(hmdIndex, out Vector3 positionMm, out Quaternion rotationQuat, out Vector3 velocity, out Vector3 angularVelocity))
            {
                LogDeviceData("HMD(头显)", hmdIndex, positionMm, rotationQuat, velocity, angularVelocity);
                hasAnyEntry = true;
            }
            else
            {
                logBuilder.AppendLine("HMD(头显): 位姿无效或未连接");
            }
        }

        // 记录左手控制器
        if (logLeftController)
        {
            if (leftControllerIndex != OpenVR.k_unTrackedDeviceIndexInvalid)
            {
                if (TryGetDevicePose(leftControllerIndex, out Vector3 positionMm, out Quaternion rotationQuat, out Vector3 velocity, out Vector3 angularVelocity))
                {
                    LogDeviceData("左手控制器", leftControllerIndex, positionMm, rotationQuat, velocity, angularVelocity);
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

        // 记录右手控制器
        if (logRightController)
        {
            if (rightControllerIndex != OpenVR.k_unTrackedDeviceIndexInvalid)
            {
                if (TryGetDevicePose(rightControllerIndex, out Vector3 positionMm, out Quaternion rotationQuat, out Vector3 velocity, out Vector3 angularVelocity))
                {
                    LogDeviceData("右手控制器", rightControllerIndex, positionMm, rotationQuat, velocity, angularVelocity);
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

        if (hasAnyEntry)
        {
            if (verboseHeader)
            {
                logBuilder.AppendLine("================================================");
            }
            Debug.Log(logBuilder.ToString().TrimEnd());
        }
    }

    /// <summary>
    /// 记录单个设备的数据
    /// </summary>
    void LogDeviceData(string deviceName, uint deviceId, Vector3 positionMm, Quaternion rotationQuat, Vector3 velocity, Vector3 angularVelocity)
    {
        string label = $"{deviceName}[ID:{deviceId}]";
        
        // 输出四元数格式
        logBuilder.AppendLine(FormatPoseLineQuaternion(label, positionMm, rotationQuat));

        // 可选：输出轴角格式
        if (logAxisAngle)
        {
            Vector3 rotAxisAngle = QuaternionToRotationVector(rotationQuat);
            string labelAa = $"{deviceName}[ID:{deviceId}]-AxisAngle";
            logBuilder.AppendLine(FormatPoseLineAxisAngle(labelAa, positionMm, rotAxisAngle));
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
    /// 获取设备位姿（位置单位：毫米）
    /// </summary>
    bool TryGetDevicePose(uint deviceId, out Vector3 positionMm, out Quaternion rotationQuat, out Vector3 velocity, out Vector3 angularVelocity)
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

    #region 公开接口

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

        vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, 0f, trackedDevicePoses);
        
        return TryGetDevicePose(hmdIndex, out positionMm, out rotation, out _, out _);
    }

    /// <summary>
    /// 获取左手控制器位姿
    /// </summary>
    public bool GetLeftControllerPose(out Vector3 positionMm, out Quaternion rotation)
    {
        positionMm = Vector3.zero;
        rotation = Quaternion.identity;

        if (vrSystem == null) return false;

        UpdateDeviceIndices();
        if (leftControllerIndex == OpenVR.k_unTrackedDeviceIndexInvalid) return false;

        vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, 0f, trackedDevicePoses);
        
        return TryGetDevicePose(leftControllerIndex, out positionMm, out rotation, out _, out _);
    }

    /// <summary>
    /// 获取右手控制器位姿
    /// </summary>
    public bool GetRightControllerPose(out Vector3 positionMm, out Quaternion rotation)
    {
        positionMm = Vector3.zero;
        rotation = Quaternion.identity;

        if (vrSystem == null) return false;

        UpdateDeviceIndices();
        if (rightControllerIndex == OpenVR.k_unTrackedDeviceIndexInvalid) return false;

        vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, 0f, trackedDevicePoses);
        
        return TryGetDevicePose(rightControllerIndex, out positionMm, out rotation, out _, out _);
    }

    /// <summary>
    /// 获取控制器速度（用于抛出物体等）
    /// </summary>
    public bool GetControllerVelocity(bool isLeftHand, out Vector3 velocity, out Vector3 angularVelocity)
    {
        velocity = Vector3.zero;
        angularVelocity = Vector3.zero;

        if (vrSystem == null) return false;

        UpdateDeviceIndices();
        uint index = isLeftHand ? leftControllerIndex : rightControllerIndex;
        if (index == OpenVR.k_unTrackedDeviceIndexInvalid) return false;

        vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, 0f, trackedDevicePoses);
        
        return TryGetDevicePose(index, out _, out _, out velocity, out angularVelocity);
    }

    /// <summary>
    /// 手动触发一次位姿记录
    /// </summary>
    [ContextMenu("立即记录位姿")]
    public void LogPosesNow()
    {
        if (vrSystem != null)
        {
            UpdateDeviceIndices();
            LogDevicePoses();
        }
    }

    /// <summary>
    /// 打印当前连接的所有设备
    /// </summary>
    [ContextMenu("打印所有连接设备")]
    public void PrintAllConnectedDevices()
    {
        if (vrSystem == null)
        {
            Debug.LogWarning("[SteamVRDevicePoseLogger] VR 系统未初始化");
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

    #endregion

    #region 辅助方法

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
        if (q.w < 0f)
        {
            q.x = -q.x;
            q.y = -q.y;
            q.z = -q.z;
            q.w = -q.w;
        }
        
        return q;
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
        q = NormalizeQuaternion(q);
        
        if (q.w < 0f)
        {
            q.x = -q.x;
            q.y = -q.y;
            q.z = -q.z;
            q.w = -q.w;
        }
        
        float wClamped = Mathf.Clamp(q.w, 0f, 1f);
        float angle = 2f * Mathf.Acos(wClamped);
        
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
                axis180 = new Vector3(1f, 0f, 0f);
            }
            return axis180 * angle;
        }
        
        if (angle < 1e-6f)
        {
            return Vector3.zero;
        }
        
        float sinHalfAngle = Mathf.Sin(angle * 0.5f);
        float scale = angle / sinHalfAngle;
        return new Vector3(q.x * scale, q.y * scale, q.z * scale);
    }

    string FormatPoseLineQuaternion(string label, Vector3 positionMm, Quaternion rotationQuat)
    {
        return string.Format(Culture,
            "{0}: [{1:F2}, {2:F2}, {3:F2}, {4:F4}, {5:F4}, {6:F4}, {7:F4}] (mm, mm, mm, qx, qy, qz, qw)",
            label,
            positionMm.x, positionMm.y, positionMm.z,
            rotationQuat.x, rotationQuat.y, rotationQuat.z, rotationQuat.w);
    }

    string FormatPoseLineAxisAngle(string label, Vector3 positionMm, Vector3 rotationRad)
    {
        return string.Format(Culture,
            "{0}: [{1:F2}, {2:F2}, {3:F2}, {4:F4}, {5:F4}, {6:F4}] (mm, mm, mm, rad, rad, rad)",
            label,
            positionMm.x, positionMm.y, positionMm.z,
            rotationRad.x, rotationRad.y, rotationRad.z);
    }

    #endregion

    void OnDestroy()
    {
        // 注意：不在这里关闭 OpenVR，因为可能有其他脚本还在使用
    }
}

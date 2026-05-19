using UnityEngine;
using Valve.VR;

/// <summary>
/// VR 世界锚点校准系统
/// 
/// 功能说明：
/// - 选择一个 Tracker 作为现实世界的锚点
/// - 将该 Tracker 的位置设为 VR 追踪系统的原点
/// - 所有其他追踪设备（HMD、控制器、其他Tracker）都相对于此锚点定位
/// 
/// 使用场景：
/// - 将 VR 场景与现实房间对齐
/// - 将 Tracker 固定在机器人基座、工作台等已知位置
/// - 多次启动时保持一致的坐标系
/// 
/// 使用方法：
/// 1. 将此脚本添加到 VR 追踪系统的根对象（如 [CameraRig]）
/// 2. 将锚点 Tracker 固定在现实世界的参考位置
/// 3. 配置 anchorTrackerIndex 为锚点 Tracker 的设备索引
/// 4. 运行时按下校准键或调用 CalibrateNow()
/// </summary>
public class VRWorldAnchor : MonoBehaviour
{
    [Header("锚点 Tracker 配置")]
    [Tooltip("锚点 Tracker 的设备索引（在 SteamVR 中查看）")]
    [Range(0, 16)]
    public int anchorTrackerIndex = 3;

    [Tooltip("是否在 Start 时自动校准")]
    public bool calibrateOnStart = true;

    [Tooltip("校准快捷键")]
    public KeyCode calibrateKey = KeyCode.C;

    [Header("锚点目标位置")]
    [Tooltip("锚点 Tracker 在 Unity 世界中的目标位置")]
    public Vector3 anchorTargetPosition = Vector3.zero;

    [Tooltip("锚点 Tracker 在 Unity 世界中的目标旋转（欧拉角）")]
    public Vector3 anchorTargetRotation = Vector3.zero;

    [Tooltip("是否同时对齐旋转（否则只对齐位置）")]
    public bool alignRotation = true;

    [Tooltip("仅对齐 Y 轴旋转（水平朝向）")]
    public bool alignYawOnly = true;

    [Header("持续追踪模式")]
    [Tooltip("启用持续追踪模式（每帧更新偏移，锚点始终保持在目标位置）")]
    public bool continuousTracking = false;

    [Tooltip("持续追踪的平滑系数（越小越平滑，0=无平滑）")]
    [Range(0f, 1f)]
    public float smoothFactor = 0.1f;

    [Header("状态显示（只读）")]
    [SerializeField] private bool _isCalibrated = false;
    [SerializeField] private Vector3 _currentOffset = Vector3.zero;
    [SerializeField] private float _currentYawOffset = 0f;
    [SerializeField] private string _anchorTrackerStatus = "未校准";

    // OpenVR 设备位姿数组
    private TrackedDevicePose_t[] _devicePoses = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
    
    // 校准时记录的初始变换
    private Vector3 _calibrationOffset = Vector3.zero;
    private Quaternion _calibrationRotation = Quaternion.identity;

    void Start()
    {
        if (OpenVR.System == null)
        {
            Debug.LogError("[VRWorldAnchor] OpenVR 未初始化！");
            enabled = false;
            return;
        }

        if (calibrateOnStart)
        {
            // 延迟一帧进行校准，确保所有设备已初始化
            Invoke(nameof(CalibrateNow), 0.5f);
        }
    }

    void Update()
    {
        // 快捷键校准
        if (Input.GetKeyDown(calibrateKey))
        {
            CalibrateNow();
        }

        // 持续追踪模式
        if (continuousTracking && _isCalibrated)
        {
            UpdateContinuousTracking();
        }
    }

    /// <summary>
    /// 立即执行校准
    /// </summary>
    [ContextMenu("立即校准")]
    public void CalibrateNow()
    {
        if (OpenVR.System == null)
        {
            Debug.LogError("[VRWorldAnchor] OpenVR 未初始化！");
            return;
        }

        // 获取锚点 Tracker 的当前位姿
        Vector3 anchorPosition;
        Quaternion anchorRotation;
        
        if (!GetTrackerPose(anchorTrackerIndex, out anchorPosition, out anchorRotation))
        {
            Debug.LogError($"[VRWorldAnchor] 无法获取锚点 Tracker (Device{anchorTrackerIndex}) 的位姿！请检查设备是否已连接。");
            _anchorTrackerStatus = "设备未连接";
            return;
        }

        // 计算位置偏移：使锚点移动到目标位置
        Quaternion targetRotation = Quaternion.Euler(anchorTargetRotation);
        
        if (alignRotation)
        {
            if (alignYawOnly)
            {
                // 仅对齐 Y 轴旋转
                float anchorYaw = anchorRotation.eulerAngles.y;
                float targetYaw = anchorTargetRotation.y;
                float yawOffset = targetYaw - anchorYaw;
                
                _calibrationRotation = Quaternion.Euler(0, yawOffset, 0);
                _currentYawOffset = yawOffset;
            }
            else
            {
                // 完全对齐旋转
                _calibrationRotation = targetRotation * Quaternion.Inverse(anchorRotation);
            }
        }
        else
        {
            _calibrationRotation = Quaternion.identity;
            _currentYawOffset = 0f;
        }

        // 应用旋转后计算位置偏移
        Vector3 rotatedAnchorPosition = _calibrationRotation * anchorPosition;
        _calibrationOffset = anchorTargetPosition - rotatedAnchorPosition;
        _currentOffset = _calibrationOffset;

        // 应用变换到 VR 根对象
        ApplyCalibration();

        _isCalibrated = true;
        _anchorTrackerStatus = $"已校准 (Device{anchorTrackerIndex})";
        
        Debug.Log($"<color=green>[VRWorldAnchor] 校准完成！</color>\n" +
                  $"  锚点 Tracker: Device{anchorTrackerIndex}\n" +
                  $"  位置偏移: {_calibrationOffset}\n" +
                  $"  旋转偏移: {_calibrationRotation.eulerAngles}");
    }

    /// <summary>
    /// 应用校准变换
    /// </summary>
    void ApplyCalibration()
    {
        // 先应用旋转（绕原点旋转）
        transform.rotation = _calibrationRotation * Quaternion.identity;
        
        // 再应用位置偏移
        transform.position = _calibrationOffset;
    }

    /// <summary>
    /// 持续追踪更新（每帧微调偏移）
    /// </summary>
    void UpdateContinuousTracking()
    {
        Vector3 anchorPosition;
        Quaternion anchorRotation;
        
        if (!GetTrackerPose(anchorTrackerIndex, out anchorPosition, out anchorRotation))
        {
            return;
        }

        Quaternion targetRotation = Quaternion.Euler(anchorTargetRotation);
        Quaternion newRotation;
        
        if (alignRotation)
        {
            if (alignYawOnly)
            {
                float anchorYaw = anchorRotation.eulerAngles.y;
                float targetYaw = anchorTargetRotation.y;
                float yawOffset = targetYaw - anchorYaw;
                newRotation = Quaternion.Euler(0, yawOffset, 0);
            }
            else
            {
                newRotation = targetRotation * Quaternion.Inverse(anchorRotation);
            }
        }
        else
        {
            newRotation = Quaternion.identity;
        }

        Vector3 rotatedAnchorPosition = newRotation * anchorPosition;
        Vector3 newOffset = anchorTargetPosition - rotatedAnchorPosition;

        // 平滑过渡
        if (smoothFactor > 0)
        {
            _calibrationOffset = Vector3.Lerp(_calibrationOffset, newOffset, smoothFactor);
            _calibrationRotation = Quaternion.Slerp(_calibrationRotation, newRotation, smoothFactor);
        }
        else
        {
            _calibrationOffset = newOffset;
            _calibrationRotation = newRotation;
        }

        _currentOffset = _calibrationOffset;
        ApplyCalibration();
    }

    /// <summary>
    /// 获取指定 Tracker 的位姿
    /// </summary>
    bool GetTrackerPose(int deviceIndex, out Vector3 position, out Quaternion rotation)
    {
        position = Vector3.zero;
        rotation = Quaternion.identity;

        if (deviceIndex < 0 || deviceIndex >= OpenVR.k_unMaxTrackedDeviceCount)
            return false;

        // 获取最新位姿
        OpenVR.System.GetDeviceToAbsoluteTrackingPose(
            ETrackingUniverseOrigin.TrackingUniverseStanding,
            0f,
            _devicePoses
        );

        var pose = _devicePoses[deviceIndex];
        if (!pose.bDeviceIsConnected || !pose.bPoseIsValid)
            return false;

        var mat = pose.mDeviceToAbsoluteTracking;
        position = new Vector3(mat.m3, mat.m7, mat.m11);
        rotation = GetRotationFromMatrix(mat);

        return true;
    }

    /// <summary>
    /// 从 HmdMatrix34_t 提取旋转
    /// </summary>
    Quaternion GetRotationFromMatrix(HmdMatrix34_t mat)
    {
        float m00 = mat.m0, m01 = mat.m1, m02 = mat.m2;
        float m10 = mat.m4, m11 = mat.m5, m12 = mat.m6;
        float m20 = mat.m8, m21 = mat.m9, m22 = mat.m10;
        
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
        else if (m00 > m11 && m00 > m22)
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
        
        return q.normalized;
    }

    /// <summary>
    /// 重置校准（恢复到原始状态）
    /// </summary>
    [ContextMenu("重置校准")]
    public void ResetCalibration()
    {
        transform.position = Vector3.zero;
        transform.rotation = Quaternion.identity;
        
        _calibrationOffset = Vector3.zero;
        _calibrationRotation = Quaternion.identity;
        _currentOffset = Vector3.zero;
        _currentYawOffset = 0f;
        _isCalibrated = false;
        _anchorTrackerStatus = "未校准";
        
        Debug.Log("[VRWorldAnchor] 校准已重置");
    }

    /// <summary>
    /// 打印当前所有设备状态
    /// </summary>
    [ContextMenu("打印设备列表")]
    public void PrintDeviceList()
    {
        if (OpenVR.System == null)
        {
            Debug.LogError("[VRWorldAnchor] OpenVR 未初始化");
            return;
        }

        OpenVR.System.GetDeviceToAbsoluteTrackingPose(
            ETrackingUniverseOrigin.TrackingUniverseStanding,
            0f,
            _devicePoses
        );

        Debug.Log("<color=cyan>[VRWorldAnchor] 已连接的设备列表:</color>");
        
        for (int i = 0; i < _devicePoses.Length; i++)
        {
            if (_devicePoses[i].bDeviceIsConnected)
            {
                var deviceClass = OpenVR.System.GetTrackedDeviceClass((uint)i);
                string className = GetDeviceClassName(deviceClass);
                bool poseValid = _devicePoses[i].bPoseIsValid;
                
                string marker = (i == anchorTrackerIndex) ? " ← 锚点" : "";
                Debug.Log($"  Device{i}: {className} | 位姿有效: {poseValid}{marker}");
            }
        }
    }

    string GetDeviceClassName(ETrackedDeviceClass deviceClass)
    {
        switch (deviceClass)
        {
            case ETrackedDeviceClass.HMD: return "头显(HMD)";
            case ETrackedDeviceClass.Controller: return "控制器";
            case ETrackedDeviceClass.GenericTracker: return "Tracker";
            case ETrackedDeviceClass.TrackingReference: return "基站";
            default: return $"未知({deviceClass})";
        }
    }

    /// <summary>
    /// 获取校准状态
    /// </summary>
    public bool IsCalibrated => _isCalibrated;

    /// <summary>
    /// 获取当前偏移量
    /// </summary>
    public Vector3 CurrentOffset => _currentOffset;

    /// <summary>
    /// 获取当前旋转偏移
    /// </summary>
    public Quaternion CalibrationRotation => _calibrationRotation;

    void OnDrawGizmos()
    {
        // 绘制目标锚点位置
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(anchorTargetPosition, 0.1f);
        
        // 绘制目标朝向
        Quaternion targetRot = Quaternion.Euler(anchorTargetRotation);
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(anchorTargetPosition, targetRot * Vector3.forward * 0.3f);
        Gizmos.color = Color.red;
        Gizmos.DrawRay(anchorTargetPosition, targetRot * Vector3.right * 0.2f);
        Gizmos.color = Color.green;
        Gizmos.DrawRay(anchorTargetPosition, targetRot * Vector3.up * 0.2f);
    }
}

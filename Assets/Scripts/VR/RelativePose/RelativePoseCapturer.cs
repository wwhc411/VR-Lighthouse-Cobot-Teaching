using UnityEngine;
using HandEyeCalibration;
using Valve.VR;

/// <summary>
/// RelativePoseCapturer: 相对位姿数据捕获器
/// 
/// 功能说明：
/// - 从 SteamVR_RelativePoseMonitor 获取 Moving Tracker 在 Reference Tracker 坐标系下的位姿
/// - 将位姿数据传递给手眼标定 UI 进行处理
/// 
/// Tracker 角色定义：
/// - Reference Tracker（参考锚点）：场景中固定放置的设备，定义局部坐标系（≈相机坐标系）
/// - Moving Tracker（移动靶点）：场景中移动的目标设备
/// 
/// 输出数据：
/// - Moving Tracker 在 Reference Tracker 坐标系下的位置和旋转
/// - 类比关系：Reference Tracker 坐标系 ≈ 绝对位姿模式中的相机坐标系
/// 
/// 使用方法：
/// 1. 在场景中创建 GameObject 并添加此组件
/// 2. 配置 relativePoseMonitor（拖入 SteamVR_RelativePoseMonitor）
/// 3. 配置 handEyeCalibrationUI（拖入 HandEyeCalibrationUI）
/// 4. 调用 CaptureRelativePose() 方法采集数据
/// </summary>
public class RelativePoseCapturer : MonoBehaviour
{
    [Header("组件引用")]
    [Tooltip("相对位姿监控器（用于采集相对位姿数据）")]
    public SteamVR_RelativePoseMonitor relativePoseMonitor;

    [Tooltip("手眼标定 UI 管理器（用于接收标定数据）")]
    public HandEyeCalibrationUI handEyeCalibrationUI;

    [Header("调试设置")]
    [Tooltip("是否输出详细日志")]
    public bool verboseLogging = true;

    [Header("状态显示 (只读)")]
    [SerializeField] private bool _isReady = false;
    [SerializeField] private int _captureCount = 0;
    [SerializeField] private Vector3 _lastMovingPoseInReference;
    [SerializeField] private Quaternion _lastMovingRotationInReference;

    // ==================== 公共属性 ====================

    /// <summary>
    /// 是否准备就绪（所有组件已配置）
    /// </summary>
    public bool IsReady => relativePoseMonitor != null && handEyeCalibrationUI != null;

    /// <summary>
    /// 已捕获的数据点数量
    /// </summary>
    public int CaptureCount => _captureCount;

    /// <summary>
    /// 最后捕获的 Moving Tracker 在 Reference 坐标系下的位置
    /// </summary>
    public Vector3 LastMovingPoseInReference => _lastMovingPoseInReference;

    /// <summary>
    /// 最后捕获的 Moving Tracker 在 Reference 坐标系下的旋转
    /// </summary>
    public Quaternion LastMovingRotationInReference => _lastMovingRotationInReference;

    // ==================== 生命周期 ====================

    void Start()
    {
        ValidateComponents();
    }

    void Update()
    {
        _isReady = IsReady;
    }

    // ==================== 公开接口 ====================

    /// <summary>
    /// 捕获 Moving Tracker 在 Reference Tracker 坐标系下的位姿并发送到手眼标定 UI
    /// </summary>
    /// <returns>是否成功捕获</returns>
    [ContextMenu("捕获相对位姿")]
    public bool CaptureRelativePose()
    {
        // 验证组件
        if (!ValidateComponents())
        {
            return false;
        }

        // 获取 Moving Tracker 在 Reference 坐标系下的位姿
        Vector3 movingPosition;
        Quaternion movingRotation;
        if (!relativePoseMonitor.GetRelativePose(out movingPosition, out movingRotation))
        {
            Debug.LogError("[RelativePoseCapturer] 无法获取相对位姿数据，请检查 RelativePoseMonitor 配置！");
            return false;
        }

        // 保存最后捕获的数据
        _lastMovingPoseInReference = movingPosition;
        _lastMovingRotationInReference = movingRotation;
        _captureCount++;

        // 调用手眼标定UI的相对位姿数据采集方法
        handEyeCalibrationUI.CaptureCalibrationDataWithRelativePose(movingPosition, movingRotation);

        if (verboseLogging)
        {
            Debug.Log($"<color=green>[RelativePoseCapturer] 数据已捕获 (第 {_captureCount} 次)\n" +
                      $"  Moving Tracker 在 Reference 坐标系下的位置(m): ({movingPosition.x:F4}, {movingPosition.y:F4}, {movingPosition.z:F4})\n" +
                      $"  Moving Tracker 在 Reference 坐标系下的旋转: (x:{movingRotation.x:F4}, y:{movingRotation.y:F4}, z:{movingRotation.z:F4}, w:{movingRotation.w:F4})</color>");
        }

        return true;
    }

    /// <summary>
    /// 获取当前 Moving Tracker 在 Reference 坐标系下的位姿（不发送到标定 UI）
    /// </summary>
    /// <param name="movingPosition">输出：Moving Tracker 在 Reference 坐标系下的位置（米）</param>
    /// <param name="movingRotation">输出：Moving Tracker 在 Reference 坐标系下的旋转（四元数）</param>
    /// <returns>是否成功获取</returns>
    public bool GetCurrentRelativePose(out Vector3 movingPosition, out Quaternion movingRotation)
    {
        movingPosition = Vector3.zero;
        movingRotation = Quaternion.identity;

        if (relativePoseMonitor == null)
        {
            Debug.LogError("[RelativePoseCapturer] RelativePoseMonitor 未设置！");
            return false;
        }

        return relativePoseMonitor.GetRelativePose(out movingPosition, out movingRotation);
    }

    /// <summary>
    /// 获取当前 Moving Tracker 在 Reference 坐标系下的位姿矩阵
    /// </summary>
    /// <param name="poseMatrix">输出：T_moving_in_reference 变换矩阵</param>
    /// <returns>是否成功获取</returns>
    public bool GetCurrentRelativePoseMatrix(out Matrix4x4 poseMatrix)
    {
        poseMatrix = Matrix4x4.identity;

        if (relativePoseMonitor == null)
        {
            Debug.LogError("[RelativePoseCapturer] RelativePoseMonitor 未设置！");
            return false;
        }

        return relativePoseMonitor.GetRelativePoseMatrix(out poseMatrix);
    }

    /// <summary>
    /// 重置捕获计数
    /// </summary>
    [ContextMenu("重置计数")]
    public void ResetCaptureCount()
    {
        _captureCount = 0;
        _lastMovingPoseInReference = Vector3.zero;
        _lastMovingRotationInReference = Quaternion.identity;
        Debug.Log("[RelativePoseCapturer] 捕获计数已重置");
    }

    // ==================== 内部方法 ====================

    /// <summary>
    /// 验证必要组件是否已配置
    /// </summary>
    private bool ValidateComponents()
    {
        bool isValid = true;

        if (relativePoseMonitor == null)
        {
            Debug.LogError("[RelativePoseCapturer] RelativePoseMonitor 未设置，请在 Inspector 中拖入 SteamVR_RelativePoseMonitor 组件！");
            isValid = false;
        }

        if (handEyeCalibrationUI == null)
        {
            Debug.LogError("[RelativePoseCapturer] HandEyeCalibrationUI 未设置，请在 Inspector 中拖入 HandEyeCalibrationUI 组件！");
            isValid = false;
        }

        return isValid;
    }

    // ==================== 调试功能 ====================

    /// <summary>
    /// 打印当前 Moving Tracker 在 Reference 坐标系下的位姿
    /// </summary>
    [ContextMenu("打印当前相对位姿")]
    public void PrintCurrentRelativePose()
    {
        if (GetCurrentRelativePose(out Vector3 pos, out Quaternion rot))
        {
            Vector3 euler = rot.eulerAngles;
            Debug.Log($"[RelativePoseCapturer] Moving Tracker 在 Reference 坐标系下的位姿:\n" +
                      $"  位置(m): ({pos.x:F4}, {pos.y:F4}, {pos.z:F4})\n" +
                      $"  位置(mm): ({pos.x * 1000f:F2}, {pos.y * 1000f:F2}, {pos.z * 1000f:F2})\n" +
                      $"  旋转(四元数): (x:{rot.x:F4}, y:{rot.y:F4}, z:{rot.z:F4}, w:{rot.w:F4})\n" +
                      $"  旋转(欧拉角°): ({euler.x:F2}, {euler.y:F2}, {euler.z:F2})");
        }
        else
        {
            Debug.LogWarning("[RelativePoseCapturer] 无法获取当前相对位姿");
        }
    }
}

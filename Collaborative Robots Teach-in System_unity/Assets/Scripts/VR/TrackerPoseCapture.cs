using UnityEngine;
using handeye;
using Valve.VR;

/// <summary>
/// 位姿数据源类型
/// </summary>
public enum PoseDataSource
{
    /// <summary>绝对位姿 - 使用 ViveTrackerPoseLogger (SteamVR世界坐标系)</summary>
    AbsolutePose,
    
    /// <summary>相对位姿 - 使用 SteamVR_RelativePoseMonitor (Reference Tracker局部坐标系)</summary>
    RelativePose
}

/// <summary>
/// Tracker 位姿快速捕获与执行工具
/// 
/// 功能:
/// - 一键捕获 Tracker 设备的当前位姿（支持绝对位姿和相对位姿）
/// - 自动转换为 UR 基座坐标系 (mm, rad)
/// - 直接调用控制接口执行 MoveL 命令
/// - 完全替代手动输入数据的过程
/// 
/// 使用方法:
/// 1. 在 Inspector 中选择数据源（绝对位姿 或 相对位姿）
/// 2. 配置对应的组件引用
/// 3. 按空格键捕获位姿并执行控制
/// </summary>
public class TrackerPoseCapture : MonoBehaviour
{
    // ==================== 数据源配置 ====================
    
    [Header("数据源选择")]
    [Tooltip("数据源类型：绝对位姿使用手眼标定转换，相对位姿直接使用")]
    public PoseDataSource dataSource = PoseDataSource.AbsolutePose;
    
    // ==================== 绝对位姿配置 ====================
    
    [Header("绝对位姿配置")]
    [Tooltip("要捕获的 Tracker 设备 ID (绝对位姿模式)")]
    public uint trackerDeviceId = 2;
    
    [Tooltip("ViveTrackerPoseLogger 组件引用")]
    public ViveTrackerPoseLogger poseLogger;
    
    // ==================== 相对位姿配置 ====================
    
    [Header("相对位姿配置")]
    [Tooltip("SteamVR_RelativePoseMonitor 组件引用")]
    public SteamVR_RelativePoseMonitor relativePoseMonitor;
    
    [Tooltip("要捕获的 Moving Tracker 索引（在 SteamVR_RelativePoseMonitor.movingTrackerIndices 中的索引，从0开始）\n-1 表示使用第一个 Moving Tracker")]
    public int movingTrackerArrayIndex = 0;

    [Header("组件引用")]
    [Tooltip("Main UI Control 组件引用")]
    public main_ui_control uiControl;

    [Header("显示选项")]
    [Tooltip("是否显示详细的调试信息")]
    public bool verboseOutput = true;

    [Tooltip("是否自动查找组件")]
    public bool autoFindComponents = true;

    [Header("位姿偏移配置")]
    [Tooltip("是否启用位置偏移（沿 Tracker 坐标系 Z 轴正向平移）")]
    public bool enablePositionOffset = false;
    
    [Tooltip("沿 Tracker Z 轴正向的偏移距离 (mm)，正值向前，负值向后")]
    public float zAxisOffsetMm = 100f;

    [Header("运动参数配置（可选）")]
    [Tooltip("是否使用自定义加速度（取消勾选则使用 UI 默认值）")]
    public bool useCustomAcceleration = false;
    
    [Tooltip("自定义加速度 (m/s²)，范围: 0.1 ~ 1.5")]
    [Range(0.1f, 1.5f)]
    public float customAcceleration = 0.5f;

    [Tooltip("是否使用自定义线速度（取消勾选则使用 UI 默认值）")]
    public bool useCustomLinearSpeed = false;
    
    [Tooltip("自定义线速度 (m/s)，范围: 0.01 ~ 0.5")]
    [Range(0.01f, 0.5f)]
    public float customLinearSpeed = 0.1f;

    [Tooltip("是否使用自定义混合半径（取消勾选则使用 UI 默认值）")]
    public bool useCustomBlendRadius = false;
    
    [Tooltip("自定义混合半径 (m)，范围: 0 ~ 0.1")]
    [Range(0f, 0.1f)]
    public float customBlendRadius = 0.0f;

    void Start()
    {
        // 自动查找组件
        if (autoFindComponents)
        {
            if (poseLogger == null)
            {
                poseLogger = FindObjectOfType<ViveTrackerPoseLogger>();
                if (poseLogger == null && dataSource == PoseDataSource.AbsolutePose)
                {
                    Debug.LogWarning("[TrackerCapture] 未找到 ViveTrackerPoseLogger 组件（绝对位姿模式需要）");
                }
            }
            
            if (relativePoseMonitor == null)
            {
                relativePoseMonitor = FindObjectOfType<SteamVR_RelativePoseMonitor>();
                if (relativePoseMonitor == null && dataSource == PoseDataSource.RelativePose)
                {
                    Debug.LogWarning("[TrackerCapture] 未找到 SteamVR_RelativePoseMonitor 组件（相对位姿模式需要）");
                }
            }
            
            if (uiControl == null)
            {
                uiControl = FindObjectOfType<main_ui_control>();
                if (uiControl == null)
                {
                    Debug.LogError("[TrackerCapture] 未找到 main_ui_control 组件！无法执行控制命令。");
                }
            }
        }

        // 验证配置
        ValidateConfiguration();
        
        if (uiControl != null)
        {
            Debug.Log($"<color=green>[TrackerCapture] 初始化完成</color>");
            Debug.Log($"  数据源: {dataSource}");
            
            if (dataSource == PoseDataSource.AbsolutePose)
            {
                Debug.Log($"  Tracker ID: {trackerDeviceId}");
                Debug.Log($"  坐标转换: SteamVR世界坐标系 → UR基座坐标系 (手眼标定)");
            }
            else
            {
                Debug.Log($"  坐标系: Reference Tracker 局部坐标系");
                Debug.Log($"  坐标转换: Reference局部坐标系 → UR基座坐标系 (需验证)");
            }
            
            Debug.Log($"  快捷键: 空格键 - 捕获并执行");
        }
    }

    /// <summary>
    /// OnDisable: 停止所有协程，防止退出 Play 模式时崩溃
    /// </summary>
    void OnDisable()
    {
        // 停止所有正在运行的协程
        StopAllCoroutines();
    }

    /// <summary>
    /// 捕获当前 Tracker 位姿并执行 MoveL 控制
    /// </summary>
    [ContextMenu("捕获并执行控制")]
    public void CaptureAndExecute()
    {
        if (uiControl == null)
        {
            Debug.LogError("[TrackerCapture] UI Control 未设置！无法执行控制命令。");
            return;
        }

        // 根据数据源获取位姿数据
        Vector3 positionMm;
        Quaternion rotation;
        
        if (!GetPoseData(out positionMm, out rotation))
        {
            return;
        }

        // 转换四元数为旋转矢量 (轴角表示, 弧度)
        Vector3 rotationVectorRad = QuaternionToRotationVector(rotation);

        // 应用位置偏移（如果启用）
        if (enablePositionOffset)
        {
            positionMm = ApplyPositionOffset(positionMm, rotation, zAxisOffsetMm);
            
            if (verboseOutput)
            {
                Debug.Log($"<color=yellow>【位置偏移】{dataSource} 模式</color>");
                Debug.Log($"  偏移方向: Tracker 坐标系 Z 轴正向");
                Debug.Log($"  偏移距离: {zAxisOffsetMm:F2} mm");
                Debug.Log($"  偏移后位置 (mm): ({positionMm.x:F2}, {positionMm.y:F2}, {positionMm.z:F2})");
            }
        }

        if (verboseOutput)
        {
            Debug.Log($"========== Tracker 位姿捕获 ({dataSource}) ==========");
            Debug.Log($"<color=cyan>【步骤1: 捕获原始数据】</color>");
            Debug.Log($"  数据源: {dataSource}");
            
            if (dataSource == PoseDataSource.AbsolutePose)
            {
                Debug.Log($"  设备 ID: {trackerDeviceId}");
                Debug.Log($"  坐标系: SteamVR 世界坐标系");
            }
            else
            {
                Debug.Log($"  坐标系: Reference Tracker 局部坐标系");
            }
            
            Debug.Log($"  位置 (mm): ({positionMm.x:F2}, {positionMm.y:F2}, {positionMm.z:F2})");
            Debug.Log($"  旋转 (四元数): (x:{rotation.x:F4}, y:{rotation.y:F4}, z:{rotation.z:F4}, w:{rotation.w:F4})");
            Debug.Log($"  旋转 (轴角 rad): ({rotationVectorRad.x:F4}, {rotationVectorRad.y:F4}, {rotationVectorRad.z:F4})");
            Debug.Log($"  旋转角度: {rotationVectorRad.magnitude * Mathf.Rad2Deg:F2}°");
            if (enablePositionOffset)
            {
                Debug.Log($"  <color=cyan>已应用偏移: Z 轴 +{zAxisOffsetMm:F2} mm</color>");
            }
        }

        // 填充到 MoveL 输入框并执行（模拟手动输入）
        // 注意：位置保持 mm 单位，因为 moveL_inputIsSteamVrToggle 勾选后期望输入为 mm
        FillMoveLInputsAndExecute(positionMm, rotationVectorRad);
    }

    /// <summary>
    /// 填充 MoveL 输入框并执行控制（模拟手动 UI 输入流程）
    /// 
    /// 重要说明:
    /// - 绝对位姿模式: 勾选 SteamVR Toggle → 应用手眼标定转换 T_cam2base
    /// - 相对位姿模式: 不勾选 SteamVR Toggle → 直接作为 UR 坐标使用
    ///   (前提: Reference Tracker 已与 UR 基座坐标系对齐)
    /// </summary>
    /// <param name="positionMm">位置（单位：毫米）</param>
    /// <param name="rotationRad">旋转矢量（单位：弧度）</param>
    void FillMoveLInputsAndExecute(Vector3 positionMm, Vector3 rotationRad)
    {
        // 获取 MoveL 输入框组件
        var moveL_xInput = uiControl.moveL_xInput;
        var moveL_yInput = uiControl.moveL_yInput;
        var moveL_zInput = uiControl.moveL_zInput;
        var moveL_rxInput = uiControl.moveL_rxInput;
        var moveL_ryInput = uiControl.moveL_ryInput;
        var moveL_rzInput = uiControl.moveL_rzInput;
        var moveL_inputIsSteamVrToggle = uiControl.moveL_inputIsSteamVrToggle;
        
        // 获取运动参数输入框
        var moveL_accelerationInput = uiControl.moveL_accelerationInput;
        var moveL_linearSpeedInput = uiControl.moveL_linearSpeedInput;
        var moveL_blendRadiusInput = uiControl.moveL_blendRadiusInput;

        if (moveL_xInput == null || moveL_yInput == null || moveL_zInput == null ||
            moveL_rxInput == null || moveL_ryInput == null || moveL_rzInput == null)
        {
            Debug.LogError("[TrackerCapture] MoveL 输入框未找到！请检查 main_ui_control 的 Inspector 配置。");
            return;
        }

        if (verboseOutput)
        {
            Debug.Log($"\n<color=yellow>【步骤2: 填充 UI 输入框】</color>");
        }

        // 根据数据源类型选择填充方式
        if (dataSource == PoseDataSource.RelativePose)
        {
            // ========== 相对位姿模式 ==========
            // 假设: Reference Tracker 已与 UR 基座坐标系对齐
            // 策略: 不勾选 SteamVR Toggle，直接作为 UR 坐标使用
            
            // 位置单位转换: mm → m (UR 期望单位为米)
            Vector3 positionM = positionMm / 1000f;
            
            moveL_xInput.text = positionM.x.ToString("F4", System.Globalization.CultureInfo.InvariantCulture);
            moveL_yInput.text = positionM.y.ToString("F4", System.Globalization.CultureInfo.InvariantCulture);
            moveL_zInput.text = positionM.z.ToString("F4", System.Globalization.CultureInfo.InvariantCulture);
            
            if (verboseOutput)
            {
                Debug.Log($"  <color=cyan>【相对位姿模式】</color>");
                Debug.Log($"  策略: 跳过手眼标定转换，直接作为 UR 基座坐标");
                Debug.Log($"  前提: Reference Tracker 已与 UR 基座对齐");
                Debug.Log($"  填充位置 (m): [{positionM.x:F4}, {positionM.y:F4}, {positionM.z:F4}]");
                Debug.Log($"  <color=yellow>⚠️ 警告: 如 Reference 未对齐，机器人运动将不正确！</color>");
            }
            
            // 不勾选 SteamVR Toggle (关键！)
            if (moveL_inputIsSteamVrToggle != null)
            {
                moveL_inputIsSteamVrToggle.isOn = false;
                if (verboseOutput)
                {
                    Debug.Log($"  SteamVR Toggle: <color=red>未勾选</color> → 跳过坐标转换");
                }
            }
        }
        else
        {
            // ========== 绝对位姿模式 ==========
            // 策略: 勾选 SteamVR Toggle，应用手眼标定转换
            
            moveL_xInput.text = positionMm.x.ToString("F2", System.Globalization.CultureInfo.InvariantCulture);
            moveL_yInput.text = positionMm.y.ToString("F2", System.Globalization.CultureInfo.InvariantCulture);
            moveL_zInput.text = positionMm.z.ToString("F2", System.Globalization.CultureInfo.InvariantCulture);
            
            if (verboseOutput)
            {
                Debug.Log($"  <color=cyan>【绝对位姿模式】</color>");
                Debug.Log($"  策略: 应用手眼标定转换 T_cam2base");
                Debug.Log($"  填充位置 (mm): [{positionMm.x:F2}, {positionMm.y:F2}, {positionMm.z:F2}]");
                Debug.Log($"  <color=green>勾选 SteamVR Toggle 后，UI 期望位置单位为 mm</color>");
            }
            
            // 勾选 "输入为 SteamVR 坐标" Toggle（触发坐标转换）
            if (moveL_inputIsSteamVrToggle != null)
            {
                moveL_inputIsSteamVrToggle.isOn = true;
                if (verboseOutput)
                {
                    Debug.Log($"  SteamVR Toggle: <color=green>已勾选</color> → 应用手眼标定转换");
                }
            }
            else
            {
                Debug.LogWarning("[TrackerCapture] moveL_inputIsSteamVrToggle 未找到，将直接使用输入值（不转换）");
            }
        }

        // 填充旋转输入框（两种模式相同，都是弧度）
        moveL_rxInput.text = rotationRad.x.ToString("F4", System.Globalization.CultureInfo.InvariantCulture);
        moveL_ryInput.text = rotationRad.y.ToString("F4", System.Globalization.CultureInfo.InvariantCulture);
        moveL_rzInput.text = rotationRad.z.ToString("F4", System.Globalization.CultureInfo.InvariantCulture);

        // 填充可选的运动参数
        if (useCustomAcceleration && moveL_accelerationInput != null)
        {
            moveL_accelerationInput.text = customAcceleration.ToString("F3", System.Globalization.CultureInfo.InvariantCulture);
            if (verboseOutput)
            {
                Debug.Log($"  <color=cyan>自定义加速度: {customAcceleration:F3} m/s²</color>");
            }
        }
        else if (verboseOutput)
        {
            Debug.Log($"  加速度: 使用 UI 默认值");
        }

        if (useCustomLinearSpeed && moveL_linearSpeedInput != null)
        {
            moveL_linearSpeedInput.text = customLinearSpeed.ToString("F3", System.Globalization.CultureInfo.InvariantCulture);
            if (verboseOutput)
            {
                Debug.Log($"  <color=cyan>自定义线速度: {customLinearSpeed:F3} m/s</color>");
            }
        }
        else if (verboseOutput)
        {
            Debug.Log($"  线速度: 使用 UI 默认值");
        }

        if (useCustomBlendRadius && moveL_blendRadiusInput != null)
        {
            moveL_blendRadiusInput.text = customBlendRadius.ToString("F3", System.Globalization.CultureInfo.InvariantCulture);
            if (verboseOutput)
            {
                Debug.Log($"  <color=cyan>自定义混合半径: {customBlendRadius:F3} m</color>");
            }
        }
        else if (verboseOutput)
        {
            Debug.Log($"  混合半径: 使用 UI 默认值");
        }

        if (verboseOutput)
        {
            Debug.Log($"\n<color=cyan>【步骤3: 调用 UI 控制流程】</color>");
        }

        // 调用 UI Control 的标准流程
        uiControl.BuildAndSetMoveLFromInputs();

        // 触发执行脉冲
        StartCoroutine(ExecutePulseCoroutine(0.1f));

        if (verboseOutput)
        {
            if (dataSource == PoseDataSource.RelativePose)
            {
                Debug.Log($"<color=green>✓ 已调用 BuildAndSetMoveLFromInputs() - 相对位姿直接使用</color>");
            }
            else
            {
                Debug.Log($"<color=green>✓ 已调用 BuildAndSetMoveLFromInputs() - 坐标转换由 UI Control 处理</color>");
            }
            Debug.Log($"<color=green>✓ 执行脉冲已触发</color>");
            Debug.Log("==========================================\n");
        }
    }

    /// <summary>
    /// 执行脉冲协程 (激活手动发送标志)
    /// </summary>
    private System.Collections.IEnumerator ExecutePulseCoroutine(float seconds)
    {
        ur_data_processing.UR_Control_Data.manual_send_active = true;
        yield return new WaitForSeconds(seconds);
        ur_data_processing.UR_Control_Data.manual_send_active = false;
    }

    /// <summary>
    /// 仅捕获位姿，不执行控制 (用于调试)
    /// </summary>
    [ContextMenu("仅捕获位姿 (不执行)")]
    public void CaptureOnly()
    {
        if (poseLogger == null)
        {
            Debug.LogError("[TrackerCapture] PoseLogger 未设置！");
            return;
        }

        if (!poseLogger.GetTrackerPoseForCalibration(trackerDeviceId, out Vector3 positionMm, out Quaternion rotation))
        {
            Debug.LogError($"[TrackerCapture] 无法获取 Tracker[ID:{trackerDeviceId}] 的位姿数据！");
            return;
        }

        Vector3 originalPositionMm = positionMm;  // 保存原始位置
        Vector3 rotationVectorRad = QuaternionToRotationVector(rotation);

        // 应用位置偏移（如果启用）
        if (enablePositionOffset)
        {
            positionMm = ApplyPositionOffset(positionMm, rotation, zAxisOffsetMm);
        }

        // 执行坐标变换查看结果
        SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
            positionMm,
            rotationVectorRad,
            posInMillimeters: true,
            out Vector3 outputPositionM,
            out Vector3 outputRotationRad
        );

        Vector3 outputPositionMm = outputPositionM * 1000f;

        Debug.Log($"========== Tracker[ID:{trackerDeviceId}] 位姿 ==========");
        Debug.Log($"<color=cyan>【原始 - SteamVR】</color>");
        Debug.Log($"  位置 (mm): [{originalPositionMm.x:F2}, {originalPositionMm.y:F2}, {originalPositionMm.z:F2}]");
        if (enablePositionOffset)
        {
            Debug.Log($"<color=yellow>【偏移应用】</color>");
            Debug.Log($"  偏移距离: Z 轴 +{zAxisOffsetMm:F2} mm");
            Debug.Log($"  偏移后位置 (mm): [{positionMm.x:F2}, {positionMm.y:F2}, {positionMm.z:F2}]");
        }
        Debug.Log($"  旋转 (rad): [{rotationVectorRad.x:F4}, {rotationVectorRad.y:F4}, {rotationVectorRad.z:F4}]");
        Debug.Log($"  角度: {rotationVectorRad.magnitude * Mathf.Rad2Deg:F2}°");
        Debug.Log($"\n<color=yellow>【变换后 - UR Base】</color>");
        Debug.Log($"  位置 (m): [{outputPositionM.x:F4}, {outputPositionM.y:F4}, {outputPositionM.z:F4}]");
        Debug.Log($"  位置 (mm): [{outputPositionMm.x:F2}, {outputPositionMm.y:F2}, {outputPositionMm.z:F2}]");
        Debug.Log($"  旋转 (rad): [{outputRotationRad.x:F4}, {outputRotationRad.y:F4}, {outputRotationRad.z:F4}]");
        Debug.Log($"  角度: {outputRotationRad.magnitude * Mathf.Rad2Deg:F2}°");
        Debug.Log("================================================\n");
    }

    /// <summary>
    /// 列出所有可用的 Tracker 设备
    /// </summary>
    [ContextMenu("列出所有 Tracker")]
    public void ListAllTrackers()
    {
        if (poseLogger == null)
        {
            Debug.LogError("[TrackerCapture] PoseLogger 未设置！");
            return;
        }

        var trackers = poseLogger.GetAllTrackerPoses();
        
        if (trackers.Count == 0)
        {
            Debug.LogWarning("[TrackerCapture] 未找到任何 Tracker 设备！");
            Debug.LogWarning("  请确认: 1) SteamVR 已启动  2) Tracker 已连接  3) Tracker 已配对");
            return;
        }

        Debug.Log($"========== 发现 {trackers.Count} 个 Tracker 设备 ==========");
        
        foreach (var kvp in trackers)
        {
            uint deviceId = kvp.Key;
            Vector3 position = kvp.Value.position;
            Quaternion rotation = kvp.Value.rotation;
            Vector3 rotationVector = QuaternionToRotationVector(rotation);
            
            Debug.Log($"\n<color=cyan>● Tracker ID: {deviceId}</color>");
            Debug.Log($"  位置 (mm): ({position.x:F2}, {position.y:F2}, {position.z:F2})");
            Debug.Log($"  旋转 (rad): ({rotationVector.x:F4}, {rotationVector.y:F4}, {rotationVector.z:F4})");
            Debug.Log($"  角度: {rotationVector.magnitude * Mathf.Rad2Deg:F2}°");
        }
        
        Debug.Log("\n=======================================\n");
    }

    /// <summary>
    /// 应用位置偏移 - 沿 Tracker 坐标系 Z 轴正向平移
    /// </summary>
    /// <param name="originalPosition">原始位置 (mm)</param>
    /// <param name="rotation">Tracker 的旋转四元数</param>
    /// <param name="offsetMm">沿 Z 轴的偏移距离 (mm)</param>
    /// <returns>偏移后的位置 (mm)</returns>
    private Vector3 ApplyPositionOffset(Vector3 originalPosition, Quaternion rotation, float offsetMm)
    {
        // Tracker 坐标系的 Z 轴正方向（局部坐标）
        Vector3 localZAxis = new Vector3(0f, 0f, 1f);
        
        // 通过四元数旋转将局部 Z 轴转换到世界坐标系
        Vector3 worldZAxis = rotation * localZAxis;
        
        // 沿世界坐标系中的 Z 轴方向平移
        Vector3 offsetVector = worldZAxis * offsetMm;
        
        // 应用偏移
        Vector3 offsetPosition = originalPosition + offsetVector;
        
        if (verboseOutput)
        {
            Debug.Log($"<color=cyan>【位置偏移计算详情】</color>");
            Debug.Log($"  原始位置 (mm): ({originalPosition.x:F2}, {originalPosition.y:F2}, {originalPosition.z:F2})");
            Debug.Log($"  Tracker Z 轴方向 (世界): ({worldZAxis.x:F4}, {worldZAxis.y:F4}, {worldZAxis.z:F4})");
            Debug.Log($"  偏移向量 (mm): ({offsetVector.x:F2}, {offsetVector.y:F2}, {offsetVector.z:F2})");
            Debug.Log($"  偏移后位置 (mm): ({offsetPosition.x:F2}, {offsetPosition.y:F2}, {offsetPosition.z:F2})");
        }
        
        return offsetPosition;
    }

    // ==================== 数据获取封装 ====================
    
    /// <summary>
    /// 根据配置的数据源获取位姿数据
    /// </summary>
    /// <param name="positionMm">输出: 位置 (毫米)</param>
    /// <param name="rotation">输出: 旋转 (四元数)</param>
    /// <returns>是否成功获取</returns>
    private bool GetPoseData(out Vector3 positionMm, out Quaternion rotation)
    {
        positionMm = Vector3.zero;
        rotation = Quaternion.identity;
        
        switch (dataSource)
        {
            case PoseDataSource.AbsolutePose:
                return GetAbsolutePoseData(out positionMm, out rotation);
                
            case PoseDataSource.RelativePose:
                return GetRelativePoseData(out positionMm, out rotation);
                
            default:
                Debug.LogError($"[TrackerCapture] 未知的数据源: {dataSource}");
                return false;
        }
    }
    
    /// <summary>
    /// 获取绝对位姿数据 (SteamVR 世界坐标系)
    /// </summary>
    private bool GetAbsolutePoseData(out Vector3 positionMm, out Quaternion rotation)
    {
        positionMm = Vector3.zero;
        rotation = Quaternion.identity;
        
        if (poseLogger == null)
        {
            Debug.LogError("[TrackerCapture] PoseLogger 未设置！请配置 ViveTrackerPoseLogger 组件。");
            Debug.LogError("  解决方案: 在 Inspector 中拖入 ViveTrackerPoseLogger，或启用 Auto Find Components");
            return false;
        }
        
        if (!poseLogger.GetTrackerPoseForCalibration(trackerDeviceId, out positionMm, out rotation))
        {
            Debug.LogError($"[TrackerCapture] 无法获取 Tracker[ID:{trackerDeviceId}] 的绝对位姿数据！");
            Debug.LogError("  可能原因: 1) 设备未连接  2) 设备ID错误  3) SteamVR未初始化");
            return false;
        }
        
        if (verboseOutput)
        {
            Debug.Log($"<color=cyan>【绝对位姿数据获取】</color>");
            Debug.Log($"  设备 ID: {trackerDeviceId}");
            Debug.Log($"  坐标系: SteamVR 世界坐标系 (X右, Y上, Z后)");
            Debug.Log($"  位置 (mm): ({positionMm.x:F2}, {positionMm.y:F2}, {positionMm.z:F2})");
        }
        
        return true;
    }
    
    /// <summary>
    /// 获取相对位姿数据 (Reference Tracker 局部坐标系)
    /// 
    /// 重要说明：
    /// - 相对位姿 = Moving Tracker 在 Reference Tracker 坐标系下的位姿
    /// - Reference Tracker 通常固定在场景中，定义了一个局部坐标原点
    /// - 如果 Reference Tracker 固定在机器人基座上，相对位姿可直接用于控制
    /// - 否则需要额外标定 Reference 坐标系到 UR 基座坐标系的变换
    /// </summary>
    private bool GetRelativePoseData(out Vector3 positionMm, out Quaternion rotation)
    {
        positionMm = Vector3.zero;
        rotation = Quaternion.identity;
        
        if (relativePoseMonitor == null)
        {
            Debug.LogError("[TrackerCapture] RelativePoseMonitor 未设置！请配置 SteamVR_RelativePoseMonitor 组件。");
            Debug.LogError("  解决方案: 在 Inspector 中拖入 SteamVR_RelativePoseMonitor，或启用 Auto Find Components");
            return false;
        }
        
        // 获取配置的 Moving Tracker 设备索引
        int[] movingIndices = relativePoseMonitor.movingTrackerIndices;
        if (movingIndices == null || movingIndices.Length == 0)
        {
            Debug.LogError("[TrackerCapture] RelativePoseMonitor 未配置任何 Moving Trackers！");
            Debug.LogError("  解决方案: 在 SteamVR_RelativePoseMonitor 组件中配置 Moving Tracker Indices");
            return false;
        }
        
        // 验证数组索引
        if (movingTrackerArrayIndex < 0 || movingTrackerArrayIndex >= movingIndices.Length)
        {
            Debug.LogError($"[TrackerCapture] Moving Tracker 数组索引 {movingTrackerArrayIndex} 越界！");
            Debug.LogError($"  当前配置的 Moving Trackers 数量: {movingIndices.Length}");
            Debug.LogError($"  有效索引范围: 0 ~ {movingIndices.Length - 1}");
            Debug.LogError($"  将使用第一个 Moving Tracker (索引 0)");
            movingTrackerArrayIndex = 0;
        }
        
        // 获取实际的设备 ID
        int targetDeviceId = movingIndices[movingTrackerArrayIndex];
        
        Vector3 positionM;
        if (!relativePoseMonitor.GetMovingTrackerPoseInReference(targetDeviceId, out positionM, out rotation))
        {
            Debug.LogError($"[TrackerCapture] 无法获取 Moving Tracker (Device{targetDeviceId}) 的相对位姿！");
            Debug.LogError($"  数组索引: {movingTrackerArrayIndex}");
            Debug.LogError($"  设备 ID: {targetDeviceId}");
            Debug.LogError("  可能原因: 1) Reference/Moving Tracker未连接  2) 设备索引错误  3) SteamVR未初始化");
            return false;
        }
        
        // 单位转换: 米 → 毫米
        positionMm = positionM * 1000f;
        
        if (verboseOutput)
        {
            Debug.Log($"<color=cyan>【相对位姿数据获取】</color>");
            Debug.Log($"  Moving Tracker 数组索引: {movingTrackerArrayIndex} / {movingIndices.Length}");
            Debug.Log($"  Moving Tracker 设备 ID: Device{targetDeviceId}");
            Debug.Log($"  Reference Tracker 设备索引: Device{relativePoseMonitor.referenceTrackerIndex}");
            Debug.Log($"  坐标系: Reference Tracker 局部坐标系");
            Debug.Log($"  原始位置 (m): ({positionM.x:F4}, {positionM.y:F4}, {positionM.z:F4})");
            Debug.Log($"  转换位置 (mm): ({positionMm.x:F2}, {positionMm.y:F2}, {positionMm.z:F2})");
            Debug.Log($"  <color=yellow>⚠️ 注意: 相对位姿假设 Reference Tracker 与 UR 基座对齐</color>");
            Debug.Log($"  <color=yellow>   如果 Reference 未固定在基座上,需要额外坐标转换！</color>");
        }
        
        return true;
    }
    
    /// <summary>
    /// 验证当前数据源的配置是否正确
    /// </summary>
    private void ValidateConfiguration()
    {
        Debug.Log($"========== TrackerPoseCapture 配置验证 ==========");
        Debug.Log($"<color=cyan>【数据源】: {dataSource}</color>");
        
        switch (dataSource)
        {
            case PoseDataSource.AbsolutePose:
                if (poseLogger == null)
                {
                    Debug.LogError("  ❌ 绝对位姿模式: ViveTrackerPoseLogger 未设置！");
                    Debug.LogError("     → 请在 Inspector 中拖入 ViveTrackerPoseLogger 组件");
                }
                else
                {
                    Debug.Log($"  ✓ ViveTrackerPoseLogger: {poseLogger.gameObject.name}");
                    Debug.Log($"  ✓ Tracker ID: {trackerDeviceId}");
                    Debug.Log($"  ✓ 坐标转换: 使用手眼标定矩阵 T_cam2base");
                    Debug.Log($"  ✓ 适用场景: Tracker 在 SteamVR 世界坐标系中自由移动");
                }
                break;
                
            case PoseDataSource.RelativePose:
                if (relativePoseMonitor == null)
                {
                    Debug.LogError("  ❌ 相对位姿模式: SteamVR_RelativePoseMonitor 未设置！");
                    Debug.LogError("     → 请在 Inspector 中拖入 SteamVR_RelativePoseMonitor 组件");
                }
                else
                {
                    Debug.Log($"  ✓ RelativePoseMonitor: {relativePoseMonitor.gameObject.name}");
                    
                    // 验证 Moving Tracker 配置
                    int[] movingIndices = relativePoseMonitor.movingTrackerIndices;
                    if (movingIndices == null || movingIndices.Length == 0)
                    {
                        Debug.LogError("  ❌ RelativePoseMonitor 未配置 Moving Trackers！");
                        Debug.LogError("     → 请在 SteamVR_RelativePoseMonitor 中配置 Moving Tracker Indices");
                    }
                    else
                    {
                        Debug.Log($"  ✓ 已配置 {movingIndices.Length} 个 Moving Trackers:");
                        for (int i = 0; i < movingIndices.Length; i++)
                        {
                            string marker = (i == movingTrackerArrayIndex) ? " ← 当前选中" : "";
                            Debug.Log($"     [{i}] Device{movingIndices[i]}{marker}");
                        }
                        
                        if (movingTrackerArrayIndex < 0 || movingTrackerArrayIndex >= movingIndices.Length)
                        {
                            Debug.LogWarning($"  ⚠️ Moving Tracker 数组索引 {movingTrackerArrayIndex} 越界！");
                            Debug.LogWarning($"     有效范围: 0 ~ {movingIndices.Length - 1}");
                        }
                        else
                        {
                            Debug.Log($"  ✓ 将捕获 Device{movingIndices[movingTrackerArrayIndex]} (数组索引 {movingTrackerArrayIndex})");
                        }
                    }
                    
                    Debug.Log($"  <color=yellow>⚠️ 坐标转换: 假设 Reference Tracker 与 UR 基座对齐</color>");
                    Debug.Log($"  <color=yellow>⚠️ 重要前提:</color>");
                    Debug.Log($"     1) Reference Tracker 必须固定在机器人基座上或与基座坐标系对齐");
                    Debug.Log($"     2) Reference Tracker 的坐标轴方向需与 UR 基座一致");
                    Debug.Log($"     3) 如不满足上述条件，输出位姿将无法正确控制机器人！");
                    Debug.Log($"  ✓ 适用场景: Reference 固定，Moving Tracker 在局部范围内移动");
                }
                break;
        }
        
        if (uiControl == null)
        {
            Debug.LogError("  ❌ main_ui_control 未设置！");
            Debug.LogError("     → 请在 Inspector 中拖入 main_ui_control 组件");
        }
        else
        {
            Debug.Log($"  ✓ UI Control: {uiControl.gameObject.name}");
        }
        
        Debug.Log("================================================\n");
    }

    /// <summary>
    /// 四元数转旋转矢量 (修复版本)
    /// </summary>
    private Vector3 QuaternionToRotationVector(Quaternion q)
    {
        // 归一化
        q = NormalizeQuaternion(q);
        
        // 四元数符号规范化 (强制 q.w >= 0)
        if (q.w < 0f)
        {
            q.x = -q.x;
            q.y = -q.y;
            q.z = -q.z;
            q.w = -q.w;
        }
        
        // 计算旋转角度
        float wClamped = Mathf.Clamp(q.w, 0f, 1f);
        float angle = 2f * Mathf.Acos(wClamped);
        
        // 处理特殊情况
        
        // 接近 180°
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
        
        // 接近 0°
        if (angle < 1e-6f)
        {
            return Vector3.zero;
        }
        
        // 一般情况
        float sinHalfAngle = Mathf.Sin(angle * 0.5f);
        float scale = angle / sinHalfAngle;
        return new Vector3(q.x * scale, q.y * scale, q.z * scale);
    }

    private Quaternion NormalizeQuaternion(Quaternion q)
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

    #region 快捷键控制
    
    void Update()
    {
        // 空格键: 捕获并执行控制
        if (Input.GetKeyDown(KeyCode.Space))
        {
            CaptureAndExecute();
        }

        // C 键: 仅捕获不执行 (调试用)
        if (Input.GetKeyDown(KeyCode.C))
        {
            CaptureOnly();
        }

        // L 键: 列出所有 Tracker
        if (Input.GetKeyDown(KeyCode.L))
        {
            ListAllTrackers();
        }
    }

    #endregion
}

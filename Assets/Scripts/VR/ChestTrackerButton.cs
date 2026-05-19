using System.Collections;
using UnityEngine;
using Valve.VR;

/// <summary>
/// ChestTrackerButton: 用于响应绑定到胸部位置的 Tracker 的 SteamVR 按钮事件
/// 
/// 主要功能：
/// - 监听胸部 Tracker 的 Power 按钮事件
/// - Power 按钮按下时触发自动捕获位姿并执行 MoveL 控制
/// - 替代空格键的手动触发方式，实现物理按钮控制
/// 
/// 使用方法：
/// 1. 在 Inspector 中配置胸部 Tracker 的设备 ID
/// 2. 确保 TrackerPoseCapture 组件已正确配置
/// 3. 按下胸部 Tracker 的 Power 按钮触发控制
/// 
/// 设计参考：
/// - 严格遵循 KeyboardTrackerButton.cs 和 TriggerTester.cs 的事件注册模式
/// - 使用 SteamVR_Input_Sources.Chest 作为输入源
/// - 复用 TrackerPoseCapture 的自动执行逻辑
/// </summary>
public class ChestTrackerButton : MonoBehaviour
{
    // ==================== SteamVR Input 动作绑定 ====================
    [Header("SteamVR 输入动作")]
    [Tooltip("Power 按钮动作（主要功能触发按钮）")]
    public SteamVR_Action_Boolean booleanPower;

    [Tooltip("Grip 按钮动作（备用功能）")]
    public SteamVR_Action_Boolean booleanGrip;

    [Tooltip("Trigger 按钮动作（备用功能）")]
    public SteamVR_Action_Boolean booleanTrigger;

    [Tooltip("Trackpad 按钮动作（备用功能）")]
    public SteamVR_Action_Boolean booleanTrackpad;

    [Tooltip("Menu 按钮动作（备用功能）")]
    public SteamVR_Action_Boolean booleanMenu;

    // ==================== 组件引用 ====================
    [Header("组件引用")]
    [Tooltip("TrackerPoseCapture 组件引用 - 用于触发自动捕获和执行 MoveL")]
    public TrackerPoseCapture trackerPoseCapture;

    [Tooltip("是否自动查找 TrackerPoseCapture 组件")]
    public bool autoFindComponents = true;

    // ==================== 胸部 Tracker 配置 ====================
    [Header("胸部 Tracker 配置")]
    [Tooltip("胸部 Tracker 的设备 ID（需要与 TrackerPoseCapture 中配置的 ID 一致）")]
    public uint chestTrackerDeviceId = 3;

    [Tooltip("是否启用调试日志输出")]
    public bool enableDebugLog = true;

    [Header("位姿偏移配置")]
    [Tooltip("是否启用位置偏移（沿 Tracker 坐标系 Z 轴正向平移）")]
    public bool enablePositionOffset = false;
    
    [Tooltip("沿 Tracker Z 轴正向的偏移距离 (mm)，正值向前，负值向后")]
    public float zAxisOffsetMm = 100f;

    [Header("运动参数配置（可选）")]
    [Tooltip("是否使用自定义加速度（取消勾选则使用 TrackerPoseCapture 或 UI 默认值）")]
    public bool useCustomAcceleration = false;
    
    [Tooltip("自定义加速度 (m/s²)，范围: 0.1 ~ 1.5")]
    [Range(0.1f, 1.5f)]
    public float customAcceleration = 0.5f;

    [Tooltip("是否使用自定义线速度（取消勾选则使用 TrackerPoseCapture 或 UI 默认值）")]
    public bool useCustomLinearSpeed = false;
    
    [Tooltip("自定义线速度 (m/s)，范围: 0.01 ~ 0.5")]
    [Range(0.01f, 0.5f)]
    public float customLinearSpeed = 0.1f;

    [Tooltip("是否使用自定义混合半径（取消勾选则使用 TrackerPoseCapture 或 UI 默认值）")]
    public bool useCustomBlendRadius = false;
    
    [Tooltip("自定义混合半径 (m)，范围: 0 ~ 0.1")]
    [Range(0f, 0.1f)]
    public float customBlendRadius = 0.0f;

    // ==================== Unity 生命周期 ====================

    /// <summary>
    /// Start: 初始化组件引用并注册 SteamVR 事件
    /// </summary>
    void Start()
    {
        // 自动查找 TrackerPoseCapture 组件
        if (autoFindComponents && trackerPoseCapture == null)
        {
            trackerPoseCapture = FindObjectOfType<TrackerPoseCapture>();
            if (trackerPoseCapture == null)
            {
                Debug.LogError("<color=red>[ChestTrackerButton] 未找到 TrackerPoseCapture 组件！" +
                              "请确保场景中存在该组件，或手动在 Inspector 中指定。</color>");
            }
            else
            {
                LogDebug($"<color=green>[ChestTrackerButton] 已自动找到 TrackerPoseCapture: {trackerPoseCapture.gameObject.name}</color>");
            }
        }

        // 检查 SteamVR 输入系统是否已初始化
        if (SteamVR.initializedState != SteamVR.InitializedStates.InitializeSuccess)
        {
            Debug.LogWarning($"<color=yellow>[ChestTrackerButton] SteamVR 尚未初始化（当前状态: {SteamVR.initializedState}），将推迟事件注册</color>");
            StartCoroutine(WaitForSteamVRInitialization());
        }
        else
        {
            // 注册 SteamVR 按钮事件（使用 Chest 输入源）
            RegisterButtonEvents();
        }

        // 输出初始化日志
        if (trackerPoseCapture != null)
        {
            LogDebug($"<color=green>[ChestTrackerButton] 初始化完成</color>");
            LogDebug($"  输入源: SteamVR_Input_Sources.Chest");
            LogDebug($"  胸部 Tracker 设备 ID: {chestTrackerDeviceId}");
            LogDebug($"  功能: Power 按钮 → 触发自动 MoveL 控制");
            LogDebug($"  提示: 按下胸部 Tracker 的 Power 按钮可触发机器人控制");
        }
    }

    /// <summary>
    /// 等待 SteamVR 初始化完成后再注册事件
    /// 防止过早访问 SteamVR 输入系统导致输入阻塞
    /// </summary>
    private System.Collections.IEnumerator WaitForSteamVRInitialization()
    {
        float timeout = 5f;
        float elapsed = 0f;

        while (SteamVR.initializedState != SteamVR.InitializedStates.InitializeSuccess && elapsed < timeout)
        {
            elapsed += Time.deltaTime;
            yield return null;
        }

        if (SteamVR.initializedState == SteamVR.InitializedStates.InitializeSuccess)
        {
            LogDebug("<color=green>[ChestTrackerButton] SteamVR 初始化完成，现在注册按钮事件</color>");
            RegisterButtonEvents();
        }
        else
        {
            Debug.LogError($"<color=red>[ChestTrackerButton] SteamVR 初始化超时（状态: {SteamVR.initializedState}），按钮事件未注册</color>");
            Debug.LogError("  <color=red>可能原因: SteamVR 未运行、VR 设备未连接、或 Chest 输入源未配置</color>");
            Debug.LogError("  <color=yellow>建议: 暂时禁用此组件，使用键盘快捷键（空格键）代替</color>");
        }
    }

    /// <summary>
    /// OnDestroy: 注销 SteamVR 事件，防止内存泄漏
    /// </summary>
    private void OnDestroy()
    {
        UnregisterButtonEvents();
        LogDebug("<color=yellow>[ChestTrackerButton] 已注销事件监听</color>");
    }

    // ==================== 事件注册/注销 ====================

    /// <summary>
    /// 注册 SteamVR 按钮事件
    /// 参考模式: KeyboardTrackerButton.cs 和 TriggerTester.cs
    /// 增加了异常保护，防止初始化失败影响其他系统
    /// </summary>
    private void RegisterButtonEvents()
    {
        try
        {
            LogDebug("<color=cyan>[ChestTrackerButton] 开始注册 SteamVR 事件...</color>");

            if (booleanPower != null)
            {
                // 检查输入源是否可用
                if (booleanPower[SteamVR_Input_Sources.Chest] != null)
                {
                    booleanPower[SteamVR_Input_Sources.Chest].onStateDown += OnStateDownPower;
                    LogDebug("<color=green>[ChestTrackerButton] ✓ 已注册 Power 按钮事件</color>");
                }
                else
                {
                    Debug.LogError("<color=red>[ChestTrackerButton] ✗ booleanPower[Chest] 返回 null！Chest 输入源未定义或未绑定</color>");
                    Debug.LogError("  <color=yellow>解决方案: 打开 Window → SteamVR Input → 确保 Chest 源已绑定到动作</color>");
                }
            }
            else
            {
                Debug.LogWarning("<color=yellow>[ChestTrackerButton] booleanPower 动作未设置，请在 Inspector 中配置</color>");
            }

            // 注册其他按钮事件（可选，用于扩展功能）
            if (booleanGrip != null && booleanGrip[SteamVR_Input_Sources.Chest] != null)
            {
                booleanGrip[SteamVR_Input_Sources.Chest].onStateDown += OnStateDownGrip;
                LogDebug("  ✓ 已注册 Grip 按钮");
            }
            
            if (booleanTrigger != null && booleanTrigger[SteamVR_Input_Sources.Chest] != null)
            {
                booleanTrigger[SteamVR_Input_Sources.Chest].onStateDown += OnStateDownTrigger;
                LogDebug("  ✓ 已注册 Trigger 按钮");
            }
            
            if (booleanTrackpad != null && booleanTrackpad[SteamVR_Input_Sources.Chest] != null)
            {
                booleanTrackpad[SteamVR_Input_Sources.Chest].onStateDown += OnStateDownTrackpad;
                LogDebug("  ✓ 已注册 Trackpad 按钮");
            }
            
            if (booleanMenu != null && booleanMenu[SteamVR_Input_Sources.Chest] != null)
            {
                booleanMenu[SteamVR_Input_Sources.Chest].onStateDown += OnStateDownMenu;
                LogDebug("  ✓ 已注册 Menu 按钮");
            }

            LogDebug("<color=green>[ChestTrackerButton] SteamVR 事件注册完成</color>");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"<color=red>[ChestTrackerButton] ✗ 注册 SteamVR 事件时发生严重错误!</color>");
            Debug.LogError($"  错误信息: {ex.Message}");
            Debug.LogError($"  错误类型: {ex.GetType().Name}");
            Debug.LogError($"  堆栈跟踪: {ex.StackTrace}");
            Debug.LogError("  <color=yellow>可能原因:</color>");
            Debug.LogError("    1) SteamVR 输入系统未正确初始化");
            Debug.LogError("    2) Chest 输入源未在 SteamVR Input 中定义");
            Debug.LogError("    3) VR 设备未连接或 SteamVR 未运行");
            Debug.LogError("  <color=yellow>临时解决方案:</color>");
            Debug.LogError("    → 在 Unity Inspector 中禁用 ChestTrackerButton 组件");
            Debug.LogError("    → 使用键盘空格键代替胸部 Tracker 按钮");
        }
    }

    /// <summary>
    /// 组件禁用时停止所有协程，防止退出 Play 模式时崩溃
    /// </summary>
    private void OnDisable()
    {
        StopAllCoroutines();
    }

    /// <summary>
    /// 注销 SteamVR 按钮事件
    /// 增加了空引用检查，防止退出 Play 模式时 SteamVR 系统已销毁导致的崩溃
    /// </summary>
    private void UnregisterButtonEvents()
    {
        try
        {
            if (booleanPower != null && booleanPower[SteamVR_Input_Sources.Chest] != null)
                booleanPower[SteamVR_Input_Sources.Chest].onStateDown -= OnStateDownPower;
            
            if (booleanGrip != null && booleanGrip[SteamVR_Input_Sources.Chest] != null)
                booleanGrip[SteamVR_Input_Sources.Chest].onStateDown -= OnStateDownGrip;
            
            if (booleanTrigger != null && booleanTrigger[SteamVR_Input_Sources.Chest] != null)
                booleanTrigger[SteamVR_Input_Sources.Chest].onStateDown -= OnStateDownTrigger;
            
            if (booleanTrackpad != null && booleanTrackpad[SteamVR_Input_Sources.Chest] != null)
                booleanTrackpad[SteamVR_Input_Sources.Chest].onStateDown -= OnStateDownTrackpad;
            
            if (booleanMenu != null && booleanMenu[SteamVR_Input_Sources.Chest] != null)
                booleanMenu[SteamVR_Input_Sources.Chest].onStateDown -= OnStateDownMenu;
        }
        catch (System.Exception ex)
        {
            // 捕获退出时可能出现的任何异常，防止崩溃
            if (enableDebugLog)
            {
                Debug.LogWarning($"[ChestTrackerButton] 注销事件时发生异常（可忽略）: {ex.Message}");
            }
        }
    }

    // ==================== 按钮事件回调 ====================

    /// <summary>
    /// Power 按钮按下事件回调 - 核心功能
    /// 功能: 触发 TrackerPoseCapture 的自动捕获和执行 MoveL 控制
    /// 等价于: 按下空格键的效果
    /// </summary>
    private void OnStateDownPower(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        LogDebug("<color=magenta>[ChestTrackerButton] ★ Power 按钮按下 - 开始触发自动 MoveL 控制</color>");

        // 检查 TrackerPoseCapture 组件是否可用
        if (trackerPoseCapture == null)
        {
            Debug.LogError("<color=red>[ChestTrackerButton] TrackerPoseCapture 组件未设置！无法执行自动控制。</color>");
            Debug.LogError("  解决方案: 在 Inspector 中拖入 TrackerPoseCapture 组件，或启用 Auto Find Components");
            return;
        }

        // 验证设备 ID 是否一致
        if (trackerPoseCapture.trackerDeviceId != chestTrackerDeviceId)
        {
            Debug.LogWarning($"<color=yellow>[ChestTrackerButton] 设备 ID 不一致警告:</color>");
            Debug.LogWarning($"  ChestTrackerButton 配置的设备 ID: {chestTrackerDeviceId}");
            Debug.LogWarning($"  TrackerPoseCapture 配置的设备 ID: {trackerPoseCapture.trackerDeviceId}");
            Debug.LogWarning($"  将使用 TrackerPoseCapture 的设备 ID: {trackerPoseCapture.trackerDeviceId}");
        }

        // 应用位置偏移（如果启用）
        if (enablePositionOffset)
        {
            trackerPoseCapture.enablePositionOffset = true;
            trackerPoseCapture.zAxisOffsetMm = zAxisOffsetMm;
            LogDebug($"<color=cyan>[ChestTrackerButton] 启用位置偏移: Z 轴 +{zAxisOffsetMm:F2} mm</color>");
        }
        else
        {
            // 确保不会意外启用偏移
            trackerPoseCapture.enablePositionOffset = false;
        }

        // 应用自定义运动参数（如果启用）
        bool paramsModified = false;
        
        if (useCustomAcceleration)
        {
            trackerPoseCapture.useCustomAcceleration = true;
            trackerPoseCapture.customAcceleration = customAcceleration;
            paramsModified = true;
            LogDebug($"<color=cyan>[ChestTrackerButton] 应用自定义加速度: {customAcceleration:F3} m/s²</color>");
        }
        
        if (useCustomLinearSpeed)
        {
            trackerPoseCapture.useCustomLinearSpeed = true;
            trackerPoseCapture.customLinearSpeed = customLinearSpeed;
            paramsModified = true;
            LogDebug($"<color=cyan>[ChestTrackerButton] 应用自定义线速度: {customLinearSpeed:F3} m/s</color>");
        }
        
        if (useCustomBlendRadius)
        {
            trackerPoseCapture.useCustomBlendRadius = true;
            trackerPoseCapture.customBlendRadius = customBlendRadius;
            paramsModified = true;
            LogDebug($"<color=cyan>[ChestTrackerButton] 应用自定义混合半径: {customBlendRadius:F3} m</color>");
        }

        if (paramsModified || enablePositionOffset)
        {
            LogDebug("<color=yellow>[ChestTrackerButton] 已应用自定义配置</color>");
        }

        // 调用 TrackerPoseCapture 的核心方法
        // 这个方法会:
        // 1. 捕获指定 Tracker 的当前位姿
        // 2. 转换为 UR 基座坐标系
        // 3. 填充到 UI 输入框（包含运动参数）
        // 4. 勾选 SteamVR 坐标 Toggle
        // 5. 调用 BuildAndSetMoveLFromInputs()
        // 6. 触发执行脉冲
        trackerPoseCapture.CaptureAndExecute();

        LogDebug("<color=green>[ChestTrackerButton] ✓ 已调用 TrackerPoseCapture.CaptureAndExecute()</color>");
        LogDebug("  后续处理由 TrackerPoseCapture 自动完成，请查看 Console 日志");
    }

    /// <summary>
    /// Grip 按钮按下事件回调 - 备用功能
    /// </summary>
    private void OnStateDownGrip(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        LogDebug("<color=cyan>[ChestTrackerButton] Grip 按钮按下</color>");
        // 可在此添加其他功能
    }

    /// <summary>
    /// Trigger 按钮按下事件回调 - 备用功能
    /// </summary>
    private void OnStateDownTrigger(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        LogDebug("<color=cyan>[ChestTrackerButton] Trigger 按钮按下</color>");
        // 可在此添加其他功能
    }

    /// <summary>
    /// Trackpad 按钮按下事件回调 - 备用功能
    /// </summary>
    private void OnStateDownTrackpad(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        LogDebug("<color=cyan>[ChestTrackerButton] Trackpad 按钮按下</color>");
        // 可在此添加其他功能
    }

    /// <summary>
    /// Menu 按钮按下事件回调 - 备用功能
    /// </summary>
    private void OnStateDownMenu(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        LogDebug("<color=cyan>[ChestTrackerButton] Menu 按钮按下</color>");
        // 可在此添加其他功能
    }

    // ==================== 辅助方法 ====================

    /// <summary>
    /// 条件调试日志输出
    /// </summary>
    private void LogDebug(string message)
    {
        if (enableDebugLog)
        {
            Debug.Log(message);
        }
    }

    /// <summary>
    /// 公共方法：手动触发自动 MoveL 控制（供 UI 按钮或其他脚本调用）
    /// </summary>
    [ContextMenu("手动触发自动控制")]
    public void TriggerAutoMoveL()
    {
        OnStateDownPower(null, SteamVR_Input_Sources.Chest);
    }

    /// <summary>
    /// 公共方法：更新胸部 Tracker 设备 ID（运行时修改）
    /// </summary>
    public void SetChestTrackerDeviceId(uint deviceId)
    {
        chestTrackerDeviceId = deviceId;
        
        // 同步到 TrackerPoseCapture
        if (trackerPoseCapture != null)
        {
            trackerPoseCapture.trackerDeviceId = deviceId;
            LogDebug($"<color=yellow>[ChestTrackerButton] 已更新胸部 Tracker 设备 ID: {deviceId}</color>");
        }
    }

    /// <summary>
    /// 公共方法：验证配置是否正确
    /// </summary>
    [ContextMenu("验证配置")]
    public void ValidateConfiguration()
    {
        Debug.Log("========== ChestTrackerButton 配置验证 ==========");
        
        // 检查 TrackerPoseCapture
        if (trackerPoseCapture == null)
        {
            Debug.LogError("  ❌ TrackerPoseCapture: 未设置");
        }
        else
        {
            Debug.Log($"  ✓ TrackerPoseCapture: 已设置 ({trackerPoseCapture.gameObject.name})");
            Debug.Log($"    - 配置的设备 ID: {trackerPoseCapture.trackerDeviceId}");
        }

        // 检查 SteamVR 动作
        Debug.Log($"  ✓ booleanPower: {(booleanPower != null ? "已设置" : "❌ 未设置")}");
        Debug.Log($"  ✓ booleanGrip: {(booleanGrip != null ? "已设置" : "未设置（可选）")}");
        Debug.Log($"  ✓ booleanTrigger: {(booleanTrigger != null ? "已设置" : "未设置（可选）")}");
        Debug.Log($"  ✓ booleanTrackpad: {(booleanTrackpad != null ? "已设置" : "未设置（可选）")}");
        Debug.Log($"  ✓ booleanMenu: {(booleanMenu != null ? "已设置" : "未设置（可选）")}");

        // 检查设备 ID
        Debug.Log($"  - 胸部 Tracker 设备 ID: {chestTrackerDeviceId}");
        if (trackerPoseCapture != null && trackerPoseCapture.trackerDeviceId != chestTrackerDeviceId)
        {
            Debug.LogWarning($"  ⚠️ 设备 ID 不匹配: ChestTrackerButton={chestTrackerDeviceId}, TrackerPoseCapture={trackerPoseCapture.trackerDeviceId}");
        }

        Debug.Log("================================================\n");
    }
}

using UnityEngine;

/// <summary>
/// 位姿偏移配置检查工具
/// 用于诊断位姿偏移功能是否正确配置
/// 使用方法: 将此脚本添加到场景中任意 GameObject，进入 Play 模式查看 Console 日志
/// </summary>
public class OffsetConfigChecker : MonoBehaviour
{
    [Header("检查目标")]
    [Tooltip("拖入需要检查的 ChestTrackerButton 组件")]
    public ChestTrackerButton chestTrackerButton;
    
    [Tooltip("拖入需要检查的 TrackerPoseCapture 组件")]
    public TrackerPoseCapture trackerPoseCapture;

    [Header("选项")]
    [Tooltip("是否在 Start 时自动检查")]
    public bool checkOnStart = true;

    [Tooltip("是否每帧持续检查（用于调试）")]
    public bool continuousCheck = false;

    [Tooltip("持续检查的间隔时间（秒）")]
    public float checkInterval = 2f;

    private float lastCheckTime = 0f;

    void Start()
    {
        if (checkOnStart)
        {
            Debug.Log("<color=cyan>========== 位姿偏移配置检查 ==========</color>");
            CheckConfiguration();
            Debug.Log("<color=cyan>=====================================</color>");
        }
    }

    void Update()
    {
        if (continuousCheck && Time.time - lastCheckTime >= checkInterval)
        {
            lastCheckTime = Time.time;
            Debug.Log($"<color=yellow>【定时检查 t={Time.time:F1}s】</color>");
            CheckConfiguration();
        }
    }

    [ContextMenu("立即检查配置")]
    public void CheckConfiguration()
    {
        Debug.Log($"<color=yellow>开始检查位姿偏移配置...</color>");
        Debug.Log($"  检查时间: {System.DateTime.Now:yyyy-MM-dd HH:mm:ss}");
        Debug.Log($"  游戏时间: {Time.time:F2}s");
        Debug.Log("");

        // 检查 ChestTrackerButton
        CheckChestTrackerButton();
        Debug.Log("");

        // 检查 TrackerPoseCapture
        CheckTrackerPoseCapture();
        Debug.Log("");

        // 总结
        ProvideSummary();
    }

    private void CheckChestTrackerButton()
    {
        Debug.Log("<color=cyan>【1. ChestTrackerButton 配置】</color>");

        if (chestTrackerButton == null)
        {
            Debug.LogWarning("  ⚠️ ChestTrackerButton 未设置！");
            Debug.LogWarning("  → 解决方案: 在 Inspector 中拖入 ChestTrackerButton 组件");
            return;
        }

        Debug.Log($"  组件已找到: {chestTrackerButton.gameObject.name}");
        Debug.Log($"  组件启用状态: {(chestTrackerButton.enabled ? "✓ 启用" : "✗ 禁用")}");
        Debug.Log("");

        // 检查位置偏移配置
        Debug.Log("  <color=yellow>位置偏移配置:</color>");
        Debug.Log($"    Enable Position Offset: {GetCheckmark(chestTrackerButton.enablePositionOffset)} {chestTrackerButton.enablePositionOffset}");
        if (chestTrackerButton.enablePositionOffset)
        {
            Debug.Log($"    Z Axis Offset Mm: {chestTrackerButton.zAxisOffsetMm:F2} mm");
        }
        else
        {
            Debug.LogWarning($"    ⚠️ 位置偏移未启用！");
            Debug.LogWarning($"    → 解决方案: 勾选 ChestTrackerButton 的 Enable Position Offset");
        }
        Debug.Log("");

        // 检查运动参数配置
        Debug.Log("  <color=yellow>运动参数配置:</color>");
        Debug.Log($"    Use Custom Acceleration: {GetCheckmark(chestTrackerButton.useCustomAcceleration)} {chestTrackerButton.useCustomAcceleration}");
        if (chestTrackerButton.useCustomAcceleration)
        {
            Debug.Log($"      → Custom Acceleration: {chestTrackerButton.customAcceleration:F3} m/s²");
        }

        Debug.Log($"    Use Custom Linear Speed: {GetCheckmark(chestTrackerButton.useCustomLinearSpeed)} {chestTrackerButton.useCustomLinearSpeed}");
        if (chestTrackerButton.useCustomLinearSpeed)
        {
            Debug.Log($"      → Custom Linear Speed: {chestTrackerButton.customLinearSpeed:F3} m/s");
        }

        Debug.Log($"    Use Custom Blend Radius: {GetCheckmark(chestTrackerButton.useCustomBlendRadius)} {chestTrackerButton.useCustomBlendRadius}");
        if (chestTrackerButton.useCustomBlendRadius)
        {
            Debug.Log($"      → Custom Blend Radius: {chestTrackerButton.customBlendRadius:F3} m");
        }
        Debug.Log("");

        // 检查其他配置
        Debug.Log("  <color=yellow>其他配置:</color>");
        Debug.Log($"    Enable Debug Log: {GetCheckmark(chestTrackerButton.enableDebugLog)} {chestTrackerButton.enableDebugLog}");
        Debug.Log($"    Chest Tracker Device ID: {chestTrackerButton.chestTrackerDeviceId}");
    }

    private void CheckTrackerPoseCapture()
    {
        Debug.Log("<color=cyan>【2. TrackerPoseCapture 配置】</color>");

        if (trackerPoseCapture == null)
        {
            Debug.LogWarning("  ⚠️ TrackerPoseCapture 未设置！");
            Debug.LogWarning("  → 解决方案: 在 Inspector 中拖入 TrackerPoseCapture 组件");
            return;
        }

        Debug.Log($"  组件已找到: {trackerPoseCapture.gameObject.name}");
        Debug.Log($"  组件启用状态: {(trackerPoseCapture.enabled ? "✓ 启用" : "✗ 禁用")}");
        Debug.Log("");

        // 检查位置偏移配置
        Debug.Log("  <color=yellow>位置偏移配置:</color>");
        Debug.Log($"    Enable Position Offset: {GetCheckmark(trackerPoseCapture.enablePositionOffset)} {trackerPoseCapture.enablePositionOffset}");
        if (trackerPoseCapture.enablePositionOffset)
        {
            Debug.Log($"    Z Axis Offset Mm: {trackerPoseCapture.zAxisOffsetMm:F2} mm");
        }
        else
        {
            Debug.LogWarning($"    ⚠️ 位置偏移未启用！");
            Debug.LogWarning($"    → 如果使用空格键触发，需要勾选 TrackerPoseCapture 的 Enable Position Offset");
        }
        Debug.Log("");

        // 检查运动参数配置
        Debug.Log("  <color=yellow>运动参数配置:</color>");
        Debug.Log($"    Use Custom Acceleration: {GetCheckmark(trackerPoseCapture.useCustomAcceleration)} {trackerPoseCapture.useCustomAcceleration}");
        if (trackerPoseCapture.useCustomAcceleration)
        {
            Debug.Log($"      → Custom Acceleration: {trackerPoseCapture.customAcceleration:F3} m/s²");
        }

        Debug.Log($"    Use Custom Linear Speed: {GetCheckmark(trackerPoseCapture.useCustomLinearSpeed)} {trackerPoseCapture.useCustomLinearSpeed}");
        if (trackerPoseCapture.useCustomLinearSpeed)
        {
            Debug.Log($"      → Custom Linear Speed: {trackerPoseCapture.customLinearSpeed:F3} m/s");
        }

        Debug.Log($"    Use Custom Blend Radius: {GetCheckmark(trackerPoseCapture.useCustomBlendRadius)} {trackerPoseCapture.useCustomBlendRadius}");
        if (trackerPoseCapture.useCustomBlendRadius)
        {
            Debug.Log($"      → Custom Blend Radius: {trackerPoseCapture.customBlendRadius:F3} m");
        }
        Debug.Log("");

        // 检查其他配置
        Debug.Log("  <color=yellow>其他配置:</color>");
        Debug.Log($"    Verbose Output: {GetCheckmark(trackerPoseCapture.verboseOutput)} {trackerPoseCapture.verboseOutput}");
        Debug.Log($"    Tracker Device ID: {trackerPoseCapture.trackerDeviceId}");
    }

    private void ProvideSummary()
    {
        Debug.Log("<color=cyan>【3. 诊断总结】</color>");

        bool hasIssue = false;

        // 检查 ChestTrackerButton
        if (chestTrackerButton == null)
        {
            Debug.LogError("  ✗ ChestTrackerButton 组件未设置");
            hasIssue = true;
        }
        else
        {
            if (!chestTrackerButton.enabled)
            {
                Debug.LogWarning("  ⚠️ ChestTrackerButton 组件已禁用");
                hasIssue = true;
            }

            if (!chestTrackerButton.enablePositionOffset)
            {
                Debug.LogError("  ✗ ChestTrackerButton 的位置偏移未启用！");
                Debug.LogError("  → 这是偏移功能不生效的主要原因！");
                Debug.LogError("  → 解决方案: 勾选 ChestTrackerButton → Enable Position Offset");
                hasIssue = true;
            }
            else
            {
                Debug.Log($"  ✓ ChestTrackerButton 位置偏移已启用 (偏移量: {chestTrackerButton.zAxisOffsetMm:F2} mm)");
            }

            if (!chestTrackerButton.enableDebugLog)
            {
                Debug.LogWarning("  ⚠️ ChestTrackerButton 调试日志未启用");
                Debug.LogWarning("  → 建议启用以查看详细的执行过程");
            }
        }

        // 检查 TrackerPoseCapture
        if (trackerPoseCapture == null)
        {
            Debug.LogWarning("  ⚠️ TrackerPoseCapture 组件未设置");
        }
        else
        {
            if (!trackerPoseCapture.enabled)
            {
                Debug.LogWarning("  ⚠️ TrackerPoseCapture 组件已禁用");
                hasIssue = true;
            }

            if (!trackerPoseCapture.verboseOutput)
            {
                Debug.LogWarning("  ⚠️ TrackerPoseCapture 详细输出未启用");
                Debug.LogWarning("  → 建议启用以查看位姿捕获的详细信息");
            }
        }

        Debug.Log("");

        if (!hasIssue)
        {
            Debug.Log("<color=green>  ✓✓✓ 配置正确！位姿偏移功能应该可以正常工作。</color>");
        }
        else
        {
            Debug.LogError("<color=red>  ✗✗✗ 发现配置问题！请按照上述建议修复。</color>");
        }
    }

    private string GetCheckmark(bool value)
    {
        return value ? "✓" : "✗";
    }

    // 在 Inspector 中显示帮助信息
    [ContextMenu("显示使用说明")]
    private void ShowHelp()
    {
        Debug.Log("<color=cyan>========== OffsetConfigChecker 使用说明 ==========</color>");
        Debug.Log("");
        Debug.Log("【功能】");
        Debug.Log("  检查位姿偏移功能的配置状态，帮助诊断为什么偏移不生效");
        Debug.Log("");
        Debug.Log("【使用步骤】");
        Debug.Log("  1. 将此脚本添加到场景中任意 GameObject");
        Debug.Log("  2. 在 Inspector 中:");
        Debug.Log("     - 拖入 ChestTrackerButton 组件到对应字段");
        Debug.Log("     - 拖入 TrackerPoseCapture 组件到对应字段");
        Debug.Log("  3. 进入 Play 模式，查看 Console 中的检查报告");
        Debug.Log("");
        Debug.Log("【手动触发】");
        Debug.Log("  - 在 Inspector 中右键点击此组件");
        Debug.Log("  - 选择 '立即检查配置'");
        Debug.Log("");
        Debug.Log("【持续监控】");
        Debug.Log("  - 勾选 'Continuous Check' 选项");
        Debug.Log("  - 每隔 'Check Interval' 秒自动检查一次");
        Debug.Log("  - 用于实时监控配置变化");
        Debug.Log("");
        Debug.Log("=================================================");
    }
}

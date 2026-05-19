using UnityEngine;
using UnityEngine.UI;
using HandEyeCalibration;

/// <summary>
/// UI按钮控制脚本,用于触发标定数据采集功能
/// 支持切换探针标定和手眼标定两种采集模式
/// </summary>
public class GripButtonUI : MonoBehaviour
{
    /// <summary>
    /// 标定类型枚举
    /// </summary>
    public enum CalibrationType
    {
        [Tooltip("探针标定 - 计算探针尖端位置")]
        ProbeCalibration,
        [Tooltip("手眼标定 - 计算相机与机器人基座的转换关系")]
        HandEyeCalibration
    }

    [Header("标定类型选择")]
    [Tooltip("选择Power按钮触发的标定类型")]
    public CalibrationType calibrationType = CalibrationType.ProbeCalibration;

    [Header("引用设置")]
    [Tooltip("拖入场景中的KeyboardTrackerButton组件（用于探针标定）")]
    public KeyboardTrackerButton keyboardTrackerButton;

    [Tooltip("拖入场景中的HandEyeCalibrationUI组件（用于手眼标定）")]
    public HandEyeCalibrationUI handEyeCalibrationUI;

    private Button button;

    void Start()
    {
        // 获取按钮组件
        button = GetComponent<Button>();
        
        if (button == null)
        {
            Debug.LogError("[GripButtonUI] 未找到Button组件！");
            return;
        }

        // 根据标定类型验证引用
        if (calibrationType == CalibrationType.ProbeCalibration)
        {
            if (keyboardTrackerButton == null)
            {
                Debug.LogError("[GripButtonUI] 探针标定模式下未设置KeyboardTrackerButton引用！");
                return;
            }
        }
        else if (calibrationType == CalibrationType.HandEyeCalibration)
        {
            if (handEyeCalibrationUI == null)
            {
                Debug.LogError("[GripButtonUI] 手眼标定模式下未设置HandEyeCalibrationUI引用！");
                return;
            }
        }

        // 注册按钮点击事件
        button.onClick.AddListener(OnButtonClick);
        
        Debug.Log($"<color=green>[GripButtonUI] UI按钮已初始化 - 当前模式: {GetCalibrationTypeName()}</color>");
    }

    private void OnButtonClick()
    {
        // 根据选择的标定类型调用相应的方法
        switch (calibrationType)
        {
            case CalibrationType.ProbeCalibration:
                // 探针标定：调用KeyboardTrackerButton的数据采集方法
                if (keyboardTrackerButton != null)
                {
                    keyboardTrackerButton.TriggerProbeCalibration();
                    Debug.Log("<color=cyan>[GripButtonUI] UI按钮已按下 - 触发探针标定数据采集</color>");
                }
                else
                {
                    Debug.LogError("[GripButtonUI] KeyboardTrackerButton引用未设置，无法执行探针标定！");
                }
                break;

            case CalibrationType.HandEyeCalibration:
                // 手眼标定：调用HandEyeCalibrationUI的数据采集方法
                if (handEyeCalibrationUI != null)
                {
                    handEyeCalibrationUI.CaptureCalibrationData();
                    Debug.Log("<color=cyan>[GripButtonUI] UI按钮已按下 - 触发手眼标定数据采集</color>");
                }
                else
                {
                    Debug.LogError("[GripButtonUI] HandEyeCalibrationUI引用未设置，无法执行手眼标定！");
                }
                break;
        }
    }

    /// <summary>
    /// 获取当前标定类型的友好名称
    /// </summary>
    private string GetCalibrationTypeName()
    {
        switch (calibrationType)
        {
            case CalibrationType.ProbeCalibration:
                return "探针标定";
            case CalibrationType.HandEyeCalibration:
                return "手眼标定";
            default:
                return "未知";
        }
    }

    private void OnDestroy()
    {
        // 清理事件监听
        if (button != null)
        {
            button.onClick.RemoveListener(OnButtonClick);
        }
    }
}

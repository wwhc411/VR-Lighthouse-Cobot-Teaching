// -------------------- System -------------------- //
using System.Text;
// -------------------- Unity -------------------- //
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
// -------------------- SteamVR -------------------- //
using Valve.VR.InteractionSystem;

/// <summary>
/// VR 按钮速度控制脚本
/// 支持 VR 控制器和鼠标两种交互方式
/// - VR 控制器：通过 SteamVR Hand 系统触发
/// - 鼠标：通过 Unity EventSystem 触发
/// </summary>
[RequireComponent(typeof(Interactable))]
public class VRButtonCheck : MonoBehaviour, IPointerDownHandler, IPointerUpHandler
{
    // ==================== 速度控制参数 ==================== //
    
    [Header("URScript 速度控制参数")]
    [Tooltip("加速度参数 a（单位 m/s² 或 rad/s²）")]
    public string acceleration = "1.0";
    
    [Tooltip("命令有效时长 t（单位：s）")]
    public string time = "0.05";
    
    [Tooltip("速度向量 [vx, vy, vz, rx, ry, rz]（m/s, rad/s）")]
    public string[] speed_param = new string[6] { "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" };
    
    [Header("按钮索引")]
    [Tooltip("本按钮在全局按钮数组中的索引（0..11）")]
    public int index;

    // ==================== 私有变量 ==================== //
    
    // UTF8 编码器
    private UTF8Encoding utf8 = new UTF8Encoding();
    
    // 当前交互的 VR 手柄
    private Hand currentHand;
    
    // VR 按钮是否被按下
    private bool isVRPressed = false;

    // ==================== Unity 生命周期 ==================== //

    protected virtual void Awake()
    {
        // 如果有 Button 组件，注册点击事件
        Button button = GetComponent<Button>();
        if (button)
        {
            button.onClick.AddListener(OnButtonClick);
        }
    }

    protected virtual void OnDisable()
    {
        // 禁用时确保释放按钮状态
        if (isVRPressed || ur_data_processing.UR_Control_Data.button_pressed[index])
        {
            ReleaseButton();
        }
    }

    // ==================== SteamVR Hand 事件 ==================== //

    /// <summary>
    /// VR 手柄开始悬停
    /// </summary>
    protected virtual void OnHandHoverBegin(Hand hand)
    {
        currentHand = hand;
        
        // 显示按钮提示
        if (InputModule.instance != null)
        {
            InputModule.instance.HoverBegin(gameObject);
        }
        ControllerButtonHints.ShowButtonHint(hand, hand.uiInteractAction);
    }

    /// <summary>
    /// VR 手柄结束悬停
    /// </summary>
    protected virtual void OnHandHoverEnd(Hand hand)
    {
        // 如果正在按下状态，先释放
        if (isVRPressed)
        {
            ReleaseButton();
        }
        
        if (InputModule.instance != null)
        {
            InputModule.instance.HoverEnd(gameObject);
        }
        ControllerButtonHints.HideButtonHint(hand, hand.uiInteractAction);
        currentHand = null;
    }

    /// <summary>
    /// VR 手柄悬停更新（每帧调用）
    /// </summary>
    protected virtual void HandHoverUpdate(Hand hand)
    {
        if (hand.uiInteractAction == null) return;

        // 检测 VR 按钮按下
        if (hand.uiInteractAction.GetStateDown(hand.handType))
        {
            PressButton();
            isVRPressed = true;
            
            // 隐藏提示
            ControllerButtonHints.HideButtonHint(hand, hand.uiInteractAction);
        }
        
        // 检测 VR 按钮释放
        if (hand.uiInteractAction.GetStateUp(hand.handType))
        {
            if (isVRPressed)
            {
                ReleaseButton();
                isVRPressed = false;
            }
        }
    }

    // ==================== Unity EventSystem 事件（鼠标/触摸） ==================== //

    /// <summary>
    /// 鼠标/触摸按下
    /// </summary>
    public void OnPointerDown(PointerEventData eventData)
    {
        PressButton();
    }

    /// <summary>
    /// 鼠标/触摸抬起
    /// </summary>
    public void OnPointerUp(PointerEventData eventData)
    {
        ReleaseButton();
    }

    // ==================== Button 组件点击事件 ==================== //

    /// <summary>
    /// Button 组件点击回调（单次点击，非持续按压）
    /// </summary>
    protected virtual void OnButtonClick()
    {
        // 如果需要单次点击也发送命令，可以在这里处理
        // 目前设计为持续按压发送命令
    }

    // ==================== 核心控制逻辑 ==================== //

    /// <summary>
    /// 按下按钮 - 生成速度命令并标记按下状态
    /// </summary>
    private void PressButton()
    {
        // 生成 URScript speedl 命令
        ur_data_processing.UR_Control_Data.aux_command_str = 
            "speedl([" + speed_param[0] + "," + speed_param[1] + "," + speed_param[2] +
            "," + speed_param[3] + "," + speed_param[4] + "," + speed_param[5] + 
            "], a=" + acceleration + ", t=" + time + ")" + "\n";
        
        // 编码为字节数组
        ur_data_processing.UR_Control_Data.command = 
            utf8.GetBytes(ur_data_processing.UR_Control_Data.aux_command_str);
        
        // 标记按钮按下
        ur_data_processing.UR_Control_Data.button_pressed[index] = true;
        
        Debug.Log($"[VRButtonCheck] 按钮 {index} 按下，命令: {ur_data_processing.UR_Control_Data.aux_command_str}");
    }

    /// <summary>
    /// 释放按钮 - 清除按下标志
    /// </summary>
    private void ReleaseButton()
    {
        ur_data_processing.UR_Control_Data.button_pressed[index] = false;
        
        Debug.Log($"[VRButtonCheck] 按钮 {index} 释放");
    }
}

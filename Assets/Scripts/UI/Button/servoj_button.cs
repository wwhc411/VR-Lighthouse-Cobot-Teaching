// System
using System;
// Unity
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Serialization;

/// <summary>
/// 作用：为 Servoj（伺服位置控制）面板的"发送"按钮提供交互：
/// 按下 -> 持续发送 servoj；松开 -> 停止发送。
/// 命令内容来自 main_ui_control 的输入框（x/y/z/rx/ry/rz/t/lookAheadTime/gain）。
/// 
/// servoj 特点：
/// - 提供高频实时控制（125Hz），适合轨迹跟踪和闭环控制
/// - 需要持续发送命令保持运动，停止发送则机器人停止
/// - 比 movej 提供更平滑的实时轨迹跟随能力
/// </summary>
public class servoj_button : MonoBehaviour, IPointerDownHandler, IPointerUpHandler
{
    // 在 Inspector 里将挂有 main_ui_control 的对象拖拽到这里
    [FormerlySerializedAs("ui")]
    public main_ui_control cartesion_btn;

    /// <summary>
    /// 按下：从输入框构造 servoj 命令，并开始持续发送
    /// servoj 需要高频持续发送以维持运动
    /// </summary>
    public void OnPointerDown(PointerEventData eventData)
    {
        if (cartesion_btn != null)
        {
            // 读取 UI 输入并更新 UR_Control_Data.command（UTF-8）
            cartesion_btn.BuildAndSetServojFromInputs();
        }
        // 开始持续发送（控制线程每周期会写一次当前命令）
        ur_data_processing.UR_Control_Data.manual_send_active = true;
    }

    /// <summary>
    /// 松开：停止发送
    /// 停止发送后机器人将停止运动
    /// </summary>
    public void OnPointerUp(PointerEventData eventData)
    {
        ur_data_processing.UR_Control_Data.manual_send_active = false;
    }
}

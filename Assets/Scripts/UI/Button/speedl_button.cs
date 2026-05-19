// System
using System;
// Unity
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Serialization;

// 作用：为 Speedl（Cartesian 速度控制）面板的“发送”按钮提供与 joystick 一致的交互：
// 按下 -> 持续发送 speedl；松开 -> 停止发送。
// 命令内容来自 main_ui_control 的输入框（vx/vy/vz/rx/ry/rz/a/t）。
public class speedl_button : MonoBehaviour, IPointerDownHandler, IPointerUpHandler
{
    // 在 Inspector 里将挂有 main_ui_control 的对象拖拽到这里
    [FormerlySerializedAs("ui")]
    public main_ui_control cartesion_btn;

    // 按下：从输入框构造命令，并开始持续发送
    public void OnPointerDown(PointerEventData eventData)
    {
        if (cartesion_btn != null)
        {
            // 注意：BuildAndSetSpeedlFromInputs 已被移除，改用 servoj
            // 如果需要使用 speedl，请调用 BuildAndSetSpeedl 方法
            Debug.LogWarning("[speedl_button] BuildAndSetSpeedlFromInputs 方法已废弃，请改用 servoj_button");
        }
        // 开始持续发送（控制线程每周期会写一次当前命令）
        ur_data_processing.UR_Control_Data.manual_send_active = true;
    }

    // 松开：停止发送
    public void OnPointerUp(PointerEventData eventData)
    {
        ur_data_processing.UR_Control_Data.manual_send_active = false;
    }
}

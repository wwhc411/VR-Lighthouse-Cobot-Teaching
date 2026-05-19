// Unity
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Serialization;

// Movel 面板按钮：按下开始持续发送 movel，松开停止。
// 命令内容从 main_ui_control 的 UI 输入读取（p[x,y,z,rx,ry,rz], a, v, t=0, r）。
public class movel_button : MonoBehaviour, IPointerDownHandler, IPointerUpHandler
{
    // 在 Inspector 里将挂有 main_ui_control 的对象拖拽到这里
    public main_ui_control movel_btn;

    public void OnPointerDown(PointerEventData eventData)
    {
        if (movel_btn != null)
        {
            // 读取 UI 输入并更新 UR_Control_Data.command（URScript: movel）
            movel_btn.BuildAndSetMoveLFromInputs();
        }
        // 开始持续发送
        ur_data_processing.UR_Control_Data.manual_send_active = true;
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        // 停止持续发送
        ur_data_processing.UR_Control_Data.manual_send_active = false;
    }
}

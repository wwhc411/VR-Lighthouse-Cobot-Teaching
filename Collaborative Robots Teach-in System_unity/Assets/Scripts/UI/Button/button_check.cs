// -------------------- System -------------------- //
using System.Text;
// -------------------- Unity -------------------- //
using UnityEngine.EventSystems;
using UnityEngine;

// 按钮速度控制脚本：
// - 将 UI 上配置的速度向量、加速度 a、时间片 t 拼接为 URScript 的 speedl 指令
// - 按下按钮时写入命令并置位对应的按钮按下标志；抬起时清除标志
// - 控制线程会在有任意按钮被按下时，周期性发送当前 command 实现速度控制
public class button_check: MonoBehaviour, IPointerDownHandler, IPointerUpHandler
{
    // -------------------- String -------------------- //
    // 加速度参数 a（URScript speedl 的 a，单位 m/s^2 / rad/s^2），以字符串形式便于与命令拼接
    public string acceleration = "1.0";
    // 命令有效时长 t（单位：s），控制器会在该时间片内按给定速度运动
    public string time = "0.05";
    // 速度向量 [vx, vy, vz, rx, ry, rz]（m/s, rad/s），作为字符串数组，方便从 UI 输入
    public string[] speed_param      = new string[6] {"0.0", "0.0", "0.0", "0.0","0.0","0.0"};
    // 空速度向量（保留字段，可用于复位或占位）
    public string[] speed_param_null = new string[6] { "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" };
    // -------------------- Int -------------------- //
    // 本按钮在全局按钮数组中的索引（0..11），用于设置对应的按压状态
    public int index;
    // -------------------- UTF8Encoding -------------------- //
    // 用于将 URScript 文本命令转换为 UTF-8 字节数组
    private UTF8Encoding utf8 = new UTF8Encoding();

    // -------------------- Button -> Pressed -------------------- //
    // 按钮按下回调：生成 speedl 指令并标记该按钮为按下
    public void OnPointerDown(PointerEventData eventData)
    {
        // 生成用于 UR 机器人速度控制的辅助命令字符串（URScript: speedl）
        ur_data_processing.UR_Control_Data.aux_command_str = "speedl([" + speed_param[0] +","+  speed_param[1] + "," + speed_param[2]
                                                                   + "," + speed_param[3] + "," + speed_param[4] + "," + speed_param[5] + "], a =" + acceleration + ", t =" + time + ")" + "\n";
        // 将字符串命令编码为字节数组，供 TCP 发送
        ur_data_processing.UR_Control_Data.command = utf8.GetBytes(ur_data_processing.UR_Control_Data.aux_command_str);
        // 设置对应按钮按压标志为 true，用于上层逻辑统计是否有按钮被按下
        ur_data_processing.UR_Control_Data.button_pressed[index] = true;
    }

    // -------------------- Button -> Un-Pressed -------------------- //
    // 按钮抬起回调：清除该按钮的按下标志
    public void OnPointerUp(PointerEventData eventData)
    {
        // 清除对应按钮按压标志，上层逻辑会在全部按钮松开时停止发送速度命令
        ur_data_processing.UR_Control_Data.button_pressed[index] = false;
    }
}

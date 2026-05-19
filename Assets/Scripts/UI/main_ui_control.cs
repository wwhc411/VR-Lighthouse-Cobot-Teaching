// System 
using System;
using System.Text;
using System.Globalization; // 用于 InvariantCulture 格式化/解析
using System.Collections;   // 用于协程 IEnumerator
// Unity
using UnityEngine;
using UnityEngine.UI;
// TM 
using TMPro;

/// <summary>
/// UR 机械臂主 UI 控制脚本
/// 
/// 功能模块:
/// 1. 连接管理: IP地址配置, TCP连接建立/断开, 状态指示
/// 2. 数据显示: 实时显示机械臂 TCP 位姿(笛卡尔坐标+关节角度)
/// 3. 命令生成: 构建 URScript 命令(servoj, movel, speedl)
/// 4. 坐标转换: 可选将 SteamVR 坐标转换为 UR 基座坐标
/// 5. 面板管理: 控制各功能面板的显示/隐藏
/// 
/// 支持的控制模式:
/// - Servoj: 伺服位置控制模式(x,y,z,rx,ry,rz + 实时控制参数) [主要模式]
/// - Movej: 位姿控制模式(x,y,z,rx,ry,rz + 运动参数)
/// - Speedl: 速度控制模式(vx,vy,vz,rx,ry,rz) [保留功能]
/// 
/// MoveL 坐标输入模式:
/// - 直接输入: UR 基座坐标系(米, 弧度)
/// - SteamVR 输入: 勾选 Toggle 后自动调用坐标转换
/// 
/// 数据流:
///   UI输入 → [可选:坐标转换] → URScript生成 → TCP发送 → 机械臂执行
/// 
/// 相关文档: 完整流程说明.md - 阶段3: 机械臂命令生成
/// </summary>
public class main_ui_control : MonoBehaviour
{
    // -------------------- GameObject -------------------- //
    // 相机对象：用于在按钮回调中切换视角位置与角度
    public GameObject camera_obj;
    // -------------------- Image -------------------- //
    // UI面板：连接/诊断/摇杆/Speedl/Movel 五个面板，以及连接状态小圆点图片
    public Image connection_panel_img, diagnostic_panel_img, joystick_panel_img, speedl_panel_img, movel_panel_img, calibration_panel_img;
    public Image connection_info_img;
    // -------------------- TMP_InputField -------------------- //
    // 机器人IP输入框：将其内容同步到 TCP 客户端的读写两端
    public TMP_InputField ip_address_txt;

    // Servoj（伺服位置控制）输入框：x, y, z, rx, ry, rz（笛卡尔位姿）
    // 注意：单位为 x/y/z: m；rx/ry/rz: rad
    public TMP_InputField servoj_xInput, servoj_yInput, servoj_zInput;
    public TMP_InputField servoj_rxInput, servoj_ryInput, servoj_rzInput;

    // 如果勾选：把UI输入解释为SteamVR坐标系下的目标（位置单位：mm；姿态：轴角rad）
    // 将通过手眼标定结果转换到UR基座坐标系再发送servoj
    public Toggle servoj_inputIsSteamVrToggle;

    // -------------------- Inspector 直接输入: Tracker 位姿（Servoj）-------------------- //
    [Header("Servoj Tracker 位姿输入 (Inspector)")]
    [Tooltip("勾选后使用 Inspector 中的 Tracker 位姿，忽略 UI 输入框")]
    public bool useInspectorTrackerPose = false;
    
    [Tooltip("Tracker 位姿输入是否为 SteamVR 坐标系（勾选=SteamVR，不勾选=UR基座）")]
    public bool trackerPoseIsSteamVr = true;
    
    [Header("位置 (SteamVR时单位:mm, UR基座时单位:m)")]
    public float tracker_x = 0f;
    public float tracker_y = 0f;
    public float tracker_z = 0f;
    
    [Header("姿态 (轴角表示, 单位:弧度)")]
    public float tracker_rx = 0f;
    public float tracker_ry = 0f;
    public float tracker_rz = 0f;

    [Header("Servoj 实时控制参数 (Inspector 可调)")]
    [Tooltip("时间步长(s), 必须与TCP发送频率匹配！125Hz=0.008, 90Hz=0.0111, 60Hz=0.0167")]
    [Range(0.002f, 0.2f)]
    public float inspector_servoj_timeStep = 0.008f;  // 默认125Hz，匹配TCP发送频率
    
    [Tooltip("前瞻时间(s), 范围0.03-0.2, 用于轨迹平滑，推荐0.03-0.15")]
    [Range(0.03f, 0.2f)]
    public float inspector_servoj_lookAheadTime = 0.1f;
    
    [Tooltip("控制增益, 范围100-2000, 值越小响应越快，推荐200-600")]
    [Range(100f, 2000f)]
    public float inspector_servoj_gain = 300f;

    [Header("Servoj 连续控制")]
    [Tooltip("右键菜单启动后的持续控制状态")]
    public bool servojContinuousSendActive = false;
    
    [Tooltip("连续发送的频率（Hz），必须与TimeStep和TCP频率匹配！推荐125Hz")]
    public float servojSendFrequency = 125f;  // 匹配TCP线程的125Hz发送频率

    // MoveL（位姿控制）专用输入框：x, y, z, rx, ry, rz, a, v, r
    // 注意：单位为 x/y/z: m；rx/ry/rz: rad；a: m/s^2；v: m/s；r: m
    public TMP_InputField moveL_xInput, moveL_yInput, moveL_zInput;
    public TMP_InputField moveL_rxInput, moveL_ryInput, moveL_rzInput;
    public TMP_InputField moveL_accelerationInput, moveL_linearSpeedInput, moveL_blendRadiusInput;

    // 如果勾选：把UI输入解释为SteamVR坐标系下的目标（位置单位：mm；姿态：轴角rad）
    // 将通过手眼标定结果转换到UR基座坐标系再发送movel
    public Toggle moveL_inputIsSteamVrToggle;

    // Servoj（伺服位置控制）专用参数输入框
    // a：加速度（rad/s^2），默认为0表示无限制
    // v：速度（rad/s），默认为0表示无限制
    // t：时间步长（S），通常为0.008秒（125Hz控制频率）
    // lookAheadTime：前瞻时间（S），范围（0.03-0.2），用于轨迹平滑
    // gain：控制增益，范围（100-2000），值越小响应越快
    public TMP_InputField servoj_accelerationInput, servoj_velocityInput;
    public TMP_InputField servoj_timeStepInput, servoj_lookAheadTimeInput, servoj_gainInput;
    
    // Servoj 默认参数（与TCP发送频率125Hz匹配）
    private string defaultServojAcceleration = "0.001";        // 关节加速度
    private string defaultServojVelocity = "0.01";             // 关节速度
    private string defaultServojTimeStep = "0.008";            // 125Hz控制频率，匹配TCP线程
    private string defaultServojLookAheadTime = "0.1";         // 中等前瞻时间
    private string defaultServojGain = "300";                  // 中等控制增益

    // 可选参数输入框（已弃用或用于其他功能）
    public TMP_InputField accelerationInput, timeInput;
    private string defaultTime = "0.03";           // 默认时间片（用于脉冲发送等）
    private string defaultAcceleration = "0.15";   // 默认加速度（用于其他命令）
    
    // movel 专用默认参数
    private string defaultLinearSpeed = "0.25";    // movel 线速度（m/s）
    private string defaultBlendRadius = "0.0";     // movel 圆弧过渡半径（m），0表示无过渡

    // -------------------- Float -------------------- //
    // 面板初始偏移量：用来把未激活的面板移到屏幕外（简单隐藏）
    private float ex_param = 100f;
    // -------------------- TextMeshProUGUI -------------------- //
    // 诊断面板显示字段：TCP 笛卡尔坐标、姿态（欧拉角/旋转向量转角度）、六关节角度
    public TextMeshProUGUI position_x_txt, position_y_txt, position_z_txt;
    public TextMeshProUGUI position_rx_txt, position_ry_txt, position_rz_txt;
    public TextMeshProUGUI position_j1_txt, position_j2_txt, position_j3_txt;
    public TextMeshProUGUI position_j4_txt, position_j5_txt, position_j6_txt;
    public TextMeshProUGUI connectionInfo_txt;
    // -------------------- UTF8Encoding -------------------- //
    // UTF-8 编码器：用于把 URScript 文本命令转为字节数组发送
    private UTF8Encoding utf8 = new UTF8Encoding();

    // ------------------------------------------------------------------------------------------------------------------------ //
    // ------------------------------------------------ INITIALIZATION {START} ------------------------------------------------ //
    // ------------------------------------------------------------------------------------------------------------------------ //
    void Start()
    {
        // 连接状态指示（图片）：初始为红色（断开）
        connection_info_img.GetComponent<Image>().color = new Color32(255, 0, 48, 50);
        // 连接状态指示（文字）：初始为 Disconnect
        connectionInfo_txt.text = "Disconnect";

        // 面板初始化：将三个面板移动到屏幕外，默认只显示需要的面板
        connection_panel_img.transform.localPosition = new Vector3(1215f + (ex_param), 0f, 0f);
        diagnostic_panel_img.transform.localPosition = new Vector3(780f + (ex_param), 0f, 0f);
        joystick_panel_img.transform.localPosition = new Vector3(1550f + (ex_param), 0f, 0f);
        speedl_panel_img.transform.localPosition = new Vector3(1880f + (ex_param), 0f, 0f);
        movel_panel_img.transform.localPosition = new Vector3(2055f + (ex_param), 0f, 0f);
        calibration_panel_img.transform.localPosition = new Vector3(2320f + (ex_param), 0f, 0f);

        // 位置显示（笛卡尔）：X/Y/Z 初始为 0.00
        position_x_txt.text = "0.00";
        position_y_txt.text = "0.00";
        position_z_txt.text = "0.00";
        // 姿态显示（旋转向量转角度后显示为欧拉角）：RX/RY/RZ 初始为 0.00
        position_rx_txt.text = "0.00";
        position_ry_txt.text = "0.00";
        position_rz_txt.text = "0.00";
        // 关节角显示：J1~J6 初始为 0.00
        position_j1_txt.text = "0.00";
        position_j2_txt.text = "0.00";
        position_j3_txt.text = "0.00";
        position_j4_txt.text = "0.00";
        position_j5_txt.text = "0.00";
        position_j6_txt.text = "0.00";

        // 机器人 IP 地址：默认填充
        ip_address_txt.text = "192.168.1.103";

        // 辅助初始命令：将速度向量设为 0，并指定加速度 a 与时间片 t（URScript: speedl）
        // 命令（字符串）
        ur_data_processing.UR_Control_Data.aux_command_str = "speedl([0.0,0.0,0.0,0.0,0.0,0.0], a = 0.05, t = 0.03)" + "\n";
        // 将字符串命令转为字节数组，供 TCP 发送
        ur_data_processing.UR_Control_Data.command = utf8.GetBytes(ur_data_processing.UR_Control_Data.aux_command_str);

        // *****这两行分别做两件事：先构造一条 URScript 的速度控制指令字符串（speedl），再把它转成字节数组以便通过 TCP 发送给机器人。

        // 1.构造指令字符串
        // speedl([0.0,0.0,0.0,0.0,0.0,0.0], a = 0.15, t = 0.03)\n
        // 含义：
        // speedl 是 URScript 的“笛卡尔速度控制”指令。
        // 参数向量 [vx, vy, vz, rx, ry, rz] 分别是 TCP 在基坐标系下的线速度(m/s)与角速度(rad/s)。这里全 0，表示不移动，相当于初始化为“零速度”。
        // a = 0.15 是加速度限制（线与角的统一标量，加速度/角加速度的单位分别为 m/s^2 与 rad/s^2，UR 会按约定使用）。
        // t = 0.03 表示该速度命令生效的时间片为 0.03 秒；若要持续运动，需要在控制循环中反复发送更新的 speedl。
        // 末尾 \n 是换行符，用于结束一条 URScript 语句，便于控制器解析执行。
        // 这条指令作为“初始命令”放入全局 aux_command_str，实际控制时按钮脚本会根据 UI 参数动态重写它。
        
        // 2.转为字节数组
        // utf8.GetBytes(ur_data_processing.UR_Control_Data.aux_command_str)
        // 将上面的字符串用 UTF-8 编码为字节数组，赋给 UR_Control_Data.command，以便控制线程通过 NetworkStream.Write(...) 发往机器人控制端口 30003。
        // UR 控制器接受 ASCII/UTF-8 文本命令，带换行即可正常解析。
    }

    /// <summary>
    /// OnDisable: 停止所有协程，防止退出 Play 模式时崩溃
    /// </summary>
    void OnDisable()
    {
        StopAllCoroutines();
    }

    // ------------------------------------------------------------------------------------------------------------------------ //
    // ------------------------------------------------ MAIN FUNCTION {Cyclic} ------------------------------------------------ //
    // ------------------------------------------------------------------------------------------------------------------------ //
    // 固定周期更新：
    // 1) 同步 UI 中的 IP 地址到 读/写 两个 TCP 客户端
    // 2) 根据 connect/disconnect 标志更新连接状态指示
    // 3) 将 UR_Stream_Data 的实时值渲染到诊断面板
    void FixedUpdate()
    {
        // 机器人 IP（读通道）-> 同步到数据流读取模块（30013端口）
        ur_data_processing.UR_Stream_Data.ip_address = ip_address_txt.text;
        // 机器人 IP（写通道）-> 同步到控制发送模块（30003端口）
        ur_data_processing.UR_Control_Data.ip_address = ip_address_txt.text;

        // ------------------------ Connection Information ------------------------//
        // 若按下连接/断开按钮，改变连接状态图标颜色与提示文字
        if (ur_data_processing.GlobalVariables_Main_Control.connect == true)
        {
            // 绿色：已连接
            connection_info_img.GetComponent<Image>().color = new Color32(135, 255, 0, 50);
            connectionInfo_txt.text = "Connect";
        }
        else if(ur_data_processing.GlobalVariables_Main_Control.disconnect == true)
        {
            // 红色：断开
            connection_info_img.GetComponent<Image>().color = new Color32(255, 0, 48, 50);
            connectionInfo_txt.text = "Disconnect";
        }

        // ------------------------ Cyclic read parameters {diagnostic panel} ------------------------ //
        // 将读取到的实时数据（米/弧度）转换为显示单位并保留两位小数：
        // - 笛卡尔位置：米 -> 毫米（乘以1000）
        // - 旋转/关节角：弧度 -> 角度（乘以 180/π）
        // 笛卡尔位置 X..Z（mm）
        position_x_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.C_Position[0] * (1000f), 2)).ToString();
        position_y_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.C_Position[1] * (1000f), 2)).ToString();
        position_z_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.C_Position[2] * (1000f), 2)).ToString();
        // 姿态（RX..RZ，deg）
        position_rx_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.C_Orientation[0] * (180 / Math.PI), 2)).ToString();
        position_ry_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.C_Orientation[1] * (180 / Math.PI), 2)).ToString();
        position_rz_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.C_Orientation[2] * (180 / Math.PI), 2)).ToString();
        // 关节角（J1..J6，deg）
        position_j1_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.J_Orientation[0] * (180 / Math.PI), 2)).ToString();
        position_j2_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.J_Orientation[1] * (180 / Math.PI), 2)).ToString();
        position_j3_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.J_Orientation[2] * (180 / Math.PI), 2)).ToString();
        position_j4_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.J_Orientation[3] * (180 / Math.PI), 2)).ToString();
        position_j5_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.J_Orientation[4] * (180 / Math.PI), 2)).ToString();
        position_j6_txt.text = ((float)Math.Round(ur_data_processing.UR_Stream_Data.J_Orientation[5] * (180 / Math.PI), 2)).ToString();
    }

    // ------------------------------------------------------------------------------------------------------------------------//
    // -------------------------------------------------------- FUNCTIONS -----------------------------------------------------//
    // ------------------------------------------------------------------------------------------------------------------------//

    // -------------------- Send Once (Pulse only) -------------------- //
    /// <summary>
    /// 点击发送单次 MoveJ 命令：
    /// 1. 从输入框构造一次 movej 命令
    /// 2. 短暂置位发送开关（脉冲发送）
    /// 3. 发送结束后自动关闭发送标志
    /// </summary>
    public void TaskOnClick_SendOnceFromInputs()
    {
        BuildAndSetMoveLFromInputs();

        // 取 t 作为脉冲宽度的上限，给线程调度留裕量（2ms~200ms 之间夹紧）
        double t = timeInput != null && !string.IsNullOrWhiteSpace(timeInput.text)
            ? ParseInvariant(timeInput.text)
            : ParseInvariant(defaultTime);

        float pulseSeconds = Mathf.Clamp((float)t, 0.02f, 0.20f);
        StartCoroutine(SendPulseCoroutine(pulseSeconds));
    }

    /// <summary>
    /// 脉冲发送协程：
    /// 在指定时间内置位 manual_send_active，发送完成后自动关闭
    /// 避免被按钮状态机每帧覆盖
    /// </summary>
    /// <param name="seconds">脉冲持续时间（秒）</param>
    private IEnumerator SendPulseCoroutine(float seconds)
    {
        // 使用独立的手动发送标志，避免被按钮状态机每帧覆盖
        ur_data_processing.UR_Control_Data.manual_send_active = true;
        yield return new WaitForSeconds(seconds);

        // 脉冲结束后关闭手动发送标志（仅保留"点一下就发一小段"效果）
        ur_data_processing.UR_Control_Data.manual_send_active = false;
    }

    //********************************cartesian_panel----servoj**************************************//
    /// <summary>
    /// 基于 UI 输入或 Inspector 输入，构造 servoj 命令（伺服位置控制）
    /// servoj 用于实时控制机器人到指定笛卡尔位置，提供更高频率的实时控制
    /// 适合轨迹跟踪和闭环控制场景
    /// 
    /// 支持两种输入模式：
    /// 1. UI 输入框模式（useInspectorTrackerPose = false）
    ///    - 从 UI 输入框读取位姿数据
    ///    - 可选择是否为 SteamVR 坐标系（通过 Toggle）
    /// 2. Inspector 直接输入模式（useInspectorTrackerPose = true）
    ///    - 从 Inspector 面板读取 tracker_x/y/z/rx/ry/rz
    ///    - 坐标系由 trackerPoseIsSteamVr 标志控制
    ///    - 控制参数使用 Inspector 滑块值（实时可调）
    /// 
    /// 坐标转换：
    /// - 若 inputIsSteamVr = true，则调用手眼标定转换（SteamVR → UR基座）
    /// - 否则直接使用 UR 基座坐标
    /// </summary>
    public void BuildAndSetServojFromInputs()
    {
        double x, y, z, rx, ry, rz;
        bool inputIsSteamVr = false;

        // 判断输入来源：Inspector 还是 UI 输入框
        if (useInspectorTrackerPose)
        {
            // 模式1: 使用 Inspector 中的 Tracker 位姿
            x  = tracker_x;
            y  = tracker_y;
            z  = tracker_z;
            rx = tracker_rx;
            ry = tracker_ry;
            rz = tracker_rz;
            
            inputIsSteamVr = trackerPoseIsSteamVr;
            
            Debug.Log($"[Servoj] 使用 Inspector 输入模式");
            Debug.Log($"  输入坐标系: {(inputIsSteamVr ? "SteamVR" : "UR基座")}");
        }
        else
        {
            // 模式2: 使用 UI 输入框
            x  = ParseInvariant(servoj_xInput?.text);
            y  = ParseInvariant(servoj_yInput?.text);
            z  = ParseInvariant(servoj_zInput?.text);
            rx = ParseInvariant(servoj_rxInput?.text);
            ry = ParseInvariant(servoj_ryInput?.text);
            rz = ParseInvariant(servoj_rzInput?.text);
            
            inputIsSteamVr = (servoj_inputIsSteamVrToggle != null) && servoj_inputIsSteamVrToggle.isOn;
            
            Debug.Log($"[Servoj] 使用 UI 输入框模式");
        }

        // 读取 servoj 专用参数
        double a = servoj_accelerationInput != null && !string.IsNullOrWhiteSpace(servoj_accelerationInput.text)
            ? ParseInvariant(servoj_accelerationInput.text)
            : ParseInvariant(defaultServojAcceleration);

        double v = servoj_velocityInput != null && !string.IsNullOrWhiteSpace(servoj_velocityInput.text)
            ? ParseInvariant(servoj_velocityInput.text)
            : ParseInvariant(defaultServojVelocity);

        // Inspector 动态参数优先，UI输入框次之，最后才用硬编码默认值
        double t = useInspectorTrackerPose 
            ? inspector_servoj_timeStep  // Inspector 模式：使用 Inspector 滑块值
            : (servoj_timeStepInput != null && !string.IsNullOrWhiteSpace(servoj_timeStepInput.text)
                ? ParseInvariant(servoj_timeStepInput.text)
                : ParseInvariant(defaultServojTimeStep));

        double lookAheadTime = useInspectorTrackerPose
            ? inspector_servoj_lookAheadTime  // Inspector 模式：使用 Inspector 滑块值
            : (servoj_lookAheadTimeInput != null && !string.IsNullOrWhiteSpace(servoj_lookAheadTimeInput.text)
                ? ParseInvariant(servoj_lookAheadTimeInput.text)
                : ParseInvariant(defaultServojLookAheadTime));

        double gain = useInspectorTrackerPose
            ? inspector_servoj_gain  // Inspector 模式：使用 Inspector 滑块值
            : (servoj_gainInput != null && !string.IsNullOrWhiteSpace(servoj_gainInput.text)
                ? ParseInvariant(servoj_gainInput.text)
                : ParseInvariant(defaultServojGain));

        // 若输入为SteamVR位姿，则先进行坐标变换（SteamVR -> UR基座）
        if (inputIsSteamVr)
        {
            // UI中：位置来自SteamVR原始日志，单位为mm；姿态为轴角(rad)
            var posSteamVr_mm = new Vector3((float)x, (float)y, (float)z);
            var rotSteamVr_r  = new Vector3((float)rx, (float)ry, (float)rz);

            Debug.Log($"[Servoj坐标转换] 输入 SteamVR 位姿:");
            Debug.Log($"  位置(mm): ({posSteamVr_mm.x:F3}, {posSteamVr_mm.y:F3}, {posSteamVr_mm.z:F3})");
            Debug.Log($"  姿态(rad): ({rotSteamVr_r.x:F4}, {rotSteamVr_r.y:F4}, {rotSteamVr_r.z:F4})");

            // 使用手眼标定结果做变换
            handeye.SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                posSteamVr_mm, rotSteamVr_r, posInMillimeters: true,
                out Vector3 posUr_m, out Vector3 rotUr_r);

            Debug.Log($"[Servoj坐标转换] 输出 UR Base 位姿:");
            Debug.Log($"  位置(m): ({posUr_m.x:F4}, {posUr_m.y:F4}, {posUr_m.z:F4})");
            Debug.Log($"  姿态(rad): ({rotUr_r.x:F4}, {rotUr_r.y:F4}, {rotUr_r.z:F4})");

            // 覆盖为UR基座系下（m,rad），直接用于URScript servoj
            x  = posUr_m.x;
            y  = posUr_m.y;
            z  = posUr_m.z;
            rx = rotUr_r.x;
            ry = rotUr_r.y;
            rz = rotUr_r.z;
        }
        else
        {
            Debug.Log($"[Servoj] 直接使用 UR 基座坐标:");
            Debug.Log($"  位置(m): ({x:F4}, {y:F4}, {z:F4})");
            Debug.Log($"  姿态(rad): ({rx:F4}, {ry:F4}, {rz:F4})");
        }

        BuildAndSetServoj(x, y, z, rx, ry, rz, a, v, t, lookAheadTime, gain);
    }

    /// <summary>
    /// 基于 MoveL 专用输入框，构造 movej 命令（关节空间插值运动）
    /// 
    /// 功能说明：
    /// - movej 使用关节空间插值，轨迹在笛卡尔空间可能不是直线
    /// - 适合点到点快速运动，不要求笛卡尔空间轨迹精度
    /// 
    /// 参数来源：
    /// - 位姿: moveL_xInput ~ moveL_rzInput
    /// - 加速度: moveL_accelerationInput (默认0.15 rad/s²)
    /// - 速度: moveL_linearSpeedInput (默认0.25 rad/s)
    /// - 混合半径: moveL_blendRadiusInput (默认0.0 m)
    /// - 时间: 固定为0，使a和v生效
    /// 
    /// 坐标转换：
    /// - 若 moveL_inputIsSteamVrToggle = true，调用手眼标定转换
    /// - 否则直接使用 UR 基座坐标
    /// </summary>
    public void BuildAndSetMoveLFromInputs()
    {
        double x  = ParseInvariant(moveL_xInput?.text);
        double y  = ParseInvariant(moveL_yInput?.text);
        double z  = ParseInvariant(moveL_zInput?.text);
        double rx = ParseInvariant(moveL_rxInput?.text);
        double ry = ParseInvariant(moveL_ryInput?.text);
        double rz = ParseInvariant(moveL_rzInput?.text);

        // a：若提供覆盖输入则使用；否则使用默认
        double a = moveL_accelerationInput != null && !string.IsNullOrWhiteSpace(moveL_accelerationInput.text)
            ? ParseInvariant(moveL_accelerationInput.text)
            : ParseInvariant(defaultAcceleration);

        // v：若提供覆盖输入则使用；否则使用默认
        double v = moveL_linearSpeedInput != null && !string.IsNullOrWhiteSpace(moveL_linearSpeedInput.text)
            ? ParseInvariant(moveL_linearSpeedInput.text)
            : ParseInvariant(defaultLinearSpeed);
        // r：blend 半径；若提供覆盖输入则使用；否则使用默认
        double r = moveL_blendRadiusInput != null && !string.IsNullOrWhiteSpace(moveL_blendRadiusInput.text)
            ? ParseInvariant(moveL_blendRadiusInput.text)
            : ParseInvariant(defaultBlendRadius);

        // 根据 UR 文档：设置 t>0 会忽略 a 和 v。此处按需固定 t = 0 以使 a/v 生效。
        double tForMoveL = 0.0;

        // 若勾选“输入为SteamVR位姿”，则先进行坐标变换（SteamVR -> UR基座）
        bool inputIsSteamVr = (moveL_inputIsSteamVrToggle != null) && moveL_inputIsSteamVrToggle.isOn;
        if (inputIsSteamVr)
        {
            // UI中：位置来自SteamVR原始日志，单位为mm；姿态为轴角(rad)
            var posSteamVr_mm = new Vector3((float)x, (float)y, (float)z);
            var rotSteamVr_r  = new Vector3((float)rx, (float)ry, (float)rz);

            Debug.Log($"[MoveJ坐标转换] 输入 SteamVR 位姿:");
            Debug.Log($"  位置(mm): ({posSteamVr_mm.x:F3}, {posSteamVr_mm.y:F3}, {posSteamVr_mm.z:F3})");
            Debug.Log($"  姿态(rad): ({rotSteamVr_r.x:F4}, {rotSteamVr_r.y:F4}, {rotSteamVr_r.z:F4})");

            // 使用手眼标定结果做变换
            handeye.SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                posSteamVr_mm, rotSteamVr_r, posInMillimeters: true,
                out Vector3 posUr_m, out Vector3 rotUr_r);

            Debug.Log($"[MoveJ坐标转换] 输出 UR Base 位姿:");
            Debug.Log($"  位置(m): ({posUr_m.x:F4}, {posUr_m.y:F4}, {posUr_m.z:F4})");
            Debug.Log($"  姿态(rad): ({rotUr_r.x:F4}, {rotUr_r.y:F4}, {rotUr_r.z:F4})");

            // 覆盖为UR基座系下（m,rad），直接用于URScript movej p[...]
            x  = posUr_m.x;
            y  = posUr_m.y;
            z  = posUr_m.z;
            rx = rotUr_r.x;
            ry = rotUr_r.y;
            rz = rotUr_r.z;
        }

        BuildAndSetMoveL(x, y, z, rx, ry, rz, a, v, tForMoveL, r);
    }


    /// <summary>
    /// 构建 URScript servoj 命令并设置到发送缓冲区（笛卡尔空间到关节空间转换后执行伺服运动）
    /// 
    /// 功能描述：
    ///   servoj函数用于实时控制机器人到指定笛卡尔位置，使用逆运动学转换
    ///   注意：相比movej，servoj提供更高频率的实时控制，适合轨迹跟踪和闭环控制
    /// 
    /// URScript 命令格式:
    ///   servoj(get_inverse_kin(p[x,y,z,rx,ry,rz], qnear=[j0,j1,j2,j3,j4,j5]), a, v, t, lookahead_time, gain)
    /// 
    /// 参数说明:
    /// - p[x,y,z,rx,ry,rz]: 目标位姿（笛卡尔空间）
    ///   * x,y,z: 位置(米)
    ///   * rx,ry,rz: 姿态, 轴角表示(弧度)
    /// - qnear=[j0..j5]: 当前关节角度(rad)，用于逆运动学求解参考
    /// - a: 关节加速度(rad/s²), 0表示无限制
    /// - v: 关节速度(rad/s), 0表示无限制
    /// - t: 时间步长(s), 必须与TCP发送频率匹配！125Hz=0.008, 90Hz=0.0111, 60Hz=0.0167
    /// - lookahead_time: 前瞻时间(s), 范围(0.03-0.2), 用于轨迹平滑，推荐0.03-0.15
    /// - gain: 控制增益, 范围(100-2000), 值越小响应越快，推荐200-600
    /// 
    /// 注意: 
    /// - 所有数值使用 InvariantCulture 格式化, 确保小数点为 "."
    /// - TimeStep 必须与TCP发送频率严格匹配，否则会导致机器人卡顿或拒绝执行
    /// 
    /// 相关文档: 完整流程说明.md - 阶段3.3 URScript命令生成
    /// </summary>
    /// <param name="x">目标 X 坐标(m)</param>
    /// <param name="y">目标 Y 坐标(m)</param>
    /// <param name="z">目标 Z 坐标(m)</param>
    /// <param name="rx">目标 RX 姿态(rad)</param>
    /// <param name="ry">目标 RY 姿态(rad)</param>
    /// <param name="rz">目标 RZ 姿态(rad)</param>
    /// <param name="acceleration">关节加速度(rad/s²), 0=无限制</param>
    /// <param name="velocity">关节速度(rad/s), 0=无限制</param>
    /// <param name="timeStep">时间步长(s), 必须与TCP发送频率匹配</param>
    /// <param name="lookAheadTime">前瞻时间(s), 范围0.03-0.2</param>
    /// <param name="gain">控制增益, 范围100-2000</param>
    public void BuildAndSetServoj(double x, double y, double z,
                                   double rx, double ry, double rz,
                                   double acceleration, double velocity,
                                   double timeStep, double lookAheadTime, double gain)
    {
        // 获取当前六个关节角度作为qnear参考（单位：弧度）
        double j0 = ur_data_processing.UR_Stream_Data.J_Orientation[0];
        double j1 = ur_data_processing.UR_Stream_Data.J_Orientation[1];
        double j2 = ur_data_processing.UR_Stream_Data.J_Orientation[2];
        double j3 = ur_data_processing.UR_Stream_Data.J_Orientation[3];
        double j4 = ur_data_processing.UR_Stream_Data.J_Orientation[4];
        double j5 = ur_data_processing.UR_Stream_Data.J_Orientation[5];

        // 构建servoj命令，使用get_inverse_kin将笛卡尔坐标转换为关节角度
        // 格式: servoj(q, a, v, t, lookahead_time, gain)
        string cmd = string.Format(CultureInfo.InvariantCulture,
            "servoj(get_inverse_kin(p[{0:0.0000},{1:0.0000},{2:0.0000},{3:0.0000},{4:0.0000},{5:0.0000}], qnear=[{6:0.0000},{7:0.0000},{8:0.0000},{9:0.0000},{10:0.0000},{11:0.0000}]), {12:0.0000}, {13:0.0000}, {14:0.0000}, {15:0.0000}, {16:0.0000})\n",
            x, y, z, rx, ry, rz, 
            j0, j1, j2, j3, j4, j5,
            acceleration, velocity, timeStep, lookAheadTime, gain);

        // URScript命令日志（简化输出）
        Debug.Log($"[VisualServo] URScript: {cmd.Trim()}");

        // 更新全局命令字符串与 UTF-8 字节缓冲
        ur_data_processing.UR_Control_Data.aux_command_str = cmd;
        ur_data_processing.UR_Control_Data.command = utf8.GetBytes(cmd);
    }

    //Speedl 组装：速度向量 [vx,vy,vz,rx,ry,rz]，可指定 a（m/s^2）、t（s，可选，>0 则按定时到达）
    /// <summary>
    /// 构建 URScript speedl 命令并设置到发送缓冲区（笛卡尔速度控制）
    /// 
    /// 功能描述：
    ///   speedl 用于控制TCP以指定的笛卡尔速度运动
    ///   适合需要持续速度控制的场景，如手动操作、轨迹跟踪等
    /// 
    /// URScript 命令格式:
    ///   speedl([vx,vy,vz,rx,ry,rz], a = 加速度, t = 时间)
    /// 
    /// 参数说明:
    /// - [vx,vy,vz]: 线速度向量(m/s)
    /// - [rx,ry,rz]: 角速度向量(rad/s)
    /// - a: 加速度(m/s²和rad/s²的统一标量)
    /// - t: 持续时间(s)，指定该速度命令的生效时间
    /// 
    /// 注意: 需要在控制循环中持续发送以维持运动
    /// </summary>
    /// <param name="vx">X方向线速度(m/s)</param>
    /// <param name="vy">Y方向线速度(m/s)</param>
    /// <param name="vz">Z方向线速度(m/s)</param>
    /// <param name="rx">X轴角速度(rad/s)</param>
    /// <param name="ry">Y轴角速度(rad/s)</param>
    /// <param name="rz">Z轴角速度(rad/s)</param>
    /// <param name="acceleration">加速度(m/s²)</param>
    /// <param name="time">持续时间(s)</param>
    public void BuildAndSetSpeedl(double vx, double vy, double vz,
                                  double rx, double ry, double rz,
                                  double acceleration, double time)
    {
        string cmd = string.Format(CultureInfo.InvariantCulture,
            "speedl([{0},{1},{2},{3},{4},{5}], a = {6}, t = {7})\n",
            vx, vy, vz, rx, ry, rz, acceleration, time);

        // 更新全局命令字符串与 UTF-8 字节缓冲
        ur_data_processing.UR_Control_Data.aux_command_str = cmd;
        ur_data_processing.UR_Control_Data.command = utf8.GetBytes(cmd);
    }

    /// <summary>
    /// 构建 URScript movej 命令并设置到发送缓冲区（笛卡尔空间到关节空间转换后执行关节运动）
    /// 
    /// 功能描述：
    ///   movej 函数用于点到点运动控制，机器人会以关节插值方式移动到目标位置。
    ///   注意：运动轨迹在关节空间内是线性的，但在笛卡尔空间可能不是直线。
    /// 
    /// URScript 命令格式:
    ///   
    /// )
    /// 
    /// 参数说明:
    /// - p[x,y,z,rx,ry,rz]: 目标位姿（笛卡尔空间）
    ///   * x,y,z: 位置(米)
    ///   * rx,ry,rz: 姿态, 轴角表示(弧度)
    /// - qnear=[j0..j5]: 当前关节角度(rad)，用于逆运动学求解参考
    /// - a: 关节加速度(rad/s²), 范围通常为0.1-10
    /// - v: 关节速度(rad/s), 范围通常为0.1-3.14
    /// - t: 运动时间(s), 如果指定时间>0，则忽略速度和加速度参数
    /// - r: 混合半径(m), 用于平滑连接多个运动指令，范围0-0.1
    /// 
    /// 注意: 所有数值使用 InvariantCulture 格式化, 确保小数点为 "."
    /// 
    /// 相关文档: 完整流程说明.md - 阶段3.3 URScript命令生成
    /// </summary>
    /// <param name="x">目标 X 坐标(m)</param>
    /// <param name="y">目标 Y 坐标(m)</param>
    /// <param name="z">目标 Z 坐标(m)</param>
    /// <param name="rx">目标 RX 姿态(rad)</param>
    /// <param name="ry">目标 RY 姿态(rad)</param>
    /// <param name="rz">目标 RZ 姿态(rad)</param>
    /// <param name="acceleration">关节加速度(rad/s²)</param>
    /// <param name="linearSpeed">关节速度(rad/s)</param>
    /// <param name="time">时间(s), >0 时忽略 a 和 v</param>
    /// <param name="blendRadius">混合半径(m)</param>
    public void BuildAndSetMoveL(double x, double y, double z,
                                 double rx, double ry, double rz,
                                 double acceleration, double linearSpeed, double time, double blendRadius)
    {
        // 获取当前六个关节角度作为qnear参考（单位：弧度）
        double j0 = ur_data_processing.UR_Stream_Data.J_Orientation[0];
        double j1 = ur_data_processing.UR_Stream_Data.J_Orientation[1];
        double j2 = ur_data_processing.UR_Stream_Data.J_Orientation[2];
        double j3 = ur_data_processing.UR_Stream_Data.J_Orientation[3];
        double j4 = ur_data_processing.UR_Stream_Data.J_Orientation[4];
        double j5 = ur_data_processing.UR_Stream_Data.J_Orientation[5];

        // 构建movej命令，使用get_inverse_kin将笛卡尔坐标转换为关节角度
        string cmd = string.Format(CultureInfo.InvariantCulture,
            "movej(get_inverse_kin(p[{0:0.0000},{1:0.0000},{2:0.0000},{3:0.0000},{4:0.0000},{5:0.0000}], qnear=[{6:0.0000},{7:0.0000},{8:0.0000},{9:0.0000},{10:0.0000},{11:0.0000}]), a={12:0.0000}, v={13:0.0000}, t={14:0.0000}, r={15:0.0000})\n",
            x, y, z, rx, ry, rz, 
            j0, j1, j2, j3, j4, j5,
            acceleration, linearSpeed, time, blendRadius);

        Debug.Log($"[命令发送] MoveJ 命令:");
        Debug.Log($"  目标位置(m): ({x:F4}, {y:F4}, {z:F4})");
        Debug.Log($"  目标姿态(rad): ({rx:F4}, {ry:F4}, {rz:F4})");
        Debug.Log($"  参考关节角(rad): j0={j0:F4}, j1={j1:F4}, j2={j2:F4}, j3={j3:F4}, j4={j4:F4}, j5={j5:F4}");
        Debug.Log($"  运动参数: a={acceleration:F4}, v={linearSpeed:F4}, t={time:F4}, r={blendRadius:F4}");
        Debug.Log($"  URScript命令: {cmd.Trim()}");

        // 更新全局命令字符串与 UTF-8 字节缓冲
        ur_data_processing.UR_Control_Data.aux_command_str = cmd;
        ur_data_processing.UR_Control_Data.command = utf8.GetBytes(cmd);
    }

    // -------------------- Helpers -------------------- //
    // 以 InvariantCulture 解析数字；容忍逗号小数点并回退为 0
    private static double ParseInvariant(string s)
    {
        if (string.IsNullOrWhiteSpace(s)) return 0.0;
        s = s.Trim().Replace(',', '.');
        if (double.TryParse(s, NumberStyles.Float | NumberStyles.AllowThousands,
                            CultureInfo.InvariantCulture, out double v))
        {
            return v;
        }
        return 0.0; // 失败回退为 0，避免拼接出非法 URScript
    }

    // -------------------- Destroy Blocks -------------------- //
    // 应用退出回调：销毁自身（UI脚本）
    void OnApplicationQuit()
    {
        // 销毁当前组件实例
        Destroy(this);
    }

    // -------------------- Connection Panel -> Visible On -------------------- //
    // 显示“连接”面板，并隐藏其它两个面板
    public void TaskOnClick_ConnectionBTN()
    {
        // 连接面板 -> 显示（放回原位）
        connection_panel_img.transform.localPosition = new Vector3(0f, 0f, 0f);
        // 诊断/摇杆面板 -> 隐藏（移出屏幕）
        diagnostic_panel_img.transform.localPosition = new Vector3(780f + (ex_param), 0f, 0f);
        joystick_panel_img.transform.localPosition = new Vector3(1550f + (ex_param), 0f, 0f);
        speedl_panel_img.transform.localPosition = new Vector3(1880f + (ex_param), 0f, 0f);
        movel_panel_img.transform.localPosition = new Vector3(2055f + (ex_param), 0f, 0f);
        calibration_panel_img.transform.localPosition = new Vector3(2320f + (ex_param), 0f, 0f);
    }

    // -------------------- Connection Panel -> Visible off -------------------- //
    // 隐藏“连接”面板
    public void TaskOnClick_EndConnectionBTN()
    {
        connection_panel_img.transform.localPosition = new Vector3(1215f + (ex_param), 0f, 0f);
    }

    // -------------------- Diagnostic Panel -> Visible On -------------------- //
    // 显示“诊断”面板，并隐藏其它两个面板
    public void TaskOnClick_DiagnosticBTN()
    {
        // 诊断面板 -> 显示
        diagnostic_panel_img.transform.localPosition = new Vector3(0f, 0f, 0f);
        // 连接/摇杆面板 -> 隐藏
        connection_panel_img.transform.localPosition = new Vector3(1215f + (ex_param), 0f, 0f);
        joystick_panel_img.transform.localPosition = new Vector3(1550f + (ex_param), 0f, 0f);
        speedl_panel_img.transform.localPosition = new Vector3(1880f + (ex_param), 0f, 0f);
        movel_panel_img.transform.localPosition = new Vector3(2055f + (ex_param), 0f, 0f);
        calibration_panel_img.transform.localPosition = new Vector3(2320f + (ex_param), 0f, 0f);
    }

    // -------------------- Diagnostic Panel -> Visible Off -------------------- //
    // 隐藏“诊断”面板
    public void TaskOnClick_EndDiagnosticBTN()
    {
        diagnostic_panel_img.transform.localPosition = new Vector3(780f + (ex_param), 0f, 0f);
    }

    // -------------------- Joystick Panel -> Visible On -------------------- //
    // 显示“摇杆”面板，并隐藏其它两个面板
    public void TaskOnClick_JoystickBTN()
    {
        // 摇杆面板 -> 显示（其默认显示位置稍有偏移）
        joystick_panel_img.transform.localPosition = new Vector3(-265f, -129f, 0f);
        // 连接/诊断面板 -> 隐藏
        connection_panel_img.transform.localPosition = new Vector3(1215f + (ex_param), 0f, 0f);
        diagnostic_panel_img.transform.localPosition = new Vector3(780f + (ex_param), 0f, 0f);
        speedl_panel_img.transform.localPosition = new Vector3(1880f + (ex_param), 0f, 0f);
        movel_panel_img.transform.localPosition = new Vector3(2055f + (ex_param), 0f, 0f);
        calibration_panel_img.transform.localPosition = new Vector3(2320f + (ex_param), 0f, 0f);
    }

    // -------------------- Joystick Panel -> Visible Off -------------------- //
    // 隐藏“摇杆”面板
    public void TaskOnClick_EndJoystickBTN()
    {
        joystick_panel_img.transform.localPosition = new Vector3(1550f + (ex_param), 0f, 0f);
    }

    // -------------------- SPEEDl Pane -> Visible On -------------------- //
    // 显示“自定义空页面”面板，并隐藏其它三个面板
    public void TaskOnClick_SPEEDL_BTN()
    {
        if (speedl_panel_img == null) return;
        // 新面板 -> 显示
        speedl_panel_img.transform.localPosition = new Vector3(-265f, -129f, 0f);
        // 其它面板 -> 隐藏
        connection_panel_img.transform.localPosition = new Vector3(1215f + (ex_param), 0f, 0f);
        diagnostic_panel_img.transform.localPosition = new Vector3(780f + (ex_param), 0f, 0f);
        joystick_panel_img.transform.localPosition = new Vector3(1550f + (ex_param), 0f, 0f);
        movel_panel_img.transform.localPosition = new Vector3(2055f + (ex_param), 0f, 0f);
        calibration_panel_img.transform.localPosition = new Vector3(2320f + (ex_param), 0f, 0f);
    }

    // -------------------- SPEEDl Pane -> Visible Off -------------------- //
    // 隐藏“自定义空页面”面板
    public void TaskOnClick_End_SPEEDL_BTN()
    {
        if (speedl_panel_img == null) return;
        speedl_panel_img.transform.localPosition = new Vector3(1880f + (ex_param), 0f, 0f);
    }

    // -------------------- Movel Pane -> Visible On -------------------- //
    // 显示“Movel 控制页面”面板，并隐藏其它四个面板
    public void TaskOnClick_MovelBTN()
    {
        if (movel_panel_img == null) return;
        // Movel 面板 -> 显示
        movel_panel_img.transform.localPosition = new Vector3(-265f, -129f, 0f);
        // 其它面板 -> 隐藏
        connection_panel_img.transform.localPosition = new Vector3(1215f + (ex_param), 0f, 0f);
        diagnostic_panel_img.transform.localPosition = new Vector3(780f + (ex_param), 0f, 0f);
        joystick_panel_img.transform.localPosition = new Vector3(1550f + (ex_param), 0f, 0f);
        speedl_panel_img.transform.localPosition = new Vector3(1880f + (ex_param), 0f, 0f);
        calibration_panel_img.transform.localPosition = new Vector3(2320f + (ex_param), 0f, 0f);
    }

    // -------------------- Movel Pane -> Visible Off -------------------- //
    // 隐藏“Movel 控制页面”面板
    public void TaskOnClick_EndMovelBTN()
    {
        if (movel_panel_img == null) return;
        movel_panel_img.transform.localPosition = new Vector3(2055f + (ex_param), 0f, 0f);
    }

        public void TaskOnClick_CalibrationBTN()
    {
        if (calibration_panel_img == null) return;
        // Calibration 面板 -> 显示
        calibration_panel_img.transform.localPosition = new Vector3(0f, 0f, 0f);
        // 其它面板 -> 隐藏
        connection_panel_img.transform.localPosition = new Vector3(1215f + (ex_param), 0f, 0f);
        diagnostic_panel_img.transform.localPosition = new Vector3(780f + (ex_param), 0f, 0f);
        joystick_panel_img.transform.localPosition = new Vector3(1550f + (ex_param), 0f, 0f);
        speedl_panel_img.transform.localPosition = new Vector3(1880f + (ex_param), 0f, 0f);
        movel_panel_img.transform.localPosition = new Vector3(2055f + (ex_param), 0f, 0f);
    }

        // -------------------- Calibration Pane -> Visible Off -------------------- //
    // 隐藏“Calibration 控制页面”面板
    public void TaskOnClick_EndCalibrationBTN()
    {
        if (calibration_panel_img == null) return;
        calibration_panel_img.transform.localPosition = new Vector3(2320f + (ex_param), 0f, 0f);
    }
    // -------------------- Camera Position -> Right -------------------- //
    // 相机视角：右侧视角
    public void TaskOnClick_CamViewRBTN()
    {
        camera_obj.transform.localPosition = new Vector3(1.65f, 1.05f, -1.85f);
        camera_obj.transform.localEulerAngles = new Vector3(10f, -30f, 0f);
    }

    // -------------------- Camera Position -> Left -------------------- //
    // 相机视角：左侧视角
    public void TaskOnClick_CamViewLBTN()
    {
        camera_obj.transform.localPosition = new Vector3(-1f, 1.1f, -2.2f);
        camera_obj.transform.localEulerAngles = new Vector3(10f, 30f, 0f);
    }

    // -------------------- Camera Position -> Home (in front) -------------------- //
    // 相机视角：正前方（Home）
    public void TaskOnClick_CamViewHBTN()
    {
        camera_obj.transform.localPosition = new Vector3(0.45f, 0.7f, -2.55f);
        camera_obj.transform.localEulerAngles = new Vector3(0f, 0f, 0f);
    }

    // -------------------- Camera Position -> Top -------------------- //
    // 相机视角：顶视图
    public void TaskOnClick_CamViewTBTN()
    {
        camera_obj.transform.localPosition = new Vector3(0.25f, 3.15f, -0.25f);
        camera_obj.transform.localEulerAngles = new Vector3(90f, 0f, 0f);
    }

    // -------------------- Connect Button -> is pressed -------------------- //
    // “连接”按钮：置位连接标志，清除断开标志
    public void TaskOnClick_ConnectBTN()
    {
        ur_data_processing.GlobalVariables_Main_Control.connect    = true;
        ur_data_processing.GlobalVariables_Main_Control.disconnect = false;
    }

    // -------------------- Disconnect Button -> is pressed -------------------- //
    // "断开"按钮：清除连接标志，置位断开标志
    public void TaskOnClick_DisconnectBTN()
    {
        ur_data_processing.GlobalVariables_Main_Control.connect    = false;
        ur_data_processing.GlobalVariables_Main_Control.disconnect = true;
    }

    // ==================== Inspector 右键菜单 - Servoj 控制 ==================== //
    
    /// <summary>
    /// [右键菜单] 开始连续发送 Servoj 命令
    /// 使用 Inspector 中配置的 Tracker 位姿数据
    /// 按照设定的频率（默认90Hz）持续发送控制命令
    /// </summary>
    [ContextMenu("▶ 开始 Servoj 连续控制")]
    private void StartServojContinuousControl()
    {
        if (!Application.isPlaying)
        {
            Debug.LogWarning("[Servoj 右键菜单] 请在 Play 模式下使用此功能！");
            return;
        }

        if (!ur_data_processing.GlobalVariables_Main_Control.connect)
        {
            Debug.LogWarning("[Servoj 右键菜单] 请先连接机械臂！");
            return;
        }

        servojContinuousSendActive = true;
        useInspectorTrackerPose = true;  // 强制使用 Inspector 输入
        
        // 频率一致性检查
        float expectedFreqFromTimeStep = 1f / inspector_servoj_timeStep;
        float frequencyDiff = Mathf.Abs(servojSendFrequency - expectedFreqFromTimeStep);
        
        Debug.Log($"[Servoj 右键菜单] ✅ 开始连续控制");
        Debug.Log($"  协程发送频率: {servojSendFrequency} Hz");
        Debug.Log($"  TimeStep设定频率: {expectedFreqFromTimeStep:F1} Hz (来自 t={inspector_servoj_timeStep}s)");
        Debug.Log($"  TCP线程发送频率: 125 Hz (固定)");
        
        if (frequencyDiff > 1f)
        {
            Debug.LogWarning($"⚠️ 频率不匹配警告: 协程频率({servojSendFrequency}Hz) 与 TimeStep频率({expectedFreqFromTimeStep:F1}Hz) 相差 {frequencyDiff:F1}Hz");
            Debug.LogWarning($"   这可能导致卡顿！建议设置 servojSendFrequency = {expectedFreqFromTimeStep:F0}");
        }
        else
        {
            Debug.Log($"  ✅ 频率匹配正常 (误差 {frequencyDiff:F2}Hz)");
        }
        
        Debug.Log($"  坐标系: {(trackerPoseIsSteamVr ? "SteamVR" : "UR基座")}");
        Debug.Log($"  位置: ({tracker_x:F3}, {tracker_y:F3}, {tracker_z:F3})");
        Debug.Log($"  姿态: ({tracker_rx:F4}, {tracker_ry:F4}, {tracker_rz:F4})");
        
        // 启动连续发送协程
        StartCoroutine(ServojContinuousSendCoroutine());
    }

    /// <summary>
    /// [右键菜单] 停止连续发送 Servoj 命令
    /// </summary>
    [ContextMenu("⏸ 停止 Servoj 连续控制")]
    private void StopServojContinuousControl()
    {
        if (!Application.isPlaying)
        {
            Debug.LogWarning("[Servoj 右键菜单] 请在 Play 模式下使用此功能！");
            return;
        }

        servojContinuousSendActive = false;
        ur_data_processing.UR_Control_Data.manual_send_active = false;
        
        Debug.Log("[Servoj 右键菜单] ⏸ 已停止连续控制");
    }

    /// <summary>
    /// [右键菜单] 发送单次 Servoj 命令（测试用）
    /// 使用 Inspector 中当前配置的位姿发送一次命令
    /// </summary>
    [ContextMenu("🎯 发送单次 Servoj 命令")]
    private void SendSingleServojCommand()
    {
        if (!Application.isPlaying)
        {
            Debug.LogWarning("[Servoj 右键菜单] 请在 Play 模式下使用此功能！");
            return;
        }

        if (!ur_data_processing.GlobalVariables_Main_Control.connect)
        {
            Debug.LogWarning("[Servoj 右键菜单] 请先连接机械臂！");
            return;
        }

        useInspectorTrackerPose = true;  // 强制使用 Inspector 输入
        
        Debug.Log("[Servoj 右键菜单] 🎯 发送单次命令");
        
        // 构建并发送命令
        BuildAndSetServojFromInputs();
        ur_data_processing.UR_Control_Data.manual_send_active = true;
        
        // 短暂发送后自动关闭
        StartCoroutine(SendPulseCoroutine(0.05f));
    }

    /// <summary>
    /// [右键菜单] 显示当前 Inspector 配置信息
    /// </summary>
    [ContextMenu("ℹ️ 显示当前配置")]
    private void ShowCurrentConfiguration()
    {
        Debug.Log("========== Servoj Inspector 当前配置 ==========");
        Debug.Log($"使用 Inspector 输入: {useInspectorTrackerPose}");
        Debug.Log($"坐标系: {(trackerPoseIsSteamVr ? "SteamVR (mm, rad)" : "UR基座 (m, rad)")}");
        Debug.Log($"位置: X={tracker_x:F3}, Y={tracker_y:F3}, Z={tracker_z:F3}");
        Debug.Log($"姿态: Rx={tracker_rx:F4}, Ry={tracker_ry:F4}, Rz={tracker_rz:F4}");
        Debug.Log($"发送频率: {servojSendFrequency} Hz");
        Debug.Log($"连续控制状态: {(servojContinuousSendActive ? "运行中 ✅" : "已停止 ⏸")}");
        Debug.Log($"机械臂连接: {(ur_data_processing.GlobalVariables_Main_Control.connect ? "已连接 ✅" : "未连接 ❌")}");
        Debug.Log("==============================================");
    }

    /// <summary>
    /// 连续发送 Servoj 命令的协程
    /// </summary>
    private IEnumerator ServojContinuousSendCoroutine()
    {
        float sendInterval = 1f / servojSendFrequency;
        
        Debug.Log($"[Servoj 连续控制] 协程已启动");
        Debug.Log($"  发送频率: {servojSendFrequency}Hz, 间隔: {sendInterval:F4}秒 ({sendInterval*1000:F2}ms)");
        Debug.Log($"  TimeStep: {inspector_servoj_timeStep}s ({1f/inspector_servoj_timeStep:F1}Hz)");
        Debug.Log($"  ⚠️ 提示: 发送频率应与TimeStep严格匹配，避免卡顿！");
        
        while (servojContinuousSendActive)
        {
            // 构建并发送命令
            BuildAndSetServojFromInputs();
            ur_data_processing.UR_Control_Data.manual_send_active = true;
            
            // 等待下一个周期
            yield return new WaitForSeconds(sendInterval);
        }
        
        Debug.Log("[Servoj 连续控制] 协程已结束");
    }

}

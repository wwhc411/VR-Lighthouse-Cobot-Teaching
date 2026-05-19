using System;
using System.Globalization;
using System.Text;
using UnityEngine;
using handeye;  // 手眼标定坐标转换器

/// <summary>
/// 刚体数据Servoj指令生成器
/// 功能：基于动捕系统的刚体数据生成URScript servoj命令
/// 
/// 数据转换流程：
///   1. CSV原始数据 (SteamVR坐标系, mm, 四元数)
///   2. 应用Tracker本地坐标系偏移 (可选)
///   3. 手眼标定坐标变换 (SteamVR → UR Base)
///   4. 生成URScript servoj命令
/// 
/// 指令格式：servoj(get_inverse_kin(p[x,y,z,rx,ry,rz], qnear=[j0,j1,j2,j3,j4,j5]), a, v, t, lookahead_time, gain)
/// 更新日期: 2025-12-02
/// </summary>
public class RigidBodyServojCommandGenerator : MonoBehaviour
{
    /// <summary>
    /// 是否启用坐标转换（SteamVR → UR Base）
    /// 默认启用，CSV数据来自Tracker录制，需要转换
    /// </summary>
    public static bool EnableCoordinateTransform = true;

    /// <summary>
    /// 是否启用Tracker本地坐标系偏移
    /// 用于补偿Tracker安装位置与实际控制点的偏差
    /// </summary>
    public static bool EnableTrackerOffset = false;

    /// <summary>
    /// Tracker本地坐标系位置偏移（毫米）
    /// 偏移量在Tracker本地坐标系中表示
    /// 例如：(0, -150, 0) 表示沿Tracker的-Y方向偏移150mm
    /// </summary>
    public static Vector3 TrackerPositionOffset = Vector3.zero;

    /// <summary>
    /// Tracker本地坐标系旋转偏移（欧拉角，度）
    /// </summary>
    public static Vector3 TrackerRotationOffset = Vector3.zero;

    /// <summary>
    /// 是否输出调试日志
    /// </summary>
    public static bool EnableDebugLog = false;
    /// <summary>
    /// Servoj控制参数结构
    /// </summary>
    [Serializable]
    public struct ServojParameters
    {
        [Tooltip("关节加速度(rad/s²), 0=无限制")]
        [Range(0f, 10f)]
        public float Acceleration;

        [Tooltip("关节速度(rad/s), 0=无限制")]
        [Range(0f, 3.14f)]
        public float Velocity;

        [Tooltip("时间步长(s), 必须与TCP发送频率匹配！125Hz=0.008")]
        [Range(0.002f, 0.2f)]
        public float TimeStep;

        [Tooltip("前瞻时间(s), 用于轨迹平滑，推荐0.03-0.15")]
        [Range(0.03f, 0.2f)]
        public float LookAheadTime;

        [Tooltip("控制增益, 值越小响应越快，推荐200-600")]
        [Range(100f, 2000f)]
        public float Gain;

        /// <summary>
        /// 默认参数（125Hz控制频率）
        /// </summary>
        public static ServojParameters Default => new ServojParameters
        {
            Acceleration = 0.001f,
            Velocity = 0.01f,
            TimeStep = 0.008f,      // 125Hz匹配
            LookAheadTime = 0.1f,
            Gain = 300f
        };

        /// <summary>
        /// 验证参数是否在有效范围内
        /// </summary>
        public bool Validate(out string errorMessage)
        {
            if (Acceleration < 0 || Acceleration > 10)
            {
                errorMessage = $"加速度超出范围(0-10): {Acceleration}";
                return false;
            }

            if (Velocity < 0 || Velocity > 3.14f)
            {
                errorMessage = $"速度超出范围(0-3.14): {Velocity}";
                return false;
            }

            if (TimeStep < 0.002f || TimeStep > 0.2f)
            {
                errorMessage = $"时间步长超出范围(0.002-0.2): {TimeStep}";
                return false;
            }

            if (LookAheadTime < 0.03f || LookAheadTime > 0.2f)
            {
                errorMessage = $"前瞻时间超出范围(0.03-0.2): {LookAheadTime}";
                return false;
            }

            if (Gain < 100f || Gain > 2000f)
            {
                errorMessage = $"增益超出范围(100-2000): {Gain}";
                return false;
            }

            errorMessage = "参数验证通过";
            return true;
        }
    }

    /// <summary>
    /// 基于单帧刚体数据生成Servoj命令
    /// 
    /// 数据处理流程:
    ///   1. 从FrameData提取位置(mm)和四元数
    ///   2. 四元数 → 旋转矢量(rad)
    ///   3. 调用手眼标定转换 SteamVR → UR Base
    ///   4. 生成URScript servoj命令
    /// 
    /// 注意：输入数据为SteamVR坐标系，输出命令为UR基座坐标系
    /// </summary>
    /// <param name="frameData">单帧刚体数据（SteamVR坐标系）</param>
    /// <param name="parameters">Servoj控制参数</param>
    /// <param name="currentJointAngles">当前关节角度(rad)，用于逆运动学参考，若为null则从UR_Stream_Data读取</param>
    /// <returns>格式化的URScript servoj命令字符串</returns>
    public static string GenerateServojCommand(FrameData frameData, ServojParameters parameters, 
                                               double[] currentJointAngles = null)
    {
        if (frameData == null)
        {
            Debug.LogError("[RigidBodyServojCommandGenerator] 帧数据为空");
            return null;
        }

        // 验证参数
        if (!parameters.Validate(out string errorMessage))
        {
            Debug.LogError($"[RigidBodyServojCommandGenerator] 参数验证失败: {errorMessage}");
            return null;
        }

        // 获取当前关节角度作为qnear参考
        double[] qnear = currentJointAngles;
        if (qnear == null || qnear.Length != 6)
        {
            // 从UR数据流读取当前关节角度
            qnear = new double[6];
            for (int i = 0; i < 6; i++)
            {
                qnear[i] = ur_data_processing.UR_Stream_Data.J_Orientation[i];
            }
        }

        // ========== 位姿数据转换流程 ==========
        double x, y, z, rx, ry, rz;

        if (EnableCoordinateTransform)
        {
            // 【启用坐标转换】SteamVR → UR Base
            // 这与 TrackerPoseCapture.cs 中的转换流程完全一致
            
            // 步骤1: 提取原始位置(mm)和四元数
            Vector3 posSteamVr_mm = new Vector3(
                (float)frameData.Position.X,
                (float)frameData.Position.Y,
                (float)frameData.Position.Z
            );
            Quaternion quatSteamVr = frameData.GetQuaternion();

            if (EnableDebugLog)
            {
                Debug.Log($"[Servoj坐标转换] 原始 SteamVR 位姿:");
                Debug.Log($"  位置(mm): ({posSteamVr_mm.x:F2}, {posSteamVr_mm.y:F2}, {posSteamVr_mm.z:F2})");
                Debug.Log($"  四元数: (w:{quatSteamVr.w:F4}, x:{quatSteamVr.x:F4}, y:{quatSteamVr.y:F4}, z:{quatSteamVr.z:F4})");
            }

            // 步骤1.5: 应用Tracker本地坐标系偏移（如果启用）
            // 与 TrackerCoordinateOffsetWrapper.cs 中的 ApplyOffset() 逻辑一致
            if (EnableTrackerOffset)
            {
                // 位置偏移：将本地偏移转换到世界坐标系
                // worldOffset = rotation * localOffset
                // 注意：TrackerPositionOffset 单位为毫米，与CSV数据一致
                Vector3 worldOffsetMm = quatSteamVr * TrackerPositionOffset;
                posSteamVr_mm = posSteamVr_mm + worldOffsetMm;

                // 旋转偏移：在本地坐标系中应用
                // newRotation = originalRotation * rotationOffset
                if (TrackerRotationOffset != Vector3.zero)
                {
                    Quaternion rotationOffsetQuat = Quaternion.Euler(TrackerRotationOffset);
                    quatSteamVr = quatSteamVr * rotationOffsetQuat;
                }

                if (EnableDebugLog)
                {
                    Debug.Log($"[Servoj坐标转换] 应用Tracker偏移后:");
                    Debug.Log($"  位置偏移(mm): ({TrackerPositionOffset.x:F2}, {TrackerPositionOffset.y:F2}, {TrackerPositionOffset.z:F2})");
                    Debug.Log($"  旋转偏移(deg): ({TrackerRotationOffset.x:F2}, {TrackerRotationOffset.y:F2}, {TrackerRotationOffset.z:F2})");
                    Debug.Log($"  偏移后位置(mm): ({posSteamVr_mm.x:F2}, {posSteamVr_mm.y:F2}, {posSteamVr_mm.z:F2})");
                }
            }

            // 步骤2: 调用手眼标定坐标转换器
            // 输入: SteamVR坐标系 (mm, 四元数)
            // 输出: UR基座坐标系 (m, 旋转矢量rad)
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                posSteamVr_mm,
                quatSteamVr,
                posInMillimeters: true,
                out Vector3 posUr_m,
                out Vector3 rotUr_rad
            );

            if (EnableDebugLog)
            {
                Debug.Log($"[Servoj坐标转换] 输出 UR Base 位姿:");
                Debug.Log($"  位置(m): ({posUr_m.x:F4}, {posUr_m.y:F4}, {posUr_m.z:F4})");
                Debug.Log($"  旋转(rad): ({rotUr_rad.x:F4}, {rotUr_rad.y:F4}, {rotUr_rad.z:F4})");
            }

            // 使用转换后的UR基座坐标
            x = posUr_m.x;
            y = posUr_m.y;
            z = posUr_m.z;
            rx = rotUr_rad.x;
            ry = rotUr_rad.y;
            rz = rotUr_rad.z;
        }
        else
        {
            // 【禁用坐标转换】直接使用原始数据（假设已是UR基座坐标）
            x = frameData.Position.X / 1000.0;  // mm → m
            y = frameData.Position.Y / 1000.0;
            z = frameData.Position.Z / 1000.0;
            
            Vector3 rotation = frameData.GetRotationVector();
            rx = rotation.x;
            ry = rotation.y;
            rz = rotation.z;
        }

        // ========== 构建servoj命令 ==========
        // 格式: servoj(get_inverse_kin(p[x,y,z,rx,ry,rz], qnear=[j0,j1,j2,j3,j4,j5]), a, v, t, lookahead_time, gain)
        string command = string.Format(CultureInfo.InvariantCulture,
            "servoj(get_inverse_kin(p[{0},{1},{2},{3},{4},{5}], qnear=[{6},{7},{8},{9},{10},{11}]), {12}, {13}, {14}, {15}, {16})\n",
            x, y, z, rx, ry, rz,                                        // 目标位姿 p[x,y,z,rx,ry,rz]
            qnear[0], qnear[1], qnear[2], qnear[3], qnear[4], qnear[5], // 参考关节角 qnear[j0..j5]
            parameters.Acceleration, parameters.Velocity,               // 加速度、速度
            parameters.TimeStep, parameters.LookAheadTime, parameters.Gain); // 时间步长、前瞻时间、增益

        // 输出简洁的回放日志（帧号 + 原始数据 + 生成命令）
        if (EnableDebugLog)
        {
            Debug.Log($"<color=cyan>[回放] 帧{frameData.FrameNumber}</color> " +
                      $"原始:({frameData.Position.X:F1},{frameData.Position.Y:F1},{frameData.Position.Z:F1})mm " +
                      $"→ UR:p[{x:F4},{y:F4},{z:F4},{rx:F3},{ry:F3},{rz:F3}]");
        }

        return command;
    }

    /// <summary>
    /// 直接使用Vector3位姿生成Servoj命令（便捷方法）
    /// 注意: 此方法假设输入已经是UR基座坐标系，不进行坐标转换
    /// </summary>
    /// <param name="position">位置向量(m) - UR基座坐标系</param>
    /// <param name="rotationVector">旋转矢量(rad) - UR基座坐标系</param>
    /// <param name="parameters">Servoj控制参数</param>
    /// <param name="currentJointAngles">当前关节角度(rad)，若为null则从UR_Stream_Data读取</param>
    /// <returns>格式化的URScript servoj命令字符串</returns>
    public static string GenerateServojCommandDirect(Vector3 position, Vector3 rotationVector, 
                                                     ServojParameters parameters, double[] currentJointAngles = null)
    {
        // 验证参数
        if (!parameters.Validate(out string errorMessage))
        {
            Debug.LogError($"[RigidBodyServojCommandGenerator] 参数验证失败: {errorMessage}");
            return null;
        }

        // 获取当前关节角度作为qnear参考
        double[] qnear = currentJointAngles;
        if (qnear == null || qnear.Length != 6)
        {
            qnear = new double[6];
            for (int i = 0; i < 6; i++)
            {
                qnear[i] = ur_data_processing.UR_Stream_Data.J_Orientation[i];
            }
        }

        // 直接使用输入的UR基座坐标（不转换）
        string command = string.Format(CultureInfo.InvariantCulture,
            "servoj(get_inverse_kin(p[{0},{1},{2},{3},{4},{5}], qnear=[{6},{7},{8},{9},{10},{11}]), {12}, {13}, {14}, {15}, {16})\n",
            position.x, position.y, position.z, 
            rotationVector.x, rotationVector.y, rotationVector.z,
            qnear[0], qnear[1], qnear[2], qnear[3], qnear[4], qnear[5],
            parameters.Acceleration, parameters.Velocity,
            parameters.TimeStep, parameters.LookAheadTime, parameters.Gain);

        return command;
    }

    /// <summary>
    /// 使用CSV记录的TCP数据直接生成Servoj命令（跳过坐标转换）
    /// 
    /// 这个方法直接使用录制时记录的TCP位姿，无需经过手眼标定转换。
    /// 适用于精确复现录制时的机械臂轨迹。
    /// </summary>
    /// <param name="frameData">单帧数据（必须包含TcpPose数据）</param>
    /// <param name="parameters">Servoj控制参数</param>
    /// <param name="currentJointAngles">当前关节角度(rad)，若为null则从UR_Stream_Data读取</param>
    /// <returns>格式化的URScript servoj命令字符串，如果没有TCP数据返回null</returns>
    public static string GenerateServojCommandFromTcpData(FrameData frameData, ServojParameters parameters,
                                                          double[] currentJointAngles = null)
    {
        if (frameData == null || !frameData.HasTcpData || frameData.TcpPose == null)
        {
            Debug.LogError("[RigidBodyServojCommandGenerator] 帧数据不包含TCP位姿数据");
            return null;
        }

        // 验证参数
        if (!parameters.Validate(out string errorMessage))
        {
            Debug.LogError($"[RigidBodyServojCommandGenerator] 参数验证失败: {errorMessage}");
            return null;
        }

        // 获取当前关节角度作为qnear参考
        double[] qnear = currentJointAngles;
        if (qnear == null || qnear.Length != 6)
        {
            qnear = new double[6];
            for (int i = 0; i < 6; i++)
            {
                qnear[i] = ur_data_processing.UR_Stream_Data.J_Orientation[i];
            }
        }

        // 直接使用录制的TCP数据（已经是UR基座坐标系）
        TcpPoseData tcp = frameData.TcpPose;
        
        string command = string.Format(CultureInfo.InvariantCulture,
            "servoj(get_inverse_kin(p[{0},{1},{2},{3},{4},{5}], qnear=[{6},{7},{8},{9},{10},{11}]), {12}, {13}, {14}, {15}, {16})\n",
            tcp.X, tcp.Y, tcp.Z, tcp.RX, tcp.RY, tcp.RZ,
            qnear[0], qnear[1], qnear[2], qnear[3], qnear[4], qnear[5],
            parameters.Acceleration, parameters.Velocity,
            parameters.TimeStep, parameters.LookAheadTime, parameters.Gain);

        if (EnableDebugLog)
        {
            Debug.Log($"<color=cyan>[TCP回放] 帧{frameData.FrameNumber}</color> " +
                      $"TCP:p[{tcp.X:F4},{tcp.Y:F4},{tcp.Z:F4},{tcp.RX:F3},{tcp.RY:F3},{tcp.RZ:F3}]");
        }

        return command;
    }

    /// <summary>
    /// 从SteamVR位姿生成Servoj命令（包含坐标转换）
    /// </summary>
    /// <param name="positionMm">位置向量(mm) - SteamVR坐标系</param>
    /// <param name="quaternion">四元数 - SteamVR坐标系</param>
    /// <param name="parameters">Servoj控制参数</param>
    /// <param name="currentJointAngles">当前关节角度(rad)，若为null则从UR_Stream_Data读取</param>
    /// <returns>格式化的URScript servoj命令字符串</returns>
    public static string GenerateServojCommandFromSteamVR(Vector3 positionMm, Quaternion quaternion,
                                                          ServojParameters parameters, double[] currentJointAngles = null)
    {
        // 调用手眼标定坐标转换
        SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
            positionMm,
            quaternion,
            posInMillimeters: true,
            out Vector3 posUr_m,
            out Vector3 rotUr_rad
        );

        return GenerateServojCommandDirect(posUr_m, rotUr_rad, parameters, currentJointAngles);
    }

    /// <summary>
    /// 将生成的命令发送到UR控制数据缓冲区
    /// </summary>
    /// <param name="command">URScript命令字符串</param>
    public static void SendCommandToUR(string command)
    {
        if (string.IsNullOrEmpty(command))
        {
            Debug.LogError("[RigidBodyServojCommandGenerator] 命令为空，无法发送");
            return;
        }

        // 使用UTF-8编码转换为字节数组
        UTF8Encoding utf8 = new UTF8Encoding();
        byte[] commandBytes = utf8.GetBytes(command);

        // 更新控制数据缓冲区
        ur_data_processing.UR_Control_Data.aux_command_str = command;
        ur_data_processing.UR_Control_Data.command = commandBytes;
        ur_data_processing.UR_Control_Data.manual_send_active = true;

        Debug.Log($"[RigidBodyServojCommandGenerator] 命令已发送: {command.Trim()}");
    }

    /// <summary>
    /// 生成停止命令（退出伺服模式，释放机械臂控制）
    /// </summary>
    /// <returns>停止命令字符串</returns>
    public static string GenerateStopCommand()
    {
        // 使用 stopl() 退出伺服模式并平滑停止
        // stopl(a) - 在笛卡尔空间减速停止，a=减速度(m/s²)
        // 这会正确结束servoj的伺服控制，让机械臂可以被自由移动
        return "stopl(5)\n";
    }
}

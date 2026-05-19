using System;
using System.Globalization;
using System.Text;
using UnityEngine;
using handeye;  // 手眼标定坐标转换器
using MathNet.Numerics.LinearAlgebra;

/// <summary>
/// 刚体数据Servoj指令生成器
/// 功能：基于动捕系统的刚体数据生成URScript servoj命令
/// 
/// 数据转换流程：
///   1. CSV原始数据 (SteamVR坐标系, mm, 四元数)
///   2. 应用Tracker本地坐标系偏移 (可选)
///   3. 应用Kabsch点云刚性对齐校正 - 仅校正位置xyz (可选)
///   4. 手眼标定坐标变换 (SteamVR → UR Base)
///   5. 旋转矢量连续性校正（避免π边界跳变）
///   6. 生成URScript servoj命令
/// 
/// 指令格式：servoj(get_inverse_kin(p[x,y,z,rx,ry,rz], qnear=[j0,j1,j2,j3,j4,j5]), a, v, t, lookahead_time, gain)
/// 更新日期: 2026-01-26
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
    /// 是否启用旋转矢量连续性校正
    /// 解决旋转角度穿越±π边界时的跳变问题
    /// </summary>
    public static bool EnableRotationContinuity = true;

    /// <summary>
    /// 是否启用Kabsch点云刚性对齐校正
    /// </summary>
    public static bool EnableKabschAlignment = false;

    /// <summary>
    /// Kabsch对齐旋转矩阵（3x3）
    /// </summary>
    private static Matrix<double> kabschRotationMatrix = null;

    /// <summary>
    /// Kabsch对齐平移向量（3x1）
    /// </summary>
    private static Vector<double> kabschTranslationVector = null;

    /// <summary>
    /// 上一帧的旋转矢量（用于连续性校正）
    /// </summary>
    private static Vector3 previousRotationVector = Vector3.zero;

    /// <summary>
    /// 是否已初始化上一帧旋转矢量
    /// </summary>
    private static bool hasPreviousRotation = false;

    /// <summary>
    /// 重置旋转连续性状态（开始新的回放时调用）
    /// </summary>
    public static void ResetRotationContinuityState()
    {
        previousRotationVector = Vector3.zero;
        hasPreviousRotation = false;
        if (EnableDebugLog)
        {
            Debug.Log("[RigidBodyServojCommandGenerator] 旋转连续性状态已重置");
        }
    }

    /// <summary>
    /// 设置Kabsch对齐变换参数
    /// </summary>
    /// <param name="rotationMatrix">旋转矩阵（3x3）</param>
    /// <param name="translationVector">平移向量（3x1）</param>
    public static void SetKabschTransform(Matrix<double> rotationMatrix, Vector<double> translationVector)
    {
        kabschRotationMatrix = rotationMatrix;
        kabschTranslationVector = translationVector;
        
        if (EnableDebugLog)
        {
            Debug.Log("[RigidBodyServojCommandGenerator] Kabsch变换已设置");
            Debug.Log($"  旋转矩阵: [{rotationMatrix[0,0]:F6}, {rotationMatrix[0,1]:F6}, {rotationMatrix[0,2]:F6}]");
            Debug.Log($"             [{rotationMatrix[1,0]:F6}, {rotationMatrix[1,1]:F6}, {rotationMatrix[1,2]:F6}]");
            Debug.Log($"             [{rotationMatrix[2,0]:F6}, {rotationMatrix[2,1]:F6}, {rotationMatrix[2,2]:F6}]");
            Debug.Log($"  平移向量: [{translationVector[0]:F6}, {translationVector[1]:F6}, {translationVector[2]:F6}]");
        }
    }

    /// <summary>
    /// 清除Kabsch对齐变换参数
    /// </summary>
    public static void ClearKabschTransform()
    {
        kabschRotationMatrix = null;
        kabschTranslationVector = null;
        
        if (EnableDebugLog)
        {
            Debug.Log("[RigidBodyServojCommandGenerator] Kabsch变换已清除");
        }
    }

    /// <summary>
    /// 应用Kabsch刚性变换到位置（四元数格式）
    /// 变换公式: p' = R * p + t
    /// 注意: Kabsch对齐只校正位置，姿态保持不变（因为训练点云只包含位置信息）
    /// </summary>
    /// <param name="position">输入位置（米）</param>
    /// <param name="quaternion">输入旋转（四元数）- 保持不变</param>
    /// <param name="transformedPosition">输出变换后的位置（米）</param>
    /// <param name="transformedQuaternion">输出旋转（四元数）- 与输入相同</param>
    /// <returns>是否成功应用变换</returns>
    private static bool ApplyKabschTransform(Vector3 position, Quaternion quaternion,
                                            out Vector3 transformedPosition, out Quaternion transformedQuaternion)
    {
        transformedPosition = position;
        transformedQuaternion = quaternion;  // 姿态保持不变

        if (!EnableKabschAlignment || kabschRotationMatrix == null || kabschTranslationVector == null)
        {
            return false;  // 未启用或未设置Kabsch变换
        }

        try
        {
            // 位置变换: p' = R * p + t
            var posVec = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(new double[] 
            { 
                position.x, position.y, position.z 
            });
            var transformedPosVec = kabschRotationMatrix.Multiply(posVec) + kabschTranslationVector;
            
            transformedPosition = new Vector3(
                (float)transformedPosVec[0],
                (float)transformedPosVec[1],
                (float)transformedPosVec[2]
            );

            // 姿态保持不变 - Kabsch只校正位置
            // transformedQuaternion 已经在函数开始时设置为输入值

            if (EnableDebugLog)
            {
                Debug.Log($"[Kabsch变换] 位置: ({position.x:F4},{position.y:F4},{position.z:F4}) → ({transformedPosition.x:F4},{transformedPosition.y:F4},{transformedPosition.z:F4})");
                Debug.Log($"[Kabsch变换] 姿态: 保持不变 (q: {quaternion.x:F4},{quaternion.y:F4},{quaternion.z:F4},{quaternion.w:F4})");
            }

            return true;
        }
        catch (Exception e)
        {
            Debug.LogError($"[RigidBodyServojCommandGenerator] Kabsch变换失败: {e.Message}");
            return false;
        }
    }

    /// <summary>
    /// 应用Kabsch刚性变换到位置（旋转矢量格式）
    /// 用于TCP数据（已经是位置+旋转矢量格式）
    /// 变换公式: p' = R * p + t
    /// 注意: Kabsch对齐只校正位置，旋转矢量保持不变
    /// </summary>
    /// <param name="position">输入位置（米）</param>
    /// <param name="rotationVector">输入旋转矢量（弧度）- 保持不变</param>
    /// <param name="transformedPosition">输出变换后的位置（米）</param>
    /// <param name="transformedRotationVector">输出旋转矢量（弧度）- 与输入相同</param>
    /// <returns>是否成功应用变换</returns>
    private static bool ApplyKabschTransformToRotationVector(Vector3 position, Vector3 rotationVector,
                                                              out Vector3 transformedPosition, out Vector3 transformedRotationVector)
    {
        transformedPosition = position;
        transformedRotationVector = rotationVector;  // 旋转矢量保持不变

        if (!EnableKabschAlignment || kabschRotationMatrix == null || kabschTranslationVector == null)
        {
            return false;  // 未启用或未设置Kabsch变换
        }

        try
        {
            // 位置变换: p' = R * p + t
            var posVec = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(new double[] 
            { 
                position.x, position.y, position.z 
            });
            var transformedPosVec = kabschRotationMatrix.Multiply(posVec) + kabschTranslationVector;
            
            transformedPosition = new Vector3(
                (float)transformedPosVec[0],
                (float)transformedPosVec[1],
                (float)transformedPosVec[2]
            );

            // 旋转矢量保持不变 - Kabsch只校正位置
            // transformedRotationVector 已经在函数开始时设置为输入值

            if (EnableDebugLog)
            {
                Debug.Log($"[Kabsch变换] 位置: ({position.x:F4},{position.y:F4},{position.z:F4}) → ({transformedPosition.x:F4},{transformedPosition.y:F4},{transformedPosition.z:F4})");
                Debug.Log($"[Kabsch变换] 旋转矢量: 保持不变 ({rotationVector.x:F4},{rotationVector.y:F4},{rotationVector.z:F4})");
            }

            return true;
        }
        catch (Exception e)
        {
            Debug.LogError($"[RigidBodyServojCommandGenerator] Kabsch变换失败: {e.Message}");
            return false;
        }
    }

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

            // 步骤2: 应用Kabsch点云刚性对齐校正（如果启用）
            // 在手眼标定转换之前应用，因为Kabsch校正的也是SteamVR坐标系下的数据
            if (EnableKabschAlignment)
            {
                // 将mm转换为m（Kabsch处理米单位）
                Vector3 posSteamVr_m = posSteamVr_mm * 0.001f;
                
                if (ApplyKabschTransform(posSteamVr_m, quatSteamVr, 
                    out Vector3 kabschPos_m, out Quaternion kabschQuat))
                {
                    posSteamVr_mm = kabschPos_m * 1000f;  // 转回mm
                    quatSteamVr = kabschQuat;
                    
                    if (EnableDebugLog)
                    {
                        Debug.Log($"[Servoj坐标转换] 应用Kabsch校正后:");
                        Debug.Log($"  位置(mm): ({posSteamVr_mm.x:F2}, {posSteamVr_mm.y:F2}, {posSteamVr_mm.z:F2})");
                    }
                }
            }

            // 步骤3: 调用手眼标定坐标转换器
            // 输入: SteamVR坐标系 (mm, 四元数)
            // 输出: UR基座坐标系 (m, 旋转矢量rad)
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                posSteamVr_mm,
                quatSteamVr,
                posInMillimeters: true,
                out Vector3 posUr_m,
                out Vector3 rotUr_rad
            );

            // 步骤4: 旋转矢量连续性校正（避免π边界跳变）
            if (EnableRotationContinuity)
            {
                rotUr_rad = EnsureRotationVectorContinuity(rotUr_rad);
            }

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
    /// 使用CSV记录的TCP数据直接生成Servoj命令
    /// 
    /// 数据处理流程:
    ///   1. 从FrameData提取TCP位姿（UR基座坐标系，米+弧度）
    ///   2. 应用Kabsch点云刚性对齐校正（可选，新增支持）
    ///   3. 应用旋转矢量连续性校正（可选）
    ///   4. 生成URScript servoj命令
    /// 
    /// 注意：如果启用Kabsch校正，训练的点云必须也是UR基座坐标系
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

        // 获取TCP数据（已经是UR基座坐标系，米+弧度）
        TcpPoseData tcp = frameData.TcpPose;
        
        // 提取位置和旋转矢量
        Vector3 tcpPosition = new Vector3((float)tcp.X, (float)tcp.Y, (float)tcp.Z);
        Vector3 tcpRotation = new Vector3((float)tcp.RX, (float)tcp.RY, (float)tcp.RZ);

        // 应用Kabsch刚性对齐校正（如果启用）
        if (EnableKabschAlignment)
        {
            if (ApplyKabschTransformToRotationVector(tcpPosition, tcpRotation,
                out Vector3 kabschPos, out Vector3 kabschRot))
            {
                if (EnableDebugLog)
                {
                    Debug.Log($"[TCP-Kabsch] 原始: p[{tcpPosition.x:F4},{tcpPosition.y:F4},{tcpPosition.z:F4},{tcpRotation.x:F4},{tcpRotation.y:F4},{tcpRotation.z:F4}]");
                    Debug.Log($"[TCP-Kabsch] 校正: p[{kabschPos.x:F4},{kabschPos.y:F4},{kabschPos.z:F4},{kabschRot.x:F4},{kabschRot.y:F4},{kabschRot.z:F4}]");
                }
                
                tcpPosition = kabschPos;
                tcpRotation = kabschRot;
            }
        }

        // 应用旋转矢量连续性校正（避免π边界跳变）
        if (EnableRotationContinuity)
        {
            tcpRotation = EnsureRotationVectorContinuity(tcpRotation);
        }

        // 生成servoj命令
        string command = string.Format(CultureInfo.InvariantCulture,
            "servoj(get_inverse_kin(p[{0},{1},{2},{3},{4},{5}], qnear=[{6},{7},{8},{9},{10},{11}]), {12}, {13}, {14}, {15}, {16})\n",
            tcpPosition.x, tcpPosition.y, tcpPosition.z, 
            tcpRotation.x, tcpRotation.y, tcpRotation.z,
            qnear[0], qnear[1], qnear[2], qnear[3], qnear[4], qnear[5],
            parameters.Acceleration, parameters.Velocity,
            parameters.TimeStep, parameters.LookAheadTime, parameters.Gain);

        if (EnableDebugLog)
        {
            string modeTag = EnableKabschAlignment ? "TCP+Kabsch" : "TCP直接";
            Debug.Log($"<color=cyan>[{modeTag}回放] 帧{frameData.FrameNumber}</color> " +
                      $"p[{tcpPosition.x:F4},{tcpPosition.y:F4},{tcpPosition.z:F4},{tcpRotation.x:F3},{tcpRotation.y:F3},{tcpRotation.z:F3}]");
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

    /// <summary>
    /// 旋转矢量连续性校正
    /// 
    /// 问题背景：
    ///   旋转矢量 r = θ * axis 的表示存在不连续性：
    ///   1. 同一旋转可以用 (θ, axis) 或 (2π-θ, -axis) 表示
    ///   2. 当θ穿越π时，旋转矢量会发生符号翻转导致剧烈跳变
    /// 
    /// 解决方案：
    ///   比较当前帧与上一帧的旋转矢量，如果差值过大，尝试等效表示使其连续
    ///   等效表示: r' = (2π - |r|) * (-r/|r|) = -r * (2π/|r| - 1)
    /// 
    /// 适用场景：
    ///   - 实时回放控制
    ///   - CSV批量转换
    /// </summary>
    /// <param name="currentRotVec">当前帧的旋转矢量</param>
    /// <returns>连续性校正后的旋转矢量</returns>
    private static Vector3 EnsureRotationVectorContinuity(Vector3 currentRotVec)
    {
        const float PI = Mathf.PI;
        const float TWO_PI = 2f * Mathf.PI;
        const float MAX_SINGLE_FRAME_CHANGE = 0.5f; // 单帧最大变化量（约28度）

        // 第一帧，直接记录并返回
        if (!hasPreviousRotation)
        {
            previousRotationVector = currentRotVec;
            hasPreviousRotation = true;
            return currentRotVec;
        }

        // 计算当前旋转矢量的模（旋转角度）
        float currentAngle = currentRotVec.magnitude;
        float prevAngle = previousRotationVector.magnitude;

        // ========== 异常检测：当前帧角度异常接近0 ==========
        // 当旋转矩阵数值不稳定时，可能输出近零的旋转矢量
        if (currentAngle < 0.1f && prevAngle > 1.0f)
        {
            if (EnableDebugLog)
            {
                Debug.LogWarning($"[旋转连续性] 检测到异常近零值: 当前角度={currentAngle:F4}, 上一帧角度={prevAngle:F4}, 使用上一帧值");
            }
            // 直接使用上一帧的值，不更新previousRotationVector
            return previousRotationVector;
        }

        // 如果角度真的接近0（且上一帧也接近0），正常处理
        if (currentAngle < 0.001f)
        {
            previousRotationVector = currentRotVec;
            return currentRotVec;
        }

        // 计算与上一帧的差值
        Vector3 diff = currentRotVec - previousRotationVector;
        float diffMag = diff.magnitude;

        // 如果差值较小（小于π/2），认为是连续的
        if (diffMag < PI * 0.5f)
        {
            previousRotationVector = currentRotVec;
            return currentRotVec;
        }

        // ========== 尝试多种等效表示 ==========
        Vector3 axis = currentRotVec / currentAngle;
        
        // 等效表示1: (2π - θ) 绕 -axis
        float altAngle1 = TWO_PI - currentAngle;
        Vector3 altRotVec1 = -axis * altAngle1;
        
        // 等效表示2: 负角度表示 (-θ) 绕 -axis = θ 绕 axis 的逆
        Vector3 altRotVec2 = -currentRotVec;
        
        // 等效表示3: (θ - 2π) 绕 axis（负角度区间）
        float altAngle3 = currentAngle - TWO_PI;
        Vector3 altRotVec3 = axis * altAngle3;

        // 比较所有表示与上一帧的差异，选择最小的
        float diffOriginal = (currentRotVec - previousRotationVector).magnitude;
        float diffAlt1 = (altRotVec1 - previousRotationVector).magnitude;
        float diffAlt2 = (altRotVec2 - previousRotationVector).magnitude;
        float diffAlt3 = (altRotVec3 - previousRotationVector).magnitude;

        // 找最小差值
        float minDiff = diffOriginal;
        Vector3 result = currentRotVec;
        
        if (diffAlt1 < minDiff) { minDiff = diffAlt1; result = altRotVec1; }
        if (diffAlt2 < minDiff) { minDiff = diffAlt2; result = altRotVec2; }
        if (diffAlt3 < minDiff) { minDiff = diffAlt3; result = altRotVec3; }

        // ========== 最后检查：限制单帧最大变化 ==========
        Vector3 finalDiff = result - previousRotationVector;
        float finalDiffMag = finalDiff.magnitude;
        
        if (finalDiffMag > MAX_SINGLE_FRAME_CHANGE)
        {
            if (EnableDebugLog)
            {
                Debug.LogWarning($"[旋转连续性] 变化过大({finalDiffMag:F4}rad)，限制到{MAX_SINGLE_FRAME_CHANGE}rad");
            }
            // 限制变化量
            result = previousRotationVector + finalDiff.normalized * MAX_SINGLE_FRAME_CHANGE;
        }

        if (EnableDebugLog && result != currentRotVec)
        {
            Debug.Log($"[旋转连续性] 使用等效表示: ({currentRotVec.x:F3},{currentRotVec.y:F3},{currentRotVec.z:F3}) → ({result.x:F3},{result.y:F3},{result.z:F3})");
        }

        previousRotationVector = result;
        return result;
    }

    /// <summary>
    /// 生成movej命令（用于轨迹起始点的缓慢移动）
    /// 
    /// movej功能说明：
    ///   - 使用关节空间插值，轨迹在关节空间是线性的，笛卡尔空间可能不是直线
    ///   - 适合点到点运动，不要求笛卡尔空间轨迹精度
    /// 
    /// URScript命令格式:
    ///   movej(get_inverse_kin(p[x,y,z,rx,ry,rz], qnear=[j0,j1,j2,j3,j4,j5]), a, v, t, r)
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
    /// </summary>
    /// <param name="frameData">帧数据（包含位置和姿态）</param>
    /// <param name="acceleration">关节加速度(rad/s²), 推荐0.3-0.5表示缓慢</param>
    /// <param name="velocity">关节速度(rad/s), 推荐0.2-0.3表示缓慢</param>
    /// <param name="time">运动时间(s), 设为0则使用a和v参数</param>
    /// <param name="blendRadius">混合半径(m), 单点移动设为0</param>
    /// <param name="currentJointAngles">当前关节角度数组(6个double值)</param>
    /// <param name="useTcpDirectMode">是否使用TCP直接模式（由调用方控制，与后续帧保持一致）</param>
    /// <returns>URScript movej命令字符串</returns>
    public static string GenerateMovejCommand(FrameData frameData, 
                                               double acceleration = 0.5, 
                                               double velocity = 0.3,
                                               double time = 0.0,
                                               double blendRadius = 0.0,
                                               double[] currentJointAngles = null,
                                               bool useTcpDirectMode = false)
    {
        try
        {
            // 获取UR基座坐标系位姿
            Vector3 posUr_m;
            Vector3 rotUr_rad;

            // 获取位姿数据（根据调用方指定的模式，确保与后续servoj帧一致）
            // useTcpDirectMode 由调用方传入，与 RigidBodyServojController.useRecordedTcpData 对应
            if (useTcpDirectMode && frameData.HasTcpData && frameData.TcpPose != null)
            {
                // TCP直接模式
                posUr_m = new Vector3(
                    (float)frameData.TcpPose.X,
                    (float)frameData.TcpPose.Y,
                    (float)frameData.TcpPose.Z);
                rotUr_rad = new Vector3(
                    (float)frameData.TcpPose.RX,
                    (float)frameData.TcpPose.RY,
                    (float)frameData.TcpPose.RZ);
                
                if (EnableDebugLog)
                {
                    Debug.Log($"[Movej] TCP直接模式 - p[{posUr_m.x:F4},{posUr_m.y:F4},{posUr_m.z:F4}]");
                }
            }
            else
            {
                // Tracker+坐标转换模式（与GenerateServojCommand保持完全一致的处理流程）
                Vector3 posMm = new Vector3(
                    (float)frameData.Position.X,
                    (float)frameData.Position.Y,
                    (float)frameData.Position.Z);
                Quaternion quat = frameData.GetQuaternion();

                if (EnableDebugLog)
                {
                    Debug.Log($"[Movej] Tracker模式 - 原始位置(mm): ({posMm.x:F2}, {posMm.y:F2}, {posMm.z:F2})");
                }

                // 应用Tracker偏移（与GenerateServojCommand完全一致的逻辑）
                if (EnableTrackerOffset)
                {
                    // 位置偏移：将本地偏移转换到世界坐标系
                    // worldOffset = rotation * localOffset
                    Vector3 worldOffsetMm = quat * TrackerPositionOffset;
                    posMm = posMm + worldOffsetMm;

                    // 旋转偏移：在本地坐标系中应用
                    if (TrackerRotationOffset != Vector3.zero)
                    {
                        Quaternion rotationOffsetQuat = Quaternion.Euler(TrackerRotationOffset);
                        quat = quat * rotationOffsetQuat;
                    }

                    if (EnableDebugLog)
                    {
                        Debug.Log($"[Movej] 应用Tracker偏移后位置(mm): ({posMm.x:F2}, {posMm.y:F2}, {posMm.z:F2})");
                    }
                }

                // 应用Kabsch校正
                if (EnableKabschAlignment)
                {
                    Vector3 posMeter = posMm * 0.001f;
                    ApplyKabschTransform(posMeter, quat, out Vector3 transformedPos, out Quaternion transformedQuat);
                    posMm = transformedPos * 1000f;
                    quat = transformedQuat;
                }

                // 坐标转换
                if (EnableCoordinateTransform)
                {
                    SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                        posMm, quat, posInMillimeters: true,
                        out posUr_m, out rotUr_rad);
                }
                else
                {
                    posUr_m = posMm * 0.001f;
                    // 将四元数转为旋转矢量 - 使用与TrackerPoseCapture相同的方法
                    Vector3 axis;
                    float angle;
                    quat.ToAngleAxis(out angle, out axis);
                    angle *= Mathf.Deg2Rad;  // 转换为弧度
                    rotUr_rad = axis * angle;
                }
                
                if (EnableDebugLog)
                {
                    Debug.Log($"[Movej] 转换后UR位姿 - p[{posUr_m.x:F4},{posUr_m.y:F4},{posUr_m.z:F4},{rotUr_rad.x:F3},{rotUr_rad.y:F3},{rotUr_rad.z:F3}]");
                }
            }

            // 获取当前关节角度（用于逆运动学参考）
            double[] jointAngles = currentJointAngles;
            if (jointAngles == null || jointAngles.Length != 6)
            {
                jointAngles = new double[]
                {
                    ur_data_processing.UR_Stream_Data.J_Orientation[0],
                    ur_data_processing.UR_Stream_Data.J_Orientation[1],
                    ur_data_processing.UR_Stream_Data.J_Orientation[2],
                    ur_data_processing.UR_Stream_Data.J_Orientation[3],
                    ur_data_processing.UR_Stream_Data.J_Orientation[4],
                    ur_data_processing.UR_Stream_Data.J_Orientation[5]
                };
            }

            // 构建movej命令
            string command = string.Format(CultureInfo.InvariantCulture,
                "movej(get_inverse_kin(p[{0:F6},{1:F6},{2:F6},{3:F6},{4:F6},{5:F6}], qnear=[{6:F6},{7:F6},{8:F6},{9:F6},{10:F6},{11:F6}]), a={12:F4}, v={13:F4}, t={14:F1}, r={15:F1})\n",
                posUr_m.x, posUr_m.y, posUr_m.z,
                rotUr_rad.x, rotUr_rad.y, rotUr_rad.z,
                jointAngles[0], jointAngles[1], jointAngles[2],
                jointAngles[3], jointAngles[4], jointAngles[5],
                acceleration, velocity, time, blendRadius);

            return command;
        }
        catch (Exception e)
        {
            Debug.LogError($"[RigidBodyServojCommandGenerator] 生成movej命令失败: {e.Message}");
            return string.Empty;
        }
    }
}

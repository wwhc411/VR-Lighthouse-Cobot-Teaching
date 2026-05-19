using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

/// <summary>
/// MoveL 批量脚本生成器
/// 
/// 功能：将完整轨迹转换为URScript脚本（使用movel指令），一次性发送执行
/// 
/// 与其他回放模式的区别：
///   - Servoj：实时伺服控制，每帧精确跟踪，适合高精度轨迹还原（125Hz）
///   - MoveJ：关节空间插值运动，轨迹在笛卡尔空间可能弯曲
///   - MoveL：笛卡尔空间线性运动，TCP走直线，适合需要直线轨迹的场景
/// 
/// MoveL特点：
///   - 笛卡尔空间轨迹是直线（点与点之间）
///   - 支持混合半径(blend radius)实现平滑连续运动
///   - 在关节空间可能出现非线性运动
///   - 适合焊接、涂胶等需要直线路径的应用
/// 
/// 脚本结构：
///   def trajectory_replay_movel():
///     P0 = p[x, y, z, rx, ry, rz]
///     P1 = p[...]
///     ...
///     movel(P0, a=..., v=..., t=..., r=blend_radius)
///     movel(P1, a=..., v=..., t=..., r=blend_radius)
///     ...
///     stopl(5)
///   end
///   trajectory_replay_movel()
/// 
/// 更新日期: 2026-02-03
/// </summary>
public static class RigidBodyMovelScriptGenerator
{
    #region 脚本参数结构

    /// <summary>
    /// MoveL批量脚本生成参数
    /// </summary>
    [Serializable]
    public struct MovelScriptParameters
    {
        [Tooltip("线性加速度(m/s²)，推荐0.5-2.0")]
        public float Acceleration;

        [Tooltip("线性速度(m/s)，推荐0.1-0.5")]
        public float Velocity;

        [Tooltip("运动时间(s)，>0时忽略a和v参数，设为0使用a/v控制")]
        public float Time;

        [Tooltip("混合半径(m)，用于平滑连接多个movel指令\n" +
                 "0=精确到达每个点\n" +
                 ">0=在混合半径内开始下一段运动（更平滑）")]
        public float BlendRadius;

        [Tooltip("点采样步长（1=全部点，2=隔点采样）")]
        public int PointStep;

        [Tooltip("启用坐标转换（SteamVR → UR Base）")]
        public bool EnableCoordinateTransform;

        [Tooltip("启用Kabsch点云对齐校正")]
        public bool EnableKabschAlignment;

        [Tooltip("启用旋转矢量连续性校正")]
        public bool EnableRotationContinuity;

        [Tooltip("启用Tracker本地坐标偏移")]
        public bool EnableTrackerOffset;

        [Tooltip("Tracker位置偏移(mm)")]
        public Vector3 TrackerPositionOffset;

        [Tooltip("Tracker旋转偏移(度)")]
        public Vector3 TrackerRotationOffset;

        /// <summary>
        /// 默认参数
        /// </summary>
        public static MovelScriptParameters Default => new MovelScriptParameters
        {
            Acceleration = 1.2f,
            Velocity = 0.25f,
            Time = 0f,
            BlendRadius = 0.01f,  // 10mm混合半径
            PointStep = 1,
            EnableCoordinateTransform = true,
            EnableKabschAlignment = false,
            EnableRotationContinuity = true,
            EnableTrackerOffset = false,
            TrackerPositionOffset = Vector3.zero,
            TrackerRotationOffset = Vector3.zero
        };

        /// <summary>
        /// 验证参数有效性
        /// </summary>
        public bool Validate(out string errorMessage)
        {
            if (Acceleration < 0.01f || Acceleration > 10f)
            {
                errorMessage = $"加速度超出范围(0.01-10 m/s²): {Acceleration}";
                return false;
            }

            if (Velocity < 0.01f || Velocity > 2f)
            {
                errorMessage = $"速度超出范围(0.01-2 m/s): {Velocity}";
                return false;
            }

            if (Time < 0f)
            {
                errorMessage = $"时间不能为负: {Time}";
                return false;
            }

            if (BlendRadius < 0f || BlendRadius > 0.5f)
            {
                errorMessage = $"混合半径超出范围(0-0.5m): {BlendRadius}";
                return false;
            }

            if (PointStep < 1 || PointStep > 100)
            {
                errorMessage = $"采样步长超出范围(1-100): {PointStep}";
                return false;
            }

            errorMessage = "参数验证通过";
            return true;
        }
    }

    #endregion

    #region 脚本生成结果

    /// <summary>
    /// 脚本生成结果
    /// </summary>
    public struct ScriptGenerationResult
    {
        public bool Success;
        public string Script;
        public int TotalPoints;
        public int ProcessedPoints;
        public long ScriptSizeBytes;
        public float EstimatedDurationSeconds;
        public string ErrorMessage;
    }

    #endregion

    #region 常量

    /// <summary>
    /// 每个脚本的最大点数（UR脚本缓冲区限制约2MB）
    /// MoveL指令与MoveJ类似，可以容纳较多点
    /// </summary>
    public const int MAX_POINTS_PER_SCRIPT = 10000;

    /// <summary>
    /// 是否启用调试日志
    /// </summary>
    public static bool EnableDebugLog = false;

    #endregion

    #region 旋转连续性状态

    private static Vector3 _previousRotationVector = Vector3.zero;
    private static bool _hasPreviousRotation = false;

    /// <summary>
    /// 重置旋转连续性状态（开始新轨迹时调用）
    /// </summary>
    public static void ResetRotationContinuityState()
    {
        _previousRotationVector = Vector3.zero;
        _hasPreviousRotation = false;
    }

    #endregion

    #region Kabsch变换参数

    private static Matrix<double> _kabschRotationMatrix = null;
    private static MathNet.Numerics.LinearAlgebra.Vector<double> _kabschTranslationVector = null;

    /// <summary>
    /// 设置Kabsch变换参数
    /// </summary>
    public static void SetKabschTransform(Matrix<double> rotationMatrix,
        MathNet.Numerics.LinearAlgebra.Vector<double> translationVector)
    {
        _kabschRotationMatrix = rotationMatrix;
        _kabschTranslationVector = translationVector;
    }

    /// <summary>
    /// 清除Kabsch变换参数
    /// </summary>
    public static void ClearKabschTransform()
    {
        _kabschRotationMatrix = null;
        _kabschTranslationVector = null;
    }

    #endregion

    #region 主要API

    /// <summary>
    /// 从轨迹数据生成完整的MoveL URScript脚本
    /// </summary>
    /// <param name="captureData">CSV加载的轨迹数据</param>
    /// <param name="parameters">脚本生成参数</param>
    /// <param name="useTcpMode">是否使用TCP直接回放模式</param>
    /// <returns>脚本生成结果</returns>
    public static ScriptGenerationResult GenerateTrajectoryScript(
        RigidBodyCaptureData captureData,
        MovelScriptParameters parameters,
        bool useTcpMode = false)
    {
        var result = new ScriptGenerationResult();

        try
        {
            // 验证参数
            if (!parameters.Validate(out string errorMessage))
            {
                result.Success = false;
                result.ErrorMessage = errorMessage;
                return result;
            }

            // 验证数据
            if (captureData == null || captureData.FrameData == null || captureData.FrameData.Count == 0)
            {
                result.Success = false;
                result.ErrorMessage = "轨迹数据为空";
                return result;
            }

            List<FrameData> frames = captureData.FrameData;
            int frameCount = frames.Count;
            result.TotalPoints = frameCount;

            // 检查是否需要分段
            int effectivePoints = (frameCount + parameters.PointStep - 1) / parameters.PointStep;
            if (effectivePoints > MAX_POINTS_PER_SCRIPT)
            {
                Debug.LogWarning($"[MoveL脚本] 点数({effectivePoints})超过限制({MAX_POINTS_PER_SCRIPT})，建议增大采样步长或分段发送");
            }

            // 重置旋转连续性状态
            ResetRotationContinuityState();

            // 构建脚本
            StringBuilder sb = new StringBuilder();

            // ========== 脚本头部 ==========
            sb.AppendLine("def trajectory_replay_movel():");
            sb.AppendLine("");

            // ========== 第一遍：生成位姿定义 ==========
            List<int> processedIndices = new List<int>();
            int pointIndex = 0;

            for (int i = 0; i < frameCount; i += parameters.PointStep)
            {
                FrameData frame = frames[i];

                // 检查位置有效性
                if (!frame.IsPositionValid())
                {
                    if (EnableDebugLog)
                    {
                        Debug.LogWarning($"[MoveL脚本] 帧{i}位置无效，跳过");
                    }
                    continue;
                }

                // 获取UR基座坐标系位姿
                Vector3 posUr_m;
                Vector3 rotUr_rad;

                if (useTcpMode && frame.HasTcpData && frame.TcpPose != null)
                {
                    // TCP直接模式：数据已是UR基座坐标系
                    posUr_m = new Vector3(
                        (float)frame.TcpPose.X,
                        (float)frame.TcpPose.Y,
                        (float)frame.TcpPose.Z);
                    rotUr_rad = new Vector3(
                        (float)frame.TcpPose.RX,
                        (float)frame.TcpPose.RY,
                        (float)frame.TcpPose.RZ);

                    // TCP模式下的Kabsch校正
                    if (parameters.EnableKabschAlignment && _kabschRotationMatrix != null)
                    {
                        ApplyKabschTransform(ref posUr_m);
                    }
                }
                else
                {
                    // Tracker + 坐标转换模式
                    ProcessTrackerPose(frame, parameters, out posUr_m, out rotUr_rad);
                }

                // 旋转矢量连续性校正
                if (parameters.EnableRotationContinuity)
                {
                    rotUr_rad = EnsureRotationVectorContinuity(rotUr_rad);
                }

                // 写入位姿定义
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
                    "  P{0} = p[{1:F6}, {2:F6}, {3:F6}, {4:F6}, {5:F6}, {6:F6}]",
                    pointIndex, posUr_m.x, posUr_m.y, posUr_m.z,
                    rotUr_rad.x, rotUr_rad.y, rotUr_rad.z));

                processedIndices.Add(pointIndex);
                pointIndex++;
            }

            result.ProcessedPoints = pointIndex;

            if (pointIndex == 0)
            {
                result.Success = false;
                result.ErrorMessage = "没有有效的轨迹点";
                return result;
            }

            sb.AppendLine("");

            // ========== 第二遍：生成MoveL运动指令 ==========
            for (int i = 0; i < processedIndices.Count; i++)
            {
                int idx = processedIndices[i];

                // 计算混合半径：最后一个点不使用混合（必须精确到达）
                float blendRadius = (i == processedIndices.Count - 1) ? 0f : parameters.BlendRadius;

                // 生成movel命令
                // 格式: movel(pose, a=加速度, v=速度, t=时间, r=混合半径)
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
                    "  movel(P{0}, a={1:F4}, v={2:F4}, t={3:F2}, r={4:F4})",
                    idx,
                    parameters.Acceleration,
                    parameters.Velocity,
                    parameters.Time,
                    blendRadius));
            }

            // ========== 脚本尾部 ==========
            sb.AppendLine("");
            sb.AppendLine("  stopl(5)");  // 平滑停止
            sb.AppendLine("end");
            sb.AppendLine("");
            sb.AppendLine("trajectory_replay_movel()");  // 调用函数执行

            // 计算预估时长
            // MoveL时长难以精确估计，这里给一个粗略估计
            // 假设每个点平均0.1秒（取决于点间距和速度参数）
            float avgTimePerPoint = parameters.Time > 0 ? parameters.Time : 0.1f;
            result.EstimatedDurationSeconds = pointIndex * avgTimePerPoint;

            // 生成结果
            result.Script = sb.ToString();
            result.ScriptSizeBytes = Encoding.UTF8.GetByteCount(result.Script);
            result.Success = true;

            if (EnableDebugLog)
            {
                Debug.Log($"[MoveL脚本] 生成完成: {pointIndex}点, {result.ScriptSizeBytes}字节, 预计{result.EstimatedDurationSeconds:F1}秒");
            }

            return result;
        }
        catch (Exception ex)
        {
            result.Success = false;
            result.ErrorMessage = $"脚本生成异常: {ex.Message}";
            Debug.LogError($"[MoveL脚本] {result.ErrorMessage}\n{ex.StackTrace}");
            return result;
        }
    }

    /// <summary>
    /// 将脚本保存到文件（用于调试）
    /// </summary>
    public static bool SaveScriptToFile(string script, string filePath)
    {
        try
        {
            string directory = Path.GetDirectoryName(filePath);
            if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
            {
                Directory.CreateDirectory(directory);
            }

            File.WriteAllText(filePath, script, Encoding.UTF8);
            Debug.Log($"[MoveL脚本] 脚本已保存到: {filePath}");
            return true;
        }
        catch (Exception ex)
        {
            Debug.LogError($"[MoveL脚本] 保存失败: {ex.Message}");
            return false;
        }
    }

    /// <summary>
    /// 发送脚本到UR控制器
    /// 注意：批量脚本只能发送一次！
    /// </summary>
    public static bool SendScriptToUR(string script)
    {
        if (string.IsNullOrEmpty(script))
        {
            Debug.LogError("[MoveL脚本] 脚本为空，无法发送");
            return false;
        }

        try
        {
            byte[] scriptBytes = Encoding.UTF8.GetBytes(script);

            // 设置命令缓冲区
            ur_data_processing.UR_Control_Data.aux_command_str = script;
            ur_data_processing.UR_Control_Data.command = scriptBytes;

            // 激活发送
            ur_data_processing.UR_Control_Data.manual_send_active = true;

            // 等待发送完成
            System.Threading.Thread.Sleep(20);

            // 立即关闭，防止重复发送
            ur_data_processing.UR_Control_Data.manual_send_active = false;

            // 清空命令缓冲区
            ur_data_processing.UR_Control_Data.command = new byte[0];
            ur_data_processing.UR_Control_Data.aux_command_str = "";

            Debug.Log($"[MoveL脚本] 脚本已发送到UR，大小: {scriptBytes.Length} 字节");
            return true;
        }
        catch (Exception ex)
        {
            Debug.LogError($"[MoveL脚本] 发送失败: {ex.Message}");
            return false;
        }
    }

    /// <summary>
    /// 发送紧急停止命令
    /// </summary>
    public static void SendEmergencyStop()
    {
        string stopCommand = "stopl(2)\n";
        byte[] commandBytes = Encoding.UTF8.GetBytes(stopCommand);

        ur_data_processing.UR_Control_Data.aux_command_str = stopCommand;
        ur_data_processing.UR_Control_Data.command = commandBytes;
        ur_data_processing.UR_Control_Data.manual_send_active = true;

        System.Threading.Thread.Sleep(20);

        ur_data_processing.UR_Control_Data.manual_send_active = false;
        ur_data_processing.UR_Control_Data.command = new byte[0];
        ur_data_processing.UR_Control_Data.aux_command_str = "";

        Debug.Log("[MoveL脚本] 已发送紧急停止命令");
    }

    #endregion

    #region 内部方法

    /// <summary>
    /// 处理Tracker位姿（坐标转换）
    /// </summary>
    private static void ProcessTrackerPose(FrameData frame, MovelScriptParameters parameters,
        out Vector3 posUr_m, out Vector3 rotUr_rad)
    {
        // 提取原始位置(mm)和四元数
        Vector3 posSteamVr_mm = new Vector3(
            (float)frame.Position.X,
            (float)frame.Position.Y,
            (float)frame.Position.Z);
        Quaternion quatSteamVr = frame.GetQuaternion();

        // 应用Tracker本地偏移（可选）
        if (parameters.EnableTrackerOffset)
        {
            Vector3 worldOffsetMm = quatSteamVr * parameters.TrackerPositionOffset;
            posSteamVr_mm = posSteamVr_mm + worldOffsetMm;

            if (parameters.TrackerRotationOffset != Vector3.zero)
            {
                Quaternion rotationOffsetQuat = Quaternion.Euler(parameters.TrackerRotationOffset);
                quatSteamVr = quatSteamVr * rotationOffsetQuat;
            }
        }

        // 应用Kabsch校正（可选，仅位置）
        if (parameters.EnableKabschAlignment && _kabschRotationMatrix != null)
        {
            Vector3 posSteamVr_m = posSteamVr_mm * 0.001f;
            ApplyKabschTransform(ref posSteamVr_m);
            posSteamVr_mm = posSteamVr_m * 1000f;
        }

        // 坐标转换
        if (parameters.EnableCoordinateTransform)
        {
            // 调用手眼标定坐标转换器
            handeye.SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                posSteamVr_mm,
                quatSteamVr,
                posInMillimeters: true,
                out posUr_m,
                out rotUr_rad);
        }
        else
        {
            // 不转换，直接使用（假设已是UR坐标）
            posUr_m = posSteamVr_mm * 0.001f;
            rotUr_rad = frame.GetRotationVector();
        }
    }

    /// <summary>
    /// 应用Kabsch变换（仅位置）
    /// </summary>
    private static void ApplyKabschTransform(ref Vector3 position)
    {
        if (_kabschRotationMatrix == null || _kabschTranslationVector == null)
            return;

        var posVec = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(new double[]
        {
            position.x, position.y, position.z
        });

        var transformedVec = _kabschRotationMatrix.Multiply(posVec) + _kabschTranslationVector;

        position = new Vector3(
            (float)transformedVec[0],
            (float)transformedVec[1],
            (float)transformedVec[2]);
    }

    /// <summary>
    /// 确保旋转矢量连续性
    /// 避免 ±π 附近的跳变
    /// </summary>
    private static Vector3 EnsureRotationVectorContinuity(Vector3 currentRot)
    {
        if (!_hasPreviousRotation)
        {
            _previousRotationVector = currentRot;
            _hasPreviousRotation = true;
            return currentRot;
        }

        // 计算与上一帧的差值
        Vector3 diff = currentRot - _previousRotationVector;

        // 检查每个分量是否需要调整
        Vector3 adjustedRot = currentRot;

        // X分量
        if (Mathf.Abs(diff.x) > Mathf.PI)
        {
            adjustedRot.x = currentRot.x - Mathf.Sign(diff.x) * 2 * Mathf.PI;
        }

        // Y分量
        if (Mathf.Abs(diff.y) > Mathf.PI)
        {
            adjustedRot.y = currentRot.y - Mathf.Sign(diff.y) * 2 * Mathf.PI;
        }

        // Z分量
        if (Mathf.Abs(diff.z) > Mathf.PI)
        {
            adjustedRot.z = currentRot.z - Mathf.Sign(diff.z) * 2 * Mathf.PI;
        }

        _previousRotationVector = adjustedRot;
        return adjustedRot;
    }

    #endregion
}

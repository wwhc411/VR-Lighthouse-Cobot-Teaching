using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

/// <summary>
/// Servoj 批量脚本生成器
/// 
/// 功能：将完整轨迹转换为URScript脚本，一次性发送执行
/// 
/// 与单帧发送的区别：
///   - 单帧发送：每帧通过Unity协程发送一条命令，受网络延迟和Unity帧率影响
///   - 批量脚本：构建完整URScript脚本，一次性发送，由机器人内部精确执行
/// 
/// 脚本结构：
///   def trajectory_replay():
///     qnear = get_actual_joint_positions()
///     P0 = p[x, y, z, rx, ry, rz]
///     P1 = p[...]
///     ...
///     servoj(get_inverse_kin(P0, qnear=...), t=0.008, lookahead_time=0.1, gain=300)
///     servoj(get_inverse_kin(P1, qnear=...), t=0.008, ...)
///     ...
///     stopl(5)
///   end
///   trajectory_replay()
/// 
/// 更新日期: 2026-01-29
/// </summary>
public static class RigidBodyServojScriptGenerator
{
    #region 脚本参数结构

    /// <summary>
    /// 批量脚本生成参数
    /// </summary>
    [Serializable]
    public struct ServojScriptParameters
    {
        [Tooltip("发送频率(Hz) → 计算时间步长 t = 1/频率")]
        public float SendFrequencyHz;

        [Tooltip("Servoj前瞻时间(s)，推荐0.06-0.1")]
        public float LookAheadTime;

        [Tooltip("Servoj控制增益，推荐300-500")]
        public float Gain;

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

        [Tooltip("第一帧MoveJ关节加速度(rad/s²)")]
        public float FirstFrameAcceleration;

        [Tooltip("第一帧MoveJ关节速度(rad/s)")]
        public float FirstFrameVelocity;

        /// <summary>
        /// 默认参数（125Hz控制频率）
        /// </summary>
        public static ServojScriptParameters Default => new ServojScriptParameters
        {
            SendFrequencyHz = 125f,
            LookAheadTime = 0.1f,
            Gain = 300f,
            PointStep = 1,
            EnableCoordinateTransform = true,
            EnableKabschAlignment = false,
            EnableRotationContinuity = true,
            EnableTrackerOffset = false,
            TrackerPositionOffset = Vector3.zero,
            TrackerRotationOffset = Vector3.zero,
            FirstFrameAcceleration = 0.5f,
            FirstFrameVelocity = 0.3f
        };

        /// <summary>
        /// 验证参数有效性
        /// </summary>
        public bool Validate(out string errorMessage)
        {
            if (SendFrequencyHz < 10f || SendFrequencyHz > 500f)
            {
                errorMessage = $"频率超出范围(10-500Hz): {SendFrequencyHz}";
                return false;
            }

            if (LookAheadTime < 0.03f || LookAheadTime > 0.2f)
            {
                errorMessage = $"前瞻时间超出范围(0.03-0.2s): {LookAheadTime}";
                return false;
            }

            if (Gain < 100f || Gain > 2000f)
            {
                errorMessage = $"增益超出范围(100-2000): {Gain}";
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
    /// 每点约170字节，安全阈值设为8000点
    /// </summary>
    public const int MAX_POINTS_PER_SCRIPT = 8000;

    /// <summary>
    /// 是否启用调试日志
    /// </summary>
    public static bool EnableDebugLog = false;

    #endregion

    #region 旋转连续性状态（独立于单帧发送）

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

    #region Kabsch变换参数（独立于单帧发送）

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
    /// 从轨迹数据生成完整的URScript脚本
    /// </summary>
    /// <param name="captureData">CSV加载的轨迹数据</param>
    /// <param name="parameters">脚本生成参数</param>
    /// <param name="useTcpMode">是否使用TCP直接回放模式</param>
    /// <returns>脚本生成结果</returns>
    public static ScriptGenerationResult GenerateTrajectoryScript(
        RigidBodyCaptureData captureData,
        ServojScriptParameters parameters,
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
                Debug.LogWarning($"[批量脚本] 点数({effectivePoints})超过限制({MAX_POINTS_PER_SCRIPT})，建议增大采样步长或分段发送");
            }

            // 计算时间步长
            double timeStep = 1.0 / parameters.SendFrequencyHz;

            // 重置旋转连续性状态
            ResetRotationContinuityState();

            // 构建脚本
            StringBuilder sb = new StringBuilder();
            
            // ========== 脚本头部 ==========
            sb.AppendLine("def trajectory_replay():");
            sb.AppendLine("  qnear = get_actual_joint_positions()");
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
                        Debug.LogWarning($"[批量脚本] 帧{i}位置无效，跳过");
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

            // ========== 第二遍：生成运动指令 ==========
            // 第一帧使用movej缓慢移动到起始点，其余帧使用servoj
            for (int i = 0; i < processedIndices.Count; i++)
            {
                int idx = processedIndices[i];
                
                if (i == 0)
                {
                    // 第一帧：使用movej匀速缓慢移动到轨迹起始点
                    // movej工作原理：
                    //   - 使用关节空间插值，轨迹在关节空间是线性的
                    //   - a: 关节加速度(rad/s²), v: 关节速度(rad/s)
                    //   - t: 运动时间(s), 若t>0则忽略a和v参数
                    //   - r: 混合半径(m), 用于平滑连接（单点移动设为0）
                    sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
                        "  movej(get_inverse_kin(P{0}, qnear=qnear), a={1:F4}, v={2:F4}, t={3:F1}, r={4:F1})",
                        idx, 
                        parameters.FirstFrameAcceleration,  // 使用参数配置的加速度
                        parameters.FirstFrameVelocity,      // 使用参数配置的速度
                        0.0,    // t: 0表示使用a和v参数控制速度
                        0.0));  // r: 混合半径0（单点移动无需过渡）
                }
                else
                {
                    // 其余帧：使用servoj实时跟踪
                    // servoj工作原理：
                    //   - servoj有内部队列，命令会被缓冲并按时间参数t依次平滑执行
                    //   - 不需要额外的sleep或sync，连续排列即可
                    //   - t参数控制每个点的目标执行时间
                    //   - lookahead_time用于轨迹平滑预测
                    //   - gain控制伺服刚度
                    sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
                        "  servoj(get_inverse_kin(P{0}, qnear=get_actual_joint_positions()), " +
                        "t={1:F6}, lookahead_time={2:F4}, gain={3:F0})",
                        idx, timeStep, parameters.LookAheadTime, parameters.Gain));
                }
            }

            // ========== 脚本尾部 ==========
            sb.AppendLine("");
            sb.AppendLine("  stopl(5)");  // 平滑停止
            sb.AppendLine("end");
            sb.AppendLine("");
            sb.AppendLine("trajectory_replay()");  // 调用函数执行

            // 生成结果
            result.Script = sb.ToString();
            result.ScriptSizeBytes = Encoding.UTF8.GetByteCount(result.Script);
            result.EstimatedDurationSeconds = (float)(pointIndex * timeStep);
            result.Success = true;

            if (EnableDebugLog)
            {
                Debug.Log($"[批量脚本] 生成完成: {pointIndex}点, {result.ScriptSizeBytes}字节, 预计{result.EstimatedDurationSeconds:F1}秒");
            }

            return result;
        }
        catch (Exception ex)
        {
            result.Success = false;
            result.ErrorMessage = $"脚本生成异常: {ex.Message}";
            Debug.LogError($"[批量脚本] {result.ErrorMessage}\n{ex.StackTrace}");
            return result;
        }
    }

    /// <summary>
    /// 生成分段脚本（用于超长轨迹）
    /// </summary>
    public static List<ScriptGenerationResult> GenerateSegmentedScripts(
        RigidBodyCaptureData captureData,
        ServojScriptParameters parameters,
        bool useTcpMode = false,
        int maxPointsPerSegment = MAX_POINTS_PER_SCRIPT)
    {
        var results = new List<ScriptGenerationResult>();
        
        if (captureData == null || captureData.FrameData == null)
        {
            results.Add(new ScriptGenerationResult 
            { 
                Success = false, 
                ErrorMessage = "轨迹数据为空" 
            });
            return results;
        }

        int frameCount = captureData.FrameData.Count;
        int effectivePoints = (frameCount + parameters.PointStep - 1) / parameters.PointStep;
        
        if (effectivePoints <= maxPointsPerSegment)
        {
            // 不需要分段
            results.Add(GenerateTrajectoryScript(captureData, parameters, useTcpMode));
            return results;
        }

        // 需要分段
        int framesPerSegment = maxPointsPerSegment * parameters.PointStep;
        int segments = (frameCount + framesPerSegment - 1) / framesPerSegment;

        Debug.Log($"[批量脚本] 轨迹过长，分{segments}段发送");

        for (int seg = 0; seg < segments; seg++)
        {
            int startFrame = seg * framesPerSegment;
            int endFrame = Math.Min(startFrame + framesPerSegment, frameCount);

            // 创建子数据集
            var segmentData = new RigidBodyCaptureData
            {
                Metadata = captureData.Metadata,
                FrameData = captureData.FrameData.GetRange(startFrame, endFrame - startFrame)
            };

            // 生成分段脚本（注意：第一段需要重置旋转连续性，后续段保持连续）
            if (seg == 0)
            {
                ResetRotationContinuityState();
            }

            var segmentResult = GenerateTrajectoryScript(segmentData, parameters, useTcpMode);
            segmentResult.ErrorMessage = $"第{seg + 1}/{segments}段: {segmentResult.ProcessedPoints}点";
            results.Add(segmentResult);
        }

        return results;
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
            Debug.Log($"[批量脚本] 脚本已保存到: {filePath}");
            return true;
        }
        catch (Exception ex)
        {
            Debug.LogError($"[批量脚本] 保存失败: {ex.Message}");
            return false;
        }
    }

    /// <summary>
    /// 发送脚本到UR控制器
    /// 注意：批量脚本只能发送一次！发送后立即关闭manual_send_active
    /// </summary>
    public static bool SendScriptToUR(string script)
    {
        if (string.IsNullOrEmpty(script))
        {
            Debug.LogError("[批量脚本] 脚本为空，无法发送");
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
            
            // 关键：等待一个控制周期确保发送完成，然后立即关闭
            // UR控制线程以8ms周期运行，等待20ms确保至少发送一次
            System.Threading.Thread.Sleep(20);
            
            // 立即关闭，防止控制线程重复发送脚本
            ur_data_processing.UR_Control_Data.manual_send_active = false;
            
            // 清空命令缓冲区，防止后续摇杆操作误发脚本
            ur_data_processing.UR_Control_Data.command = new byte[0];
            ur_data_processing.UR_Control_Data.aux_command_str = "";

            Debug.Log($"[批量脚本] 脚本已发送到UR，大小: {scriptBytes.Length} 字节（单次发送）");
            return true;
        }
        catch (Exception ex)
        {
            Debug.LogError($"[批量脚本] 发送失败: {ex.Message}");
            return false;
        }
    }

    /// <summary>
    /// 发送紧急停止命令
    /// </summary>
    public static void SendEmergencyStop()
    {
        string stopCommand = "stopj(2)\n";
        byte[] commandBytes = Encoding.UTF8.GetBytes(stopCommand);
        
        ur_data_processing.UR_Control_Data.aux_command_str = stopCommand;
        ur_data_processing.UR_Control_Data.command = commandBytes;
        ur_data_processing.UR_Control_Data.manual_send_active = true;
        
        // 等待发送完成
        System.Threading.Thread.Sleep(20);
        
        // 关闭发送并清空缓冲区
        ur_data_processing.UR_Control_Data.manual_send_active = false;
        ur_data_processing.UR_Control_Data.command = new byte[0];
        ur_data_processing.UR_Control_Data.aux_command_str = "";
        
        Debug.Log("[批量脚本] 已发送紧急停止命令");
    }

    #endregion

    #region 内部方法

    /// <summary>
    /// 处理Tracker位姿（坐标转换）
    /// </summary>
    private static void ProcessTrackerPose(FrameData frame, ServojScriptParameters parameters,
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
    /// 旋转矢量连续性校正（独立状态，不影响单帧发送）
    /// </summary>
    private static Vector3 EnsureRotationVectorContinuity(Vector3 currentRotVec)
    {
        const float PI = Mathf.PI;
        const float TWO_PI = 2f * Mathf.PI;
        const float MAX_SINGLE_FRAME_CHANGE = 0.5f;

        // 第一帧
        if (!_hasPreviousRotation)
        {
            _previousRotationVector = currentRotVec;
            _hasPreviousRotation = true;
            return currentRotVec;
        }

        float currentAngle = currentRotVec.magnitude;
        float prevAngle = _previousRotationVector.magnitude;

        // 异常近零值检测
        if (currentAngle < 0.1f && prevAngle > 1.0f)
        {
            return _previousRotationVector;
        }

        if (currentAngle < 0.001f)
        {
            _previousRotationVector = currentRotVec;
            return currentRotVec;
        }

        // 差值检查
        Vector3 diff = currentRotVec - _previousRotationVector;
        float diffMag = diff.magnitude;

        if (diffMag < PI * 0.5f)
        {
            _previousRotationVector = currentRotVec;
            return currentRotVec;
        }

        // 尝试等效表示
        Vector3 axis = currentRotVec / currentAngle;
        
        float altAngle1 = TWO_PI - currentAngle;
        Vector3 altRotVec1 = -axis * altAngle1;
        Vector3 altRotVec2 = -currentRotVec;
        float altAngle3 = currentAngle - TWO_PI;
        Vector3 altRotVec3 = axis * altAngle3;

        float diffOriginal = (currentRotVec - _previousRotationVector).magnitude;
        float diffAlt1 = (altRotVec1 - _previousRotationVector).magnitude;
        float diffAlt2 = (altRotVec2 - _previousRotationVector).magnitude;
        float diffAlt3 = (altRotVec3 - _previousRotationVector).magnitude;

        float minDiff = diffOriginal;
        Vector3 result = currentRotVec;

        if (diffAlt1 < minDiff) { minDiff = diffAlt1; result = altRotVec1; }
        if (diffAlt2 < minDiff) { minDiff = diffAlt2; result = altRotVec2; }
        if (diffAlt3 < minDiff) { minDiff = diffAlt3; result = altRotVec3; }

        // 限制单帧变化
        Vector3 finalDiff = result - _previousRotationVector;
        float finalDiffMag = finalDiff.magnitude;

        if (finalDiffMag > MAX_SINGLE_FRAME_CHANGE)
        {
            result = _previousRotationVector + finalDiff.normalized * MAX_SINGLE_FRAME_CHANGE;
        }

        _previousRotationVector = result;
        return result;
    }

    #endregion
}

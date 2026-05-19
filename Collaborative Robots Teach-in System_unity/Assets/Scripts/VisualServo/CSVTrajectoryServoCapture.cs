using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using handeye;
using Debug = UnityEngine.Debug;

namespace VisualServo
{
    /// <summary>
    /// CSV轨迹视觉伺服TCP采集控制器
    /// 
    /// 功能概述：
    /// - 读取CSV文件中的Tracker位姿信息
    /// - 对每一帧目标位姿执行：MoveJ移动 → 捕捉TCP → 视觉伺服 → 捕捉TCP
    /// - 将所有帧的MoveJ后TCP位姿和视觉伺服后TCP位姿输出到CSV文件
    /// 
    /// 应用场景：
    /// - 分析MoveJ直接到达与视觉伺服精细调整后的TCP位姿差异
    /// - 评估视觉伺服的补偿效果
    /// - 为轨迹回放提供补偿参考数据
    /// 
    /// 更新日期: 2026-01-31
    /// </summary>
    public class CSVTrajectoryServoCapture : MonoBehaviour
    {
        // ==================== 数据结构定义 ====================

        /// <summary>
        /// TCP位姿数据结构
        /// </summary>
        [Serializable]
        public class TcpPose
        {
            public double X;      // 位置X (米)
            public double Y;      // 位置Y (米)
            public double Z;      // 位置Z (米)
            public double RX;     // 旋转矢量X (弧度)
            public double RY;     // 旋转矢量Y (弧度)
            public double RZ;     // 旋转矢量Z (弧度)
            public DateTime Timestamp;  // 捕捉时间戳
            public bool IsValid;        // 数据是否有效

            public TcpPose()
            {
                IsValid = false;
                Timestamp = DateTime.Now;
            }

            public override string ToString()
            {
                return $"({X:F6}, {Y:F6}, {Z:F6}) m, ({RX:F6}, {RY:F6}, {RZ:F6}) rad";
            }
        }

        /// <summary>
        /// 单帧采集结果数据结构
        /// </summary>
        [Serializable]
        public class FrameCaptureResult
        {
            public int FrameIndex;                    // 帧索引
            public FrameData TargetTrackerPose;       // 目标Tracker位姿
            public TcpPose MovejTcpPose;              // MoveJ后TCP位姿
            public TcpPose ServoTcpPose;              // 视觉伺服后TCP位姿
            public bool Converged;                    // 是否收敛
            public int Iterations;                    // 迭代次数
            public float TcpPosDiff;                  // MoveJ与Servo TCP位置差异(mm)
            public float TcpRotDiff;                  // MoveJ与Servo TCP旋转差异(deg)
            public float FinalTrackerPosError;        // 最终Tracker位置误差(mm) - 收敛后Tracker2与目标位置的差异
            public float FinalTrackerRotError;        // 最终Tracker旋转误差(deg) - 收敛后Tracker2与目标旋转的差异
            public int RetryCount;                    // 重试次数（视觉伺服未收敛时重试）
            public float ProcessingTime;              // 处理耗时(秒)
            public string ErrorMessage;               // 错误信息（如有）
            
            // 收敛后的Tracker2位姿（用于验证稳定性）
            public Vector3 ConvergedTracker2Position;   // 收敛后Tracker2位置(米)
            public Quaternion ConvergedTracker2Rotation; // 收敛后Tracker2旋转
            public bool HasConvergedTrackerPose;         // 是否有收敛后Tracker位姿数据

            public FrameCaptureResult()
            {
                MovejTcpPose = new TcpPose();
                ServoTcpPose = new TcpPose();
                ErrorMessage = "";
                HasConvergedTrackerPose = false;
            }
        }

        // ==================== Inspector 配置 ====================

        [Header("输入配置")]
        [Tooltip("CSV轨迹文件路径（绝对路径或相对于StreamingAssets）")]
        public string csvFilePath = "";

        [Tooltip("起始帧索引（从0开始）")]
        [Range(0, 10000)]
        public int startFrameIndex = 0;

        [Tooltip("结束帧索引（负值表示到末尾）")]
        public int endFrameIndex = -1;

        [Tooltip("帧采样间隔（1表示每帧都处理）")]
        [Range(1, 100)]
        public int frameSamplingInterval = 1;

        [Header("输出配置")]
        [Tooltip("输出CSV文件路径（为空则自动生成到输入文件同目录）")]
        public string outputCsvPath = "";

        [Header("第一帧MoveJ参数（慢速移动）")]
        [Tooltip("第一帧关节加速度（rad/s²）")]
        [Range(0.1f, 5f)]
        public float firstFrameAcceleration = 0.5f;

        [Tooltip("第一帧关节速度（rad/s）")]
        [Range(0.1f, 2f)]
        public float firstFrameVelocity = 0.3f;

        [Tooltip("第一帧MoveJ等待时间（秒）")]
        [Range(1f, 20f)]
        public float firstFrameWaitTime = 5.0f;

        [Header("正常帧MoveJ参数")]
        [Tooltip("正常帧关节加速度（rad/s²）")]
        [Range(0.1f, 10f)]
        public float normalMoveJAcceleration = 1.0f;

        [Tooltip("正常帧关节速度（rad/s）")]
        [Range(0.1f, 5f)]
        public float normalMoveJVelocity = 0.8f;

        [Tooltip("正常帧MoveJ等待时间（秒）")]
        [Range(0.5f, 10f)]
        public float normalMoveJWaitTime = 2.0f;

        [Tooltip("MoveJ后额外稳定时间（秒），等待机械臂完全静止后再捕捉TCP")]
        [Range(0f, 5f)]
        public float stabilizationTime = 1.0f;

        [Tooltip("视觉伺服收敛后稳定延迟（秒），等待机械臂完全静止后再捕捉TCP和Tracker位姿")]
        [Range(0f, 5f)]
        public float servoConvergeStabilizationTime = 1.0f;

        [Tooltip("TCP采样后延迟（秒），采样完成后等待一段时间再开始下一步移动操作")]
        [Range(0f, 5f)]
        public float postCaptureDelay = 1.0f;

        [Header("位姿偏移配置")]
        [Tooltip("是否启用位姿偏移（沿Tracker Z轴方向平移）")]
        public bool enableTargetOffset = false;

        [Tooltip("沿Tracker Z轴正向的偏移距离 (mm)，正值向前，负值向后")]
        public float zAxisOffsetMm = 150f;

        [Header("视觉伺服配置")]
        [Tooltip("是否执行视觉伺服（关闭则只记录MoveJ后的TCP）")]
        public bool enableVisualServo = true;

        [Tooltip("最大迭代次数（单帧视觉伺服servoj循环上限）")]
        [Range(1, 9000)]
        public int maxServoIterations = 500;

        [Tooltip("位置收敛阈值（米）")]
        [Range(0.0001f, 0.01f)]
        public float positionThreshold = 0.001f;  // 1mm

        [Tooltip("旋转收敛阈值（弧度）")]
        [Range(0.001f, 0.1f)]
        public float rotationThreshold = 0.01745f;  // 1°

        [Tooltip("连续收敛次数要求，误差需连续n次满足阈值才视为真正收敛（防止噪声抖动误判）")]
        [Range(1, 50)]
        public int consecutiveConvergenceCount = 5;

        [Tooltip("视觉伺服超时时间（秒）")]
        [Range(10f, 300f)]
        public float servoTimeout = 60f;

        [Tooltip("Servoj 控制频率（Hz）")]
        [Range(1f, 500f)]
        public float servojFrequency = 45f;

        [Tooltip("视觉伺服补偿循环频率（Hz）")]
        [Range(1f, 200f)]
        public float compensationLoopFrequency = 90f;

        [Header("Servoj 参数")]
        [Tooltip("加速度（rad/s²，0表示无限制）")]
        [Range(0f, 10f)]
        public float servojAcceleration = 0f;

        [Tooltip("速度（rad/s，0表示无限制）")]
        [Range(0f, 5f)]
        public float servojVelocity = 0f;

        [Tooltip("前瞻时间（秒）")]
        [Range(0.01f, 0.5f)]
        public float lookAheadTime = 0.1f;

        [Tooltip("增益 - 必须100-300才能使机器人响应Servoj命令")]
        [Range(10f, 500f)]
        public float gain = 300f;

        [Header("Tracker配置")]
        [Tooltip("Tracker1 设备ID（目标位姿）")]
        public uint tracker1DeviceId = 3;

        [Tooltip("Tracker2 设备ID（TCP末端）")]
        public uint tracker2DeviceId = 4;

        [Tooltip("Tracker2连续无数据容忍次数，超过则中止视觉伺服")]
        [Range(5, 100)]
        public int tracker2DataLossTolerance = 15;

        [Header("补偿限幅（不影响采集中止）")]
        [Tooltip("补偿累积移动距离记录阈值（米），仅用于统计，不会中止采集")]
        [Range(0.1f, 10f)]
        public float maxCumulativeMovement = 5.0f;

        [Tooltip("单次迭代最大补偿量（米），超过则限幅到此值，不会中止采集")]
        [Range(0.001f, 0.1f)]
        public float maxSingleStepMovement = 0.05f;

        [Header("调试选项")]
        [Tooltip("是否输出详细日志")]
        public bool verboseLogging = true;

        [Tooltip("是否在场景中可视化")]
        public bool visualizeProgress = true;

        // ==================== 运行时状态（只读显示）====================

        [Header("运行时状态")]
        [SerializeField]
        private bool isRunning = false;

        [SerializeField]
        private int currentFrameIndex = 0;

        [SerializeField]
        private int totalFrames = 0;

        [SerializeField]
        private int completedFrames = 0;

        [SerializeField]
        private int convergedFrames = 0;

        // ==================== 组件引用 ====================

        private ViveTrackerPoseLogger trackerPoseLogger;
        private main_ui_control urController;

        // ==================== 内部状态 ====================

        private RigidBodyCaptureData trajectoryData;
        private List<int> targetFrameIndices;
        private List<FrameCaptureResult> captureResults;
        private Coroutine captureCoroutine;

        // 视觉伺服状态
        private bool servoCompleted = false;
        private bool servoConverged = false;
        private int servoIterations = 0;
        private PoseError latestServoError;
        private Coroutine servoCoroutine;

        // 补偿循环状态
        private Vector3 currentBaseTarget;
        private Quaternion currentBaseTargetRot;
        private Vector3 compensationStartPosition;
        private float totalCumulativeMovement = 0f;

        // 计时
        private Stopwatch totalStopwatch;
        private Stopwatch frameStopwatch;

        // ==================== 公共事件 ====================

        /// <summary>
        /// 单帧采集完成事件
        /// </summary>
        public event Action<FrameCaptureResult> OnFrameCaptureComplete;

        /// <summary>
        /// 全部采集完成事件
        /// </summary>
        public event Action<List<FrameCaptureResult>> OnAllCaptureComplete;

        /// <summary>
        /// 进度更新事件 (当前帧, 总帧数)
        /// </summary>
        public event Action<int, int> OnProgressUpdate;

        // ==================== 生命周期 ====================

        void Start()
        {
            // 获取组件引用
            trackerPoseLogger = FindObjectOfType<ViveTrackerPoseLogger>();
            if (trackerPoseLogger == null)
            {
                Debug.LogError("[TCP采集] 未找到 ViveTrackerPoseLogger 组件！");
            }

            urController = FindObjectOfType<main_ui_control>();
            if (urController == null)
            {
                Debug.LogError("[TCP采集] 未找到 main_ui_control 组件！");
            }

            totalStopwatch = new Stopwatch();
            frameStopwatch = new Stopwatch();

            Debug.Log("[TCP采集] 控制器初始化完成");
        }

        // ==================== 公共接口 ====================

        /// <summary>
        /// 启动采集流程
        /// </summary>
        [ContextMenu("启动采集")]
        public void StartCapture()
        {
            if (isRunning)
            {
                Debug.LogWarning("[TCP采集] 采集已在运行中，无法重复启动");
                return;
            }

            // 验证组件
            if (trackerPoseLogger == null || urController == null)
            {
                Debug.LogError("[TCP采集] 必要组件未找到，无法启动");
                return;
            }

            // 验证UR连接
            if (!ur_data_processing.UR_Stream_Data.is_alive)
            {
                Debug.LogError("[TCP采集] UR机器人未连接，无法启动");
                return;
            }

            // 加载轨迹数据
            if (!LoadTrajectoryData())
            {
                Debug.LogError("[TCP采集] 加载轨迹数据失败，无法启动");
                return;
            }

            // 生成输出路径
            if (string.IsNullOrEmpty(outputCsvPath))
            {
                outputCsvPath = GenerateOutputPath();
            }

            Debug.Log($"[TCP采集] 启动采集流程");
            Debug.Log($"  输入文件: {csvFilePath}");
            Debug.Log($"  输出文件: {outputCsvPath}");
            Debug.Log($"  目标帧数: {targetFrameIndices.Count}");
            Debug.Log($"  视觉伺服: {(enableVisualServo ? "启用" : "禁用")}");
            Debug.Log($"  位姿偏移: {(enableTargetOffset ? $"启用 ({zAxisOffsetMm}mm)" : "禁用")}");

            // 初始化状态
            isRunning = true;
            completedFrames = 0;
            convergedFrames = 0;
            captureResults = new List<FrameCaptureResult>();
            totalStopwatch.Restart();

            // 启动采集协程
            captureCoroutine = StartCoroutine(CaptureCoroutine());
        }

        /// <summary>
        /// 停止采集流程
        /// </summary>
        [ContextMenu("停止采集")]
        public void StopCapture()
        {
            if (!isRunning)
            {
                Debug.LogWarning("[TCP采集] 采集流程未在运行");
                return;
            }

            Debug.Log("[TCP采集] 手动停止采集流程");

            // 停止协程
            if (captureCoroutine != null)
            {
                StopCoroutine(captureCoroutine);
                captureCoroutine = null;
            }

            if (servoCoroutine != null)
            {
                StopCoroutine(servoCoroutine);
                servoCoroutine = null;
            }

            // 停止UR发送
            ur_data_processing.UR_Control_Data.manual_send_active = false;

            isRunning = false;

            // 保存已完成的结果
            if (captureResults != null && captureResults.Count > 0)
            {
                Debug.Log($"[TCP采集] 保存已完成的 {captureResults.Count} 帧结果");
                SaveResultsToCSV();
            }
        }

        /// <summary>
        /// 预览轨迹信息（不执行采集）
        /// </summary>
        [ContextMenu("预览轨迹")]
        public void PreviewTrajectory()
        {
            if (!LoadTrajectoryData())
            {
                return;
            }

            Debug.Log("========== 轨迹预览 ==========");
            Debug.Log($"  文件: {csvFilePath}");
            Debug.Log($"  总帧数: {trajectoryData.FrameData.Count}");
            Debug.Log($"  选中帧数: {targetFrameIndices.Count}");
            Debug.Log($"  帧范围: {startFrameIndex} ~ {(endFrameIndex < 0 ? trajectoryData.FrameData.Count - 1 : endFrameIndex)}");
            Debug.Log($"  采样间隔: {frameSamplingInterval}");

            // 显示前几帧位置
            int previewCount = Mathf.Min(3, targetFrameIndices.Count);
            Debug.Log("  前几帧位置 (SteamVR坐标系):");
            for (int i = 0; i < previewCount; i++)
            {
                int idx = targetFrameIndices[i];
                var frame = trajectoryData.FrameData[idx];
                Debug.Log($"    帧{i + 1} (数据帧{idx}): ({frame.Position.X:F1}, {frame.Position.Y:F1}, {frame.Position.Z:F1}) mm");
            }

            // 计算轨迹总长度
            float totalLength = 0f;
            for (int i = 1; i < targetFrameIndices.Count; i++)
            {
                var prevFrame = trajectoryData.FrameData[targetFrameIndices[i - 1]];
                var currFrame = trajectoryData.FrameData[targetFrameIndices[i]];

                Vector3 prevPos = new Vector3((float)prevFrame.Position.X, (float)prevFrame.Position.Y, (float)prevFrame.Position.Z);
                Vector3 currPos = new Vector3((float)currFrame.Position.X, (float)currFrame.Position.Y, (float)currFrame.Position.Z);

                totalLength += Vector3.Distance(prevPos, currPos);
            }
            Debug.Log($"  轨迹总长度: {totalLength:F1} mm");

            // 估算时间
            float estimatedTime = targetFrameIndices.Count * (enableVisualServo ? 10f : 3f);
            Debug.Log($"  预计耗时: {estimatedTime / 60f:F1} 分钟");

            // 清理临时数据
            trajectoryData = null;
            targetFrameIndices = null;
        }

        /// <summary>
        /// 获取当前运行状态
        /// </summary>
        public bool IsRunning => isRunning;

        /// <summary>
        /// 获取当前进度
        /// </summary>
        public (int completed, int total) GetProgress()
        {
            return (completedFrames, totalFrames);
        }

        // ==================== 核心采集流程 ====================

        private IEnumerator CaptureCoroutine()
        {
            Debug.Log("[TCP采集] 开始采集协程...");

            // 遍历每一帧
            for (int i = 0; i < targetFrameIndices.Count; i++)
            {
                if (!isRunning) yield break;

                int frameIdx = targetFrameIndices[i];
                currentFrameIndex = frameIdx;
                FrameData targetFrame = trajectoryData.FrameData[frameIdx];
                frameStopwatch.Restart();

                // 创建帧结果记录
                FrameCaptureResult result = new FrameCaptureResult();
                result.FrameIndex = frameIdx;
                result.TargetTrackerPose = targetFrame;

                LogInfo($"========== 处理帧 {i + 1}/{targetFrameIndices.Count} (数据帧{frameIdx}) ==========");

                // 4.2 坐标转换: Tracker位姿(SteamVR) → TCP目标(UR基座系)
                Vector3 trackerPos = new Vector3(
                    (float)(targetFrame.Position.X / 1000.0),  // mm → m
                    (float)(targetFrame.Position.Y / 1000.0),
                    (float)(targetFrame.Position.Z / 1000.0)
                );
                Quaternion trackerRot = targetFrame.GetQuaternion();

                // 应用位姿偏移（如果启用）
                Vector3 adjustedTrackerPos = trackerPos;
                if (enableTargetOffset)
                {
                    float offsetM = zAxisOffsetMm / 1000f;
                    adjustedTrackerPos = ApplyPositionOffset(trackerPos, trackerRot, offsetM);
                    LogInfo($"  位姿偏移: {zAxisOffsetMm:F1}mm, 调整后位置: ({adjustedTrackerPos.x:F4}, {adjustedTrackerPos.y:F4}, {adjustedTrackerPos.z:F4}) m");
                }

                // 转换到UR基座坐标系
                SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                    adjustedTrackerPos, trackerRot, false,
                    out Vector3 baseTargetPos, out Vector3 baseTargetRotVec);

                LogInfo($"  目标位姿(基座系): Pos=({baseTargetPos.x:F4}, {baseTargetPos.y:F4}, {baseTargetPos.z:F4}) m");

                // 4.3 发送MoveJ命令
                bool isFirstFrame = (i == 0);
                SendMoveJCommand(baseTargetPos, baseTargetRotVec, isFirstFrame);

                // 4.4 等待MoveJ完成
                float waitTime = isFirstFrame ? firstFrameWaitTime : normalMoveJWaitTime;
                LogInfo($"  等待MoveJ完成: {waitTime:F1} 秒 {(isFirstFrame ? "(第一帧慢速)" : "")}");
                yield return new WaitForSeconds(waitTime);

                // 4.5 MoveJ后稳定延迟，等待机械臂完全静止
                if (stabilizationTime > 0)
                {
                    LogInfo($"  MoveJ后稳定延迟: {stabilizationTime:F1} 秒");
                    yield return new WaitForSeconds(stabilizationTime);
                }

                // 4.6 捕捉MoveJ后的TCP位姿
                result.MovejTcpPose = CaptureTcpPose();
                if (result.MovejTcpPose.IsValid)
                {
                    LogInfo($"  MoveJ后TCP: {result.MovejTcpPose}");
                }
                else
                {
                    LogWarning($"  MoveJ后TCP捕捉失败！");
                    result.ErrorMessage = "MoveJ后TCP捕捉失败";
                }

                // 4.6.1 MoveJ采样后延迟，确保采样与移动不同时进行
                if (postCaptureDelay > 0 && enableVisualServo)
                {
                    LogInfo($"  MoveJ采样后延迟: {postCaptureDelay:F1} 秒");
                    yield return new WaitForSeconds(postCaptureDelay);
                }

                // 4.7 执行视觉伺服（如果启用）- 必须收敛才能继续下一帧
                if (enableVisualServo)
                {
                    LogInfo($"  启动视觉伺服 (最大{maxServoIterations}次迭代)...");

                    // 执行视觉伺服（单次servoj循环，最多maxServoIterations次迭代）
                    yield return StartCoroutine(ExecuteVisualServoCoroutine(trackerPos, trackerRot));

                    // 记录伺服结果
                    result.Converged = servoConverged;
                    result.Iterations = servoIterations;
                    result.RetryCount = 0;  // 不再使用重试机制

                    if (!servoConverged)
                    {
                        LogError($"  ⚠ 帧{frameIdx}视觉伺服未收敛！已迭代{servoIterations}次");
                        LogError($"  ❌ 采集流程中止：视觉伺服无法收敛");
                        result.ErrorMessage = $"视觉伺服未收敛(迭代{servoIterations}次)，采集中止";
                        
                        // 记录当前帧结果
                        result.ProcessingTime = (float)frameStopwatch.Elapsed.TotalSeconds;
                        captureResults.Add(result);
                        completedFrames++;
                        
                        // 保存已完成的结果
                        SaveResultsToCSV();
                        OutputStatistics();
                        
                        isRunning = false;
                        OnAllCaptureComplete?.Invoke(captureResults);
                        
                        Debug.LogError("[TCP采集] 采集流程因视觉伺服未收敛而中止！");
                        yield break;  // 停止整个采集流程
                    }
                    else
                    {
                        LogInfo($"  ✓ 视觉伺服收敛！迭代{servoIterations}次");
                        convergedFrames++;
                        
                        // 视觉伺服收敛后稳定延迟
                        if (servoConvergeStabilizationTime > 0)
                        {
                            LogInfo($"  收敛后稳定延迟: {servoConvergeStabilizationTime:F1} 秒");
                            yield return new WaitForSeconds(servoConvergeStabilizationTime);
                        }
                    }
                }
                else
                {
                    result.Converged = true;  // 不启用伺服时视为成功
                    result.Iterations = 0;
                    result.RetryCount = 0;
                }

                // 4.8 捕捉视觉伺服后的TCP位姿
                result.ServoTcpPose = CaptureTcpPose();
                if (result.ServoTcpPose.IsValid)
                {
                    LogInfo($"  伺服后TCP: {result.ServoTcpPose}");
                }
                else
                {
                    LogWarning($"  伺服后TCP捕捉失败！");
                    if (string.IsNullOrEmpty(result.ErrorMessage))
                        result.ErrorMessage = "伺服后TCP捕捉失败";
                }

                // 4.8.1 伺服采样后延迟，确保采样与下一帧MoveJ不同时进行
                if (postCaptureDelay > 0 && i < targetFrameIndices.Count - 1)
                {
                    LogInfo($"  伺服采样后延迟: {postCaptureDelay:F1} 秒");
                    yield return new WaitForSeconds(postCaptureDelay);
                }

                // 4.9 捕捉收敛后的Tracker2位姿（用于验证稳定性）
                if (enableVisualServo && result.Converged)
                {
                    if (GetTracker2Pose(out Vector3 convergedTrackerPos, out Quaternion convergedTrackerRot))
                    {
                        result.ConvergedTracker2Position = convergedTrackerPos;
                        result.ConvergedTracker2Rotation = convergedTrackerRot;
                        result.HasConvergedTrackerPose = true;
                        LogInfo($"  收敛后Tracker2: Pos=({convergedTrackerPos.x:F4}, {convergedTrackerPos.y:F4}, {convergedTrackerPos.z:F4})m");
                        
                        // 计算收敛后Tracker2与目标Tracker位姿的差异（最终误差）
                        result.FinalTrackerPosError = Vector3.Distance(convergedTrackerPos, trackerPos) * 1000f;  // m → mm
                        result.FinalTrackerRotError = Quaternion.Angle(convergedTrackerRot, trackerRot);  // 度
                        LogInfo($"  ★ 最终Tracker误差: 位置={result.FinalTrackerPosError:F3}mm, 旋转={result.FinalTrackerRotError:F3}°");
                    }
                    else
                    {
                        LogWarning($"  收敛后Tracker2位姿捕捉失败！");
                        result.HasConvergedTrackerPose = false;
                    }
                }

                // 4.10 计算MoveJ TCP与Servo TCP之间的位姿差异
                if (result.MovejTcpPose.IsValid && result.ServoTcpPose.IsValid)
                {
                    // 位置差异 (米 → 毫米)
                    Vector3 movejPos = new Vector3((float)result.MovejTcpPose.X, (float)result.MovejTcpPose.Y, (float)result.MovejTcpPose.Z);
                    Vector3 servoPos = new Vector3((float)result.ServoTcpPose.X, (float)result.ServoTcpPose.Y, (float)result.ServoTcpPose.Z);
                    result.TcpPosDiff = Vector3.Distance(movejPos, servoPos) * 1000f;  // m → mm

                    // 旋转差异 (弧度 → 度)
                    Vector3 movejRotVec = new Vector3((float)result.MovejTcpPose.RX, (float)result.MovejTcpPose.RY, (float)result.MovejTcpPose.RZ);
                    Vector3 servoRotVec = new Vector3((float)result.ServoTcpPose.RX, (float)result.ServoTcpPose.RY, (float)result.ServoTcpPose.RZ);
                    Quaternion movejRot = PoseErrorCalculator.RotationVectorToQuaternion(movejRotVec);
                    Quaternion servoRot = PoseErrorCalculator.RotationVectorToQuaternion(servoRotVec);
                    float angleDiff = Quaternion.Angle(movejRot, servoRot);  // 返回度
                    result.TcpRotDiff = angleDiff;

                    LogInfo($"  TCP位姿差异(MoveJ vs Servo): 位置={result.TcpPosDiff:F3}mm, 旋转={result.TcpRotDiff:F3}°");
                }
                else
                {
                    result.TcpPosDiff = 0;
                    result.TcpRotDiff = 0;
                }

                // 4.11 记录处理时间
                result.ProcessingTime = (float)frameStopwatch.Elapsed.TotalSeconds;

                // 4.12 添加到结果列表
                captureResults.Add(result);
                completedFrames++;

                // 4.13 触发事件
                OnFrameCaptureComplete?.Invoke(result);
                OnProgressUpdate?.Invoke(completedFrames, totalFrames);

                LogInfo($"  帧处理完成，耗时: {result.ProcessingTime:F2} 秒");
            }

            // 5. 保存结果到CSV
            SaveResultsToCSV();

            // 6. 输出统计信息
            OutputStatistics();

            isRunning = false;

            // 7. 触发完成事件
            OnAllCaptureComplete?.Invoke(captureResults);

            Debug.Log("[TCP采集] 采集流程完成！");
        }

        // ==================== 视觉伺服核心逻辑 ====================

        /// <summary>
        /// 执行视觉伺服补偿循环
        /// 
        /// 控制逻辑：增量补偿模式（参考 VisualServoCompensationController）
        /// - 每次迭代读取实时TCP位姿
        /// - 计算相机坐标系下的位姿误差
        /// - 将误差转换到基座坐标系，施加到实时TCP位姿上
        /// - 发送修正后的Servoj命令
        /// - 直到Tracker2实际位姿与目标位姿的误差小于阈值（收敛）
        /// </summary>
        private IEnumerator ExecuteVisualServoCoroutine(Vector3 targetTrackerPos, Quaternion targetTrackerRot)
        {
            servoCompleted = false;
            servoConverged = false;
            servoIterations = 0;
            latestServoError = null;

            // 输出视觉伺服配置信息
            LogInfo($"  [视觉伺服配置] Tracker2 DeviceID={tracker2DeviceId} (TCP末端Tracker)");
            LogInfo($"  [视觉伺服目标] CSV位姿(相机系): Pos=({targetTrackerPos.x:F4}, {targetTrackerPos.y:F4}, {targetTrackerPos.z:F4})m");

            // 设置FixedUpdate频率
            float originalFixedDeltaTime = Time.fixedDeltaTime;
            Time.fixedDeltaTime = 1f / compensationLoopFrequency;

            // 获取当前Tracker2位姿作为起始位置
            LogInfo($"  [视觉伺服] 尝试获取Tracker2(ID={tracker2DeviceId})位姿...");
            if (!GetTracker2Pose(out Vector3 tracker2Pos, out Quaternion tracker2Rot))
            {
                LogError($"  ❌ 无法获取Tracker2位姿！请检查:");
                LogError($"     1. Tracker2 DeviceID={tracker2DeviceId} 是否正确?");
                LogError($"     2. 该Tracker是否已连接并被SteamVR识别?");
                LogError($"     3. ViveTrackerPoseLogger组件是否正常工作?");
                servoCompleted = true;
                Time.fixedDeltaTime = originalFixedDeltaTime;  // 恢复频率
                yield break;
            }
            LogInfo($"  ✓ Tracker2初始位姿: Pos=({tracker2Pos.x:F4}, {tracker2Pos.y:F4}, {tracker2Pos.z:F4})m");

            // 记录起始位置用于统计
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                tracker2Pos, tracker2Rot, false,
                out compensationStartPosition, out Vector3 _);
            totalCumulativeMovement = 0f;

            // 激活Servoj发送
            ur_data_processing.UR_Control_Data.manual_send_active = true;

            // 超时计时
            Stopwatch servoStopwatch = Stopwatch.StartNew();
            int consecutiveFailures = 0;
            int convergenceCounter = 0;  // 连续收敛计数器

            // 视觉伺服循环 - 增量补偿模式
            while (servoIterations < maxServoIterations && !servoConverged)
            {
                // 检查超时
                if (servoStopwatch.Elapsed.TotalSeconds > servoTimeout)
                {
                    LogWarning($"  视觉伺服超时 ({servoTimeout}s)");
                    break;
                }

                // ============ 步骤1: 获取Tracker2实时位姿 ============
                if (!GetTracker2Pose(out tracker2Pos, out tracker2Rot))
                {
                    consecutiveFailures++;
                    if (consecutiveFailures > tracker2DataLossTolerance)
                    {
                        LogError($"  Tracker2持续无数据({consecutiveFailures}次)，中止视觉伺服");
                        break;
                    }
                    yield return new WaitForFixedUpdate();
                    continue;
                }
                consecutiveFailures = 0;

                // ============ 步骤2: 计算位姿误差（相机坐标系）============
                PoseError error = PoseErrorCalculator.CalculateError(
                    targetTrackerPos, targetTrackerRot,
                    tracker2Pos, tracker2Rot);
                latestServoError = error;

                // 每100次迭代输出一次误差信息
                if (servoIterations % 100 == 0)
                {
                    LogInfo($"  [迭代{servoIterations}] 位置误差={error.positionMagnitude * 1000f:F2}mm, 旋转误差={error.rotationMagnitude * Mathf.Rad2Deg:F2}°, 连续收敛={convergenceCounter}/{consecutiveConvergenceCount}");
                }

                // ============ 步骤3: 检查是否收敛（需连续满足n次）============
                bool currentlyConverged = error.positionMagnitude < positionThreshold && error.rotationMagnitude < rotationThreshold;
                
                if (currentlyConverged)
                {
                    convergenceCounter++;
                    if (convergenceCounter >= consecutiveConvergenceCount)
                    {
                        servoConverged = true;
                        LogInfo($"  ✓ 视觉伺服收敛！迭代{servoIterations}次，连续{convergenceCounter}次满足阈值");
                        LogInfo($"    最终误差: 位置={error.positionMagnitude * 1000f:F3}mm, 旋转={error.rotationMagnitude * Mathf.Rad2Deg:F3}°");
                        break;
                    }
                }
                else
                {
                    // 误差超出阈值，重置计数器
                    if (convergenceCounter > 0)
                    {
                        LogInfo($"  [迭代{servoIterations}] 收敛中断，重置计数器 (位置={error.positionMagnitude * 1000f:F2}mm, 旋转={error.rotationMagnitude * Mathf.Rad2Deg:F2}°)");
                    }
                    convergenceCounter = 0;
                }

                // ============ 步骤4: 将误差转换到基座坐标系 ============
                Quaternion errorRotQuat = PoseErrorCalculator.RotationVectorToQuaternion(error.rotationError);
                SteamVrUrCoordinateConverter.ConvertErrorVectorToUrBase(
                    error.positionError,        // 输入：相机系位置误差 (m)
                    errorRotQuat,               // 输入：相机系旋转误差 (四元数)
                    out Vector3 basePosError,   // 输出：基座系位置误差 (m)
                    out Vector3 baseRotError);  // 输出：基座系旋转误差 (旋转矢量 rad)

                // ============ 步骤5: 单步补偿量限制 ============
                if (basePosError.magnitude > maxSingleStepMovement)
                {
                    basePosError = basePosError.normalized * maxSingleStepMovement;
                }

                // 单步旋转补偿量限制（最大30度）
                float rotMagnitude = baseRotError.magnitude;
                float rotDegrees = rotMagnitude * Mathf.Rad2Deg;
                const float MAX_ROTATION_STEP = 30f;
                if (rotDegrees > MAX_ROTATION_STEP)
                {
                    baseRotError = baseRotError.normalized * (MAX_ROTATION_STEP * Mathf.Deg2Rad);
                }

                // ============ 步骤6: 读取实时TCP位姿并施加补偿 ============
                Vector3 currentTcpBase;
                Quaternion currentTcpRotBase;
                
                if (!GetRobotTcpPose(out Vector3 currentTcpMm, out Vector3 currentTcpRotRad))
                {
                    LogWarning($"  [迭代{servoIterations}] 无法读取实时TCP位姿，使用上次目标");
                    currentTcpBase = currentBaseTarget;
                    currentTcpRotBase = currentBaseTargetRot;
                }
                else
                {
                    currentTcpBase = currentTcpMm / 1000f;  // mm → m
                    currentTcpRotBase = PoseErrorCalculator.RotationVectorToQuaternion(currentTcpRotRad);
                }

                // ✅ 将补偿量施加到实时TCP位姿上（核心逻辑！）
                currentBaseTarget = currentTcpBase + basePosError;

                // ✅ 旋转误差合成 - 基于当前实际旋转
                Quaternion baseRotErrorQuat = PoseErrorCalculator.RotationVectorToQuaternion(baseRotError);
                currentBaseTargetRot = baseRotErrorQuat * currentTcpRotBase;

                // 记录累积移动量（仅统计）
                totalCumulativeMovement += basePosError.magnitude;

                // ============ 步骤7: 发送修正后的Servoj命令 ============
                SendServojCommand(currentBaseTarget, currentBaseTargetRot);

                servoIterations++;

                // 等待下一次循环
                yield return new WaitForFixedUpdate();
            }

            // 停止Servoj发送
            ur_data_processing.UR_Control_Data.manual_send_active = false;

            // 恢复FixedUpdate频率
            Time.fixedDeltaTime = originalFixedDeltaTime;

            servoCompleted = true;
        }

        /// <summary>
        /// 获取机器人当前TCP位姿（基座坐标系，毫米和弧度）
        /// </summary>
        private bool GetRobotTcpPose(out Vector3 positionMm, out Vector3 rotationRad)
        {
            positionMm = Vector3.zero;
            rotationRad = Vector3.zero;

            if (!ur_data_processing.UR_Stream_Data.is_alive)
            {
                return false;
            }

            var tcpPos = ur_data_processing.UR_Stream_Data.C_Position;
            var tcpRot = ur_data_processing.UR_Stream_Data.C_Orientation;

            if (tcpPos == null || tcpPos.Length < 3 || tcpRot == null || tcpRot.Length < 3)
            {
                return false;
            }

            // C_Position 单位是米，转换为毫米
            positionMm = new Vector3(
                (float)(tcpPos[0] * 1000.0),
                (float)(tcpPos[1] * 1000.0),
                (float)(tcpPos[2] * 1000.0)
            );

            // C_Orientation 单位是弧度
            rotationRad = new Vector3(
                (float)tcpRot[0],
                (float)tcpRot[1],
                (float)tcpRot[2]
            );

            return true;
        }

        // ==================== 辅助方法 ====================

        /// <summary>
        /// 加载轨迹数据
        /// </summary>
        private bool LoadTrajectoryData()
        {
            if (string.IsNullOrEmpty(csvFilePath))
            {
                Debug.LogError("[TCP采集] CSV文件路径为空");
                return false;
            }

            trajectoryData = CSVCaptureReader.LoadFromCSV(csvFilePath);
            if (trajectoryData == null || trajectoryData.FrameData == null || trajectoryData.FrameData.Count == 0)
            {
                Debug.LogError($"[TCP采集] 加载CSV失败或数据为空: {csvFilePath}");
                return false;
            }

            // 筛选目标帧
            targetFrameIndices = new List<int>();
            int maxFrame = trajectoryData.FrameData.Count - 1;
            int actualEndFrame = endFrameIndex < 0 ? maxFrame : Mathf.Min(endFrameIndex, maxFrame);

            for (int i = startFrameIndex; i <= actualEndFrame; i += frameSamplingInterval)
            {
                targetFrameIndices.Add(i);
            }

            totalFrames = targetFrameIndices.Count;

            if (totalFrames == 0)
            {
                Debug.LogError("[TCP采集] 筛选后没有目标帧");
                return false;
            }

            Debug.Log($"[TCP采集] 已加载轨迹: {trajectoryData.FrameData.Count}帧, 筛选后{totalFrames}帧");
            return true;
        }

        /// <summary>
        /// 发送MoveJ命令
        /// </summary>
        private void SendMoveJCommand(Vector3 position, Vector3 rotationVector, bool isFirstFrame)
        {
            float acceleration = isFirstFrame ? firstFrameAcceleration : normalMoveJAcceleration;
            float velocity = isFirstFrame ? firstFrameVelocity : normalMoveJVelocity;

            // 验证关节数据可用
            double[] currentJoints = ur_data_processing.UR_Stream_Data.J_Orientation;
            if (currentJoints == null || currentJoints.Length < 6)
            {
                Debug.LogError("[TCP采集] 无法获取当前关节角度");
                return;
            }

            // 发送命令 - 使用 BuildAndSetMoveL（内部会构建movej命令）
            // t=0 表示使用速度和加速度控制，r=0 表示无混合半径
            urController.BuildAndSetMoveL(
                position.x, position.y, position.z,
                rotationVector.x, rotationVector.y, rotationVector.z,
                acceleration, velocity, 0, 0);
            ur_data_processing.UR_Control_Data.manual_send_active = true;

            // 等待一帧后关闭（MoveJ只需发送一次）
            StartCoroutine(DisableManualSendAfterFrame());

            LogInfo($"  MoveJ命令: a={acceleration}, v={velocity}");
        }

        private IEnumerator DisableManualSendAfterFrame()
        {
            yield return null;
            ur_data_processing.UR_Control_Data.manual_send_active = false;
        }

        /// <summary>
        /// 发送Servoj命令
        /// </summary>
        private void SendServojCommand(Vector3 position, Quaternion rotation)
        {
            // 四元数转旋转矢量
            Vector3 rotVec = QuaternionToRotationVector(rotation);

            // 验证关节数据可用
            double[] currentJoints = ur_data_processing.UR_Stream_Data.J_Orientation;
            if (currentJoints == null || currentJoints.Length < 6) return;

            float timeStep = 1f / servojFrequency;

            // 使用 BuildAndSetServoj 发送命令
            urController.BuildAndSetServoj(
                position.x, position.y, position.z,
                rotVec.x, rotVec.y, rotVec.z,
                servojAcceleration, servojVelocity, timeStep, lookAheadTime, gain);
        }

        /// <summary>
        /// 捕捉当前TCP位姿
        /// </summary>
        private TcpPose CaptureTcpPose()
        {
            TcpPose pose = new TcpPose();
            pose.Timestamp = DateTime.Now;

            if (!ur_data_processing.UR_Stream_Data.is_alive)
            {
                pose.IsValid = false;
                return pose;
            }

            var tcpPos = ur_data_processing.UR_Stream_Data.C_Position;
            var tcpRot = ur_data_processing.UR_Stream_Data.C_Orientation;

            if (tcpPos != null && tcpPos.Length >= 3 && tcpRot != null && tcpRot.Length >= 3)
            {
                pose.X = tcpPos[0];
                pose.Y = tcpPos[1];
                pose.Z = tcpPos[2];
                pose.RX = tcpRot[0];
                pose.RY = tcpRot[1];
                pose.RZ = tcpRot[2];
                pose.IsValid = true;
            }

            return pose;
        }

        /// <summary>
        /// 获取Tracker2位姿（返回米和四元数）
        /// </summary>
        private bool GetTracker2Pose(out Vector3 position, out Quaternion rotation)
        {
            position = Vector3.zero;
            rotation = Quaternion.identity;

            if (trackerPoseLogger == null)
            {
                if (verboseLogging) Debug.LogWarning("[TCP采集] GetTracker2Pose失败: trackerPoseLogger为null");
                return false;
            }

            // GetTrackerPoseForCalibration 返回的位置是毫米，需要转换为米
            if (!trackerPoseLogger.GetTrackerPoseForCalibration(tracker2DeviceId, out Vector3 positionMm, out rotation))
            {
                // 注意：GetTrackerPoseForCalibration内部会输出详细的失败原因
                return false;
            }

            // 毫米转米
            position = positionMm / 1000f;
            return true;
        }

        /// <summary>
        /// 应用位置偏移（沿Tracker Z轴方向）
        /// </summary>
        private Vector3 ApplyPositionOffset(Vector3 position, Quaternion rotation, float offsetMeters)
        {
            // Tracker的Z轴方向
            Vector3 zAxis = rotation * Vector3.forward;
            return position + zAxis * offsetMeters;
        }

        /// <summary>
        /// 四元数转旋转矢量
        /// </summary>
        private Vector3 QuaternionToRotationVector(Quaternion q)
        {
            q.ToAngleAxis(out float angle, out Vector3 axis);
            float angleRad = angle * Mathf.Deg2Rad;
            return axis * angleRad;
        }

        /// <summary>
        /// 生成输出文件路径
        /// </summary>
        private string GenerateOutputPath()
        {
            string dir = Path.GetDirectoryName(csvFilePath);
            string fileName = Path.GetFileNameWithoutExtension(csvFilePath);
            string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            return Path.Combine(dir, $"{fileName}_servo_capture_{timestamp}.csv");
        }

        /// <summary>
        /// 保存结果到CSV
        /// </summary>
        private void SaveResultsToCSV()
        {
            if (captureResults == null || captureResults.Count == 0)
            {
                Debug.LogWarning("[TCP采集] 没有结果可保存");
                return;
            }

            try
            {
                StringBuilder sb = new StringBuilder();

                // 写入文件头注释
                sb.AppendLine("# CSV Trajectory Visual Servo TCP Capture Result");
                sb.AppendLine($"# Generated: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
                sb.AppendLine($"# Input File: {csvFilePath}");
                sb.AppendLine($"# Total Frames: {captureResults.Count}, Converged: {convergedFrames}, Success Rate: {(captureResults.Count > 0 ? (float)convergedFrames / captureResults.Count * 100f : 0):F1}%");
                sb.AppendLine($"# First Frame: Slow MoveJ (a={firstFrameAcceleration}, v={firstFrameVelocity}, wait={firstFrameWaitTime}s)");
                sb.AppendLine($"# Normal Frame: MoveJ (a={normalMoveJAcceleration}, v={normalMoveJVelocity}, wait={normalMoveJWaitTime}s)");
                sb.AppendLine($"# Visual Servo: {(enableVisualServo ? $"Enabled (MaxIter={maxServoIterations}, PosThresh={positionThreshold * 1000f:F1}mm, RotThresh={rotationThreshold * Mathf.Rad2Deg:F1}deg, ConsecConv={consecutiveConvergenceCount})" : "Disabled")}");
                sb.AppendLine($"# Stabilization: MoveJ后={stabilizationTime:F1}s, 收敛后={servoConvergeStabilizationTime:F1}s");
                sb.AppendLine($"# Target Offset: {(enableTargetOffset ? $"Enabled ({zAxisOffsetMm}mm along Z)" : "Disabled")}");
                sb.AppendLine("#");

                // 写入表头（包含TCP位姿差异和最终Tracker误差）
                sb.AppendLine("FrameIndex,Target_X_mm,Target_Y_mm,Target_Z_mm,Target_QX,Target_QY,Target_QZ,Target_QW," +
                    "Movej_TCP_X_m,Movej_TCP_Y_m,Movej_TCP_Z_m,Movej_TCP_RX_rad,Movej_TCP_RY_rad,Movej_TCP_RZ_rad," +
                    "Servo_TCP_X_m,Servo_TCP_Y_m,Servo_TCP_Z_m,Servo_TCP_RX_rad,Servo_TCP_RY_rad,Servo_TCP_RZ_rad," +
                    "Converged_Tracker2_X_m,Converged_Tracker2_Y_m,Converged_Tracker2_Z_m,Converged_Tracker2_QX,Converged_Tracker2_QY,Converged_Tracker2_QZ,Converged_Tracker2_QW," +
                    "Converged,Iterations,RetryCount,TCP_Pos_Diff_mm,TCP_Rot_Diff_deg,Final_Tracker_Pos_mm,Final_Tracker_Rot_deg,Time_s,Error");

                // 写入数据行
                foreach (var result in captureResults)
                {
                    var target = result.TargetTrackerPose;
                    var movej = result.MovejTcpPose;
                    var servo = result.ServoTcpPose;
                    Quaternion targetQ = target.GetQuaternion();
                    
                    // 收敛后Tracker2位姿
                    Vector3 convT2Pos = result.HasConvergedTrackerPose ? result.ConvergedTracker2Position : Vector3.zero;
                    Quaternion convT2Rot = result.HasConvergedTrackerPose ? result.ConvergedTracker2Rotation : Quaternion.identity;

                    sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
                        "{0},{1:F3},{2:F3},{3:F3},{4:F6},{5:F6},{6:F6},{7:F6}," +
                        "{8:F6},{9:F6},{10:F6},{11:F6},{12:F6},{13:F6}," +
                        "{14:F6},{15:F6},{16:F6},{17:F6},{18:F6},{19:F6}," +
                        "{20:F6},{21:F6},{22:F6},{23:F6},{24:F6},{25:F6},{26:F6}," +
                        "{27},{28},{29},{30:F3},{31:F3},{32:F3},{33:F3},{34:F2},{35}",
                        result.FrameIndex,
                        target.Position.X, target.Position.Y, target.Position.Z,
                        targetQ.x, targetQ.y, targetQ.z, targetQ.w,
                        movej.X, movej.Y, movej.Z, movej.RX, movej.RY, movej.RZ,
                        servo.X, servo.Y, servo.Z, servo.RX, servo.RY, servo.RZ,
                        convT2Pos.x, convT2Pos.y, convT2Pos.z, convT2Rot.x, convT2Rot.y, convT2Rot.z, convT2Rot.w,
                        result.Converged, result.Iterations, result.RetryCount,
                        result.TcpPosDiff, result.TcpRotDiff,
                        result.FinalTrackerPosError, result.FinalTrackerRotError,
                        result.ProcessingTime,
                        result.ErrorMessage.Replace(",", ";")  // 避免逗号干扰CSV
                    ));
                }

                // 确保输出目录存在
                string outputDir = Path.GetDirectoryName(outputCsvPath);
                if (!string.IsNullOrEmpty(outputDir) && !Directory.Exists(outputDir))
                {
                    Directory.CreateDirectory(outputDir);
                }

                File.WriteAllText(outputCsvPath, sb.ToString(), Encoding.UTF8);
                Debug.Log($"[TCP采集] 结果已保存: {outputCsvPath}");
            }
            catch (Exception ex)
            {
                Debug.LogError($"[TCP采集] 保存CSV失败: {ex.Message}");
            }
        }

        /// <summary>
        /// 输出统计信息
        /// </summary>
        private void OutputStatistics()
        {
            float totalTime = (float)totalStopwatch.Elapsed.TotalSeconds;
            float successRate = totalFrames > 0 ? (float)convergedFrames / totalFrames * 100f : 0f;

            Debug.Log("========== TCP采集统计 ==========");
            Debug.Log($"  总帧数: {totalFrames}");
            Debug.Log($"  完成帧: {completedFrames}");
            Debug.Log($"  收敛帧: {convergedFrames} ({successRate:F1}%)");
            Debug.Log($"  总耗时: {totalTime:F1} 秒 ({totalTime / 60f:F1} 分钟)");

            if (completedFrames > 0)
            {
                float avgTimePerFrame = totalTime / completedFrames;
                Debug.Log($"  平均每帧: {avgTimePerFrame:F2} 秒");

                // 计算平均误差
                float avgTrackerPosError = 0f, avgTrackerRotError = 0f;
                float avgTcpPosDiff = 0f, avgTcpRotDiff = 0f;
                int validCount = 0;
                foreach (var result in captureResults)
                {
                    if (result.Converged && result.HasConvergedTrackerPose)
                    {
                        avgTrackerPosError += result.FinalTrackerPosError;
                        avgTrackerRotError += result.FinalTrackerRotError;
                        avgTcpPosDiff += result.TcpPosDiff;
                        avgTcpRotDiff += result.TcpRotDiff;
                        validCount++;
                    }
                }
                if (validCount > 0)
                {
                    Debug.Log($"  ★ 最终Tracker误差: 平均位置={avgTrackerPosError / validCount:F2}mm, 平均旋转={avgTrackerRotError / validCount:F2}°");
                    Debug.Log($"    TCP位姿差异(MoveJ vs Servo): 平均位置={avgTcpPosDiff / validCount:F2}mm, 平均旋转={avgTcpRotDiff / validCount:F2}°");
                }
            }

            Debug.Log($"  输出文件: {outputCsvPath}");
        }

        // ==================== 日志辅助 ====================

        private void LogInfo(string message)
        {
            if (verboseLogging)
            {
                Debug.Log($"[TCP采集] {message}");
            }
        }

        private void LogWarning(string message)
        {
            Debug.LogWarning($"[TCP采集] {message}");
        }

        private void LogError(string message)
        {
            Debug.LogError($"[TCP采集] {message}");
        }

        // ==================== Editor辅助 ====================

#if UNITY_EDITOR
        private void OnValidate()
        {
            // 参数验证
            if (firstFrameWaitTime < normalMoveJWaitTime)
            {
                // 第一帧等待时间通常应该更长
            }
        }
#endif
    }
}

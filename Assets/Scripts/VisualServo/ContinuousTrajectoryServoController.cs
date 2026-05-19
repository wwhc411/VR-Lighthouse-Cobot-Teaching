using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace VisualServo
{
    /// <summary>
    /// 连续轨迹视觉伺服控制器
    /// 
    /// 功能: 读取 CSV 文件中的 Tracker1 轨迹，逐帧执行视觉伺服
    /// 
    /// 控制逻辑:
    /// 1. 加载 CSV 轨迹文件
    /// 2. 首帧使用 MoveJ 快速接近 + Servoj 精细补偿
    /// 3. 后续帧直接使用 Servoj 补偿循环（不使用 MoveJ）
    /// 4. 每帧必须收敛才能继续下一帧
    /// 5. 任何帧超时/未收敛则立即中断并报错
    /// 
    /// 使用方法:
    /// 1. 配置 csvFilePath 指向录制的轨迹文件
    /// 2. 配置收敛阈值和超时参数
    /// 3. 按 T 键或调用 StartTrajectoryServo() 开始执行
    /// 4. 监听 OnTrajectoryComplete 事件获取完成通知
    /// 
    /// 快捷键:
    /// - T: 开始轨迹伺服
    /// - Y: 暂停/继续
    /// - U: 停止
    /// </summary>
    public class ContinuousTrajectoryServoController : MonoBehaviour
    {
        // ==================== Inspector 配置 ====================

        [Header("轨迹文件配置")]
        [Tooltip("CSV 轨迹文件路径 (相对于 StreamingAssets)")]
        public string csvFilePath = "TrackerRecordings/trajectory.csv";

        [Header("帧选择")]
        [Tooltip("起始帧索引 (0-based)")]
        public int startFrameIndex = 0;

        [Tooltip("结束帧索引 (-1 表示最后一帧)")]
        public int endFrameIndex = -1;

        [Tooltip("帧采样间隔 (1=每帧, 2=隔帧, 以此类推)")]
        [Range(1, 100)]
        public int frameSamplingInterval = 1;

        [Header("伺服参数")]
        [Tooltip("单帧最大迭代次数")]
        [Range(10, 2000)]
        public int maxIterationsPerFrame = 450;

        [Tooltip("位置收敛阈值 (mm)")]
        [Range(0.1f, 10f)]
        public float positionThresholdMm = 1.0f;

        [Tooltip("旋转收敛阈值 (度)")]
        [Range(0.1f, 10f)]
        public float rotationThresholdDeg = 1.0f;

        [Header("超时保护")]
        [Tooltip("单帧伺服超时时间 (秒)")]
        [Range(1f, 60f)]
        public float frameServoTimeoutSec = 10f;

        [Header("帧间等待")]
        [Tooltip("帧切换后的稳定等待时间 (秒)")]
        [Range(0f, 2f)]
        public float frameTransitionWaitSec = 0.1f;

        [Header("组件引用")]
        [Tooltip("单点视觉伺服控制器")]
        public VisualServoCompensationController singlePointServo;

        [Header("状态显示 (只读)")]
        [SerializeField] private bool _isRunning = false;
        [SerializeField] private int _currentFrameIndex = 0;
        [SerializeField] private int _totalFrames = 0;
        [SerializeField] private int _successFrames = 0;
        [SerializeField] private float _currentFrameError = 0f;
        [SerializeField] private float _elapsedTime = 0f;

        // ==================== 内部状态 ====================

        private RigidBodyCaptureData trajectoryData;
        private List<int> targetFrameIndices;  // 实际要执行的帧索引列表
        private Coroutine trajectoryCoroutine;
        private bool isRunning = false;
        private bool isPaused = false;

        // 统计信息
        private int currentFrameIndex = 0;
        private int successFrames = 0;
        private System.Diagnostics.Stopwatch trajectoryStopwatch;

        // 单帧伺服完成标志
        private bool frameServoCompleted = false;
        private bool frameServoConverged = false;

        // ==================== 公共事件 ====================

        /// <summary>
        /// 单帧伺服完成事件
        /// 参数: (帧索引, 是否收敛, 迭代次数, 最终误差)
        /// </summary>
        public event Action<int, bool, int, PoseError> OnFrameComplete;

        /// <summary>
        /// 轨迹完成事件
        /// 参数: (总帧数, 成功帧数, 是否全部完成, 总耗时秒)
        /// </summary>
        public event Action<int, int, bool, float> OnTrajectoryComplete;

        /// <summary>
        /// 进度更新事件
        /// 参数: (当前帧索引, 总帧数, 进度百分比)
        /// </summary>
        public event Action<int, int, float> OnProgressUpdate;

        // ==================== 公共属性 ====================

        /// <summary>
        /// 是否正在运行
        /// </summary>
        public bool IsRunning => isRunning;

        /// <summary>
        /// 是否暂停中
        /// </summary>
        public bool IsPaused => isPaused;

        /// <summary>
        /// 当前帧索引
        /// </summary>
        public int CurrentFrameIndex => currentFrameIndex;

        /// <summary>
        /// 总帧数
        /// </summary>
        public int TotalFrames => targetFrameIndices?.Count ?? 0;

        /// <summary>
        /// 成功帧数
        /// </summary>
        public int SuccessFrames => successFrames;

        // ==================== 生命周期 ====================

        void Start()
        {
            // 验证单点伺服控制器
            if (singlePointServo == null)
            {
                singlePointServo = GetComponent<VisualServoCompensationController>();
                if (singlePointServo == null)
                {
                    singlePointServo = FindObjectOfType<VisualServoCompensationController>();
                }
            }

            if (singlePointServo == null)
            {
                Debug.LogError("[轨迹伺服] 未找到 VisualServoCompensationController!");
                enabled = false;
                return;
            }

            trajectoryStopwatch = new System.Diagnostics.Stopwatch();

            Debug.Log("<color=cyan>[轨迹伺服] 初始化完成</color>");
            Debug.Log("  快捷键: T=开始轨迹伺服, Y=暂停/继续, U=停止");
        }

        void Update()
        {
            // 快捷键检测
            if (Input.GetKeyDown(KeyCode.T))
            {
                StartTrajectoryServo();
            }
            else if (Input.GetKeyDown(KeyCode.Y))
            {
                TogglePause();
            }
            else if (Input.GetKeyDown(KeyCode.U))
            {
                StopTrajectoryServo();
            }

            // 更新 Inspector 显示
            _isRunning = isRunning;
            _currentFrameIndex = currentFrameIndex;
            _totalFrames = targetFrameIndices?.Count ?? 0;
            _successFrames = successFrames;
            _elapsedTime = trajectoryStopwatch?.IsRunning == true ? 
                (float)trajectoryStopwatch.Elapsed.TotalSeconds : _elapsedTime;
            
            // 获取当前误差
            if (singlePointServo != null && singlePointServo.IsCompensating())
            {
                var error = singlePointServo.GetCurrentError();
                _currentFrameError = error?.positionMagnitude * 1000f ?? 0f;
            }
        }

        void OnDestroy()
        {
            // 清理：停止运行中的伺服
            if (isRunning)
            {
                StopTrajectoryServo();
            }
        }

        // ==================== 公开接口 ====================

        /// <summary>
        /// 开始连续轨迹视觉伺服
        /// </summary>
        [ContextMenu("开始轨迹伺服 (T键)")]
        public void StartTrajectoryServo()
        {
            if (isRunning)
            {
                Debug.LogWarning("[轨迹伺服] 已在运行中");
                return;
            }

            // 加载轨迹数据
            if (!LoadTrajectoryData())
            {
                return;
            }

            // 初始化状态
            currentFrameIndex = 0;
            successFrames = 0;
            isRunning = true;
            isPaused = false;
            frameServoCompleted = false;
            frameServoConverged = false;

            // 订阅单点伺服完成事件
            singlePointServo.OnCompensationComplete += OnSinglePointServoComplete;

            // 启动轨迹伺服协程
            trajectoryStopwatch.Restart();
            trajectoryCoroutine = StartCoroutine(TrajectoryServoCoroutine());

            Debug.Log("<color=green>==================== 开始轨迹伺服 ====================</color>");
            Debug.Log($"  轨迹文件: {csvFilePath}");
            Debug.Log($"  目标帧数: {targetFrameIndices.Count}");
            Debug.Log($"  帧范围: {startFrameIndex} ~ {(endFrameIndex < 0 ? "最后" : endFrameIndex.ToString())}");
            Debug.Log($"  采样间隔: {frameSamplingInterval}");
            Debug.Log($"  收敛阈值: 位置 < {positionThresholdMm}mm, 旋转 < {rotationThresholdDeg}°");
            Debug.Log($"  单帧超时: {frameServoTimeoutSec}s");
        }

        /// <summary>
        /// 停止轨迹伺服
        /// </summary>
        [ContextMenu("停止轨迹伺服 (U键)")]
        public void StopTrajectoryServo()
        {
            if (!isRunning)
            {
                return;
            }

            // 停止单点伺服
            if (singlePointServo != null && singlePointServo.IsCompensating())
            {
                singlePointServo.StopCompensation();
            }

            // 取消订阅
            if (singlePointServo != null)
            {
                singlePointServo.OnCompensationComplete -= OnSinglePointServoComplete;
            }

            // 停止协程
            if (trajectoryCoroutine != null)
            {
                StopCoroutine(trajectoryCoroutine);
                trajectoryCoroutine = null;
            }

            isRunning = false;
            isPaused = false;
            trajectoryStopwatch.Stop();

            Debug.Log("<color=yellow>[轨迹伺服] 已手动停止</color>");

            // 输出统计
            OutputStatistics();
        }

        /// <summary>
        /// 暂停/继续轨迹伺服
        /// </summary>
        [ContextMenu("暂停/继续 (Y键)")]
        public void TogglePause()
        {
            if (!isRunning) return;

            isPaused = !isPaused;
            
            if (isPaused)
            {
                Debug.Log("<color=yellow>[轨迹伺服] 已暂停</color>");
                // 暂停时停止当前单点伺服
                if (singlePointServo != null && singlePointServo.IsCompensating())
                {
                    singlePointServo.StopCompensation();
                }
            }
            else
            {
                Debug.Log("<color=green>[轨迹伺服] 继续执行</color>");
            }
        }

        /// <summary>
        /// 跳转到指定帧 (用于调试)
        /// </summary>
        /// <param name="frameIndex">目标帧索引</param>
        public void JumpToFrame(int frameIndex)
        {
            if (!isRunning)
            {
                Debug.LogWarning("[轨迹伺服] 未在运行, 无法跳转帧");
                return;
            }

            if (targetFrameIndices == null || frameIndex < 0 || frameIndex >= targetFrameIndices.Count)
            {
                Debug.LogError($"[轨迹伺服] 帧索引超出范围: {frameIndex} (有效范围: 0~{(targetFrameIndices?.Count ?? 0) - 1})");
                return;
            }

            // 停止当前伺服
            if (singlePointServo != null && singlePointServo.IsCompensating())
            {
                singlePointServo.StopCompensation();
            }

            currentFrameIndex = frameIndex;
            frameServoCompleted = true;  // 触发切换到新帧

            Debug.Log($"[轨迹伺服] 跳转到帧 {frameIndex + 1}/{targetFrameIndices.Count}");
        }

        // ==================== 核心逻辑 ====================

        /// <summary>
        /// 加载轨迹数据
        /// </summary>
        private bool LoadTrajectoryData()
        {
            Debug.Log($"[轨迹伺服] 加载轨迹文件: {csvFilePath}");
            
            trajectoryData = CSVCaptureReader.LoadFromCSV(csvFilePath);

            if (trajectoryData == null || trajectoryData.FrameData == null || trajectoryData.FrameData.Count == 0)
            {
                Debug.LogError($"[轨迹伺服] 无法加载轨迹数据: {csvFilePath}");
                return false;
            }

            // 构建目标帧索引列表
            targetFrameIndices = new List<int>();

            int totalFramesInFile = trajectoryData.FrameData.Count;
            int actualEndFrame = endFrameIndex < 0 ?
                totalFramesInFile - 1 :
                Mathf.Min(endFrameIndex, totalFramesInFile - 1);

            // 验证起始帧
            if (startFrameIndex < 0 || startFrameIndex >= totalFramesInFile)
            {
                Debug.LogError($"[轨迹伺服] 起始帧索引无效: {startFrameIndex} (文件总帧数: {totalFramesInFile})");
                return false;
            }

            // 按采样间隔选择帧
            for (int i = startFrameIndex; i <= actualEndFrame; i += frameSamplingInterval)
            {
                // 验证帧数据有效性
                var frame = trajectoryData.FrameData[i];
                if (frame != null && frame.IsPositionValid())
                {
                    targetFrameIndices.Add(i);
                }
                else
                {
                    Debug.LogWarning($"[轨迹伺服] 跳过无效帧: {i}");
                }
            }

            if (targetFrameIndices.Count == 0)
            {
                Debug.LogError("[轨迹伺服] 没有有效的目标帧");
                return false;
            }

            Debug.Log($"[轨迹伺服] 已加载轨迹: 原始 {totalFramesInFile} 帧, 采样后 {targetFrameIndices.Count} 帧");

            return true;
        }

        /// <summary>
        /// 轨迹伺服主协程
        /// </summary>
        private IEnumerator TrajectoryServoCoroutine()
        {
            Debug.Log("[轨迹伺服] 进入轨迹伺服循环");

            bool isFirstFrame = true;

            while (currentFrameIndex < targetFrameIndices.Count)
            {
                // ============ 暂停检查 ============
                while (isPaused)
                {
                    yield return null;
                }

                int dataFrameIndex = targetFrameIndices[currentFrameIndex];
                FrameData currentFrame = trajectoryData.FrameData[dataFrameIndex];

                // ============ 2.1 获取当前帧目标位姿 ============
                // CSV 中位置单位是 mm，转换为 m
                Vector3 targetPosition = new Vector3(
                    (float)currentFrame.Position.X / 1000f,
                    (float)currentFrame.Position.Y / 1000f,
                    (float)currentFrame.Position.Z / 1000f
                );

                Quaternion targetRotation = currentFrame.GetQuaternion();
                Vector3 targetRotVec = PoseErrorCalculator.QuaternionToRotationVector(targetRotation);

                Debug.Log($"<color=cyan>========== [轨迹伺服] 帧 {currentFrameIndex + 1}/{targetFrameIndices.Count} (数据帧 {dataFrameIndex}) ==========</color>");
                Debug.Log($"  目标位置 (SteamVR, m): ({targetPosition.x:F4}, {targetPosition.y:F4}, {targetPosition.z:F4})");
                Debug.Log($"  目标旋转 (rad): ({targetRotVec.x:F4}, {targetRotVec.y:F4}, {targetRotVec.z:F4})");

                // ============ 2.2 判断是否需要 MoveJ（仅首帧）============
                bool useMoveJ = isFirstFrame;
                if (isFirstFrame)
                {
                    Debug.Log("  <color=yellow>[首帧] 使用 MoveJ 快速接近目标</color>");
                }
                else
                {
                    Debug.Log("  [后续帧] 直接使用 Servoj 补偿 (跳过 MoveJ)");
                }
                isFirstFrame = false;

                // ============ 2.3 配置单点伺服参数 ============
                singlePointServo.maxIterations = maxIterationsPerFrame;
                singlePointServo.positionThreshold = positionThresholdMm / 1000f;  // mm → m
                singlePointServo.rotationThreshold = rotationThresholdDeg * Mathf.Deg2Rad;  // 度 → 弧度
                singlePointServo.enableTargetOffset = useMoveJ;  // 仅首帧启用位姿偏移
                singlePointServo.skipMoveJ = !useMoveJ;  // 后续帧跳过 MoveJ

                // ============ 2.4 设置目标并启动单点伺服 ============
                singlePointServo.SetTargetPose(targetPosition, targetRotVec);

                frameServoCompleted = false;
                frameServoConverged = false;

                // 启动单点伺服
                singlePointServo.StartCompensation();

                // ============ 2.5 等待单帧伺服完成或超时 ============
                float frameStartTime = Time.realtimeSinceStartup;

                while (!frameServoCompleted)
                {
                    // 检查超时
                    float elapsed = Time.realtimeSinceStartup - frameStartTime;
                    if (elapsed > frameServoTimeoutSec)
                    {
                        Debug.LogError($"[轨迹伺服] ❌ 帧 {currentFrameIndex + 1} 超时 ({elapsed:F1}s > {frameServoTimeoutSec}s)! 中断轨迹伺服");
                        singlePointServo.StopCompensation();
                        frameServoCompleted = true;
                        frameServoConverged = false;
                        break;
                    }

                    // 暂停检查
                    if (isPaused)
                    {
                        singlePointServo.StopCompensation();
                        Debug.Log($"[轨迹伺服] 帧 {currentFrameIndex + 1} 暂停中...");
                        
                        while (isPaused)
                        {
                            yield return null;
                        }
                        
                        // 暂停恢复后重新开始当前帧
                        Debug.Log($"[轨迹伺服] 帧 {currentFrameIndex + 1} 恢复执行");
                        singlePointServo.StartCompensation();
                        frameStartTime = Time.realtimeSinceStartup;
                    }

                    yield return null;
                }

                // ============ 2.6 判断本帧结果 ============
                PoseError finalError = singlePointServo.GetCurrentError();
                int iterations = singlePointServo.GetIterationCount();

                if (frameServoConverged)
                {
                    successFrames++;
                    Debug.Log($"<color=green>  ✅ 帧 {currentFrameIndex + 1} 收敛成功!</color>");
                    Debug.Log($"     迭代次数: {iterations}");
                    if (finalError != null)
                    {
                        Debug.Log($"     最终误差: 位置={finalError.positionMagnitude * 1000f:F3}mm, 旋转={finalError.rotationMagnitude * Mathf.Rad2Deg:F3}°");
                    }

                    // 触发帧完成事件（成功）
                    OnFrameComplete?.Invoke(currentFrameIndex, true, iterations, finalError);

                    // 触发进度更新事件
                    float progress = (currentFrameIndex + 1f) / targetFrameIndices.Count * 100f;
                    OnProgressUpdate?.Invoke(currentFrameIndex + 1, targetFrameIndices.Count, progress);

                    // ============ 2.7 帧间等待 ============
                    if (frameTransitionWaitSec > 0 && currentFrameIndex < targetFrameIndices.Count - 1)
                    {
                        Debug.Log($"  帧间等待: {frameTransitionWaitSec}s");
                        yield return new WaitForSeconds(frameTransitionWaitSec);
                    }

                    // 切换到下一帧
                    currentFrameIndex++;
                }
                else
                {
                    // 超时或未收敛：立即中断退出
                    Debug.LogError($"[轨迹伺服] ❌ 帧 {currentFrameIndex + 1} 未收敛, 中断轨迹伺服!");
                    if (finalError != null)
                    {
                        Debug.LogError($"  最终误差: 位置={finalError.positionMagnitude * 1000f:F2}mm (阈值:{positionThresholdMm}mm), " +
                                       $"旋转={finalError.rotationMagnitude * Mathf.Rad2Deg:F2}° (阈值:{rotationThresholdDeg}°)");
                    }
                    Debug.LogError($"  迭代次数: {iterations} / {maxIterationsPerFrame}");

                    // 触发帧完成事件（失败）
                    OnFrameComplete?.Invoke(currentFrameIndex, false, iterations, finalError);

                    // 立即退出循环
                    break;
                }
            }

            // ============ 阶段3: 轨迹结束 ============
            isRunning = false;
            trajectoryStopwatch.Stop();

            // 取消订阅
            singlePointServo.OnCompensationComplete -= OnSinglePointServoComplete;

            // 输出统计
            OutputStatistics();

            // 判断是否全部完成
            bool allCompleted = (successFrames == targetFrameIndices.Count);

            if (allCompleted)
            {
                Debug.Log("<color=green>==================== 轨迹伺服全部完成 ✅ ====================</color>");
            }
            else
            {
                Debug.LogError($"==================== 轨迹伺服中断 ❌ (完成 {successFrames}/{targetFrameIndices.Count} 帧) ====================");
            }

            // 触发完成事件
            OnTrajectoryComplete?.Invoke(
                targetFrameIndices.Count,
                successFrames,
                allCompleted,
                (float)trajectoryStopwatch.Elapsed.TotalSeconds
            );
        }

        /// <summary>
        /// 单点伺服完成回调
        /// </summary>
        private void OnSinglePointServoComplete(bool converged, int iterations)
        {
            frameServoCompleted = true;
            frameServoConverged = converged;
            
            Debug.Log($"[轨迹伺服] 单点伺服回调: converged={converged}, iterations={iterations}");
        }

        /// <summary>
        /// 输出统计信息
        /// </summary>
        private void OutputStatistics()
        {
            float totalTime = (float)trajectoryStopwatch.Elapsed.TotalSeconds;
            int totalFrames = targetFrameIndices?.Count ?? 0;
            float successRate = totalFrames > 0 ? (float)successFrames / totalFrames * 100f : 0f;

            Debug.Log("========== 轨迹伺服统计 ==========");
            Debug.Log($"  总帧数: {totalFrames}");
            Debug.Log($"  成功帧: {successFrames} ({successRate:F1}%)");
            Debug.Log($"  总耗时: {totalTime:F2} 秒");

            if (successFrames > 0 && totalTime > 0)
            {
                float avgTimePerFrame = totalTime / successFrames;
                Debug.Log($"  平均每帧: {avgTimePerFrame:F2} 秒");
            }

            if (successFrames < totalFrames)
            {
                Debug.LogWarning($"  ⚠️ 轨迹未完成! 在第 {successFrames + 1} 帧中断");
            }
        }

        // ==================== 调试功能 ====================

        /// <summary>
        /// 预览轨迹（不执行伺服，仅显示信息）
        /// </summary>
        [ContextMenu("预览轨迹信息")]
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

            // 显示前几帧和后几帧的位置
            int previewCount = Mathf.Min(3, targetFrameIndices.Count);
            
            Debug.Log("  前几帧位置:");
            for (int i = 0; i < previewCount; i++)
            {
                int idx = targetFrameIndices[i];
                var frame = trajectoryData.FrameData[idx];
                Debug.Log($"    帧{i + 1} (数据帧{idx}): ({frame.Position.X:F1}, {frame.Position.Y:F1}, {frame.Position.Z:F1}) mm");
            }

            if (targetFrameIndices.Count > previewCount * 2)
            {
                Debug.Log("    ...");
            }

            if (targetFrameIndices.Count > previewCount)
            {
                Debug.Log("  后几帧位置:");
                for (int i = Mathf.Max(previewCount, targetFrameIndices.Count - previewCount); i < targetFrameIndices.Count; i++)
                {
                    int idx = targetFrameIndices[i];
                    var frame = trajectoryData.FrameData[idx];
                    Debug.Log($"    帧{i + 1} (数据帧{idx}): ({frame.Position.X:F1}, {frame.Position.Y:F1}, {frame.Position.Z:F1}) mm");
                }
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

            // 清理临时数据
            trajectoryData = null;
            targetFrameIndices = null;
        }
    }
}

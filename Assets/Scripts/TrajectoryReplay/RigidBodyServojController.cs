using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

/// <summary>
/// 刚体数据Servoj回放控制器
/// 功能：
/// 1. 从CSV文件加载Tracker位姿数据
/// 2. 支持两种发送模式：
///    - 单帧实时发送：协程循环发送，可暂停/停止
///    - 批量脚本发送：一次性发送完整轨迹脚本
/// 3. 提供播放控制（播放/暂停/停止）
/// 4. UI实时显示播放状态
/// 5. 按L键快速加载CSV并开始回放
/// 更新: 2026-01-29 - 新增批量脚本发送模式
/// </summary>
public class RigidBodyServojController : MonoBehaviour
{
    // -------------------- Inspector 配置 -------------------- //
    
    [Header("CSV文件配置")]
    [Tooltip("CSV文件路径（相对于StreamingAssets或绝对路径）")]
    public string csvFilePath = "TrackerRecordings/TrackerRecord_2_20251202_120000.csv";

    [Header("Servoj控制参数")]
    [Tooltip("发送频率(Hz)，建议与数据捕获帧率一致")]
    [Range(10f, 500f)]
    public float sendFrequencyHz = 125f;

    [Tooltip("Servoj加速度参数(rad/s²)")]
    [Range(0f, 10f)]
    public float servojAcceleration = 0.001f;

    [Tooltip("Servoj速度参数(rad/s)")]
    [Range(0f, 3.14f)]
    public float servojVelocity = 0.01f;

    [Tooltip("Servoj前瞻时间(s)")]
    [Range(0.03f, 0.2f)]
    public float servojLookAheadTime = 0.1f;

    [Tooltip("Servoj控制增益")]
    [Range(100f, 2000f)]
    public float servojGain = 300f;

    [Header("回放模式配置")]
    [Tooltip("使用录制的TCP数据直接回放\n" +
             "✓ 勾选：直接使用CSV中记录的TCP位姿（跳过坐标转换）\n" +
             "✗ 不勾选：使用Tracker数据经过手眼标定转换后回放\n" +
             "注意：需要CSV包含TCP数据列")]
    public bool useRecordedTcpData = false;

    [Header("坐标转换配置（当不使用TCP直接回放时生效）")]
    [Tooltip("启用坐标转换（SteamVR → UR Base）\n" +
             "✓ 勾选：CSV数据来自Tracker录制，需要通过手眼标定矩阵转换\n" +
             "✗ 不勾选：CSV数据已经是UR基座坐标系")]
    public bool enableCoordinateTransform = true;

    [Tooltip("启用调试日志（显示每帧的坐标转换详情）")]
    public bool enableTransformDebugLog = false;

    [Header("Kabsch点云刚性对齐校正")]
    [Tooltip("启用Kabsch点云对齐校正\n" +
             "✓ 勾选：从KabschAlignment组件读取变换矩阵并应用校正\n" +
             "✗ 不勾选：不使用Kabsch校正")]
    public bool enableKabschAlignment = false;

    [Tooltip("Kabsch对齐组件引用\n" +
             "需要预先执行对齐计算，控制器将读取其计算结果")]
    public KabschAlignment kabschAlignmentComponent;

    [Header("Tracker坐标系偏移")]
    [Tooltip("启用Tracker本地坐标系偏移\n" +
             "用于补偿Tracker安装位置与实际TCP控制点的偏差")]
    public bool enableTrackerOffset = false;

    [Tooltip("Tracker本地坐标系位置偏移（毫米）\n" +
             "偏移量在Tracker本地坐标系中表示\n" +
             "Y轴向上，-Y方向为向下\n" +
             "例如：(0, -150, 0) 表示向下偏移150mm")]
    public Vector3 trackerPositionOffset = new Vector3(0f, -150f, 0f);

    [Tooltip("Tracker本地坐标系旋转偏移（欧拉角，度）")]
    public Vector3 trackerRotationOffset = Vector3.zero;

    [Header("=== 发送模式选择 ===")]
    [Tooltip("使用批量脚本发送模式\n" +
             "✓ 勾选：构建完整URScript脚本一次性发送，由机器人内部精确执行\n" +
             "✗ 不勾选：使用协程逐帧发送，可实时暂停/停止\n\n" +
             "批量模式优点：时序精确，网络开销低\n" +
             "批量模式缺点：发送后无法暂停（只能紧急停止）")]
    public bool useBatchScriptMode = false;

    [Tooltip("批量脚本回放指令类型\n" +
             "Servoj：实时伺服控制，高精度轨迹还原，时序精确\n" +
             "MoveJ：关节空间插值运动，平滑连续，适合低频轨迹点\n" +
             "MoveL：笛卡尔空间线性运动，TCP走直线，适合焊接/涂胶")]
    public BatchScriptCommandType batchCommandType = BatchScriptCommandType.Servoj;

    /// <summary>
    /// 批量脚本回放指令类型枚举
    /// </summary>
    public enum BatchScriptCommandType
    {
        [Tooltip("使用servoj指令回放，适合高频轨迹(125Hz)，精确轨迹还原")]
        Servoj,
        
        [Tooltip("使用movej指令回放，适合低频轨迹点，关节空间平滑运动")]
        MoveJ,
        
        [Tooltip("使用movel指令回放，笛卡尔空间线性运动，TCP走直线")]
        MoveL
    }

    [Header("=== 批量脚本参数（仅批量模式生效）===")]
    [Tooltip("点采样步长\n1=使用全部点\n2=隔点采样（轨迹点数减半）")]
    [Range(1, 10)]
    public int pointSamplingStep = 1;

    [Tooltip("每个脚本最大点数（超过则分段发送）")]
    [Range(1000, 10000)]
    public int maxPointsPerScript = 8000;

    [Tooltip("发送前保存脚本到文件（用于调试）")]
    public bool saveScriptToFile = false;

    [Tooltip("脚本保存目录（相对于StreamingAssets）")]
    public string scriptSaveDirectory = "GeneratedScripts";

    [Header("=== MoveJ批量回放参数（仅MoveJ模式生效）===")]
    [Tooltip("MoveJ关节加速度(rad/s²)\n推荐0.5-2.0，值越大加速越快")]
    [Range(0.1f, 5.0f)]
    public float movejAcceleration = 1.0f;

    [Tooltip("MoveJ关节速度(rad/s)\n推荐0.3-1.0，值越大运动越快")]
    [Range(0.1f, 2.0f)]
    public float movejVelocity = 0.5f;

    [Tooltip("MoveJ运动时间(s)\n>0时忽略加速度和速度参数\n设为0使用a/v参数控制")]
    [Range(0f, 5.0f)]
    public float movejTime = 0f;

    [Tooltip("MoveJ混合半径(m)\n用于平滑连接多个movej指令\n0=精确到达每个点，>0=提前过渡（更平滑）\n推荐0.005-0.02m")]
    [Range(0f, 0.1f)]
    public float movejBlendRadius = 0.01f;

    [Header("=== MoveL批量回放参数（仅MoveL模式生效）===")]
    [Tooltip("MoveL线性加速度(m/s²)\n推荐0.5-2.0，值越大加速越快")]
    [Range(0.01f, 5.0f)]
    public float movelAcceleration = 1.2f;

    [Tooltip("MoveL线性速度(m/s)\n推荐0.1-0.5，值越大运动越快")]
    [Range(0.01f, 1.0f)]
    public float movelVelocity = 0.25f;

    [Tooltip("MoveL运动时间(s)\n>0时忽略加速度和速度参数\n设为0使用a/v参数控制")]
    [Range(0f, 5.0f)]
    public float movelTime = 0f;

    [Tooltip("MoveL混合半径(m)\n用于平滑连接多个movel指令\n0=精确到达每个点，>0=提前过渡（更平滑）\n推荐0.005-0.02m")]
    [Range(0f, 0.1f)]
    public float movelBlendRadius = 0.01f;

    [Header("=== 第一帧MoveJ参数（两种模式均生效）===")]
    [Tooltip("第一帧MoveJ关节加速度(rad/s²)\n推荐0.3-0.5表示缓慢启动")]
    [Range(0.1f, 2.0f)]
    public float firstFrameAcceleration = 0.5f;

    [Tooltip("第一帧MoveJ关节速度(rad/s)\n推荐0.2-0.3表示缓慢移动")]
    [Range(0.1f, 1.0f)]
    public float firstFrameVelocity = 0.3f;

    [Tooltip("第一帧MoveJ执行等待时间(秒)\n仅单帧模式生效，推荐3-5秒")]
    [Range(1.0f, 10.0f)]
    public float firstFrameWaitTime = 5.0f;

    [Header("UI绑定")]
    public Button loadButton;           // 加载按钮
    public Button playButton;           // 播放按钮
    public Button pauseButton;          // 暂停按钮
    public Button stopButton;           // 停止按钮
    public TextMeshProUGUI statusText;  // 状态显示
    public TextMeshProUGUI progressText;// 进度显示
    public Slider progressSlider;       // 进度条

    [Header("轨迹可视化")]
    [Tooltip("加载数据后自动预览轨迹")]
    public bool autoPreviewAfterLoad = true;
    
    [Tooltip("轨迹预览显示时长（秒）")]
    [Range(5f, 60f)]
    public float previewDuration = 10f;
    
    [Tooltip("轨迹颜色")]
    public Color trajectoryColor = Color.cyan;

    [Header("回放过程自动录制")]
    [Tooltip("启用回放过程自动录制 Tracker 数据\n" +
             "回放开始时自动触发录制，结束后自动保存")]
    public bool enableAutoRecordDuringPlayback = false;

    [Tooltip("高频 Tracker 录制器组件引用\n" +
             "需要场景中已挂载 HighFrequencyTrackerRecorder 组件")]
    public HighFrequencyTrackerRecorder trackerRecorder;

    [Tooltip("如果未手动指定，是否自动查找场景中的录制器")]
    public bool autoFindRecorder = true;

    [Tooltip("录制文件名前缀（自动附加回放文件名）")]
    public string recordFilePrefix = "PlaybackRecord";

    // -------------------- 运行时数据 -------------------- //
    private RigidBodyCaptureData captureData;       // 加载的捕获数据
    private bool isPlaying = false;                 // 播放状态
    private bool isPaused = false;                  // 暂停状态
    private int currentFrameIndex = 0;              // 当前播放帧索引
    private Coroutine playbackCoroutine;            // 播放协程
    private UTF8Encoding utf8 = new UTF8Encoding(); // UTF-8编码器
    private bool isBatchScriptExecuting = false;    // 批量脚本执行中标志

    // -------------------- Unity生命周期 -------------------- //
    void Start()
    {
        // 绑定UI按钮事件
        if (loadButton != null) loadButton.onClick.AddListener(LoadAndPlay);
        if (playButton != null) playButton.onClick.AddListener(StartPlayback);
        if (pauseButton != null) pauseButton.onClick.AddListener(PausePlayback);
        if (stopButton != null) stopButton.onClick.AddListener(StopPlayback);

        UpdateStatusText("就绪 - 按L键加载CSV并回放");
        UpdateProgressText(0, 0);

        // 自动查找录制器组件
        if (trackerRecorder == null && autoFindRecorder)
        {
            trackerRecorder = FindObjectOfType<HighFrequencyTrackerRecorder>();
            if (trackerRecorder != null)
            {
                Debug.Log("[回放控制器] 自动找到 HighFrequencyTrackerRecorder 组件");
            }
        }

        // 初始化日志已精简，仅在需要时输出
    }

    void OnDisable()
    {
        // 停止所有协程，防止崩溃
        StopAllCoroutines();
        SendStopCommand();
    }
    
    void OnDestroy()
    {
        // 释放加载的数据内存
        ReleaseCaptureData();
    }
    
    /// <summary>
    /// 释放加载的捕获数据，回收内存
    /// </summary>
    public void ReleaseCaptureData()
    {
        if (captureData != null)
        {
            // 清空帧数据列表
            if (captureData.FrameData != null)
            {
                captureData.FrameData.Clear();
                captureData.FrameData = null;
            }
            captureData.Metadata = null;
            captureData = null;
            
            // 建议GC回收（不强制，由GC自行决定时机）
            // GC.Collect() 不推荐频繁调用
            
            // 数据已释放
        }
    }

    void Update()
    {
        // 按键检测
        if (Input.GetKeyDown(KeyCode.L))
        {
            LoadAndPlay();
        }
        else if (Input.GetKeyDown(KeyCode.P))
        {
            // 批量脚本模式下P键无效（无法暂停）
            if (!useBatchScriptMode)
            {
                PausePlayback();
            }
            else if (isBatchScriptExecuting)
            {
                Debug.LogWarning("[回放控制器] 批量脚本执行中无法暂停，按X键紧急停止");
            }
        }
        else if (Input.GetKeyDown(KeyCode.X))
        {
            StopPlayback();
        }
        else if (Input.GetKeyDown(KeyCode.V))
        {
            PreviewTrajectory();
        }
        
        // 更新进度条
        if (captureData != null && captureData.FrameData != null && captureData.FrameData.Count > 0)
        {
            if (progressSlider != null)
            {
                progressSlider.maxValue = captureData.FrameData.Count - 1;
                progressSlider.value = currentFrameIndex;
            }
        }
    }

    // -------------------- 数据加载 -------------------- //
    
    /// <summary>
    /// 加载CSV数据并开始回放（按L键触发）
    /// </summary>
    [ContextMenu("加载CSV并回放 (L键)")]
    public void LoadAndPlay()
    {
        // 如果正在播放，先停止
        if (isPlaying)
        {
            StopPlayback();
        }
        
        // 加载数据
        LoadCaptureData();
        
        // 如果加载成功，预览轨迹（可选）
        if (captureData != null && captureData.FrameData != null && captureData.FrameData.Count > 0)
        {
            if (autoPreviewAfterLoad)
            {
                PreviewTrajectory();
            }
            StartPlayback();
        }
    }

    /// <summary>
    /// 仅生成脚本并保存到文件（不发送，用于调试）
    /// </summary>
    [ContextMenu("仅生成脚本保存到文件")]
    public void GenerateScriptOnly()
    {
        // 加载数据
        if (captureData == null || captureData.FrameData == null || captureData.FrameData.Count == 0)
        {
            LoadCaptureData();
        }

        if (captureData == null || captureData.FrameData == null || captureData.FrameData.Count == 0)
        {
            Debug.LogError("[脚本生成] 未加载数据");
            return;
        }

        // 构建参数
        var scriptParams = new RigidBodyServojScriptGenerator.ServojScriptParameters
        {
            SendFrequencyHz = sendFrequencyHz,
            LookAheadTime = servojLookAheadTime,
            Gain = servojGain,
            PointStep = pointSamplingStep,
            EnableCoordinateTransform = enableCoordinateTransform,
            EnableKabschAlignment = enableKabschAlignment,
            EnableRotationContinuity = true,
            EnableTrackerOffset = enableTrackerOffset,
            TrackerPositionOffset = trackerPositionOffset,
            TrackerRotationOffset = trackerRotationOffset,
            FirstFrameAcceleration = firstFrameAcceleration,
            FirstFrameVelocity = firstFrameVelocity
        };

        // 生成脚本
        var result = RigidBodyServojScriptGenerator.GenerateTrajectoryScript(
            captureData, scriptParams, useRecordedTcpData);

        if (result.Success)
        {
            string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string fileName = $"trajectory_script_{timestamp}.txt";
            string savePath = Path.Combine(Application.streamingAssetsPath, scriptSaveDirectory, fileName);
            
            RigidBodyServojScriptGenerator.SaveScriptToFile(result.Script, savePath);
            
            Debug.Log($"[脚本生成] 完成! 保存到: {savePath}");
            Debug.Log($"  点数: {result.ProcessedPoints}, 大小: {result.ScriptSizeBytes}字节, 时长: {result.EstimatedDurationSeconds:F1}秒");
        }
        else
        {
            Debug.LogError($"[脚本生成] 失败: {result.ErrorMessage}");
        }
    }

    /// <summary>
    /// 紧急停止（批量脚本执行时使用）
    /// </summary>
    [ContextMenu("紧急停止 (X键)")]
    public void EmergencyStop()
    {
        RigidBodyServojScriptGenerator.SendEmergencyStop();
        isBatchScriptExecuting = false;
        isPlaying = false;
        UpdateStatusText("紧急停止");
        Debug.Log("[回放控制器] 紧急停止命令已发送");
    }
    
    /// <summary>
    /// 从CSV文件加载捕获数据
    /// </summary>
    public void LoadCaptureData()
    {
        UpdateStatusText("正在加载CSV...");

        captureData = CSVCaptureReader.LoadFromCSV(csvFilePath);

        if (captureData == null)
        {
            UpdateStatusText("加载失败 - 检查文件路径和格式");
            Debug.LogError($"[RigidBodyServojController] 加载失败: {csvFilePath}");
            return;
        }

        // 验证数据
        if (CSVCaptureReader.ValidateCSVFile(csvFilePath, out string errorMessage))
        {
            UpdateStatusText($"加载成功 - {captureData.Metadata.RigidBodyName}: {captureData.Metadata.TotalFrames}帧");

            Debug.Log($"[RigidBodyServojController] 数据名称: {captureData.Metadata.RigidBodyName}, " +
                     $"总帧数: {captureData.Metadata.TotalFrames}, " +
                     $"发送频率: {sendFrequencyHz:F2}Hz");
        }
        else
        {
            UpdateStatusText($"数据验证失败 - {errorMessage}");
            captureData = null;
        }
    }

    // -------------------- 播放控制 -------------------- //
    /// <summary>
    /// 开始播放
    /// </summary>
    public void StartPlayback()
    {
        if (captureData == null || captureData.FrameData == null || captureData.FrameData.Count == 0)
        {
            UpdateStatusText("错误 - 未加载数据");
            Debug.LogWarning("[RigidBodyServojController] 未加载数据，无法播放");
            return;
        }

        if (!ur_data_processing.UR_Control_Data.is_alive)
        {
            UpdateStatusText("错误 - 机器人未连接");
            Debug.LogError("[RigidBodyServojController] 机器人未连接");
            return;
        }

        if (isPlaying && !isPaused)
        {
            Debug.LogWarning("[RigidBodyServojController] 已在播放中");
            return;
        }

        // 从暂停恢复（仅单帧模式支持）
        if (isPaused && !useBatchScriptMode)
        {
            isPaused = false;
            UpdateStatusText("播放中...");
            Debug.Log("[RigidBodyServojController] 从暂停恢复播放");
            return;
        }

        // 开始新的播放
        isPlaying = true;
        isPaused = false;
        currentFrameIndex = 0;

        // 根据模式选择发送方式
        if (useBatchScriptMode)
        {
            // ========== 批量脚本发送模式 ==========
            StartBatchScriptPlayback();
        }
        else
        {
            // ========== 单帧实时发送模式 ==========
            UpdateStatusText("播放中...");
            Debug.Log($"[RigidBodyServojController] 开始单帧播放 - 频率{sendFrequencyHz}Hz");
            Debug.Log($"  坐标转换: {(enableCoordinateTransform ? "启用" : "禁用")}");

            playbackCoroutine = StartCoroutine(PlaybackCoroutine());
        }
    }

    /// <summary>
    /// 批量脚本发送模式的播放入口
    /// 支持Servoj、MoveJ和MoveL三种指令类型
    /// </summary>
    private void StartBatchScriptPlayback()
    {
        string commandTypeStr = batchCommandType switch
        {
            BatchScriptCommandType.MoveJ => "MoveJ",
            BatchScriptCommandType.MoveL => "MoveL",
            _ => "Servoj"
        };
        Debug.Log($"[批量脚本] 开始生成轨迹脚本 (指令类型: {commandTypeStr})...");
        Debug.Log($"  总帧数: {captureData.FrameData.Count}");
        Debug.Log($"  采样步长: {pointSamplingStep}");
        Debug.Log($"  模式: {(useRecordedTcpData ? "TCP直接回放" : "Tracker+坐标转换")}");

        // 配置Kabsch变换（如果启用）- 所有模式都需要
        if (enableKabschAlignment)
        {
            if (kabschAlignmentComponent == null || !kabschAlignmentComponent.IsAlignmentComputed)
            {
                Debug.LogError("[批量脚本] Kabsch对齐未配置或未计算！");
                UpdateStatusText("错误 - Kabsch未就绪");
                isPlaying = false;
                return;
            }

            // 设置到对应的生成器
            switch (batchCommandType)
            {
                case BatchScriptCommandType.MoveJ:
                    RigidBodyMovejScriptGenerator.SetKabschTransform(
                        kabschAlignmentComponent.RotationMatrix,
                        kabschAlignmentComponent.TranslationVector);
                    break;
                case BatchScriptCommandType.MoveL:
                    RigidBodyMovelScriptGenerator.SetKabschTransform(
                        kabschAlignmentComponent.RotationMatrix,
                        kabschAlignmentComponent.TranslationVector);
                    break;
                default:
                    RigidBodyServojScriptGenerator.SetKabschTransform(
                        kabschAlignmentComponent.RotationMatrix,
                        kabschAlignmentComponent.TranslationVector);
                    break;
            }
            
            Debug.Log($"[批量脚本] Kabsch校正已配置 (RMSE={kabschAlignmentComponent.RMSE:F6})");
        }
        else
        {
            RigidBodyServojScriptGenerator.ClearKabschTransform();
            RigidBodyMovejScriptGenerator.ClearKabschTransform();
            RigidBodyMovelScriptGenerator.ClearKabschTransform();
        }

        // 根据指令类型生成脚本
        UpdateStatusText($"生成{commandTypeStr}脚本中...");
        
        string script;
        int processedPoints, totalPoints;
        long scriptSizeBytes;
        float estimatedDurationSeconds;

        if (batchCommandType == BatchScriptCommandType.MoveL)
        {
            // ========== MoveL模式 ==========
            var movelParams = new RigidBodyMovelScriptGenerator.MovelScriptParameters
            {
                Acceleration = movelAcceleration,
                Velocity = movelVelocity,
                Time = movelTime,
                BlendRadius = movelBlendRadius,
                PointStep = pointSamplingStep,
                EnableCoordinateTransform = enableCoordinateTransform,
                EnableKabschAlignment = enableKabschAlignment,
                EnableRotationContinuity = true,
                EnableTrackerOffset = enableTrackerOffset,
                TrackerPositionOffset = trackerPositionOffset,
                TrackerRotationOffset = trackerRotationOffset
            };

            var movelResult = RigidBodyMovelScriptGenerator.GenerateTrajectoryScript(
                captureData, movelParams, useRecordedTcpData);

            if (!movelResult.Success)
            {
                Debug.LogError($"[MoveL脚本] 生成失败: {movelResult.ErrorMessage}");
                UpdateStatusText($"脚本生成失败: {movelResult.ErrorMessage}");
                isPlaying = false;
                return;
            }

            script = movelResult.Script;
            processedPoints = movelResult.ProcessedPoints;
            totalPoints = movelResult.TotalPoints;
            scriptSizeBytes = movelResult.ScriptSizeBytes;
            estimatedDurationSeconds = movelResult.EstimatedDurationSeconds;
            
            Debug.Log($"[MoveL脚本] MoveL参数: a={movelAcceleration}, v={movelVelocity}, t={movelTime}, r={movelBlendRadius}");
        }
        else if (batchCommandType == BatchScriptCommandType.MoveJ)
        {
            // ========== MoveJ模式 ==========
            var movejParams = new RigidBodyMovejScriptGenerator.MovejScriptParameters
            {
                Acceleration = movejAcceleration,
                Velocity = movejVelocity,
                Time = movejTime,
                BlendRadius = movejBlendRadius,
                PointStep = pointSamplingStep,
                EnableCoordinateTransform = enableCoordinateTransform,
                EnableKabschAlignment = enableKabschAlignment,
                EnableRotationContinuity = true,
                EnableTrackerOffset = enableTrackerOffset,
                TrackerPositionOffset = trackerPositionOffset,
                TrackerRotationOffset = trackerRotationOffset
            };

            var movejResult = RigidBodyMovejScriptGenerator.GenerateTrajectoryScript(
                captureData, movejParams, useRecordedTcpData);

            if (!movejResult.Success)
            {
                Debug.LogError($"[MoveJ脚本] 生成失败: {movejResult.ErrorMessage}");
                UpdateStatusText($"脚本生成失败: {movejResult.ErrorMessage}");
                isPlaying = false;
                return;
            }

            script = movejResult.Script;
            processedPoints = movejResult.ProcessedPoints;
            totalPoints = movejResult.TotalPoints;
            scriptSizeBytes = movejResult.ScriptSizeBytes;
            estimatedDurationSeconds = movejResult.EstimatedDurationSeconds;
            
            Debug.Log($"[MoveJ脚本] MoveJ参数: a={movejAcceleration}, v={movejVelocity}, t={movejTime}, r={movejBlendRadius}");
        }
        else
        {
            // ========== Servoj模式（原有逻辑）==========
            var scriptParams = new RigidBodyServojScriptGenerator.ServojScriptParameters
            {
                SendFrequencyHz = sendFrequencyHz,
                LookAheadTime = servojLookAheadTime,
                Gain = servojGain,
                PointStep = pointSamplingStep,
                EnableCoordinateTransform = enableCoordinateTransform,
                EnableKabschAlignment = enableKabschAlignment,
                EnableRotationContinuity = true,
                EnableTrackerOffset = enableTrackerOffset,
                TrackerPositionOffset = trackerPositionOffset,
                TrackerRotationOffset = trackerRotationOffset,
                FirstFrameAcceleration = firstFrameAcceleration,
                FirstFrameVelocity = firstFrameVelocity
            };

            var servojResult = RigidBodyServojScriptGenerator.GenerateTrajectoryScript(
                captureData, scriptParams, useRecordedTcpData);

            if (!servojResult.Success)
            {
                Debug.LogError($"[Servoj脚本] 生成失败: {servojResult.ErrorMessage}");
                UpdateStatusText($"脚本生成失败: {servojResult.ErrorMessage}");
                isPlaying = false;
                return;
            }

            script = servojResult.Script;
            processedPoints = servojResult.ProcessedPoints;
            totalPoints = servojResult.TotalPoints;
            scriptSizeBytes = servojResult.ScriptSizeBytes;
            estimatedDurationSeconds = servojResult.EstimatedDurationSeconds;
        }

        Debug.Log($"[{commandTypeStr}脚本] 生成成功:");
        Debug.Log($"  处理点数: {processedPoints}/{totalPoints}");
        Debug.Log($"  脚本大小: {scriptSizeBytes} 字节");
        Debug.Log($"  预计时长: {estimatedDurationSeconds:F1} 秒");

        // 保存脚本到文件（可选，用于调试）
        if (saveScriptToFile)
        {
            string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string typePrefix = batchCommandType switch
            {
                BatchScriptCommandType.MoveJ => "movej",
                BatchScriptCommandType.MoveL => "movel",
                _ => "servoj"
            };
            string fileName = $"trajectory_{typePrefix}_{timestamp}.txt";
            string savePath = Path.Combine(Application.streamingAssetsPath, scriptSaveDirectory, fileName);
            
            switch (batchCommandType)
            {
                case BatchScriptCommandType.MoveJ:
                    RigidBodyMovejScriptGenerator.SaveScriptToFile(script, savePath);
                    break;
                case BatchScriptCommandType.MoveL:
                    RigidBodyMovelScriptGenerator.SaveScriptToFile(script, savePath);
                    break;
                default:
                    RigidBodyServojScriptGenerator.SaveScriptToFile(script, savePath);
                    break;
            }
            Debug.Log($"[{commandTypeStr}脚本] 脚本已保存: {savePath}");
        }

        // 发送脚本到UR
        UpdateStatusText("发送脚本...");
        bool sendSuccess = batchCommandType switch
        {
            BatchScriptCommandType.MoveJ => RigidBodyMovejScriptGenerator.SendScriptToUR(script),
            BatchScriptCommandType.MoveL => RigidBodyMovelScriptGenerator.SendScriptToUR(script),
            _ => RigidBodyServojScriptGenerator.SendScriptToUR(script)
        };

        if (sendSuccess)
        {
            isBatchScriptExecuting = true;
            UpdateStatusText($"执行中({commandTypeStr}) - 预计{estimatedDurationSeconds:F1}秒");
            Debug.Log($"[{commandTypeStr}脚本] ✓ 脚本已发送，机器人开始执行");
            
            // ========== 🎬 开始自动录制（脚本发送成功后）==========
            StartAutoRecording();
            
            // 启动超时检测协程
            StartCoroutine(BatchScriptTimeoutCoroutine(estimatedDurationSeconds + 5f));
        }
        else
        {
            Debug.LogError($"[{commandTypeStr}脚本] 发送失败");
            UpdateStatusText("脚本发送失败");
            isPlaying = false;
        }
    }

    /// <summary>
    /// 批量脚本执行超时检测协程
    /// </summary>
    private IEnumerator BatchScriptTimeoutCoroutine(float timeoutSeconds)
    {
        yield return new WaitForSeconds(timeoutSeconds);
        
        if (isBatchScriptExecuting)
        {
            isBatchScriptExecuting = false;
            isPlaying = false;
            
            // ========== 🎬 停止自动录制（执行完成/超时后）==========
            StopAutoRecording();
            
            UpdateStatusText("执行完成");
            Debug.Log("[批量脚本] 执行超时/完成");
        }
    }

    /// <summary>
    /// 暂停播放
    /// </summary>
    public void PausePlayback()
    {
        if (!isPlaying)
        {
            Debug.LogWarning("[RigidBodyServojController] 当前未播放");
            return;
        }

        isPaused = !isPaused;
        UpdateStatusText(isPaused ? "已暂停" : "播放中...");
        Debug.Log($"[RigidBodyServojController] {(isPaused ? "暂停" : "继续")}播放");

        if (isPaused)
        {
            SendStopCommand(); // 暂停时发送停止命令
        }
    }

    /// <summary>
    /// 停止播放
    /// </summary>
    public void StopPlayback()
    {
        StopPlayback(releaseData: false);
    }
    
    /// <summary>
    /// 停止播放（可选释放数据）
    /// </summary>
    /// <param name="releaseData">是否同时释放加载的数据以回收内存</param>
    public void StopPlayback(bool releaseData)
    {
        // 停止单帧模式的协程
        if (playbackCoroutine != null)
        {
            StopCoroutine(playbackCoroutine);
            playbackCoroutine = null;
        }

        // 批量脚本模式：发送紧急停止
        if (isBatchScriptExecuting)
        {
            RigidBodyServojScriptGenerator.SendEmergencyStop();
            isBatchScriptExecuting = false;
            Debug.Log("[批量脚本] 已发送紧急停止命令");
        }
        else
        {
            SendStopCommand();
        }

        // ========== 🎬 手动停止时也停止自动录制 ==========
        if (enableAutoRecordDuringPlayback && trackerRecorder != null && trackerRecorder.IsRecording)
        {
            trackerRecorder.StopHighFrequencyRecording();
            Debug.Log("[回放控制器] 手动停止：自动录制也已停止");
        }

        isPlaying = false;
        isPaused = false;
        currentFrameIndex = 0;

        UpdateStatusText("已停止");
        UpdateProgressText(0, captureData?.FrameData?.Count ?? 0);

        Debug.Log("[RigidBodyServojController] 停止播放");
        
        // 可选：释放数据内存
        if (releaseData)
        {
            ReleaseCaptureData();
        }
    }

    // -------------------- 播放协程 -------------------- //
    /// <summary>
    /// 播放协程 - 按频率发送servoj命令
    /// </summary>
    private IEnumerator PlaybackCoroutine()
    {
        // 同步坐标转换设置到命令生成器
        RigidBodyServojCommandGenerator.EnableCoordinateTransform = enableCoordinateTransform;
        RigidBodyServojCommandGenerator.EnableDebugLog = enableTransformDebugLog;
        
        // 同步Tracker偏移设置
        RigidBodyServojCommandGenerator.EnableTrackerOffset = enableTrackerOffset;
        RigidBodyServojCommandGenerator.TrackerPositionOffset = trackerPositionOffset;
        RigidBodyServojCommandGenerator.TrackerRotationOffset = trackerRotationOffset;

        // 确定回放模式（提前判断避免重复检查）
        bool useTcpMode = useRecordedTcpData && captureData.FrameData.Count > 0 && captureData.FrameData[0].HasTcpData;
        
        // 同步并配置Kabsch对齐设置
        RigidBodyServojCommandGenerator.EnableKabschAlignment = enableKabschAlignment;
        
        if (enableKabschAlignment)
        {
            if (kabschAlignmentComponent == null)
            {
                Debug.LogError("[回放] 启用了Kabsch对齐但未指定KabschAlignment组件！");
                UpdateStatusText("错误 - 缺少Kabsch组件");
                StopPlayback();
                yield break;
            }

            if (!kabschAlignmentComponent.IsAlignmentComputed)
            {
                Debug.LogError("[回放] KabschAlignment组件尚未执行对齐计算！请先执行对齐");
                UpdateStatusText("错误 - Kabsch未对齐");
                StopPlayback();
                yield break;
            }

            // 从KabschAlignment组件读取变换结果并设置到命令生成器
            RigidBodyServojCommandGenerator.SetKabschTransform(
                kabschAlignmentComponent.RotationMatrix,
                kabschAlignmentComponent.TranslationVector
            );
            
            Debug.Log($"[回放] Kabsch对齐已启用 - RMSE: {kabschAlignmentComponent.RMSE:F6}");
            
            // 根据回放模式提示训练要求
            if (useTcpMode)
            {
                Debug.LogWarning("[回放] TCP模式+Kabsch校正：确保Kabsch训练点云为UR基座坐标系！");
            }
            else
            {
                Debug.Log("[回放] Tracker模式+Kabsch校正：Kabsch训练点云应为SteamVR坐标系");
            }
        }
        else
        {
            // 如果禁用，清除之前的Kabsch变换
            RigidBodyServojCommandGenerator.ClearKabschTransform();
        }
        
        // 输出当前回放模式
        if (useTcpMode)
        {
            string kabschStatus = enableKabschAlignment ? "含Kabsch校正" : "无校正";
            Debug.Log($"[回放] 模式：TCP直接回放（{kabschStatus}）");
        }
        else
        {
            Debug.Log($"[回放] 模式：Tracker+坐标转换（Kabsch={enableKabschAlignment}）");
        }

        // 重置旋转连续性状态（新的回放开始时必须重置）
        RigidBodyServojCommandGenerator.ResetRotationContinuityState();

        // 构建Servoj参数
        RigidBodyServojCommandGenerator.ServojParameters parameters = new RigidBodyServojCommandGenerator.ServojParameters
        {
            Acceleration = servojAcceleration,
            Velocity = servojVelocity,
            TimeStep = 1.0f / sendFrequencyHz,  // 根据频率自动计算
            LookAheadTime = servojLookAheadTime,
            Gain = servojGain
        };

        // 验证参数
        if (!parameters.Validate(out string errorMessage))
        {
            Debug.LogError($"[RigidBodyServojController] 参数无效: {errorMessage}");
            StopPlayback();
            yield break;
        }

        // 计算发送间隔
        float sendInterval = 1.0f / sendFrequencyHz;

        // 播放参数日志已精简

        // ========== 🎬 开始自动录制（第一帧发送前）==========
        StartAutoRecording();

        // 播放循环
        int sentCount = 0;
        bool isFirstFrame = true;  // 标记第一帧
        
        // 验证TCP模式的数据可用性
        if (useRecordedTcpData && !useTcpMode)
        {
            Debug.LogWarning("[回放] CSV不包含TCP数据，将自动切换为Tracker+坐标转换模式");
        }
        
        while (currentFrameIndex < captureData.FrameData.Count)
        {
            // 暂停检查
            while (isPaused)
            {
                yield return null;
            }

            // 获取当前帧数据
            FrameData currentFrame = captureData.FrameData[currentFrameIndex];
            
            // 检查位置数据有效性
            if (!currentFrame.IsPositionValid())
            {
                Debug.LogWarning($"[RigidBodyServojController] 帧{currentFrameIndex}位置数据无效，跳过");
                currentFrameIndex++;
                yield return new WaitForSeconds(sendInterval);
                continue;
            }

            // 生成命令 - 第一帧使用movej，其余帧使用servoj
            string command;
            if (isFirstFrame)
            {
                // 第一帧：使用movej缓慢移动到轨迹起始点
                // 重要：传入useTcpMode参数，确保与后续servoj帧使用相同的数据源和坐标转换逻辑
                command = RigidBodyServojCommandGenerator.GenerateMovejCommand(
                    currentFrame,
                    acceleration: firstFrameAcceleration,  // 使用Inspector配置的加速度
                    velocity: firstFrameVelocity,          // 使用Inspector配置的速度
                    time: 0.0,                              // 0表示使用a和v参数控制速度
                    blendRadius: 0.0,                       // 混合半径0（单点移动无需过渡）
                    currentJointAngles: null,               // 使用当前关节角度
                    useTcpDirectMode: useTcpMode);          // 关键：与后续帧保持一致的模式
                
                isFirstFrame = false;  // 标记已处理第一帧
                string modeDesc = useTcpMode ? "TCP直接模式" : "Tracker+手眼转换模式";
                Debug.Log($"[回放] 第一帧movej ({modeDesc}) - a={firstFrameAcceleration:F2}, v={firstFrameVelocity:F2}");
            }
            else
            {
                // 其余帧：使用servoj实时跟踪
                if (useTcpMode)
                {
                    // TCP直接回放模式：使用录制的TCP数据
                    command = RigidBodyServojCommandGenerator.GenerateServojCommandFromTcpData(currentFrame, parameters);
                }
                else
                {
                    // Tracker+坐标转换模式：使用原有逻辑
                    command = RigidBodyServojCommandGenerator.GenerateServojCommand(currentFrame, parameters);
                }
            }

            if (!string.IsNullOrEmpty(command))
            {
                // 发送命令到UR
                SendCommandToUR(command);
                sentCount++;

                // 首帧输出命令
                if (sentCount == 1)
                {
                    Debug.Log($"[回放] 首帧命令: {command.TrimEnd()}");
                }

                // 更新进度显示
                UpdateProgressText(currentFrameIndex + 1, captureData.FrameData.Count);
            }
            else
            {
                Debug.LogError($"[RigidBodyServojController] 帧{currentFrameIndex}命令生成失败");
            }

            // 等待下一个发送周期
            // 第一帧movej需要更长时间等待到达
            if (sentCount == 1)
            {
                // 第一帧：等待movej执行完成
                Debug.Log($"[回放] 等待movej执行完成（{firstFrameWaitTime}秒）...");
                yield return new WaitForSeconds(firstFrameWaitTime);
            }
            else
            {
                // 其余帧：正常周期发送
                yield return new WaitForSeconds(sendInterval);
            }

            currentFrameIndex++;
        }

        // ========== 🎬 停止自动录制（播放完成后）==========
        StopAutoRecording();

        // 播放完成 - 停止
        isPlaying = false;
        SendStopCommand();
        UpdateStatusText("播放完成");
        Debug.Log($"[回放] 完成，共发送 {sentCount} 条命令");
    }

    // -------------------- 命令发送 -------------------- //
    /// <summary>
    /// 发送命令到UR控制缓冲区
    /// </summary>
    private void SendCommandToUR(string command)
    {
        byte[] commandBytes = utf8.GetBytes(command);
        ur_data_processing.UR_Control_Data.aux_command_str = command;
        ur_data_processing.UR_Control_Data.command = commandBytes;
        ur_data_processing.UR_Control_Data.manual_send_active = true;
        
        // 调试：输出发送的命令（前100字符）
        if (enableTransformDebugLog)
        {
            string cmdPreview = command.Length > 100 ? command.Substring(0, 100) + "..." : command.TrimEnd();
            Debug.Log($"<color=yellow>[发送命令]</color> {cmdPreview}");
        }
    }

    /// <summary>
    /// 发送停止命令
    /// </summary>
    private void SendStopCommand()
    {
        string stopCommand = RigidBodyServojCommandGenerator.GenerateStopCommand();
        SendCommandToUR(stopCommand);
        
        // 等待发送完成（控制线程周期8ms，等待20ms确保发送）
        System.Threading.Thread.Sleep(20);
        
        // 重要：发送停止命令后，必须重置manual_send_active标志
        // 否则控制线程会持续发送stopl命令，导致机械臂无法被其他功能控制
        ur_data_processing.UR_Control_Data.manual_send_active = false;
        
        // 关键：清空command缓冲区，防止后续摇杆操作误发之前的命令
        ur_data_processing.UR_Control_Data.command = new byte[0];
        ur_data_processing.UR_Control_Data.aux_command_str = "";
        
        Debug.Log("[回放控制器] 停止命令已发送，缓冲区已清空");
    }

    // -------------------- UI更新 -------------------- //
    /// <summary>
    /// 更新状态文本
    /// </summary>
    private void UpdateStatusText(string status)
    {
        if (statusText != null)
        {
            statusText.text = $"状态: {status}";
        }
    }

    /// <summary>
    /// 更新进度文本
    /// </summary>
    private void UpdateProgressText(int current, int total)
    {
        if (progressText != null)
        {
            float percentage = total > 0 ? (current / (float)total) * 100f : 0f;
            progressText.text = $"进度: {current}/{total} ({percentage:F1}%)";
        }
    }
    
    // -------------------- 轨迹可视化 -------------------- //
    
    /// <summary>
    /// 预览加载的轨迹（Scene视图中显示10秒）
    /// </summary>
    [ContextMenu("预览播放轨迹 (V键)")]
    public void PreviewTrajectory()
    {
        if (captureData == null || captureData.FrameData == null || captureData.FrameData.Count < 2)
        {
            Debug.LogWarning("[回放控制器] 没有轨迹数据可预览，请先加载CSV");
            return;
        }
        
        // 提取轨迹点（转换为Unity世界坐标）
        List<Vector3> trajectoryPoints = new List<Vector3>();
        
        foreach (var frame in captureData.FrameData)
        {
            if (frame.IsPositionValid())
            {
                // 使用Tracker数据（毫米转米）
                // 注意：PositionData使用大写的X, Y, Z属性
                Vector3 position = new Vector3(
                    (float)(frame.Position.X * 0.001),
                    (float)(frame.Position.Y * 0.001),
                    (float)(frame.Position.Z * 0.001)
                );
                trajectoryPoints.Add(position);
            }
        }
        
        if (trajectoryPoints.Count < 2)
        {
            Debug.LogWarning("[回放控制器] 有效轨迹点少于2个");
            return;
        }
        
        DrawTrajectory(trajectoryPoints, trajectoryColor, previewDuration);
        
        Debug.Log($"========== 轨迹预览 ==========");
        Debug.Log($"已在Scene视图中绘制轨迹（持续{previewDuration}秒）：");
        Debug.Log($"  <color={ColorToHex(trajectoryColor)}>青色</color> = 播放轨迹");
        Debug.Log($"  轨迹点数: {trajectoryPoints.Count}");
        Debug.Log($"  总帧数: {captureData.FrameData.Count}");
        Debug.Log($"\n请观察：");
        Debug.Log($"  1. 轨迹形状是否符合预期？");
        Debug.Log($"  2. 是否有异常跳变或断裂？");
        Debug.Log($"  3. 轨迹范围是否合理？");
        Debug.Log($"  4. 确认后按P键开始播放");
        Debug.Log($"================================\n");
    }
    
    /// <summary>
    /// 绘制轨迹到Scene视图
    /// </summary>
    private void DrawTrajectory(List<Vector3> points, Color color, float duration)
    {
        if (points == null || points.Count < 2) return;
        
        // 绘制轨迹线段
        for (int i = 0; i < points.Count - 1; i++)
        {
            Debug.DrawLine(points[i], points[i + 1], color, duration);
        }
        
        // 标记起点（向上延伸的竖线）
        if (points.Count > 0)
        {
            Debug.DrawLine(
                points[0],
                points[0] + Vector3.up * 0.05f,  // 向上5cm
                Color.yellow,
                duration
            );
        }
        
        // 标记终点（向上延伸的竖线）
        if (points.Count > 1)
        {
            Debug.DrawLine(
                points[points.Count - 1],
                points[points.Count - 1] + Vector3.up * 0.05f,
                Color.red,
                duration
            );
        }
    }
    
    /// <summary>
    /// 颜色转十六进制字符串（用于富文本）
    /// </summary>
    private string ColorToHex(Color color)
    {
        int r = (int)(color.r * 255);
        int g = (int)(color.g * 255);
        int b = (int)(color.b * 255);
        return $"#{r:X2}{g:X2}{b:X2}";
    }

    #region 回放过程自动录制

    /// <summary>
    /// 开始自动录制（回放开始时调用）
    /// </summary>
    private void StartAutoRecording()
    {
        if (!enableAutoRecordDuringPlayback) return;
        
        if (trackerRecorder == null)
        {
            Debug.LogWarning("[回放控制器] 自动录制：未找到 HighFrequencyTrackerRecorder");
            return;
        }
        
        // 注意：不要在这里调用 StopHighFrequencyRecording()
        // 因为它会同步等待后台线程结束（最多2秒），可能阻塞回放协程
        // 如果录制器已在录制，记录警告日志并跳过
        if (trackerRecorder.IsRecording)
        {
            Debug.LogWarning("[回放控制器] 自动录制：录制器已在录制中，跳过自动启动（请先手动停止）");
            return;
        }
        
        // 设置录制文件名前缀（关联回放文件名）
        string csvName = Path.GetFileNameWithoutExtension(csvFilePath);
        trackerRecorder.fileNamePrefix = $"{recordFilePrefix}_{csvName}";
        
        // 开始录制
        trackerRecorder.StartHighFrequencyRecording();
        
        Debug.Log($"<color=cyan>[回放控制器] 🎬 自动录制已启动</color>");
        Debug.Log($"  文件前缀: {trackerRecorder.fileNamePrefix}");
    }

    /// <summary>
    /// 停止自动录制（回放结束时调用）
    /// </summary>
    private void StopAutoRecording()
    {
        if (!enableAutoRecordDuringPlayback) return;
        if (trackerRecorder == null || !trackerRecorder.IsRecording) return;
        
        // 延迟少许时间确保最后几帧数据被记录
        StartCoroutine(DelayedStopRecording(0.5f));
    }

    /// <summary>
    /// 延迟停止录制协程
    /// </summary>
    private IEnumerator DelayedStopRecording(float delaySeconds)
    {
        yield return new WaitForSeconds(delaySeconds);
        
        if (trackerRecorder != null && trackerRecorder.IsRecording)
        {
            trackerRecorder.StopHighFrequencyRecording();
            Debug.Log("<color=cyan>[回放控制器] 🎬 自动录制已停止并保存</color>");
        }
    }

    #endregion
}

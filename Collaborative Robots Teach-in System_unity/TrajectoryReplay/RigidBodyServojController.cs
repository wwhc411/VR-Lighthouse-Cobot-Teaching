using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

/// <summary>
/// 刚体数据Servoj回放控制器
/// 功能：
/// 1. 从CSV文件加载Tracker位姿数据
/// 2. 按指定频率生成并发送servoj命令
/// 3. 提供播放控制（播放/暂停/停止）
/// 4. UI实时显示播放状态
/// 5. 按L键快速加载CSV并开始回放
/// 更新: 2025-12-02
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

    [Header("UI绑定")]
    public Button loadButton;           // 加载按钮
    public Button playButton;           // 播放按钮
    public Button pauseButton;          // 暂停按钮
    public Button stopButton;           // 停止按钮
    public TextMeshProUGUI statusText;  // 状态显示
    public TextMeshProUGUI progressText;// 进度显示
    public Slider progressSlider;       // 进度条

    // -------------------- 运行时数据 -------------------- //
    private RigidBodyCaptureData captureData;       // 加载的捕获数据
    private bool isPlaying = false;                 // 播放状态
    private bool isPaused = false;                  // 暂停状态
    private int currentFrameIndex = 0;              // 当前播放帧索引
    private Coroutine playbackCoroutine;            // 播放协程
    private UTF8Encoding utf8 = new UTF8Encoding(); // UTF-8编码器

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
            PausePlayback();
        }
        else if (Input.GetKeyDown(KeyCode.X))
        {
            StopPlayback();
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
        
        // 如果加载成功，开始播放
        if (captureData != null && captureData.FrameData != null && captureData.FrameData.Count > 0)
        {
            StartPlayback();
        }
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

        // 从暂停恢复
        if (isPaused)
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

        UpdateStatusText("播放中...");
        Debug.Log($"[RigidBodyServojController] 开始播放 - 频率{sendFrequencyHz}Hz");
        Debug.Log($"  坐标转换: {(enableCoordinateTransform ? "启用" : "禁用")}");

        playbackCoroutine = StartCoroutine(PlaybackCoroutine());
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
        if (playbackCoroutine != null)
        {
            StopCoroutine(playbackCoroutine);
            playbackCoroutine = null;
        }

        isPlaying = false;
        isPaused = false;
        currentFrameIndex = 0;

        SendStopCommand();
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

        // 播放循环
        int sentCount = 0;
        
        // 确定回放模式
        bool useTcpMode = useRecordedTcpData && captureData.FrameData.Count > 0 && captureData.FrameData[0].HasTcpData;
        if (useRecordedTcpData && !useTcpMode)
        {
            Debug.LogWarning("[回放] CSV不包含TCP数据，将使用Tracker+坐标转换模式");
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

            // 生成servoj命令 - 根据模式选择不同方法
            string command;
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
            yield return new WaitForSeconds(sendInterval);

            currentFrameIndex++;
        }

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
        
        // 重要：发送停止命令后，必须重置manual_send_active标志
        // 否则控制线程会持续发送stopl命令，导致机械臂无法被其他功能控制
        ur_data_processing.UR_Control_Data.manual_send_active = false;
        
        // 停止命令已发送
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
}

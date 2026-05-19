// ***********************************************************************
// Assembly         : Assembly-CSharp
// Author           : GitHub Copilot
// Created          : 01-22-2026
//
// Last Modified By : GitHub Copilot
// Last Modified On : 01-22-2026
// ***********************************************************************
// <copyright file="DualPoseSystemLogger.cs" company="">
//     Copyright (c) . All rights reserved.
// </copyright>
// <summary>
// 双位姿捕捉系统同步输出器
// 同时采集 Nokov 光学动捕系统和 HTC Vive Tracker 的位姿数据
// 在统一时间戳下输出到控制台和/或CSV文件
// 
// 特性:
// - 统一输出频率
// - 位置单位统一为毫米(mm)
// - 保持各系统原始坐标系
// - 旋转格式可配置(四元数/旋转矢量)
// - 安全的初始化和注销流程，避免退出Play模式时崩溃
// </summary>
// ***********************************************************************

using System;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// 双位姿捕捉系统同步输出器
/// 同时采集 Nokov 和 Vive Tracker 的位姿数据，以统一频率输出
/// </summary>
/// <remarks>
/// 使用负数的 DefaultExecutionOrder 确保此组件在依赖组件之前被清理，
/// 避免在 ViveTrackerPoseLogger 调用 OpenVR.Shutdown() 后访问 VR 系统导致崩溃。
/// </remarks>
[DefaultExecutionOrder(-100)]  // 确保在其他组件之前执行 OnDisable/OnDestroy
public class DualPoseSystemLogger : MonoBehaviour
{
    #region 枚举定义

    /// <summary>
    /// 旋转格式枚举
    /// </summary>
    public enum RotationFormat
    {
        /// <summary>四元数 (X, Y, Z, W)</summary>
        Quaternion,
        /// <summary>旋转矢量/Rodrigues (rx, ry, rz) 弧度</summary>
        RotationVector
    }

    #endregion

    #region Inspector 配置参数

    [Header("Nokov 系统配置")]
    [Tooltip("Nokov StreamingClient 引用，留空则自动查找")]
    public StreamingClient streamingClient;

    [Tooltip("要监控的刚体名称")]
    public string nokovRigidBodyName = "";

    [Tooltip("是否启用 Nokov 系统")]
    public bool nokovEnabled = true;

    [Header("Vive Tracker 系统配置")]
    [Tooltip("ViveTrackerPoseLogger 引用，留空则自动查找")]
    public ViveTrackerPoseLogger viveTrackerLogger;

    [Tooltip("要监控的 Tracker 设备 ID")]
    public uint viveDeviceId = 0;

    [Tooltip("是否启用 Vive Tracker 系统")]
    public bool viveEnabled = true;

    [Header("输出配置")]
    [Tooltip("位姿采样频率 (Hz) - 决定数据采集和文件写入频率")]
    [Range(1f, 120f)]
    public float outputFrequency = 10f;

    [Tooltip("启用控制台日志输出")]
    public bool enableConsoleLog = true;

    [Tooltip("控制台日志输出间隔（秒）- 独立于采样频率，用于控制日志刷屏速度")]
    [Range(0.1f, 10f)]
    public float consoleLogInterval = 1.0f;

    [Tooltip("启用文件日志输出")]
    public bool enableFileLog = false;

    [Tooltip("日志文件名前缀")]
    public string logFileName = "DualPose";

    [Tooltip("日志文件存放目录")]
    public string logDirectory = "Logs";

    [Tooltip("跳过两系统都无效的记录")]
    public bool skipInvalidRecords = false;

    [Header("格式配置")]
    [Tooltip("控制台日志显示的旋转格式")]
    public RotationFormat rotationFormat = RotationFormat.Quaternion;

    [Tooltip("位置小数位数 (mm)")]
    [Range(1, 6)]
    public int positionDecimalPlaces = 2;

    [Tooltip("旋转小数位数 (quat/rad)")]
    [Range(1, 8)]
    public int rotationDecimalPlaces = 6;

    [Tooltip("显示 Unity 时间")]
    public bool showUnityTime = true;

    [Tooltip("使用紧凑格式输出")]
    public bool compactFormat = false;

    [Header("运行状态（只读）")]
    [SerializeField]
    private bool isLogging = false;

    [SerializeField]
    private int sampledFrameCount = 0;

    [SerializeField]
    private string nokovStatus = "未初始化";

    [SerializeField]
    private string viveStatus = "未初始化";

    #endregion

    #region 私有变量

    // 采样控制
    private float sampleInterval;
    private float nextSampleTime;

    // 控制台日志输出控制（独立于采样频率）
    private float nextConsoleLogTime;

    // 文件写入
    private StreamWriter fileWriter;
    private string fullLogPath;
    private readonly object fileLock = new object();

    // 字符串构建
    private StringBuilder stringBuilder;

    // 初始化状态标志
    private bool isInitialized = false;
    private bool isShuttingDown = false;

    // 组件可用性标志
    private bool nokovAvailable = false;
    private bool viveAvailable = false;

    // 格式化字符串缓存
    private string positionFormatString;
    private string rotationFormatString;

    // 缓存的组件引用（用于安全检查）
    private bool hasValidNokovReference = false;
    private bool hasValidViveReference = false;

    #endregion

    #region Unity 生命周期

    /// <summary>
    /// 组件唤醒时的早期初始化
    /// </summary>
    void Awake()
    {
        // 注册 Application.quitting 事件，确保安全退出
        Application.quitting += OnApplicationQuitting;
    }

    /// <summary>
    /// 组件启动时初始化
    /// </summary>
    void Start()
    {
        SafeInitialize();
    }

    /// <summary>
    /// 每帧更新，检查是否需要采样
    /// </summary>
    void Update()
    {
        // 安全检查：如果正在关闭或未初始化，不执行采样
        if (isShuttingDown || !isInitialized)
        {
            return;
        }

        // 关键安全检查：验证依赖组件仍然有效
        // 这是防止退出 Play 模式时崩溃的核心保护
        if (!ValidateDependencies())
        {
            return;
        }

        // 检查是否到达采样时刻
        if (Time.time < nextSampleTime)
        {
            return;
        }

        // 执行同步采样
        SampleBothSystems();

        // 更新下次采样时间
        nextSampleTime = Time.time + sampleInterval;
    }

    /// <summary>
    /// 组件禁用时安全清理
    /// 这是防止崩溃的关键点：在依赖组件被销毁前停止所有操作
    /// </summary>
    void OnDisable()
    {
        // 立即设置关闭标志，阻止 Update 中的任何操作
        isShuttingDown = true;
        isInitialized = false;
        
        // 清除组件引用有效性标志
        hasValidNokovReference = false;
        hasValidViveReference = false;
        
        // 不要在这里将 streamingClient 和 viveTrackerLogger 设为 null
        // 因为 OnEnable 时可能需要重新使用它们
        // 但标记它们为不可用
        nokovAvailable = false;
        viveAvailable = false;
        
        SafeCleanup();
    }

    /// <summary>
    /// 组件启用时重新初始化
    /// </summary>
    void OnEnable()
    {
        // 重置关闭标志
        isShuttingDown = false;
        
        // 如果是首次启用，Start() 会处理初始化
        // 如果是重新启用（比如从 Disable 恢复），需要重新验证依赖
        if (stringBuilder != null)  // 说明之前已经初始化过
        {
            // 重新验证依赖组件
            ValidateAndCacheDependencies();
            
            if (enableFileLog && fileWriter == null)
            {
                SafeInitializeFileWriter();
            }
            
            isInitialized = true;
            isLogging = true;
        }
    }

    /// <summary>
    /// 组件销毁时彻底清理
    /// </summary>
    void OnDestroy()
    {
        // 取消注册事件，防止内存泄漏
        Application.quitting -= OnApplicationQuitting;

        // 执行最终清理
        SafeCleanup();
    }

    /// <summary>
    /// 应用程序退出回调
    /// </summary>
    private void OnApplicationQuitting()
    {
        isShuttingDown = true;
        isInitialized = false;
        hasValidNokovReference = false;
        hasValidViveReference = false;
        nokovAvailable = false;
        viveAvailable = false;
        SafeCleanup();
    }

    #endregion

    #region 依赖验证方法

    /// <summary>
    /// 验证依赖组件是否仍然有效
    /// 这是防止退出 Play 模式时崩溃的核心方法
    /// </summary>
    /// <returns>至少有一个系统可用时返回 true</returns>
    private bool ValidateDependencies()
    {
        // 快速路径：如果正在关闭，直接返回 false
        if (isShuttingDown)
        {
            return false;
        }

        // 检查 Nokov 系统
        // 使用 Unity 的隐式 bool 转换检查组件是否已被销毁
        // 注意：不能使用 == null，因为 Unity 重载了 == 操作符
        // 但在对象被销毁后访问可能导致问题
        if (hasValidNokovReference)
        {
            try
            {
                // 尝试访问组件，如果已销毁会抛出 MissingReferenceException
                if (streamingClient == null || !streamingClient.isActiveAndEnabled)
                {
                    hasValidNokovReference = false;
                    nokovAvailable = false;
                }
            }
            catch
            {
                hasValidNokovReference = false;
                nokovAvailable = false;
            }
        }

        // 检查 Vive 系统
        // 新增：同时检查 IsVRSystemAvailable 属性，确保 OpenVR 运行时仍然可用
        if (hasValidViveReference)
        {
            try
            {
                if (viveTrackerLogger == null || !viveTrackerLogger.isActiveAndEnabled || !viveTrackerLogger.IsVRSystemAvailable)
                {
                    hasValidViveReference = false;
                    viveAvailable = false;
                }
            }
            catch
            {
                hasValidViveReference = false;
                viveAvailable = false;
            }
        }

        // 如果两个系统都不可用，返回 false
        return hasValidNokovReference || hasValidViveReference;
    }

    /// <summary>
    /// 验证并缓存依赖组件引用
    /// </summary>
    private void ValidateAndCacheDependencies()
    {
        // 验证 Nokov
        hasValidNokovReference = false;
        if (nokovEnabled && streamingClient != null)
        {
            try
            {
                if (streamingClient.isActiveAndEnabled)
                {
                    hasValidNokovReference = true;
                    nokovAvailable = true;
                }
            }
            catch
            {
                hasValidNokovReference = false;
                nokovAvailable = false;
            }
        }

        // 验证 Vive
        // 新增：同时检查 IsVRSystemAvailable 属性
        hasValidViveReference = false;
        if (viveEnabled && viveTrackerLogger != null)
        {
            try
            {
                if (viveTrackerLogger.isActiveAndEnabled && viveTrackerLogger.IsVRSystemAvailable)
                {
                    hasValidViveReference = true;
                    viveAvailable = true;
                }
            }
            catch
            {
                hasValidViveReference = false;
                viveAvailable = false;
            }
        }
    }

    #endregion

    #region 初始化方法

    /// <summary>
    /// 安全初始化所有组件
    /// </summary>
    private void SafeInitialize()
    {
        try
        {
            // 初始化内部变量
            stringBuilder = new StringBuilder(1024);
            sampleInterval = 1.0f / Mathf.Max(outputFrequency, 1f);
            nextSampleTime = Time.time;
            nextConsoleLogTime = Time.time;  // 初始化控制台日志输出时间

            // 更新格式化字符串
            UpdateFormatStrings();

            // 初始化 Nokov 系统
            InitializeNokovSystem();

            // 初始化 Vive 系统
            InitializeViveSystem();

            // 初始化文件写入器
            if (enableFileLog)
            {
                SafeInitializeFileWriter();
            }

            isInitialized = true;
            isLogging = true;

            // 输出初始化状态
            Debug.Log($"[DualPoseSystemLogger] 初始化完成\n" +
                      $"  Nokov: {nokovStatus}\n" +
                      $"  Vive: {viveStatus}\n" +
                      $"  采样频率: {outputFrequency} Hz (间隔 {sampleInterval * 1000:F1} ms)\n" +
                      $"  控制台日志: {enableConsoleLog}, 文件日志: {enableFileLog}", this);
        }
        catch (Exception ex)
        {
            Debug.LogError($"[DualPoseSystemLogger] 初始化失败: {ex.Message}\n{ex.StackTrace}", this);
            isInitialized = false;
        }
    }

    /// <summary>
    /// 初始化 Nokov 系统
    /// </summary>
    private void InitializeNokovSystem()
    {
        nokovAvailable = false;

        if (!nokovEnabled)
        {
            nokovStatus = "已禁用";
            return;
        }

        try
        {
            // 自动查找 StreamingClient
            if (streamingClient == null)
            {
                streamingClient = StreamingClient.FindDefaultClient();
            }

            if (streamingClient == null)
            {
                nokovStatus = "未找到 StreamingClient";
                Debug.LogWarning("[DualPoseSystemLogger] 未找到 Nokov StreamingClient，Nokov 系统不可用", this);
                return;
            }

            // 验证刚体名称
            if (string.IsNullOrEmpty(nokovRigidBodyName))
            {
                nokovStatus = "未配置刚体名称";
                Debug.LogWarning("[DualPoseSystemLogger] 未配置 Nokov 刚体名称", this);
                return;
            }

            nokovAvailable = true;
            hasValidNokovReference = true;
            nokovStatus = $"就绪 [{nokovRigidBodyName}]";
        }
        catch (Exception ex)
        {
            nokovStatus = $"初始化异常: {ex.Message}";
            hasValidNokovReference = false;
            Debug.LogError($"[DualPoseSystemLogger] Nokov 初始化异常: {ex.Message}", this);
        }
    }

    /// <summary>
    /// 初始化 Vive Tracker 系统
    /// </summary>
    private void InitializeViveSystem()
    {
        viveAvailable = false;

        if (!viveEnabled)
        {
            viveStatus = "已禁用";
            return;
        }

        try
        {
            // 自动查找 ViveTrackerPoseLogger
            if (viveTrackerLogger == null)
            {
                viveTrackerLogger = FindObjectOfType<ViveTrackerPoseLogger>();
            }

            if (viveTrackerLogger == null)
            {
                viveStatus = "未找到 ViveTrackerPoseLogger";
                Debug.LogWarning("[DualPoseSystemLogger] 未找到 ViveTrackerPoseLogger，Vive 系统不可用", this);
                return;
            }

            viveAvailable = true;
            hasValidViveReference = true;
            viveStatus = $"就绪 [ID:{viveDeviceId}]";
        }
        catch (Exception ex)
        {
            viveStatus = $"初始化异常: {ex.Message}";
            hasValidViveReference = false;
            Debug.LogError($"[DualPoseSystemLogger] Vive 初始化异常: {ex.Message}", this);
        }
    }

    /// <summary>
    /// 安全初始化文件写入器
    /// </summary>
    private void SafeInitializeFileWriter()
    {
        if (!enableFileLog || isShuttingDown)
        {
            return;
        }

        try
        {
            // 确定日志目录
            string baseDir;
#if UNITY_EDITOR
            baseDir = Path.Combine(Application.dataPath, "..", logDirectory);
#else
            baseDir = Path.Combine(Application.persistentDataPath, logDirectory);
#endif

            // 确保目录存在
            if (!Directory.Exists(baseDir))
            {
                Directory.CreateDirectory(baseDir);
            }

            // 生成带时间戳的文件名
            string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string fileName = $"{logFileName}_{timestamp}.csv";
            fullLogPath = Path.Combine(baseDir, fileName);

            // 创建文件写入器
            lock (fileLock)
            {
                fileWriter = new StreamWriter(fullLogPath, false, Encoding.UTF8);
                fileWriter.AutoFlush = true;
            }

            // 写入 CSV 表头
            WriteCSVHeader();

            Debug.Log($"[DualPoseSystemLogger] 日志文件已创建: {fullLogPath}", this);
        }
        catch (Exception ex)
        {
            Debug.LogError($"[DualPoseSystemLogger] 创建日志文件失败: {ex.Message}", this);
            enableFileLog = false;
        }
    }

    /// <summary>
    /// 写入 CSV 文件表头
    /// </summary>
    private void WriteCSVHeader()
    {
        if (fileWriter == null) return;

        // CSV 表头：包含四元数和旋转矢量两种格式
        string header = "Timestamp,UnityTime," +
                        "NokovName,NokovValid,NokovPosX_mm,NokovPosY_mm,NokovPosZ_mm," +
                        "NokovQuatX,NokovQuatY,NokovQuatZ,NokovQuatW," +
                        "NokovRotVecX_rad,NokovRotVecY_rad,NokovRotVecZ_rad," +
                        "ViveID,ViveValid,VivePosX_mm,VivePosY_mm,VivePosZ_mm," +
                        "ViveQuatX,ViveQuatY,ViveQuatZ,ViveQuatW," +
                        "ViveRotVecX_rad,ViveRotVecY_rad,ViveRotVecZ_rad," +
                        "CoordSys";

        try
        {
            lock (fileLock)
            {
                fileWriter?.WriteLine(header);
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"[DualPoseSystemLogger] 写入 CSV 表头失败: {ex.Message}", this);
        }
    }

    /// <summary>
    /// 更新格式化字符串
    /// </summary>
    private void UpdateFormatStrings()
    {
        positionFormatString = $"F{positionDecimalPlaces}";
        rotationFormatString = $"F{rotationDecimalPlaces}";
    }

    #endregion

    #region 清理方法

    /// <summary>
    /// 安全清理所有资源
    /// </summary>
    private void SafeCleanup()
    {
        isLogging = false;

        // 安全关闭文件写入器
        SafeCloseFileWriter();
    }

    /// <summary>
    /// 安全关闭文件写入器
    /// </summary>
    private void SafeCloseFileWriter()
    {
        if (fileWriter == null) return;

        try
        {
            lock (fileLock)
            {
                if (fileWriter != null)
                {
                    fileWriter.Flush();
                    fileWriter.Close();
                    fileWriter.Dispose();
                    fileWriter = null;

                    if (!string.IsNullOrEmpty(fullLogPath))
                    {
                        Debug.Log($"[DualPoseSystemLogger] 日志文件已保存: {fullLogPath}", this);
                    }
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"[DualPoseSystemLogger] 关闭日志文件失败: {ex.Message}", this);
        }
    }

    #endregion

    #region 核心采样方法

    /// <summary>
    /// 同时采样两个系统的位姿数据
    /// </summary>
    private void SampleBothSystems()
    {
        // 安全检查
        if (isShuttingDown)
        {
            return;
        }

        // 记录统一时间戳
        string timestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff");
        float unityTime = Time.time;

        // 采样结果变量
        bool nokovValid = false;
        Vector3 nokovPositionMm = Vector3.zero;
        Quaternion nokovRotation = Quaternion.identity;
        Vector3 nokovRotVec = Vector3.zero;

        bool viveValid = false;
        Vector3 vivePositionMm = Vector3.zero;
        Quaternion viveRotation = Quaternion.identity;
        Vector3 viveRotVec = Vector3.zero;

        // ========== 采样 Nokov 系统 ==========
        // 使用缓存的有效性标志，避免在组件被销毁后访问
        if (nokovEnabled && hasValidNokovReference && nokovAvailable)
        {
            try
            {
                // 再次检查组件有效性（双重保护）
                if (streamingClient != null && streamingClient.isActiveAndEnabled && !string.IsNullOrEmpty(nokovRigidBodyName))
                {
                    NokovRigidBodyState rbState = streamingClient.GetLatestRigidBodyState(nokovRigidBodyName);
                    if (rbState != null)
                    {
                        // Nokov 位置单位是米，转换为毫米
                        nokovPositionMm = rbState.Pose.Position * 1000f;
                        nokovRotation = rbState.Pose.Orientation;
                        nokovRotVec = QuaternionToRotationVector(nokovRotation);
                        nokovValid = true;
                    }
                }
            }
            catch (MissingReferenceException)
            {
                // 组件已被销毁，标记为不可用
                hasValidNokovReference = false;
                nokovAvailable = false;
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[DualPoseSystemLogger] Nokov 采样异常: {ex.Message}");
            }
        }

        // ========== 采样 Vive 系统 ==========
        // 使用缓存的有效性标志，避免在组件被销毁后访问
        // 这是防止 OpenVR.Shutdown() 后访问 VR 系统导致崩溃的关键保护
        if (viveEnabled && hasValidViveReference && viveAvailable)
        {
            try
            {
                // 再次检查组件有效性（双重保护）
                // 新增：检查 ViveTrackerPoseLogger 的 IsVRSystemAvailable 属性
                // 这可以在 OpenVR 运行时关闭时立即返回，避免访问无效的 VR 系统
                if (viveTrackerLogger != null && viveTrackerLogger.isActiveAndEnabled && viveTrackerLogger.IsVRSystemAvailable)
                {
                    // GetTrackerPoseForCalibration 返回的位置已经是毫米
                    if (viveTrackerLogger.GetTrackerPoseForCalibration(viveDeviceId, out Vector3 posMm, out Quaternion rot))
                    {
                        vivePositionMm = posMm;
                        viveRotation = rot;
                        viveRotVec = QuaternionToRotationVector(viveRotation);
                        viveValid = true;
                    }
                }
            }
            catch (MissingReferenceException)
            {
                // 组件已被销毁，标记为不可用
                hasValidViveReference = false;
                viveAvailable = false;
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[DualPoseSystemLogger] Vive 采样异常: {ex.Message}");
            }
        }

        // ========== 检查数据有效性 ==========
        if (!nokovValid && !viveValid && skipInvalidRecords)
        {
            return; // 跳过两系统都无效的记录
        }

        // ========== 格式化输出 ==========
        sampledFrameCount++;

        // 控制台输出（受 consoleLogInterval 控制，独立于采样频率）
        if (enableConsoleLog && Time.time >= nextConsoleLogTime)
        {
            OutputToConsole(timestamp, unityTime,
                           nokovValid, nokovPositionMm, nokovRotation, nokovRotVec,
                           viveValid, vivePositionMm, viveRotation, viveRotVec);
            nextConsoleLogTime = Time.time + consoleLogInterval;
        }

        // 文件输出
        if (enableFileLog && fileWriter != null && !isShuttingDown)
        {
            OutputToCSV(timestamp, unityTime,
                       nokovValid, nokovPositionMm, nokovRotation, nokovRotVec,
                       viveValid, vivePositionMm, viveRotation, viveRotVec);
        }
    }

    #endregion

    #region 输出方法

    /// <summary>
    /// 输出到控制台
    /// </summary>
    private void OutputToConsole(string timestamp, float unityTime,
                                  bool nokovValid, Vector3 nokovPos, Quaternion nokovRot, Vector3 nokovRotVec,
                                  bool viveValid, Vector3 vivePos, Quaternion viveRot, Vector3 viveRotVec)
    {
        stringBuilder.Clear();

        if (compactFormat)
        {
            // 紧凑格式
            stringBuilder.Append($"[DualPose] t={unityTime.ToString("F2")}s | ");
            
            // Nokov
            if (nokovValid)
            {
                stringBuilder.Append($"Nokov: ({nokovPos.x.ToString(positionFormatString)},{nokovPos.y.ToString(positionFormatString)},{nokovPos.z.ToString(positionFormatString)})mm ");
                if (rotationFormat == RotationFormat.Quaternion)
                {
                    stringBuilder.Append($"Q({nokovRot.x.ToString(rotationFormatString)},{nokovRot.y.ToString(rotationFormatString)},{nokovRot.z.ToString(rotationFormatString)},{nokovRot.w.ToString(rotationFormatString)})");
                }
                else
                {
                    stringBuilder.Append($"RV({nokovRotVec.x.ToString(rotationFormatString)},{nokovRotVec.y.ToString(rotationFormatString)},{nokovRotVec.z.ToString(rotationFormatString)})rad");
                }
            }
            else
            {
                stringBuilder.Append("Nokov: N/A");
            }

            stringBuilder.Append(" | ");

            // Vive
            if (viveValid)
            {
                stringBuilder.Append($"Vive: ({vivePos.x.ToString(positionFormatString)},{vivePos.y.ToString(positionFormatString)},{vivePos.z.ToString(positionFormatString)})mm ");
                if (rotationFormat == RotationFormat.Quaternion)
                {
                    stringBuilder.Append($"Q({viveRot.x.ToString(rotationFormatString)},{viveRot.y.ToString(rotationFormatString)},{viveRot.z.ToString(rotationFormatString)},{viveRot.w.ToString(rotationFormatString)})");
                }
                else
                {
                    stringBuilder.Append($"RV({viveRotVec.x.ToString(rotationFormatString)},{viveRotVec.y.ToString(rotationFormatString)},{viveRotVec.z.ToString(rotationFormatString)})rad");
                }
            }
            else
            {
                stringBuilder.Append("Vive: N/A");
            }
        }
        else
        {
            // 详细格式
            stringBuilder.AppendLine($"=== Dual Pose Snapshot [{timestamp}] Unity: {unityTime.ToString("F2")}s ===");

            // Nokov 部分
            stringBuilder.AppendLine($"Nokov [{nokovRigidBodyName}] (原始Nokov坐标系):");
            if (nokovValid)
            {
                stringBuilder.AppendLine($"  Position: ({nokovPos.x.ToString(positionFormatString)}, {nokovPos.y.ToString(positionFormatString)}, {nokovPos.z.ToString(positionFormatString)}) mm");
                if (rotationFormat == RotationFormat.Quaternion)
                {
                    stringBuilder.AppendLine($"  Rotation: ({nokovRot.x.ToString(rotationFormatString)}, {nokovRot.y.ToString(rotationFormatString)}, {nokovRot.z.ToString(rotationFormatString)}, {nokovRot.w.ToString(rotationFormatString)}) [Quat XYZW]");
                }
                else
                {
                    stringBuilder.AppendLine($"  Rotation: ({nokovRotVec.x.ToString(rotationFormatString)}, {nokovRotVec.y.ToString(rotationFormatString)}, {nokovRotVec.z.ToString(rotationFormatString)}) rad [Rodrigues]");
                }
                stringBuilder.AppendLine("  Status: ✓ Valid");
            }
            else
            {
                stringBuilder.AppendLine("  Status: ✗ Invalid / Unavailable");
            }

            stringBuilder.AppendLine();

            // Vive 部分
            stringBuilder.AppendLine($"Vive [ID:{viveDeviceId}] (原始SteamVR坐标系):");
            if (viveValid)
            {
                stringBuilder.AppendLine($"  Position: ({vivePos.x.ToString(positionFormatString)}, {vivePos.y.ToString(positionFormatString)}, {vivePos.z.ToString(positionFormatString)}) mm");
                if (rotationFormat == RotationFormat.Quaternion)
                {
                    stringBuilder.AppendLine($"  Rotation: ({viveRot.x.ToString(rotationFormatString)}, {viveRot.y.ToString(rotationFormatString)}, {viveRot.z.ToString(rotationFormatString)}, {viveRot.w.ToString(rotationFormatString)}) [Quat XYZW]");
                }
                else
                {
                    stringBuilder.AppendLine($"  Rotation: ({viveRotVec.x.ToString(rotationFormatString)}, {viveRotVec.y.ToString(rotationFormatString)}, {viveRotVec.z.ToString(rotationFormatString)}) rad [Rodrigues]");
                }
                stringBuilder.AppendLine("  Status: ✓ Valid");
            }
            else
            {
                stringBuilder.AppendLine("  Status: ✗ Invalid / Unavailable");
            }

            stringBuilder.Append("==================================================================");
        }

        Debug.Log(stringBuilder.ToString());
    }

    /// <summary>
    /// 输出到 CSV 文件
    /// </summary>
    private void OutputToCSV(string timestamp, float unityTime,
                              bool nokovValid, Vector3 nokovPos, Quaternion nokovRot, Vector3 nokovRotVec,
                              bool viveValid, Vector3 vivePos, Quaternion viveRot, Vector3 viveRotVec)
    {
        if (isShuttingDown || fileWriter == null) return;

        try
        {
            stringBuilder.Clear();

            // 时间戳
            stringBuilder.Append(timestamp);
            stringBuilder.Append(',');
            stringBuilder.Append(unityTime.ToString("F3"));
            stringBuilder.Append(',');

            // Nokov 数据
            stringBuilder.Append(nokovRigidBodyName);
            stringBuilder.Append(',');
            stringBuilder.Append(nokovValid ? "1" : "0");
            stringBuilder.Append(',');

            if (nokovValid)
            {
                // 位置 (mm)
                stringBuilder.Append(nokovPos.x.ToString(positionFormatString));
                stringBuilder.Append(',');
                stringBuilder.Append(nokovPos.y.ToString(positionFormatString));
                stringBuilder.Append(',');
                stringBuilder.Append(nokovPos.z.ToString(positionFormatString));
                stringBuilder.Append(',');

                // 四元数
                stringBuilder.Append(nokovRot.x.ToString(rotationFormatString));
                stringBuilder.Append(',');
                stringBuilder.Append(nokovRot.y.ToString(rotationFormatString));
                stringBuilder.Append(',');
                stringBuilder.Append(nokovRot.z.ToString(rotationFormatString));
                stringBuilder.Append(',');
                stringBuilder.Append(nokovRot.w.ToString(rotationFormatString));
                stringBuilder.Append(',');

                // 旋转矢量 (rad)
                stringBuilder.Append(nokovRotVec.x.ToString(rotationFormatString));
                stringBuilder.Append(',');
                stringBuilder.Append(nokovRotVec.y.ToString(rotationFormatString));
                stringBuilder.Append(',');
                stringBuilder.Append(nokovRotVec.z.ToString(rotationFormatString));
            }
            else
            {
                // 无效数据填充
                stringBuilder.Append("0,0,0,0,0,0,0,0,0,0");
            }
            stringBuilder.Append(',');

            // Vive 数据
            stringBuilder.Append(viveDeviceId);
            stringBuilder.Append(',');
            stringBuilder.Append(viveValid ? "1" : "0");
            stringBuilder.Append(',');

            if (viveValid)
            {
                // 位置 (mm)
                stringBuilder.Append(vivePos.x.ToString(positionFormatString));
                stringBuilder.Append(',');
                stringBuilder.Append(vivePos.y.ToString(positionFormatString));
                stringBuilder.Append(',');
                stringBuilder.Append(vivePos.z.ToString(positionFormatString));
                stringBuilder.Append(',');

                // 四元数
                stringBuilder.Append(viveRot.x.ToString(rotationFormatString));
                stringBuilder.Append(',');
                stringBuilder.Append(viveRot.y.ToString(rotationFormatString));
                stringBuilder.Append(',');
                stringBuilder.Append(viveRot.z.ToString(rotationFormatString));
                stringBuilder.Append(',');
                stringBuilder.Append(viveRot.w.ToString(rotationFormatString));
                stringBuilder.Append(',');

                // 旋转矢量 (rad)
                stringBuilder.Append(viveRotVec.x.ToString(rotationFormatString));
                stringBuilder.Append(',');
                stringBuilder.Append(viveRotVec.y.ToString(rotationFormatString));
                stringBuilder.Append(',');
                stringBuilder.Append(viveRotVec.z.ToString(rotationFormatString));
            }
            else
            {
                // 无效数据填充
                stringBuilder.Append("0,0,0,0,0,0,0,0,0,0");
            }
            stringBuilder.Append(',');

            // 坐标系标记
            string coordSys = "";
            if (nokovValid && viveValid)
            {
                coordSys = "Nokov_Raw+SteamVR_Raw";
            }
            else if (nokovValid)
            {
                coordSys = "Nokov_Raw+SteamVR_Invalid";
            }
            else if (viveValid)
            {
                coordSys = "Nokov_Invalid+SteamVR_Raw";
            }
            else
            {
                coordSys = "Both_Invalid";
            }
            stringBuilder.Append(coordSys);

            // 写入文件
            lock (fileLock)
            {
                fileWriter?.WriteLine(stringBuilder.ToString());
            }
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[DualPoseSystemLogger] CSV 写入异常: {ex.Message}");
        }
    }

    #endregion

    #region 旋转转换方法

    /// <summary>
    /// 四元数转旋转矢量 (Rodrigues 表示，弧度)
    /// 使用与 SteamVrUrCoordinateConverter 相同的算法
    /// </summary>
    private Vector3 QuaternionToRotationVector(Quaternion q)
    {
        // 1. 归一化
        q = NormalizeQuaternion(q);

        // 2. 规范化符号: 强制 q.w >= 0，确保角度在 [0, π] 范围
        if (q.w < 0f)
        {
            q.x = -q.x;
            q.y = -q.y;
            q.z = -q.z;
            q.w = -q.w;
        }

        // 3. 限制 qw 在 [0, 1] 范围内
        float wClamped = Mathf.Clamp(q.w, 0f, 1f);

        // 4. 计算角度: θ = 2 * arccos(qw)
        float angle = 2f * Mathf.Acos(wClamped);

        // 5. 特殊情况: 角度接近 0（无旋转）
        if (angle < 1e-6f)
        {
            return Vector3.zero;
        }

        // 6. 特殊情况: 角度接近 π（180度）
        if (angle > Mathf.PI - 1e-4f)
        {
            Vector3 axis180 = new Vector3(q.x, q.y, q.z);
            float axisLen = axis180.magnitude;
            if (axisLen > 1e-6f)
            {
                axis180 = axis180 / axisLen;
                return axis180 * angle;
            }
            return Vector3.zero;
        }

        // 7. 一般情况
        float halfAngle = angle * 0.5f;
        float s = Mathf.Sin(halfAngle);
        Vector3 axis = new Vector3(q.x / s, q.y / s, q.z / s);
        return axis * angle;
    }

    /// <summary>
    /// 旋转矢量转四元数
    /// </summary>
    private Quaternion RotationVectorToQuaternion(Vector3 rotVec)
    {
        float angle = rotVec.magnitude;
        if (angle < 1e-8f)
        {
            return Quaternion.identity;
        }

        Vector3 axis = rotVec / angle;
        float halfAngle = angle * 0.5f;
        float s = Mathf.Sin(halfAngle);
        float c = Mathf.Cos(halfAngle);

        return new Quaternion(axis.x * s, axis.y * s, axis.z * s, c);
    }

    /// <summary>
    /// 归一化四元数
    /// </summary>
    private Quaternion NormalizeQuaternion(Quaternion q)
    {
        float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (mag < 1e-12f) return Quaternion.identity;
        float inv = 1f / mag;
        return new Quaternion(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
    }

    #endregion

    #region 公开方法

    /// <summary>
    /// 立即采样一次（供外部调用或 Inspector 按钮）
    /// </summary>
    [ContextMenu("立即采样")]
    public void SampleNow()
    {
        if (!isInitialized || isShuttingDown)
        {
            Debug.LogWarning("[DualPoseSystemLogger] 系统未初始化或正在关闭");
            return;
        }

        SampleBothSystems();
    }

    /// <summary>
    /// 打开日志文件目录
    /// </summary>
    [ContextMenu("打开日志目录")]
    public void OpenLogDirectory()
    {
#if UNITY_EDITOR
        string baseDir = Path.Combine(Application.dataPath, "..", logDirectory);
#else
        string baseDir = Path.Combine(Application.persistentDataPath, logDirectory);
#endif

        if (Directory.Exists(baseDir))
        {
            System.Diagnostics.Process.Start("explorer.exe", baseDir.Replace("/", "\\"));
        }
        else
        {
            Debug.LogWarning($"[DualPoseSystemLogger] 日志目录不存在: {baseDir}");
        }
    }

    /// <summary>
    /// 刷新系统状态
    /// </summary>
    [ContextMenu("刷新系统状态")]
    public void RefreshStatus()
    {
        InitializeNokovSystem();
        InitializeViveSystem();
        Debug.Log($"[DualPoseSystemLogger] 状态已刷新\n  Nokov: {nokovStatus}\n  Vive: {viveStatus}");
    }

    /// <summary>
    /// 获取采样统计信息
    /// </summary>
    public string GetStatistics()
    {
        return $"已采样 {sampledFrameCount} 帧, Nokov: {nokovStatus}, Vive: {viveStatus}";
    }

    #endregion
}

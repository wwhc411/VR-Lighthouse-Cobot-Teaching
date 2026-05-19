// ***********************************************************************
// Assembly         : Assembly-CSharp
// Author           : GitHub Copilot
// Created          : 01-21-2026
//
// Last Modified By : GitHub Copilot
// Last Modified On : 01-21-2026
// ***********************************************************************
// <copyright file="RigidBodyPoseLogger.cs" company="Nokov">
//     Copyright (c) Nokov. All rights reserved.
// </copyright>
// <summary>刚体位姿日志打印组件 - 用于输出Nokov刚体的位置和姿态数据</summary>
// ***********************************************************************

using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Text;
using UnityEngine;

/// <summary>
/// 刚体位姿日志打印组件
/// 用于实时输出Nokov软件捕获的刚体位置及姿态数据
/// 支持控制台输出和文件记录
/// </summary>
public class RigidBodyPoseLogger : MonoBehaviour
{
    #region 配置参数

    [Header("数据源配置")]
    [Tooltip("Nokov数据流客户端引用，留空则自动查找场景中的StreamingClient")]
    public StreamingClient streamingClient;

    [Tooltip("要监控的刚体名称列表")]
    public string[] targetRigidBodyNames = new string[] { };

    [Tooltip("是否监控所有已知刚体（忽略targetRigidBodyNames列表）")]
    public bool logAllRigidBodies = true;

    [Header("日志输出配置")]
    [Tooltip("日志输出间隔(秒)，0表示每帧输出（注意性能影响）")]
    [Range(0f, 5f)]
    public float logInterval = 0.5f;

    [Tooltip("是否输出到Unity控制台")]
    public bool enableConsoleLog = true;

    [Tooltip("是否输出到文件")]
    public bool enableFileLog = false;

    [Tooltip("日志文件名前缀（自动添加时间戳后缀）")]
    public string logFileName = "RigidBodyPose";

    [Tooltip("日志文件存放目录（相对于项目根目录或persistentDataPath）")]
    public string logDirectory = "Logs";

    [Header("格式配置")]
    [Tooltip("是否显示欧拉角（否则显示四元数）")]
    public bool showEulerAngles = true;

    [Tooltip("位置数据小数位数")]
    [Range(1, 6)]
    public int positionDecimalPlaces = 4;

    [Tooltip("角度/四元数数据小数位数")]
    [Range(1, 6)]
    public int rotationDecimalPlaces = 4;

    [Tooltip("显示时间戳")]
    public bool showTimestamp = true;

    [Tooltip("控制台日志使用紧凑格式（单行显示）")]
    public bool compactConsoleFormat = false;

    [Header("运行状态（只读）")]
    [Tooltip("当前是否正在记录")]
    [SerializeField]
    private bool isLogging = false;

    [Tooltip("已记录的帧数")]
    [SerializeField]
    private int loggedFrameCount = 0;

    [Tooltip("检测到的刚体数量")]
    [SerializeField]
    private int detectedRigidBodyCount = 0;

    #endregion

    #region 私有变量

    private float lastLogTime;
    private StreamWriter fileWriter;
    private string fullLogPath;
    private StringBuilder stringBuilder;
    private bool isInitialized = false;
    private List<string> cachedRigidBodyNames;
    private object fileLock = new object();

    #endregion

    #region Unity生命周期

    /// <summary>
    /// 组件启动时初始化
    /// </summary>
    void Start()
    {
        Initialize();
    }

    /// <summary>
    /// 每帧更新，检查是否需要输出日志
    /// </summary>
    void Update()
    {
        if (!isInitialized || streamingClient == null)
        {
            return;
        }

        // 检查日志输出间隔
        float currentTime = Time.time;
        if (currentTime - lastLogTime < logInterval)
        {
            return;
        }

        lastLogTime = currentTime;
        LogAllTargetRigidBodies();
    }

    /// <summary>
    /// 组件销毁时清理资源
    /// </summary>
    void OnDestroy()
    {
        CloseFileWriter();
    }

    /// <summary>
    /// 组件禁用时关闭文件写入
    /// </summary>
    void OnDisable()
    {
        CloseFileWriter();
        isLogging = false;
    }

    /// <summary>
    /// 组件启用时重新初始化
    /// </summary>
    void OnEnable()
    {
        if (isInitialized)
        {
            InitializeFileWriter();
            isLogging = true;
        }
    }

    #endregion

    #region 初始化方法

    /// <summary>
    /// 初始化日志记录器
    /// </summary>
    private void Initialize()
    {
        stringBuilder = new StringBuilder(512);
        cachedRigidBodyNames = new List<string>();

        // 查找StreamingClient
        if (streamingClient == null)
        {
            streamingClient = StreamingClient.FindDefaultClient();

            if (streamingClient == null)
            {
                Debug.LogError($"[RigidBodyPoseLogger] 未找到StreamingClient组件，请确保场景中存在Nokov Client对象。组件已禁用。", this);
                enabled = false;
                return;
            }
            else
            {
                Debug.Log($"[RigidBodyPoseLogger] 自动找到StreamingClient: {streamingClient.gameObject.name}", this);
            }
        }

        // 初始化文件写入器
        if (enableFileLog)
        {
            InitializeFileWriter();
        }

        lastLogTime = Time.time;
        isInitialized = true;
        isLogging = true;

        Debug.Log($"[RigidBodyPoseLogger] 初始化完成。控制台日志: {enableConsoleLog}, 文件日志: {enableFileLog}, 间隔: {logInterval}秒", this);
    }

    /// <summary>
    /// 初始化文件写入器
    /// </summary>
    private void InitializeFileWriter()
    {
        if (!enableFileLog)
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
            fileWriter = new StreamWriter(fullLogPath, false, Encoding.UTF8);
            fileWriter.AutoFlush = true;

            // 写入CSV表头
            WriteCSVHeader();

            Debug.Log($"[RigidBodyPoseLogger] 日志文件已创建: {fullLogPath}", this);
        }
        catch (Exception ex)
        {
            Debug.LogError($"[RigidBodyPoseLogger] 创建日志文件失败: {ex.Message}", this);
            enableFileLog = false;
        }
    }

    /// <summary>
    /// 写入CSV文件表头
    /// </summary>
    private void WriteCSVHeader()
    {
        if (fileWriter == null) return;

        string header = "Timestamp,FrameID,RigidBodyName,PosX,PosY,PosZ,QuatX,QuatY,QuatZ,QuatW,EulerX,EulerY,EulerZ";
        lock (fileLock)
        {
            fileWriter.WriteLine(header);
        }
    }

    /// <summary>
    /// 关闭文件写入器
    /// </summary>
    private void CloseFileWriter()
    {
        if (fileWriter != null)
        {
            try
            {
                lock (fileLock)
                {
                    fileWriter.Flush();
                    fileWriter.Close();
                    fileWriter.Dispose();
                    fileWriter = null;
                }
                Debug.Log($"[RigidBodyPoseLogger] 日志文件已保存: {fullLogPath}", this);
            }
            catch (Exception ex)
            {
                Debug.LogError($"[RigidBodyPoseLogger] 关闭日志文件失败: {ex.Message}", this);
            }
        }
    }

    #endregion

    #region 日志记录方法

    /// <summary>
    /// 记录所有目标刚体的位姿
    /// </summary>
    private void LogAllTargetRigidBodies()
    {
        if (logAllRigidBodies)
        {
            // 刷新刚体名称缓存（如果需要）
            RefreshRigidBodyNameCache();

            foreach (string rbName in cachedRigidBodyNames)
            {
                LogSingleRigidBody(rbName);
            }
        }
        else
        {
            // 只记录指定的刚体
            foreach (string rbName in targetRigidBodyNames)
            {
                if (!string.IsNullOrEmpty(rbName))
                {
                    LogSingleRigidBody(rbName);
                }
            }
        }
    }

    /// <summary>
    /// 刷新刚体名称缓存
    /// </summary>
    private void RefreshRigidBodyNameCache()
    {
        // 检查是否需要刷新（当Nokov端刚体定义变化时）
        if (streamingClient.GetNeedRefreshRigid())
        {
            cachedRigidBodyNames.Clear();
            streamingClient.SetNeedRefreshRigid();
        }

        // 如果缓存为空，尝试从定义中获取刚体名称
        if (cachedRigidBodyNames.Count == 0)
        {
            // 使用反射获取StreamingClient中的刚体定义列表
            var rigidBodyDefsField = streamingClient.GetType().GetField("m_rigidBodyDefinitions", 
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            
            if (rigidBodyDefsField != null)
            {
                var rigidBodyDefs = rigidBodyDefsField.GetValue(streamingClient) as System.Collections.IList;
                
                if (rigidBodyDefs != null && rigidBodyDefs.Count > 0)
                {
                    foreach (var rbDef in rigidBodyDefs)
                    {
                        // 获取刚体定义的Name属性
                        var nameProperty = rbDef.GetType().GetProperty("Name");
                        if (nameProperty != null)
                        {
                            string rbName = nameProperty.GetValue(rbDef) as string;
                            if (!string.IsNullOrEmpty(rbName) && !cachedRigidBodyNames.Contains(rbName))
                            {
                                cachedRigidBodyNames.Add(rbName);
                            }
                        }
                    }
                    
                    if (cachedRigidBodyNames.Count > 0)
                    {
                        Debug.Log($"[RigidBodyPoseLogger] 检测到 {cachedRigidBodyNames.Count} 个刚体: {string.Join(", ", cachedRigidBodyNames)}", this);
                    }
                }
            }
            
            // 如果反射失败，回退到使用targetRigidBodyNames
            if (cachedRigidBodyNames.Count == 0 && targetRigidBodyNames != null && targetRigidBodyNames.Length > 0)
            {
                foreach (string name in targetRigidBodyNames)
                {
                    if (!string.IsNullOrEmpty(name) && !cachedRigidBodyNames.Contains(name))
                    {
                        cachedRigidBodyNames.Add(name);
                    }
                }
                
                Debug.LogWarning($"[RigidBodyPoseLogger] 无法自动检测刚体列表，使用手动配置的刚体名称: {string.Join(", ", cachedRigidBodyNames)}", this);
            }
            
            // 如果还是没有刚体，输出警告
            if (cachedRigidBodyNames.Count == 0)
            {
                Debug.LogWarning("[RigidBodyPoseLogger] 未检测到任何刚体！请确保：\n" +
                    "1. Nokov服务器已连接\n" +
                    "2. Nokov软件中已创建并激活刚体\n" +
                    "3. 或在Inspector中手动填写刚体名称", this);
            }
        }

        detectedRigidBodyCount = cachedRigidBodyNames.Count;
    }

    /// <summary>
    /// 记录单个刚体的位姿数据
    /// </summary>
    /// <param name="rigidBodyName">刚体名称</param>
    private void LogSingleRigidBody(string rigidBodyName)
    {
        // 获取刚体状态
        NokovRigidBodyState rbState = streamingClient.GetLatestRigidBodyState(rigidBodyName);

        if (rbState == null || rbState.Pose == null)
        {
            // 刚体不存在或无数据，静默跳过（避免日志刷屏）
            return;
        }

        loggedFrameCount++;

        // 提取位姿数据
        Vector3 position = rbState.Pose.Position;
        Quaternion orientation = rbState.Pose.Orientation;
        Vector3 eulerAngles = orientation.eulerAngles;

        // 获取当前时间戳
        string timestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff");

        // 输出到控制台
        if (enableConsoleLog)
        {
            LogToConsole(rigidBodyName, position, orientation, eulerAngles, timestamp);
        }

        // 输出到文件
        if (enableFileLog && fileWriter != null)
        {
            LogToFile(rigidBodyName, position, orientation, eulerAngles, timestamp);
        }
    }

    /// <summary>
    /// 输出到控制台
    /// </summary>
    private void LogToConsole(string rigidBodyName, Vector3 position, Quaternion orientation, Vector3 eulerAngles, string timestamp)
    {
        stringBuilder.Clear();

        string posFormat = $"F{positionDecimalPlaces}";
        string rotFormat = $"F{rotationDecimalPlaces}";

        if (compactConsoleFormat)
        {
            // 紧凑格式（单行）
            stringBuilder.Append($"[Nokov] {rigidBodyName}: ");
            stringBuilder.Append($"Pos({position.x.ToString(posFormat)}, {position.y.ToString(posFormat)}, {position.z.ToString(posFormat)}) ");

            if (showEulerAngles)
            {
                stringBuilder.Append($"Rot({eulerAngles.x.ToString(rotFormat)}°, {eulerAngles.y.ToString(rotFormat)}°, {eulerAngles.z.ToString(rotFormat)}°)");
            }
            else
            {
                stringBuilder.Append($"Quat({orientation.x.ToString(rotFormat)}, {orientation.y.ToString(rotFormat)}, {orientation.z.ToString(rotFormat)}, {orientation.w.ToString(rotFormat)})");
            }

            if (showTimestamp)
            {
                stringBuilder.Append($" @ {timestamp}");
            }
        }
        else
        {
            // 详细格式（多行）
            stringBuilder.AppendLine($"[Nokov] RigidBody \"{rigidBodyName}\" Pose:");
            stringBuilder.AppendLine($"  Position: ({position.x.ToString(posFormat)}, {position.y.ToString(posFormat)}, {position.z.ToString(posFormat)}) m");

            if (showEulerAngles)
            {
                stringBuilder.AppendLine($"  Rotation: ({eulerAngles.x.ToString(rotFormat)}°, {eulerAngles.y.ToString(rotFormat)}°, {eulerAngles.z.ToString(rotFormat)}°) [Euler XYZ]");
            }
            else
            {
                stringBuilder.AppendLine($"  Rotation: ({orientation.x.ToString(rotFormat)}, {orientation.y.ToString(rotFormat)}, {orientation.z.ToString(rotFormat)}, {orientation.w.ToString(rotFormat)}) [Quaternion XYZW]");
            }

            if (showTimestamp)
            {
                stringBuilder.Append($"  Timestamp: {timestamp}");
            }
        }

        Debug.Log(stringBuilder.ToString(), this);
    }

    /// <summary>
    /// 输出到CSV文件
    /// </summary>
    private void LogToFile(string rigidBodyName, Vector3 position, Quaternion orientation, Vector3 eulerAngles, string timestamp)
    {
        try
        {
            stringBuilder.Clear();

            string posFormat = $"F{positionDecimalPlaces}";
            string rotFormat = $"F{rotationDecimalPlaces}";

            // CSV格式: Timestamp,FrameID,RigidBodyName,PosX,PosY,PosZ,QuatX,QuatY,QuatZ,QuatW,EulerX,EulerY,EulerZ
            stringBuilder.Append(timestamp);
            stringBuilder.Append(",");
            stringBuilder.Append(loggedFrameCount);
            stringBuilder.Append(",");
            stringBuilder.Append(rigidBodyName);
            stringBuilder.Append(",");
            stringBuilder.Append(position.x.ToString(posFormat));
            stringBuilder.Append(",");
            stringBuilder.Append(position.y.ToString(posFormat));
            stringBuilder.Append(",");
            stringBuilder.Append(position.z.ToString(posFormat));
            stringBuilder.Append(",");
            stringBuilder.Append(orientation.x.ToString(rotFormat));
            stringBuilder.Append(",");
            stringBuilder.Append(orientation.y.ToString(rotFormat));
            stringBuilder.Append(",");
            stringBuilder.Append(orientation.z.ToString(rotFormat));
            stringBuilder.Append(",");
            stringBuilder.Append(orientation.w.ToString(rotFormat));
            stringBuilder.Append(",");
            stringBuilder.Append(eulerAngles.x.ToString(rotFormat));
            stringBuilder.Append(",");
            stringBuilder.Append(eulerAngles.y.ToString(rotFormat));
            stringBuilder.Append(",");
            stringBuilder.Append(eulerAngles.z.ToString(rotFormat));

            lock (fileLock)
            {
                fileWriter?.WriteLine(stringBuilder.ToString());
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"[RigidBodyPoseLogger] 写入日志文件失败: {ex.Message}", this);
        }
    }

    #endregion

    #region 公共接口

    /// <summary>
    /// 手动触发一次日志记录
    /// </summary>
    public void LogNow()
    {
        if (isInitialized && streamingClient != null)
        {
            LogAllTargetRigidBodies();
        }
    }

    /// <summary>
    /// 添加要监控的刚体名称
    /// </summary>
    /// <param name="rigidBodyName">刚体名称</param>
    public void AddTargetRigidBody(string rigidBodyName)
    {
        if (string.IsNullOrEmpty(rigidBodyName)) return;

        List<string> nameList = new List<string>(targetRigidBodyNames);
        if (!nameList.Contains(rigidBodyName))
        {
            nameList.Add(rigidBodyName);
            targetRigidBodyNames = nameList.ToArray();
        }

        if (!cachedRigidBodyNames.Contains(rigidBodyName))
        {
            cachedRigidBodyNames.Add(rigidBodyName);
        }
    }

    /// <summary>
    /// 移除监控的刚体名称
    /// </summary>
    /// <param name="rigidBodyName">刚体名称</param>
    public void RemoveTargetRigidBody(string rigidBodyName)
    {
        if (string.IsNullOrEmpty(rigidBodyName)) return;

        List<string> nameList = new List<string>(targetRigidBodyNames);
        nameList.Remove(rigidBodyName);
        targetRigidBodyNames = nameList.ToArray();

        cachedRigidBodyNames.Remove(rigidBodyName);
    }

    /// <summary>
    /// 清空所有监控的刚体
    /// </summary>
    public void ClearTargetRigidBodies()
    {
        targetRigidBodyNames = new string[] { };
        cachedRigidBodyNames.Clear();
    }

    /// <summary>
    /// 获取当前日志文件路径
    /// </summary>
    /// <returns>日志文件完整路径，如果未启用文件日志则返回空字符串</returns>
    public string GetLogFilePath()
    {
        return fullLogPath ?? string.Empty;
    }

    /// <summary>
    /// 重新开始新的日志文件
    /// </summary>
    public void StartNewLogFile()
    {
        CloseFileWriter();
        loggedFrameCount = 0;
        
        if (enableFileLog)
        {
            InitializeFileWriter();
        }
    }

    /// <summary>
    /// 获取指定刚体的最新位姿数据
    /// </summary>
    /// <param name="rigidBodyName">刚体名称</param>
    /// <param name="position">输出位置</param>
    /// <param name="rotation">输出旋转（四元数）</param>
    /// <returns>是否成功获取数据</returns>
    public bool TryGetRigidBodyPose(string rigidBodyName, out Vector3 position, out Quaternion rotation)
    {
        position = Vector3.zero;
        rotation = Quaternion.identity;

        if (streamingClient == null) return false;

        NokovRigidBodyState rbState = streamingClient.GetLatestRigidBodyState(rigidBodyName);
        if (rbState == null || rbState.Pose == null) return false;

        position = rbState.Pose.Position;
        rotation = rbState.Pose.Orientation;
        return true;
    }

    /// <summary>
    /// 获取指定刚体的最新位姿数据（包含欧拉角）
    /// </summary>
    /// <param name="rigidBodyName">刚体名称</param>
    /// <param name="position">输出位置</param>
    /// <param name="eulerAngles">输出欧拉角</param>
    /// <returns>是否成功获取数据</returns>
    public bool TryGetRigidBodyPoseEuler(string rigidBodyName, out Vector3 position, out Vector3 eulerAngles)
    {
        position = Vector3.zero;
        eulerAngles = Vector3.zero;

        if (TryGetRigidBodyPose(rigidBodyName, out position, out Quaternion rotation))
        {
            eulerAngles = rotation.eulerAngles;
            return true;
        }

        return false;
    }

    #endregion

    #region 编辑器辅助

#if UNITY_EDITOR
    /// <summary>
    /// 编辑器中验证参数
    /// </summary>
    void OnValidate()
    {
        if (logInterval < 0) logInterval = 0;
        if (positionDecimalPlaces < 1) positionDecimalPlaces = 1;
        if (rotationDecimalPlaces < 1) rotationDecimalPlaces = 1;
    }
#endif

    #endregion
}

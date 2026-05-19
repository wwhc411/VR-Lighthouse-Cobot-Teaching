using UnityEngine;
using System;
using System.Text;
using System.Collections.Generic;
using CSNokovSDK;

/// <summary>
/// Nokov动捕数据日志打印器
/// 用于调试和监控SDK输出的刚体位姿数据
/// </summary>
public class NokovDataLogger : MonoBehaviour
{
    #region 日志配置
    [Header("日志配置")]
    [Tooltip("启用日志打印")]
    public bool enableLogging = true;
    
    [Tooltip("日志输出模式")]
    public LogMode logMode = LogMode.FrameSummary;
    
    [Tooltip("日志输出间隔(秒) - 避免刷屏")]
    [Range(0.1f, 5f)]
    public float logInterval = 1f;
    
    [Tooltip("仅记录指定ID的刚体(为空则记录全部)")]
    public List<int> filterRigidBodyIDs = new List<int>();
    
    public enum LogMode
    {
        FrameSummary,      // 帧摘要(仅统计信息)
        RigidBodyBasic,    // 刚体基础信息(位置+旋转)
        RigidBodyDetailed, // 刚体详细信息(含Marker点)
        AllData            // 完整数据(含MarkerSet/Skeleton等)
    }
    #endregion

    #region 数据格式化
    [Header("数据格式化")]
    [Tooltip("位置单位转换")]
    public PositionUnit positionUnit = PositionUnit.Millimeter;
    
    [Tooltip("旋转格式")]
    public RotationFormat rotationFormat = RotationFormat.Quaternion;
    
    [Tooltip("数据有效性检查")]
    public bool checkDataValidity = true;
    
    public enum PositionUnit
    {
        Millimeter,  // 毫米(SDK原始)
        Meter        // 米(Unity常用)
    }
    
    public enum RotationFormat
    {
        Quaternion,  // 四元数
        EulerAngles  // 欧拉角(度)
    }
    #endregion

    #region 统计信息
    [Header("统计信息(只读)")]
    [SerializeField] private int totalFramesReceived = 0;
    [SerializeField] private int validFramesCount = 0;
    [SerializeField] private int invalidFramesCount = 0;
    [SerializeField] private float averageFPS = 0f;
    
    private float lastLogTime = 0f;
    private int framesSinceLastLog = 0;
    private float logStartTime = 0f;
    #endregion

    #region Unity生命周期
    void Start()
    {
        // 使用Start()而不是OnEnable()，确保NokovSDKManager.Instance已经初始化
        if (NokovSDKManager.Instance == null)
        {
            Debug.LogError("[NokovLogger] NokovSDKManager未找到,请确保场景中存在NokovSDKManager组件");
            return;
        }

        // 订阅事件
        NokovSDKManager.Instance.OnFrameDataReceived += OnFrameDataReceived;
        NokovSDKManager.Instance.OnRigidBodyDataReceived += OnRigidBodyDataReceived;
        NokovSDKManager.Instance.OnConnectionStateChanged += OnConnectionStateChanged;
        NokovSDKManager.Instance.OnNotifyMessage += OnNotifyMessage;
        
        Debug.Log("<color=cyan>[NokovLogger] 已订阅NokovSDKManager事件</color>");

        logStartTime = Time.time;
        ResetStatistics();
    }

    void OnDisable()
    {
        if (NokovSDKManager.Instance != null)
        {
            NokovSDKManager.Instance.OnFrameDataReceived -= OnFrameDataReceived;
            NokovSDKManager.Instance.OnRigidBodyDataReceived -= OnRigidBodyDataReceived;
            NokovSDKManager.Instance.OnConnectionStateChanged -= OnConnectionStateChanged;
            NokovSDKManager.Instance.OnNotifyMessage -= OnNotifyMessage;
            
            Debug.Log("<color=cyan>[NokovLogger] 已取消订阅NokovSDKManager事件</color>");
        }
    }
    #endregion

    #region 事件处理
    /// <summary>
    /// 完整帧数据接收处理
    /// </summary>
    private void OnFrameDataReceived(sFrameOfMocapData frame)
    {
        if (!enableLogging)
        {
            Debug.LogWarning($"<color=yellow>[NokovLogger] enableLogging=false, 日志已禁用!</color>");
            return;
        }

        totalFramesReceived++;
        framesSinceLastLog++;
        
        // 首次接收数据时输出提示
        if (totalFramesReceived == 1)
        {
            Debug.Log($"<color=green>[NokovLogger] 首次接收到帧数据! 帧号={frame.FrameNumber}, 刚体数={frame.RigidBodyCount}</color>");
            Debug.Log($"<color=green>[NokovLogger] 当前日志模式: {logMode}, 日志间隔: {logInterval}s</color>");
        }

        // 检查日志间隔
        if (Time.time - lastLogTime < logInterval)
        {
            // 添加调试信息，帮助诊断为何没有输出
            if (totalFramesReceived <= 5)
            {
                Debug.Log($"<color=cyan>[NokovLogger] 等待日志间隔... 剩余时间: {logInterval - (Time.time - lastLogTime):F2}s</color>");
            }
            return;
        }

        lastLogTime = Time.time;

        // 计算平均帧率
        float elapsed = Time.time - logStartTime;
        averageFPS = totalFramesReceived / (elapsed > 0 ? elapsed : 1f);

        // 添加诊断日志
        Debug.Log($"<color=magenta>[NokovLogger] 准备输出日志 | 模式: {logMode} | 帧号: {frame.FrameNumber} | 刚体数: {frame.RigidBodyCount}</color>");

        // 根据日志模式输出
        switch (logMode)
        {
            case LogMode.FrameSummary:
                LogFrameSummary(frame);
                break;
            case LogMode.RigidBodyBasic:
                LogRigidBodyBasic(frame);
                break;
            case LogMode.RigidBodyDetailed:
                LogRigidBodyDetailed(frame);
                break;
            case LogMode.AllData:
                LogAllData(frame);
                break;
        }

        framesSinceLastLog = 0;
    }

    /// <summary>
    /// 刚体数据接收处理(仅在需要时使用)
    /// </summary>
    private void OnRigidBodyDataReceived(sRigidBodyData[] rigids)
    {
        // 该事件用于轻量级处理,日志主要使用OnFrameDataReceived
    }

    /// <summary>
    /// 连接状态变化处理
    /// </summary>
    private void OnConnectionStateChanged(bool connected)
    {
        if (connected)
        {
            Debug.Log("<color=green>═══════════════════════════════════════</color>");
            Debug.Log("<color=green>[NokovLogger] Nokov SDK已连接,开始接收数据</color>");
            Debug.Log("<color=green>═══════════════════════════════════════</color>");
            ResetStatistics();
        }
        else
        {
            Debug.Log("<color=red>═══════════════════════════════════════</color>");
            Debug.Log("<color=red>[NokovLogger] Nokov SDK已断开连接</color>");
            Debug.Log($"<color=yellow>统计: 总帧数={totalFramesReceived}, 有效={validFramesCount}, 无效={invalidFramesCount}</color>");
            Debug.Log("<color=red>═══════════════════════════════════════</color>");
        }
    }

    /// <summary>
    /// SDK通知消息处理
    /// </summary>
    private void OnNotifyMessage(sNotifyMsg notify)
    {
        Debug.Log($"<color=cyan>[NokovSDK通知] 类型:{notify.Type} | 值:{notify.Value} | " +
                  $"时间戳:{notify.TimeStamp} | 参数:{notify.Param1}</color>");
    }
    #endregion

    #region 日志输出方法
    /// <summary>
    /// 帧摘要日志
    /// </summary>
    private void LogFrameSummary(sFrameOfMocapData frame)
    {
        StringBuilder sb = new StringBuilder();
        sb.AppendLine("┌─────────────────────────────────────────────────────────┐");
        sb.AppendLine($"│ 帧摘要 - 帧号: {frame.FrameNumber,-10} 时间戳: {frame.Timestamp,-15} │");
        sb.AppendLine("├─────────────────────────────────────────────────────────┤");
        sb.AppendLine($"│ 刚体数量:        {frame.RigidBodyCount,-5} │ 平均FPS:      {averageFPS,6:F1}    │");
        sb.AppendLine($"│ MarkerSet数量:   {frame.MarkerSetCount,-5} │ 总接收帧数:   {totalFramesReceived,6}    │");
        sb.AppendLine($"│ 骨骼数量:        {frame.SkeletonCount,-5} │ 延迟(ms):     {frame.FLatency * 1000,6:F2}    │");
        sb.AppendLine($"│ 未命名Marker:    {frame.OtherMarkerCount,-5} │ 命名Marker:   {frame.LabeledMarkerCount,6}    │");
        sb.AppendLine("└─────────────────────────────────────────────────────────┘");

        Debug.Log(sb.ToString());
    }

    /// <summary>
    /// 刚体基础信息日志
    /// </summary>
    private void LogRigidBodyBasic(sFrameOfMocapData frame)
    {
        Debug.Log($"<color=yellow>[NokovLogger] 调用LogRigidBodyBasic | 刚体数: {frame.RigidBodyCount}</color>");
        
        if (frame.RigidBodyCount == 0)
        {
            Debug.Log($"<color=yellow>[NokovLogger] 帧{frame.FrameNumber}: 无刚体数据</color>");
            return;
        }

        StringBuilder sb = new StringBuilder();
        sb.AppendLine($"\n╔═══════════ 帧号: {frame.FrameNumber} | 时间戳: {frame.Timestamp} ═══════════╗");
        sb.AppendLine($"║ 刚体数量: {frame.RigidBodyCount} | 平均FPS: {averageFPS:F1} | 延迟: {frame.FLatency * 1000:F2}ms");
        sb.AppendLine("╠══════════════════════════════════════════════════════════════════╣");

        for (int i = 0; i < frame.RigidBodyCount; i++)
        {
            var rigid = frame.RigidBodies[i];

            // 过滤刚体ID
            if (filterRigidBodyIDs.Count > 0 && !filterRigidBodyIDs.Contains(rigid.Id))
                continue;

            // 有效性检查
            bool isValid = NokovSDKManager.IsValidPosition(rigid.X, rigid.Y, rigid.Z);
            if (!isValid)
            {
                invalidFramesCount++;
                if (checkDataValidity)
                {
                    sb.AppendLine($"║ 刚体[{rigid.Id}] <color=red>数据无效</color> (位置异常: 9999999)");
                    continue;
                }
            }
            else
            {
                validFramesCount++;
            }

            // 位置转换
            Vector3 position = GetFormattedPosition(rigid.X, rigid.Y, rigid.Z);
            string posUnit = positionUnit == PositionUnit.Millimeter ? "mm" : "m";

            // 旋转转换
            string rotationStr = GetFormattedRotation(rigid.QX, rigid.QY, rigid.QZ, rigid.QW);

            sb.AppendLine($"║ 刚体[{rigid.Id}]:");
            sb.AppendLine($"║   位置({posUnit}):  X={position.x,9:F2}  Y={position.y,9:F2}  Z={position.z,9:F2}");
            sb.AppendLine($"║   {rotationStr}");
            sb.AppendLine($"║   Marker数: {rigid.NMarkers,-3} | 均方误差: {rigid.MeanError:F4}");
            sb.AppendLine("║ ──────────────────────────────────────────────────────────────");
        }

        sb.AppendLine("╚══════════════════════════════════════════════════════════════════╝");
        Debug.Log(sb.ToString());
    }

    /// <summary>
    /// 刚体详细信息日志(含Marker点)
    /// </summary>
    private void LogRigidBodyDetailed(sFrameOfMocapData frame)
    {
        Debug.Log($"<color=yellow>[NokovLogger] 调用LogRigidBodyDetailed | 刚体数: {frame.RigidBodyCount}</color>");
        
        if (frame.RigidBodyCount == 0)
        {
            Debug.Log($"<color=yellow>[NokovLogger] 帧{frame.FrameNumber}: 无刚体数据</color>");
            return;
        }

        StringBuilder sb = new StringBuilder();
        sb.AppendLine($"\n╔═══════════ 详细刚体数据 - 帧{frame.FrameNumber} ═══════════╗");

        for (int i = 0; i < frame.RigidBodyCount; i++)
        {
            var rigid = frame.RigidBodies[i];

            // 过滤刚体ID
            if (filterRigidBodyIDs.Count > 0 && !filterRigidBodyIDs.Contains(rigid.Id))
                continue;

            // 有效性检查
            bool isValid = NokovSDKManager.IsValidPosition(rigid.X, rigid.Y, rigid.Z);
            string validityTag = isValid ? "<color=green>[有效]</color>" : "<color=red>[无效]</color>";

            sb.AppendLine($"║");
            sb.AppendLine($"║ ══ 刚体ID: {rigid.Id} {validityTag} ══");
            
            // 位置和旋转
            Vector3 position = GetFormattedPosition(rigid.X, rigid.Y, rigid.Z);
            string posUnit = positionUnit == PositionUnit.Millimeter ? "mm" : "m";
            string rotationStr = GetFormattedRotation(rigid.QX, rigid.QY, rigid.QZ, rigid.QW);

            sb.AppendLine($"║ 位置({posUnit}): ({position.x:F2}, {position.y:F2}, {position.z:F2})");
            sb.AppendLine($"║ {rotationStr}");
            sb.AppendLine($"║ 均方误差: {rigid.MeanError:F4}");
            sb.AppendLine($"║ Marker点数量: {rigid.NMarkers}");

            // Marker点详细信息
            if (rigid.NMarkers > 0 && rigid.Markers != IntPtr.Zero)
            {
                sb.AppendLine("║ Marker点详情:");
                for (int m = 0; m < rigid.NMarkers; m++)
                {
                    IntPtr markerPtr = IntPtr.Add(rigid.Markers, 
                        System.Runtime.InteropServices.Marshal.SizeOf(typeof(tMarkerData)) * m);
                    var marker = (tMarkerData)System.Runtime.InteropServices.Marshal.PtrToStructure(
                        markerPtr, typeof(tMarkerData));

                    int markerId = -1;
                    if (rigid.MarkerIDs != IntPtr.Zero)
                    {
                        IntPtr idPtr = IntPtr.Add(rigid.MarkerIDs, sizeof(int) * m);
                        markerId = System.Runtime.InteropServices.Marshal.ReadInt32(idPtr);
                    }

                    Vector3 markerPos = GetFormattedPosition(
                        marker.Values[0], marker.Values[1], marker.Values[2]);

                    sb.AppendLine($"║   [{m}] ID={markerId,-3} 位置=({markerPos.x,8:F2}, {markerPos.y,8:F2}, {markerPos.z,8:F2}) {posUnit}");
                }
            }

            sb.AppendLine("║ ────────────────────────────────────────────────────");
        }

        sb.AppendLine("╚══════════════════════════════════════════════════════════════════╝");
        Debug.Log(sb.ToString());
    }

    /// <summary>
    /// 完整数据日志(含MarkerSet/Skeleton等)
    /// </summary>
    private void LogAllData(sFrameOfMocapData frame)
    {
        StringBuilder sb = new StringBuilder();
        sb.AppendLine($"\n╔═══════════════════ 完整帧数据 ═══════════════════╗");
        sb.AppendLine($"║ 帧号: {frame.FrameNumber} | 时间戳: {frame.Timestamp}");
        sb.AppendLine($"║ 延迟: {frame.FLatency * 1000:F2}ms | Timecode: {frame.Timecode}");
        sb.AppendLine("╠═══════════════════════════════════════════════════╣");

        // MarkerSet信息
        sb.AppendLine($"║ MarkerSet数量: {frame.MarkerSetCount}");
        for (int i = 0; i < frame.MarkerSetCount; i++)
        {
            var markerSet = frame.MarkerSets[i];
            string name = new string(markerSet.Name).TrimEnd('\0');
            sb.AppendLine($"║   [{i}] {name} - Marker数: {markerSet.MarkerCount}");
        }

        // 刚体信息(简化)
        sb.AppendLine($"║ 刚体数量: {frame.RigidBodyCount}");
        for (int i = 0; i < frame.RigidBodyCount; i++)
        {
            var rigid = frame.RigidBodies[i];
            bool isValid = NokovSDKManager.IsValidPosition(rigid.X, rigid.Y, rigid.Z);
            string validTag = isValid ? "✓" : "✗";
            sb.AppendLine($"║   [{rigid.Id}] {validTag} Markers:{rigid.NMarkers} Error:{rigid.MeanError:F4}");
        }

        // 骨骼信息
        sb.AppendLine($"║ 骨骼数量: {frame.SkeletonCount}");
        for (int i = 0; i < frame.SkeletonCount; i++)
        {
            var skeleton = frame.Skeletons[i];
            sb.AppendLine($"║   [{skeleton.Id}] 刚体/骨骼数: {skeleton.RigidBodyCount}");
        }

        // Marker统计
        sb.AppendLine($"║ 命名Marker: {frame.LabeledMarkerCount} | 未命名Marker: {frame.OtherMarkerCount}");
        
        // 模拟数据
        sb.AppendLine($"║ 模拟数据通道: {frame.AnalogdataCount}");

        sb.AppendLine("╚═══════════════════════════════════════════════════╝");
        Debug.Log(sb.ToString());

        // 再输出详细的刚体数据
        LogRigidBodyDetailed(frame);
    }
    #endregion

    #region 格式化工具
    /// <summary>
    /// 格式化位置数据
    /// </summary>
    private Vector3 GetFormattedPosition(float x, float y, float z)
    {
        if (positionUnit == PositionUnit.Meter)
        {
            return new Vector3(x / 1000f, y / 1000f, z / 1000f);
        }
        return new Vector3(x, y, z);
    }

    /// <summary>
    /// 格式化旋转数据
    /// </summary>
    private string GetFormattedRotation(float qx, float qy, float qz, float qw)
    {
        Quaternion quat = new Quaternion(qx, qy, qz, qw);

        if (rotationFormat == RotationFormat.Quaternion)
        {
            return $"四元数:   QX={qx,7:F4}  QY={qy,7:F4}  QZ={qz,7:F4}  QW={qw,7:F4}";
        }
        else
        {
            Vector3 euler = quat.eulerAngles;
            return $"欧拉角(°): Roll={euler.x,7:F2}  Pitch={euler.y,7:F2}  Yaw={euler.z,7:F2}";
        }
    }
    #endregion

    #region 统计管理
    /// <summary>
    /// 重置统计信息
    /// </summary>
    [ContextMenu("重置统计数据")]
    public void ResetStatistics()
    {
        totalFramesReceived = 0;
        validFramesCount = 0;
        invalidFramesCount = 0;
        averageFPS = 0f;
        framesSinceLastLog = 0;
        logStartTime = Time.time;
        
        Debug.Log("<color=cyan>[NokovLogger] 统计数据已重置</color>");
    }

    /// <summary>
    /// 打印当前统计信息
    /// </summary>
    [ContextMenu("打印统计摘要")]
    public void PrintStatistics()
    {
        float elapsed = Time.time - logStartTime;
        StringBuilder sb = new StringBuilder();
        
        sb.AppendLine("\n╔═══════════════ Nokov数据统计 ═══════════════╗");
        sb.AppendLine($"║ 运行时长:     {elapsed,8:F1} 秒");
        sb.AppendLine($"║ 总接收帧数:   {totalFramesReceived,8}");
        sb.AppendLine($"║ 有效帧数:     {validFramesCount,8}");
        sb.AppendLine($"║ 无效帧数:     {invalidFramesCount,8}");
        sb.AppendLine($"║ 平均帧率:     {averageFPS,8:F2} FPS");
        
        if (totalFramesReceived > 0)
        {
            float validRate = (float)validFramesCount / totalFramesReceived * 100f;
            sb.AppendLine($"║ 数据有效率:   {validRate,8:F2} %");
        }
        
        sb.AppendLine("╚════════════════════════════════════════════╝");
        Debug.Log(sb.ToString());
    }
    #endregion

    #region GUI显示
    void OnGUI()
    {
        if (!enableLogging) return;

        GUIStyle style = new GUIStyle();
        style.fontSize = 14;
        style.normal.textColor = Color.white;

        int yOffset = 85; // NokovSDKManager已占用前60像素

        GUI.Label(new Rect(10, yOffset, 400, 25),
            $"日志模式: {logMode} | 间隔: {logInterval}s", style);

        GUI.Label(new Rect(10, yOffset + 25, 400, 25),
            $"总帧数: {totalFramesReceived} | 有效: {validFramesCount} | 无效: {invalidFramesCount}", style);

        if (filterRigidBodyIDs.Count > 0)
        {
            style.normal.textColor = Color.yellow;
            GUI.Label(new Rect(10, yOffset + 50, 400, 25),
                $"过滤刚体ID: {string.Join(", ", filterRigidBodyIDs)}", style);
        }
    }
    #endregion
}

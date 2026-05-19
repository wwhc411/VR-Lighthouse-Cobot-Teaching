using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;
using UnityEngine;
using Valve.VR;
using Debug = UnityEngine.Debug;

/// <summary>
/// 高频 Tracker 位姿录制器（后台线程版）
/// 
/// 原理:
/// - 使用独立后台线程绑过 Unity 主线程限制
/// - 直接轮询 OpenVR API 获取高频位姿数据
/// - 同时记录UR机械臂TCP位姿（从TCP/IP协议读取）
/// - 理论采样率可达 ~1000 Hz（取决于硬件和系统负载）
/// 
/// 优势:
/// - 不受 Unity 帧率/FixedUpdate 限制
/// - 可获取 IMU 融合后的高频位姿
/// - 纳秒级时间戳精度
/// - Tracker与TCP位姿时间同步
/// 
/// 使用方法:
/// - 按 S 键开始高频录制
/// - 按 E 键停止录制并保存
/// 
/// 注意事项:
/// - 高频录制会产生大量数据（1000Hz × 10秒 = 10000条记录）
/// - 建议短时间录制（几秒到几十秒）
/// - 需要UR机器人已连接才能录制TCP数据
/// </summary>
public class HighFrequencyTrackerRecorder : MonoBehaviour
{
    // ==================== Inspector 配置 ====================
    
    [Header("Tracker 配置")]
    [Tooltip("要录制的 Tracker 设备 ID")]
    public uint trackerDeviceId = 2;
    
    [Header("采样参数")]
    [Tooltip("目标采样频率 (Hz)，0=最大频率")]
    [Range(0, 2000)]
    public int targetSampleRateHz = 1000;
    
    [Tooltip("最大录制时长（秒），0=无限制")]
    public float maxRecordDurationSec = 30f;
    
    [Tooltip("最大录制样本数，0=无限制")]
    public int maxSamples = 50000;
    
    [Header("文件保存")]
    [Tooltip("CSV 文件保存子目录")]
    public string saveDirectory = "TrackerRecordings";
    
    [Tooltip("文件名前缀")]
    public string fileNamePrefix = "TrackerRecord";
    
    [Header("UR TCP录制")]
    [Tooltip("同时录制UR机械臂TCP位姿")]
    public bool recordURTcp = true;
    
    [Header("状态显示（只读）")]
    [SerializeField] private bool _isRecording = false;
    [SerializeField] private int _sampleCount = 0;
    [SerializeField] private float _actualSampleRateHz = 0f;
    [SerializeField] private float _recordDuration = 0f;
    
    // ==================== 内部状态 ====================
    
    private volatile bool isRecording = false;
    private volatile bool shouldStop = false;
    private Thread recordingThread;
    
    // 线程安全队列，存储采集的位姿数据
    private ConcurrentQueue<HighFreqPoseRecord> poseQueue;
    
    // 统计信息
    private Stopwatch sessionStopwatch;
    private int totalSamples = 0;
    private long firstSampleTicks = 0;
    private long lastSampleTicks = 0;
    
    // OpenVR 引用
    private CVRSystem vrSystem;
    
    // 格式化
    private static readonly CultureInfo InvariantCulture = CultureInfo.InvariantCulture;
    
    // ==================== 数据结构 ====================
    
    /// <summary>
    /// 高频位姿记录（包含Tracker和UR TCP数据）
    /// </summary>
    public struct HighFreqPoseRecord
    {
        // 时间信息
        public long ticksFromStart;      // Stopwatch ticks (高精度)
        public double timeFromStartSec;  // 秒
        
        // Tracker位姿 (SteamVR坐标系)
        public float x_mm, y_mm, z_mm;   // 位置 (mm)
        public float qx, qy, qz, qw;     // 四元数
        public float rx_rad, ry_rad, rz_rad;  // 旋转矢量 (rad)
        public bool isValid;             // Tracker数据是否有效
        
        // UR TCP位姿 (UR基座坐标系)
        public double tcp_x, tcp_y, tcp_z;       // TCP位置 (m)
        public double tcp_rx, tcp_ry, tcp_rz;    // TCP姿态-旋转矢量 (rad)
        public bool tcpValid;                     // TCP数据是否有效
    }
    
    // ==================== 生命周期 ====================
    
    void Start()
    {
        poseQueue = new ConcurrentQueue<HighFreqPoseRecord>();
        sessionStopwatch = new Stopwatch();
        
        // 获取 OpenVR 系统引用
        if (OpenVR.System == null)
        {
            EVRInitError error = EVRInitError.None;
            vrSystem = OpenVR.Init(ref error, EVRApplicationType.VRApplication_Overlay);
            if (error != EVRInitError.None)
            {
                Debug.LogError($"[HighFreqRecorder] OpenVR 初始化失败: {error}");
                enabled = false;
                return;
            }
        }
        else
        {
            vrSystem = OpenVR.System;
        }
        
        Debug.Log("<color=cyan>[高频录制器] 初始化完成</color>");
        Debug.Log($"  目标采样率: {(targetSampleRateHz > 0 ? $"{targetSampleRateHz} Hz" : "最大频率")}");
        Debug.Log($"  <color=yellow>快捷键: S=开始高频录制, E=停止并保存</color>");
    }
    
    void Update()
    {
        // 按键检测
        if (Input.GetKeyDown(KeyCode.S))
        {
            StartHighFrequencyRecording();
        }
        else if (Input.GetKeyDown(KeyCode.E))
        {
            StopHighFrequencyRecording();
        }
        
        // 更新 Inspector 显示
        _isRecording = isRecording;
        _sampleCount = totalSamples;
        _recordDuration = sessionStopwatch.IsRunning ? 
            (float)sessionStopwatch.Elapsed.TotalSeconds : 0f;
        
        if (totalSamples > 1 && lastSampleTicks > firstSampleTicks)
        {
            double durationSec = (lastSampleTicks - firstSampleTicks) / (double)Stopwatch.Frequency;
            _actualSampleRateHz = (float)((totalSamples - 1) / durationSec);
        }
    }
    
    void OnDestroy()
    {
        if (isRecording)
        {
            StopHighFrequencyRecording();
        }
        
        // 清理资源
        CleanupResources();
    }
    
    void OnApplicationQuit()
    {
        if (isRecording)
        {
            shouldStop = true;
            recordingThread?.Join(1000);
        }
        
        // 清理资源
        CleanupResources();
    }
    
    /// <summary>
    /// 清理所有资源，释放内存
    /// </summary>
    private void CleanupResources()
    {
        // 清空队列中残留的数据
        if (poseQueue != null)
        {
            while (poseQueue.TryDequeue(out _)) { }
        }
        
        // 清理线程引用
        recordingThread = null;
        
        // 重置统计
        totalSamples = 0;
        firstSampleTicks = 0;
        lastSampleTicks = 0;
        
        Debug.Log("[HighFreqRecorder] 资源已清理");
    }
    
    // ==================== 公开方法 ====================
    
    /// <summary>
    /// 开始高频录制
    /// </summary>
    [ContextMenu("开始录制 (S键)")]
    public void StartHighFrequencyRecording()
    {
        if (isRecording)
        {
            Debug.LogWarning("[HighFreqRecorder] 已在录制中");
            return;
        }
        
        if (vrSystem == null)
        {
            Debug.LogError("[HighFreqRecorder] OpenVR 未初始化");
            return;
        }
        
        // 验证 Tracker 可用
        if (!vrSystem.IsTrackedDeviceConnected(trackerDeviceId))
        {
            Debug.LogError($"[HighFreqRecorder] Tracker[ID:{trackerDeviceId}] 未连接");
            return;
        }
        
        // 清空队列和状态
        while (poseQueue.TryDequeue(out _)) { }
        totalSamples = 0;
        firstSampleTicks = 0;
        lastSampleTicks = 0;
        shouldStop = false;
        
        // 启动后台录制线程
        isRecording = true;
        sessionStopwatch.Restart();
        
        recordingThread = new Thread(RecordingThreadFunc)
        {
            Name = "HighFreqTrackerRecorder",
            IsBackground = true,
            Priority = System.Threading.ThreadPriority.Highest  // 高优先级
        };
        recordingThread.Start();
        
        Debug.Log("<color=green>==================== 开始高频录制 ====================</color>");
        Debug.Log($"  Tracker ID: {trackerDeviceId}");
        Debug.Log($"  目标频率: {(targetSampleRateHz > 0 ? $"{targetSampleRateHz} Hz" : "最大频率")}");
        Debug.Log($"  最大时长: {(maxRecordDurationSec > 0 ? $"{maxRecordDurationSec} 秒" : "无限制")}");
        Debug.Log($"  TCP录制: {(recordURTcp ? (ur_data_processing.UR_Stream_Data.is_alive ? "<color=green>启用(UR已连接)</color>" : "<color=yellow>启用(UR未连接)</color>") : "禁用")}");
        Debug.Log($"  <color=cyan>按 E 键停止录制</color>");
    }
    
    /// <summary>
    /// 停止高频录制并保存
    /// </summary>
    [ContextMenu("停止录制 (E键)")]
    public void StopHighFrequencyRecording()
    {
        if (!isRecording)
        {
            Debug.LogWarning("[HighFreqRecorder] 当前未在录制");
            return;
        }
        
        // 请求停止线程
        shouldStop = true;
        
        // 等待线程结束
        if (recordingThread != null && recordingThread.IsAlive)
        {
            recordingThread.Join(2000);  // 最多等2秒
        }
        recordingThread = null;  // 释放线程引用
        
        isRecording = false;
        sessionStopwatch.Stop();
        
        // 计算实际采样率
        double actualDuration = 0;
        if (totalSamples > 1 && lastSampleTicks > firstSampleTicks)
        {
            actualDuration = (lastSampleTicks - firstSampleTicks) / (double)Stopwatch.Frequency;
        }
        double actualRate = actualDuration > 0 ? (totalSamples - 1) / actualDuration : 0;
        
        Debug.Log("<color=green>==================== 高频录制完成 ====================</color>");
        Debug.Log($"  总样本数: {totalSamples}");
        Debug.Log($"  录制时长: {actualDuration:F3} 秒");
        Debug.Log($"  <color=yellow>实际采样率: {actualRate:F1} Hz</color>");
        Debug.Log($"  Stopwatch 精度: {1000000.0 / Stopwatch.Frequency:F3} 微秒/tick");
        
        // 保存到文件
        if (totalSamples > 0)
        {
            SaveToCSV();
        }
        else
        {
            Debug.LogWarning("[HighFreqRecorder] 没有数据可保存");
        }
    }
    
    // ==================== 后台线程 ====================
    
    /// <summary>
    /// 录制线程主函数
    /// </summary>
    private void RecordingThreadFunc()
    {
        // 线程局部变量
        TrackedDevicePose_t[] poses = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
        Stopwatch localStopwatch = Stopwatch.StartNew();
        
        // 计算每次采样的间隔（如果设置了目标频率）
        long targetIntervalTicks = 0;
        if (targetSampleRateHz > 0)
        {
            targetIntervalTicks = Stopwatch.Frequency / targetSampleRateHz;
        }
        
        long nextSampleTicks = 0;
        long maxDurationTicks = maxRecordDurationSec > 0 
            ? (long)(maxRecordDurationSec * Stopwatch.Frequency) 
            : long.MaxValue;
        
        try
        {
            while (!shouldStop)
            {
                long currentTicks = localStopwatch.ElapsedTicks;
                
                // 检查最大时长
                if (currentTicks >= maxDurationTicks)
                {
                    Debug.Log("[HighFreqRecorder] 已达到最大录制时长");
                    break;
                }
                
                // 检查最大样本数
                if (maxSamples > 0 && totalSamples >= maxSamples)
                {
                    Debug.Log("[HighFreqRecorder] 已达到最大样本数");
                    break;
                }
                
                // 频率控制
                if (targetIntervalTicks > 0 && currentTicks < nextSampleTicks)
                {
                    // 高精度等待（忙等待，消耗CPU但精度高）
                    // 对于 1000Hz，每次只等待约 1ms
                    Thread.SpinWait(10);
                    continue;
                }
                
                nextSampleTicks = currentTicks + targetIntervalTicks;
                
                // 获取位姿数据
                // 注意：fPredictedSecondsToPhotonsFromNow = 0 表示获取当前时刻的位姿
                vrSystem.GetDeviceToAbsoluteTrackingPose(
                    ETrackingUniverseOrigin.TrackingUniverseStanding,
                    0f,  // 不预测，获取当前位姿
                    poses
                );
                
                var pose = poses[trackerDeviceId];
                
                // 创建记录
                HighFreqPoseRecord record = new HighFreqPoseRecord
                {
                    ticksFromStart = currentTicks,
                    timeFromStartSec = currentTicks / (double)Stopwatch.Frequency,
                    isValid = pose.bPoseIsValid && pose.bDeviceIsConnected,
                    tcpValid = false  // 默认TCP无效
                };
                
                // 记录Tracker位姿
                if (record.isValid)
                {
                    var m = pose.mDeviceToAbsoluteTracking;
                    
                    // 位置 (mm)
                    record.x_mm = m.m3 * 1000f;
                    record.y_mm = m.m7 * 1000f;
                    record.z_mm = m.m11 * 1000f;
                    
                    // 四元数
                    Quaternion q = QuaternionFromMatrix(m);
                    record.qx = q.x;
                    record.qy = q.y;
                    record.qz = q.z;
                    record.qw = q.w;
                    
                    // 旋转矢量
                    Vector3 rv = QuaternionToRotationVector(q);
                    record.rx_rad = rv.x;
                    record.ry_rad = rv.y;
                    record.rz_rad = rv.z;
                }
                
                // 记录UR TCP位姿（从TCP/IP协议读取）
                if (recordURTcp && ur_data_processing.UR_Stream_Data.is_alive)
                {
                    record.tcpValid = true;
                    // TCP位置 (m) - 直接从UR数据流读取
                    record.tcp_x = ur_data_processing.UR_Stream_Data.C_Position[0];
                    record.tcp_y = ur_data_processing.UR_Stream_Data.C_Position[1];
                    record.tcp_z = ur_data_processing.UR_Stream_Data.C_Position[2];
                    // TCP姿态 (rad) - 旋转矢量格式
                    record.tcp_rx = ur_data_processing.UR_Stream_Data.C_Orientation[0];
                    record.tcp_ry = ur_data_processing.UR_Stream_Data.C_Orientation[1];
                    record.tcp_rz = ur_data_processing.UR_Stream_Data.C_Orientation[2];
                }
                
                // 入队
                poseQueue.Enqueue(record);
                
                // 更新统计
                if (totalSamples == 0)
                {
                    firstSampleTicks = currentTicks;
                }
                lastSampleTicks = currentTicks;
                Interlocked.Increment(ref totalSamples);
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"[HighFreqRecorder] 录制线程异常: {ex.Message}");
        }
    }
    
    // ==================== 文件保存 ====================
    
    /// <summary>
    /// 保存数据到 CSV 文件（包含Tracker和TCP位姿）
    /// </summary>
    private void SaveToCSV()
    {
        string directory = Path.Combine(Application.streamingAssetsPath, saveDirectory);
        if (!Directory.Exists(directory))
        {
            Directory.CreateDirectory(directory);
        }
        
        string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string fileName = $"{fileNamePrefix}_{trackerDeviceId}_{timestamp}.csv";
        string filePath = Path.Combine(directory, fileName);
        
        // 计算实际采样率
        double actualDuration = (lastSampleTicks - firstSampleTicks) / (double)Stopwatch.Frequency;
        double actualRate = actualDuration > 0 ? (totalSamples - 1) / actualDuration : 0;
        
        // 检查是否有TCP数据
        bool hasTcpData = false;
        
        try
        {
            // 获取录制开始的基准时间戳
            long baseTimeStampMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() - (long)(actualDuration * 1000);
            
            using (StreamWriter writer = new StreamWriter(filePath, false, Encoding.UTF8))
            {
                // 写入表头（包含Tracker和TCP数据）
                // Tracker: X_mm,Y_mm,Z_mm (SteamVR坐标系, mm)
                // TCP: TCP_X_m,TCP_Y_m,TCP_Z_m (UR基座坐标系, m)
                writer.WriteLine("FrameNumber,TimeStamp_ms,TimeFromStart_s," +
                                "X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,RX_rad,RY_rad,RZ_rad," +
                                "TCP_X_m,TCP_Y_m,TCP_Z_m,TCP_RX_rad,TCP_RY_rad,TCP_RZ_rad");
                
                // 写入数据
                int frameNumber = 0;
                while (poseQueue.TryDequeue(out HighFreqPoseRecord record))
                {
                    // 跳过Tracker无效数据
                    if (!record.isValid)
                        continue;
                    
                    // 计算时间戳
                    long timeStampMs = baseTimeStampMs + (long)(record.timeFromStartSec * 1000);
                    
                    // 检查是否有有效TCP数据
                    if (record.tcpValid)
                        hasTcpData = true;
                    
                    // 写入一行数据：Tracker位姿 + TCP位姿
                    writer.WriteLine(string.Format(InvariantCulture,
                        "{0},{1},{2:F6}," +
                        "{3:F4},{4:F4},{5:F4},{6:F6},{7:F6},{8:F6},{9:F6},{10:F6},{11:F6},{12:F6}," +
                        "{13:F6},{14:F6},{15:F6},{16:F6},{17:F6},{18:F6}",
                        frameNumber++,
                        timeStampMs,
                        record.timeFromStartSec,
                        // Tracker位姿 (SteamVR坐标系)
                        record.x_mm, record.y_mm, record.z_mm,
                        record.qx, record.qy, record.qz, record.qw,
                        record.rx_rad, record.ry_rad, record.rz_rad,
                        // TCP位姿 (UR基座坐标系)
                        record.tcpValid ? record.tcp_x : 0,
                        record.tcpValid ? record.tcp_y : 0,
                        record.tcpValid ? record.tcp_z : 0,
                        record.tcpValid ? record.tcp_rx : 0,
                        record.tcpValid ? record.tcp_ry : 0,
                        record.tcpValid ? record.tcp_rz : 0
                    ));
                }
                
                // 更新实际保存的帧数
                totalSamples = frameNumber;
            }
            
            Debug.Log($"<color=green>[录制器] CSV 文件已保存:</color>");
            Debug.Log($"  路径: {filePath}");
            Debug.Log($"  有效帧数: {totalSamples}");
            Debug.Log($"  实际采样率: {actualRate:F1} Hz");
            Debug.Log($"  TCP数据: {(hasTcpData ? "<color=green>已录制</color>" : "<color=yellow>未录制(UR未连接)</color>")}");
            Debug.Log($"  大小: ~{new FileInfo(filePath).Length / 1024:F1} KB");
        }
        catch (Exception ex)
        {
            Debug.LogError($"[录制器] 保存 CSV 失败: {ex.Message}");
        }
    }
    
    // ==================== 数学工具 ====================
    
    /// <summary>
    /// 从 HmdMatrix34_t 提取四元数
    /// </summary>
    private static Quaternion QuaternionFromMatrix(HmdMatrix34_t m)
    {
        float m00 = m.m0, m01 = m.m1, m02 = m.m2;
        float m10 = m.m4, m11 = m.m5, m12 = m.m6;
        float m20 = m.m8, m21 = m.m9, m22 = m.m10;
        
        float trace = m00 + m11 + m22;
        Quaternion q = new Quaternion();
        
        if (trace > 0f)
        {
            float s = Mathf.Sqrt(trace + 1f) * 2f;
            q.w = 0.25f * s;
            q.x = (m21 - m12) / s;
            q.y = (m02 - m20) / s;
            q.z = (m10 - m01) / s;
        }
        else if (m00 > m11 && m00 > m22)
        {
            float s = Mathf.Sqrt(1f + m00 - m11 - m22) * 2f;
            q.w = (m21 - m12) / s;
            q.x = 0.25f * s;
            q.y = (m01 + m10) / s;
            q.z = (m02 + m20) / s;
        }
        else if (m11 > m22)
        {
            float s = Mathf.Sqrt(1f + m11 - m00 - m22) * 2f;
            q.w = (m02 - m20) / s;
            q.x = (m01 + m10) / s;
            q.y = 0.25f * s;
            q.z = (m12 + m21) / s;
        }
        else
        {
            float s = Mathf.Sqrt(1f + m22 - m00 - m11) * 2f;
            q.w = (m10 - m01) / s;
            q.x = (m02 + m20) / s;
            q.y = (m12 + m21) / s;
            q.z = 0.25f * s;
        }
        
        // 归一化
        float mag = Mathf.Sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
        if (mag > Mathf.Epsilon)
        {
            q.x /= mag; q.y /= mag; q.z /= mag; q.w /= mag;
        }
        
        // 符号规范化
        if (q.w < 0f)
        {
            q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w;
        }
        
        return q;
    }
    
    /// <summary>
    /// 四元数转旋转矢量
    /// </summary>
    private static Vector3 QuaternionToRotationVector(Quaternion q)
    {
        if (q.w < 0f)
        {
            q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w;
        }
        
        float wClamped = Mathf.Clamp(q.w, 0f, 1f);
        float angle = 2f * Mathf.Acos(wClamped);
        
        if (angle < 1e-6f)
            return Vector3.zero;
        
        if (angle > Mathf.PI - 1e-4f)
        {
            Vector3 axis = new Vector3(q.x, q.y, q.z);
            float axisMag = axis.magnitude;
            if (axisMag > 1e-8f)
                axis /= axisMag;
            else
                axis = new Vector3(1f, 0f, 0f);
            return axis * angle;
        }
        
        float sinHalf = Mathf.Sin(angle * 0.5f);
        float scale = angle / sinHalf;
        return new Vector3(q.x * scale, q.y * scale, q.z * scale);
    }
}

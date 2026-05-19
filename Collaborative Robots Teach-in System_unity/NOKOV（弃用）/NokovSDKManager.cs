using UnityEngine;
using System;
using System.Runtime.InteropServices;
using CSNokovSDK;
using System.Collections.Generic;

/// <summary>
/// Nokov动作捕捉系统SDK管理器
/// 支持回调模式和轮询模式两种数据接收方式
/// </summary>
public class NokovSDKManager : MonoBehaviour
{
    #region Singleton
    public static NokovSDKManager Instance { get; private set; }
    #endregion

    #region 连接设置
    [Header("连接设置")]
    [Tooltip("Nokov服务器IP地址")]
    public string serverIP = "10.1.1.198";
    
    [Tooltip("启动时自动连接")]
    public bool autoConnect = true;
    
    [Tooltip("连接失败后自动重试")]
    public bool autoReconnect = true;
    
    [Tooltip("重连间隔(秒)")]
    public float reconnectInterval = 5f;
    #endregion

    #region 数据接收模式
    [Header("数据接收模式")]
    [Tooltip("回调模式: SDK自动推送数据 | 轮询模式: 手动读取数据")]
    public DataReceiveMode receiveMode = DataReceiveMode.Callback;
    
    [Tooltip("轮询模式下的更新频率(Hz)")]
    [Range(1, 200)]
    public int pollingFrequency = 60;
    
    public enum DataReceiveMode
    {
        Callback,   // 回调模式(推荐)
        Polling     // 轮询模式
    }
    #endregion

    #region 调试设置
    [Header("调试设置")]
    [Tooltip("启用详细日志")]
    public bool enableDebugLog = true;
    
    [Tooltip("显示帧率统计")]
    public bool showFrameStats = true;
    
    [Tooltip("统计间隔(秒)")]
    public float statsInterval = 1f;
    #endregion

    #region 私有字段
    private IntPtr clientHandler = IntPtr.Zero;
    private bool isConnected = false;
    private float lastReconnectTime = 0f;
    
    // 回调委托(必须是静态字段防止GC回收)
    private static NokovSDKFrameReceivedCallback dataCallback;
    private static NokovNotifyMsgCallback notifyCallback;
    
    // 轮询模式
    private float pollingInterval;
    private float lastPollingTime = 0f;
    
    // 帧率统计
    private int frameCount = 0;
    private float lastStatsTime = 0f;
    private float currentFPS = 0f;
    
    // 回调线程到主线程的帧队列
    private readonly Queue<sFrameOfMocapData> frameQueue = new Queue<sFrameOfMocapData>();
    private readonly object frameQueueLock = new object();
    #endregion

    #region 事件
    /// <summary>
    /// 刚体数据更新事件
    /// </summary>
    public event Action<sRigidBodyData[]> OnRigidBodyDataReceived;
    
    /// <summary>
    /// 完整帧数据更新事件
    /// </summary>
    public event Action<sFrameOfMocapData> OnFrameDataReceived;
    
    /// <summary>
    /// 连接状态变化事件
    /// </summary>
    public event Action<bool> OnConnectionStateChanged;
    
    /// <summary>
    /// SDK通知消息事件
    /// </summary>
    public event Action<sNotifyMsg> OnNotifyMessage;
    #endregion

    #region 公共属性
    /// <summary>
    /// 是否已连接到Nokov服务器
    /// </summary>
    public bool IsConnected => isConnected;
    
    /// <summary>
    /// 当前接收帧率
    /// </summary>
    public float CurrentFPS => currentFPS;
    
    /// <summary>
    /// 客户端句柄
    /// </summary>
    public IntPtr ClientHandler => clientHandler;
    #endregion

    #region Unity生命周期
    void Awake()
    {
        // 单例模式
        if (Instance != null && Instance != this)
        {
            Debug.LogWarning("NokovSDKManager已存在,销毁重复实例");
            Destroy(gameObject);
            return;
        }
        Instance = this;
        DontDestroyOnLoad(gameObject);

        // 初始化回调委托
        dataCallback = OnDataReceivedCallback;
        notifyCallback = OnNotifyCallback;
        
        // 计算轮询间隔
        pollingInterval = 1f / pollingFrequency;
        
        LogDebug("NokovSDKManager初始化完成");
    }

    void Start()
    {
        if (autoConnect)
        {
            Connect();
        }
    }

    void Update()
    {
        // 处理来自SDK回调线程的帧队列（在主线程中执行，允许安全调用Unity API）
        lock (frameQueueLock)
        {
            while (frameQueue.Count > 0)
            {
                sFrameOfMocapData frame = frameQueue.Dequeue();
                ProcessFrameData(frame);
            }
        }

        // 轮询模式下主动读取数据
        if (receiveMode == DataReceiveMode.Polling && isConnected)
        {
            if (Time.time - lastPollingTime >= pollingInterval)
            {
                lastPollingTime = Time.time;
                PollFrameData();
            }
        }

        // 自动重连
        if (!isConnected && autoReconnect)
        {
            if (Time.time - lastReconnectTime >= reconnectInterval)
            {
                lastReconnectTime = Time.time;
                LogDebug("尝试重新连接...");
                Connect();
            }
        }

        // 帧率统计
        if (showFrameStats && Time.time - lastStatsTime >= statsInterval)
        {
            currentFPS = frameCount / statsInterval;
            if (enableDebugLog)
            {
                Debug.Log($"[Nokov] 接收帧率: {currentFPS:F1} FPS");
            }
            frameCount = 0;
            lastStatsTime = Time.time;
        }
    }

    void OnApplicationQuit()
    {
        Disconnect();
    }

    void OnDestroy()
    {
        Disconnect();
        if (Instance == this)
        {
            Instance = null;
        }
    }
    #endregion

    #region 连接管理
    /// <summary>
    /// 连接到Nokov服务器
    /// </summary>
    public bool Connect()
    {
        if (isConnected)
        {
            LogDebug("已经连接到Nokov服务器");
            return true;
        }

        try
        {
            // 创建客户端
            CNokovSDK.CreateClient(out clientHandler);
            if (clientHandler == IntPtr.Zero)
            {
                Debug.LogError("[Nokov] 创建客户端失败");
                return false;
            }

            // 获取SDK版本
            byte[] ver = new byte[4];
            CNokovSDK.NokovVersion(clientHandler, ver);
            LogDebug($"NokovSDK版本: {ver[0]}.{ver[1]}.{ver[2]}.{ver[3]}");

            // 连接服务器
            int result = (int)CNokovSDK.Initialize(clientHandler, serverIP);
            if (result != 0)
            {
                Debug.LogError($"[Nokov] 连接服务器失败,错误码: {result}, IP: {serverIP}");
                CNokovSDK.DestroyClient(clientHandler);
                clientHandler = IntPtr.Zero;
                return false;
            }

            // 获取数据描述信息
            IntPtr pDataDescriptions = IntPtr.Zero;
            int descResult = (int)CNokovSDK.GetDataDescriptions(clientHandler, out pDataDescriptions);
            if (pDataDescriptions != IntPtr.Zero)
            {
                ProcessDataDescriptions(pDataDescriptions);
            }

            // 根据模式设置数据接收方式
            if (receiveMode == DataReceiveMode.Callback)
            {
                CNokovSDK.SetDataCallback(clientHandler, dataCallback, IntPtr.Zero);
                LogDebug("已启用回调模式");
            }
            else
            {
                LogDebug($"已启用轮询模式,频率: {pollingFrequency}Hz");
            }

            // 设置通知回调
            CNokovSDK.SetNotifyMsgCallback(clientHandler, notifyCallback, IntPtr.Zero);

            isConnected = true;
            Debug.Log($"[Nokov] 成功连接到服务器: {serverIP}");
            OnConnectionStateChanged?.Invoke(true);
            
            return true;
        }
        catch (Exception e)
        {
            Debug.LogError($"[Nokov] 连接异常: {e.Message}\n{e.StackTrace}");
            return false;
        }
    }

    /// <summary>
    /// 断开连接
    /// </summary>
    public void Disconnect()
    {
        if (!isConnected || clientHandler == IntPtr.Zero)
        {
            return;
        }

        try
        {
            CNokovSDK.Uninitialize(clientHandler);
            CNokovSDK.DestroyClient(clientHandler);
            clientHandler = IntPtr.Zero;
            isConnected = false;
            
            Debug.Log("[Nokov] 已断开连接");
            OnConnectionStateChanged?.Invoke(false);
        }
        catch (Exception e)
        {
            Debug.LogError($"[Nokov] 断开连接异常: {e.Message}");
        }
    }

    /// <summary>
    /// 切换数据接收模式
    /// </summary>
    public void SwitchReceiveMode(DataReceiveMode mode)
    {
        if (receiveMode == mode) return;

        receiveMode = mode;

        if (!isConnected) return;

        if (mode == DataReceiveMode.Callback)
        {
            CNokovSDK.SetDataCallback(clientHandler, dataCallback, IntPtr.Zero);
            LogDebug("已切换到回调模式");
        }
        else
        {
            // 回调模式下传入null可能无法取消,建议重新连接
            LogDebug($"已切换到轮询模式,频率: {pollingFrequency}Hz");
        }
    }
    #endregion

    #region 数据接收
    /// <summary>
    /// 回调模式数据处理
    /// </summary>
    private static void OnDataReceivedCallback(IntPtr pFrameOfData, IntPtr pUserData)
    {
        if (Instance == null || !Instance.isConnected)
            return;

        try
        {
            // 解析帧数据
            var frame = (sFrameOfMocapData)Marshal.PtrToStructure(
                pFrameOfData, typeof(sFrameOfMocapData));

            // 将解析后的帧数据放入队列，由主线程在 Update() 中处理。
            // 回调线程禁止直接调用Unity API（例如Time/Debug/Transform等）。
            lock (Instance.frameQueueLock)
            {
                Instance.frameQueue.Enqueue(frame);
                
                // 仅在队列刚开始接收数据时输出一次（避免刷屏）
                if (Instance.frameQueue.Count == 1 && Instance.frameCount == 0)
                {
                    System.Console.WriteLine($"[Nokov] 首次接收到帧数据: 帧号={frame.FrameNumber}, 刚体数={frame.RigidBodyCount}");
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[Nokov] 回调数据处理异常: {e.Message}");
        }
    }

    /// <summary>
    /// 轮询模式数据读取
    /// </summary>
    private void PollFrameData()
    {
        if (clientHandler == IntPtr.Zero)
            return;

        try
        {
            IntPtr framePtr = CNokovSDK.GetLastFrameOfMocapData(clientHandler);
            if (framePtr != IntPtr.Zero)
            {
                var frame = (sFrameOfMocapData)Marshal.PtrToStructure(
                    framePtr, typeof(sFrameOfMocapData));

                ProcessFrameData(frame);

                // 释放帧内存
                CNokovSDK.NokovFreeFrame(clientHandler, framePtr);
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[Nokov] 轮询数据处理异常: {e.Message}");
        }
    }

    /// <summary>
    /// 处理帧数据
    /// </summary>
    private void ProcessFrameData(sFrameOfMocapData frame)
    {
        frameCount++;

        // 触发完整帧数据事件
        OnFrameDataReceived?.Invoke(frame);

        // 提取并触发刚体数据事件
        if (frame.RigidBodyCount > 0)
        {
            sRigidBodyData[] rigids = new sRigidBodyData[frame.RigidBodyCount];
            Array.Copy(frame.RigidBodies, rigids, frame.RigidBodyCount);
            OnRigidBodyDataReceived?.Invoke(rigids);
        }
    }

    /// <summary>
    /// 通知消息回调
    /// </summary>
    private static void OnNotifyCallback(IntPtr pNotify, IntPtr pUserData)
    {
        if (Instance == null)
            return;

        try
        {
            var notify = (sNotifyMsg)Marshal.PtrToStructure(
                pNotify, typeof(sNotifyMsg));

            Instance.LogDebug($"SDK通知 - 类型:{notify.Type}, 值:{notify.Value}, " +
                            $"时间戳:{notify.TimeStamp}, 参数:{notify.Param1}");

            Instance.OnNotifyMessage?.Invoke(notify);
        }
        catch (Exception e)
        {
            Debug.LogError($"[Nokov] 通知回调异常: {e.Message}");
        }
    }
    #endregion

    #region 数据描述处理
    /// <summary>
    /// 处理数据描述信息
    /// </summary>
    private void ProcessDataDescriptions(IntPtr pDataDescriptions)
    {
        try
        {
            sDataDescriptions dataDescriptions = (sDataDescriptions)Marshal.PtrToStructure(
                pDataDescriptions, typeof(sDataDescriptions));

            LogDebug($"数据描述数量: {dataDescriptions.DataDescriptionCount}");

            for (Int32 i = 0; i < dataDescriptions.DataDescriptionCount; ++i)
            {
                sDataDescription desc = dataDescriptions.DataDescriptions[i];

                switch (desc.DescriptionType)
                {
                    case (Int32)NokovSDKDataDescriptionType.NokovSDKDataDescriptionType_MarkerSet:
                        ProcessMarkerSetDescription(desc);
                        break;

                    case (Int32)NokovSDKDataDescriptionType.NokovSDKDataDescriptionType_RigidBody:
                        ProcessRigidBodyDescription(desc);
                        break;

                    case (Int32)NokovSDKDataDescriptionType.NokovSDKDataDescriptionType_Skeleton:
                        ProcessSkeletonDescription(desc);
                        break;

                    case (Int32)NokovSDKDataDescriptionType.NokovSDKDataDescriptionType_Param:
                        ProcessParamDescription(desc);
                        break;

                    default:
                        break;
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[Nokov] 处理数据描述异常: {e.Message}");
        }
    }

    private void ProcessMarkerSetDescription(sDataDescription desc)
    {
        sMarkerSetDescription markerSetDesc = (sMarkerSetDescription)Marshal.PtrToStructure(
            (IntPtr)desc.Description, typeof(sMarkerSetDescription));
        
        string name = new string(markerSetDesc.Name).TrimEnd('\0');
        LogDebug($"MarkerSet: {name}, Marker数量: {markerSetDesc.MarkerCount}");
    }

    private void ProcessRigidBodyDescription(sDataDescription desc)
    {
        sRigidBodyDescription rigidBodyDesc = (sRigidBodyDescription)Marshal.PtrToStructure(
            (IntPtr)desc.Description, typeof(sRigidBodyDescription));
        
        string name = new string(rigidBodyDesc.Name).TrimEnd('\0');
        LogDebug($"RigidBody: {name}, ID: {rigidBodyDesc.Id}");
    }

    private void ProcessSkeletonDescription(sDataDescription desc)
    {
        sSkeletonDescription skeletonDesc = (sSkeletonDescription)Marshal.PtrToStructure(
            (IntPtr)desc.Description, typeof(sSkeletonDescription));
        
        string name = new string(skeletonDesc.Name).TrimEnd('\0');
        LogDebug($"Skeleton: {name}, ID: {skeletonDesc.Id}, 骨骼数量: {skeletonDesc.RigidBodyCount}");
    }

    private void ProcessParamDescription(sDataDescription desc)
    {
        sDataParam dataParam = (sDataParam)Marshal.PtrToStructure(
            (IntPtr)desc.Description, typeof(sDataParam));
        
        LogDebug($"帧率: {dataParam.FrameRate}Hz");
    }
    #endregion

    #region 工具方法
    /// <summary>
    /// 条件日志输出
    /// </summary>
    private void LogDebug(string message)
    {
        if (enableDebugLog)
        {
            Debug.Log($"[Nokov] {message}");
        }
    }

    /// <summary>
    /// 获取指定ID的刚体数据
    /// </summary>
    public bool TryGetRigidBodyData(int rigidBodyId, out sRigidBodyData rigidData)
    {
        rigidData = default;

        if (!isConnected || clientHandler == IntPtr.Zero)
            return false;

        // 仅在轮询模式下可用,回调模式需通过事件订阅
        if (receiveMode != DataReceiveMode.Polling)
        {
            Debug.LogWarning("[Nokov] TryGetRigidBodyData仅在轮询模式下可用,回调模式请订阅事件");
            return false;
        }

        try
        {
            IntPtr framePtr = CNokovSDK.GetLastFrameOfMocapData(clientHandler);
            if (framePtr != IntPtr.Zero)
            {
                var frame = (sFrameOfMocapData)Marshal.PtrToStructure(
                    framePtr, typeof(sFrameOfMocapData));

                for (int i = 0; i < frame.RigidBodyCount; i++)
                {
                    if (frame.RigidBodies[i].Id == rigidBodyId)
                    {
                        rigidData = frame.RigidBodies[i];
                        CNokovSDK.NokovFreeFrame(clientHandler, framePtr);
                        return true;
                    }
                }

                CNokovSDK.NokovFreeFrame(clientHandler, framePtr);
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[Nokov] 获取刚体数据异常: {e.Message}");
        }

        return false;
    }

    /// <summary>
    /// 验证位置数据有效性
    /// </summary>
    public static bool IsValidPosition(float x, float y, float z)
    {
        const float INVALID_VALUE = 9999999f;
        return Math.Abs(x - INVALID_VALUE) > 0.1f &&
               Math.Abs(y - INVALID_VALUE) > 0.1f &&
               Math.Abs(z - INVALID_VALUE) > 0.1f;
    }

    /// <summary>
    /// 将SDK刚体数据转换为Unity坐标系
    /// </summary>
    public static void ConvertToUnityCoordinates(sRigidBodyData rigidData, 
        out Vector3 position, out Quaternion rotation, bool invertZ = false)
    {
        // 位置: 毫米 → 米
        position = new Vector3(
            rigidData.X / 1000f,
            rigidData.Y / 1000f,
            (invertZ ? -rigidData.Z : rigidData.Z) / 1000f
        );

        // 四元数转换 (根据坐标系可能需要调整)
        rotation = new Quaternion(
            rigidData.QX,
            rigidData.QY,
            invertZ ? -rigidData.QZ : rigidData.QZ,
            rigidData.QW
        );
    }
    #endregion

    #region GUI显示
    void OnGUI()
    {
        if (!showFrameStats) return;

        GUIStyle style = new GUIStyle();
        style.fontSize = 16;
        style.normal.textColor = isConnected ? Color.green : Color.red;

        string status = isConnected ? "已连接" : "未连接";
        string mode = receiveMode == DataReceiveMode.Callback ? "回调模式" : "轮询模式";
        
        GUI.Label(new Rect(10, 10, 300, 25), 
            $"Nokov状态: {status} | {mode}", style);
        
        if (isConnected)
        {
            style.normal.textColor = Color.white;
            GUI.Label(new Rect(10, 35, 300, 25), 
                $"帧率: {currentFPS:F1} FPS | IP: {serverIP}", style);
        }
    }
    #endregion
}

using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Tracker 位置滤波器组件
/// 
/// 使用 1€ 滤波器 (One Euro Filter) 对 VR Tracker 位置数据进行实时平滑处理
/// 可有效去除高频抖动噪声，同时保持对快速运动的响应
/// 
/// 使用方法:
/// 1. 将此脚本挂载到场景中的任意 GameObject 上
/// 2. 在 ViveTrackerPoseLogger 或其他需要滤波的组件中引用此组件
/// 3. 调用 FilterPosition(deviceId, rawPosition, rawVelocity) 获取滤波后的位置
/// 
/// 参考论文: Casiez, G., Roussel, N., & Vogel, D. (2012). 
/// "1€ filter: a simple speed-based low-pass filter for noisy input in interactive systems"
/// </summary>
public class TrackerPositionFilter : MonoBehaviour
{
    #region Inspector 参数

    [Header("滤波器开关")]
    [Tooltip("是否启用位置滤波（关闭则直接返回原始数据）")]
    public bool enableFilter = true;

    [Header("1€ 滤波器参数")]
    [Tooltip("最小截止频率 (Hz)：控制静止时的平滑强度，越小越平滑但延迟越大")]
    [Range(0.1f, 5.0f)]
    public float minCutoffFrequency = 1.0f;

    [Tooltip("速度系数 (Beta)：控制对速度变化的敏感度，越大对快速移动响应越快")]
    [Range(0.0001f, 1.0f)]
    public float beta = 0.007f;

    [Tooltip("速度估计截止频率 (Hz)：用于平滑速度估计的截止频率")]
    [Range(0.1f, 5.0f)]
    public float derivativeCutoffFrequency = 1.0f;

    [Header("调试选项")]
    [Tooltip("在控制台输出滤波器状态信息")]
    public bool debugLog = false;

    [Tooltip("显示当前自适应截止频率（只读）")]
    [SerializeField]
    private float currentCutoffFrequency = 0f;

    #endregion

    #region 私有字段

    // 每个设备独立的滤波器实例
    private Dictionary<uint, OneEuroFilter3D> deviceFilters = new Dictionary<uint, OneEuroFilter3D>();

    // 单例模式（可选，方便全局访问）
    private static TrackerPositionFilter _instance;
    public static TrackerPositionFilter Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindObjectOfType<TrackerPositionFilter>();
            }
            return _instance;
        }
    }

    #endregion

    #region Unity 生命周期

    void Awake()
    {
        if (_instance == null)
        {
            _instance = this;
        }
        else if (_instance != this)
        {
            Debug.LogWarning("[TrackerPositionFilter] 场景中存在多个 TrackerPositionFilter 实例，建议只保留一个");
        }
    }

    void OnDestroy()
    {
        if (_instance == this)
        {
            _instance = null;
        }
        
        // 清理滤波器
        deviceFilters.Clear();
    }

    void OnValidate()
    {
        // Inspector 参数变化时，更新所有现有滤波器的参数
        UpdateFilterParameters();
    }

    #endregion

    #region 公开接口

    /// <summary>
    /// 对 Tracker 位置进行滤波
    /// </summary>
    /// <param name="deviceId">设备 ID</param>
    /// <param name="rawPositionMm">原始位置（毫米）</param>
    /// <param name="rawVelocityMs">原始速度（米/秒，来自 SteamVR）</param>
    /// <param name="deltaTime">时间间隔（秒）</param>
    /// <returns>滤波后的位置（毫米）</returns>
    public Vector3 FilterPosition(uint deviceId, Vector3 rawPositionMm, Vector3 rawVelocityMs, float deltaTime)
    {
        // 如果滤波器禁用，直接返回原始数据
        if (!enableFilter)
        {
            return rawPositionMm;
        }

        // 获取或创建该设备的滤波器
        if (!deviceFilters.TryGetValue(deviceId, out OneEuroFilter3D filter))
        {
            filter = new OneEuroFilter3D(minCutoffFrequency, beta, derivativeCutoffFrequency);
            deviceFilters[deviceId] = filter;
            
            if (debugLog)
            {
                Debug.Log($"[TrackerPositionFilter] 为设备 {deviceId} 创建滤波器实例");
            }
        }

        // 将速度从 m/s 转换为 mm/s（与位置单位一致）
        Vector3 rawVelocityMmS = rawVelocityMs * 1000f;

        // 执行滤波
        Vector3 filteredPosition = filter.Filter(rawPositionMm, rawVelocityMmS, deltaTime);

        // 更新调试显示
        if (debugLog)
        {
            currentCutoffFrequency = filter.GetCurrentCutoffFrequency();
        }

        return filteredPosition;
    }

    /// <summary>
    /// 对 Tracker 位置进行滤波（简化版，自动使用 Time.deltaTime）
    /// </summary>
    public Vector3 FilterPosition(uint deviceId, Vector3 rawPositionMm, Vector3 rawVelocityMs)
    {
        return FilterPosition(deviceId, rawPositionMm, rawVelocityMs, Time.deltaTime);
    }

    /// <summary>
    /// 重置指定设备的滤波器状态
    /// 在设备断开重连或异常情况下调用
    /// </summary>
    /// <param name="deviceId">设备 ID</param>
    public void ResetFilter(uint deviceId)
    {
        if (deviceFilters.TryGetValue(deviceId, out OneEuroFilter3D filter))
        {
            filter.Reset();
            
            if (debugLog)
            {
                Debug.Log($"[TrackerPositionFilter] 设备 {deviceId} 的滤波器已重置");
            }
        }
    }

    /// <summary>
    /// 重置所有设备的滤波器状态
    /// </summary>
    public void ResetAllFilters()
    {
        foreach (var filter in deviceFilters.Values)
        {
            filter.Reset();
        }
        
        if (debugLog)
        {
            Debug.Log($"[TrackerPositionFilter] 所有滤波器已重置（共 {deviceFilters.Count} 个）");
        }
    }

    /// <summary>
    /// 移除指定设备的滤波器
    /// </summary>
    public void RemoveFilter(uint deviceId)
    {
        if (deviceFilters.Remove(deviceId) && debugLog)
        {
            Debug.Log($"[TrackerPositionFilter] 设备 {deviceId} 的滤波器已移除");
        }
    }

    /// <summary>
    /// 检查滤波器是否已启用
    /// </summary>
    public bool IsFilterEnabled => enableFilter;

    /// <summary>
    /// 获取当前活跃的滤波器数量
    /// </summary>
    public int ActiveFilterCount => deviceFilters.Count;

    #endregion

    #region 私有方法

    /// <summary>
    /// 更新所有滤波器的参数
    /// </summary>
    private void UpdateFilterParameters()
    {
        foreach (var filter in deviceFilters.Values)
        {
            filter.UpdateParameters(minCutoffFrequency, beta, derivativeCutoffFrequency);
        }
    }

    #endregion

    #region 上下文菜单

    [ContextMenu("重置所有滤波器")]
    private void ContextMenuResetAll()
    {
        ResetAllFilters();
    }

    [ContextMenu("输出滤波器状态")]
    private void ContextMenuPrintStatus()
    {
        Debug.Log($"[TrackerPositionFilter] 状态: 启用={enableFilter}, 活跃滤波器数={deviceFilters.Count}");
        Debug.Log($"  参数: fcMin={minCutoffFrequency}Hz, beta={beta}, fcDeriv={derivativeCutoffFrequency}Hz");
        
        foreach (var kvp in deviceFilters)
        {
            Debug.Log($"  设备 {kvp.Key}: 当前截止频率={kvp.Value.GetCurrentCutoffFrequency():F2}Hz");
        }
    }

    #endregion
}

#region 1€ 滤波器实现

/// <summary>
/// 一维 1€ 滤波器
/// </summary>
public class OneEuroFilter1D
{
    // 滤波器参数
    private float minCutoff;      // 最小截止频率 (Hz)
    private float beta;           // 速度系数
    private float derivCutoff;    // 速度估计截止频率 (Hz)

    // 滤波器状态
    private float xFiltered;           // 滤波后的值
    private float velocityFiltered;    // 滤波后的速度
    private bool isFirstFrame;         // 是否是第一帧
    private float lastCutoff;          // 上一次使用的截止频率（用于调试）

    public OneEuroFilter1D(float minCutoff, float beta, float derivCutoff)
    {
        this.minCutoff = minCutoff;
        this.beta = beta;
        this.derivCutoff = derivCutoff;
        Reset();
    }

    /// <summary>
    /// 执行滤波
    /// </summary>
    /// <param name="xRaw">原始值</param>
    /// <param name="velocityRaw">原始速度（来自外部，如 SteamVR）</param>
    /// <param name="dt">时间间隔（秒）</param>
    /// <returns>滤波后的值</returns>
    public float Filter(float xRaw, float velocityRaw, float dt)
    {
        // 防止除零
        if (dt < 1e-6f) dt = 1e-6f;

        // 异常大的时间间隔（如暂停后恢复），重置滤波器
        if (dt > 0.1f)
        {
            Reset();
            xFiltered = xRaw;
            velocityFiltered = velocityRaw;
            isFirstFrame = false;
            return xRaw;
        }

        // 首帧初始化
        if (isFirstFrame)
        {
            xFiltered = xRaw;
            velocityFiltered = velocityRaw;
            isFirstFrame = false;
            return xRaw;
        }

        // 步骤 1：对速度进行低通滤波
        float alphaD = ComputeAlpha(derivCutoff, dt);
        velocityFiltered = velocityFiltered + alphaD * (velocityRaw - velocityFiltered);

        // 步骤 2：计算自适应截止频率
        float cutoff = minCutoff + beta * Mathf.Abs(velocityFiltered);
        lastCutoff = cutoff;

        // 步骤 3：对位置进行低通滤波
        float alpha = ComputeAlpha(cutoff, dt);
        xFiltered = xFiltered + alpha * (xRaw - xFiltered);

        return xFiltered;
    }

    /// <summary>
    /// 计算低通滤波器的平滑因子 alpha
    /// </summary>
    private float ComputeAlpha(float cutoff, float dt)
    {
        float tau = 1f / (2f * Mathf.PI * cutoff);
        return 1f / (1f + tau / dt);
    }

    /// <summary>
    /// 重置滤波器状态
    /// </summary>
    public void Reset()
    {
        isFirstFrame = true;
        xFiltered = 0f;
        velocityFiltered = 0f;
        lastCutoff = minCutoff;
    }

    /// <summary>
    /// 更新滤波器参数
    /// </summary>
    public void UpdateParameters(float minCutoff, float beta, float derivCutoff)
    {
        this.minCutoff = minCutoff;
        this.beta = beta;
        this.derivCutoff = derivCutoff;
    }

    /// <summary>
    /// 获取当前截止频率（用于调试）
    /// </summary>
    public float GetCurrentCutoff() => lastCutoff;
}

/// <summary>
/// 三维 1€ 滤波器（对 x, y, z 分量独立滤波）
/// </summary>
public class OneEuroFilter3D
{
    private OneEuroFilter1D filterX;
    private OneEuroFilter1D filterY;
    private OneEuroFilter1D filterZ;

    public OneEuroFilter3D(float minCutoff, float beta, float derivCutoff)
    {
        filterX = new OneEuroFilter1D(minCutoff, beta, derivCutoff);
        filterY = new OneEuroFilter1D(minCutoff, beta, derivCutoff);
        filterZ = new OneEuroFilter1D(minCutoff, beta, derivCutoff);
    }

    /// <summary>
    /// 对三维向量进行滤波
    /// </summary>
    /// <param name="rawPosition">原始位置</param>
    /// <param name="rawVelocity">原始速度（与位置单位一致）</param>
    /// <param name="dt">时间间隔（秒）</param>
    /// <returns>滤波后的位置</returns>
    public Vector3 Filter(Vector3 rawPosition, Vector3 rawVelocity, float dt)
    {
        return new Vector3(
            filterX.Filter(rawPosition.x, rawVelocity.x, dt),
            filterY.Filter(rawPosition.y, rawVelocity.y, dt),
            filterZ.Filter(rawPosition.z, rawVelocity.z, dt)
        );
    }

    /// <summary>
    /// 重置滤波器状态
    /// </summary>
    public void Reset()
    {
        filterX.Reset();
        filterY.Reset();
        filterZ.Reset();
    }

    /// <summary>
    /// 更新滤波器参数
    /// </summary>
    public void UpdateParameters(float minCutoff, float beta, float derivCutoff)
    {
        filterX.UpdateParameters(minCutoff, beta, derivCutoff);
        filterY.UpdateParameters(minCutoff, beta, derivCutoff);
        filterZ.UpdateParameters(minCutoff, beta, derivCutoff);
    }

    /// <summary>
    /// 获取当前平均截止频率（用于调试）
    /// </summary>
    public float GetCurrentCutoffFrequency()
    {
        return (filterX.GetCurrentCutoff() + filterY.GetCurrentCutoff() + filterZ.GetCurrentCutoff()) / 3f;
    }
}

#endregion

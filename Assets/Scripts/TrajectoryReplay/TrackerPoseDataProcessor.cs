using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Tracker位姿数据后处理器
/// 
/// 功能：
/// - 读取原始CSV位姿数据
/// - 应用多种滤波算法：低通滤波、滑动平均、卡尔曼滤波
/// - 输出处理后的CSV文件
/// 
/// 使用方法：
/// 1. 将脚本挂载到场景中任意GameObject
/// 2. 在Inspector中设置输入CSV文件路径
/// 3. 选择滤波方法和参数
/// 4. 右键点击脚本 → "执行数据后处理"
/// 
/// CSV格式: FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,RX_rad,RY_rad,RZ_rad
/// </summary>
public class TrackerPoseDataProcessor : MonoBehaviour
{
    // ==================== Inspector 配置 ====================
    
    [Header("输入/输出")]
    [Tooltip("输入CSV文件路径（相对于StreamingAssets或绝对路径）")]
    public string inputCsvPath = "TrackerRecordings/input.csv";
    
    [Tooltip("输出文件名后缀")]
    public string outputSuffix = "_filtered";
    
    [Header("滤波方法选择")]
    [Tooltip("选择要应用的滤波方法")]
    public FilterMethod filterMethod = FilterMethod.LowPass;
    
    public enum FilterMethod
    {
        LowPass,            // 低通滤波
        MovingAverage,      // 滑动平均
        Kalman,             // 卡尔曼滤波（静态模型）
        Combined,           // 组合滤波（先滑动平均再低通，去除高频抖动保留轨迹形状）
        QuinticBSpline      // 五次B样条曲线平滑
    }
    
    [Header("低通滤波参数")]
    [Tooltip("平滑系数 α (0-1)，越小越平滑但延迟越大")]
    [Range(0.01f, 1.0f)]
    public float lowPassAlpha = 0.3f;
    
    [Header("滑动平均参数")]
    [Tooltip("滑动窗口大小（帧数），越大越平滑")]
    [Range(3, 50)]
    public int movingAverageWindowSize = 5;
    
    [Header("卡尔曼滤波参数")]
    [Tooltip("过程噪声协方差 Q，越大越信任测量值")]
    [Range(0.0001f, 1.0f)]
    public float kalmanProcessNoise = 0.001f;
    
    [Tooltip("测量噪声协方差 R，越大越信任预测值")]
    [Range(0.001f, 10.0f)]
    public float kalmanMeasurementNoise = 0.1f;
    
    [Tooltip("初始估计误差协方差 P，影响滤波器初始响应速度")]
    [Range(0.01f, 10.0f)]
    public float kalmanInitialErrorCovariance = 1.0f;
    
    [Header("五次B样条参数")]
    [Tooltip("B样条控制点间隔（每隔多少帧取一个控制点）")]
    [Range(1, 20)]
    public int bSplineControlPointInterval = 3;
    
    [Tooltip("B样条采样密度（每个控制点区间的采样数）")]
    [Range(5, 50)]
    public int bSplineSamplingDensity = 10;
    
    [Tooltip("是否保持原始帧数（true=输出与输入帧数相同，false=根据采样密度输出）")]
    public bool bSplineKeepOriginalFrameCount = true;
    
    [Header("处理结果（只读）")]
    [SerializeField] private int processedFrameCount;
    [SerializeField] private string lastProcessTime;
    [SerializeField] private string outputFilePath;
    
    // 数字解析格式
    private static readonly CultureInfo InvariantCulture = CultureInfo.InvariantCulture;
    
    // ==================== 数据结构 ====================
    
    /// <summary>
    /// 原始帧数据（完整CSV行信息）
    /// </summary>
    private class RawFrameData
    {
        public int frameNumber;
        public long timestampMs;
        public double timeFromStartSec;
        public double x_mm, y_mm, z_mm;
        public double qx, qy, qz, qw;
        public double rx_rad, ry_rad, rz_rad;
        
        // 可选的TCP数据字段
        public bool hasTcpData;
        public double tcp_x, tcp_y, tcp_z;
        public double tcp_rx, tcp_ry, tcp_rz;
    }
    
    // ==================== 主入口 ====================
    
    /// <summary>
    /// 通过Inspector右键菜单触发数据处理
    /// </summary>
    [ContextMenu("执行数据后处理")]
    public void ProcessData()
    {
        Debug.Log("========== Tracker位姿数据后处理开始 ==========");
        
        // 1. 读取CSV数据
        List<RawFrameData> rawData = LoadCSV(inputCsvPath);
        if (rawData == null || rawData.Count == 0)
        {
            Debug.LogError("[数据处理] 无法加载CSV数据");
            return;
        }
        
        Debug.Log($"[数据处理] 成功加载 {rawData.Count} 帧数据");
        
        // 2. 应用滤波
        List<RawFrameData> filteredData = ApplyFilter(rawData);
        
        // 3. 保存结果
        SaveFilteredCSV(filteredData);
        
        processedFrameCount = filteredData.Count;
        lastProcessTime = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");
        
        Debug.Log("========== 数据后处理完成 ==========");
    }
    
    // ==================== CSV读取 ====================
    
    /// <summary>
    /// 从CSV文件加载数据
    /// </summary>
    private List<RawFrameData> LoadCSV(string filePath)
    {
        try
        {
            string fullPath = filePath;
            if (!Path.IsPathRooted(filePath))
            {
                fullPath = Path.Combine(Application.streamingAssetsPath, filePath);
            }
            
            if (!File.Exists(fullPath))
            {
                Debug.LogError($"[数据处理] 文件不存在: {fullPath}");
                return null;
            }
            
            string[] lines = File.ReadAllLines(fullPath);
            var data = new List<RawFrameData>();
            
            // 跳过表头
            int startLine = 0;
            for (int i = 0; i < lines.Length; i++)
            {
                string line = lines[i].Trim();
                if (string.IsNullOrEmpty(line) || line.StartsWith("#") || line.StartsWith("FrameNumber"))
                {
                    startLine = i + 1;
                    continue;
                }
                break;
            }
            
            // 解析数据行
            for (int i = startLine; i < lines.Length; i++)
            {
                string line = lines[i].Trim();
                if (string.IsNullOrEmpty(line)) continue;
                
                var frame = ParseCSVLine(line);
                if (frame != null)
                {
                    data.Add(frame);
                }
            }
            
            return data;
        }
        catch (Exception ex)
        {
            Debug.LogError($"[数据处理] 读取CSV失败: {ex.Message}");
            return null;
        }
    }
    
    /// <summary>
    /// 解析CSV单行数据
    /// 格式: FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,RX_rad,RY_rad,RZ_rad[,TCP_X,TCP_Y,TCP_Z,TCP_RX,TCP_RY,TCP_RZ]
    /// </summary>
    private RawFrameData ParseCSVLine(string line)
    {
        try
        {
            string[] parts = line.Split(',');
            if (parts.Length < 13)
            {
                return null;
            }
            
            var frame = new RawFrameData
            {
                frameNumber = int.Parse(parts[0], InvariantCulture),
                timestampMs = long.Parse(parts[1], InvariantCulture),
                timeFromStartSec = double.Parse(parts[2], InvariantCulture),
                x_mm = double.Parse(parts[3], InvariantCulture),
                y_mm = double.Parse(parts[4], InvariantCulture),
                z_mm = double.Parse(parts[5], InvariantCulture),
                qx = double.Parse(parts[6], InvariantCulture),
                qy = double.Parse(parts[7], InvariantCulture),
                qz = double.Parse(parts[8], InvariantCulture),
                qw = double.Parse(parts[9], InvariantCulture),
                rx_rad = double.Parse(parts[10], InvariantCulture),
                ry_rad = double.Parse(parts[11], InvariantCulture),
                rz_rad = double.Parse(parts[12], InvariantCulture)
            };
            
            // 检查是否有TCP数据
            if (parts.Length >= 19)
            {
                frame.hasTcpData = true;
                frame.tcp_x = double.Parse(parts[13], InvariantCulture);
                frame.tcp_y = double.Parse(parts[14], InvariantCulture);
                frame.tcp_z = double.Parse(parts[15], InvariantCulture);
                frame.tcp_rx = double.Parse(parts[16], InvariantCulture);
                frame.tcp_ry = double.Parse(parts[17], InvariantCulture);
                frame.tcp_rz = double.Parse(parts[18], InvariantCulture);
            }
            
            return frame;
        }
        catch
        {
            return null;
        }
    }
    
    // ==================== 滤波算法 ====================
    
    /// <summary>
    /// 根据选择的方法应用滤波
    /// </summary>
    private List<RawFrameData> ApplyFilter(List<RawFrameData> rawData)
    {
        Debug.Log($"[数据处理] 应用滤波方法: {filterMethod}");
        
        switch (filterMethod)
        {
            case FilterMethod.LowPass:
                return ApplyLowPassFilter(rawData);
            
            case FilterMethod.MovingAverage:
                return ApplyMovingAverageFilter(rawData);
            
            case FilterMethod.Kalman:
                return ApplyKalmanFilter(rawData);
            
            case FilterMethod.Combined:
                // 先滑动平均再低通，去除高频抖动同时保留轨迹形状
                var maFiltered = ApplyMovingAverageFilter(rawData);
                return ApplyLowPassFilter(maFiltered);
            
            case FilterMethod.QuinticBSpline:
                return ApplyQuinticBSplineFilter(rawData);
            
            default:
                return rawData;
        }
    }
    
    /// <summary>
    /// 低通滤波器
    /// 公式: filtered[n] = α × current[n] + (1-α) × filtered[n-1]
    /// </summary>
    private List<RawFrameData> ApplyLowPassFilter(List<RawFrameData> rawData)
    {
        if (rawData.Count == 0) return rawData;
        
        var result = new List<RawFrameData>();
        float alpha = lowPassAlpha;
        float oneMinusAlpha = 1f - alpha;
        
        // 第一帧不做滤波
        result.Add(CloneFrame(rawData[0]));
        
        for (int i = 1; i < rawData.Count; i++)
        {
            var current = rawData[i];
            var prevFiltered = result[i - 1];
            
            var filtered = CloneFrame(current);
            
            // 位置滤波
            filtered.x_mm = alpha * current.x_mm + oneMinusAlpha * prevFiltered.x_mm;
            filtered.y_mm = alpha * current.y_mm + oneMinusAlpha * prevFiltered.y_mm;
            filtered.z_mm = alpha * current.z_mm + oneMinusAlpha * prevFiltered.z_mm;
            
            // 旋转向量滤波
            filtered.rx_rad = alpha * current.rx_rad + oneMinusAlpha * prevFiltered.rx_rad;
            filtered.ry_rad = alpha * current.ry_rad + oneMinusAlpha * prevFiltered.ry_rad;
            filtered.rz_rad = alpha * current.rz_rad + oneMinusAlpha * prevFiltered.rz_rad;
            
            // 四元数球面插值（SLERP）
            var q1 = new Quaternion((float)prevFiltered.qx, (float)prevFiltered.qy, 
                                     (float)prevFiltered.qz, (float)prevFiltered.qw);
            var q2 = new Quaternion((float)current.qx, (float)current.qy, 
                                     (float)current.qz, (float)current.qw);
            var qFiltered = Quaternion.Slerp(q1, q2, alpha);
            
            filtered.qx = qFiltered.x;
            filtered.qy = qFiltered.y;
            filtered.qz = qFiltered.z;
            filtered.qw = qFiltered.w;
            
            result.Add(filtered);
        }
        
        Debug.Log($"[低通滤波] 完成，α={alpha:F2}");
        return result;
    }
    
    /// <summary>
    /// 滑动平均滤波器
    /// 对每帧取前后窗口内的平均值
    /// </summary>
    private List<RawFrameData> ApplyMovingAverageFilter(List<RawFrameData> rawData)
    {
        if (rawData.Count == 0) return rawData;
        
        var result = new List<RawFrameData>();
        int halfWindow = movingAverageWindowSize / 2;
        
        for (int i = 0; i < rawData.Count; i++)
        {
            int start = Math.Max(0, i - halfWindow);
            int end = Math.Min(rawData.Count - 1, i + halfWindow);
            int count = end - start + 1;
            
            double sumX = 0, sumY = 0, sumZ = 0;
            double sumRx = 0, sumRy = 0, sumRz = 0;
            
            // 收集四元数用于平均
            var quaternions = new List<Quaternion>();
            
            for (int j = start; j <= end; j++)
            {
                sumX += rawData[j].x_mm;
                sumY += rawData[j].y_mm;
                sumZ += rawData[j].z_mm;
                sumRx += rawData[j].rx_rad;
                sumRy += rawData[j].ry_rad;
                sumRz += rawData[j].rz_rad;
                
                quaternions.Add(new Quaternion(
                    (float)rawData[j].qx, (float)rawData[j].qy,
                    (float)rawData[j].qz, (float)rawData[j].qw));
            }
            
            var filtered = CloneFrame(rawData[i]);
            
            // 位置平均
            filtered.x_mm = sumX / count;
            filtered.y_mm = sumY / count;
            filtered.z_mm = sumZ / count;
            
            // 旋转向量平均
            filtered.rx_rad = sumRx / count;
            filtered.ry_rad = sumRy / count;
            filtered.rz_rad = sumRz / count;
            
            // 四元数平均（使用迭代SLERP方法）
            var avgQuat = AverageQuaternions(quaternions);
            filtered.qx = avgQuat.x;
            filtered.qy = avgQuat.y;
            filtered.qz = avgQuat.z;
            filtered.qw = avgQuat.w;
            
            result.Add(filtered);
        }
        
        Debug.Log($"[滑动平均] 完成，窗口大小={movingAverageWindowSize}");
        return result;
    }
    
    /// <summary>
    /// 卡尔曼滤波器
    /// 对位置和旋转分别进行一维卡尔曼滤波
    /// </summary>
    private List<RawFrameData> ApplyKalmanFilter(List<RawFrameData> rawData)
    {
        if (rawData.Count == 0) return rawData;
        
        var result = new List<RawFrameData>();
        
        // 为每个维度创建卡尔曼滤波器
        var kfX = new SimpleKalmanFilter(kalmanProcessNoise, kalmanMeasurementNoise, rawData[0].x_mm, kalmanInitialErrorCovariance);
        var kfY = new SimpleKalmanFilter(kalmanProcessNoise, kalmanMeasurementNoise, rawData[0].y_mm, kalmanInitialErrorCovariance);
        var kfZ = new SimpleKalmanFilter(kalmanProcessNoise, kalmanMeasurementNoise, rawData[0].z_mm, kalmanInitialErrorCovariance);
        var kfRx = new SimpleKalmanFilter(kalmanProcessNoise, kalmanMeasurementNoise, rawData[0].rx_rad, kalmanInitialErrorCovariance);
        var kfRy = new SimpleKalmanFilter(kalmanProcessNoise, kalmanMeasurementNoise, rawData[0].ry_rad, kalmanInitialErrorCovariance);
        var kfRz = new SimpleKalmanFilter(kalmanProcessNoise, kalmanMeasurementNoise, rawData[0].rz_rad, kalmanInitialErrorCovariance);
        
        // 四元数分量的卡尔曼滤波器
        var kfQx = new SimpleKalmanFilter(kalmanProcessNoise, kalmanMeasurementNoise, rawData[0].qx, kalmanInitialErrorCovariance);
        var kfQy = new SimpleKalmanFilter(kalmanProcessNoise, kalmanMeasurementNoise, rawData[0].qy, kalmanInitialErrorCovariance);
        var kfQz = new SimpleKalmanFilter(kalmanProcessNoise, kalmanMeasurementNoise, rawData[0].qz, kalmanInitialErrorCovariance);
        var kfQw = new SimpleKalmanFilter(kalmanProcessNoise, kalmanMeasurementNoise, rawData[0].qw, kalmanInitialErrorCovariance);
        
        for (int i = 0; i < rawData.Count; i++)
        {
            var current = rawData[i];
            var filtered = CloneFrame(current);
            
            // 位置滤波
            filtered.x_mm = kfX.Update(current.x_mm);
            filtered.y_mm = kfY.Update(current.y_mm);
            filtered.z_mm = kfZ.Update(current.z_mm);
            
            // 旋转向量滤波
            filtered.rx_rad = kfRx.Update(current.rx_rad);
            filtered.ry_rad = kfRy.Update(current.ry_rad);
            filtered.rz_rad = kfRz.Update(current.rz_rad);
            
            // 四元数滤波
            double qx = kfQx.Update(current.qx);
            double qy = kfQy.Update(current.qy);
            double qz = kfQz.Update(current.qz);
            double qw = kfQw.Update(current.qw);
            
            // 归一化四元数
            double qMag = Math.Sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
            if (qMag > 1e-10)
            {
                filtered.qx = qx / qMag;
                filtered.qy = qy / qMag;
                filtered.qz = qz / qMag;
                filtered.qw = qw / qMag;
            }
            
            result.Add(filtered);
        }
        
        Debug.Log($"[卡尔曼滤波] 完成，Q={kalmanProcessNoise:F4}, R={kalmanMeasurementNoise:F4}, P0={kalmanInitialErrorCovariance:F2}");
        return result;
    }
    
    /// <summary>
    /// 五次B样条曲线平滑滤波器
    /// 使用五次（5阶，6次多项式）B样条基函数对轨迹进行平滑拟合
    /// </summary>
    private List<RawFrameData> ApplyQuinticBSplineFilter(List<RawFrameData> rawData)
    {
        if (rawData.Count < 6)
        {
            Debug.LogWarning("[B样条滤波] 数据点数不足6个，无法进行五次B样条拟合");
            return rawData;
        }
        
        var result = new List<RawFrameData>();
        
        // 提取各维度数据作为控制点
        int n = rawData.Count;
        double[] xPoints = new double[n];
        double[] yPoints = new double[n];
        double[] zPoints = new double[n];
        double[] rxPoints = new double[n];
        double[] ryPoints = new double[n];
        double[] rzPoints = new double[n];
        double[] qxPoints = new double[n];
        double[] qyPoints = new double[n];
        double[] qzPoints = new double[n];
        double[] qwPoints = new double[n];
        double[] timePoints = new double[n];
        
        for (int i = 0; i < n; i++)
        {
            xPoints[i] = rawData[i].x_mm;
            yPoints[i] = rawData[i].y_mm;
            zPoints[i] = rawData[i].z_mm;
            rxPoints[i] = rawData[i].rx_rad;
            ryPoints[i] = rawData[i].ry_rad;
            rzPoints[i] = rawData[i].rz_rad;
            qxPoints[i] = rawData[i].qx;
            qyPoints[i] = rawData[i].qy;
            qzPoints[i] = rawData[i].qz;
            qwPoints[i] = rawData[i].qw;
            timePoints[i] = rawData[i].timeFromStartSec;
        }
        
        // 根据控制点间隔选择控制点
        var controlIndices = new List<int>();
        for (int i = 0; i < n; i += bSplineControlPointInterval)
        {
            controlIndices.Add(i);
        }
        // 确保最后一个点被包含
        if (controlIndices[controlIndices.Count - 1] != n - 1)
        {
            controlIndices.Add(n - 1);
        }
        
        int numControlPoints = controlIndices.Count;
        
        if (numControlPoints < 6)
        {
            Debug.LogWarning($"[B样条滤波] 控制点数 ({numControlPoints}) 不足6个，减小控制点间隔或使用其他滤波方法");
            return rawData;
        }
        
        // 提取控制点数据
        double[] ctrlX = new double[numControlPoints];
        double[] ctrlY = new double[numControlPoints];
        double[] ctrlZ = new double[numControlPoints];
        double[] ctrlRx = new double[numControlPoints];
        double[] ctrlRy = new double[numControlPoints];
        double[] ctrlRz = new double[numControlPoints];
        double[] ctrlQx = new double[numControlPoints];
        double[] ctrlQy = new double[numControlPoints];
        double[] ctrlQz = new double[numControlPoints];
        double[] ctrlQw = new double[numControlPoints];
        
        for (int i = 0; i < numControlPoints; i++)
        {
            int idx = controlIndices[i];
            ctrlX[i] = xPoints[idx];
            ctrlY[i] = yPoints[idx];
            ctrlZ[i] = zPoints[idx];
            ctrlRx[i] = rxPoints[idx];
            ctrlRy[i] = ryPoints[idx];
            ctrlRz[i] = rzPoints[idx];
            ctrlQx[i] = qxPoints[idx];
            ctrlQy[i] = qyPoints[idx];
            ctrlQz[i] = qzPoints[idx];
            ctrlQw[i] = qwPoints[idx];
        }
        
        if (bSplineKeepOriginalFrameCount)
        {
            // 保持原始帧数，对每一帧进行B样条插值
            for (int i = 0; i < n; i++)
            {
                // 计算归一化参数 t (0 到 numControlPoints-5)
                double t = (double)i / (n - 1) * (numControlPoints - 5);
                
                var filtered = CloneFrame(rawData[i]);
                
                // 使用五次B样条插值各分量
                filtered.x_mm = EvaluateQuinticBSpline(ctrlX, t);
                filtered.y_mm = EvaluateQuinticBSpline(ctrlY, t);
                filtered.z_mm = EvaluateQuinticBSpline(ctrlZ, t);
                filtered.rx_rad = EvaluateQuinticBSpline(ctrlRx, t);
                filtered.ry_rad = EvaluateQuinticBSpline(ctrlRy, t);
                filtered.rz_rad = EvaluateQuinticBSpline(ctrlRz, t);
                
                // 四元数插值并归一化
                double qx = EvaluateQuinticBSpline(ctrlQx, t);
                double qy = EvaluateQuinticBSpline(ctrlQy, t);
                double qz = EvaluateQuinticBSpline(ctrlQz, t);
                double qw = EvaluateQuinticBSpline(ctrlQw, t);
                
                double qMag = Math.Sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
                if (qMag > 1e-10)
                {
                    filtered.qx = qx / qMag;
                    filtered.qy = qy / qMag;
                    filtered.qz = qz / qMag;
                    filtered.qw = qw / qMag;
                }
                
                result.Add(filtered);
            }
        }
        else
        {
            // 根据采样密度生成新的帧
            int totalSamples = (numControlPoints - 5) * bSplineSamplingDensity + 1;
            double startTime = rawData[0].timeFromStartSec;
            double endTime = rawData[n - 1].timeFromStartSec;
            double duration = endTime - startTime;
            
            for (int i = 0; i < totalSamples; i++)
            {
                double t = (double)i / (totalSamples - 1) * (numControlPoints - 5);
                double normalizedT = (double)i / (totalSamples - 1);
                
                var filtered = new RawFrameData
                {
                    frameNumber = i + 1,
                    timeFromStartSec = startTime + normalizedT * duration,
                    timestampMs = (long)((startTime + normalizedT * duration) * 1000)
                };
                
                // 使用五次B样条插值各分量
                filtered.x_mm = EvaluateQuinticBSpline(ctrlX, t);
                filtered.y_mm = EvaluateQuinticBSpline(ctrlY, t);
                filtered.z_mm = EvaluateQuinticBSpline(ctrlZ, t);
                filtered.rx_rad = EvaluateQuinticBSpline(ctrlRx, t);
                filtered.ry_rad = EvaluateQuinticBSpline(ctrlRy, t);
                filtered.rz_rad = EvaluateQuinticBSpline(ctrlRz, t);
                
                // 四元数插值并归一化
                double qx = EvaluateQuinticBSpline(ctrlQx, t);
                double qy = EvaluateQuinticBSpline(ctrlQy, t);
                double qz = EvaluateQuinticBSpline(ctrlQz, t);
                double qw = EvaluateQuinticBSpline(ctrlQw, t);
                
                double qMag = Math.Sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
                if (qMag > 1e-10)
                {
                    filtered.qx = qx / qMag;
                    filtered.qy = qy / qMag;
                    filtered.qz = qz / qMag;
                    filtered.qw = qw / qMag;
                }
                
                // 检查原始数据是否有TCP数据
                if (rawData[0].hasTcpData)
                {
                    filtered.hasTcpData = true;
                    // TCP数据简单插值（找最近的原始帧）
                    int nearestIdx = (int)Math.Round(normalizedT * (n - 1));
                    nearestIdx = Math.Max(0, Math.Min(n - 1, nearestIdx));
                    filtered.tcp_x = rawData[nearestIdx].tcp_x;
                    filtered.tcp_y = rawData[nearestIdx].tcp_y;
                    filtered.tcp_z = rawData[nearestIdx].tcp_z;
                    filtered.tcp_rx = rawData[nearestIdx].tcp_rx;
                    filtered.tcp_ry = rawData[nearestIdx].tcp_ry;
                    filtered.tcp_rz = rawData[nearestIdx].tcp_rz;
                }
                
                result.Add(filtered);
            }
        }
        
        Debug.Log($"[五次B样条滤波] 完成，控制点间隔={bSplineControlPointInterval}，控制点数={numControlPoints}，输出帧数={result.Count}");
        return result;
    }
    
    /// <summary>
    /// 计算五次B样条基函数值
    /// 五次B样条基函数 N_{i,5}(t)，支撑区间为 [i, i+6)
    /// </summary>
    /// <param name="i">基函数索引</param>
    /// <param name="t">参数值</param>
    /// <returns>基函数在t处的值</returns>
    private double QuinticBSplineBasis(int i, double t)
    {
        // 将t转换为相对于基函数i的局部参数
        double u = t - i;
        
        // 五次B样条基函数定义在 [0, 6) 区间
        if (u < 0 || u >= 6) return 0;
        
        // 五次B样条基函数（均匀节点）
        // 使用Cox-de Boor递推公式预计算的结果
        double u2 = u * u;
        double u3 = u2 * u;
        double u4 = u3 * u;
        double u5 = u4 * u;
        
        if (u < 1)
        {
            // [0, 1): (1/120) * u^5
            return u5 / 120.0;
        }
        else if (u < 2)
        {
            // [1, 2): (1/120) * (-5u^5 + 30u^4 - 60u^3 + 60u^2 - 30u + 6)
            double v = u - 1;
            double v2 = v * v;
            double v3 = v2 * v;
            double v4 = v3 * v;
            double v5 = v4 * v;
            return (1.0 + 5*v + 10*v2 + 10*v3 + 5*v4 - 5*v5) / 120.0;
        }
        else if (u < 3)
        {
            // [2, 3)
            double v = u - 2;
            double v2 = v * v;
            double v3 = v2 * v;
            double v4 = v3 * v;
            double v5 = v4 * v;
            return (26.0 + 50*v + 20*v2 - 20*v3 - 20*v4 + 10*v5) / 120.0;
        }
        else if (u < 4)
        {
            // [3, 4)
            double v = u - 3;
            double v2 = v * v;
            double v3 = v2 * v;
            double v4 = v3 * v;
            double v5 = v4 * v;
            return (66.0 - 60*v2 + 30*v4 - 10*v5) / 120.0;
        }
        else if (u < 5)
        {
            // [4, 5)
            double v = u - 4;
            double v2 = v * v;
            double v3 = v2 * v;
            double v4 = v3 * v;
            double v5 = v4 * v;
            return (26.0 - 50*v + 20*v2 + 20*v3 - 20*v4 + 5*v5) / 120.0;
        }
        else // u < 6
        {
            // [5, 6): (1/120) * (6-u)^5
            double v = 6 - u;
            return v * v * v * v * v / 120.0;
        }
    }
    
    /// <summary>
    /// 使用五次B样条曲线计算插值点
    /// </summary>
    /// <param name="controlPoints">控制点数组</param>
    /// <param name="t">参数值 (0 到 n-5，其中n为控制点数)</param>
    /// <returns>插值结果</returns>
    private double EvaluateQuinticBSpline(double[] controlPoints, double t)
    {
        int n = controlPoints.Length;
        
        // 确保t在有效范围内
        double maxT = n - 5;
        if (maxT < 0) maxT = 0;
        t = Math.Max(0, Math.Min(maxT, t));
        
        // 确定影响当前t值的控制点范围
        int startIdx = (int)Math.Floor(t);
        startIdx = Math.Max(0, Math.Min(n - 6, startIdx));
        
        double result = 0;
        
        // 五次B样条在任意t处最多受6个控制点影响
        for (int i = 0; i < 6 && (startIdx + i) < n; i++)
        {
            int ctrlIdx = startIdx + i;
            double basis = QuinticBSplineBasis(startIdx, t - startIdx + i);
            result += controlPoints[ctrlIdx] * basis;
        }
        
        return result;
    }
    
    // ==================== CSV保存 ====================
    
    /// <summary>
    /// 保存滤波后的数据到CSV
    /// </summary>
    private void SaveFilteredCSV(List<RawFrameData> data)
    {
        try
        {
            // 构建输出路径
            string inputFullPath = inputCsvPath;
            if (!Path.IsPathRooted(inputCsvPath))
            {
                inputFullPath = Path.Combine(Application.streamingAssetsPath, inputCsvPath);
            }
            
            string directory = Path.GetDirectoryName(inputFullPath);
            string fileName = Path.GetFileNameWithoutExtension(inputFullPath);
            string extension = Path.GetExtension(inputFullPath);
            
            outputFilePath = Path.Combine(directory, $"{fileName}{outputSuffix}_{filterMethod}{extension}");
            
            // 确保目录存在
            if (!Directory.Exists(directory))
            {
                Directory.CreateDirectory(directory);
            }
            
            var sb = new StringBuilder();
            
            // 写入注释和表头
            sb.AppendLine($"# 滤波后数据 - 方法: {filterMethod}");
            sb.AppendLine($"# 处理时间: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            sb.AppendLine($"# 原始文件: {inputCsvPath}");
            
            switch (filterMethod)
            {
                case FilterMethod.LowPass:
                    sb.AppendLine($"# 参数: α = {lowPassAlpha:F2}");
                    break;
                case FilterMethod.MovingAverage:
                    sb.AppendLine($"# 参数: 窗口大小 = {movingAverageWindowSize}");
                    break;
                case FilterMethod.Kalman:
                    sb.AppendLine($"# 参数: Q = {kalmanProcessNoise:F4}, R = {kalmanMeasurementNoise:F4}, P0 = {kalmanInitialErrorCovariance:F2}");
                    break;
                case FilterMethod.Combined:
                    sb.AppendLine($"# 参数: Kalman(Q={kalmanProcessNoise:F4}, R={kalmanMeasurementNoise:F4}, P0={kalmanInitialErrorCovariance:F2}) + LowPass(α={lowPassAlpha:F2})");
                    break;
                case FilterMethod.QuinticBSpline:
                    sb.AppendLine($"# 参数: 控制点间隔 = {bSplineControlPointInterval}, 保持原始帧数 = {bSplineKeepOriginalFrameCount}");
                    break;
            }
            
            // 检查是否有TCP数据
            bool hasTcp = data.Count > 0 && data[0].hasTcpData;
            
            if (hasTcp)
            {
                sb.AppendLine("FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,RX_rad,RY_rad,RZ_rad,TCP_X,TCP_Y,TCP_Z,TCP_RX,TCP_RY,TCP_RZ");
            }
            else
            {
                sb.AppendLine("FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,RX_rad,RY_rad,RZ_rad");
            }
            
            // 写入数据行
            foreach (var frame in data)
            {
                string line = string.Format(InvariantCulture,
                    "{0},{1},{2:F6},{3:F4},{4:F4},{5:F4},{6:F6},{7:F6},{8:F6},{9:F6},{10:F6},{11:F6},{12:F6}",
                    frame.frameNumber, frame.timestampMs, frame.timeFromStartSec,
                    frame.x_mm, frame.y_mm, frame.z_mm,
                    frame.qx, frame.qy, frame.qz, frame.qw,
                    frame.rx_rad, frame.ry_rad, frame.rz_rad);
                
                if (hasTcp && frame.hasTcpData)
                {
                    line += string.Format(InvariantCulture,
                        ",{0:F6},{1:F6},{2:F6},{3:F6},{4:F6},{5:F6}",
                        frame.tcp_x, frame.tcp_y, frame.tcp_z,
                        frame.tcp_rx, frame.tcp_ry, frame.tcp_rz);
                }
                
                sb.AppendLine(line);
            }
            
            File.WriteAllText(outputFilePath, sb.ToString());
            Debug.Log($"<color=green>[数据处理] 滤波结果已保存: {outputFilePath}</color>");
        }
        catch (Exception ex)
        {
            Debug.LogError($"[数据处理] 保存CSV失败: {ex.Message}");
        }
    }
    
    // ==================== 辅助方法 ====================
    
    /// <summary>
    /// 克隆帧数据
    /// </summary>
    private RawFrameData CloneFrame(RawFrameData source)
    {
        return new RawFrameData
        {
            frameNumber = source.frameNumber,
            timestampMs = source.timestampMs,
            timeFromStartSec = source.timeFromStartSec,
            x_mm = source.x_mm,
            y_mm = source.y_mm,
            z_mm = source.z_mm,
            qx = source.qx,
            qy = source.qy,
            qz = source.qz,
            qw = source.qw,
            rx_rad = source.rx_rad,
            ry_rad = source.ry_rad,
            rz_rad = source.rz_rad,
            hasTcpData = source.hasTcpData,
            tcp_x = source.tcp_x,
            tcp_y = source.tcp_y,
            tcp_z = source.tcp_z,
            tcp_rx = source.tcp_rx,
            tcp_ry = source.tcp_ry,
            tcp_rz = source.tcp_rz
        };
    }
    
    /// <summary>
    /// 四元数平均（迭代SLERP方法）
    /// </summary>
    private Quaternion AverageQuaternions(List<Quaternion> quaternions)
    {
        if (quaternions.Count == 0) return Quaternion.identity;
        if (quaternions.Count == 1) return quaternions[0];
        
        // 使用迭代SLERP方法计算平均四元数
        Quaternion avg = quaternions[0];
        
        for (int i = 1; i < quaternions.Count; i++)
        {
            // 确保四元数在同一半球（避免插值走"长路"）
            if (Quaternion.Dot(avg, quaternions[i]) < 0)
            {
                quaternions[i] = new Quaternion(-quaternions[i].x, -quaternions[i].y, 
                                                  -quaternions[i].z, -quaternions[i].w);
            }
            
            float t = 1f / (i + 1);
            avg = Quaternion.Slerp(avg, quaternions[i], t);
        }
        
        return avg.normalized;
    }
    
    // ==================== 简易卡尔曼滤波器 ====================
    
    /// <summary>
    /// 一维简易卡尔曼滤波器（静态模型）
    /// 
    /// 状态方程: x[k] = x[k-1] + w (w为过程噪声)
    /// 测量方程: z[k] = x[k] + v (v为测量噪声)
    /// 
    /// 注意: 此为静态模型，适用于抖动抑制。
    /// </summary>
    private class SimpleKalmanFilter
    {
        private double Q; // 过程噪声协方差
        private double R; // 测量噪声协方差
        private double x; // 状态估计值
        private double P; // 估计误差协方差
        
        public SimpleKalmanFilter(double processNoise, double measurementNoise, double initialValue, double initialErrorCovariance = 1.0)
        {
            Q = processNoise;
            R = measurementNoise;
            x = initialValue;
            P = initialErrorCovariance; // 初始估计误差协方差
        }
        
        /// <summary>
        /// 更新滤波器并返回估计值
        /// </summary>
        /// <param name="measurement">新的测量值</param>
        /// <returns>滤波后的估计值</returns>
        public double Update(double measurement)
        {
            // 预测步骤 (Predict)
            // 状态预测: x_pred = x (假设静态模型)
            // 误差协方差预测: P_pred = P + Q
            double P_pred = P + Q;
            
            // 更新步骤 (Update)
            // 卡尔曼增益: K = P_pred / (P_pred + R)
            double K = P_pred / (P_pred + R);
            
            // 状态更新: x = x_pred + K * (measurement - x_pred)
            x = x + K * (measurement - x);
            
            // 误差协方差更新: P = (1 - K) * P_pred
            P = (1.0 - K) * P_pred;
            
            return x;
        }
    }

#if UNITY_EDITOR
    /// <summary>
    /// 编辑器验证 - 确保所有参数在有效范围内
    /// </summary>
    private void OnValidate()
    {
        // 低通滤波参数验证
        lowPassAlpha = Mathf.Clamp(lowPassAlpha, 0.01f, 1.0f);
        
        // 滑动平均参数验证
        movingAverageWindowSize = Mathf.Clamp(movingAverageWindowSize, 3, 50);
        
        // 卡尔曼滤波参数验证
        kalmanProcessNoise = Mathf.Clamp(kalmanProcessNoise, 0.0001f, 1.0f);
        kalmanMeasurementNoise = Mathf.Clamp(kalmanMeasurementNoise, 0.001f, 10.0f);
        kalmanInitialErrorCovariance = Mathf.Clamp(kalmanInitialErrorCovariance, 0.01f, 10.0f);
        
        // B样条参数验证
        bSplineControlPointInterval = Mathf.Clamp(bSplineControlPointInterval, 1, 20);
        bSplineSamplingDensity = Mathf.Clamp(bSplineSamplingDensity, 5, 50);
    }
#endif
}

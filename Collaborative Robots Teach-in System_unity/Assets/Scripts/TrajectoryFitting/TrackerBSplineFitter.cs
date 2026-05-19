using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Tracker 位置 B-样条拟合器
/// 
/// 功能：
/// - 从 CSV 文件读取 Tracker 位姿数据
/// - 使用 ALGLIB 罚函数样条拟合对位置数据（X, Y, Z）进行平滑处理
/// - 姿态数据保持原始值（通过最近邻插值获取）
/// - 输出平滑后的轨迹到新的 CSV 文件
/// 
/// 使用方法：
/// 1. 将此脚本挂载到场景中的 GameObject
/// 2. 在 Inspector 中配置输入/输出路径和拟合参数
/// 3. 点击右键菜单 "执行B样条拟合" 或调用 ExecuteFitting()
/// 
/// 依赖：ALGLIB 库（Assets/Plugins/ALGLIB）
/// </summary>
public class TrackerBSplineFitter : MonoBehaviour
{
    #region Inspector 配置

    [Header("输入配置")]
    [Tooltip("待拟合的 CSV 文件路径（相对于 StreamingAssets）")]
    public string inputCsvPath = "TrackerRecordings/TrackerRecord_2_20260201_100000.csv";

    [Header("拟合参数")]
    [Tooltip("基函数数量（样条节点数）\n0 = 自动计算（根据数据量智能选择）\n越大越贴近原始数据，越小越平滑\n推荐：小数据集(<500)用N/3，大数据集(>1000)用N/5")]
    [Range(0, 2000)]
    public int basisFunctionCount = 0;

    [Tooltip("平滑惩罚系数 λ\n越大越平滑，越小越贴近原始数据\n推荐范围：0.001 ~ 1.0\n高频数据(>500Hz)建议0.001~0.01\n设为负数则自动计算")]
    [Range(-1f, 10f)]
    public float smoothingLambda = -1f;  // -1 = 自动计算

    [Header("输出配置")]
    [Tooltip("输出 CSV 文件路径（相对于 StreamingAssets）\n留空则自动生成")]
    public string outputCsvPath = "";

    [Tooltip("输出采样率 (Hz)\n0 = 与输入相同\n>0 = 重新采样到指定频率")]
    [Range(0, 1000)]
    public float outputSampleRate = 0f;

    [Header("调试选项")]
    [Tooltip("显示详细拟合报告")]
    public bool showFittingReport = true;

    [Tooltip("显示处理进度")]
    public bool showProgress = true;

    #endregion

    #region 内部数据结构

    /// <summary>
    /// 原始轨迹数据
    /// </summary>
    private class RawTrajectoryData
    {
        public double[] time;           // 时间序列 (秒)
        public double[] posX, posY, posZ;   // 位置 (mm)
        public double[] quatX, quatY, quatZ, quatW; // 四元数
        public double[] rotX, rotY, rotZ;   // 旋转矢量 (rad)
        public long[] timeStampMs;      // 时间戳 (ms)
        public int[] frameNumbers;      // 帧号
        
        // TCP 数据（可选）
        public bool hasTcpData;
        public double[] tcpX, tcpY, tcpZ;
        public double[] tcpRX, tcpRY, tcpRZ;
        
        public int count;
    }

    /// <summary>
    /// 拟合报告
    /// </summary>
    private class FittingReport
    {
        public string channel;
        public double rmsError;
        public double maxError;
        public double avgError;
        public int dataPoints;
        public int basisFunctions;
    }

    #endregion

    #region 格式化常量

    private static readonly CultureInfo InvariantCulture = CultureInfo.InvariantCulture;

    #endregion

    #region 公开方法

    /// <summary>
    /// 执行 B-样条拟合
    /// </summary>
    [ContextMenu("执行B样条拟合")]
    public void ExecuteFitting()
    {
        try
        {
            Debug.Log("<color=cyan>==================== B-样条拟合开始 ====================</color>");

            // 1. 加载数据
            if (showProgress) Debug.Log("[拟合] 步骤 1/4: 加载 CSV 数据...");
            RawTrajectoryData rawData = LoadCSVData(inputCsvPath);
            if (rawData == null || rawData.count == 0)
            {
                Debug.LogError("[拟合] 加载数据失败");
                return;
            }
            Debug.Log($"[拟合] 已加载 {rawData.count} 个数据点");

            // 2. 分析数据特征
            var dataStats = AnalyzeDataStatistics(rawData);
            if (showProgress)
            {
                Debug.Log($"[拟合] 数据统计: 位置变化范围 {dataStats.positionRange:F2} mm, 噪声水平估计 {dataStats.noiseLevel:F4} mm");
            }

            // 3. 计算拟合参数
            int M = basisFunctionCount;
            if (M <= 0)
            {
                // 智能计算基函数数量：根据数据量自适应
                // - 小数据集(<200): M = N/3 (保留更多细节)
                // - 中数据集(200~1000): M = N/4
                // - 大数据集(>1000): M = N/5 (平衡精度与计算效率)
                // - 超大数据集(>5000): M = N/6 但不超过1000
                if (rawData.count < 200)
                {
                    M = Mathf.Max(10, rawData.count / 3);
                }
                else if (rawData.count < 1000)
                {
                    M = rawData.count / 4;
                }
                else if (rawData.count < 5000)
                {
                    M = rawData.count / 5;
                }
                else
                {
                    M = Mathf.Min(rawData.count / 6, 1000);
                }
                
                // 确保 M 在合法范围内 (4 <= M <= N)
                M = Mathf.Clamp(M, 4, rawData.count - 1);
                
                Debug.Log($"[拟合] 自动计算基函数数量: M = {M} (数据点 N = {rawData.count}, 比例 {(float)M / rawData.count:P1})");
            }

            // 4. 计算平滑系数 λ
            double lambda = smoothingLambda;
            if (smoothingLambda < 0)
            {
                // 自适应计算 λ：基于噪声水平和数据规模
                // λ ≈ (噪声方差)^2 / (数据点数)
                lambda = Math.Pow(dataStats.noiseLevel, 2) / Math.Sqrt(rawData.count);
                
                // 限制范围：避免过小或过大
                lambda = Math.Max(0.00001, Math.Min(lambda, 0.1));
                
                Debug.Log($"[拟合] 自动计算平滑系数: λ = {lambda:E4} (基于噪声水平 {dataStats.noiseLevel:F4} mm)");
            }
            else
            {
                Debug.Log($"[拟合] 使用指定平滑系数: λ = {lambda}");
            }

            // 5. 执行拟合
            if (showProgress) Debug.Log("[拟合] 步骤 2/4: 执行 B-样条拟合...");
            
            var reports = new List<FittingReport>();
            
            // 拟合 X 轴
            alglib.spline1dinterpolant splineX;
            FittingReport reportX = FitChannel("X", rawData.time, rawData.posX, rawData.count, M, lambda, out splineX);
            reports.Add(reportX);

            // 拟合 Y 轴
            alglib.spline1dinterpolant splineY;
            FittingReport reportY = FitChannel("Y", rawData.time, rawData.posY, rawData.count, M, lambda, out splineY);
            reports.Add(reportY);

            // 拟合 Z 轴
            alglib.spline1dinterpolant splineZ;
            FittingReport reportZ = FitChannel("Z", rawData.time, rawData.posZ, rawData.count, M, lambda, out splineZ);
            reports.Add(reportZ);

            // 4. 生成平滑轨迹
            if (showProgress) Debug.Log("[拟合] 步骤 3/4: 生成平滑轨迹...");
            
            // 确定输出时间点
            double[] outputTime;
            int outputCount;
            
            if (outputSampleRate > 0)
            {
                // 重新采样
                double tMin = rawData.time[0];
                double tMax = rawData.time[rawData.count - 1];
                double dt = 1.0 / outputSampleRate;
                outputCount = (int)Math.Ceiling((tMax - tMin) / dt) + 1;
                outputTime = new double[outputCount];
                for (int i = 0; i < outputCount; i++)
                {
                    outputTime[i] = tMin + i * dt;
                    if (outputTime[i] > tMax) outputTime[i] = tMax;
                }
                Debug.Log($"[拟合] 重新采样: {rawData.count} → {outputCount} 点 ({outputSampleRate} Hz)");
            }
            else
            {
                // 使用原始时间点
                outputCount = rawData.count;
                outputTime = rawData.time;
            }

            // 计算平滑后的位置
            double[] smoothX = new double[outputCount];
            double[] smoothY = new double[outputCount];
            double[] smoothZ = new double[outputCount];

            for (int i = 0; i < outputCount; i++)
            {
                smoothX[i] = alglib.spline1dcalc(splineX, outputTime[i]);
                smoothY[i] = alglib.spline1dcalc(splineY, outputTime[i]);
                smoothZ[i] = alglib.spline1dcalc(splineZ, outputTime[i]);
            }

            // 5. 保存结果
            if (showProgress) Debug.Log("[拟合] 步骤 4/4: 保存结果...");
            
            string outputPath = outputCsvPath;
            if (string.IsNullOrEmpty(outputPath))
            {
                string inputName = Path.GetFileNameWithoutExtension(inputCsvPath);
                outputPath = $"TrackerRecordings/{inputName}_fitted.csv";
            }

            SaveFittedCSV(outputPath, outputTime, smoothX, smoothY, smoothZ, rawData, outputCount);

            // 6. 输出报告
            if (showFittingReport)
            {
                PrintFittingReport(reports, rawData.count, M, lambda, dataStats);
            }

            Debug.Log("<color=green>==================== B-样条拟合完成 ====================</color>");
        }
        catch (Exception ex)
        {
            Debug.LogError($"[拟合] 执行失败: {ex.Message}");
            Debug.LogError($"  堆栈: {ex.StackTrace}");
        }
    }

    #endregion

    #region 数据加载

    /// <summary>
    /// 加载 CSV 数据
    /// </summary>
    private RawTrajectoryData LoadCSVData(string filePath)
    {
        string fullPath = filePath;
        if (!Path.IsPathRooted(filePath))
        {
            fullPath = Path.Combine(Application.streamingAssetsPath, filePath);
        }

        if (!File.Exists(fullPath))
        {
            Debug.LogError($"[拟合] 文件不存在: {fullPath}");
            return null;
        }

        string[] lines = File.ReadAllLines(fullPath);
        if (lines.Length < 2)
        {
            Debug.LogError("[拟合] CSV 数据不足");
            return null;
        }

        // 跳过注释行，找到表头
        int headerLine = 0;
        while (headerLine < lines.Length && lines[headerLine].TrimStart().StartsWith("#"))
        {
            headerLine++;
        }

        if (headerLine >= lines.Length)
        {
            Debug.LogError("[拟合] 未找到有效表头");
            return null;
        }

        // 解析表头，获取列索引
        string[] headers = lines[headerLine].Split(',');
        var colIndex = new Dictionary<string, int>();
        for (int i = 0; i < headers.Length; i++)
        {
            colIndex[headers[i].Trim()] = i;
        }

        // 验证必需列
        string[] requiredCols = { "TimeFromStart_s", "X_mm", "Y_mm", "Z_mm", "QX", "QY", "QZ", "QW" };
        foreach (var col in requiredCols)
        {
            if (!colIndex.ContainsKey(col))
            {
                Debug.LogError($"[拟合] 缺少必需列: {col}");
                return null;
            }
        }

        // 检查可选列
        bool hasRotVec = colIndex.ContainsKey("RX_rad") && colIndex.ContainsKey("RY_rad") && colIndex.ContainsKey("RZ_rad");
        bool hasTcp = colIndex.ContainsKey("TCP_X_m") && colIndex.ContainsKey("TCP_Y_m") && colIndex.ContainsKey("TCP_Z_m");
        bool hasTimeStamp = colIndex.ContainsKey("TimeStamp_ms");
        bool hasFrameNumber = colIndex.ContainsKey("FrameNumber");

        // 预估数据量
        int dataCount = lines.Length - headerLine - 1;
        var data = new RawTrajectoryData
        {
            time = new double[dataCount],
            posX = new double[dataCount],
            posY = new double[dataCount],
            posZ = new double[dataCount],
            quatX = new double[dataCount],
            quatY = new double[dataCount],
            quatZ = new double[dataCount],
            quatW = new double[dataCount],
            rotX = new double[dataCount],
            rotY = new double[dataCount],
            rotZ = new double[dataCount],
            timeStampMs = new long[dataCount],
            frameNumbers = new int[dataCount],
            hasTcpData = hasTcp
        };

        if (hasTcp)
        {
            data.tcpX = new double[dataCount];
            data.tcpY = new double[dataCount];
            data.tcpZ = new double[dataCount];
            data.tcpRX = new double[dataCount];
            data.tcpRY = new double[dataCount];
            data.tcpRZ = new double[dataCount];
        }

        // 解析数据行
        int validCount = 0;
        for (int i = headerLine + 1; i < lines.Length; i++)
        {
            string line = lines[i].Trim();
            if (string.IsNullOrEmpty(line)) continue;

            try
            {
                string[] values = line.Split(',');

                data.time[validCount] = double.Parse(values[colIndex["TimeFromStart_s"]], InvariantCulture);
                data.posX[validCount] = double.Parse(values[colIndex["X_mm"]], InvariantCulture);
                data.posY[validCount] = double.Parse(values[colIndex["Y_mm"]], InvariantCulture);
                data.posZ[validCount] = double.Parse(values[colIndex["Z_mm"]], InvariantCulture);
                data.quatX[validCount] = double.Parse(values[colIndex["QX"]], InvariantCulture);
                data.quatY[validCount] = double.Parse(values[colIndex["QY"]], InvariantCulture);
                data.quatZ[validCount] = double.Parse(values[colIndex["QZ"]], InvariantCulture);
                data.quatW[validCount] = double.Parse(values[colIndex["QW"]], InvariantCulture);

                if (hasRotVec)
                {
                    data.rotX[validCount] = double.Parse(values[colIndex["RX_rad"]], InvariantCulture);
                    data.rotY[validCount] = double.Parse(values[colIndex["RY_rad"]], InvariantCulture);
                    data.rotZ[validCount] = double.Parse(values[colIndex["RZ_rad"]], InvariantCulture);
                }

                if (hasTimeStamp)
                {
                    data.timeStampMs[validCount] = long.Parse(values[colIndex["TimeStamp_ms"]], InvariantCulture);
                }

                if (hasFrameNumber)
                {
                    data.frameNumbers[validCount] = int.Parse(values[colIndex["FrameNumber"]], InvariantCulture);
                }
                else
                {
                    data.frameNumbers[validCount] = validCount;
                }

                if (hasTcp)
                {
                    data.tcpX[validCount] = double.Parse(values[colIndex["TCP_X_m"]], InvariantCulture);
                    data.tcpY[validCount] = double.Parse(values[colIndex["TCP_Y_m"]], InvariantCulture);
                    data.tcpZ[validCount] = double.Parse(values[colIndex["TCP_Z_m"]], InvariantCulture);
                    data.tcpRX[validCount] = double.Parse(values[colIndex["TCP_RX_rad"]], InvariantCulture);
                    data.tcpRY[validCount] = double.Parse(values[colIndex["TCP_RY_rad"]], InvariantCulture);
                    data.tcpRZ[validCount] = double.Parse(values[colIndex["TCP_RZ_rad"]], InvariantCulture);
                }

                validCount++;
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[拟合] 第 {i + 1} 行解析失败: {ex.Message}");
            }
        }

        data.count = validCount;

        // 验证时间单调性
        for (int i = 1; i < validCount; i++)
        {
            if (data.time[i] <= data.time[i - 1])
            {
                Debug.LogWarning($"[拟合] 时间非单调: t[{i - 1}]={data.time[i - 1]:F6}, t[{i}]={data.time[i]:F6}");
            }
        }

        return data;
    }

    #endregion

    #region 数据分析

    /// <summary>
    /// 数据统计信息
    /// </summary>
    private struct DataStatistics
    {
        public double positionRange;   // 位置变化范围 (mm)
        public double noiseLevel;      // 估计的噪声水平 (mm)
        public double avgVelocity;     // 平均速度 (mm/s)
    }

    /// <summary>
    /// 分析数据统计特征
    /// </summary>
    private DataStatistics AnalyzeDataStatistics(RawTrajectoryData data)
    {
        var stats = new DataStatistics();

        // 计算位置范围
        double minX = double.MaxValue, maxX = double.MinValue;
        double minY = double.MaxValue, maxY = double.MinValue;
        double minZ = double.MaxValue, maxZ = double.MinValue;

        for (int i = 0; i < data.count; i++)
        {
            minX = Math.Min(minX, data.posX[i]); maxX = Math.Max(maxX, data.posX[i]);
            minY = Math.Min(minY, data.posY[i]); maxY = Math.Max(maxY, data.posY[i]);
            minZ = Math.Min(minZ, data.posZ[i]); maxZ = Math.Max(maxZ, data.posZ[i]);
        }

        double rangeX = maxX - minX;
        double rangeY = maxY - minY;
        double rangeZ = maxZ - minZ;
        stats.positionRange = Math.Sqrt(rangeX * rangeX + rangeY * rangeY + rangeZ * rangeZ);

        // 估计噪声水平：使用相邻点差分的中位数绝对偏差（MAD）
        var diffs = new List<double>();
        for (int i = 1; i < data.count; i++)
        {
            double dx = data.posX[i] - data.posX[i - 1];
            double dy = data.posY[i] - data.posY[i - 1];
            double dz = data.posZ[i] - data.posZ[i - 1];
            double diff = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            diffs.Add(diff);
        }

        diffs.Sort();
        double medianDiff = diffs[diffs.Count / 2];
        
        // MAD 估计噪声：σ ≈ MAD / 0.6745
        stats.noiseLevel = medianDiff / 0.6745;
        
        // 平均速度
        if (data.count > 1)
        {
            double totalDistance = 0;
            for (int i = 1; i < data.count; i++)
            {
                double dx = data.posX[i] - data.posX[i - 1];
                double dy = data.posY[i] - data.posY[i - 1];
                double dz = data.posZ[i] - data.posZ[i - 1];
                totalDistance += Math.Sqrt(dx * dx + dy * dy + dz * dz);
            }
            double totalTime = data.time[data.count - 1] - data.time[0];
            stats.avgVelocity = totalDistance / totalTime;
        }

        return stats;
    }

    #endregion

    #region B-样条拟合

    /// <summary>
    /// 对单个通道执行 B-样条拟合
    /// </summary>
    private FittingReport FitChannel(string channelName, double[] t, double[] y, int n, int m, double lambda, 
        out alglib.spline1dinterpolant spline)
    {
        var report = new FittingReport
        {
            channel = channelName,
            dataPoints = n,
            basisFunctions = m
        };

        try
        {
            alglib.spline1dfitreport fitReport;
            alglib.spline1dfit(t, y, n, m, lambda, out spline, out fitReport);

            report.rmsError = fitReport.rmserror;
            report.maxError = fitReport.maxerror;
            report.avgError = fitReport.avgerror;

            if (showProgress)
            {
                Debug.Log($"[拟合] {channelName} 轴: RMS={report.rmsError:F4} mm, Max={report.maxError:F4} mm");
            }
        }
        catch (alglib.alglibexception ex)
        {
            Debug.LogError($"[拟合] {channelName} 轴拟合失败: {ex.msg}");
            spline = null;
        }

        return report;
    }

    #endregion

    #region 结果保存

    /// <summary>
    /// 保存拟合后的 CSV 文件
    /// </summary>
    private void SaveFittedCSV(string outputPath, double[] time, double[] smoothX, double[] smoothY, double[] smoothZ,
        RawTrajectoryData rawData, int outputCount)
    {
        string fullPath = outputPath;
        if (!Path.IsPathRooted(outputPath))
        {
            fullPath = Path.Combine(Application.streamingAssetsPath, outputPath);
        }

        // 确保目录存在
        string directory = Path.GetDirectoryName(fullPath);
        if (!Directory.Exists(directory))
        {
            Directory.CreateDirectory(directory);
        }

        var sb = new StringBuilder();

        // 写入表头
        sb.Append("FrameNumber,TimeStamp_ms,TimeFromStart_s,");
        sb.Append("X_mm,Y_mm,Z_mm,");  // 平滑后的位置
        sb.Append("QX,QY,QZ,QW,");
        sb.Append("RX_rad,RY_rad,RZ_rad");
        
        if (rawData.hasTcpData)
        {
            sb.Append(",TCP_X_m,TCP_Y_m,TCP_Z_m,TCP_RX_rad,TCP_RY_rad,TCP_RZ_rad");
        }
        sb.AppendLine();

        // 写入数据
        for (int i = 0; i < outputCount; i++)
        {
            // 找到最近的原始数据点（用于姿态插值）
            int nearestIdx = FindNearestIndex(rawData.time, rawData.count, time[i]);

            // 计算时间戳（基于原始数据插值或推算）
            long timeStampMs;
            if (nearestIdx < rawData.count && rawData.timeStampMs[nearestIdx] > 0)
            {
                // 基于最近点的时间戳 + 时间差
                double timeDiff = time[i] - rawData.time[nearestIdx];
                timeStampMs = rawData.timeStampMs[nearestIdx] + (long)(timeDiff * 1000);
            }
            else
            {
                // 推算时间戳
                timeStampMs = rawData.timeStampMs[0] + (long)(time[i] * 1000);
            }

            sb.Append($"{i},");
            sb.Append($"{timeStampMs},");
            sb.Append(string.Format(InvariantCulture, "{0:F6},", time[i]));
            
            // 平滑后的位置
            sb.Append(string.Format(InvariantCulture, "{0:F4},{1:F4},{2:F4},", smoothX[i], smoothY[i], smoothZ[i]));
            
            // 原始姿态（最近邻插值）
            sb.Append(string.Format(InvariantCulture, "{0:F6},{1:F6},{2:F6},{3:F6},",
                rawData.quatX[nearestIdx], rawData.quatY[nearestIdx], 
                rawData.quatZ[nearestIdx], rawData.quatW[nearestIdx]));
            sb.Append(string.Format(InvariantCulture, "{0:F6},{1:F6},{2:F6}",
                rawData.rotX[nearestIdx], rawData.rotY[nearestIdx], rawData.rotZ[nearestIdx]));

            // TCP 数据（如果有）
            if (rawData.hasTcpData)
            {
                sb.Append(string.Format(InvariantCulture, ",{0:F6},{1:F6},{2:F6},{3:F6},{4:F6},{5:F6}",
                    rawData.tcpX[nearestIdx], rawData.tcpY[nearestIdx], rawData.tcpZ[nearestIdx],
                    rawData.tcpRX[nearestIdx], rawData.tcpRY[nearestIdx], rawData.tcpRZ[nearestIdx]));
            }

            sb.AppendLine();
        }

        File.WriteAllText(fullPath, sb.ToString(), Encoding.UTF8);

        Debug.Log($"<color=green>[拟合] 已保存: {fullPath}</color>");
        Debug.Log($"  输出点数: {outputCount}");
        Debug.Log($"  文件大小: ~{new FileInfo(fullPath).Length / 1024:F1} KB");
    }

    /// <summary>
    /// 查找最近的索引（二分查找）
    /// </summary>
    private int FindNearestIndex(double[] sortedArray, int count, double target)
    {
        if (count == 0) return 0;
        if (target <= sortedArray[0]) return 0;
        if (target >= sortedArray[count - 1]) return count - 1;

        int left = 0, right = count - 1;
        while (left < right - 1)
        {
            int mid = (left + right) / 2;
            if (sortedArray[mid] <= target)
                left = mid;
            else
                right = mid;
        }

        // 返回更接近的那个
        return (target - sortedArray[left] < sortedArray[right] - target) ? left : right;
    }

    #endregion

    #region 报告输出

    /// <summary>
    /// 打印拟合报告
    /// </summary>
    private void PrintFittingReport(List<FittingReport> reports, int dataPoints, int basisFunctions, double lambda, DataStatistics dataStats)
    {
        Debug.Log("========== 拟合质量报告 ==========");
        Debug.Log($"  数据点数: {dataPoints}");
        Debug.Log($"  基函数数: {basisFunctions} (M/N = {(float)basisFunctions / dataPoints:P1})");
        Debug.Log($"  平滑系数: λ = {lambda:E4}");
        Debug.Log($"  噪声水平: {dataStats.noiseLevel:F4} mm");
        Debug.Log("----------------------------------");

        double totalRms = 0;
        double maxMaxError = 0;

        foreach (var report in reports)
        {
            Debug.Log($"  {report.channel} 轴:");
            Debug.Log($"    RMS 误差: {report.rmsError:F4} mm");
            Debug.Log($"    最大误差: {report.maxError:F4} mm");
            Debug.Log($"    平均误差: {report.avgError:F4} mm");

            totalRms += report.rmsError * report.rmsError;
            maxMaxError = Math.Max(maxMaxError, report.maxError);
        }

        double combinedRms = Math.Sqrt(totalRms / reports.Count);
        Debug.Log("----------------------------------");
        Debug.Log($"  综合 RMS: {combinedRms:F4} mm");
        Debug.Log($"  最大误差: {maxMaxError:F4} mm");
        Debug.Log("==================================");

        // 质量警告和参数诊断
        if (combinedRms > 1.0)
        {
            Debug.LogWarning($"[拟合] 警告: RMS 误差较大 ({combinedRms:F4} mm)");
            
            // 诊断：λ 是否太大导致欠拟合
            if (lambda > 0.001)
            {
                Debug.LogWarning($"  → λ 太大 ({lambda:E4})，严重限制了拟合精度");
                Debug.LogWarning($"  → 建议: 降低 λ 到 {lambda * 0.1:E4} 或更小");
            }
            
            // 诊断：M 是否太小
            double mRatio = (double)basisFunctions / dataPoints;
            if (mRatio < 0.15)
            {
                Debug.LogWarning($"  → M/N 比例太小 ({mRatio:P1})，限制了样条灵活性");
                Debug.LogWarning($"  → 建议: 增大 M 到 {(int)(dataPoints * 0.25)} 或更多");
            }
            
            Debug.LogWarning($"  → 重要: λ 太大时，增加 M 几乎无效！请先降低 λ");
        }

        if (maxMaxError > 5.0)
        {
            Debug.LogWarning($"[拟合] 警告: 最大误差较大 ({maxMaxError:F4} mm)，可能存在异常数据点");
        }
        
        // 成功提示
        if (combinedRms < 0.5)
        {
            Debug.Log($"<color=green>[拟合] 拟合质量优秀！RMS < 0.5mm</color>");
        }
        else if (combinedRms < 1.0)
        {
            Debug.Log($"<color=yellow>[拟合] 拟合质量良好，RMS < 1mm</color>");
        }
    }

    #endregion

    #region 编辑器工具

    /// <summary>
    /// 快速测试：使用当前配置执行拟合
    /// </summary>
    [ContextMenu("快速测试拟合")]
    private void QuickTest()
    {
        ExecuteFitting();
    }

    /// <summary>
    /// 重置为默认参数
    /// </summary>
    [ContextMenu("重置为默认参数")]
    private void ResetToDefaults()
    {
        basisFunctionCount = 0;
        smoothingLambda = 0.01f;
        outputSampleRate = 0f;
        showFittingReport = true;
        showProgress = true;
        Debug.Log("[拟合] 已重置为默认参数");
    }

    #endregion
}

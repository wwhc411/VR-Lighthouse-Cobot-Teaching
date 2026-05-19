using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine;

/// <summary>
/// 轨迹弧长重采样器
/// 
/// 功能: 将时间均匀采样的轨迹转换为弧长均匀采样的轨迹
/// 
/// 处理流程:
///   1. 读取经 SG 滤波后的 CSV 轨迹数据
///   2. 计算累积弧长（弦长累积法）
///   3. 使用线性插值进行均匀重采样
///   4. 输出重采样后的 CSV 文件
/// 
/// 适用场景:
///   - Kabsch 点云对齐前的预处理
///   - 轨迹对比分析（需要两条轨迹采样点数相同）
///   - 消除速度变化导致的采样不均匀
/// 
/// 使用方法:
///   1. 将此脚本挂载到任意 GameObject
///   2. 在 Inspector 中设置输入 CSV 文件路径
///   3. 设置目标重采样点数 N
///   4. 右键点击组件标题 → 选择"执行弧长重采样"
/// 
/// 更新日期: 2026-01-29
/// </summary>
public class ArcLengthResampler : MonoBehaviour
{
    #region Inspector 配置字段

    [Header("=== CSV 文件设置 ===")]
    [Tooltip("输入 CSV 文件路径（可以是绝对路径或相对于 StreamingAssets 的路径）\n建议使用经过 SG 滤波后的文件")]
    [SerializeField]
    private string inputCsvPath = "TrackerRecordings/data_filtered.csv";

    [Tooltip("输出 CSV 文件路径（留空则自动在原文件名后加 _resampled）")]
    [SerializeField]
    private string outputCsvPath = "";

    [Header("=== 重采样参数 ===")]
    [Tooltip("目标重采样点数 N\n推荐值: 1000-2000（一般轨迹）\n注意: 两条对比轨迹必须使用相同的 N")]
    [SerializeField]
    [Range(1, 5000)]
    private int targetPointCount = 1000;

    [Header("=== 姿态处理选项 ===")]
    [Tooltip("姿态数据处理方式（仅对位置重采样，姿态使用最近点原始值）")]
    [SerializeField]
    private NearestPointMode nearestPointMode = NearestPointMode.NearestByArcLength;

    [Header("=== 输出选项 ===")]
    [Tooltip("是否在控制台输出详细日志")]
    [SerializeField]
    private bool verboseLogging = true;

    [Tooltip("是否输出验证统计信息")]
    [SerializeField]
    private bool outputValidationStats = true;

    #endregion

    #region 枚举定义

    public enum NearestPointMode
    {
        [Tooltip("使用弧长距离最近的原始点的姿态（推荐）")]
        NearestByArcLength,

        [Tooltip("使用插值区间起点的姿态")]
        IntervalStart,

        [Tooltip("使用插值区间终点的姿态")]
        IntervalEnd
    }

    #endregion

    #region 数据结构

    /// <summary>
    /// 轨迹点数据结构
    /// </summary>
    private struct TrajectoryPoint
    {
        // 原始列
        public int FrameNumber;
        public long TimeStamp_ms;
        public double TimeFromStart_s;

        // 位置 (mm)
        public double X_mm;
        public double Y_mm;
        public double Z_mm;

        // 四元数
        public double QX;
        public double QY;
        public double QZ;
        public double QW;

        // 旋转向量 (rad)
        public double RX_rad;
        public double RY_rad;
        public double RZ_rad;

        // TCP 位姿（可选）
        public double TCP_X_m;
        public double TCP_Y_m;
        public double TCP_Z_m;
        public double TCP_RX_rad;
        public double TCP_RY_rad;
        public double TCP_RZ_rad;

        // 弧长
        public double ArcLength;

        /// <summary>
        /// 获取位置向量
        /// </summary>
        public Vector3 Position => new Vector3((float)X_mm, (float)Y_mm, (float)Z_mm);

        /// <summary>
        /// 获取四元数
        /// </summary>
        public Quaternion Rotation => new Quaternion((float)QX, (float)QY, (float)QZ, (float)QW);
    }

    #endregion

    #region Unity Editor 菜单命令

    [ContextMenu("执行弧长重采样")]
    public void ExecuteResampling()
    {
        try
        {
            // 验证参数
            if (!ValidateParameters())
            {
                return;
            }

            // 读取 CSV
            string fullInputPath = GetFullPath(inputCsvPath);
            if (!File.Exists(fullInputPath))
            {
                Debug.LogError($"[弧长重采样] 输入文件不存在: {fullInputPath}");
                return;
            }

            Log($"开始处理文件: {fullInputPath}");
            Log($"目标重采样点数: {targetPointCount}");

            // 解析 CSV
            var (header, trajectoryPoints) = ReadCsvFile(fullInputPath);
            if (trajectoryPoints.Count == 0)
            {
                Debug.LogError("[弧长重采样] CSV 文件没有数据行");
                return;
            }

            Log($"读取到 {trajectoryPoints.Count} 个原始轨迹点");

            // 步骤 2: 计算累积弧长
            ComputeArcLengths(trajectoryPoints);
            double totalLength = trajectoryPoints[trajectoryPoints.Count - 1].ArcLength;
            Log($"轨迹总弧长: {totalLength:F4} mm");

            // 步骤 3 & 4: 均匀重采样
            var resampledPoints = ResampleTrajectory(trajectoryPoints, targetPointCount);
            Log($"重采样完成，生成 {resampledPoints.Count} 个点");

            // 验证重采样质量
            if (outputValidationStats)
            {
                ValidateResampling(resampledPoints);
            }

            // 输出 CSV
            string fullOutputPath = GetOutputPath(fullInputPath);
            WriteCsv(fullOutputPath, header, resampledPoints);

            Log($"输出文件: {fullOutputPath}");
            Debug.Log($"<color=green>[弧长重采样] 成功! {trajectoryPoints.Count} → {resampledPoints.Count} 个点</color>");
        }
        catch (Exception ex)
        {
            Debug.LogError($"[弧长重采样] 处理失败: {ex.Message}\n{ex.StackTrace}");
        }
    }

    [ContextMenu("预览弧长统计")]
    public void PreviewArcLengthStats()
    {
        try
        {
            string fullInputPath = GetFullPath(inputCsvPath);
            if (!File.Exists(fullInputPath))
            {
                Debug.LogError($"[弧长重采样] 输入文件不存在: {fullInputPath}");
                return;
            }

            var (header, trajectoryPoints) = ReadCsvFile(fullInputPath);
            if (trajectoryPoints.Count == 0) return;

            ComputeArcLengths(trajectoryPoints);

            // 计算相邻点距离统计
            var distances = new List<double>();
            for (int i = 1; i < trajectoryPoints.Count; i++)
            {
                double dist = trajectoryPoints[i].ArcLength - trajectoryPoints[i - 1].ArcLength;
                distances.Add(dist);
            }

            double totalLength = trajectoryPoints[trajectoryPoints.Count - 1].ArcLength;
            double meanDist = distances.Average();
            double stdDist = Math.Sqrt(distances.Select(d => Math.Pow(d - meanDist, 2)).Average());
            double minDist = distances.Min();
            double maxDist = distances.Max();

            Debug.Log($"[弧长统计预览]\n" +
                     $"  原始点数: {trajectoryPoints.Count}\n" +
                     $"  总弧长: {totalLength:F4} mm\n" +
                     $"  相邻点距离:\n" +
                     $"    平均: {meanDist:F4} mm\n" +
                     $"    标准差: {stdDist:F4} mm\n" +
                     $"    最小: {minDist:F4} mm\n" +
                     $"    最大: {maxDist:F4} mm\n" +
                     $"    变异系数(CV): {stdDist / meanDist * 100:F2}%\n" +
                     $"  重采样后每段长度: {totalLength / (targetPointCount - 1):F4} mm");
        }
        catch (Exception ex)
        {
            Debug.LogError($"[弧长统计预览] 失败: {ex.Message}");
        }
    }

    #endregion

    #region 核心算法

    /// <summary>
    /// 步骤 2: 计算累积弧长（弦长累积法）
    /// </summary>
    private void ComputeArcLengths(List<TrajectoryPoint> points)
    {
        // 第一个点弧长为 0
        var firstPoint = points[0];
        firstPoint.ArcLength = 0;
        points[0] = firstPoint;

        for (int i = 1; i < points.Count; i++)
        {
            var prev = points[i - 1];
            var curr = points[i];

            // 计算欧氏距离
            double dx = curr.X_mm - prev.X_mm;
            double dy = curr.Y_mm - prev.Y_mm;
            double dz = curr.Z_mm - prev.Z_mm;
            double segmentLength = Math.Sqrt(dx * dx + dy * dy + dz * dz);

            // 累积弧长
            curr.ArcLength = prev.ArcLength + segmentLength;
            points[i] = curr;
        }
    }

    /// <summary>
    /// 步骤 3 & 4: 均匀重采样
    /// </summary>
    private List<TrajectoryPoint> ResampleTrajectory(List<TrajectoryPoint> originalPoints, int N)
    {
        var resampledPoints = new List<TrajectoryPoint>(N);
        double totalLength = originalPoints[originalPoints.Count - 1].ArcLength;

        // 预先提取弧长数组用于二分查找
        double[] arcLengths = originalPoints.Select(p => p.ArcLength).ToArray();

        for (int j = 0; j < N; j++)
        {
            // 计算目标弧长
            double targetS = (j / (double)(N - 1)) * totalLength;

            // 二分查找区间
            int i = BinarySearchInterval(arcLengths, targetS);

            // 线性插值
            TrajectoryPoint interpolatedPoint = InterpolatePoint(originalPoints, i, targetS);

            // 更新帧号（重新编号）
            interpolatedPoint.FrameNumber = j;

            resampledPoints.Add(interpolatedPoint);
        }

        return resampledPoints;
    }

    /// <summary>
    /// 二分查找：找到 arcLengths[i] <= targetS < arcLengths[i+1] 的索引 i
    /// </summary>
    private int BinarySearchInterval(double[] arcLengths, double targetS)
    {
        int left = 0;
        int right = arcLengths.Length - 1;

        // 边界情况
        if (targetS <= arcLengths[0]) return 0;
        if (targetS >= arcLengths[right]) return right - 1;

        while (left < right - 1)
        {
            int mid = (left + right) / 2;
            if (arcLengths[mid] <= targetS)
            {
                left = mid;
            }
            else
            {
                right = mid;
            }
        }

        return left;
    }

    /// <summary>
    /// 在两点之间进行插值（仅位置，姿态使用最近点原始值）
    /// </summary>
    private TrajectoryPoint InterpolatePoint(List<TrajectoryPoint> points, int i, double targetS)
    {
        // 确保索引有效
        i = Math.Max(0, Math.Min(i, points.Count - 2));

        var p0 = points[i];
        var p1 = points[i + 1];

        // 计算插值参数 t
        double ds = p1.ArcLength - p0.ArcLength;
        double t = ds > 1e-10 ? (targetS - p0.ArcLength) / ds : 0;
        t = Math.Max(0, Math.Min(1, t)); // 限制在 [0, 1]

        var result = new TrajectoryPoint();

        // 时间插值
        result.TimeStamp_ms = (long)(p0.TimeStamp_ms + t * (p1.TimeStamp_ms - p0.TimeStamp_ms));
        result.TimeFromStart_s = p0.TimeFromStart_s + t * (p1.TimeFromStart_s - p0.TimeFromStart_s);

        // 位置线性插值
        result.X_mm = p0.X_mm + t * (p1.X_mm - p0.X_mm);
        result.Y_mm = p0.Y_mm + t * (p1.Y_mm - p0.Y_mm);
        result.Z_mm = p0.Z_mm + t * (p1.Z_mm - p0.Z_mm);

        // 姿态不插值，使用最近点的原始值
        TrajectoryPoint nearestPoint = GetNearestPointForPose(points, i, t);
        
        // 四元数 - 直接复制最近点
        result.QX = nearestPoint.QX;
        result.QY = nearestPoint.QY;
        result.QZ = nearestPoint.QZ;
        result.QW = nearestPoint.QW;

        // 旋转向量 - 直接复制最近点
        result.RX_rad = nearestPoint.RX_rad;
        result.RY_rad = nearestPoint.RY_rad;
        result.RZ_rad = nearestPoint.RZ_rad;

        // TCP 位置插值，TCP 姿态使用最近点
        result.TCP_X_m = p0.TCP_X_m + t * (p1.TCP_X_m - p0.TCP_X_m);
        result.TCP_Y_m = p0.TCP_Y_m + t * (p1.TCP_Y_m - p0.TCP_Y_m);
        result.TCP_Z_m = p0.TCP_Z_m + t * (p1.TCP_Z_m - p0.TCP_Z_m);
        result.TCP_RX_rad = nearestPoint.TCP_RX_rad;
        result.TCP_RY_rad = nearestPoint.TCP_RY_rad;
        result.TCP_RZ_rad = nearestPoint.TCP_RZ_rad;

        // 弧长
        result.ArcLength = targetS;

        return result;
    }

    /// <summary>
    /// 根据配置获取最近点用于姿态复制
    /// </summary>
    private TrajectoryPoint GetNearestPointForPose(List<TrajectoryPoint> points, int intervalIndex, double t)
    {
        switch (nearestPointMode)
        {
            case NearestPointMode.IntervalStart:
                return points[intervalIndex];

            case NearestPointMode.IntervalEnd:
                return points[Math.Min(intervalIndex + 1, points.Count - 1)];

            case NearestPointMode.NearestByArcLength:
            default:
                // t < 0.5 使用起点，t >= 0.5 使用终点
                return t < 0.5 ? points[intervalIndex] : points[Math.Min(intervalIndex + 1, points.Count - 1)];
        }
    }

    #endregion

    #region CSV 读写

    private (string[] header, List<TrajectoryPoint> points) ReadCsvFile(string path)
    {
        var lines = File.ReadAllLines(path);
        if (lines.Length == 0)
        {
            return (Array.Empty<string>(), new List<TrajectoryPoint>());
        }

        string[] header = lines[0].Split(',');
        var points = new List<TrajectoryPoint>();

        // 获取列索引
        int idxFrame = Array.IndexOf(header, "FrameNumber");
        int idxTimeStamp = Array.IndexOf(header, "TimeStamp_ms");
        int idxTimeFromStart = Array.IndexOf(header, "TimeFromStart_s");
        int idxX = Array.IndexOf(header, "X_mm");
        int idxY = Array.IndexOf(header, "Y_mm");
        int idxZ = Array.IndexOf(header, "Z_mm");
        int idxQX = Array.IndexOf(header, "QX");
        int idxQY = Array.IndexOf(header, "QY");
        int idxQZ = Array.IndexOf(header, "QZ");
        int idxQW = Array.IndexOf(header, "QW");
        int idxRX = Array.IndexOf(header, "RX_rad");
        int idxRY = Array.IndexOf(header, "RY_rad");
        int idxRZ = Array.IndexOf(header, "RZ_rad");
        int idxTcpX = Array.IndexOf(header, "TCP_X_m");
        int idxTcpY = Array.IndexOf(header, "TCP_Y_m");
        int idxTcpZ = Array.IndexOf(header, "TCP_Z_m");
        int idxTcpRX = Array.IndexOf(header, "TCP_RX_rad");
        int idxTcpRY = Array.IndexOf(header, "TCP_RY_rad");
        int idxTcpRZ = Array.IndexOf(header, "TCP_RZ_rad");

        for (int i = 1; i < lines.Length; i++)
        {
            if (string.IsNullOrWhiteSpace(lines[i])) continue;

            string[] fields = lines[i].Split(',');

            var point = new TrajectoryPoint
            {
                FrameNumber = TryParseInt(fields, idxFrame, i - 1),
                TimeStamp_ms = TryParseLong(fields, idxTimeStamp, 0),
                TimeFromStart_s = TryParseDouble(fields, idxTimeFromStart, 0),
                X_mm = TryParseDouble(fields, idxX, 0),
                Y_mm = TryParseDouble(fields, idxY, 0),
                Z_mm = TryParseDouble(fields, idxZ, 0),
                QX = TryParseDouble(fields, idxQX, 0),
                QY = TryParseDouble(fields, idxQY, 0),
                QZ = TryParseDouble(fields, idxQZ, 0),
                QW = TryParseDouble(fields, idxQW, 1),
                RX_rad = TryParseDouble(fields, idxRX, 0),
                RY_rad = TryParseDouble(fields, idxRY, 0),
                RZ_rad = TryParseDouble(fields, idxRZ, 0),
                TCP_X_m = TryParseDouble(fields, idxTcpX, 0),
                TCP_Y_m = TryParseDouble(fields, idxTcpY, 0),
                TCP_Z_m = TryParseDouble(fields, idxTcpZ, 0),
                TCP_RX_rad = TryParseDouble(fields, idxTcpRX, 0),
                TCP_RY_rad = TryParseDouble(fields, idxTcpRY, 0),
                TCP_RZ_rad = TryParseDouble(fields, idxTcpRZ, 0),
            };

            points.Add(point);
        }

        return (header, points);
    }

    private void WriteCsv(string path, string[] originalHeader, List<TrajectoryPoint> points)
    {
        var sb = new StringBuilder();

        // 写入表头（保持原始格式，但只保留主要列）
        string[] outputHeader = {
            "FrameNumber", "TimeStamp_ms", "TimeFromStart_s",
            "X_mm", "Y_mm", "Z_mm",
            "QX", "QY", "QZ", "QW",
            "RX_rad", "RY_rad", "RZ_rad",
            "TCP_X_m", "TCP_Y_m", "TCP_Z_m",
            "TCP_RX_rad", "TCP_RY_rad", "TCP_RZ_rad"
        };
        sb.AppendLine(string.Join(",", outputHeader));

        // 写入数据
        foreach (var p in points)
        {
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
                "{0},{1},{2:F6},{3:F6},{4:F6},{5:F6},{6:F6},{7:F6},{8:F6},{9:F6},{10:F6},{11:F6},{12:F6},{13:F6},{14:F6},{15:F6},{16:F6},{17:F6},{18:F6}",
                p.FrameNumber,
                p.TimeStamp_ms,
                p.TimeFromStart_s,
                p.X_mm,
                p.Y_mm,
                p.Z_mm,
                p.QX,
                p.QY,
                p.QZ,
                p.QW,
                p.RX_rad,
                p.RY_rad,
                p.RZ_rad,
                p.TCP_X_m,
                p.TCP_Y_m,
                p.TCP_Z_m,
                p.TCP_RX_rad,
                p.TCP_RY_rad,
                p.TCP_RZ_rad
            ));
        }

        File.WriteAllText(path, sb.ToString(), Encoding.UTF8);
    }

    #endregion

    #region 辅助方法

    private bool ValidateParameters()
    {
        if (string.IsNullOrEmpty(inputCsvPath))
        {
            Debug.LogError("[弧长重采样] 请指定输入 CSV 文件路径");
            return false;
        }

        if (targetPointCount < 2)
        {
            Debug.LogError("[弧长重采样] 目标点数必须至少为 2");
            return false;
        }

        return true;
    }

    private string GetFullPath(string path)
    {
        if (Path.IsPathRooted(path))
        {
            return path;
        }
        return Path.Combine(Application.streamingAssetsPath, path);
    }

    private string GetOutputPath(string inputPath)
    {
        if (!string.IsNullOrEmpty(outputCsvPath))
        {
            return GetFullPath(outputCsvPath);
        }

        string dir = Path.GetDirectoryName(inputPath);
        string name = Path.GetFileNameWithoutExtension(inputPath);
        string ext = Path.GetExtension(inputPath);

        return Path.Combine(dir, $"{name}_resampled_{targetPointCount}{ext}");
    }

    private void Log(string message)
    {
        if (verboseLogging)
        {
            Debug.Log($"[弧长重采样] {message}");
        }
    }

    /// <summary>
    /// 验证重采样质量
    /// </summary>
    private void ValidateResampling(List<TrajectoryPoint> resampledPoints)
    {
        if (resampledPoints.Count < 2) return;

        var distances = new List<double>();
        for (int i = 1; i < resampledPoints.Count; i++)
        {
            var p0 = resampledPoints[i - 1];
            var p1 = resampledPoints[i];
            double dx = p1.X_mm - p0.X_mm;
            double dy = p1.Y_mm - p0.Y_mm;
            double dz = p1.Z_mm - p0.Z_mm;
            double dist = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            distances.Add(dist);
        }

        double meanDist = distances.Average();
        double stdDist = Math.Sqrt(distances.Select(d => Math.Pow(d - meanDist, 2)).Average());
        double cv = stdDist / meanDist * 100;

        string quality;
        if (cv < 5)
            quality = "✅ 优秀";
        else if (cv < 10)
            quality = "✅ 良好";
        else
            quality = "⚠️ 较差";

        Debug.Log($"[弧长重采样] 质量验证:\n" +
                 $"  相邻点平均距离: {meanDist:F4} mm\n" +
                 $"  相邻点距离标准差: {stdDist:F4} mm\n" +
                 $"  均匀性指标 (CV): {cv:F2}%\n" +
                 $"  重采样质量: {quality}");
    }

    private int TryParseInt(string[] fields, int index, int defaultValue)
    {
        if (index < 0 || index >= fields.Length) return defaultValue;
        return int.TryParse(fields[index], out int result) ? result : defaultValue;
    }

    private long TryParseLong(string[] fields, int index, long defaultValue)
    {
        if (index < 0 || index >= fields.Length) return defaultValue;
        return long.TryParse(fields[index], out long result) ? result : defaultValue;
    }

    private double TryParseDouble(string[] fields, int index, double defaultValue)
    {
        if (index < 0 || index >= fields.Length) return defaultValue;
        return double.TryParse(fields[index], NumberStyles.Any, CultureInfo.InvariantCulture, out double result) ? result : defaultValue;
    }

    #endregion

    #region 静态工具方法（供其他脚本调用）

    /// <summary>
    /// 对 Vector3 轨迹进行弧长重采样（静态方法）
    /// </summary>
    /// <param name="originalPoints">原始轨迹点数组</param>
    /// <param name="targetCount">目标重采样点数</param>
    /// <returns>重采样后的轨迹点数组</returns>
    public static Vector3[] ResampleTrajectory(Vector3[] originalPoints, int targetCount)
    {
        if (originalPoints == null || originalPoints.Length < 2)
        {
            Debug.LogError("[弧长重采样] 输入点数不足");
            return originalPoints;
        }

        // 计算累积弧长
        double[] arcLengths = new double[originalPoints.Length];
        arcLengths[0] = 0;

        for (int i = 1; i < originalPoints.Length; i++)
        {
            float dist = Vector3.Distance(originalPoints[i], originalPoints[i - 1]);
            arcLengths[i] = arcLengths[i - 1] + dist;
        }

        double totalLength = arcLengths[arcLengths.Length - 1];

        // 均匀重采样
        Vector3[] resampled = new Vector3[targetCount];

        for (int j = 0; j < targetCount; j++)
        {
            double targetS = (j / (double)(targetCount - 1)) * totalLength;

            // 二分查找
            int i = Array.BinarySearch(arcLengths, targetS);
            if (i < 0) i = ~i - 1;
            i = Math.Max(0, Math.Min(i, originalPoints.Length - 2));

            // 线性插值
            double ds = arcLengths[i + 1] - arcLengths[i];
            float t = ds > 1e-10 ? (float)((targetS - arcLengths[i]) / ds) : 0;
            t = Mathf.Clamp01(t);

            resampled[j] = Vector3.Lerp(originalPoints[i], originalPoints[i + 1], t);
        }

        return resampled;
    }

    /// <summary>
    /// 对位置和四元数轨迹同时进行弧长重采样（静态方法）
    /// 注意：仅对位置进行插值重采样，姿态使用最近点的原始值
    /// </summary>
    public static (Vector3[] positions, Quaternion[] rotations) ResampleTrajectory(
        Vector3[] originalPositions,
        Quaternion[] originalRotations,
        int targetCount)
    {
        if (originalPositions == null || originalPositions.Length < 2)
        {
            Debug.LogError("[弧长重采样] 输入点数不足");
            return (originalPositions, originalRotations);
        }

        if (originalPositions.Length != originalRotations.Length)
        {
            Debug.LogError("[弧长重采样] 位置和旋转数组长度不一致");
            return (originalPositions, originalRotations);
        }

        // 计算累积弧长
        double[] arcLengths = new double[originalPositions.Length];
        arcLengths[0] = 0;

        for (int i = 1; i < originalPositions.Length; i++)
        {
            float dist = Vector3.Distance(originalPositions[i], originalPositions[i - 1]);
            arcLengths[i] = arcLengths[i - 1] + dist;
        }

        double totalLength = arcLengths[arcLengths.Length - 1];

        // 均匀重采样
        Vector3[] resampledPos = new Vector3[targetCount];
        Quaternion[] resampledRot = new Quaternion[targetCount];

        for (int j = 0; j < targetCount; j++)
        {
            double targetS = (j / (double)(targetCount - 1)) * totalLength;

            // 二分查找
            int i = Array.BinarySearch(arcLengths, targetS);
            if (i < 0) i = ~i - 1;
            i = Math.Max(0, Math.Min(i, originalPositions.Length - 2));

            // 线性插值 - 仅位置
            double ds = arcLengths[i + 1] - arcLengths[i];
            float t = ds > 1e-10 ? (float)((targetS - arcLengths[i]) / ds) : 0;
            t = Mathf.Clamp01(t);

            resampledPos[j] = Vector3.Lerp(originalPositions[i], originalPositions[i + 1], t);
            
            // 姿态不插值，使用最近点的原始值
            int nearestIdx = t < 0.5f ? i : Math.Min(i + 1, originalPositions.Length - 1);
            resampledRot[j] = originalRotations[nearestIdx];
        }

        return (resampledPos, resampledRot);
    }

    #endregion
}

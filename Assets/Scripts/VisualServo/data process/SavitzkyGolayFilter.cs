using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

/// <summary>
/// Savitzky-Golay 滤波器 - 基于 Math.NET Numerics 实现
/// 
/// 功能: 对CSV轨迹采样点进行平滑滤波处理
/// 处理列: X_mm, Y_mm, Z_mm (位置) 和 QX, QY, QZ, QW (四元数姿态)
/// 
/// Savitzky-Golay 滤波器优点:
/// - 保留信号的高频特征（峰值形状、边缘等）
/// - 比简单移动平均更好地保持数据趋势
/// - 可同时平滑并计算导数（速度、加速度）
/// 
/// 使用方法:
///   1. 将此脚本挂载到任意GameObject
///   2. 在Inspector中设置输入CSV文件路径
///   3. 调整滤波参数（窗口大小、多项式阶数）
///   4. 右键点击组件标题 → 选择"执行Savitzky-Golay滤波"
/// 
/// 更新日期: 2026-01-28
/// </summary>
public class SavitzkyGolayFilter : MonoBehaviour
{
    #region Inspector 配置字段

    [Header("=== CSV文件设置 ===")]
    [Tooltip("输入CSV文件路径（可以是绝对路径或相对于StreamingAssets的路径）")]
    [SerializeField]
    private string inputCsvPath = "TrackerRecordings/data.csv";

    [Tooltip("输出CSV文件路径（留空则自动在原文件名后加 _filtered）")]
    [SerializeField]
    private string outputCsvPath = "";

    [Header("=== Savitzky-Golay 滤波参数 ===")]
    [Tooltip("滤波窗口大小（必须是奇数，典型值: 5, 7, 9, 11）\n窗口越大平滑效果越强，但可能损失细节")]
    [SerializeField]
    [Range(3, 31)]
    private int windowSize = 7;

    [Tooltip("多项式拟合阶数（必须小于窗口大小，典型值: 2, 3, 4）\n阶数越高保留细节越多，但抗噪能力越弱")]
    [SerializeField]
    [Range(1, 6)]
    private int polynomialOrder = 3;

    [Header("=== 四元数处理选项 ===")]
    [Tooltip("四元数滤波策略")]
    [SerializeField]
    private QuaternionFilterMode quaternionMode = QuaternionFilterMode.DirectFilter;

    [Tooltip("是否在滤波后重新归一化四元数")]
    [SerializeField]
    private bool normalizeQuaternion = true;

    [Header("=== 输出选项 ===")]
    [Tooltip("是否保留原始列（true=追加滤波列，false=替换原始列）")]
    [SerializeField]
    private bool keepOriginalColumns = false;

    [Tooltip("是否在控制台输出详细日志")]
    [SerializeField]
    private bool verboseLogging = true;

    #endregion

    #region 枚举定义

    public enum QuaternionFilterMode
    {
        [Tooltip("直接对QXYZ和QW分别滤波（简单快速，适合小幅度旋转）")]
        DirectFilter,
        
        [Tooltip("转换为轴角表示后滤波，再转回四元数（适合大幅度旋转）")]
        AxisAngleFilter,
        
        [Tooltip("使用SLERP插值进行滤波（保证四元数单位性）")]
        SlerpFilter
    }

    #endregion

    #region 私有变量

    private double[,] _convolutionCoefficients;
    private int _halfWindow;

    #endregion

    #region Unity Editor 菜单命令

    [ContextMenu("执行 Savitzky-Golay 滤波")]
    public void ExecuteFilter()
    {
        try
        {
            // 验证参数
            if (!ValidateParameters())
            {
                return;
            }

            // 计算卷积系数
            ComputeConvolutionCoefficients();

            // 读取CSV
            string fullInputPath = GetFullPath(inputCsvPath);
            if (!File.Exists(fullInputPath))
            {
                Debug.LogError($"[SG滤波器] 输入文件不存在: {fullInputPath}");
                return;
            }

            Log($"开始处理文件: {fullInputPath}");
            Log($"滤波参数: 窗口大小={windowSize}, 多项式阶数={polynomialOrder}");

            // 解析CSV
            var (header, dataRows) = ReadCsvFile(fullInputPath);
            if (dataRows.Count == 0)
            {
                Debug.LogError("[SG滤波器] CSV文件没有数据行");
                return;
            }

            Log($"读取到 {dataRows.Count} 行数据");

            // 获取列索引
            var columnIndices = GetColumnIndices(header);
            if (!columnIndices.HasValue)
            {
                return;
            }

            // 提取需要滤波的数据
            var (positions, quaternions) = ExtractData(dataRows, columnIndices.Value);

            // 执行滤波
            var filteredPositions = FilterPositions(positions);
            var filteredQuaternions = FilterQuaternions(quaternions);

            // 生成输出
            string fullOutputPath = GetOutputPath(fullInputPath);
            WriteFilteredCsv(fullOutputPath, header, dataRows, columnIndices.Value, 
                           filteredPositions, filteredQuaternions);

            Log($"滤波完成! 输出文件: {fullOutputPath}");
            Debug.Log($"<color=green>[SG滤波器] 成功处理 {dataRows.Count} 个采样点</color>");
        }
        catch (Exception ex)
        {
            Debug.LogError($"[SG滤波器] 处理失败: {ex.Message}\n{ex.StackTrace}");
        }
    }

    [ContextMenu("预览滤波效果 (仅位置)")]
    public void PreviewPositionFilter()
    {
        try
        {
            if (!ValidateParameters()) return;
            ComputeConvolutionCoefficients();

            string fullInputPath = GetFullPath(inputCsvPath);
            if (!File.Exists(fullInputPath)) return;

            var (header, dataRows) = ReadCsvFile(fullInputPath);
            var columnIndices = GetColumnIndices(header);
            if (!columnIndices.HasValue) return;

            var (positions, _) = ExtractData(dataRows, columnIndices.Value);
            var filtered = FilterPositions(positions);

            // 计算统计信息
            double totalDiff = 0;
            double maxDiff = 0;
            for (int i = 0; i < positions.Length; i++)
            {
                double diff = Vector3.Distance(
                    new Vector3((float)positions[i].x, (float)positions[i].y, (float)positions[i].z),
                    new Vector3((float)filtered[i].x, (float)filtered[i].y, (float)filtered[i].z)
                );
                totalDiff += diff;
                maxDiff = Math.Max(maxDiff, diff);
            }

            Debug.Log($"[SG滤波预览] 位置变化统计:\n" +
                     $"  平均位移变化: {totalDiff / positions.Length:F4} mm\n" +
                     $"  最大位移变化: {maxDiff:F4} mm\n" +
                     $"  采样点数: {positions.Length}");
        }
        catch (Exception ex)
        {
            Debug.LogError($"[SG滤波预览] 失败: {ex.Message}");
        }
    }

    #endregion

    #region Savitzky-Golay 核心算法

    /// <summary>
    /// 计算 Savitzky-Golay 卷积系数
    /// 基于最小二乘多项式拟合推导的卷积核
    /// </summary>
    private void ComputeConvolutionCoefficients()
    {
        _halfWindow = windowSize / 2;
        int n = windowSize;
        int m = polynomialOrder;

        // 构建 Vandermonde 矩阵 J
        // J[i,j] = i^j, 其中 i = -halfWindow to +halfWindow, j = 0 to polynomialOrder
        var J = DenseMatrix.Create(n, m + 1, (i, j) =>
        {
            int x = i - _halfWindow;
            return Math.Pow(x, j);
        });

        // 计算 (J^T * J)^(-1) * J^T
        // 这给出了平滑和导数的系数
        var JT = J.Transpose();
        var JTJ = JT * J;
        var JTJ_inv = JTJ.Inverse();
        var coeffMatrix = JTJ_inv * JT;

        // 存储系数 - 第0行是平滑系数，第1行是一阶导数系数，以此类推
        _convolutionCoefficients = coeffMatrix.ToArray();

        if (verboseLogging)
        {
            // 输出平滑系数用于验证
            var smoothCoeffs = new double[n];
            for (int i = 0; i < n; i++)
            {
                smoothCoeffs[i] = _convolutionCoefficients[0, i];
            }
            Log($"平滑卷积系数: [{string.Join(", ", smoothCoeffs.Select(c => c.ToString("F6")))}]");
        }
    }

    /// <summary>
    /// 对单个数据序列应用 Savitzky-Golay 滤波
    /// </summary>
    private double[] ApplySGFilter(double[] data)
    {
        int n = data.Length;
        double[] result = new double[n];

        // 获取平滑系数（第0行）
        double[] smoothCoeffs = new double[windowSize];
        for (int i = 0; i < windowSize; i++)
        {
            smoothCoeffs[i] = _convolutionCoefficients[0, i];
        }

        for (int i = 0; i < n; i++)
        {
            double sum = 0;

            // 处理边界情况
            for (int j = 0; j < windowSize; j++)
            {
                int dataIndex = i + j - _halfWindow;

                // 边界反射处理
                if (dataIndex < 0)
                {
                    dataIndex = -dataIndex;
                }
                else if (dataIndex >= n)
                {
                    dataIndex = 2 * n - dataIndex - 2;
                }

                // 确保索引有效
                dataIndex = Math.Max(0, Math.Min(n - 1, dataIndex));
                sum += smoothCoeffs[j] * data[dataIndex];
            }

            result[i] = sum;
        }

        return result;
    }

    #endregion

    #region 位置和四元数滤波

    private (double x, double y, double z)[] FilterPositions((double x, double y, double z)[] positions)
    {
        int n = positions.Length;

        // 分离各轴
        double[] x = positions.Select(p => p.x).ToArray();
        double[] y = positions.Select(p => p.y).ToArray();
        double[] z = positions.Select(p => p.z).ToArray();

        // 分别滤波
        double[] fx = ApplySGFilter(x);
        double[] fy = ApplySGFilter(y);
        double[] fz = ApplySGFilter(z);

        // 合并结果
        var result = new (double x, double y, double z)[n];
        for (int i = 0; i < n; i++)
        {
            result[i] = (fx[i], fy[i], fz[i]);
        }

        return result;
    }

    private (double x, double y, double z, double w)[] FilterQuaternions(
        (double x, double y, double z, double w)[] quaternions)
    {
        switch (quaternionMode)
        {
            case QuaternionFilterMode.DirectFilter:
                return FilterQuaternionsDirect(quaternions);
            case QuaternionFilterMode.AxisAngleFilter:
                return FilterQuaternionsAxisAngle(quaternions);
            case QuaternionFilterMode.SlerpFilter:
                return FilterQuaternionsSlerp(quaternions);
            default:
                return FilterQuaternionsDirect(quaternions);
        }
    }

    /// <summary>
    /// 直接滤波模式：分别对QX, QY, QZ, QW应用滤波
    /// </summary>
    private (double x, double y, double z, double w)[] FilterQuaternionsDirect(
        (double x, double y, double z, double w)[] quaternions)
    {
        int n = quaternions.Length;

        // 确保四元数连续性（避免符号翻转导致的问题）
        var continuous = EnsureQuaternionContinuity(quaternions);

        // 分离各分量
        double[] qx = continuous.Select(q => q.x).ToArray();
        double[] qy = continuous.Select(q => q.y).ToArray();
        double[] qz = continuous.Select(q => q.z).ToArray();
        double[] qw = continuous.Select(q => q.w).ToArray();

        // 分别滤波
        double[] fqx = ApplySGFilter(qx);
        double[] fqy = ApplySGFilter(qy);
        double[] fqz = ApplySGFilter(qz);
        double[] fqw = ApplySGFilter(qw);

        // 合并并归一化
        var result = new (double x, double y, double z, double w)[n];
        for (int i = 0; i < n; i++)
        {
            double x = fqx[i], y = fqy[i], z = fqz[i], w = fqw[i];

            if (normalizeQuaternion)
            {
                double mag = Math.Sqrt(x * x + y * y + z * z + w * w);
                if (mag > 1e-10)
                {
                    x /= mag;
                    y /= mag;
                    z /= mag;
                    w /= mag;
                }
            }

            result[i] = (x, y, z, w);
        }

        return result;
    }

    /// <summary>
    /// 轴角滤波模式：转换为轴角后滤波
    /// </summary>
    private (double x, double y, double z, double w)[] FilterQuaternionsAxisAngle(
        (double x, double y, double z, double w)[] quaternions)
    {
        int n = quaternions.Length;

        // 转换为轴角表示
        double[] angles = new double[n];
        Vector3[] axes = new Vector3[n];

        for (int i = 0; i < n; i++)
        {
            var q = new Quaternion((float)quaternions[i].x, (float)quaternions[i].y,
                                   (float)quaternions[i].z, (float)quaternions[i].w);
            q.ToAngleAxis(out float angle, out Vector3 axis);
            angles[i] = angle * Mathf.Deg2Rad; // 转换为弧度
            axes[i] = axis.normalized;
        }

        // 展开轴角为连续值（处理角度跳变）
        double[] axisX = new double[n];
        double[] axisY = new double[n];
        double[] axisZ = new double[n];

        for (int i = 0; i < n; i++)
        {
            // 使用指数映射：旋转向量 = axis * angle
            axisX[i] = axes[i].x * angles[i];
            axisY[i] = axes[i].y * angles[i];
            axisZ[i] = axes[i].z * angles[i];
        }

        // 滤波旋转向量
        double[] faxisX = ApplySGFilter(axisX);
        double[] faxisY = ApplySGFilter(axisY);
        double[] faxisZ = ApplySGFilter(axisZ);

        // 转回四元数
        var result = new (double x, double y, double z, double w)[n];
        for (int i = 0; i < n; i++)
        {
            double rx = faxisX[i], ry = faxisY[i], rz = faxisZ[i];
            double angle = Math.Sqrt(rx * rx + ry * ry + rz * rz);

            if (angle > 1e-10)
            {
                double halfAngle = angle / 2;
                double sinHalf = Math.Sin(halfAngle);
                double cosHalf = Math.Cos(halfAngle);

                result[i] = (
                    rx / angle * sinHalf,
                    ry / angle * sinHalf,
                    rz / angle * sinHalf,
                    cosHalf
                );
            }
            else
            {
                result[i] = (0, 0, 0, 1); // 单位四元数
            }
        }

        return result;
    }

    /// <summary>
    /// SLERP滤波模式：使用球面线性插值进行平滑
    /// </summary>
    private (double x, double y, double z, double w)[] FilterQuaternionsSlerp(
        (double x, double y, double z, double w)[] quaternions)
    {
        int n = quaternions.Length;
        var result = new (double x, double y, double z, double w)[n];

        // 确保连续性
        var continuous = EnsureQuaternionContinuity(quaternions);

        // 使用加权平均进行滤波
        double[] smoothCoeffs = new double[windowSize];
        for (int i = 0; i < windowSize; i++)
        {
            smoothCoeffs[i] = _convolutionCoefficients[0, i];
        }

        for (int i = 0; i < n; i++)
        {
            double qx = 0, qy = 0, qz = 0, qw = 0;
            double totalWeight = 0;

            for (int j = 0; j < windowSize; j++)
            {
                int dataIndex = i + j - _halfWindow;

                // 边界处理
                if (dataIndex < 0) dataIndex = 0;
                else if (dataIndex >= n) dataIndex = n - 1;

                double weight = Math.Abs(smoothCoeffs[j]);
                totalWeight += weight;

                // 确保与中心四元数同侧
                var q = continuous[dataIndex];
                var center = continuous[i];
                double dot = q.x * center.x + q.y * center.y + q.z * center.z + q.w * center.w;
                double sign = dot >= 0 ? 1 : -1;

                qx += weight * sign * q.x;
                qy += weight * sign * q.y;
                qz += weight * sign * q.z;
                qw += weight * sign * q.w;
            }

            // 归一化
            double mag = Math.Sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
            if (mag > 1e-10)
            {
                result[i] = (qx / mag, qy / mag, qz / mag, qw / mag);
            }
            else
            {
                result[i] = continuous[i];
            }
        }

        return result;
    }

    /// <summary>
    /// 确保四元数序列的连续性（避免q和-q表示同一旋转导致的跳变）
    /// </summary>
    private (double x, double y, double z, double w)[] EnsureQuaternionContinuity(
        (double x, double y, double z, double w)[] quaternions)
    {
        int n = quaternions.Length;
        var result = new (double x, double y, double z, double w)[n];
        result[0] = quaternions[0];

        for (int i = 1; i < n; i++)
        {
            var prev = result[i - 1];
            var curr = quaternions[i];

            // 计算点积
            double dot = prev.x * curr.x + prev.y * curr.y + prev.z * curr.z + prev.w * curr.w;

            // 如果点积为负，翻转当前四元数
            if (dot < 0)
            {
                result[i] = (-curr.x, -curr.y, -curr.z, -curr.w);
            }
            else
            {
                result[i] = curr;
            }
        }

        return result;
    }

    #endregion

    #region CSV 读写

    private (string[] header, List<string[]> dataRows) ReadCsvFile(string path)
    {
        var lines = File.ReadAllLines(path);
        if (lines.Length == 0)
        {
            return (Array.Empty<string>(), new List<string[]>());
        }

        string[] header = lines[0].Split(',');
        var dataRows = new List<string[]>();

        for (int i = 1; i < lines.Length; i++)
        {
            if (!string.IsNullOrWhiteSpace(lines[i]))
            {
                dataRows.Add(lines[i].Split(','));
            }
        }

        return (header, dataRows);
    }

    private struct ColumnIndices
    {
        public int X, Y, Z;
        public int QX, QY, QZ, QW;
    }

    private ColumnIndices? GetColumnIndices(string[] header)
    {
        var indices = new ColumnIndices
        {
            X = Array.IndexOf(header, "X_mm"),
            Y = Array.IndexOf(header, "Y_mm"),
            Z = Array.IndexOf(header, "Z_mm"),
            QX = Array.IndexOf(header, "QX"),
            QY = Array.IndexOf(header, "QY"),
            QZ = Array.IndexOf(header, "QZ"),
            QW = Array.IndexOf(header, "QW")
        };

        if (indices.X < 0 || indices.Y < 0 || indices.Z < 0)
        {
            Debug.LogError("[SG滤波器] CSV缺少位置列 (X_mm, Y_mm, Z_mm)");
            return null;
        }

        if (indices.QX < 0 || indices.QY < 0 || indices.QZ < 0 || indices.QW < 0)
        {
            Debug.LogError("[SG滤波器] CSV缺少四元数列 (QX, QY, QZ, QW)");
            return null;
        }

        return indices;
    }

    private ((double x, double y, double z)[] positions, (double x, double y, double z, double w)[] quaternions)
        ExtractData(List<string[]> dataRows, ColumnIndices indices)
    {
        int n = dataRows.Count;
        var positions = new (double x, double y, double z)[n];
        var quaternions = new (double x, double y, double z, double w)[n];

        for (int i = 0; i < n; i++)
        {
            var row = dataRows[i];

            positions[i] = (
                double.Parse(row[indices.X], CultureInfo.InvariantCulture),
                double.Parse(row[indices.Y], CultureInfo.InvariantCulture),
                double.Parse(row[indices.Z], CultureInfo.InvariantCulture)
            );

            quaternions[i] = (
                double.Parse(row[indices.QX], CultureInfo.InvariantCulture),
                double.Parse(row[indices.QY], CultureInfo.InvariantCulture),
                double.Parse(row[indices.QZ], CultureInfo.InvariantCulture),
                double.Parse(row[indices.QW], CultureInfo.InvariantCulture)
            );
        }

        return (positions, quaternions);
    }

    private void WriteFilteredCsv(string outputPath, string[] originalHeader, List<string[]> dataRows,
                                   ColumnIndices indices,
                                   (double x, double y, double z)[] positions,
                                   (double x, double y, double z, double w)[] quaternions)
    {
        var sb = new StringBuilder();

        if (keepOriginalColumns)
        {
            // 追加滤波列
            var newHeader = originalHeader.ToList();
            newHeader.AddRange(new[] { "X_mm_filtered", "Y_mm_filtered", "Z_mm_filtered",
                                        "QX_filtered", "QY_filtered", "QZ_filtered", "QW_filtered" });
            sb.AppendLine(string.Join(",", newHeader));

            for (int i = 0; i < dataRows.Count; i++)
            {
                var row = dataRows[i].ToList();
                row.Add(positions[i].x.ToString("F6", CultureInfo.InvariantCulture));
                row.Add(positions[i].y.ToString("F6", CultureInfo.InvariantCulture));
                row.Add(positions[i].z.ToString("F6", CultureInfo.InvariantCulture));
                row.Add(quaternions[i].x.ToString("F6", CultureInfo.InvariantCulture));
                row.Add(quaternions[i].y.ToString("F6", CultureInfo.InvariantCulture));
                row.Add(quaternions[i].z.ToString("F6", CultureInfo.InvariantCulture));
                row.Add(quaternions[i].w.ToString("F6", CultureInfo.InvariantCulture));
                sb.AppendLine(string.Join(",", row));
            }
        }
        else
        {
            // 替换原始列
            sb.AppendLine(string.Join(",", originalHeader));

            for (int i = 0; i < dataRows.Count; i++)
            {
                var row = (string[])dataRows[i].Clone();

                // 替换位置
                row[indices.X] = positions[i].x.ToString("F4", CultureInfo.InvariantCulture);
                row[indices.Y] = positions[i].y.ToString("F4", CultureInfo.InvariantCulture);
                row[indices.Z] = positions[i].z.ToString("F4", CultureInfo.InvariantCulture);

                // 替换四元数
                row[indices.QX] = quaternions[i].x.ToString("F6", CultureInfo.InvariantCulture);
                row[indices.QY] = quaternions[i].y.ToString("F6", CultureInfo.InvariantCulture);
                row[indices.QZ] = quaternions[i].z.ToString("F6", CultureInfo.InvariantCulture);
                row[indices.QW] = quaternions[i].w.ToString("F6", CultureInfo.InvariantCulture);

                sb.AppendLine(string.Join(",", row));
            }
        }

        File.WriteAllText(outputPath, sb.ToString(), Encoding.UTF8);
    }

    #endregion

    #region 辅助方法

    private bool ValidateParameters()
    {
        // 窗口大小必须是奇数
        if (windowSize % 2 == 0)
        {
            windowSize++;
            Log($"窗口大小调整为奇数: {windowSize}");
        }

        // 多项式阶数必须小于窗口大小
        if (polynomialOrder >= windowSize)
        {
            polynomialOrder = windowSize - 1;
            Log($"多项式阶数调整为: {polynomialOrder}");
        }

        if (polynomialOrder < 1)
        {
            Debug.LogError("[SG滤波器] 多项式阶数必须至少为1");
            return false;
        }

        if (string.IsNullOrEmpty(inputCsvPath))
        {
            Debug.LogError("[SG滤波器] 请指定输入CSV文件路径");
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

        return Path.Combine(dir, $"{name}_filtered{ext}");
    }

    private void Log(string message)
    {
        if (verboseLogging)
        {
            Debug.Log($"[SG滤波器] {message}");
        }
    }

    #endregion

    #region 静态工具方法（供其他脚本调用）

    /// <summary>
    /// 创建 Savitzky-Golay 滤波器实例（静态工厂方法）
    /// </summary>
    public static double[] ApplySavitzkyGolayFilter(double[] data, int windowSize = 7, int polynomialOrder = 3)
    {
        // 验证参数
        if (windowSize % 2 == 0) windowSize++;
        if (polynomialOrder >= windowSize) polynomialOrder = windowSize - 1;

        int halfWindow = windowSize / 2;

        // 构建 Vandermonde 矩阵
        var J = DenseMatrix.Create(windowSize, polynomialOrder + 1, (i, j) =>
        {
            int x = i - halfWindow;
            return Math.Pow(x, j);
        });

        // 计算系数矩阵
        var JT = J.Transpose();
        var coeffMatrix = (JT * J).Inverse() * JT;
        var coeffArray = coeffMatrix.ToArray();

        // 提取平滑系数
        double[] smoothCoeffs = new double[windowSize];
        for (int i = 0; i < windowSize; i++)
        {
            smoothCoeffs[i] = coeffArray[0, i];
        }

        // 应用滤波
        int n = data.Length;
        double[] result = new double[n];

        for (int i = 0; i < n; i++)
        {
            double sum = 0;
            for (int j = 0; j < windowSize; j++)
            {
                int dataIndex = i + j - halfWindow;
                if (dataIndex < 0) dataIndex = -dataIndex;
                else if (dataIndex >= n) dataIndex = 2 * n - dataIndex - 2;
                dataIndex = Math.Max(0, Math.Min(n - 1, dataIndex));

                sum += smoothCoeffs[j] * data[dataIndex];
            }
            result[i] = sum;
        }

        return result;
    }

    /// <summary>
    /// 对 Vector3 数组应用 Savitzky-Golay 滤波
    /// </summary>
    public static Vector3[] ApplySavitzkyGolayFilter(Vector3[] data, int windowSize = 7, int polynomialOrder = 3)
    {
        double[] x = data.Select(v => (double)v.x).ToArray();
        double[] y = data.Select(v => (double)v.y).ToArray();
        double[] z = data.Select(v => (double)v.z).ToArray();

        double[] fx = ApplySavitzkyGolayFilter(x, windowSize, polynomialOrder);
        double[] fy = ApplySavitzkyGolayFilter(y, windowSize, polynomialOrder);
        double[] fz = ApplySavitzkyGolayFilter(z, windowSize, polynomialOrder);

        Vector3[] result = new Vector3[data.Length];
        for (int i = 0; i < data.Length; i++)
        {
            result[i] = new Vector3((float)fx[i], (float)fy[i], (float)fz[i]);
        }

        return result;
    }

    /// <summary>
    /// 对 Quaternion 数组应用 Savitzky-Golay 滤波
    /// </summary>
    public static Quaternion[] ApplySavitzkyGolayFilter(Quaternion[] data, int windowSize = 7, 
                                                         int polynomialOrder = 3, bool normalize = true)
    {
        // 确保连续性
        Quaternion[] continuous = new Quaternion[data.Length];
        continuous[0] = data[0];
        for (int i = 1; i < data.Length; i++)
        {
            float dot = Quaternion.Dot(continuous[i - 1], data[i]);
            continuous[i] = dot < 0 ? new Quaternion(-data[i].x, -data[i].y, -data[i].z, -data[i].w) : data[i];
        }

        double[] qx = continuous.Select(q => (double)q.x).ToArray();
        double[] qy = continuous.Select(q => (double)q.y).ToArray();
        double[] qz = continuous.Select(q => (double)q.z).ToArray();
        double[] qw = continuous.Select(q => (double)q.w).ToArray();

        double[] fqx = ApplySavitzkyGolayFilter(qx, windowSize, polynomialOrder);
        double[] fqy = ApplySavitzkyGolayFilter(qy, windowSize, polynomialOrder);
        double[] fqz = ApplySavitzkyGolayFilter(qz, windowSize, polynomialOrder);
        double[] fqw = ApplySavitzkyGolayFilter(qw, windowSize, polynomialOrder);

        Quaternion[] result = new Quaternion[data.Length];
        for (int i = 0; i < data.Length; i++)
        {
            Quaternion q = new Quaternion((float)fqx[i], (float)fqy[i], (float)fqz[i], (float)fqw[i]);
            result[i] = normalize ? q.normalized : q;
        }

        return result;
    }

    #endregion
}

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

/// <summary>
/// CSV 轨迹数据读取器
/// 功能：从CSV文件读取Tracker位姿数据，转换为回放系统所需的数据结构
/// CSV格式: FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,RX_rad,RY_rad,RZ_rad
/// 更新日期: 2025-12-02
/// </summary>
public class CSVCaptureReader : MonoBehaviour
{
    // 用于数字解析（避免本地化问题）
    private static readonly CultureInfo InvariantCulture = CultureInfo.InvariantCulture;

    /// <summary>
    /// 从CSV文件加载轨迹数据
    /// </summary>
    /// <param name="filePath">CSV文件路径（绝对路径或相对于StreamingAssets的路径）</param>
    /// <returns>解析后的刚体数据对象，失败返回null</returns>
    public static RigidBodyCaptureData LoadFromCSV(string filePath)
    {
        try
        {
            // 检查路径是否为相对路径，若是则相对于StreamingAssets目录
            string fullPath = filePath;
            if (!Path.IsPathRooted(filePath))
            {
                fullPath = Path.Combine(Application.streamingAssetsPath, filePath);
            }

            // 检查文件是否存在
            if (!File.Exists(fullPath))
            {
                Debug.LogError($"[CSVCaptureReader] 文件不存在: {fullPath}");
                return null;
            }

            // 读取所有行
            string[] lines = File.ReadAllLines(fullPath);

            if (lines.Length < 2)
            {
                Debug.LogError($"[CSVCaptureReader] CSV文件数据不足（至少需要表头和一行数据）: {fullPath}");
                return null;
            }

            // 找到表头行（跳过可能的注释行）
            int headerLineIndex = 0;
            while (headerLineIndex < lines.Length && lines[headerLineIndex].TrimStart().StartsWith("#"))
            {
                headerLineIndex++;
            }
            
            if (headerLineIndex >= lines.Length)
            {
                Debug.LogError($"[CSVCaptureReader] CSV文件没有有效表头: {fullPath}");
                return null;
            }

            // 解析表头，验证格式
            string header = lines[headerLineIndex].Trim();
            if (!ValidateCSVHeader(header, out string headerError))
            {
                Debug.LogError($"[CSVCaptureReader] CSV表头格式错误: {headerError}");
                return null;
            }
            
            int dataStartLine = headerLineIndex + 1;

            // 创建数据结构
            RigidBodyCaptureData data = new RigidBodyCaptureData
            {
                Metadata = new Metadata
                {
                    CollectionTime = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"),
                    TotalFrames = lines.Length - dataStartLine,
                    RigidBodyName = Path.GetFileNameWithoutExtension(fullPath)
                },
                FrameData = new List<FrameData>()
            };

            // 解析数据行
            int validFrameCount = 0;
            int errorCount = 0;

            for (int i = dataStartLine; i < lines.Length; i++)
            {
                string line = lines[i].Trim();
                
                // 跳过空行
                if (string.IsNullOrEmpty(line))
                    continue;

                // 解析单行数据
                FrameData frame = ParseCSVLine(line, i, out string parseError);

                if (frame != null)
                {
                    data.FrameData.Add(frame);
                    validFrameCount++;
                }
                else
                {
                    errorCount++;
                    if (errorCount <= 5) // 只显示前5个错误
                    {
                        Debug.LogWarning($"[CSVCaptureReader] 第{i + 1}行解析失败: {parseError}");
                    }
                }
            }

            // 更新实际帧数
            data.Metadata.TotalFrames = validFrameCount;

            if (validFrameCount == 0)
            {
                Debug.LogError($"[CSV] 没有有效数据行: {fullPath}");
                return null;
            }

            // 精简日志：一行输出关键信息
            string tcpInfo = (validFrameCount > 0 && data.FrameData[0].HasTcpData) ? "含TCP" : "无TCP";
            Debug.Log($"[CSV] 已加载: {data.Metadata.RigidBodyName}, {validFrameCount}帧, {tcpInfo}");
            
            if (errorCount > 0)
            {
                Debug.LogWarning($"[CSV] 解析错误: {errorCount}行");
            }

            return data;
        }
        catch (Exception ex)
        {
            Debug.LogError($"[CSVCaptureReader] 加载CSV失败: {ex.Message}");
            Debug.LogError($"  异常详情: {ex.StackTrace}");
            return null;
        }
    }

    /// <summary>
    /// 验证CSV表头格式
    /// </summary>
    private static bool ValidateCSVHeader(string header, out string errorMessage)
    {
        errorMessage = "";

        // 必需列
        string[] requiredColumns = { "FrameNumber", "X_mm", "Y_mm", "Z_mm", "QX", "QY", "QZ", "QW" };
        string headerLower = header.ToLower();

        foreach (string col in requiredColumns)
        {
            if (!headerLower.Contains(col.ToLower()))
            {
                errorMessage = $"缺少必需列: {col}";
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// 解析CSV单行数据
    /// CSV格式: FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,RX_rad,RY_rad,RZ_rad
    /// </summary>
    private static FrameData ParseCSVLine(string line, int lineNumber, out string errorMessage)
    {
        errorMessage = "";

        try
        {
            string[] values = line.Split(',');

            // 检查列数（至少需要10列：帧号+时间戳+时间+位置3+四元数4）
            if (values.Length < 10)
            {
                errorMessage = $"列数不足: 期望至少10列, 实际{values.Length}列";
                return null;
            }

            // 解析各字段
            int frameNumber = int.Parse(values[0].Trim(), InvariantCulture);
            long timeStampMs = long.Parse(values[1].Trim(), InvariantCulture);
            double timeFromStart = double.Parse(values[2].Trim(), InvariantCulture);

            double x = double.Parse(values[3].Trim(), InvariantCulture);
            double y = double.Parse(values[4].Trim(), InvariantCulture);
            double z = double.Parse(values[5].Trim(), InvariantCulture);

            double qx = double.Parse(values[6].Trim(), InvariantCulture);
            double qy = double.Parse(values[7].Trim(), InvariantCulture);
            double qz = double.Parse(values[8].Trim(), InvariantCulture);
            double qw = double.Parse(values[9].Trim(), InvariantCulture);

            // 旋转矢量是可选的（如果有13列则包含）
            double rx = 0, ry = 0, rz = 0;
            if (values.Length >= 13)
            {
                rx = double.Parse(values[10].Trim(), InvariantCulture);
                ry = double.Parse(values[11].Trim(), InvariantCulture);
                rz = double.Parse(values[12].Trim(), InvariantCulture);
            }

            // TCP数据是可选的（如果有19列则包含）
            // 列13-18: TCP_X_m, TCP_Y_m, TCP_Z_m, TCP_RX_rad, TCP_RY_rad, TCP_RZ_rad
            bool hasTcpData = values.Length >= 19;
            TcpPoseData tcpPose = null;
            if (hasTcpData)
            {
                tcpPose = new TcpPoseData
                {
                    X = double.Parse(values[13].Trim(), InvariantCulture),
                    Y = double.Parse(values[14].Trim(), InvariantCulture),
                    Z = double.Parse(values[15].Trim(), InvariantCulture),
                    RX = double.Parse(values[16].Trim(), InvariantCulture),
                    RY = double.Parse(values[17].Trim(), InvariantCulture),
                    RZ = double.Parse(values[18].Trim(), InvariantCulture)
                };
            }

            // 创建帧数据
            FrameData frame = new FrameData
            {
                FrameNumber = frameNumber,
                UnixTimeStamp = timeStampMs,

                Position = new PositionData
                {
                    X = x,
                    Y = y,
                    Z = z
                },

                Quaternion = new QuaternionData
                {
                    X = qx,
                    Y = qy,
                    Z = qz,
                    W = qw
                },
                
                HasTcpData = hasTcpData,
                TcpPose = tcpPose
            };

            return frame;
        }
        catch (FormatException ex)
        {
            errorMessage = $"数值格式错误: {ex.Message}";
            return null;
        }
        catch (Exception ex)
        {
            errorMessage = $"解析异常: {ex.Message}";
            return null;
        }
    }

    /// <summary>
    /// 验证CSV文件格式是否有效
    /// </summary>
    public static bool ValidateCSVFile(string filePath, out string errorMessage)
    {
        errorMessage = "";

        try
        {
            RigidBodyCaptureData data = LoadFromCSV(filePath);

            if (data == null)
            {
                errorMessage = "数据为空或加载失败";
                return false;
            }

            int validFrames = 0;
            for (int i = 0; i < data.FrameData.Count; i++)
            {
                FrameData frame = data.FrameData[i];

                if (frame.IsPositionValid())
                {
                    validFrames++;

                    if (double.IsNaN(frame.Position.X) || double.IsNaN(frame.Position.Y) || double.IsNaN(frame.Position.Z))
                    {
                        errorMessage = $"帧{i}的位置数据包含无效值(NaN)";
                        return false;
                    }

                    if (double.IsNaN(frame.Quaternion.X) || double.IsNaN(frame.Quaternion.Y) ||
                        double.IsNaN(frame.Quaternion.Z) || double.IsNaN(frame.Quaternion.W))
                    {
                        errorMessage = $"帧{i}的四元数包含无效值(NaN)";
                        return false;
                    }
                }
            }

            Debug.Log($"[CSVCaptureReader] 验证完成: 总帧数{data.FrameData.Count}, 有效帧数{validFrames}");

            errorMessage = "验证通过";
            return true;
        }
        catch (Exception ex)
        {
            errorMessage = $"验证异常: {ex.Message}";
            return false;
        }
    }

    /// <summary>
    /// 获取指定帧序号的数据
    /// </summary>
    public static FrameData GetFrameByIndex(RigidBodyCaptureData data, int frameIndex)
    {
        if (data == null || data.FrameData == null)
        {
            Debug.LogError("[CSVCaptureReader] 数据为空");
            return null;
        }

        if (frameIndex < 0 || frameIndex >= data.FrameData.Count)
        {
            Debug.LogError($"[CSVCaptureReader] 帧序号超出范围: {frameIndex} (总帧数: {data.FrameData.Count})");
            return null;
        }

        return data.FrameData[frameIndex];
    }
}

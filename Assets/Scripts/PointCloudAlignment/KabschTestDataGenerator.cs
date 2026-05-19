using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

/// <summary>
/// 测试数据生成器 - 生成用于测试Kabsch算法的点云CSV文件
/// </summary>
public class KabschTestDataGenerator : MonoBehaviour
{
    [Header("生成参数")]
    [Tooltip("生成的轨迹点数")]
    public int pointCount = 100;

    [Tooltip("轨迹类型")]
    public TrajectoryType trajectoryType = TrajectoryType.Helix;

    [Header("变换参数")]
    [Tooltip("旋转角度（度）")]
    public Vector3 rotationAngles = new Vector3(15f, 30f, 10f);

    [Tooltip("平移向量")]
    public Vector3 translation = new Vector3(0.5f, 0.3f, 0.2f);

    [Header("噪声参数")]
    [Tooltip("是否添加噪声")]
    public bool addNoise = false;

    [Tooltip("噪声标准差")]
    public float noiseStdDev = 0.001f;

    [Header("输出路径")]
    public string outputFolder = "PointClouds";
    public string sourceFileName = "source_trajectory.csv";
    public string targetFileName = "target_trajectory.csv";

    public enum TrajectoryType
    {
        Helix,          // 螺旋线
        Circle,         // 圆形
        Line,           // 直线
        SineWave,       // 正弦波
        Figure8,        // 8字形
        Random          // 随机点
    }

    /// <summary>
    /// 生成测试数据
    /// </summary>
    [ContextMenu("生成测试点云数据")]
    public void GenerateTestData()
    {
        // 生成源点云（原始轨迹）
        List<Vector3> sourceCloud = GenerateTrajectory(trajectoryType, pointCount);

        // 创建变换矩阵
        Quaternion rotation = Quaternion.Euler(rotationAngles);
        
        // 生成目标点云（变换后的轨迹）
        List<Vector3> targetCloud = new List<Vector3>();
        System.Random rng = new System.Random(42);

        foreach (var point in sourceCloud)
        {
            // 应用变换: p' = R * p + t
            Vector3 transformed = rotation * point + translation;

            // 添加噪声
            if (addNoise)
            {
                transformed += new Vector3(
                    (float)NextGaussian(rng) * noiseStdDev,
                    (float)NextGaussian(rng) * noiseStdDev,
                    (float)NextGaussian(rng) * noiseStdDev
                );
            }

            targetCloud.Add(transformed);
        }

        // 保存到CSV
        string outputPath = Path.Combine(Application.streamingAssetsPath, outputFolder);
        Directory.CreateDirectory(outputPath);

        string sourcePath = Path.Combine(outputPath, sourceFileName);
        string targetPath = Path.Combine(outputPath, targetFileName);

        SavePointCloudToCsv(sourceCloud, sourcePath);
        SavePointCloudToCsv(targetCloud, targetPath);

        Debug.Log($"[TestDataGenerator] 已生成测试数据:");
        Debug.Log($"  源点云: {sourcePath}");
        Debug.Log($"  目标点云: {targetPath}");
        Debug.Log($"  点数: {pointCount}");
        Debug.Log($"  应用的旋转: {rotationAngles}");
        Debug.Log($"  应用的平移: {translation}");
        Debug.Log($"  噪声: {(addNoise ? $"是 (σ={noiseStdDev})" : "否")}");
    }

    /// <summary>
    /// 生成轨迹点
    /// </summary>
    private List<Vector3> GenerateTrajectory(TrajectoryType type, int count)
    {
        var points = new List<Vector3>();

        for (int i = 0; i < count; i++)
        {
            float t = (float)i / (count - 1);
            Vector3 point = Vector3.zero;

            switch (type)
            {
                case TrajectoryType.Helix:
                    // 螺旋线
                    float angle = t * 4f * Mathf.PI;
                    point = new Vector3(
                        0.3f * Mathf.Cos(angle),
                        t * 0.5f,
                        0.3f * Mathf.Sin(angle)
                    );
                    break;

                case TrajectoryType.Circle:
                    // 圆形
                    float circleAngle = t * 2f * Mathf.PI;
                    point = new Vector3(
                        0.5f * Mathf.Cos(circleAngle),
                        0f,
                        0.5f * Mathf.Sin(circleAngle)
                    );
                    break;

                case TrajectoryType.Line:
                    // 直线
                    point = new Vector3(t, t * 0.5f, t * 0.3f);
                    break;

                case TrajectoryType.SineWave:
                    // 正弦波
                    point = new Vector3(
                        t,
                        0.2f * Mathf.Sin(t * 4f * Mathf.PI),
                        0f
                    );
                    break;

                case TrajectoryType.Figure8:
                    // 8字形
                    float fig8Angle = t * 2f * Mathf.PI;
                    point = new Vector3(
                        0.3f * Mathf.Sin(fig8Angle),
                        0.2f * Mathf.Sin(2f * fig8Angle),
                        0f
                    );
                    break;

                case TrajectoryType.Random:
                    // 随机点
                    point = new Vector3(
                        UnityEngine.Random.Range(-1f, 1f),
                        UnityEngine.Random.Range(-1f, 1f),
                        UnityEngine.Random.Range(-1f, 1f)
                    );
                    break;
            }

            points.Add(point);
        }

        return points;
    }

    /// <summary>
    /// 保存点云到CSV
    /// </summary>
    private void SavePointCloudToCsv(List<Vector3> points, string filePath)
    {
        using (StreamWriter writer = new StreamWriter(filePath))
        {
            writer.WriteLine("x,y,z");
            foreach (var point in points)
            {
                writer.WriteLine($"{point.x:F9},{point.y:F9},{point.z:F9}");
            }
        }
    }

    /// <summary>
    /// Box-Muller变换生成高斯随机数
    /// </summary>
    private double NextGaussian(System.Random rng)
    {
        double u1 = 1.0 - rng.NextDouble();
        double u2 = 1.0 - rng.NextDouble();
        return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
    }
}

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

/// <summary>
/// Kabsch算法实现 - 用于计算两个已知对应关系的点云之间的最优刚性变换
/// 基于SVD分解，最小化均方根误差(RMSE)
/// </summary>
public class KabschAlignment : MonoBehaviour
{
    [Header("CSV文件配置")]
    [Tooltip("源点云A的CSV文件路径（相对于StreamingAssets或绝对路径）")]
    public string sourceCloudCsvPath = "C:\\Users\\15421\\Desktop\\lighthouse_1.12\\Assets\\StreamingAssets\\TrackerRecordings\\PlaybackRecord_HighFreq_7_20260202_145911_7_20260202_150124.csv";
    
    [Tooltip("目标点云B的CSV文件路径（相对于StreamingAssets或绝对路径）")]
    public string targetCloudCsvPath = "C:\\Users\\15421\\Desktop\\lighthouse_1.12\\Assets\\StreamingAssets\\TrackerRecordings\\PlaybackRecord_HighFreq_7_20260202_145911_7_20260202_150007.csv";
    
    [Tooltip("是否使用绝对路径")]
    public bool useAbsolutePath = false;

    // CSV格式固定配置（格式：FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,...）
    private const char CSV_DELIMITER = ',';
    private const int X_COLUMN_INDEX = 3;  // X_mm列
    private const int Y_COLUMN_INDEX = 4;  // Y_mm列
    private const int Z_COLUMN_INDEX = 5;  // Z_mm列
    private const bool SKIP_HEADER_ROW = true;
    private const float MM_TO_M = 0.001f;  // 毫米转米

    [Header("对齐结果")]
    [SerializeField] private Vector3 translationVector;
    [SerializeField] private Vector3 rotationEulerAngles;
    [SerializeField] private float rmseError;
    [SerializeField] private int matchedPointCount;

    [Header("调试选项")]
    public bool autoAlignOnStart = false;
    public bool logDetailedInfo = true;

    // 存储读取的点云数据
    private List<Vector3> sourcePoints = new List<Vector3>();
    private List<Vector3> targetPoints = new List<Vector3>();

    // 对齐结果
    private Matrix<double> rotationMatrix;
    private Vector<double> translationVectorResult;
    private bool alignmentComputed = false;

    /// <summary>
    /// 获取计算出的旋转矩阵（3x3）
    /// </summary>
    public Matrix<double> RotationMatrix => rotationMatrix;

    /// <summary>
    /// 获取计算出的平移向量
    /// </summary>
    public Vector<double> TranslationVector => translationVectorResult;

    /// <summary>
    /// 获取均方根误差
    /// </summary>
    public float RMSE => rmseError;

    /// <summary>
    /// 对齐是否已计算
    /// </summary>
    public bool IsAlignmentComputed => alignmentComputed;

    private void Start()
    {
        if (autoAlignOnStart)
        {
            PerformAlignment();
        }
    }

    /// <summary>
    /// 执行完整的对齐流程：读取CSV -> 计算对齐 -> 输出结果
    /// </summary>
    [ContextMenu("执行点云对齐")]
    public void PerformAlignment()
    {
        // 1. 读取点云数据
        if (!LoadPointCloudsFromCsv())
        {
            Debug.LogError("[KabschAlignment] 无法加载点云数据，对齐中止");
            return;
        }

        // 2. 验证点云
        if (!ValidatePointClouds())
        {
            return;
        }

        // 3. 执行Kabsch算法
        if (ComputeKabschAlignment(sourcePoints, targetPoints, 
            out rotationMatrix, out translationVectorResult, out rmseError))
        {
            alignmentComputed = true;
            
            // 4. 更新Inspector显示
            UpdateInspectorDisplay();
            
            // 5. 输出结果
            if (logDetailedInfo)
            {
                LogAlignmentResults();
            }
        }
    }

    /// <summary>
    /// 从CSV文件加载点云数据
    /// </summary>
    private bool LoadPointCloudsFromCsv()
    {
        string sourcePath = GetFullPath(sourceCloudCsvPath);
        string targetPath = GetFullPath(targetCloudCsvPath);

        Debug.Log($"[KabschAlignment] 加载源点云: {sourcePath}");
        Debug.Log($"[KabschAlignment] 加载目标点云: {targetPath}");

        sourcePoints = ReadPointsFromCsv(sourcePath);
        targetPoints = ReadPointsFromCsv(targetPath);

        if (sourcePoints == null || sourcePoints.Count == 0)
        {
            Debug.LogError($"[KabschAlignment] 无法读取源点云文件或文件为空: {sourcePath}");
            return false;
        }

        if (targetPoints == null || targetPoints.Count == 0)
        {
            Debug.LogError($"[KabschAlignment] 无法读取目标点云文件或文件为空: {targetPath}");
            return false;
        }

        Debug.Log($"[KabschAlignment] 成功读取源点云 {sourcePoints.Count} 个点，目标点云 {targetPoints.Count} 个点");
        return true;
    }

    /// <summary>
    /// 获取完整文件路径
    /// </summary>
    private string GetFullPath(string path)
    {
        if (useAbsolutePath || Path.IsPathRooted(path))
        {
            return path;
        }
        return Path.Combine(Application.streamingAssetsPath, path);
    }

    /// <summary>
    /// 从CSV文件读取点坐标
    /// </summary>
    private List<Vector3> ReadPointsFromCsv(string filePath)
    {
        var points = new List<Vector3>();

        try
        {
            if (!File.Exists(filePath))
            {
                Debug.LogError($"[KabschAlignment] 文件不存在: {filePath}");
                return points;
            }

            string[] lines = File.ReadAllLines(filePath);
            int startLine = SKIP_HEADER_ROW ? 1 : 0;

            for (int i = startLine; i < lines.Length; i++)
            {
                string line = lines[i].Trim();
                if (string.IsNullOrEmpty(line)) continue;

                string[] values = line.Split(CSV_DELIMITER);
                
                // CSV格式：FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,...
                if (values.Length <= Z_COLUMN_INDEX)
                {
                    Debug.LogWarning($"[KabschAlignment] 第{i + 1}行列数不足，跳过: {line}");
                    continue;
                }

                if (float.TryParse(values[X_COLUMN_INDEX].Trim(), out float x) &&
                    float.TryParse(values[Y_COLUMN_INDEX].Trim(), out float y) &&
                    float.TryParse(values[Z_COLUMN_INDEX].Trim(), out float z))
                {
                    // 坐标从毫米转换为米
                    x *= MM_TO_M;
                    y *= MM_TO_M;
                    z *= MM_TO_M;
                    points.Add(new Vector3(x, y, z));
                }
                else
                {
                    Debug.LogWarning($"[KabschAlignment] 第{i + 1}行解析失败，跳过: {line}");
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[KabschAlignment] 读取CSV文件失败: {e.Message}");
        }

        return points;
    }

    /// <summary>
    /// 验证点云数据有效性
    /// </summary>
    private bool ValidatePointClouds()
    {
        if (sourcePoints.Count != targetPoints.Count)
        {
            Debug.LogError($"[KabschAlignment] 点云数量不匹配！源点云: {sourcePoints.Count}，目标点云: {targetPoints.Count}");
            return false;
        }

        if (sourcePoints.Count < 3)
        {
            Debug.LogError("[KabschAlignment] 点云至少需要3个点才能计算刚性变换");
            return false;
        }

        matchedPointCount = sourcePoints.Count;
        return true;
    }

    /// <summary>
    /// 使用Kabsch算法计算最优刚性变换
    /// 将源点云(P)变换到目标点云(Q)
    /// </summary>
    /// <param name="sourceCloud">源点云P</param>
    /// <param name="targetCloud">目标点云Q</param>
    /// <param name="R">输出：3x3旋转矩阵</param>
    /// <param name="t">输出：平移向量</param>
    /// <param name="rmse">输出：均方根误差</param>
    /// <returns>是否成功计算</returns>
    public bool ComputeKabschAlignment(
        List<Vector3> sourceCloud, 
        List<Vector3> targetCloud,
        out Matrix<double> R,
        out Vector<double> t,
        out float rmse)
    {
        R = null;
        t = null;
        rmse = float.MaxValue;

        int n = sourceCloud.Count;
        if (n < 3 || n != targetCloud.Count)
        {
            Debug.LogError("[KabschAlignment] 点云数据无效");
            return false;
        }

        try
        {
            // Step 1: 计算质心
            Vector3 centroidP = ComputeCentroid(sourceCloud);
            Vector3 centroidQ = ComputeCentroid(targetCloud);

            if (logDetailedInfo)
            {
                Debug.Log($"[KabschAlignment] 源点云质心: {centroidP}");
                Debug.Log($"[KabschAlignment] 目标点云质心: {centroidQ}");
            }

            // Step 2: 中心化点云（减去质心）
            var P = CreateCenteredMatrix(sourceCloud, centroidP);  // n x 3 矩阵
            var Q = CreateCenteredMatrix(targetCloud, centroidQ);  // n x 3 矩阵

            // Step 3: 计算协方差矩阵 H = P^T * Q (3x3)
            var H = P.TransposeThisAndMultiply(Q);

            if (logDetailedInfo)
            {
                Debug.Log($"[KabschAlignment] 协方差矩阵 H:\n{MatrixToString(H)}");
            }

            // Step 4: SVD分解 H = U * S * V^T
            var svd = H.Svd(true);
            var U = svd.U;
            var Vt = svd.VT;
            var V = Vt.Transpose();

            if (logDetailedInfo)
            {
                Debug.Log($"[KabschAlignment] 奇异值: [{string.Join(", ", svd.S.Select(s => s.ToString("F6")))}]");
            }

            // Step 5: 计算旋转矩阵 R = V * U^T
            R = V.Multiply(U.Transpose());

            // Step 6: 检查并处理反射情况（确保det(R) = 1而不是-1）
            double det = R.Determinant();
            if (det < 0)
            {
                Debug.LogWarning("[KabschAlignment] 检测到反射，正在修正...");
                
                // 修正：将V的最后一列取反
                var VCorrected = V.Clone();
                for (int i = 0; i < 3; i++)
                {
                    VCorrected[i, 2] = -VCorrected[i, 2];
                }
                R = VCorrected.Multiply(U.Transpose());
                
                if (logDetailedInfo)
                {
                    Debug.Log($"[KabschAlignment] 修正后的行列式: {R.Determinant():F6}");
                }
            }

            // Step 7: 计算平移向量 t = centroid_Q - R * centroid_P
            var centroidPVec = DenseVector.OfArray(new double[] { centroidP.x, centroidP.y, centroidP.z });
            var centroidQVec = DenseVector.OfArray(new double[] { centroidQ.x, centroidQ.y, centroidQ.z });
            t = centroidQVec - R.Multiply(centroidPVec);

            // Step 8: 计算RMSE
            rmse = ComputeRMSE(sourceCloud, targetCloud, R, t);

            if (logDetailedInfo)
            {
                Debug.Log($"[KabschAlignment] 旋转矩阵 R:\n{MatrixToString(R)}");
                Debug.Log($"[KabschAlignment] 平移向量 t: [{t[0]:F6}, {t[1]:F6}, {t[2]:F6}]");
                Debug.Log($"[KabschAlignment] RMSE: {rmse:F6}");
            }

            return true;
        }
        catch (Exception e)
        {
            Debug.LogError($"[KabschAlignment] Kabsch算法计算失败: {e.Message}\n{e.StackTrace}");
            return false;
        }
    }

    /// <summary>
    /// 计算点云质心
    /// </summary>
    private Vector3 ComputeCentroid(List<Vector3> points)
    {
        Vector3 sum = Vector3.zero;
        foreach (var p in points)
        {
            sum += p;
        }
        return sum / points.Count;
    }

    /// <summary>
    /// 创建中心化后的点云矩阵 (n x 3)
    /// </summary>
    private Matrix<double> CreateCenteredMatrix(List<Vector3> points, Vector3 centroid)
    {
        int n = points.Count;
        var matrix = DenseMatrix.Create(n, 3, 0);

        for (int i = 0; i < n; i++)
        {
            Vector3 centered = points[i] - centroid;
            matrix[i, 0] = centered.x;
            matrix[i, 1] = centered.y;
            matrix[i, 2] = centered.z;
        }

        return matrix;
    }

    /// <summary>
    /// 计算均方根误差
    /// </summary>
    private float ComputeRMSE(List<Vector3> source, List<Vector3> target, 
        Matrix<double> R, Vector<double> t)
    {
        double sumSquaredError = 0;
        int n = source.Count;

        for (int i = 0; i < n; i++)
        {
            // 变换源点: p' = R * p + t
            var pVec = DenseVector.OfArray(new double[] { source[i].x, source[i].y, source[i].z });
            var transformed = R.Multiply(pVec) + t;

            // 计算与目标点的距离
            double dx = transformed[0] - target[i].x;
            double dy = transformed[1] - target[i].y;
            double dz = transformed[2] - target[i].z;

            sumSquaredError += dx * dx + dy * dy + dz * dz;
        }

        return (float)Math.Sqrt(sumSquaredError / n);
    }

    /// <summary>
    /// 更新Inspector显示
    /// </summary>
    private void UpdateInspectorDisplay()
    {
        if (translationVectorResult != null)
        {
            translationVector = new Vector3(
                (float)translationVectorResult[0],
                (float)translationVectorResult[1],
                (float)translationVectorResult[2]
            );
        }

        if (rotationMatrix != null)
        {
            // 从旋转矩阵提取欧拉角
            rotationEulerAngles = RotationMatrixToEulerAngles(rotationMatrix);
        }
    }

    /// <summary>
    /// 从旋转矩阵提取欧拉角（度）
    /// </summary>
    private Vector3 RotationMatrixToEulerAngles(Matrix<double> R)
    {
        float sy = (float)Math.Sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0]);
        bool singular = sy < 1e-6f;

        float x, y, z;
        if (!singular)
        {
            x = (float)Math.Atan2(R[2, 1], R[2, 2]);
            y = (float)Math.Atan2(-R[2, 0], sy);
            z = (float)Math.Atan2(R[1, 0], R[0, 0]);
        }
        else
        {
            x = (float)Math.Atan2(-R[1, 2], R[1, 1]);
            y = (float)Math.Atan2(-R[2, 0], sy);
            z = 0;
        }

        return new Vector3(x, y, z) * Mathf.Rad2Deg;
    }

    /// <summary>
    /// 输出对齐结果日志
    /// </summary>
    private void LogAlignmentResults()
    {
        Debug.Log("========== Kabsch对齐结果 ==========");
        Debug.Log($"匹配点数: {matchedPointCount}");
        Debug.Log($"均方根误差 (RMSE): {rmseError:F6} 单位");
        Debug.Log($"平移向量: ({translationVector.x:F6}, {translationVector.y:F6}, {translationVector.z:F6})");
        Debug.Log($"旋转欧拉角: ({rotationEulerAngles.x:F3}°, {rotationEulerAngles.y:F3}°, {rotationEulerAngles.z:F3}°)");
        Debug.Log("=====================================");
    }

    /// <summary>
    /// 将源点云中的点变换到目标坐标系
    /// </summary>
    public Vector3 TransformPoint(Vector3 sourcePoint)
    {
        if (!alignmentComputed || rotationMatrix == null || translationVectorResult == null)
        {
            Debug.LogWarning("[KabschAlignment] 对齐尚未计算，返回原始点");
            return sourcePoint;
        }

        var pVec = DenseVector.OfArray(new double[] { sourcePoint.x, sourcePoint.y, sourcePoint.z });
        var transformed = rotationMatrix.Multiply(pVec) + translationVectorResult;

        return new Vector3((float)transformed[0], (float)transformed[1], (float)transformed[2]);
    }

    /// <summary>
    /// 将整个源点云变换到目标坐标系
    /// </summary>
    public List<Vector3> TransformPointCloud(List<Vector3> sourceCloud)
    {
        var transformed = new List<Vector3>();
        foreach (var point in sourceCloud)
        {
            transformed.Add(TransformPoint(point));
        }
        return transformed;
    }

    /// <summary>
    /// 获取变换后的源点云
    /// </summary>
    public List<Vector3> GetTransformedSourceCloud()
    {
        return TransformPointCloud(sourcePoints);
    }

    /// <summary>
    /// 获取源点云
    /// </summary>
    public List<Vector3> GetSourceCloud() => new List<Vector3>(sourcePoints);

    /// <summary>
    /// 获取目标点云
    /// </summary>
    public List<Vector3> GetTargetCloud() => new List<Vector3>(targetPoints);

    /// <summary>
    /// 获取Unity格式的旋转四元数
    /// </summary>
    public Quaternion GetRotationQuaternion()
    {
        if (!alignmentComputed || rotationMatrix == null)
        {
            return Quaternion.identity;
        }

        // 从旋转矩阵构造四元数
        Matrix4x4 m = Matrix4x4.identity;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                m[i, j] = (float)rotationMatrix[i, j];
            }
        }

        return m.rotation;
    }

    /// <summary>
    /// 获取Unity格式的完整变换矩阵 (4x4)
    /// </summary>
    public Matrix4x4 GetTransformMatrix()
    {
        if (!alignmentComputed || rotationMatrix == null || translationVectorResult == null)
        {
            return Matrix4x4.identity;
        }

        Matrix4x4 m = Matrix4x4.identity;
        
        // 设置旋转部分
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                m[i, j] = (float)rotationMatrix[i, j];
            }
        }

        // 设置平移部分
        m[0, 3] = (float)translationVectorResult[0];
        m[1, 3] = (float)translationVectorResult[1];
        m[2, 3] = (float)translationVectorResult[2];

        return m;
    }

    /// <summary>
    /// 测试变换结果：应用刚性变换并输出每个点的误差（毫米单位输出）
    /// </summary>
    [ContextMenu("测试变换并输出误差（毫米）")]
    public void TestTransformationAndPrintErrors()
    {
        if (!alignmentComputed || rotationMatrix == null || translationVectorResult == null)
        {
            Debug.LogWarning("[KabschAlignment] 尚未计算对齐，无法测试");
            return;
        }

        if (sourcePoints.Count == 0 || targetPoints.Count == 0)
        {
            Debug.LogWarning("[KabschAlignment] 点云数据为空");
            return;
        }

        Debug.Log("========== 变换测试：逐点误差分析（对比原始误差） ==========");
        Debug.Log($"总点数: {sourcePoints.Count}");
        Debug.Log($"变换公式: p' = R * p + t");
        Debug.Log($"输出单位: 毫米 (mm)");
        Debug.Log($"注意: 内部计算使用米单位，输出时转换为毫米");
        Debug.Log("");

        // 统计变量（米单位）
        double sumErrorOriginal = 0;      // 原始误差累加
        double sumErrorTransformed = 0;   // 变换后误差累加
        double maxErrorOriginal = 0;
        double maxErrorTransformed = 0;
        double minErrorOriginal = double.MaxValue;
        double minErrorTransformed = double.MaxValue;
        int maxErrorOriginalIndex = 0;
        int maxErrorTransformedIndex = 0;
        int minErrorOriginalIndex = 0;
        int minErrorTransformedIndex = 0;

        // 遍历每个点
        for (int i = 0; i < sourcePoints.Count; i++)
        {
            Vector3 sourcePoint = sourcePoints[i];
            Vector3 targetPoint = targetPoints[i];

            // ===== 计算原始误差（未变换） =====
            double dx_orig = sourcePoint.x - targetPoint.x;
            double dy_orig = sourcePoint.y - targetPoint.y;
            double dz_orig = sourcePoint.z - targetPoint.z;
            double errorOriginal = Math.Sqrt(dx_orig * dx_orig + dy_orig * dy_orig + dz_orig * dz_orig);

            // ===== 计算变换后误差 =====
            // 变换源点: p' = R * p + t （米单位）
            var pVec = DenseVector.OfArray(new double[] { sourcePoint.x, sourcePoint.y, sourcePoint.z });
            var transformed = rotationMatrix.Multiply(pVec) + translationVectorResult;

            Vector3 transformedPoint = new Vector3(
                (float)transformed[0], 
                (float)transformed[1], 
                (float)transformed[2]
            );

            double dx_trans = transformedPoint.x - targetPoint.x;
            double dy_trans = transformedPoint.y - targetPoint.y;
            double dz_trans = transformedPoint.z - targetPoint.z;
            double errorTransformed = Math.Sqrt(dx_trans * dx_trans + dy_trans * dy_trans + dz_trans * dz_trans);

            // ===== 更新统计（原始） =====
            sumErrorOriginal += errorOriginal;
            if (errorOriginal > maxErrorOriginal)
            {
                maxErrorOriginal = errorOriginal;
                maxErrorOriginalIndex = i;
            }
            if (errorOriginal < minErrorOriginal)
            {
                minErrorOriginal = errorOriginal;
                minErrorOriginalIndex = i;
            }

            // ===== 更新统计（变换后） =====
            sumErrorTransformed += errorTransformed;
            if (errorTransformed > maxErrorTransformed)
            {
                maxErrorTransformed = errorTransformed;
                maxErrorTransformedIndex = i;
            }
            if (errorTransformed < minErrorTransformed)
            {
                minErrorTransformed = errorTransformed;
                minErrorTransformedIndex = i;
            }

            // 计算改善量
            double improvement = errorOriginal - errorTransformed;
            double improvementPercent = errorOriginal > 1e-9 ? (improvement / errorOriginal) * 100 : 0;

            // ===== 输出每个点的详细信息（转换为毫米） =====
            Debug.Log($"点 {i}:");
            Debug.Log($"  源点 (mm):           ({sourcePoint.x * 1000:F3}, {sourcePoint.y * 1000:F3}, {sourcePoint.z * 1000:F3})");
            Debug.Log($"  变换后 (mm):         ({transformedPoint.x * 1000:F3}, {transformedPoint.y * 1000:F3}, {transformedPoint.z * 1000:F3})");
            Debug.Log($"  目标点 (mm):         ({targetPoint.x * 1000:F3}, {targetPoint.y * 1000:F3}, {targetPoint.z * 1000:F3})");
            Debug.Log($"  原始误差 (mm):       {errorOriginal * 1000:F6}");
            Debug.Log($"  变换后误差 (mm):     {errorTransformed * 1000:F6}");
            Debug.Log($"  误差改善 (mm):       {improvement * 1000:F6} ({improvementPercent:F2}%)");
            Debug.Log($"  变换后误差分量 (mm): dx={dx_trans * 1000:F6}, dy={dy_trans * 1000:F6}, dz={dz_trans * 1000:F6}");
            Debug.Log("");
        }

        // ===== 输出统计信息（转换为毫米） =====
        double meanErrorOriginal = sumErrorOriginal / sourcePoints.Count;
        double meanErrorTransformed = sumErrorTransformed / sourcePoints.Count;
        double meanImprovement = meanErrorOriginal - meanErrorTransformed;
        double meanImprovementPercent = meanErrorOriginal > 1e-9 ? (meanImprovement / meanErrorOriginal) * 100 : 0;

        Debug.Log("========== 误差统计对比 ==========");
        Debug.Log("【原始轨迹误差（未变换）】");
        Debug.Log($"  平均误差 (mm):   {meanErrorOriginal * 1000:F6}");
        Debug.Log($"  最大误差 (mm):   {maxErrorOriginal * 1000:F6} (点{maxErrorOriginalIndex})");
        Debug.Log($"  最小误差 (mm):   {minErrorOriginal * 1000:F6} (点{minErrorOriginalIndex})");
        Debug.Log("");
        Debug.Log("【变换后轨迹误差】");
        Debug.Log($"  平均误差 (mm):   {meanErrorTransformed * 1000:F6}");
        Debug.Log($"  最大误差 (mm):   {maxErrorTransformed * 1000:F6} (点{maxErrorTransformedIndex})");
        Debug.Log($"  最小误差 (mm):   {minErrorTransformed * 1000:F6} (点{minErrorTransformedIndex})");
        Debug.Log($"  RMSE (mm):       {rmseError * 1000:F6}");
        Debug.Log("");
        Debug.Log("【改善效果】");
        Debug.Log($"  平均改善 (mm):   {meanImprovement * 1000:F6} ({meanImprovementPercent:F2}%)");
        Debug.Log($"  最大改善 (mm):   {(maxErrorOriginal - minErrorTransformed) * 1000:F6}");
        Debug.Log("========================================");
    }

    /// <summary>
    /// 导出对齐结果到CSV文件
    /// </summary>
    [ContextMenu("导出对齐结果")]
    public void ExportAlignmentResult()
    {
        if (!alignmentComputed)
        {
            Debug.LogWarning("[KabschAlignment] 尚未计算对齐，无法导出");
            return;
        }

        string exportPath = Path.Combine(Application.streamingAssetsPath, "PointClouds", "alignment_result.csv");
        
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(exportPath));

            using (StreamWriter writer = new StreamWriter(exportPath))
            {
                writer.WriteLine("# Kabsch Alignment Result");
                writer.WriteLine($"# RMSE: {rmseError:F6}");
                writer.WriteLine($"# Point Count: {matchedPointCount}");
                writer.WriteLine();
                
                writer.WriteLine("# Rotation Matrix (3x3)");
                for (int i = 0; i < 3; i++)
                {
                    writer.WriteLine($"{rotationMatrix[i, 0]:F9},{rotationMatrix[i, 1]:F9},{rotationMatrix[i, 2]:F9}");
                }
                
                writer.WriteLine();
                writer.WriteLine("# Translation Vector");
                writer.WriteLine($"{translationVectorResult[0]:F9},{translationVectorResult[1]:F9},{translationVectorResult[2]:F9}");
                
                writer.WriteLine();
                writer.WriteLine("# Euler Angles (degrees)");
                writer.WriteLine($"{rotationEulerAngles.x:F6},{rotationEulerAngles.y:F6},{rotationEulerAngles.z:F6}");
            }

            Debug.Log($"[KabschAlignment] 对齐结果已导出到: {exportPath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[KabschAlignment] 导出失败: {e.Message}");
        }
    }

    /// <summary>
    /// 导出变换后的点云到CSV（简化格式：仅坐标）
    /// </summary>
    [ContextMenu("导出变换后的点云（简化格式）")]
    public void ExportTransformedCloud()
    {
        if (!alignmentComputed || sourcePoints.Count == 0)
        {
            Debug.LogWarning("[KabschAlignment] 尚未计算对齐或点云为空");
            return;
        }

        string exportPath = Path.Combine(Application.streamingAssetsPath, "PointClouds", "transformed_source.csv");

        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(exportPath));
            var transformed = GetTransformedSourceCloud();

            using (StreamWriter writer = new StreamWriter(exportPath))
            {
                writer.WriteLine("X_m,Y_m,Z_m");
                foreach (var point in transformed)
                {
                    writer.WriteLine($"{point.x:F9},{point.y:F9},{point.z:F9}");
                }
            }

            Debug.Log($"[KabschAlignment] 变换后点云已导出到: {exportPath}");
            Debug.Log($"  格式: 简化格式 (仅坐标)");
            Debug.Log($"  单位: 米 (m)");
        }
        catch (Exception e)
        {
            Debug.LogError($"[KabschAlignment] 导出失败: {e.Message}");
        }
    }

    /// <summary>
    /// 导出变换后的点云到CSV（完整格式：包含毫米单位和索引）
    /// </summary>
    [ContextMenu("导出变换后的点云（完整格式）")]
    public void ExportTransformedCloudFull()
    {
        if (!alignmentComputed || sourcePoints.Count == 0)
        {
            Debug.LogWarning("[KabschAlignment] 尚未计算对齐或点云为空");
            return;
        }

        string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string exportPath = Path.Combine(Application.streamingAssetsPath, "PointClouds", 
            $"transformed_source_{timestamp}.csv");

        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(exportPath));
            var transformed = GetTransformedSourceCloud();

            using (StreamWriter writer = new StreamWriter(exportPath))
            {
                // 写入CSV头部（与输入格式一致）
                writer.WriteLine("FrameNumber,X_mm,Y_mm,Z_mm");
                
                // 写入变换后的点（转换为毫米）
                for (int i = 0; i < transformed.Count; i++)
                {
                    writer.WriteLine($"{i}," +
                        $"{transformed[i].x * 1000:F6}," +
                        $"{transformed[i].y * 1000:F6}," +
                        $"{transformed[i].z * 1000:F6}");
                }
            }

            Debug.Log($"[KabschAlignment] 变换后点云已导出到: {exportPath}");
            Debug.Log($"  格式: 完整格式 (帧号 + 坐标)");
            Debug.Log($"  单位: 毫米 (mm)");
            Debug.Log($"  点数: {transformed.Count}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[KabschAlignment] 导出失败: {e.Message}");
        }
    }

    /// <summary>
    /// 导出变换前后对比CSV
    /// </summary>
    [ContextMenu("导出变换前后对比")]
    public void ExportTransformComparison()
    {
        if (!alignmentComputed || sourcePoints.Count == 0 || targetPoints.Count == 0)
        {
            Debug.LogWarning("[KabschAlignment] 尚未计算对齐或点云为空");
            return;
        }

        string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string exportPath = Path.Combine(Application.streamingAssetsPath, "PointClouds", 
            $"transform_comparison_{timestamp}.csv");

        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(exportPath));
            var transformed = GetTransformedSourceCloud();

            using (StreamWriter writer = new StreamWriter(exportPath))
            {
                // CSV头部
                writer.WriteLine("FrameNumber," +
                    "Source_X_mm,Source_Y_mm,Source_Z_mm," +
                    "Transformed_X_mm,Transformed_Y_mm,Transformed_Z_mm," +
                    "Target_X_mm,Target_Y_mm,Target_Z_mm," +
                    "Error_mm");
                
                // 逐点写入对比数据
                int count = Math.Min(Math.Min(sourcePoints.Count, transformed.Count), targetPoints.Count);
                for (int i = 0; i < count; i++)
                {
                    // 计算误差
                    float dx = transformed[i].x - targetPoints[i].x;
                    float dy = transformed[i].y - targetPoints[i].y;
                    float dz = transformed[i].z - targetPoints[i].z;
                    float error = Mathf.Sqrt(dx*dx + dy*dy + dz*dz);

                    writer.WriteLine($"{i}," +
                        // 源点（米转毫米）
                        $"{sourcePoints[i].x * 1000:F6},{sourcePoints[i].y * 1000:F6},{sourcePoints[i].z * 1000:F6}," +
                        // 变换后（米转毫米）
                        $"{transformed[i].x * 1000:F6},{transformed[i].y * 1000:F6},{transformed[i].z * 1000:F6}," +
                        // 目标点（米转毫米）
                        $"{targetPoints[i].x * 1000:F6},{targetPoints[i].y * 1000:F6},{targetPoints[i].z * 1000:F6}," +
                        // 误差（米转毫米）
                        $"{error * 1000:F9}");
                }
            }

            Debug.Log($"[KabschAlignment] 变换对比数据已导出到: {exportPath}");
            Debug.Log($"  包含列: 源点, 变换后点, 目标点, 误差");
            Debug.Log($"  单位: 毫米 (mm)");
            Debug.Log($"  点数: {Math.Min(Math.Min(sourcePoints.Count, transformed.Count), targetPoints.Count)}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[KabschAlignment] 导出失败: {e.Message}");
        }
    }

    /// <summary>
    /// 矩阵转字符串（调试用）
    /// </summary>
    private string MatrixToString(Matrix<double> m)
    {
        var sb = new System.Text.StringBuilder();
        for (int i = 0; i < m.RowCount; i++)
        {
            sb.Append("  [");
            for (int j = 0; j < m.ColumnCount; j++)
            {
                sb.Append($"{m[i, j],12:F6}");
                if (j < m.ColumnCount - 1) sb.Append(", ");
            }
            sb.AppendLine("]");
        }
        return sb.ToString();
    }

    /// <summary>
    /// 手动设置点云数据（用于程序化调用）
    /// </summary>
    public void SetPointClouds(List<Vector3> source, List<Vector3> target)
    {
        sourcePoints = new List<Vector3>(source);
        targetPoints = new List<Vector3>(target);
        alignmentComputed = false;
    }

    /// <summary>
    /// 使用已设置的点云计算对齐（不读取CSV）
    /// </summary>
    public bool ComputeAlignmentFromData()
    {
        if (!ValidatePointClouds())
        {
            return false;
        }

        if (ComputeKabschAlignment(sourcePoints, targetPoints,
            out rotationMatrix, out translationVectorResult, out rmseError))
        {
            alignmentComputed = true;
            UpdateInspectorDisplay();

            if (logDetailedInfo)
            {
                LogAlignmentResults();
            }
            return true;
        }

        return false;
    }

    // ============================================================
    // 诊断工具：用于分析SVD效果差的原因
    // ============================================================

    /// <summary>
    /// 数据完整性检查 - 检查点云的基本信息和范围
    /// </summary>
    [ContextMenu("诊断1：数据完整性检查")]
    public void DataIntegrityCheck()
    {
        Debug.Log("========== 数据完整性检查 ==========");
        
        if (sourcePoints.Count == 0 || targetPoints.Count == 0)
        {
            Debug.LogError("点云未加载！请先执行'执行点云对齐'或加载CSV文件");
            return;
        }

        Debug.Log($"<b>点数统计：</b>");
        Debug.Log($"  源点云: {sourcePoints.Count} 个点");
        Debug.Log($"  目标点云: {targetPoints.Count} 个点");
        Debug.Log($"  点数匹配: {(sourcePoints.Count == targetPoints.Count ? "✓ 是" : "✗ 否")}");

        // 计算源点云包围盒
        Vector3 sourceMin = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        Vector3 sourceMax = new Vector3(float.MinValue, float.MinValue, float.MinValue);
        foreach (var p in sourcePoints)
        {
            sourceMin = Vector3.Min(sourceMin, p);
            sourceMax = Vector3.Max(sourceMax, p);
        }
        Vector3 sourceSize = (sourceMax - sourceMin) * 1000f; // 转换为mm

        // 计算目标点云包围盒
        Vector3 targetMin = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        Vector3 targetMax = new Vector3(float.MinValue, float.MinValue, float.MinValue);
        foreach (var p in targetPoints)
        {
            targetMin = Vector3.Min(targetMin, p);
            targetMax = Vector3.Max(targetMax, p);
        }
        Vector3 targetSize = (targetMax - targetMin) * 1000f; // 转换为mm

        Debug.Log($"\n<b>源点云范围 (mm)：</b>");
        Debug.Log($"  最小值: ({sourceMin.x * 1000:F2}, {sourceMin.y * 1000:F2}, {sourceMin.z * 1000:F2})");
        Debug.Log($"  最大值: ({sourceMax.x * 1000:F2}, {sourceMax.y * 1000:F2}, {sourceMax.z * 1000:F2})");
        Debug.Log($"  尺寸: ({sourceSize.x:F2} × {sourceSize.y:F2} × {sourceSize.z:F2}) mm");

        Debug.Log($"\n<b>目标点云范围 (mm)：</b>");
        Debug.Log($"  最小值: ({targetMin.x * 1000:F2}, {targetMin.y * 1000:F2}, {targetMin.z * 1000:F2})");
        Debug.Log($"  最大值: ({targetMax.x * 1000:F2}, {targetMax.y * 1000:F2}, {targetMax.z * 1000:F2})");
        Debug.Log($"  尺寸: ({targetSize.x:F2} × {targetSize.y:F2} × {targetSize.z:F2}) mm");

        // 尺寸对比
        Vector3 sizeDiff = targetSize - sourceSize;
        Debug.Log($"\n<b>尺寸差异 (目标-源, mm)：</b>");
        Debug.Log($"  ΔX: {sizeDiff.x:F2} mm ({Mathf.Abs(sizeDiff.x / sourceSize.x * 100):F1}%)");
        Debug.Log($"  ΔY: {sizeDiff.y:F2} mm ({Mathf.Abs(sizeDiff.y / sourceSize.y * 100):F1}%)");
        Debug.Log($"  ΔZ: {sizeDiff.z:F2} mm ({Mathf.Abs(sizeDiff.z / sourceSize.z * 100):F1}%)");

        // 判断
        float maxDiffPercent = Mathf.Max(
            Mathf.Abs(sizeDiff.x / sourceSize.x),
            Mathf.Abs(sizeDiff.y / sourceSize.y),
            Mathf.Abs(sizeDiff.z / sourceSize.z)
        ) * 100f;

        Debug.Log($"\n<b>诊断结论：</b>");
        if (maxDiffPercent > 10f)
        {
            Debug.LogWarning($"⚠️ 轨迹尺寸差异较大 ({maxDiffPercent:F1}%)，可能存在：");
            Debug.LogWarning("  1. 坐标系标定不准确");
            Debug.LogWarning("  2. 两条轨迹不是同一运动");
            Debug.LogWarning("  3. 数据缩放比例错误");
        }
        else if (maxDiffPercent > 5f)
        {
            Debug.LogWarning($"⚠️ 轨迹尺寸有一定差异 ({maxDiffPercent:F1}%)，建议检查标定精度");
        }
        else
        {
            Debug.Log($"✓ 轨迹尺寸基本一致 (差异<5%)");
        }

        Debug.Log("=====================================\n");
    }

    /// <summary>
    /// 可视化轨迹 - 在Scene视图中绘制两条轨迹
    /// </summary>
    [ContextMenu("诊断2：可视化轨迹")]
    public void VisualizeTrajectories()
    {
        if (sourcePoints.Count == 0 || targetPoints.Count == 0)
        {
            Debug.LogError("点云未加载！请先执行'执行点云对齐'");
            return;
        }

        Debug.Log("========== 轨迹可视化 ==========");
        Debug.Log("已在Scene视图中绘制轨迹（持续10秒）：");
        Debug.Log("  <color=red>红色</color> = 源轨迹");
        Debug.Log("  <color=blue>蓝色</color> = 目标轨迹");
        Debug.Log("\n请观察：");
        Debug.Log("  1. 两条轨迹的形状是否相似？");
        Debug.Log("  2. 轨迹方向是否一致？");
        Debug.Log("  3. 是否有明显的弯曲或扭曲差异？");
        Debug.Log("================================\n");

        // 绘制源轨迹（红色）
        for (int i = 0; i < sourcePoints.Count - 1; i++)
        {
            Debug.DrawLine(sourcePoints[i], sourcePoints[i + 1], Color.red, 10f);
        }

        // 绘制目标轨迹（蓝色）
        for (int i = 0; i < targetPoints.Count - 1; i++)
        {
            Debug.DrawLine(targetPoints[i], targetPoints[i + 1], Color.blue, 10f);
        }

        // 标记起点和终点
        if (sourcePoints.Count > 0)
        {
            Debug.DrawLine(sourcePoints[0], sourcePoints[0] + Vector3.up * 0.05f, Color.yellow, 10f);
        }
        if (targetPoints.Count > 0)
        {
            Debug.DrawLine(targetPoints[0], targetPoints[0] + Vector3.up * 0.05f, Color.cyan, 10f);
        }
    }

    /// <summary>
    /// 分段误差分析 - 将轨迹分段，计算每段的RMSE
    /// </summary>
    [ContextMenu("诊断3：分段误差分析（每段100点）")]
    public void SegmentedErrorAnalysis()
    {
        SegmentedErrorAnalysisWithSize(100);
    }

    /// <summary>
    /// 分段误差分析（自定义段大小）
    /// </summary>
    public void SegmentedErrorAnalysisWithSize(int segmentSize)
    {
        if (sourcePoints.Count == 0 || targetPoints.Count == 0)
        {
            Debug.LogError("点云未加载！请先执行'执行点云对齐'");
            return;
        }

        if (sourcePoints.Count != targetPoints.Count)
        {
            Debug.LogError("点云数量不匹配！");
            return;
        }

        Debug.Log("========== 分段误差分析 ==========");
        Debug.Log($"总点数: {sourcePoints.Count}");
        Debug.Log($"段大小: {segmentSize} 点/段");

        int numSegments = Mathf.CeilToInt((float)sourcePoints.Count / segmentSize);
        Debug.Log($"段数: {numSegments}\n");

        List<float> rmseList = new List<float>();

        for (int seg = 0; seg < numSegments; seg++)
        {
            int start = seg * segmentSize;
            int end = Mathf.Min(start + segmentSize, sourcePoints.Count);
            int actualSize = end - start;

            var sourceSeg = sourcePoints.GetRange(start, actualSize);
            var targetSeg = targetPoints.GetRange(start, actualSize);

            bool success = ComputeKabschAlignment(sourceSeg, targetSeg,
                out Matrix<double> R, out Vector<double> t, out float rmse);

            if (success)
            {
                float rmse_mm = rmse * 1000f;
                rmseList.Add(rmse_mm);

                string status = rmse_mm < 5f ? "✓ 优秀" :
                               rmse_mm < 10f ? "○ 良好" :
                               rmse_mm < 20f ? "△ 一般" : "✗ 较差";

                Debug.Log($"段 {seg} (点 {start}-{end - 1}, 共{actualSize}点): " +
                         $"RMSE = <b>{rmse_mm:F2} mm</b> {status}");
            }
            else
            {
                Debug.LogError($"段 {seg} 计算失败");
            }
        }

        // 统计
        if (rmseList.Count > 0)
        {
            float avgRmse = rmseList.Average();
            float maxRmse = rmseList.Max();
            float minRmse = rmseList.Min();

            Debug.Log($"\n<b>统计结果：</b>");
            Debug.Log($"  平均RMSE: {avgRmse:F2} mm");
            Debug.Log($"  最大RMSE: {maxRmse:F2} mm (段 {rmseList.IndexOf(maxRmse)})");
            Debug.Log($"  最小RMSE: {minRmse:F2} mm (段 {rmseList.IndexOf(minRmse)})");
            Debug.Log($"  RMSE范围: {maxRmse - minRmse:F2} mm");

            Debug.Log($"\n<b>诊断结论：</b>");
            if (maxRmse < 5f)
            {
                Debug.Log("✓ 所有段的RMSE都很小(<5mm)，刚性变换假设成立");
                Debug.Log("  → 数据质量良好，可以使用Kabsch对齐");
            }
            else if (avgRmse < 10f && maxRmse < 15f)
            {
                Debug.Log("○ 大部分段的RMSE较小，整体可接受");
                Debug.Log("  → 建议使用分段对齐以获得更好效果");
            }
            else if (maxRmse - minRmse > 15f)
            {
                Debug.LogWarning("⚠️ 不同段的RMSE差异很大，说明存在非刚性形变");
                Debug.LogWarning("  可能原因：");
                Debug.LogWarning("  1. 时间同步问题（前后段对应关系错位）");
                Debug.LogWarning("  2. 轨迹本身有弯曲/扭曲差异");
                Debug.LogWarning("  3. 坐标系标定在不同区域精度不同");
                Debug.LogWarning("  → 推荐方案：分段对齐 或 重新采集数据");
            }
            else
            {
                Debug.LogWarning("⚠️ RMSE普遍较大(>10mm)，数据质量不佳");
                Debug.LogWarning("  → 建议检查数据源和标定精度");
            }
        }

        Debug.Log("=====================================\n");
    }

    /// <summary>
    /// 逐点原始误差分布分析
    /// </summary>
    [ContextMenu("诊断4：原始误差分布分析")]
    public void OriginalErrorDistribution()
    {
        if (sourcePoints.Count == 0 || targetPoints.Count == 0)
        {
            Debug.LogError("点云未加载！");
            return;
        }

        if (sourcePoints.Count != targetPoints.Count)
        {
            Debug.LogError("点云数量不匹配！");
            return;
        }

        Debug.Log("========== 原始误差分布分析 ==========");
        Debug.Log("（未经任何变换的点对点距离）\n");

        List<float> errors = new List<float>();
        List<Vector3> errorVectors = new List<Vector3>();

        for (int i = 0; i < sourcePoints.Count; i++)
        {
            Vector3 diff = (targetPoints[i] - sourcePoints[i]) * 1000f; // mm
            float dist = diff.magnitude;
            errors.Add(dist);
            errorVectors.Add(diff);
        }

        // 统计
        float mean = errors.Average();
        float max = errors.Max();
        float min = errors.Min();
        int maxIdx = errors.IndexOf(max);
        int minIdx = errors.IndexOf(min);

        Debug.Log($"<b>误差统计：</b>");
        Debug.Log($"  平均误差: {mean:F2} mm");
        Debug.Log($"  最大误差: {max:F2} mm (点 {maxIdx})");
        Debug.Log($"  最小误差: {min:F2} mm (点 {minIdx})");
        Debug.Log($"  误差范围: {max - min:F2} mm");

        // 分析误差方向性（系统性偏移）
        Vector3 avgErrorVec = Vector3.zero;
        foreach (var ev in errorVectors)
        {
            avgErrorVec += ev;
        }
        avgErrorVec /= errorVectors.Count;

        Debug.Log($"\n<b>系统性偏移（平均误差向量）：</b>");
        Debug.Log($"  dx: {avgErrorVec.x:F2} mm");
        Debug.Log($"  dy: {avgErrorVec.y:F2} mm");
        Debug.Log($"  dz: {avgErrorVec.z:F2} mm");
        Debug.Log($"  |d|: {avgErrorVec.magnitude:F2} mm");

        // 分段分析误差方向
        int numSegs = 4;
        int segSize = sourcePoints.Count / numSegs;
        Debug.Log($"\n<b>分段误差方向分析：</b>");
        
        for (int seg = 0; seg < numSegs; seg++)
        {
            int start = seg * segSize;
            int end = (seg == numSegs - 1) ? errorVectors.Count : (seg + 1) * segSize;
            
            Vector3 segAvg = Vector3.zero;
            for (int i = start; i < end; i++)
            {
                segAvg += errorVectors[i];
            }
            segAvg /= (end - start);

            Debug.Log($"  段{seg} (点{start}-{end - 1}): " +
                     $"dx={segAvg.x:F1}, dy={segAvg.y:F1}, dz={segAvg.z:F1} mm");
        }

        // 判断是否存在系统性偏移
        float systematicOffset = avgErrorVec.magnitude;
        Debug.Log($"\n<b>诊断结论：</b>");
        
        if (systematicOffset > 10f)
        {
            Debug.LogWarning($"⚠️ 存在明显的系统性偏移 ({systematicOffset:F1} mm)");
            Debug.LogWarning("  → 说明两个坐标系没有正确对齐");
            Debug.LogWarning("  → Kabsch变换可以修正这个平移偏移");
        }

        // 检查不同段的误差方向是否一致
        Vector3 seg0Avg = Vector3.zero;
        Vector3 seg3Avg = Vector3.zero;
        
        for (int i = 0; i < segSize; i++)
        {
            seg0Avg += errorVectors[i];
        }
        seg0Avg /= segSize;
        
        int lastSegStart = (numSegs - 1) * segSize;
        for (int i = lastSegStart; i < errorVectors.Count; i++)
        {
            seg3Avg += errorVectors[i];
        }
        seg3Avg /= (errorVectors.Count - lastSegStart);

        float directionDiff = Vector3.Angle(seg0Avg, seg3Avg);
        Debug.Log($"\n首段与末段误差方向夹角: {directionDiff:F1}°");
        
        if (directionDiff > 90f)
        {
            Debug.LogWarning("⚠️ 首段和末段的误差方向相反！");
            Debug.LogWarning("  → 这是非刚性形变的明确证据");
            Debug.LogWarning("  → 单一刚性变换无法同时优化所有区域");
            Debug.LogWarning("  → 强烈建议使用分段对齐");
        }
        else if (directionDiff > 45f)
        {
            Debug.LogWarning("⚠️ 首段和末段的误差方向差异较大");
            Debug.LogWarning("  → 可能存在轻微的非刚性形变");
        }
        else
        {
            Debug.Log("✓ 各段误差方向基本一致，刚性变换假设合理");
        }

        Debug.Log("=====================================\n");
    }
}

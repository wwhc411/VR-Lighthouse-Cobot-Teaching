using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using handeye;  // 手眼标定坐标转换器
using MathNet.Numerics.LinearAlgebra;  // Kabsch对齐

/// <summary>
/// CSV 坐标转换工具
/// 功能：读取原始CSV文件（SteamVR坐标系），进行手眼标定坐标变换，在原始数据列后追加转换后的TCP位姿
/// 
/// 使用方法：
///   1. 将此脚本挂载到任意GameObject上
///   2. 在Inspector面板中设置输入CSV文件路径
///   3. 右键点击组件标题 → 选择"执行CSV坐标转换"
/// 
/// 数据转换流程：
///   1. 读取CSV原始数据 (SteamVR坐标系, mm, 四元数)
///   2. 应用Tracker本地坐标系偏移 (可选)
///   3. 应用Kabsch点云刚性对齐校正 - 仅校正位置xyz (可选)
///   4. 手眼标定坐标变换 (SteamVR → UR Base)
///   5. 在原始列后追加转换结果，保存到新CSV文件
/// 
/// 输入格式：FrameNumber,TimeStamp_ms,TimeFromStart_s,X_mm,Y_mm,Z_mm,QX,QY,QZ,QW,RX_rad,RY_rad,RZ_rad,...
/// 输出格式：原始所有列 + HE_TCP_X_m,HE_TCP_Y_m,HE_TCP_Z_m,HE_TCP_RX_rad,HE_TCP_RY_rad,HE_TCP_RZ_rad
/// 更新日期: 2026-01-27
/// </summary>
public class CSVCoordinateTransformer : MonoBehaviour
{
    #region Inspector 配置字段

    [Header("=== CSV文件设置 ===")]
    [Tooltip("输入CSV文件路径（可以是绝对路径或相对于StreamingAssets的路径）")]
    [SerializeField]
    private string inputCsvPath = "";

    [Tooltip("输出CSV文件路径（留空则自动在原文件名后加 _transformed）")]
    [SerializeField]
    private string outputCsvPath = "";

    [Header("=== Tracker偏移设置 ===")]
    [Tooltip("是否启用Tracker本地坐标系偏移")]
    [SerializeField]
    private bool enableTrackerOffset = false;

    [Tooltip("Tracker本地坐标系位置偏移（毫米）")]
    [SerializeField]
    private Vector3 trackerPositionOffset = Vector3.zero;

    [Tooltip("Tracker本地坐标系旋转偏移（欧拉角，度）")]
    [SerializeField]
    private Vector3 trackerRotationOffset = Vector3.zero;

    [Header("=== Kabsch刚性对齐校正 ===")]
    [Tooltip("是否启用Kabsch点云对齐校正（仅校正位置xyz，姿态保持不变）")]
    [SerializeField]
    private bool enableKabschAlignment = false;

    [Tooltip("Kabsch对齐组件引用（需要预先执行对齐计算）")]
    [SerializeField]
    private KabschAlignment kabschAlignmentComponent = null;

    [Header("=== 转换结果 ===")]
    [Tooltip("上次转换是否成功")]
    [SerializeField]
    private bool lastTransformSuccess = false;

    [Tooltip("上次转换的结果消息")]
    [SerializeField]
    [TextArea(2, 4)]
    private string lastTransformMessage = "";

    [Tooltip("上次成功转换的帧数")]
    [SerializeField]
    private int lastTransformedFrames = 0;

    #endregion

    #region 静态配置（兼容旧代码）

    /// <summary>
    /// 是否启用Tracker本地坐标系偏移（静态，供静态方法使用）
    /// </summary>
    public static bool EnableTrackerOffset = false;

    /// <summary>
    /// Tracker本地坐标系位置偏移（毫米）
    /// </summary>
    public static Vector3 TrackerPositionOffset = Vector3.zero;

    /// <summary>
    /// Tracker本地坐标系旋转偏移（欧拉角，度）
    /// </summary>
    public static Vector3 TrackerRotationOffset = Vector3.zero;

    /// <summary>
    /// 是否启用Kabsch点云对齐校正（静态，供静态方法使用）
    /// </summary>
    public static bool EnableKabschAlignment = false;

    /// <summary>
    /// Kabsch对齐旋转矩阵（3x3）
    /// </summary>
    private static Matrix<double> kabschRotationMatrix = null;

    /// <summary>
    /// Kabsch对齐平移向量（3x1）
    /// </summary>
    private static Vector<double> kabschTranslationVector = null;

    #endregion

    #region 右键菜单方法

    /// <summary>
    /// 执行CSV坐标转换（右键菜单）
    /// </summary>
    [ContextMenu("执行CSV坐标转换")]
    public void ExecuteTransform()
    {
        if (string.IsNullOrEmpty(inputCsvPath))
        {
            Debug.LogError("[CSVCoordinateTransformer] 请先在Inspector中设置输入CSV文件路径！");
            lastTransformSuccess = false;
            lastTransformMessage = "错误：未设置输入CSV文件路径";
            return;
        }

        // 同步实例配置到静态变量
        SyncInstanceConfigToStatic();

        // 执行转换
        string output = string.IsNullOrEmpty(outputCsvPath) ? null : outputCsvPath;
        TransformResult result = TransformCSVFile(inputCsvPath, output);

        // 保存结果到Inspector
        lastTransformSuccess = result.Success;
        lastTransformMessage = result.Message;
        lastTransformedFrames = result.TransformedFrames;

        if (result.Success)
        {
            Debug.Log($"<color=green>[CSVCoordinateTransformer] 转换成功！</color> {result.Message}");
        }
    }

    /// <summary>
    /// 选择输入CSV文件（右键菜单）
    /// </summary>
    [ContextMenu("选择输入CSV文件")]
    public void SelectInputFile()
    {
#if UNITY_EDITOR
        string defaultPath = string.IsNullOrEmpty(inputCsvPath) 
            ? Application.streamingAssetsPath 
            : Path.GetDirectoryName(inputCsvPath);
        
        string selectedPath = UnityEditor.EditorUtility.OpenFilePanel(
            "选择输入CSV文件", 
            defaultPath, 
            "csv"
        );
        
        if (!string.IsNullOrEmpty(selectedPath))
        {
            inputCsvPath = selectedPath;
            Debug.Log($"[CSVCoordinateTransformer] 已选择输入文件: {inputCsvPath}");
        }
#else
        Debug.LogWarning("[CSVCoordinateTransformer] 文件选择功能仅在编辑器中可用");
#endif
    }

    /// <summary>
    /// 选择输出CSV文件（右键菜单）
    /// </summary>
    [ContextMenu("选择输出CSV文件")]
    public void SelectOutputFile()
    {
#if UNITY_EDITOR
        string defaultPath = string.IsNullOrEmpty(outputCsvPath) 
            ? (string.IsNullOrEmpty(inputCsvPath) ? Application.streamingAssetsPath : Path.GetDirectoryName(inputCsvPath))
            : Path.GetDirectoryName(outputCsvPath);
        
        string defaultName = string.IsNullOrEmpty(inputCsvPath) 
            ? "output_transformed.csv" 
            : Path.GetFileNameWithoutExtension(inputCsvPath) + "_transformed.csv";
        
        string selectedPath = UnityEditor.EditorUtility.SaveFilePanel(
            "选择输出CSV文件保存位置", 
            defaultPath, 
            defaultName,
            "csv"
        );
        
        if (!string.IsNullOrEmpty(selectedPath))
        {
            outputCsvPath = selectedPath;
            Debug.Log($"[CSVCoordinateTransformer] 已设置输出文件: {outputCsvPath}");
        }
#else
        Debug.LogWarning("[CSVCoordinateTransformer] 文件选择功能仅在编辑器中可用");
#endif
    }

    /// <summary>
    /// 打开输出文件所在目录（右键菜单）
    /// </summary>
    [ContextMenu("打开输出文件目录")]
    public void OpenOutputDirectory()
    {
#if UNITY_EDITOR
        string pathToOpen = outputCsvPath;
        if (string.IsNullOrEmpty(pathToOpen) && !string.IsNullOrEmpty(inputCsvPath))
        {
            // 根据输入路径推断输出路径
            string directory = Path.GetDirectoryName(inputCsvPath);
            string fileNameWithoutExt = Path.GetFileNameWithoutExtension(inputCsvPath);
            pathToOpen = Path.Combine(directory, $"{fileNameWithoutExt}_transformed.csv");
        }

        if (!string.IsNullOrEmpty(pathToOpen) && File.Exists(pathToOpen))
        {
            UnityEditor.EditorUtility.RevealInFinder(pathToOpen);
        }
        else if (!string.IsNullOrEmpty(inputCsvPath))
        {
            string directory = Path.GetDirectoryName(inputCsvPath);
            if (Directory.Exists(directory))
            {
                UnityEditor.EditorUtility.RevealInFinder(directory);
            }
        }
        else
        {
            Debug.LogWarning("[CSVCoordinateTransformer] 未设置文件路径，无法打开目录");
        }
#else
        Debug.LogWarning("[CSVCoordinateTransformer] 此功能仅在编辑器中可用");
#endif
    }

    /// <summary>
    /// 重置配置（右键菜单）
    /// </summary>
    [ContextMenu("重置所有配置")]
    public void ResetConfig()
    {
        inputCsvPath = "";
        outputCsvPath = "";
        enableTrackerOffset = false;
        trackerPositionOffset = Vector3.zero;
        trackerRotationOffset = Vector3.zero;
        lastTransformSuccess = false;
        lastTransformMessage = "";
        lastTransformedFrames = 0;
        Debug.Log("[CSVCoordinateTransformer] 配置已重置");
    }

    /// <summary>
    /// 同步实例配置到静态变量
    /// </summary>
    private void SyncInstanceConfigToStatic()
    {
        EnableTrackerOffset = enableTrackerOffset;
        TrackerPositionOffset = trackerPositionOffset;
        TrackerRotationOffset = trackerRotationOffset;
        EnableKabschAlignment = enableKabschAlignment;
        
        // 同步Kabsch变换矩阵
        if (enableKabschAlignment && kabschAlignmentComponent != null)
        {
            if (!kabschAlignmentComponent.IsAlignmentComputed)
            {
                Debug.LogError("[CSVCoordinateTransformer] Kabsch对齐组件尚未执行对齐计算！");
                EnableKabschAlignment = false;
            }
            else
            {
                kabschRotationMatrix = kabschAlignmentComponent.RotationMatrix;
                kabschTranslationVector = kabschAlignmentComponent.TranslationVector;
                Debug.Log($"[CSVCoordinateTransformer] Kabsch变换已加载 - RMSE: {kabschAlignmentComponent.RMSE:F6}");
            }
        }
        else if (enableKabschAlignment)
        {
            Debug.LogError("[CSVCoordinateTransformer] 启用了Kabsch但未指定KabschAlignment组件！");
            EnableKabschAlignment = false;
        }
    }

    #endregion

    #region 数据结构定义

    /// <summary>
    /// 转换后的单帧数据结构
    /// </summary>
    [Serializable]
    public class TransformedFrameData
    {
        public int FrameNumber;         // 帧序号
        public long TimeStamp_ms;       // Unix时间戳（毫秒）
        public double TimeFromStart_s;  // 相对起始时间（秒）
        
        // 原始数据（SteamVR坐标系）
        public double OriginalX_mm;     // 原始位置X（毫米）
        public double OriginalY_mm;     // 原始位置Y（毫米）
        public double OriginalZ_mm;     // 原始位置Z（毫米）
        public double OriginalQX;       // 原始四元数X
        public double OriginalQY;       // 原始四元数Y
        public double OriginalQZ;       // 原始四元数Z
        public double OriginalQW;       // 原始四元数W

        // 转换后数据（UR基座坐标系）- 手眼标定后的TCP位姿
        public double HE_TCP_X_m;       // 手眼变换后TCP位置X（米）
        public double HE_TCP_Y_m;       // 手眼变换后TCP位置Y（米）
        public double HE_TCP_Z_m;       // 手眼变换后TCP位置Z（米）
        public double HE_TCP_RX_rad;    // 手眼变换后TCP旋转矢量X（弧度）
        public double HE_TCP_RY_rad;    // 手眼变换后TCP旋转矢量Y（弧度）
        public double HE_TCP_RZ_rad;    // 手眼变换后TCP旋转矢量Z（弧度）
        
        // 原始CSV行内容（用于保留所有原始列）
        public string OriginalCsvLine;
    }

    /// <summary>
    /// 转换结果汇总
    /// </summary>
    public class TransformResult
    {
        public bool Success;                            // 是否成功
        public string Message;                          // 结果消息
        public string OutputFilePath;                   // 输出文件路径
        public int TotalFrames;                         // 总帧数
        public int TransformedFrames;                   // 成功转换帧数
        public int FailedFrames;                        // 失败帧数
        public List<TransformedFrameData> FrameDataList; // 转换后的数据列表
    }

    #endregion

    #region 静态转换方法

    /// <summary>
    /// 上一帧的旋转矢量（用于连续性校正）
    /// </summary>
    private static Vector3 previousRotationVector = Vector3.zero;
    
    /// <summary>
    /// 是否已初始化上一帧旋转矢量
    /// </summary>
    private static bool hasPreviousRotation = false;

    /// <summary>
    /// 读取CSV文件并进行坐标转换，在原始列后追加手眼变换后的TCP位姿，结果保存到新文件
    /// </summary>
    /// <param name="inputCsvPath">输入CSV文件路径</param>
    /// <param name="outputCsvPath">输出CSV文件路径（可选，默认在原文件名后加 _transformed）</param>
    /// <returns>转换结果</returns>
    public static TransformResult TransformCSVFile(string inputCsvPath, string outputCsvPath = null)
    {
        // 重置连续性状态
        previousRotationVector = Vector3.zero;
        hasPreviousRotation = false;
        
        TransformResult result = new TransformResult
        {
            Success = false,
            FrameDataList = new List<TransformedFrameData>()
        };

        try
        {
            // 1. 检查文件是否存在
            string fullPath = inputCsvPath;
            if (!Path.IsPathRooted(inputCsvPath))
            {
                fullPath = Path.Combine(Application.streamingAssetsPath, inputCsvPath);
            }

            if (!File.Exists(fullPath))
            {
                result.Message = $"文件不存在: {fullPath}";
                Debug.LogError($"[CSVCoordinateTransformer] {result.Message}");
                return result;
            }

            // 2. 读取原始CSV所有行
            string[] lines = File.ReadAllLines(fullPath);
            if (lines.Length < 2)
            {
                result.Message = $"CSV文件数据不足: {fullPath}";
                Debug.LogError($"[CSVCoordinateTransformer] {result.Message}");
                return result;
            }

            // 3. 找到表头行（跳过注释行）
            int headerLineIndex = 0;
            while (headerLineIndex < lines.Length && lines[headerLineIndex].TrimStart().StartsWith("#"))
            {
                headerLineIndex++;
            }

            if (headerLineIndex >= lines.Length)
            {
                result.Message = $"CSV文件没有有效表头: {fullPath}";
                Debug.LogError($"[CSVCoordinateTransformer] {result.Message}");
                return result;
            }

            string originalHeader = lines[headerLineIndex].Trim();
            int dataStartLine = headerLineIndex + 1;
            result.TotalFrames = lines.Length - dataStartLine;

            Debug.Log($"[CSVCoordinateTransformer] 已读取 {result.TotalFrames} 行数据");

            // 4. 确定输出路径
            if (string.IsNullOrEmpty(outputCsvPath))
            {
                string directory = Path.GetDirectoryName(fullPath);
                string fileNameWithoutExt = Path.GetFileNameWithoutExtension(fullPath);
                outputCsvPath = Path.Combine(directory, $"{fileNameWithoutExt}_transformed.csv");
            }
            result.OutputFilePath = outputCsvPath;

            // 5. 解析表头，找到位置和四元数列的索引
            string[] headerColumns = originalHeader.Split(',');
            int xIndex = -1, yIndex = -1, zIndex = -1;
            int qxIndex = -1, qyIndex = -1, qzIndex = -1, qwIndex = -1;

            for (int i = 0; i < headerColumns.Length; i++)
            {
                string col = headerColumns[i].Trim().ToLower();
                if (col == "x_mm") xIndex = i;
                else if (col == "y_mm") yIndex = i;
                else if (col == "z_mm") zIndex = i;
                else if (col == "qx") qxIndex = i;
                else if (col == "qy") qyIndex = i;
                else if (col == "qz") qzIndex = i;
                else if (col == "qw") qwIndex = i;
            }

            if (xIndex < 0 || yIndex < 0 || zIndex < 0 || qxIndex < 0 || qyIndex < 0 || qzIndex < 0 || qwIndex < 0)
            {
                result.Message = $"CSV表头缺少必需列 (X_mm, Y_mm, Z_mm, QX, QY, QZ, QW)";
                Debug.LogError($"[CSVCoordinateTransformer] {result.Message}");
                return result;
            }

            // 6. 逐行进行坐标转换
            CultureInfo invariant = CultureInfo.InvariantCulture;

            for (int i = dataStartLine; i < lines.Length; i++)
            {
                string line = lines[i].Trim();
                if (string.IsNullOrEmpty(line)) continue;

                string[] columns = line.Split(',');
                if (columns.Length <= Math.Max(Math.Max(xIndex, yIndex), Math.Max(zIndex, qwIndex)))
                {
                    result.FailedFrames++;
                    continue;
                }

                try
                {
                    // 解析位置和四元数
                    float x_mm = float.Parse(columns[xIndex], invariant);
                    float y_mm = float.Parse(columns[yIndex], invariant);
                    float z_mm = float.Parse(columns[zIndex], invariant);
                    float qx = float.Parse(columns[qxIndex], invariant);
                    float qy = float.Parse(columns[qyIndex], invariant);
                    float qz = float.Parse(columns[qzIndex], invariant);
                    float qw = float.Parse(columns[qwIndex], invariant);

                    // 检查数据有效性
                    if (x_mm > 9999998.0f || y_mm > 9999998.0f || z_mm > 9999998.0f)
                    {
                        result.FailedFrames++;
                        continue;
                    }

                    Vector3 posSteamVr_mm = new Vector3(x_mm, y_mm, z_mm);
                    Quaternion quatSteamVr = new Quaternion(qx, qy, qz, qw);

                    // 应用Tracker本地坐标系偏移（如果启用）
                    if (EnableTrackerOffset)
                    {
                        Vector3 worldOffsetMm = quatSteamVr * TrackerPositionOffset;
                        posSteamVr_mm = posSteamVr_mm + worldOffsetMm;

                        if (TrackerRotationOffset != Vector3.zero)
                        {
                            Quaternion rotationOffsetQuat = Quaternion.Euler(TrackerRotationOffset);
                            quatSteamVr = quatSteamVr * rotationOffsetQuat;
                        }
                    }

                    // 应用Kabsch点云刚性对齐校正（如果启用）
                    // 注意：Kabsch只校正位置xyz，姿态保持不变
                    if (EnableKabschAlignment)
                    {
                        Vector3 posSteamVr_m = posSteamVr_mm * 0.001f;  // 转为米
                        if (ApplyKabschTransform(posSteamVr_m, out Vector3 kabschPos_m))
                        {
                            posSteamVr_mm = kabschPos_m * 1000f;  // 转回毫米
                        }
                    }

                    // 调用手眼标定坐标转换
                    SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                        posSteamVr_mm,
                        quatSteamVr,
                        posInMillimeters: true,
                        out Vector3 posUr_m,
                        out Vector3 rotUr_rad
                    );

                    // 应用旋转矢量连续性校正，避免π边界跳变
                    rotUr_rad = EnsureRotationVectorContinuity(rotUr_rad);

                    // 创建转换后的帧数据
                    TransformedFrameData transformedFrame = new TransformedFrameData
                    {
                        OriginalCsvLine = line,
                        OriginalX_mm = x_mm,
                        OriginalY_mm = y_mm,
                        OriginalZ_mm = z_mm,
                        OriginalQX = qx,
                        OriginalQY = qy,
                        OriginalQZ = qz,
                        OriginalQW = qw,
                        HE_TCP_X_m = posUr_m.x,
                        HE_TCP_Y_m = posUr_m.y,
                        HE_TCP_Z_m = posUr_m.z,
                        HE_TCP_RX_rad = rotUr_rad.x,
                        HE_TCP_RY_rad = rotUr_rad.y,
                        HE_TCP_RZ_rad = rotUr_rad.z
                    };

                    result.FrameDataList.Add(transformedFrame);
                    result.TransformedFrames++;
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"[CSVCoordinateTransformer] 第{i + 1}行解析失败: {ex.Message}");
                    result.FailedFrames++;
                }
            }

            // 7. 写入新的CSV文件（原始列 + 手眼变换后的TCP列）
            WriteAppendedCSV(originalHeader, result.FrameDataList, outputCsvPath);

            result.Success = true;
            result.Message = $"转换完成！成功: {result.TransformedFrames}/{result.TotalFrames} 帧，输出: {outputCsvPath}";
            Debug.Log($"[CSVCoordinateTransformer] {result.Message}");

            return result;
        }
        catch (Exception ex)
        {
            result.Message = $"转换失败: {ex.Message}";
            Debug.LogError($"[CSVCoordinateTransformer] {result.Message}");
            Debug.LogError($"  异常详情: {ex.StackTrace}");
            return result;
        }
    }

    /// <summary>
    /// 应用Kabsch刚性变换到位置
    /// 注意: Kabsch对齐只校正位置xyz，姿态保持不变（因为训练点云只包含位置信息）
    /// 变换公式: p' = R * p + t
    /// </summary>
    /// <param name="position">输入位置（米）</param>
    /// <param name="transformedPosition">输出变换后的位置（米）</param>
    /// <returns>是否成功应用变换</returns>
    private static bool ApplyKabschTransform(Vector3 position, out Vector3 transformedPosition)
    {
        transformedPosition = position;

        if (!EnableKabschAlignment || kabschRotationMatrix == null || kabschTranslationVector == null)
        {
            return false;  // 未启用或未设置Kabsch变换
        }

        try
        {
            // 位置变换: p' = R * p + t
            var posVec = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(new double[] 
            { 
                position.x, position.y, position.z 
            });
            var transformedPosVec = kabschRotationMatrix.Multiply(posVec) + kabschTranslationVector;
            
            transformedPosition = new Vector3(
                (float)transformedPosVec[0],
                (float)transformedPosVec[1],
                (float)transformedPosVec[2]
            );

            return true;
        }
        catch (Exception e)
        {
            Debug.LogError($"[CSVCoordinateTransformer] Kabsch变换失败: {e.Message}");
            return false;
        }
    }

    /// <summary>
    /// 将原始数据列后追加手眼变换后的TCP数据，写入CSV文件
    /// </summary>
    /// <param name="originalHeader">原始表头</param>
    /// <param name="dataList">转换后的数据列表</param>
    /// <param name="outputPath">输出文件路径</param>
    private static void WriteAppendedCSV(string originalHeader, List<TransformedFrameData> dataList, string outputPath)
    {
        StringBuilder sb = new StringBuilder();
        CultureInfo invariant = CultureInfo.InvariantCulture;

        // 写入新表头：原始表头 + 手眼变换后的TCP列
        string newHeader = originalHeader + ",HE_TCP_X_m,HE_TCP_Y_m,HE_TCP_Z_m,HE_TCP_RX_rad,HE_TCP_RY_rad,HE_TCP_RZ_rad";
        sb.AppendLine(newHeader);

        // 写入数据行：原始行 + 手眼变换后的TCP数据
        foreach (var frame in dataList)
        {
            string appendedData = string.Format(invariant,
                "{0},{1:F6},{2:F6},{3:F6},{4:F6},{5:F6},{6:F6}",
                frame.OriginalCsvLine,
                frame.HE_TCP_X_m, frame.HE_TCP_Y_m, frame.HE_TCP_Z_m,
                frame.HE_TCP_RX_rad, frame.HE_TCP_RY_rad, frame.HE_TCP_RZ_rad
            );
            sb.AppendLine(appendedData);
        }

        // 确保输出目录存在
        string directory = Path.GetDirectoryName(outputPath);
        if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
        {
            Directory.CreateDirectory(directory);
        }

        // 写入文件
        File.WriteAllText(outputPath, sb.ToString(), Encoding.UTF8);
        Debug.Log($"[CSVCoordinateTransformer] 已保存转换结果: {outputPath}");
    }

    /// <summary>
    /// 批量转换多个CSV文件
    /// </summary>
    /// <param name="inputCsvPaths">输入CSV文件路径列表</param>
    /// <param name="outputDirectory">输出目录（可选，默认与输入文件同目录）</param>
    /// <returns>所有转换结果列表</returns>
    public static List<TransformResult> TransformMultipleCSVFiles(string[] inputCsvPaths, string outputDirectory = null)
    {
        List<TransformResult> results = new List<TransformResult>();

        foreach (string inputPath in inputCsvPaths)
        {
            string outputPath = null;
            if (!string.IsNullOrEmpty(outputDirectory))
            {
                string fileName = Path.GetFileNameWithoutExtension(inputPath) + "_transformed.csv";
                outputPath = Path.Combine(outputDirectory, fileName);
            }

            TransformResult result = TransformCSVFile(inputPath, outputPath);
            results.Add(result);
        }

        // 输出汇总
        int successCount = 0;
        int failCount = 0;
        foreach (var r in results)
        {
            if (r.Success) successCount++;
            else failCount++;
        }
        Debug.Log($"[CSVCoordinateTransformer] 批量转换完成: 成功 {successCount}/{results.Count}");

        return results;
    }

    /// <summary>
    /// 配置Tracker偏移参数
    /// </summary>
    /// <param name="enable">是否启用偏移</param>
    /// <param name="positionOffset">位置偏移（毫米）</param>
    /// <param name="rotationOffset">旋转偏移（欧拉角，度）</param>
    public static void ConfigureTrackerOffset(bool enable, Vector3 positionOffset, Vector3 rotationOffset)
    {
        EnableTrackerOffset = enable;
        TrackerPositionOffset = positionOffset;
        TrackerRotationOffset = rotationOffset;
        Debug.Log($"[CSVCoordinateTransformer] Tracker偏移配置: 启用={enable}, 位置={positionOffset}mm, 旋转={rotationOffset}deg");
    }

    /// <summary>
    /// 旋转矢量连续性校正
    /// 
    /// 问题背景：
    ///   旋转矢量 r = θ * axis 的表示存在不连续性：
    ///   1. 同一旋转可以用 (θ, axis) 或 (2π-θ, -axis) 表示
    ///   2. 当θ穿越π时，旋转矢量会发生符号翻转导致剧烈跳变
    ///   3. 当θ接近π时，旋转轴方向可能不稳定导致数值抖动
    /// 
    /// 解决方案：
    ///   1. 检测与上一帧的差异是否过大
    ///   2. 尝试多种等效表示，选择最连续的
    ///   3. 对于异常跳变，使用上一帧进行平滑插值
    /// 
    /// 适用场景：
    ///   连续轨迹的坐标转换（如CSV批量转换）
    /// </summary>
    /// <param name="currentRotVec">当前帧的旋转矢量</param>
    /// <returns>连续性校正后的旋转矢量</returns>
    private static Vector3 EnsureRotationVectorContinuity(Vector3 currentRotVec)
    {
        const float PI = Mathf.PI;
        const float TWO_PI = 2f * Mathf.PI;
        
        // 第一帧，直接记录并返回
        if (!hasPreviousRotation)
        {
            previousRotationVector = currentRotVec;
            hasPreviousRotation = true;
            return currentRotVec;
        }

        // 计算当前旋转矢量的模（旋转角度）
        float currentAngle = currentRotVec.magnitude;
        float prevAngle = previousRotationVector.magnitude;
        
        // 如果当前角度接近0但上一帧角度较大，说明可能是数值不稳定
        // 这种情况下使用上一帧的值
        if (currentAngle < 0.1f && prevAngle > 1.0f)
        {
            // 异常跳变到接近零，保持上一帧的值
            Debug.LogWarning($"[旋转连续性] 检测到异常跳变(角度突变到接近0): {currentAngle:F4} rad, 使用上一帧值");
            return previousRotationVector;
        }

        // 计算与上一帧的差值
        Vector3 diff = currentRotVec - previousRotationVector;
        float diffMag = diff.magnitude;

        // 如果差值较小（小于π/2），认为是连续的
        if (diffMag < PI * 0.5f)
        {
            previousRotationVector = currentRotVec;
            return currentRotVec;
        }

        // 差值较大，尝试多种等效表示
        Vector3 bestResult = currentRotVec;
        float bestDiff = diffMag;

        // 等效表示1: r' = -axis * (2π - θ)
        if (currentAngle > 0.001f)
        {
            Vector3 axis = currentRotVec / currentAngle;
            float altAngle1 = TWO_PI - currentAngle;
            Vector3 altRotVec1 = -axis * altAngle1;
            float altDiff1 = (altRotVec1 - previousRotationVector).magnitude;
            if (altDiff1 < bestDiff)
            {
                bestResult = altRotVec1;
                bestDiff = altDiff1;
            }
        }

        // 等效表示2: r' = axis * (θ - 2π) （当θ > π时）
        if (currentAngle > PI)
        {
            Vector3 axis = currentRotVec / currentAngle;
            float altAngle2 = currentAngle - TWO_PI;
            Vector3 altRotVec2 = axis * altAngle2;
            float altDiff2 = (altRotVec2 - previousRotationVector).magnitude;
            if (altDiff2 < bestDiff)
            {
                bestResult = altRotVec2;
                bestDiff = altDiff2;
            }
        }

        // 等效表示3: r' = axis * (θ + 2π) （当θ < -π时，虽然通常θ>=0）
        if (currentAngle > 0.001f && currentAngle < PI)
        {
            Vector3 axis = currentRotVec / currentAngle;
            float altAngle3 = -(TWO_PI - currentAngle);
            Vector3 altRotVec3 = -axis * Mathf.Abs(altAngle3);
            float altDiff3 = (altRotVec3 - previousRotationVector).magnitude;
            if (altDiff3 < bestDiff)
            {
                bestResult = altRotVec3;
                bestDiff = altDiff3;
            }
        }

        // 如果所有等效表示都无法使差异小于阈值，可能是数值不稳定
        // 使用渐进式平滑：限制单帧最大变化量
        const float maxSingleFrameChange = 0.5f; // 单帧最大变化约0.5 rad ≈ 28°
        if (bestDiff > maxSingleFrameChange)
        {
            // 限制变化量，向目标方向移动但不超过最大步长
            Vector3 direction = (bestResult - previousRotationVector).normalized;
            bestResult = previousRotationVector + direction * maxSingleFrameChange;
            Debug.LogWarning($"[旋转连续性] 差异过大({bestDiff:F3}rad)，限制为最大步长{maxSingleFrameChange}rad");
        }

        previousRotationVector = bestResult;
        return bestResult;
    }

    #endregion

    #region 编辑器便捷方法

#if UNITY_EDITOR
    /// <summary>
    /// 在Unity编辑器中通过菜单调用的便捷转换方法
    /// </summary>
    [UnityEditor.MenuItem("Tools/CSV坐标转换/选择文件并转换")]
    public static void TransformCSVFromEditor()
    {
        string inputPath = UnityEditor.EditorUtility.OpenFilePanel("选择要转换的CSV文件", Application.streamingAssetsPath, "csv");
        if (string.IsNullOrEmpty(inputPath))
        {
            Debug.Log("[CSVCoordinateTransformer] 用户取消选择");
            return;
        }

        TransformResult result = TransformCSVFile(inputPath);

        if (result.Success)
        {
            UnityEditor.EditorUtility.DisplayDialog(
                "转换成功",
                $"成功转换 {result.TransformedFrames} 帧\n输出文件: {result.OutputFilePath}\n\n新增列: HE_TCP_X_m, HE_TCP_Y_m, HE_TCP_Z_m, HE_TCP_RX_rad, HE_TCP_RY_rad, HE_TCP_RZ_rad",
                "确定"
            );

            // 在资源管理器中显示文件
            UnityEditor.EditorUtility.RevealInFinder(result.OutputFilePath);
        }
        else
        {
            UnityEditor.EditorUtility.DisplayDialog(
                "转换失败",
                result.Message,
                "确定"
            );
        }
    }

    /// <summary>
    /// 配置Tracker偏移的编辑器菜单
    /// </summary>
    [UnityEditor.MenuItem("Tools/CSV坐标转换/配置Tracker偏移")]
    public static void ConfigureTrackerOffsetFromEditor()
    {
        // 简单的对话框配置（实际项目中可使用更复杂的编辑器窗口）
        EnableTrackerOffset = UnityEditor.EditorUtility.DisplayDialog(
            "Tracker偏移配置",
            $"当前状态: {(EnableTrackerOffset ? "启用" : "禁用")}\n位置偏移: {TrackerPositionOffset}mm\n旋转偏移: {TrackerRotationOffset}deg\n\n是否启用Tracker偏移？",
            "启用",
            "禁用"
        );
        Debug.Log($"[CSVCoordinateTransformer] Tracker偏移: {(EnableTrackerOffset ? "已启用" : "已禁用")}");
    }
#endif

    #endregion
}

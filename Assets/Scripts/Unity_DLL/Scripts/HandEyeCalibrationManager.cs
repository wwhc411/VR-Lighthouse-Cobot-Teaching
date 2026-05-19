using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using HandEyeCalibration.DLL;

namespace HandEyeCalibration
{
    /// <summary>
    /// 标定结果数据
    /// </summary>
    [Serializable]
    public class CalibrationResult
    {
        public bool IsSuccess;
        public string ErrorMessage;
        public Pose_Unity CameraToBaseTransform;
        public string ResultText;
        
        // 变换矩阵的各个组成部分（便于UI显示和使用）
        public Matrix4x4 TransformMatrix;
        public Vector3 Position;
        public Quaternion Rotation;
    }

    /// <summary>
    /// 机器人位姿数据（6DOF: X Y Z RX RY RZ）
    /// </summary>
    [Serializable]
    public class RobotPoseData
    {
        public double X, Y, Z;           // 位置（毫米）
        public double RX, RY, RZ;        // 旋转向量（弧度）

        public RobotPoseData(double x, double y, double z, double rx, double ry, double rz)
        {
            X = x; Y = y; Z = z;
            RX = rx; RY = ry; RZ = rz;
        }

        public override string ToString()
        {
            return $"{X:F2} {Y:F2} {Z:F2} {RX:F4} {RY:F4} {RZ:F4}";
        }
    }

    /// <summary>
    /// 相机位姿数据（7DOF: X Y Z QX QY QZ QW）
    /// </summary>
    [Serializable]
    public class CameraPoseData
    {
        public double X, Y, Z;                // 位置（毫米）
        public double QX, QY, QZ, QW;         // 四元数

        public CameraPoseData(double x, double y, double z, double qx, double qy, double qz, double qw)
        {
            X = x; Y = y; Z = z;
            QX = qx; QY = qy; QZ = qz; QW = qw;
        }

        public override string ToString()
        {
            // 输出格式：X Y Z QX QY QZ QW（与输入格式保持一致）
            return $"{X:F2} {Y:F2} {Z:F2} {QX:F4} {QY:F4} {QZ:F4} {QW:F4}";
        }
    }

    /// <summary>
    /// 手眼标定管理器（基于自制DLL实现）
    /// 核心功能：调用myDll.dll进行手眼标定计算
    /// </summary>
    public class HandEyeCalibrationManager : MonoBehaviour
    {
        [Header("标定数据")]
        [SerializeField] private List<RobotPoseData> robotPoses = new List<RobotPoseData>();
        [SerializeField] private List<CameraPoseData> cameraPoses = new List<CameraPoseData>();

        [Header("标定结果")]
        [SerializeField] private CalibrationResult lastResult;

        // 事件回调
        public event Action<CalibrationResult> OnCalibrationCompleted;
        public event Action<string> OnStatusUpdated;

        private void Start()
        {
            // 测试DLL连接
            UpdateStatus("初始化手眼标定系统...");
            if (DllInterface.TestDllConnection())
            {
                UpdateStatus("DLL加载成功,系统就绪");
            }
            else
            {
                UpdateStatus("警告:DLL加载失败!请检查Plugins文件夹中的myDll.dll");
            }
        }

        #region 数据管理

        /// <summary>
        /// 添加单组标定数据
        /// </summary>
        public void AddCalibrationData(RobotPoseData robotPose, CameraPoseData cameraPose)
        {
            robotPoses.Add(robotPose);
            cameraPoses.Add(cameraPose);
            UpdateStatus($"已添加第 {robotPoses.Count} 组标定数据");
        }

        /// <summary>
        /// 批量添加标定数据（从文本解析）
        /// </summary>
        public void AddCalibrationDataFromText(string robotDataText, string cameraDataText)
        {
            try
            {
                var robotLines = robotDataText.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
                var cameraLines = cameraDataText.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

                if (robotLines.Length != cameraLines.Length)
                {
                    UpdateStatus($"错误:数据行数不匹配(机器人{robotLines.Length}行,相机{cameraLines.Length}行)");
                    return;
                }

                int addedCount = 0;
                for (int i = 0; i < robotLines.Length; i++)
                {
                    // 解析机器人数据:X Y Z RX RY RZ
                    var robotValues = ParseDoubleArray(robotLines[i], 6);
                    if (robotValues == null)
                    {
                        UpdateStatus($"警告:机器人数据第{i + 1}行格式错误,已跳过");
                        continue;
                    }

                    // 解析相机数据:X Y Z QW QX QY QZ(注意顺序)
                    var cameraValues = ParseDoubleArray(cameraLines[i], 7);
                    if (cameraValues == null)
                    {
                        UpdateStatus($"警告:相机数据第{i + 1}行格式错误,已跳过");
                        continue;
                    }

                    // 添加数据
                    robotPoses.Add(new RobotPoseData(
                        robotValues[0], robotValues[1], robotValues[2],
                        robotValues[3], robotValues[4], robotValues[5]));

                    // 修正：UI输入格式为 X Y Z QX QY QZ QW (索引0-6)
                    cameraPoses.Add(new CameraPoseData(
                        cameraValues[0], cameraValues[1], cameraValues[2],  // X Y Z
                        cameraValues[3], cameraValues[4], cameraValues[5],  // QX QY QZ
                        cameraValues[6]));                                   // QW

                    addedCount++;
                }

                UpdateStatus($"成功添加 {addedCount} 组标定数据,总计 {robotPoses.Count} 组");
            }
            catch (Exception ex)
            {
                UpdateStatus($"数据解析错误:{ex.Message}");
                Debug.LogError($"[标定管理器] 数据解析失败: {ex.Message}\n{ex.StackTrace}");
            }
        }

        /// <summary>
        /// 清空所有标定数据
        /// </summary>
        public void ClearCalibrationData()
        {
            robotPoses.Clear();
            cameraPoses.Clear();
            UpdateStatus("已清空所有标定数据");
        }

        /// <summary>
        /// 获取当前数据数量
        /// </summary>
        public int GetDataCount()
        {
            return robotPoses.Count;
        }

        #endregion

        #region 标定执行

        /// <summary>
        /// 执行手眼标定（调用DLL）
        /// </summary>
        public CalibrationResult PerformCalibration()
        {
            var result = new CalibrationResult();
            var logBuilder = new StringBuilder();

            try
            {
                logBuilder.AppendLine("=== Hand-Eye Calibration - Eye-to-Hand ===");
                logBuilder.AppendLine("Using Custom DLL - Tsai Algorithm");
                logBuilder.AppendLine($"Calibration Points: {robotPoses.Count}");
                logBuilder.AppendLine();

                // 检查数据有效性
                if (robotPoses.Count < 3)
                {
                    result.IsSuccess = false;
                    result.ErrorMessage = "Insufficient calibration points! At least 3 data sets required (5-10 recommended)";
                    result.ResultText = logBuilder.ToString() + "\nError: " + result.ErrorMessage;
                    UpdateStatus("标定点数量不足!至少需要3组数据(推荐5-10组)");
                    return result;
                }

                if (robotPoses.Count != cameraPoses.Count)
                {
                    result.IsSuccess = false;
                    result.ErrorMessage = "Robot data and camera data count mismatch";
                    result.ResultText = logBuilder.ToString() + "\nError: " + result.ErrorMessage;
                    UpdateStatus("机器人数据和相机数据数量不匹配");
                    return result;
                }

                UpdateStatus($"开始标定... 使用 {robotPoses.Count} 组数据");

                // Prepare DLL input data
                double[,] robotData = new double[robotPoses.Count, 6];
                double[,] cameraData = new double[cameraPoses.Count, 7];

                for (int i = 0; i < robotPoses.Count; i++)
                {
                    robotData[i, 0] = robotPoses[i].X;
                    robotData[i, 1] = robotPoses[i].Y;
                    robotData[i, 2] = robotPoses[i].Z;
                    robotData[i, 3] = robotPoses[i].RX;
                    robotData[i, 4] = robotPoses[i].RY;
                    robotData[i, 5] = robotPoses[i].RZ;

                    cameraData[i, 0] = cameraPoses[i].X;
                    cameraData[i, 1] = cameraPoses[i].Y;
                    cameraData[i, 2] = cameraPoses[i].Z;
                    cameraData[i, 3] = cameraPoses[i].QX;
                    cameraData[i, 4] = cameraPoses[i].QY;
                    cameraData[i, 5] = cameraPoses[i].QZ;
                    cameraData[i, 6] = cameraPoses[i].QW;
                }

                // Create DLL input structure
                Point_Unity gripperToBase = DllInterface.CreateGripperToBaseData(robotData);
                Point_Unity targetToCam = DllInterface.CreateTargetToCamData(cameraData);

                // Call DLL for calibration
                Pose_Unity camToBase;
                bool success = DllInterface.CalculateHandAndEye(
                    gripperToBase,
                    targetToCam,
                    out camToBase);

                if (!success)
                {
                    result.IsSuccess = false;
                    result.ErrorMessage = "DLL calibration calculation failed, please check console logs";
                    result.ResultText = logBuilder.ToString() + "\nError: " + result.ErrorMessage;
                    UpdateStatus("DLL标定计算失败,请检查控制台日志");
                    return result;
                }

                // Save result
                result.IsSuccess = true;
                result.CameraToBaseTransform = camToBase;
                result.Position = camToBase.Position.ToUnityVector3();
                result.Rotation = camToBase.Quaternion.ToUnityQuaternion();
                result.TransformMatrix = Matrix4x4.TRS(result.Position, result.Rotation, Vector3.one);

                // Generate result text
                logBuilder.AppendLine("=== Calibration Success ===");
                logBuilder.AppendLine("\nCamera to Robot Base Transform (Cam → Base):");
                logBuilder.AppendLine($"  Position (mm): {camToBase.Position}");
                logBuilder.AppendLine($"  Rotation (Quaternion): {camToBase.Quaternion}");
                logBuilder.AppendLine();
                logBuilder.AppendLine("Transform in Unity Coordinate System:");
                logBuilder.AppendLine($"  Position (m): {result.Position}");
                logBuilder.AppendLine($"  Rotation (deg): {result.Rotation.eulerAngles}");
                logBuilder.AppendLine();
                logBuilder.AppendLine("4x4 Transform Matrix:");
                logBuilder.AppendLine(MatrixToString(result.TransformMatrix));

                result.ResultText = logBuilder.ToString();
                lastResult = result;

                UpdateStatus($"标定成功!使用了 {robotPoses.Count} 组数据");
                OnCalibrationCompleted?.Invoke(result);
            }
            catch (Exception ex)
            {
                result.IsSuccess = false;
                result.ErrorMessage = ex.Message;
                result.ResultText = logBuilder.ToString() + $"\n\nException: {ex.Message}\n{ex.StackTrace}";
                UpdateStatus($"标定异常:{ex.Message}");
                Debug.LogError($"[标定管理器] 标定异常: {ex.Message}\n{ex.StackTrace}");
            }

            return result;
        }

        #endregion

        #region 坐标变换

        /// <summary>
        /// 使用标定结果进行坐标变换（相机坐标→机器人基座坐标）
        /// </summary>
        public Vector3 TransformPointCameraToBase(Vector3 pointInCamera)
        {
            if (lastResult == null || !lastResult.IsSuccess)
            {
                Debug.LogWarning("[标定管理器] 尚未完成标定,无法进行坐标变换");
                return Vector3.zero;
            }

            return lastResult.TransformMatrix.MultiplyPoint3x4(pointInCamera);
        }

        /// <summary>
        /// 批量坐标变换
        /// </summary>
        public Vector3[] TransformPointsCameraToBase(Vector3[] pointsInCamera)
        {
            Vector3[] transformedPoints = new Vector3[pointsInCamera.Length];
            for (int i = 0; i < pointsInCamera.Length; i++)
            {
                transformedPoints[i] = TransformPointCameraToBase(pointsInCamera[i]);
            }
            return transformedPoints;
        }

        #endregion

        #region 辅助方法

        private void UpdateStatus(string message)
        {
            string formattedMsg = $"[{DateTime.Now:HH:mm:ss}] {message}";
            Debug.Log($"[标定管理器] {message}");
            OnStatusUpdated?.Invoke(formattedMsg);
        }

        private double[] ParseDoubleArray(string line, int expectedCount)
        {
            try
            {
                var parts = line.Split(new[] { ' ', '\t', ',' }, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length != expectedCount)
                    return null;

                double[] values = new double[expectedCount];
                for (int i = 0; i < expectedCount; i++)
                {
                    values[i] = double.Parse(parts[i]);
                }
                return values;
            }
            catch
            {
                return null;
            }
        }

        private string MatrixToString(Matrix4x4 matrix)
        {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < 4; i++)
            {
                sb.Append("  [");
                for (int j = 0; j < 4; j++)
                {
                    sb.Append($"{matrix[i, j]:F6}");
                    if (j < 3) sb.Append(", ");
                }
                sb.AppendLine("]");
            }
            return sb.ToString();
        }

        #endregion

        #region 示例数据加载

        /// <summary>
        /// 加载示例数据（与WPF项目相同的23组数据）
        /// </summary>
        public void LoadSampleData()
        {
            ClearCalibrationData();

            string sampleRobotData = @"-0.73 -195.12 694.16 -0.0048 -2.2207 2.2175
-484.30 -24.23 379.82 -0.2826 0.3906 -1.7357
172.31 476.92 309.08 -1.2908 0.0939 -0.1228
492.97 -139.06 383.11 -1.0147 1.0941 -1.3658
-77.73 -388.16 373.88 0.0100 0.4014 -3.0293
369.80 -122.78 558.52 -0.7425 0.7920 -1.4699
68.39 357.99 582.73 -0.9592 -0.0417 0.1033
441.07 -80.73 494.38 0.5365 0.3739 1.0960
81.26 107.16 388.05 -0.0061 0.0363 1.4637
43.60 111.29 564.01 0.0309 0.0666 1.4648
-129.59 455.10 297.79 -2.7623 -0.7320 0.0640
428.44 206.59 353.13 -2.4653 1.1676 -0.3048
313.34 -413.11 297.24 -1.1224 2.4419 -0.9746
-416.49 -209.73 399.00 -0.7172 -1.5094 1.8190
-182.40 399.66 318.75 -2.5694 -0.9180 0.1611
362.56 225.02 403.58 -1.6788 0.6756 -0.5618
151.06 -103.27 492.29 -0.8552 0.8272 -1.3422
363.91 159.28 421.49 -0.6647 0.2848 -0.8368
335.70 266.32 447.32 -1.0819 0.4038 -0.5925
100.36 -87.29 656.74 -0.0420 0.0682 -0.2428
398.26 291.95 280.56 1.1359 -0.3419 2.3478
-254.34 -315.34 509.33 -1.0886 -0.9040 -0.0821
-163.79 239.60 382.04 -0.6424 -0.5328 0.9553";

            string sampleCameraData = @"341.73 -361.63 -875.93 0.7025 -0.0044 -0.7116 -0.0018
152.22 -662.37 -388.77 0.8396 0.5328 -0.0950 0.0471
-332.64 -745.92 -1060.41 0.7564 0.0912 0.6386 -0.1080
296.46 -687.91 -1362.44 0.9938 0.0998 -0.0477 -0.0081
531.14 -682.24 -782.28 0.5834 0.4461 -0.5431 0.4071
277.10 -506.26 -1244.69 0.9598 0.2787 -0.0321 -0.0084
-216.17 -469.29 -957.33 0.6459 0.1865 0.6992 -0.2433
236.46 -569.48 -1313.03 -0.1012 -0.1938 -0.4385 0.8717
36.46 -667.04 -952.59 -0.0484 -0.0379 -0.6896 0.7215
31.78 -489.72 -916.76 -0.0443 -0.0381 -0.6738 0.7366
-315.78 -752.21 -757.01 0.4224 -0.3134 0.6833 0.5063
-54.49 -712.38 -1307.68 0.8051 -0.5038 0.2638 0.1686
566.80 -777.94 -1170.02 0.8546 -0.3735 -0.3283 -0.1498
343.18 -646.25 -456.48 -0.3282 -0.0329 0.9397 -0.0902
-263.82 -732.94 -697.84 0.3744 -0.2412 0.7586 0.4756
-74.82 -657.99 -1245.12 0.9192 -0.1065 0.3782 0.0255
249.74 -565.00 -1025.99 0.9745 0.2224 0.0190 -0.0217
-10.35 -642.01 -1240.86 0.8580 0.3853 0.3052 -0.1492
-116.93 -612.36 -1220.24 0.8869 0.1768 0.4109 -0.1155
232.48 -398.99 -972.75 0.5882 0.5263 0.4210 -0.4469
-139.00 -780.51 -1279.12 0.5830 0.0138 -0.4092 0.7017
450.04 -543.26 -608.78 0.4529 0.3460 0.8165 0.0918
-103.84 -667.32 -710.59 0.2133 0.1503 0.8944 -0.3632";

            AddCalibrationDataFromText(sampleRobotData, sampleCameraData);
            UpdateStatus($"已加载示例数据(23组标定点)");
        }

        #endregion
    }
}

using UnityEngine;
using System;
using System.Collections.Generic;
using System.Text;
using HandEyeCalibration;
using handeye;

/// <summary>
/// 手眼标定精度评估器
/// 
/// 功能：
/// - 读取HandEyeCalibrationUI中的手眼标定数据
/// - 使用SteamVrUrCoordinateConverter对Tracker位姿进行坐标变换
/// - 计算预测TCP与实际采集TCP的位置误差和旋转误差
/// - 输出平均误差作为标定精度评估
/// 
/// 使用方法：
/// 1. 将此脚本挂载到场景中的任意GameObject
/// 2. 在Inspector中关联HandEyeCalibrationUI组件
/// 3. 先在HandEyeCalibrationUI中输入标定数据并执行标定
/// 4. 右键点击此脚本组件，选择"计算标定精度"
/// </summary>
public class HandEyeCalibrationAccuracyEvaluator : MonoBehaviour
{
    [Header("组件引用")]
    [Tooltip("手眼标定UI组件，用于读取输入的标定数据")]
    [SerializeField] private HandEyeCalibrationUI calibrationUI;
    
    [Tooltip("手眼标定管理器，用于获取标定矩阵")]
    [SerializeField] private HandEyeCalibrationManager calibrationManager;

    [Header("评估结果（只读）")]
    [SerializeField] private float averagePositionError_mm;
    [SerializeField] private float averageRotationError_deg;
    [SerializeField] private int evaluatedDataCount;
    [SerializeField] private string lastEvaluationTime;

    [Header("详细误差数据")]
    [SerializeField] private List<float> positionErrors_mm = new List<float>();
    [SerializeField] private List<float> rotationErrors_deg = new List<float>();

    /// <summary>
    /// 通过Inspector右键菜单触发标定精度计算
    /// </summary>
    [ContextMenu("计算标定精度")]
    public void EvaluateCalibrationAccuracy()
    {
        Debug.Log("========== 手眼标定精度评估开始 ==========");

        // 1. 验证组件引用
        if (!ValidateComponents())
        {
            return;
        }

        // 2. 获取UI中的输入数据
        string robotDataText;
        string cameraDataText;
        if (!GetInputData(out robotDataText, out cameraDataText))
        {
            return;
        }

        // 3. 解析数据
        List<RobotPoseData> robotPoses;
        List<CameraPoseData> cameraPoses;
        if (!ParseCalibrationData(robotDataText, cameraDataText, out robotPoses, out cameraPoses))
        {
            return;
        }

        // 4. 确保标定矩阵已计算（调用标定）
        if (!EnsureCalibrationDone())
        {
            return;
        }

        // 5. 计算每帧误差
        CalculateErrors(robotPoses, cameraPoses);

        // 6. 输出结果
        OutputResults();
    }

    /// <summary>
    /// 验证组件引用是否有效
    /// </summary>
    private bool ValidateComponents()
    {
        if (calibrationUI == null)
        {
            calibrationUI = FindObjectOfType<HandEyeCalibrationUI>();
            if (calibrationUI == null)
            {
                Debug.LogError("[标定精度评估] 未找到HandEyeCalibrationUI组件，请在Inspector中手动关联");
                return false;
            }
        }

        if (calibrationManager == null)
        {
            calibrationManager = FindObjectOfType<HandEyeCalibrationManager>();
            if (calibrationManager == null)
            {
                Debug.LogError("[标定精度评估] 未找到HandEyeCalibrationManager组件，请在Inspector中手动关联");
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// 从UI获取输入数据
    /// </summary>
    private bool GetInputData(out string robotDataText, out string cameraDataText)
    {
        robotDataText = "";
        cameraDataText = "";

        // 通过反射获取UI中的输入框内容
        var uiType = calibrationUI.GetType();
        
        var robotInputField = uiType.GetField("robotDataInput", 
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        var cameraInputField = uiType.GetField("cameraDataInput", 
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);

        if (robotInputField == null || cameraInputField == null)
        {
            Debug.LogError("[标定精度评估] 无法访问UI输入框字段");
            return false;
        }

        var robotInput = robotInputField.GetValue(calibrationUI) as TMPro.TMP_InputField;
        var cameraInput = cameraInputField.GetValue(calibrationUI) as TMPro.TMP_InputField;

        if (robotInput == null || cameraInput == null)
        {
            Debug.LogError("[标定精度评估] UI输入框为空");
            return false;
        }

        robotDataText = robotInput.text;
        cameraDataText = cameraInput.text;

        if (string.IsNullOrWhiteSpace(robotDataText) || string.IsNullOrWhiteSpace(cameraDataText))
        {
            Debug.LogError("[标定精度评估] 输入数据为空，请先在UI中输入标定数据");
            return false;
        }

        return true;
    }

    /// <summary>
    /// 解析标定数据文本
    /// </summary>
    private bool ParseCalibrationData(string robotDataText, string cameraDataText,
        out List<RobotPoseData> robotPoses, out List<CameraPoseData> cameraPoses)
    {
        robotPoses = new List<RobotPoseData>();
        cameraPoses = new List<CameraPoseData>();

        var robotLines = robotDataText.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
        var cameraLines = cameraDataText.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

        if (robotLines.Length != cameraLines.Length)
        {
            Debug.LogError($"[标定精度评估] 数据行数不匹配: 机器人{robotLines.Length}行, 相机{cameraLines.Length}行");
            return false;
        }

        if (robotLines.Length < 1)
        {
            Debug.LogError("[标定精度评估] 无有效数据行");
            return false;
        }

        for (int i = 0; i < robotLines.Length; i++)
        {
            // 解析机器人数据: X Y Z RX RY RZ (mm, mm, mm, rad, rad, rad)
            var robotValues = ParseDoubleArray(robotLines[i], 6);
            if (robotValues == null)
            {
                Debug.LogWarning($"[标定精度评估] 机器人数据第{i + 1}行格式错误，已跳过");
                continue;
            }

            // 解析相机数据: X Y Z QX QY QZ QW (mm, mm, mm, qx, qy, qz, qw)
            var cameraValues = ParseDoubleArray(cameraLines[i], 7);
            if (cameraValues == null)
            {
                Debug.LogWarning($"[标定精度评估] 相机数据第{i + 1}行格式错误，已跳过");
                continue;
            }

            robotPoses.Add(new RobotPoseData(
                robotValues[0], robotValues[1], robotValues[2],
                robotValues[3], robotValues[4], robotValues[5]));

            cameraPoses.Add(new CameraPoseData(
                cameraValues[0], cameraValues[1], cameraValues[2],
                cameraValues[3], cameraValues[4], cameraValues[5],
                cameraValues[6]));
        }

        Debug.Log($"[标定精度评估] 成功解析 {robotPoses.Count} 组数据");
        return robotPoses.Count > 0;
    }

    /// <summary>
    /// 解析双精度数组
    /// </summary>
    private double[] ParseDoubleArray(string line, int expectedCount)
    {
        var parts = line.Split(new[] { ' ', '\t', ',' }, StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length < expectedCount)
            return null;

        var result = new double[expectedCount];
        for (int i = 0; i < expectedCount; i++)
        {
            if (!double.TryParse(parts[i], System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out result[i]))
            {
                return null;
            }
        }
        return result;
    }

    /// <summary>
    /// 确保标定已完成
    /// </summary>
    private bool EnsureCalibrationDone()
    {
        // 检查是否已有标定结果
        var resultField = calibrationManager.GetType().GetField("lastResult",
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        
        if (resultField != null)
        {
            var result = resultField.GetValue(calibrationManager) as CalibrationResult;
            if (result != null && result.IsSuccess)
            {
                Debug.Log("[标定精度评估] 使用已有的标定结果");
                return true;
            }
        }

        // 如果没有结果，尝试执行标定
        Debug.Log("[标定精度评估] 正在执行标定计算...");
        var calibResult = calibrationManager.PerformCalibration();
        
        if (!calibResult.IsSuccess)
        {
            Debug.LogError($"[标定精度评估] 标定失败: {calibResult.ErrorMessage}");
            return false;
        }

        Debug.Log("[标定精度评估] 标定计算成功");
        return true;
    }

    /// <summary>
    /// 计算每帧的位姿误差
    /// </summary>
    private void CalculateErrors(List<RobotPoseData> robotPoses, List<CameraPoseData> cameraPoses)
    {
        positionErrors_mm.Clear();
        rotationErrors_deg.Clear();

        for (int i = 0; i < robotPoses.Count; i++)
        {
            // 获取相机/Tracker位姿数据（毫米转米）
            Vector3 trackerPos_m = new Vector3(
                (float)cameraPoses[i].X / 1000f,
                (float)cameraPoses[i].Y / 1000f,
                (float)cameraPoses[i].Z / 1000f);

            Quaternion trackerRot = new Quaternion(
                (float)cameraPoses[i].QX,
                (float)cameraPoses[i].QY,
                (float)cameraPoses[i].QZ,
                (float)cameraPoses[i].QW);

            // 使用手眼变换将Tracker位姿转换为预测的TCP位姿
            Vector3 predictedPos_m;
            Vector3 predictedRotVec_rad;
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                trackerPos_m, trackerRot, false,
                out predictedPos_m, out predictedRotVec_rad);

            // 预测位置转换为毫米
            Vector3 predictedPos_mm = predictedPos_m * 1000f;

            // 获取实际采集的TCP位姿
            Vector3 actualPos_mm = new Vector3(
                (float)robotPoses[i].X,
                (float)robotPoses[i].Y,
                (float)robotPoses[i].Z);

            Vector3 actualRotVec_rad = new Vector3(
                (float)robotPoses[i].RX,
                (float)robotPoses[i].RY,
                (float)robotPoses[i].RZ);

            // 计算位置误差（欧氏距离，毫米）
            float posError = Vector3.Distance(predictedPos_mm, actualPos_mm);
            positionErrors_mm.Add(posError);

            // 计算旋转误差（角度差，度）
            float rotError = CalculateRotationError(predictedRotVec_rad, actualRotVec_rad);
            rotationErrors_deg.Add(rotError);

            // 输出每帧详细信息
            Debug.Log($"[第{i + 1}帧] 位置误差: {posError:F2}mm, 旋转误差: {rotError:F2}°");
            Debug.Log($"  预测TCP(mm): ({predictedPos_mm.x:F2}, {predictedPos_mm.y:F2}, {predictedPos_mm.z:F2})");
            Debug.Log($"  实际TCP(mm): ({actualPos_mm.x:F2}, {actualPos_mm.y:F2}, {actualPos_mm.z:F2})");
        }

        evaluatedDataCount = robotPoses.Count;
    }

    /// <summary>
    /// 计算两个旋转向量之间的角度误差（度）
    /// </summary>
    private float CalculateRotationError(Vector3 rotVec1_rad, Vector3 rotVec2_rad)
    {
        // 将旋转向量转换为四元数
        Quaternion q1 = RotationVectorToQuaternion(rotVec1_rad);
        Quaternion q2 = RotationVectorToQuaternion(rotVec2_rad);

        // 计算相对旋转
        Quaternion deltaQ = Quaternion.Inverse(q1) * q2;

        // 计算旋转角度
        // deltaQ.w = cos(theta/2), 所以 theta = 2 * acos(|w|)
        float angle_rad = 2f * Mathf.Acos(Mathf.Clamp(Mathf.Abs(deltaQ.w), 0f, 1f));
        
        return angle_rad * Mathf.Rad2Deg;
    }

    /// <summary>
    /// 旋转向量（轴角表示）转四元数
    /// </summary>
    private Quaternion RotationVectorToQuaternion(Vector3 rotVec_rad)
    {
        float theta = rotVec_rad.magnitude;
        if (theta < 1e-8f)
            return Quaternion.identity;

        Vector3 axis = rotVec_rad / theta;
        float halfAngle = theta * 0.5f;
        float s = Mathf.Sin(halfAngle);

        return new Quaternion(
            axis.x * s,
            axis.y * s,
            axis.z * s,
            Mathf.Cos(halfAngle));
    }

    /// <summary>
    /// 输出评估结果
    /// </summary>
    private void OutputResults()
    {
        if (positionErrors_mm.Count == 0)
        {
            Debug.LogError("[标定精度评估] 无有效误差数据");
            return;
        }

        // 计算平均误差
        float sumPosError = 0f;
        float sumRotError = 0f;
        float maxPosError = 0f;
        float maxRotError = 0f;
        float minPosError = float.MaxValue;
        float minRotError = float.MaxValue;

        for (int i = 0; i < positionErrors_mm.Count; i++)
        {
            sumPosError += positionErrors_mm[i];
            sumRotError += rotationErrors_deg[i];

            maxPosError = Mathf.Max(maxPosError, positionErrors_mm[i]);
            maxRotError = Mathf.Max(maxRotError, rotationErrors_deg[i]);
            minPosError = Mathf.Min(minPosError, positionErrors_mm[i]);
            minRotError = Mathf.Min(minRotError, rotationErrors_deg[i]);
        }

        averagePositionError_mm = sumPosError / positionErrors_mm.Count;
        averageRotationError_deg = sumRotError / rotationErrors_deg.Count;
        lastEvaluationTime = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");

        // 构建结果报告
        StringBuilder report = new StringBuilder();
        report.AppendLine("========== 手眼标定精度评估结果 ==========");
        report.AppendLine($"评估时间: {lastEvaluationTime}");
        report.AppendLine($"评估数据帧数: {evaluatedDataCount}");
        report.AppendLine();
        report.AppendLine("【平均误差】");
        report.AppendLine($"  位置误差: {averagePositionError_mm:F3} mm");
        report.AppendLine($"  旋转误差: {averageRotationError_deg:F3} °");
        report.AppendLine();
        report.AppendLine("【误差范围】");
        report.AppendLine($"  位置误差: {minPosError:F3} ~ {maxPosError:F3} mm");
        report.AppendLine($"  旋转误差: {minRotError:F3} ~ {maxRotError:F3} °");
        report.AppendLine("===========================================");

        Debug.Log(report.ToString());
    }

#if UNITY_EDITOR
    /// <summary>
    /// 编辑器中自动查找组件
    /// </summary>
    private void OnValidate()
    {
        if (calibrationUI == null)
        {
            calibrationUI = FindObjectOfType<HandEyeCalibrationUI>();
        }
        if (calibrationManager == null)
        {
            calibrationManager = FindObjectOfType<HandEyeCalibrationManager>();
        }
    }
#endif
}

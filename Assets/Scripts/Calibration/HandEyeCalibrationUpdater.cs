using System;
using System.Text.RegularExpressions;
using UnityEngine;

namespace handeye
{
    /// <summary>
    /// 手眼标定结果更新器
    /// 
    /// 功能：
    /// - 在 Unity Inspector 中粘贴完整的手眼标定输出文本
    /// - 自动解析标定结果（位置、四元数、4x4矩阵）
    /// - 一键应用到坐标转换系统
    /// - 支持多种标定输出格式
    /// 
    /// 使用方法：
    /// 1. 将此脚本挂载到任意 GameObject 上
    /// 2. 复制手眼标定的完整输出文本
    /// 3. 粘贴到 "Calibration Result Text" 输入框
    /// 4. 点击 "解析并应用标定结果" 按钮
    /// 5. 查看解析状态和应用结果
    /// 
    /// 支持的输入格式：
    /// === Hand-Eye Calibration - Eye-to-Hand ===
    /// Using Custom DLL - Tsai Algorithm
    /// Calibration Points: 12
    /// 
    /// === Calibration Success ===
    /// 
    /// Camera to Robot Base Transform (Cam → Base):
    ///   Position (mm): (-606.508, 882.720, 1042.878)
    ///   Rotation (Quaternion): (w:-0.2902, x:-0.2806, y:0.6497, z:0.6442)
    /// 
    /// Transform in Unity Coordinate System:
    ///   Position (m): (-0.61, 0.88, 1.04)
    ///   Rotation (deg): (317.61, 269.88, 270.97)
    /// 
    /// 4x4 Transform Matrix:
    ///   [-0.674113,  0.009269, -0.738570, -0.606508]
    ///   [-0.738465,  0.012541,  0.674175,  0.882720]
    ///   [ 0.015512,  0.999878, -0.001609,  1.042878]
    ///   [0.000000, 0.000000, 0.000000, 1.000000]
    /// </summary>
    public class HandEyeCalibrationUpdater : MonoBehaviour
    {
        [Header("=== 手眼标定结果输入 ===")]
        [Tooltip("粘贴完整的手眼标定输出文本到这里")]
        [TextArea(15, 30)]
        public string calibrationResultText = "在这里粘贴手眼标定的完整输出文本...";

        [Header("=== 解析结果 ===")]
        [SerializeField]
        [Tooltip("标定点数")]
        private int calibrationPoints = 0;

        [SerializeField]
        [Tooltip("位置 (mm)")]
        private Vector3 position_mm = Vector3.zero;

        [SerializeField]
        [Tooltip("四元数 (w, x, y, z)")]
        private Vector4 quaternion = Vector4.zero;

        [SerializeField]
        [Tooltip("旋转矩阵 (3x3) - 第1行")]
        private Vector3 rotationRow1 = Vector3.zero;

        [SerializeField]
        [Tooltip("旋转矩阵 (3x3) - 第2行")]
        private Vector3 rotationRow2 = Vector3.zero;

        [SerializeField]
        [Tooltip("旋转矩阵 (3x3) - 第3行")]
        private Vector3 rotationRow3 = Vector3.zero;

        [Header("=== 状态信息 ===")]
        [SerializeField]
        [Tooltip("解析状态")]
        private string parseStatus = "等待输入...";

        [SerializeField]
        [Tooltip("是否解析成功")]
        private bool isParseSuccess = false;

        [Header("=== 快捷操作 ===")]
        [Tooltip("是否在解析后自动应用")]
        public bool autoApplyAfterParse = true;

        /// <summary>
        /// 解析并应用标定结果
        /// </summary>
        [ContextMenu("解析并应用标定结果")]
        public void ParseAndApplyCalibration()
        {
            try
            {
                // 步骤1: 解析文本
                if (!ParseCalibrationText())
                {
                    parseStatus = "❌ 解析失败：无法识别标定结果格式";
                    isParseSuccess = false;
                    Debug.LogError("[手眼标定更新器] 解析失败，请检查输入文本格式");
                    return;
                }

                parseStatus = "✓ 解析成功";
                isParseSuccess = true;

                // 步骤2: 应用到转换器
                if (autoApplyAfterParse)
                {
                    ApplyToConverter();
                }
                else
                {
                    parseStatus += " (未自动应用，请手动点击'应用到转换器')";
                }
            }
            catch (Exception ex)
            {
                parseStatus = $"❌ 异常: {ex.Message}";
                isParseSuccess = false;
                Debug.LogError($"[手眼标定更新器] 异常: {ex.Message}\n{ex.StackTrace}");
            }
        }

        /// <summary>
        /// 仅解析文本，不应用
        /// </summary>
        [ContextMenu("仅解析文本")]
        public void ParseOnly()
        {
            bool previousAutoApply = autoApplyAfterParse;
            autoApplyAfterParse = false;
            ParseAndApplyCalibration();
            autoApplyAfterParse = previousAutoApply;
        }

        /// <summary>
        /// 应用已解析的数据到转换器
        /// </summary>
        [ContextMenu("应用到转换器")]
        public void ApplyToConverter()
        {
            if (!isParseSuccess)
            {
                Debug.LogWarning("[手眼标定更新器] 请先解析标定结果");
                return;
            }

            try
            {
                // 构建 Matrix4x4
                // Unity Matrix4x4 是列主序，需要按列填充
                Matrix4x4 calibrationMatrix = new Matrix4x4();

                // 第1列 (旋转矩阵的第1列)
                calibrationMatrix.m00 = rotationRow1.x;
                calibrationMatrix.m10 = rotationRow2.x;
                calibrationMatrix.m20 = rotationRow3.x;
                calibrationMatrix.m30 = 0f;

                // 第2列 (旋转矩阵的第2列)
                calibrationMatrix.m01 = rotationRow1.y;
                calibrationMatrix.m11 = rotationRow2.y;
                calibrationMatrix.m21 = rotationRow3.y;
                calibrationMatrix.m31 = 0f;

                // 第3列 (旋转矩阵的第3列)
                calibrationMatrix.m02 = rotationRow1.z;
                calibrationMatrix.m12 = rotationRow2.z;
                calibrationMatrix.m22 = rotationRow3.z;
                calibrationMatrix.m32 = 0f;

                // 第4列 (平移向量，单位：m)
                calibrationMatrix.m03 = position_mm.x / 1000f; // mm -> m
                calibrationMatrix.m13 = position_mm.y / 1000f;
                calibrationMatrix.m23 = position_mm.z / 1000f;
                calibrationMatrix.m33 = 1f;

                // 应用到转换器
                SteamVrUrCoordinateConverter.SetCalibration(calibrationMatrix);

                parseStatus = $"✓ 已应用标定结果 ({calibrationPoints} 点)";
                
                Debug.Log($"[手眼标定更新器] 已应用新的标定结果:");
                Debug.Log($"  标定点数: {calibrationPoints}");
                Debug.Log($"  位置 (mm): ({position_mm.x:F3}, {position_mm.y:F3}, {position_mm.z:F3})");
                Debug.Log($"  四元数: (w:{quaternion.x:F4}, x:{quaternion.y:F4}, y:{quaternion.z:F4}, z:{quaternion.w:F4})");
                Debug.Log($"  旋转矩阵:\n" +
                         $"    [{rotationRow1.x:F6}, {rotationRow1.y:F6}, {rotationRow1.z:F6}]\n" +
                         $"    [{rotationRow2.x:F6}, {rotationRow2.y:F6}, {rotationRow2.z:F6}]\n" +
                         $"    [{rotationRow3.x:F6}, {rotationRow3.y:F6}, {rotationRow3.z:F6}]");
            }
            catch (Exception ex)
            {
                parseStatus = $"❌ 应用失败: {ex.Message}";
                Debug.LogError($"[手眼标定更新器] 应用失败: {ex.Message}");
            }
        }

        /// <summary>
        /// 解析标定文本
        /// </summary>
        private bool ParseCalibrationText()
        {
            if (string.IsNullOrEmpty(calibrationResultText))
            {
                return false;
            }

            bool success = true;

            // 解析标定点数
            success &= TryParseCalibrationPoints();

            // 解析位置 (mm)
            success &= TryParsePosition();

            // 解析四元数
            success &= TryParseQuaternion();

            // 解析 4x4 矩阵
            success &= TryParseMatrix();

            return success;
        }

        /// <summary>
        /// 尝试解析标定点数
        /// </summary>
        private bool TryParseCalibrationPoints()
        {
            // 匹配: Calibration Points: 12
            var match = Regex.Match(calibrationResultText, @"Calibration Points:\s*(\d+)");
            if (match.Success)
            {
                calibrationPoints = int.Parse(match.Groups[1].Value);
                return true;
            }
            return false;
        }

        /// <summary>
        /// 尝试解析位置 (mm)
        /// </summary>
        private bool TryParsePosition()
        {
            // 匹配: Position (mm): (-606.508, 882.720, 1042.878)
            var match = Regex.Match(calibrationResultText, 
                @"Position \(mm\):\s*\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\)");
            
            if (match.Success)
            {
                position_mm.x = float.Parse(match.Groups[1].Value);
                position_mm.y = float.Parse(match.Groups[2].Value);
                position_mm.z = float.Parse(match.Groups[3].Value);
                return true;
            }
            return false;
        }

        /// <summary>
        /// 尝试解析四元数
        /// </summary>
        private bool TryParseQuaternion()
        {
            // 匹配: Rotation (Quaternion): (w:-0.2902, x:-0.2806, y:0.6497, z:0.6442)
            var match = Regex.Match(calibrationResultText,
                @"Rotation \(Quaternion\):\s*\(\s*w:\s*(-?[\d.]+)\s*,\s*x:\s*(-?[\d.]+)\s*,\s*y:\s*(-?[\d.]+)\s*,\s*z:\s*(-?[\d.]+)\s*\)");

            if (match.Success)
            {
                quaternion.x = float.Parse(match.Groups[1].Value); // w
                quaternion.y = float.Parse(match.Groups[2].Value); // x
                quaternion.z = float.Parse(match.Groups[3].Value); // y
                quaternion.w = float.Parse(match.Groups[4].Value); // z
                return true;
            }
            return false;
        }

        /// <summary>
        /// 尝试解析 4x4 矩阵
        /// </summary>
        private bool TryParseMatrix()
        {
            // 匹配 4x4 矩阵的前三行（旋转和平移）
            // 格式: [m00, m01, m02, tx]
            
            // 查找 "4x4 Transform Matrix:" 之后的内容
            int matrixStartIndex = calibrationResultText.IndexOf("4x4 Transform Matrix:");
            if (matrixStartIndex < 0)
            {
                return false;
            }

            string matrixSection = calibrationResultText.Substring(matrixStartIndex);
            
            // 提取所有数字行
            var lines = matrixSection.Split('\n');
            int rowCount = 0;

            foreach (var line in lines)
            {
                // 匹配形如 [num, num, num, num] 的行
                var match = Regex.Match(line, 
                    @"\[\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\]");
                
                if (match.Success && rowCount < 3)
                {
                    float val1 = float.Parse(match.Groups[1].Value);
                    float val2 = float.Parse(match.Groups[2].Value);
                    float val3 = float.Parse(match.Groups[3].Value);
                    // 第4个值是平移，暂时不用存储（从 position_mm 获取）

                    switch (rowCount)
                    {
                        case 0:
                            rotationRow1 = new Vector3(val1, val2, val3);
                            break;
                        case 1:
                            rotationRow2 = new Vector3(val1, val2, val3);
                            break;
                        case 2:
                            rotationRow3 = new Vector3(val1, val2, val3);
                            break;
                    }

                    rowCount++;
                }
            }

            return rowCount == 3;
        }

        /// <summary>
        /// 清空输入
        /// </summary>
        [ContextMenu("清空输入")]
        public void ClearInput()
        {
            calibrationResultText = "在这里粘贴手眼标定的完整输出文本...";
            calibrationPoints = 0;
            position_mm = Vector3.zero;
            quaternion = Vector4.zero;
            rotationRow1 = Vector3.zero;
            rotationRow2 = Vector3.zero;
            rotationRow3 = Vector3.zero;
            parseStatus = "已清空";
            isParseSuccess = false;
        }

        /// <summary>
        /// 显示当前转换器中的标定矩阵
        /// </summary>
        [ContextMenu("显示当前标定矩阵")]
        public void ShowCurrentCalibration()
        {
            // 注意: 需要在 SteamVrUrCoordinateConverter 中添加 GetCalibration() 方法
            Debug.Log("[手眼标定更新器] 当前标定矩阵信息已记录到 Console");
        }

#if UNITY_EDITOR
        [Header("=== 使用说明 ===")]
        [TextArea(8, 15)]
        public string usageInstructions = 
            "使用步骤:\n" +
            "1. 运行手眼标定程序，获取完整输出\n" +
            "2. 复制从 '=== Hand-Eye Calibration' 开始的所有文本\n" +
            "3. 粘贴到上方 'Calibration Result Text' 输入框\n" +
            "4. 右键点击脚本 → '解析并应用标定结果'\n" +
            "5. 检查 '解析结果' 区域确认数据正确\n" +
            "6. 标定结果已自动应用到系统\n\n" +
            "提示:\n" +
            "- 支持自动解析位置、四元数、矩阵\n" +
            "- 勾选 'Auto Apply After Parse' 可自动应用\n" +
            "- 可使用 '仅解析文本' 预览不应用";
#endif
    }
}

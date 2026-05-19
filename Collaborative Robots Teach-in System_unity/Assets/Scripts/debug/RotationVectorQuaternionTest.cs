using UnityEngine;

namespace handeye
{
    /// <summary>
    /// 旋转矢量↔四元数 转换问题诊断
    /// 
    /// 检查问题：
    /// 1. RotationVector → Quaternion 转换是否正确
    /// 2. Quaternion → RotationVector 转换是否正确
    /// 3. 往返转换是否有损失
    /// 4. 在大角度下的表现
    /// </summary>
    public class RotationVectorQuaternionTest : MonoBehaviour
    {
        [ContextMenu("诊断1: 基础转换正确性")]
        public void TestBasicConversion()
        {
            Debug.Log("========== 旋转矢量↔四元数 基础转换测试 ==========\n");

            // 测试1: 简单的X轴旋转
            Debug.Log("=== 测试1: X轴旋转10° ===");
            float angle1 = 10f * Mathf.Deg2Rad;
            Vector3 rv1 = new Vector3(angle1, 0, 0);  // 轴角表示
            TestRoundTrip("X轴10°", rv1);

            // 测试2: Y轴旋转90°
            Debug.Log("\n=== 测试2: Y轴旋转90° ===");
            float angle2 = 90f * Mathf.Deg2Rad;
            Vector3 rv2 = new Vector3(0, angle2, 0);
            TestRoundTrip("Y轴90°", rv2);

            // 测试3: Z轴旋转180°
            Debug.Log("\n=== 测试3: Z轴旋转180° (边界情况) ===");
            float angle3 = 180f * Mathf.Deg2Rad;
            Vector3 rv3 = new Vector3(0, 0, angle3);
            TestRoundTrip("Z轴180°", rv3);

            // 测试4: 任意轴大角度
            Debug.Log("\n=== 测试4: 任意轴212° (模拟手眼标定角度) ===");
            float angle4 = 212f * Mathf.Deg2Rad;
            Vector3 axis4 = new Vector3(-0.31f, 0.69f, -0.65f).normalized;  // 近似手眼标定轴
            Vector3 rv4 = axis4 * angle4;
            TestRoundTrip("任意轴212°", rv4);

            Debug.Log("\n========== 测试完成 ==========");
        }

        [ContextMenu("诊断2: 检查具体转换函数")]
        public void TestConversionFunctions()
        {
            Debug.Log("========== 检查转换函数实现 ==========\n");

            // 测试输入
            Vector3 rv_input = new Vector3(0.1745f, 0, 0);  // X轴10° (弧度)
            Debug.Log($"输入旋转矢量: ({rv_input.x:F4}, {rv_input.y:F4}, {rv_input.z:F4})");
            Debug.Log($"角度: {rv_input.magnitude * Mathf.Rad2Deg:F2}°");
            Debug.Log($"轴: ({(rv_input.normalized).x:F4}, {(rv_input.normalized).y:F4}, {(rv_input.normalized).z:F4})\n");

            // 步骤1: 旋转矢量 → 四元数
            Debug.Log("--- 步骤1: 旋转矢量 → 四元数 ---");
            Quaternion q = RotationVectorToQuaternion(rv_input);
            Debug.Log($"输出四元数: (w:{q.w:F4}, x:{q.x:F4}, y:{q.y:F4}, z:{q.z:F4})");
            Debug.Log($"四元数模长: {Mathf.Sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z):F6}");
            
            // 验证: 用Unity的方法对比
            Quaternion q_unity = Quaternion.AngleAxis(rv_input.magnitude * Mathf.Rad2Deg, rv_input.normalized);
            Debug.Log($"Unity四元数: (w:{q_unity.w:F4}, x:{q_unity.x:F4}, y:{q_unity.y:F4}, z:{q_unity.z:F4})");
            float quatError = Quaternion.Angle(q, q_unity);
            Debug.Log($"与Unity对比误差: {quatError:F6}°");
            
            if (quatError > 0.001f)
            {
                Debug.LogError("<color=red>✗ 旋转矢量→四元数转换有问题!</color>");
            }
            else
            {
                Debug.Log("<color=green>✓ 旋转矢量→四元数转换正确</color>");
            }

            // 步骤2: 四元数 → 旋转矢量
            Debug.Log("\n--- 步骤2: 四元数 → 旋转矢量 ---");
            Vector3 rv_output = QuaternionToRotationVector(q);
            Debug.Log($"输出旋转矢量: ({rv_output.x:F4}, {rv_output.y:F4}, {rv_output.z:F4})");
            Debug.Log($"角度: {rv_output.magnitude * Mathf.Rad2Deg:F2}°");
            Debug.Log($"轴: ({(rv_output.normalized).x:F4}, {(rv_output.normalized).y:F4}, {(rv_output.normalized).z:F4})");

            // 往返误差
            Debug.Log("\n--- 往返转换误差 ---");
            Vector3 error = rv_output - rv_input;
            Debug.Log($"旋转矢量误差: Δ({error.x:F6}, {error.y:F6}, {error.z:F6})");
            Debug.Log($"误差大小: {error.magnitude:F6} 弧度 = {error.magnitude * Mathf.Rad2Deg:F6}°");
            
            if (error.magnitude > 0.001f)
            {
                Debug.LogWarning("<color=yellow>⚠ 往返转换有精度损失</color>");
            }
            else
            {
                Debug.Log("<color=green>✓ 往返转换精度良好</color>");
            }

            Debug.Log("\n========== 检查完成 ==========");
        }

        [ContextMenu("诊断3: 模拟实际使用场景")]
        public void TestRealScenario()
        {
            Debug.Log("========== 模拟实际使用场景 ==========\n");
            Debug.Log("场景: 用户使用旋转矢量作为输入调用ConvertSteamVrPoseToUrBase\n");

            Vector3 pos = Vector3.zero;
            
            // Tracker的旋转: X轴旋转10°
            float angleDeg = 10f;
            float angleRad = angleDeg * Mathf.Deg2Rad;
            Vector3 rv_tracker = new Vector3(angleRad, 0, 0);

            Debug.Log($"=== 输入 ===");
            Debug.Log($"旋转矢量: ({rv_tracker.x:F4}, {rv_tracker.y:F4}, {rv_tracker.z:F4})");
            Debug.Log($"期望角度: {angleDeg}°");
            Debug.Log($"期望轴: (1, 0, 0)\n");

            // 方法1: 使用旋转矢量重载 (会经过 rv→quat→rv 转换)
            Debug.Log("=== 方法1: 使用旋转矢量重载 ===");
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                pos, rv_tracker, false,
                out Vector3 posOut1, out Vector3 rvOut1
            );
            Debug.Log($"输出旋转矢量: ({rvOut1.x:F4}, {rvOut1.y:F4}, {rvOut1.z:F4})");
            Debug.Log($"输出角度: {rvOut1.magnitude * Mathf.Rad2Deg:F2}°\n");

            // 方法2: 手动转换为四元数后调用 (避免重复转换)
            Debug.Log("=== 方法2: 手动转四元数后调用 ===");
            Quaternion q_tracker = RotationVectorToQuaternion(rv_tracker);
            Debug.Log($"中间四元数: (w:{q_tracker.w:F4}, x:{q_tracker.x:F4}, y:{q_tracker.y:F4}, z:{q_tracker.z:F4})");
            
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                pos, q_tracker, false,
                out Vector3 posOut2, out Vector3 rvOut2
            );
            Debug.Log($"输出旋转矢量: ({rvOut2.x:F4}, {rvOut2.y:F4}, {rvOut2.z:F4})");
            Debug.Log($"输出角度: {rvOut2.magnitude * Mathf.Rad2Deg:F2}°\n");

            // 对比两种方法
            Debug.Log("=== 对比 ===");
            Vector3 diff = rvOut1 - rvOut2;
            Debug.Log($"两种方法的输出差异: ({diff.x:F6}, {diff.y:F6}, {diff.z:F6})");
            Debug.Log($"差异大小: {diff.magnitude * Mathf.Rad2Deg:F6}°");
            
            if (diff.magnitude < 0.0001f)
            {
                Debug.Log("<color=green>✓ 两种方法结果一致</color>");
            }
            else
            {
                Debug.LogWarning("<color=yellow>⚠ 两种方法有差异，可能存在重复转换问题</color>");
            }

            Debug.Log("\n========== 测试完成 ==========");
        }

        [ContextMenu("诊断4: 检查180°附近的数值稳定性")]
        public void TestNumericalStability()
        {
            Debug.Log("========== 180°附近数值稳定性测试 ==========\n");

            float[] testAngles = { 170f, 175f, 179f, 180f, 181f, 185f, 190f };

            foreach (float angleDeg in testAngles)
            {
                float angleRad = angleDeg * Mathf.Deg2Rad;
                Vector3 rv = new Vector3(angleRad, 0, 0);  // X轴旋转

                Quaternion q = RotationVectorToQuaternion(rv);
                Vector3 rv_back = QuaternionToRotationVector(q);

                float errorAngle = Mathf.Abs(rv_back.magnitude - rv.magnitude) * Mathf.Rad2Deg;

                string status = errorAngle < 0.01f ? "✓" : "✗";
                Debug.Log($"{angleDeg:F0}°: 误差 {errorAngle:F4}° {status}");
            }

            Debug.Log("\n========== 测试完成 ==========");
        }

        // ========== 辅助方法 (复制自SteamVrUrCoordinateConverter) ==========

        private void TestRoundTrip(string label, Vector3 rvInput)
        {
            Debug.Log($"测试: {label}");
            Debug.Log($"输入旋转矢量: ({rvInput.x:F4}, {rvInput.y:F4}, {rvInput.z:F4})");
            Debug.Log($"  角度: {rvInput.magnitude * Mathf.Rad2Deg:F2}°");
            Debug.Log($"  轴: ({(rvInput.normalized).x:F4}, {(rvInput.normalized).y:F4}, {(rvInput.normalized).z:F4})");

            // rv → quat
            Quaternion q = RotationVectorToQuaternion(rvInput);
            Debug.Log($"转换为四元数: (w:{q.w:F4}, x:{q.x:F4}, y:{q.y:F4}, z:{q.z:F4})");

            // quat → rv
            Vector3 rvOutput = QuaternionToRotationVector(q);
            Debug.Log($"转换回旋转矢量: ({rvOutput.x:F4}, {rvOutput.y:F4}, {rvOutput.z:F4})");
            Debug.Log($"  角度: {rvOutput.magnitude * Mathf.Rad2Deg:F2}°");
            Debug.Log($"  轴: ({(rvOutput.normalized).x:F4}, {(rvOutput.normalized).y:F4}, {(rvOutput.normalized).z:F4})");

            // 误差分析
            Vector3 error = rvOutput - rvInput;
            float errorAngle = error.magnitude * Mathf.Rad2Deg;
            
            Debug.Log($"往返误差: {errorAngle:F6}°");
            
            if (errorAngle < 0.001f)
            {
                Debug.Log("<color=green>✓ 转换正确</color>");
            }
            else if (errorAngle < 0.1f)
            {
                Debug.LogWarning($"<color=yellow>⚠ 有轻微误差</color>");
            }
            else
            {
                Debug.LogError($"<color=red>✗ 转换有严重错误!</color>");
            }
        }

        // 从SteamVrUrCoordinateConverter复制的方法
        private Quaternion RotationVectorToQuaternion(Vector3 r)
        {
            float theta = r.magnitude;
            if (theta < 1e-8f) return Quaternion.identity;

            Vector3 axis = r / theta;
            float half = theta * 0.5f;
            float s = Mathf.Sin(half);
            return new Quaternion(axis.x * s, axis.y * s, axis.z * s, Mathf.Cos(half));
        }

        private Vector3 QuaternionToRotationVector(Quaternion q)
        {
            // 归一化
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (mag > 1e-12f)
            {
                float inv = 1f / mag;
                q.x *= inv; q.y *= inv; q.z *= inv; q.w *= inv;
            }

            // 限制 qw 在 [-1, 1] 范围内避免 arccos 出错
            float wClamped = Mathf.Clamp(q.w, -1f, 1f);

            // θ = 2 * arccos(qw)
            float angle = 2f * Mathf.Acos(wClamped);

            // s = sin(θ/2) = sqrt(1 - qw²)
            float s = Mathf.Sqrt(1f - wClamped * wClamped);

            // 如果角度接近 0，直接返回 (qx, qy, qz) * 2
            if (s < 1e-6f)
            {
                return new Vector3(q.x, q.y, q.z) * 2f;
            }

            // k = (qx, qy, qz) / sin(θ/2)
            Vector3 axis = new Vector3(q.x / s, q.y / s, q.z / s);

            // r = θ * k
            return axis * angle;
        }
    }
}

using UnityEngine;

namespace handeye
{
    /// <summary>
    /// 旋转格式转换奇异性诊断工具
    /// 
    /// 专门检测：
    /// 1. 不同输入角度下的误差分布
    /// 2. 奇异点位置（0°, 180°, 360°等）
    /// 3. 四元数翻转问题（q 和 -q）
    /// 4. 角度范围问题
    /// </summary>
    public class RotationSingularityDiagnostic : MonoBehaviour
    {
        [ContextMenu("诊断1: 扫描不同角度的误差")]
        public void ScanAngleErrors()
        {
            Debug.Log("========== 扫描不同角度的转换误差 ==========\n");
            Debug.Log("测试目标: 找出哪些角度范围误差大\n");

            // 测试X轴旋转，角度从0到360度
            Debug.Log("=== X轴旋转扫描 ===");
            Debug.Log("角度(°)  |  往返误差(°)  |  状态");
            Debug.Log("--------------------------------------");

            float maxError = 0f;
            float maxErrorAngle = 0f;

            for (float angleDeg = 0f; angleDeg <= 360f; angleDeg += 10f)
            {
                float angleRad = angleDeg * Mathf.Deg2Rad;
                Vector3 rv_input = new Vector3(angleRad, 0, 0);

                // rv → quat → rv
                Quaternion q = RotationVectorToQuaternion(rv_input);
                Vector3 rv_output = QuaternionToRotationVector(q);

                Vector3 error = rv_output - rv_input;
                float errorDeg = error.magnitude * Mathf.Rad2Deg;

                string status = "✓";
                if (errorDeg > 0.1f) status = "⚠";
                if (errorDeg > 1.0f) status = "✗";

                Debug.Log($"{angleDeg,6:F1}  |  {errorDeg,12:F4}  |  {status}");

                if (errorDeg > maxError)
                {
                    maxError = errorDeg;
                    maxErrorAngle = angleDeg;
                }
            }

            Debug.Log("--------------------------------------");
            Debug.Log($"最大误差: {maxError:F4}° 出现在 {maxErrorAngle:F1}°\n");

            if (maxError > 1.0f)
            {
                Debug.LogError($"<color=red>✗ 存在严重误差! 在 {maxErrorAngle:F1}° 附近有奇异性问题</color>");
            }
            else if (maxError > 0.1f)
            {
                Debug.LogWarning($"<color=yellow>⚠ 在 {maxErrorAngle:F1}° 附近精度下降</color>");
            }
            else
            {
                Debug.Log("<color=green>✓ 所有角度误差都很小</color>");
            }

            Debug.Log("\n========== 扫描完成 ==========");
        }

        [ContextMenu("诊断2: 检查奇异点")]
        public void CheckSingularPoints()
        {
            Debug.Log("========== 检查已知奇异点 ==========\n");

            float[] criticalAngles = { 0f, 90f, 180f, 270f, 360f, 179.9f, 180.1f };

            foreach (float angleDeg in criticalAngles)
            {
                Debug.Log($"\n--- 测试: {angleDeg}° ---");
                
                float angleRad = angleDeg * Mathf.Deg2Rad;
                Vector3 rv_input = new Vector3(angleRad, 0, 0);

                Debug.Log($"输入旋转矢量: ({rv_input.x:F6}, {rv_input.y:F6}, {rv_input.z:F6})");

                // 步骤1: rv → quat
                Quaternion q = RotationVectorToQuaternion(rv_input);
                Debug.Log($"四元数: (w:{q.w:F6}, x:{q.x:F6}, y:{q.y:F6}, z:{q.z:F6})");
                Debug.Log($"  四元数模长: {Mathf.Sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z):F6}");
                
                // 检查四元数的符号
                if (q.w < 0)
                {
                    Debug.LogWarning("  <color=yellow>⚠ q.w < 0，可能导致方向反转!</color>");
                }

                // 步骤2: quat → rv
                Vector3 rv_output = QuaternionToRotationVector(q);
                Debug.Log($"输出旋转矢量: ({rv_output.x:F6}, {rv_output.y:F6}, {rv_output.z:F6})");

                // 误差
                Vector3 error = rv_output - rv_input;
                float errorDeg = error.magnitude * Mathf.Rad2Deg;
                Debug.Log($"往返误差: {errorDeg:F6}°");

                // 检查角度是否超过π
                float outputAngleDeg = rv_output.magnitude * Mathf.Rad2Deg;
                if (outputAngleDeg > 180f)
                {
                    Debug.LogWarning($"  <color=yellow>⚠ 输出角度 {outputAngleDeg:F2}° > 180°，可能是角度规范化问题</color>");
                }

                // 判断
                if (errorDeg > 1.0f)
                {
                    Debug.LogError("  <color=red>✗ 严重误差，存在奇异性!</color>");
                }
                else if (errorDeg > 0.1f)
                {
                    Debug.LogWarning("  <color=yellow>⚠ 精度下降</color>");
                }
                else
                {
                    Debug.Log("  <color=green>✓ 正常</color>");
                }
            }

            Debug.Log("\n========== 检查完成 ==========");
        }

        [ContextMenu("诊断3: 四元数翻转问题")]
        public void CheckQuaternionFlip()
        {
            Debug.Log("========== 四元数翻转(双重覆盖)问题诊断 ==========\n");
            Debug.Log("测试: q 和 -q 是否产生相同的旋转矢量\n");

            float angleDeg = 120f;
            float angleRad = angleDeg * Mathf.Deg2Rad;
            Vector3 axis = new Vector3(1, 1, 1).normalized;
            Vector3 rv = axis * angleRad;

            Debug.Log($"测试旋转: {angleDeg}°, 轴: ({axis.x:F4}, {axis.y:F4}, {axis.z:F4})\n");

            // 转换为四元数
            Quaternion q = RotationVectorToQuaternion(rv);
            Debug.Log($"四元数 q: (w:{q.w:F6}, x:{q.x:F6}, y:{q.y:F6}, z:{q.z:F6})");

            // 翻转四元数
            Quaternion q_neg = new Quaternion(-q.x, -q.y, -q.z, -q.w);
            Debug.Log($"四元数 -q: (w:{q_neg.w:F6}, x:{q_neg.x:F6}, y:{q_neg.y:F6}, z:{q_neg.z:F6})\n");

            // 转换回旋转矢量
            Vector3 rv_from_q = QuaternionToRotationVector(q);
            Vector3 rv_from_neg_q = QuaternionToRotationVector(q_neg);

            Debug.Log($"从 q 得到: ({rv_from_q.x:F6}, {rv_from_q.y:F6}, {rv_from_q.z:F6})");
            Debug.Log($"  角度: {rv_from_q.magnitude * Mathf.Rad2Deg:F2}°");
            
            Debug.Log($"从 -q 得到: ({rv_from_neg_q.x:F6}, {rv_from_neg_q.y:F6}, {rv_from_neg_q.z:F6})");
            Debug.Log($"  角度: {rv_from_neg_q.magnitude * Mathf.Rad2Deg:F2}°\n");

            // 对比
            Vector3 diff = rv_from_q - rv_from_neg_q;
            float diffAngle = diff.magnitude * Mathf.Rad2Deg;

            Debug.Log($"两者差异: {diffAngle:F6}°");

            if (diffAngle > 0.1f)
            {
                Debug.LogError("<color=red>✗ q 和 -q 产生不同结果! 存在翻转问题!</color>");
                Debug.Log("\n<b>问题原因:</b> QuaternionToRotationVector 没有规范化 q.w 的符号");
                Debug.Log("<b>解决方案:</b> 在转换前强制 q.w >= 0");
            }
            else
            {
                Debug.Log("<color=green>✓ q 和 -q 产生相同结果，翻转处理正确</color>");
            }

            Debug.Log("\n========== 诊断完成 ==========");
        }

        [ContextMenu("诊断4: 角度超出±π的情况")]
        public void CheckAngleWrapping()
        {
            Debug.Log("========== 角度超出±π范围的处理 ==========\n");
            Debug.Log("测试目标: 检查角度 > 180° 或 < -180° 时的表现\n");

            float[] testAngles = { 190f, 200f, 270f, 350f, 360f, 370f, -190f, -270f };

            foreach (float angleDeg in testAngles)
            {
                Debug.Log($"\n--- 测试: {angleDeg}° ---");
                
                float angleRad = angleDeg * Mathf.Deg2Rad;
                Vector3 rv_input = new Vector3(angleRad, 0, 0);

                Debug.Log($"输入角度: {angleDeg}° ({angleRad:F4} rad)");

                // 转换
                Quaternion q = RotationVectorToQuaternion(rv_input);
                Vector3 rv_output = QuaternionToRotationVector(q);

                float outputAngleDeg = rv_output.magnitude * Mathf.Rad2Deg;
                Debug.Log($"输出角度: {outputAngleDeg:F2}°");

                // 检查是否规范化到 [0, 180]
                float expectedNormalized = angleDeg;
                while (expectedNormalized > 180f) expectedNormalized -= 360f;
                while (expectedNormalized < -180f) expectedNormalized += 360f;
                if (expectedNormalized < 0f) expectedNormalized = -expectedNormalized;

                Debug.Log($"期望规范化: {expectedNormalized:F2}°");

                float error = Mathf.Abs(outputAngleDeg - expectedNormalized);
                Debug.Log($"误差: {error:F2}°");

                if (error > 1.0f)
                {
                    Debug.LogError($"  <color=red>✗ 角度规范化有问题!</color>");
                }
                else
                {
                    Debug.Log($"  <color=green>✓ 角度规范化正确</color>");
                }
            }

            Debug.Log("\n========== 诊断完成 ==========");
        }

        [ContextMenu("诊断5: 实际场景误差对比")]
        public void CompareRealScenario()
        {
            Debug.Log("========== 实际使用场景误差对比 ==========\n");
            Debug.Log("对比: 使用旋转矢量输入 vs 使用四元数输入\n");

            Vector3 pos = Vector3.zero;

            // 测试不同的输入角度
            float[] testAngles = { 10f, 45f, 90f, 120f, 150f, 170f, 180f };

            Debug.Log("角度(°)  |  方法1误差(°)  |  方法2误差(°)  |  差异(°)");
            Debug.Log("-----------------------------------------------------------");

            foreach (float angleDeg in testAngles)
            {
                float angleRad = angleDeg * Mathf.Deg2Rad;
                
                // 方法1: 使用旋转矢量输入 (会经过 rv→quat 转换)
                Vector3 rv_input = new Vector3(angleRad, 0, 0);
                SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                    pos, rv_input, false,
                    out _, out Vector3 rvOut1
                );

                // 方法2: 直接使用四元数输入
                Quaternion q_input = Quaternion.Euler(angleDeg, 0, 0);
                SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                    pos, q_input, false,
                    out _, out Vector3 rvOut2
                );

                // 计算相对旋转误差（使用正确的矩阵方法）
                Matrix4x4 R_base = GetOutputRotationMatrix(Quaternion.identity);
                Matrix4x4 R1 = GetOutputRotationMatrix(RotationVectorToQuaternion(rv_input));
                Matrix4x4 R2 = GetOutputRotationMatrix(q_input);

                Matrix4x4 R_base_inv = Transpose(R_base);
                float error1 = GetRotationAngle(Multiply3x3(R_base_inv, R1));
                float error2 = GetRotationAngle(Multiply3x3(R_base_inv, R2));

                float diff = Mathf.Abs(error1 - error2);

                string status = diff < 0.1f ? "✓" : (diff < 1.0f ? "⚠" : "✗");
                Debug.Log($"{angleDeg,6:F1}  |  {error1,13:F4}  |  {error2,13:F4}  |  {diff,8:F4}  {status}");
            }

            Debug.Log("-----------------------------------------------------------");
            Debug.Log("\n说明:");
            Debug.Log("  方法1: ConvertSteamVrPoseToUrBase(pos, rotvec, ...)");
            Debug.Log("  方法2: ConvertSteamVrPoseToUrBase(pos, quat, ...)");
            Debug.Log("  如果两种方法差异大，说明 rv→quat 转换有问题\n");

            Debug.Log("========== 对比完成 ==========");
        }

        // ========== 辅助方法 ==========

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
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (mag > 1e-12f)
            {
                float inv = 1f / mag;
                q.x *= inv; q.y *= inv; q.z *= inv; q.w *= inv;
            }

            float wClamped = Mathf.Clamp(q.w, -1f, 1f);
            float angle = 2f * Mathf.Acos(wClamped);
            float s = Mathf.Sqrt(1f - wClamped * wClamped);

            if (s < 1e-6f)
            {
                return new Vector3(q.x, q.y, q.z) * 2f;
            }

            Vector3 axis = new Vector3(q.x / s, q.y / s, q.z / s);
            return axis * angle;
        }

        private Matrix4x4 GetOutputRotationMatrix(Quaternion trackerQuat)
        {
            Matrix4x4 R_tracker = QuaternionToMatrix(trackerQuat);
            Matrix4x4 R_cam2base = ExtractRotation(GetHandEyeMatrix());
            Matrix4x4 R_offset = SteamVrUrCoordinateConverter.GetTrackerToTcpOffset();
            
            Matrix4x4 R_intermediate = Multiply3x3(R_cam2base, R_tracker);
            Matrix4x4 R_tcp = Multiply3x3(R_intermediate, R_offset);
            
            return R_tcp;
        }

        private Matrix4x4 GetHandEyeMatrix()
        {
            Matrix4x4 T = new Matrix4x4(
                new Vector4(-0.674113f, -0.738465f, 0.015512f, 0f),
                new Vector4(0.009269f, 0.012541f, 0.999878f, 0f),
                new Vector4(-0.738570f, 0.674175f, -0.001609f, 0f),
                new Vector4(0f, 0f, 0f, 1f)
            );
            T.m03 = -0.606508f;
            T.m13 = 0.882720f;
            T.m23 = 1.042878f;
            return T;
        }

        private Matrix4x4 ExtractRotation(Matrix4x4 m)
        {
            Matrix4x4 rot = Matrix4x4.identity;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    rot[i, j] = m[i, j];
            return rot;
        }

        private Matrix4x4 QuaternionToMatrix(Quaternion q)
        {
            float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
            float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
            float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

            Matrix4x4 mat = Matrix4x4.identity;
            mat.m00 = 1f - 2f * (yy + zz);
            mat.m01 = 2f * (xy - wz);
            mat.m02 = 2f * (xz + wy);
            mat.m10 = 2f * (xy + wz);
            mat.m11 = 1f - 2f * (xx + zz);
            mat.m12 = 2f * (yz - wx);
            mat.m20 = 2f * (xz - wy);
            mat.m21 = 2f * (yz + wx);
            mat.m22 = 1f - 2f * (xx + yy);
            return mat;
        }

        private Matrix4x4 Multiply3x3(Matrix4x4 a, Matrix4x4 b)
        {
            Matrix4x4 result = Matrix4x4.identity;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    result[i, j] = 0f;
                    for (int k = 0; k < 3; k++)
                        result[i, j] += a[i, k] * b[k, j];
                }
            }
            return result;
        }

        private Matrix4x4 Transpose(Matrix4x4 m)
        {
            Matrix4x4 result = Matrix4x4.identity;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    result[i, j] = m[j, i];
            return result;
        }

        private float GetRotationAngle(Matrix4x4 m)
        {
            float trace = m.m00 + m.m11 + m.m22;
            float angle = Mathf.Acos(Mathf.Clamp((trace - 1f) / 2f, -1f, 1f));
            return angle * Mathf.Rad2Deg;
        }
    }
}

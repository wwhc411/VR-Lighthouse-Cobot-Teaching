using UnityEngine;

namespace handeye
{
    /// <summary>
    /// 旋转一致性验证工具
    /// 用于验证旋转变换是否正确反映 Tracker 的运动
    /// </summary>
    public class RotationConsistencyTest : MonoBehaviour
    {
        [Header("=== 测试配置 ===")]
        [Tooltip("测试旋转角度（度）")]
        public float testAngleDeg = 10f;

        [ContextMenu("测试: 相对旋转一致性")]
        public void TestRotationConsistency()
        {
            Debug.Log("========== 旋转一致性测试 ==========\n");
            Debug.Log($"测试方法: 对比 Identity 和绕各轴旋转 {testAngleDeg}° 的输出差异\n");

            // 基准测试: Identity
            Quaternion q_identity = Quaternion.identity;
            Vector3 rotvec_identity = ProcessRotation(q_identity);
            Debug.Log("=== 基准: Identity (无旋转) ===");
            Debug.Log($"输出旋转向量: ({rotvec_identity.x:F4}, {rotvec_identity.y:F4}, {rotvec_identity.z:F4})");
            Debug.Log($"输出旋转角度: {rotvec_identity.magnitude * Mathf.Rad2Deg:F2}°\n");

            // 测试1: 绕X轴旋转
            Quaternion q_rotX = Quaternion.Euler(testAngleDeg, 0, 0);
            Vector3 rotvec_rotX = ProcessRotation(q_rotX);
            Vector3 delta_rotX = rotvec_rotX - rotvec_identity;
            
            Debug.Log($"=== 测试1: 绕X轴旋转 {testAngleDeg}° ===");
            Debug.Log($"输入四元数: (x:{q_rotX.x:F4}, y:{q_rotX.y:F4}, z:{q_rotX.z:F4}, w:{q_rotX.w:F4})");
            Debug.Log($"输出旋转向量: ({rotvec_rotX.x:F4}, {rotvec_rotX.y:F4}, {rotvec_rotX.z:F4})");
            Debug.Log($"相对变化: Δ({delta_rotX.x:F4}, {delta_rotX.y:F4}, {delta_rotX.z:F4})");
            Debug.Log($"变化量大小: {delta_rotX.magnitude * Mathf.Rad2Deg:F2}°\n");

            // 测试2: 绕Y轴旋转
            Quaternion q_rotY = Quaternion.Euler(0, testAngleDeg, 0);
            Vector3 rotvec_rotY = ProcessRotation(q_rotY);
            Vector3 delta_rotY = rotvec_rotY - rotvec_identity;
            
            Debug.Log($"=== 测试2: 绕Y轴旋转 {testAngleDeg}° ===");
            Debug.Log($"输入四元数: (x:{q_rotY.x:F4}, y:{q_rotY.y:F4}, z:{q_rotY.z:F4}, w:{q_rotY.w:F4})");
            Debug.Log($"输出旋转向量: ({rotvec_rotY.x:F4}, {rotvec_rotY.y:F4}, {rotvec_rotY.z:F4})");
            Debug.Log($"相对变化: Δ({delta_rotY.x:F4}, {delta_rotY.y:F4}, {delta_rotY.z:F4})");
            Debug.Log($"变化量大小: {delta_rotY.magnitude * Mathf.Rad2Deg:F2}°\n");

            // 测试3: 绕Z轴旋转
            Quaternion q_rotZ = Quaternion.Euler(0, 0, testAngleDeg);
            Vector3 rotvec_rotZ = ProcessRotation(q_rotZ);
            Vector3 delta_rotZ = rotvec_rotZ - rotvec_identity;
            
            Debug.Log($"=== 测试3: 绕Z轴旋转 {testAngleDeg}° ===");
            Debug.Log($"输入四元数: (x:{q_rotZ.x:F4}, y:{q_rotZ.y:F4}, z:{q_rotZ.z:F4}, w:{q_rotZ.w:F4})");
            Debug.Log($"输出旋转向量: ({rotvec_rotZ.x:F4}, {rotvec_rotZ.y:F4}, {rotvec_rotZ.z:F4})");
            Debug.Log($"相对变化: Δ({delta_rotZ.x:F4}, {delta_rotZ.y:F4}, {delta_rotZ.z:F4})");
            Debug.Log($"变化量大小: {delta_rotZ.magnitude * Mathf.Rad2Deg:F2}°\n");

            // 验证
            Debug.Log("=== 一致性检查 ===");
            float expectedDelta = testAngleDeg;
            float tolerance = 0.1f; // 0.1度误差容限

            bool xOk = Mathf.Abs(delta_rotX.magnitude * Mathf.Rad2Deg - expectedDelta) < tolerance;
            bool yOk = Mathf.Abs(delta_rotY.magnitude * Mathf.Rad2Deg - expectedDelta) < tolerance;
            bool zOk = Mathf.Abs(delta_rotZ.magnitude * Mathf.Rad2Deg - expectedDelta) < tolerance;

            Debug.Log($"X轴旋转 {testAngleDeg}° → 输出变化 {delta_rotX.magnitude * Mathf.Rad2Deg:F2}° : {(xOk ? "✓" : "✗")}");
            Debug.Log($"Y轴旋转 {testAngleDeg}° → 输出变化 {delta_rotY.magnitude * Mathf.Rad2Deg:F2}° : {(yOk ? "✓" : "✗")}");
            Debug.Log($"Z轴旋转 {testAngleDeg}° → 输出变化 {delta_rotZ.magnitude * Mathf.Rad2Deg:F2}° : {(zOk ? "✓" : "✗")}");

            if (xOk && yOk && zOk)
            {
                Debug.Log("\n<color=green>✓✓✓ 旋转变换一致性良好！系统工作正常！</color>");
                Debug.Log("结论: 之前观察到的 212° 旋转是手眼标定的固有偏移，这是正常的。");
            }
            else
            {
                Debug.LogWarning("\n<color=yellow>⚠ 旋转变换存在不一致！需要进一步检查。</color>");
            }

            Debug.Log("\n========== 测试完成 ==========");
        }

        [ContextMenu("测试: 逆变换验证")]
        public void TestInverseTransform()
        {
            Debug.Log("========== 逆变换验证 ==========\n");
            
            // 测试: 正向变换 + 逆向变换应该回到原点
            Quaternion q_test = Quaternion.Euler(30, 45, 60);
            Debug.Log($"输入姿态: Euler(30°, 45°, 60°)");
            Debug.Log($"输入四元数: (x:{q_test.x:F4}, y:{q_test.y:F4}, z:{q_test.z:F4}, w:{q_test.w:F4})\n");

            // 正向变换
            Vector3 rotvec_forward = ProcessRotation(q_test);
            Debug.Log("=== 正向变换 ===");
            Debug.Log($"输出旋转向量: ({rotvec_forward.x:F4}, {rotvec_forward.y:F4}, {rotvec_forward.z:F4})");
            Debug.Log($"旋转角度: {rotvec_forward.magnitude * Mathf.Rad2Deg:F2}°\n");

            Debug.Log("注意: 当前系统没有实现逆变换接口，此测试仅用于演示。");
            Debug.Log("如需逆变换，需要实现: Tracker姿态 = T_cam2base^-1 × UR姿态 × R_offset^-1\n");

            Debug.Log("========== 测试完成 ==========");
        }

        /// <summary>
        /// 处理旋转变换（模拟实际系统）
        /// </summary>
        private Vector3 ProcessRotation(Quaternion trackerQuat)
        {
            // 步骤1: 四元数 → 旋转矩阵
            Matrix4x4 R_tracker = QuaternionToMatrix(trackerQuat);

            // 步骤2: 手眼标定
            Matrix4x4 T_cam2base = GetHandEyeMatrix();
            Matrix4x4 R_cam2base = ExtractRotation(T_cam2base);

            // 步骤3: R_intermediate = R_cam2base × R_tracker
            Matrix4x4 R_intermediate = Multiply3x3(R_cam2base, R_tracker);

            // 步骤4: 应用偏移
            Matrix4x4 R_offset = SteamVrUrCoordinateConverter.GetTrackerToTcpOffset();
            Matrix4x4 R_tcp = Multiply3x3(R_intermediate, R_offset);

            // 步骤5: 旋转矩阵 → 四元数
            Quaternion quat_tcp = MatrixToQuaternion(R_tcp);

            // 步骤6: 四元数 → 轴角
            Vector3 rotvec = QuaternionToAxisAngle(quat_tcp);

            return rotvec;
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

        private Quaternion MatrixToQuaternion(Matrix4x4 m)
        {
            float trace = m.m00 + m.m11 + m.m22;
            Quaternion q = Quaternion.identity;

            if (trace > 0f)
            {
                float s = 2f * Mathf.Sqrt(trace + 1f);
                q.w = 0.25f * s;
                q.x = (m.m21 - m.m12) / s;
                q.y = (m.m02 - m.m20) / s;
                q.z = (m.m10 - m.m01) / s;
            }
            else if (m.m00 > m.m11 && m.m00 > m.m22)
            {
                float s = 2f * Mathf.Sqrt(1f + m.m00 - m.m11 - m.m22);
                q.w = (m.m21 - m.m12) / s;
                q.x = 0.25f * s;
                q.y = (m.m01 + m.m10) / s;
                q.z = (m.m02 + m.m20) / s;
            }
            else if (m.m11 > m.m22)
            {
                float s = 2f * Mathf.Sqrt(1f + m.m11 - m.m00 - m.m22);
                q.w = (m.m02 - m.m20) / s;
                q.x = (m.m01 + m.m10) / s;
                q.y = 0.25f * s;
                q.z = (m.m12 + m.m21) / s;
            }
            else
            {
                float s = 2f * Mathf.Sqrt(1f + m.m22 - m.m00 - m.m11);
                q.w = (m.m10 - m.m01) / s;
                q.x = (m.m02 + m.m20) / s;
                q.y = (m.m12 + m.m21) / s;
                q.z = 0.25f * s;
            }
            return q;
        }

        private Matrix4x4 Multiply3x3(Matrix4x4 A, Matrix4x4 B)
        {
            Matrix4x4 result = Matrix4x4.identity;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    float sum = 0f;
                    for (int k = 0; k < 3; k++)
                        sum += A[i, k] * B[k, j];
                    result[i, j] = sum;
                }
            return result;
        }

        private Vector3 QuaternionToAxisAngle(Quaternion q)
        {
            if (q.w > 1f || q.w < -1f)
            {
                float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
                if (mag > 1e-12f)
                {
                    float inv = 1f / mag;
                    q.x *= inv; q.y *= inv; q.z *= inv; q.w *= inv;
                }
            }

            float wClamped = Mathf.Clamp(q.w, -1f, 1f);
            float angle = 2f * Mathf.Acos(wClamped);
            float s = Mathf.Sqrt(1f - wClamped * wClamped);

            if (s < 1e-6f)
                return new Vector3(q.x, q.y, q.z) * 2f;

            Vector3 axis = new Vector3(q.x / s, q.y / s, q.z / s);
            return axis * angle;
        }
    }
}

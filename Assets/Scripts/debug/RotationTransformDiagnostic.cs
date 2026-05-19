using UnityEngine;

namespace handeye
{
    /// <summary>
    /// 旋转变换诊断工具
    /// 用于排查手眼标定后旋转变换不正确的问题
    /// </summary>
    public class RotationTransformDiagnostic : MonoBehaviour
    {
        [Header("=== 测试数据输入 ===")]
        [Tooltip("输入 Tracker 的四元数 (Unity格式: x, y, z, w)")]
        public Quaternion trackerQuaternion = Quaternion.identity;

        [Header("=== 诊断选项 ===")]
        [Tooltip("是否打印详细的中间步骤")]
        public bool printDetailedSteps = true;

        [ContextMenu("执行旋转变换诊断")]
        public void DiagnoseRotationTransform()
        {
            Debug.Log("========== 旋转变换诊断开始 ==========");
            Debug.Log($"输入四元数: (x:{trackerQuaternion.x:F4}, y:{trackerQuaternion.y:F4}, z:{trackerQuaternion.z:F4}, w:{trackerQuaternion.w:F4})");

            // 步骤1: 四元数 → 旋转矩阵
            Matrix4x4 R_tracker = QuaternionToMatrix(trackerQuaternion);
            Debug.Log("\n=== 步骤1: Tracker 四元数 → 旋转矩阵 ===");
            PrintMatrix("R_tracker", R_tracker);

            // 步骤2: 获取手眼标定矩阵的旋转部分
            Matrix4x4 T_cam2base = GetHandEyeCalibrationMatrix();
            Matrix4x4 R_cam2base = ExtractRotation(T_cam2base);
            Debug.Log("\n=== 步骤2: 手眼标定旋转矩阵 ===");
            PrintMatrix("R_cam2base", R_cam2base);

            // 步骤3: 应用手眼标定 R_intermediate = R_cam2base × R_tracker
            Matrix4x4 R_intermediate = Multiply3x3(R_cam2base, R_tracker);
            Debug.Log("\n=== 步骤3: R_intermediate = R_cam2base × R_tracker ===");
            PrintMatrix("R_intermediate", R_intermediate);

            // 步骤4: 获取 Tracker到TCP 的偏移矩阵
            Matrix4x4 R_offset = SteamVrUrCoordinateConverter.GetTrackerToTcpOffset();
            Debug.Log("\n=== 步骤4: Tracker到TCP偏移矩阵 ===");
            PrintMatrix("R_tracker2tcp_offset", R_offset);

            // 步骤5: 应用偏移 R_tcp = R_intermediate × R_offset
            Matrix4x4 R_tcp = Multiply3x3(R_intermediate, R_offset);
            Debug.Log("\n=== 步骤5: R_tcp = R_intermediate × R_offset ===");
            PrintMatrix("R_tcp (最终旋转矩阵)", R_tcp);

            // 步骤6: 旋转矩阵 → 四元数
            Quaternion quat_tcp = MatrixToQuaternion(R_tcp);
            Debug.Log("\n=== 步骤6: 旋转矩阵 → 四元数 ===");
            Debug.Log($"输出四元数: (x:{quat_tcp.x:F4}, y:{quat_tcp.y:F4}, z:{quat_tcp.z:F4}, w:{quat_tcp.w:F4})");
            Debug.Log($"输出欧拉角: {quat_tcp.eulerAngles}");

            // 步骤7: 四元数 → 轴角（UR格式）
            Vector3 rotvec = QuaternionToAxisAngle(quat_tcp);
            Debug.Log("\n=== 步骤7: 四元数 → 轴角(UR格式) ===");
            Debug.Log($"旋转向量(rad): ({rotvec.x:F4}, {rotvec.y:F4}, {rotvec.z:F4})");
            Debug.Log($"旋转角度: {rotvec.magnitude * Mathf.Rad2Deg:F2}°");

            // 验证矩阵正交性
            Debug.Log("\n=== 矩阵正交性验证 ===");
            VerifyOrthogonality("R_tracker", R_tracker);
            VerifyOrthogonality("R_cam2base", R_cam2base);
            VerifyOrthogonality("R_offset", R_offset);
            VerifyOrthogonality("R_tcp", R_tcp);

            Debug.Log("\n========== 旋转变换诊断完成 ==========");
        }

        [ContextMenu("测试: 单位矩阵 (无旋转)")]
        public void TestIdentity()
        {
            trackerQuaternion = Quaternion.identity;
            DiagnoseRotationTransform();
        }

        [ContextMenu("测试: 绕X轴旋转90度")]
        public void TestRotateX90()
        {
            trackerQuaternion = Quaternion.Euler(90, 0, 0);
            DiagnoseRotationTransform();
        }

        [ContextMenu("测试: 绕Y轴旋转90度")]
        public void TestRotateY90()
        {
            trackerQuaternion = Quaternion.Euler(0, 90, 0);
            DiagnoseRotationTransform();
        }

        [ContextMenu("测试: 绕Z轴旋转90度")]
        public void TestRotateZ90()
        {
            trackerQuaternion = Quaternion.Euler(0, 0, 90);
            DiagnoseRotationTransform();
        }

        // ========== 辅助方法 ==========

        private Matrix4x4 GetHandEyeCalibrationMatrix()
        {
            // 硬编码当前的手眼标定结果
            Matrix4x4 T_cam2base = new Matrix4x4(
                new Vector4(-0.674113f, -0.738465f, 0.015512f, 0f),
                new Vector4(0.009269f, 0.012541f, 0.999878f, 0f),
                new Vector4(-0.738570f, 0.674175f, -0.001609f, 0f),
                new Vector4(0f, 0f, 0f, 1f)
            );
            T_cam2base.m03 = -0.606508f;
            T_cam2base.m13 = 0.882720f;
            T_cam2base.m23 = 1.042878f;
            return T_cam2base;
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

        private void PrintMatrix(string name, Matrix4x4 m)
        {
            Debug.Log($"{name}:");
            Debug.Log($"  [{m.m00:F4}, {m.m01:F4}, {m.m02:F4}]");
            Debug.Log($"  [{m.m10:F4}, {m.m11:F4}, {m.m12:F4}]");
            Debug.Log($"  [{m.m20:F4}, {m.m21:F4}, {m.m22:F4}]");
        }

        private void VerifyOrthogonality(string name, Matrix4x4 m)
        {
            // 计算 M^T × M，应该接近单位矩阵
            Matrix4x4 mT = Transpose(m);
            Matrix4x4 product = Multiply3x3(mT, m);

            float error = 0f;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    float expected = (i == j) ? 1f : 0f;
                    error += Mathf.Abs(product[i, j] - expected);
                }

            if (error < 0.001f)
                Debug.Log($"<color=green>✓ {name} 正交性良好 (误差: {error:F6})</color>");
            else
                Debug.LogWarning($"<color=red>⚠ {name} 正交性异常! (误差: {error:F6})</color>");
        }

        private Matrix4x4 Transpose(Matrix4x4 m)
        {
            Matrix4x4 result = Matrix4x4.identity;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    result[i, j] = m[j, i];
            return result;
        }
    }
}

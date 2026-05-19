using UnityEngine;

namespace handeye
{
    /// <summary>
    /// 旋转矩阵级别的一致性测试
    /// 绕过四元数和轴角，直接在矩阵层面验证
    /// </summary>
    public class MatrixLevelConsistencyTest : MonoBehaviour
    {
        [Header("=== 测试配置 ===")]
        [Tooltip("测试旋转角度（度）")]
        public float testAngleDeg = 10f;

        [ContextMenu("测试: 矩阵级别的旋转一致性")]
        public void TestMatrixConsistency()
        {
            Debug.Log("========== 矩阵级别一致性测试 ==========\n");
            Debug.Log("原理: 直接对比旋转矩阵，不经过四元数和轴角转换\n");

            // 获取基准矩阵
            Matrix4x4 M_identity = ProcessToMatrix(Quaternion.identity);
            Debug.Log("=== 基准矩阵 (Identity输入) ===");
            PrintMatrix("M_base", M_identity);
            Debug.Log("");

            // 测试X轴旋转
            Quaternion q_rotX = Quaternion.Euler(testAngleDeg, 0, 0);
            Matrix4x4 M_rotX = ProcessToMatrix(q_rotX);
            Matrix4x4 M_deltaX = MultiplyMatrices(Transpose(M_identity), M_rotX); // Delta = M_base^T × M_rotX
            
            Debug.Log($"=== 测试1: 输入绕X轴旋转 {testAngleDeg}° ===");
            PrintMatrix("M_rotX (输出矩阵)", M_rotX);
            Debug.Log("\n相对旋转矩阵 (ΔM = M_base^T × M_rotX):");
            PrintMatrix("ΔM_X", M_deltaX);
            
            float angleX = GetRotationAngle(M_deltaX);
            Vector3 axisX = GetRotationAxis(M_deltaX);
            Debug.Log($"相对旋转角度: {angleX:F2}°");
            Debug.Log($"相对旋转轴: ({axisX.x:F4}, {axisX.y:F4}, {axisX.z:F4})\n");

            // 测试Y轴旋转
            Quaternion q_rotY = Quaternion.Euler(0, testAngleDeg, 0);
            Matrix4x4 M_rotY = ProcessToMatrix(q_rotY);
            Matrix4x4 M_deltaY = MultiplyMatrices(Transpose(M_identity), M_rotY);
            
            Debug.Log($"=== 测试2: 输入绕Y轴旋转 {testAngleDeg}° ===");
            PrintMatrix("M_rotY (输出矩阵)", M_rotY);
            Debug.Log("\n相对旋转矩阵:");
            PrintMatrix("ΔM_Y", M_deltaY);
            
            float angleY = GetRotationAngle(M_deltaY);
            Vector3 axisY = GetRotationAxis(M_deltaY);
            Debug.Log($"相对旋转角度: {angleY:F2}°");
            Debug.Log($"相对旋转轴: ({axisY.x:F4}, {axisY.y:F4}, {axisY.z:F4})\n");

            // 测试Z轴旋转
            Quaternion q_rotZ = Quaternion.Euler(0, 0, testAngleDeg);
            Matrix4x4 M_rotZ = ProcessToMatrix(q_rotZ);
            Matrix4x4 M_deltaZ = MultiplyMatrices(Transpose(M_identity), M_rotZ);
            
            Debug.Log($"=== 测试3: 输入绕Z轴旋转 {testAngleDeg}° ===");
            PrintMatrix("M_rotZ (输出矩阵)", M_rotZ);
            Debug.Log("\n相对旋转矩阵:");
            PrintMatrix("ΔM_Z", M_deltaZ);
            
            float angleZ = GetRotationAngle(M_deltaZ);
            Vector3 axisZ = GetRotationAxis(M_deltaZ);
            Debug.Log($"相对旋转角度: {angleZ:F2}°");
            Debug.Log($"相对旋转轴: ({axisZ.x:F4}, {axisZ.y:F4}, {axisZ.z:F4})\n");

            // 验证
            Debug.Log("=== 一致性检查 ===");
            float tolerance = 0.5f; // 0.5度容差
            bool xOk = Mathf.Abs(angleX - testAngleDeg) < tolerance;
            bool yOk = Mathf.Abs(angleY - testAngleDeg) < tolerance;
            bool zOk = Mathf.Abs(angleZ - testAngleDeg) < tolerance;

            Debug.Log($"X轴: 输入 {testAngleDeg}° → 矩阵显示 {angleX:F2}° : {(xOk ? "✓" : "✗")}");
            Debug.Log($"Y轴: 输入 {testAngleDeg}° → 矩阵显示 {angleY:F2}° : {(yOk ? "✓" : "✗")}");
            Debug.Log($"Z轴: 输入 {testAngleDeg}° → 矩阵显示 {angleZ:F2}° : {(zOk ? "✓" : "✗")}");

            if (xOk && yOk && zOk)
            {
                Debug.Log("\n<color=green>✓ 矩阵级别一致性良好！</color>");
                Debug.Log("结论: 问题出在 矩阵→四元数→轴角 的转换过程中。");
            }
            else
            {
                Debug.LogError("\n<color=red>✗ 矩阵级别就已经不一致！问题在矩阵变换本身！</color>");
                Debug.Log("结论: 需要检查矩阵乘法顺序或手眼标定矩阵。");
            }

            Debug.Log("\n========== 测试完成 ==========");
        }

        [ContextMenu("测试: 对比变换顺序")]
        public void TestTransformOrder()
        {
            Debug.Log("========== 变换顺序对比测试 ==========\n");
            Debug.Log("测试不同的矩阵乘法顺序是否会改善一致性\n");

            Quaternion q_test = Quaternion.Euler(10, 0, 0);
            Matrix4x4 R_tracker = QuaternionToMatrix(q_test);
            Matrix4x4 R_cam2base = ExtractRotation(GetHandEyeMatrix());
            Matrix4x4 R_offset = SteamVrUrCoordinateConverter.GetTrackerToTcpOffset();

            // 当前顺序: R_tcp = R_cam2base × R_tracker × R_offset
            Matrix4x4 R_current = Multiply3x3(Multiply3x3(R_cam2base, R_tracker), R_offset);
            Debug.Log("=== 当前顺序: (R_cam2base × R_tracker) × R_offset ===");
            PrintMatrix("R_tcp", R_current);
            Debug.Log("");

            // 备选1: R_tcp = R_offset × R_cam2base × R_tracker
            Matrix4x4 R_alt1 = Multiply3x3(Multiply3x3(R_offset, R_cam2base), R_tracker);
            Debug.Log("=== 备选1: (R_offset × R_cam2base) × R_tracker ===");
            PrintMatrix("R_tcp", R_alt1);
            Debug.Log("");

            // 备选2: R_tcp = R_cam2base × R_offset × R_tracker
            Matrix4x4 R_alt2 = Multiply3x3(Multiply3x3(R_cam2base, R_offset), R_tracker);
            Debug.Log("=== 备选2: (R_cam2base × R_offset) × R_tracker ===");
            PrintMatrix("R_tcp", R_alt2);
            Debug.Log("");

            // 备选3: R_tcp = R_tracker × R_cam2base × R_offset
            Matrix4x4 R_alt3 = Multiply3x3(Multiply3x3(R_tracker, R_cam2base), R_offset);
            Debug.Log("=== 备选3: (R_tracker × R_cam2base) × R_offset ===");
            PrintMatrix("R_tcp", R_alt3);
            Debug.Log("");

            Debug.Log("注意: 需要根据坐标系的物理意义选择正确顺序。");
            Debug.Log("建议: 手动分析每种顺序的物理含义，选择合理的一种。\n");
            Debug.Log("========== 测试完成 ==========");
        }

        // ========== 辅助方法 ==========

        private Matrix4x4 ProcessToMatrix(Quaternion trackerQuat)
        {
            Matrix4x4 R_tracker = QuaternionToMatrix(trackerQuat);
            Matrix4x4 R_cam2base = ExtractRotation(GetHandEyeMatrix());
            Matrix4x4 R_intermediate = Multiply3x3(R_cam2base, R_tracker);
            Matrix4x4 R_offset = SteamVrUrCoordinateConverter.GetTrackerToTcpOffset();
            Matrix4x4 R_tcp = Multiply3x3(R_intermediate, R_offset);
            return R_tcp;
        }

        private float GetRotationAngle(Matrix4x4 m)
        {
            float trace = m.m00 + m.m11 + m.m22;
            float angle = Mathf.Acos(Mathf.Clamp((trace - 1f) / 2f, -1f, 1f));
            return angle * Mathf.Rad2Deg;
        }

        private Vector3 GetRotationAxis(Matrix4x4 m)
        {
            float angle = GetRotationAngle(m) * Mathf.Deg2Rad;
            if (Mathf.Abs(angle) < 1e-6f)
                return Vector3.zero;

            float s = 2f * Mathf.Sin(angle);
            if (Mathf.Abs(s) < 1e-6f)
                return Vector3.zero;

            Vector3 axis = new Vector3(
                (m.m21 - m.m12) / s,
                (m.m02 - m.m20) / s,
                (m.m10 - m.m01) / s
            );
            return axis.normalized;
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

        private Matrix4x4 Transpose(Matrix4x4 m)
        {
            Matrix4x4 result = Matrix4x4.identity;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    result[i, j] = m[j, i];
            return result;
        }

        private Matrix4x4 MultiplyMatrices(Matrix4x4 A, Matrix4x4 B)
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

        private Matrix4x4 Multiply3x3(Matrix4x4 A, Matrix4x4 B)
        {
            return MultiplyMatrices(A, B);
        }

        private void PrintMatrix(string name, Matrix4x4 m)
        {
            Debug.Log($"{name}:");
            Debug.Log($"  [{m.m00:F4}, {m.m01:F4}, {m.m02:F4}]");
            Debug.Log($"  [{m.m10:F4}, {m.m11:F4}, {m.m12:F4}]");
            Debug.Log($"  [{m.m20:F4}, {m.m21:F4}, {m.m22:F4}]");
        }
    }
}

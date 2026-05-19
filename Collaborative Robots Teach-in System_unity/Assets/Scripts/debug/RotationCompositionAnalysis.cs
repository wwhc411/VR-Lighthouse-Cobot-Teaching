using UnityEngine;

namespace handeye
{
    /// <summary>
    /// 旋转组合分析工具
    /// 
    /// 分析为什么10°输入会产生>10°的输出变化
    /// 核心问题: 在大角度基准旋转下，旋转向量不满足线性叠加
    /// </summary>
    public class RotationCompositionAnalysis : MonoBehaviour
    {
        [ContextMenu("分析: 旋转组合非线性")]
        public void AnalyzeNonlinearity()
        {
            Debug.Log("========== 旋转组合非线性分析 ==========\n");
            Debug.Log("目标: 理解为什么10°输入会产生>10°的输出变化\n");

            // 基准状态
            Debug.Log("=== 步骤1: 基准状态 ===");
            Vector3 pos = Vector3.zero;
            Quaternion qBase = Quaternion.identity;
            
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                pos, qBase, false,
                out _, out Vector3 rvBase
            );
            Debug.Log($"基准轴角: rv_base = ({rvBase.x:F4}, {rvBase.y:F4}, {rvBase.z:F4})");
            Debug.Log($"基准角度: |rv_base| = {rvBase.magnitude * Mathf.Rad2Deg:F2}°\n");

            // 旋转后状态(绕X轴+10°)
            Debug.Log("=== 步骤2: Tracker绕X轴旋转+10° ===");
            Quaternion qRotated = Quaternion.Euler(10, 0, 0);
            
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                pos, qRotated, false,
                out _, out Vector3 rvRotated
            );
            Debug.Log($"旋转后轴角: rv_rot = ({rvRotated.x:F4}, {rvRotated.y:F4}, {rvRotated.z:F4})");
            Debug.Log($"旋转后角度: |rv_rot| = {rvRotated.magnitude * Mathf.Rad2Deg:F2}°\n");

            // 计算差值
            Debug.Log("=== 步骤3: 计算简单差值 ===");
            Vector3 deltaSimple = rvRotated - rvBase;
            Debug.Log($"简单差值: Δrv = rv_rot - rv_base");
            Debug.Log($"        = ({deltaSimple.x:F4}, {deltaSimple.y:F4}, {deltaSimple.z:F4})");
            Debug.Log($"差值大小: |Δrv| = {deltaSimple.magnitude * Mathf.Rad2Deg:F2}°");
            Debug.Log("<color=yellow>⚠ 这就是VerifyRotationFix中测到的13.07°</color>\n");

            // 解释为什么会这样
            Debug.Log("=== 问题分析 ===");
            Debug.Log("旋转向量(轴角表示)不满足向量加法!");
            Debug.Log("  R1 对应 rv1");
            Debug.Log("  R2 对应 rv2");
            Debug.Log("  R1×R2 对应 rv3");
            Debug.Log("  但 rv3 ≠ rv1 + rv2 (除非角度很小)");
            Debug.Log("");
            Debug.Log("在我们的情况下:");
            Debug.Log($"  rv_base 是212°的大角度旋转");
            Debug.Log($"  rv_rot 是 212°旋转 × 10°旋转 的组合");
            Debug.Log($"  简单差值 |rv_rot - rv_base| = {deltaSimple.magnitude * Mathf.Rad2Deg:F2}°");
            Debug.Log($"  并不等于10°!\n");

            // 正确的计算方法
            Debug.Log("=== 步骤4: 正确的相对旋转计算 ===");
            Debug.Log("应该计算: R_rel = R_base^(-1) × R_rot");
            Debug.Log("然后从 R_rel 提取轴角\n");

            // 重新构建两个旋转矩阵
            Matrix4x4 R_base = GetOutputRotationMatrix(qBase);
            Matrix4x4 R_rot = GetOutputRotationMatrix(qRotated);

            Debug.Log("R_base:");
            PrintMatrix(R_base);
            Debug.Log($"角度: {GetRotationAngle(R_base):F2}°\n");

            Debug.Log("R_rot:");
            PrintMatrix(R_rot);
            Debug.Log($"角度: {GetRotationAngle(R_rot):F2}°\n");

            // 计算相对旋转矩阵
            Matrix4x4 R_base_inv = Transpose(R_base); // 正交矩阵的转置 = 逆
            Matrix4x4 R_rel = Multiply3x3(R_base_inv, R_rot);

            Debug.Log("R_rel = R_base^(-1) × R_rot:");
            PrintMatrix(R_rel);
            
            float relAngle = GetRotationAngle(R_rel);
            Vector3 relAxis = GetRotationAxis(R_rel);
            Debug.Log($"相对旋转角度: {relAngle:F2}°");
            Debug.Log($"相对旋转轴: ({relAxis.x:F4}, {relAxis.y:F4}, {relAxis.z:F4})");
            Debug.Log($"<color=green>这才是真正的输入旋转大小!</color>\n");

            // 结论
            Debug.Log("=== 结论 ===");
            if (Mathf.Abs(relAngle - 10f) < 0.5f)
            {
                Debug.Log("<color=green>✓ 相对旋转矩阵显示10°，说明变换本身是对的!</color>");
                Debug.Log("<color=yellow>⚠ 问题在于: 我们的测试方法不对</color>");
                Debug.Log("  - 不应该用 |rv_rot - rv_base| 来衡量变化");
                Debug.Log("  - 应该用相对旋转矩阵 R_rel");
            }
            else
            {
                Debug.Log($"<color=red>✗ 相对旋转矩阵显示 {relAngle:F2}°，仍然不对</color>");
                Debug.Log("  需要进一步检查矩阵变换");
            }

            Debug.Log("\n========== 分析完成 ==========");
        }

        [ContextMenu("验证: 所有轴的相对旋转")]
        public void VerifyAllAxes()
        {
            Debug.Log("========== 验证所有轴的相对旋转矩阵 ==========\n");

            Vector3 pos = Vector3.zero;
            Quaternion qBase = Quaternion.identity;
            Matrix4x4 R_base = GetOutputRotationMatrix(qBase);
            Matrix4x4 R_base_inv = Transpose(R_base);

            Debug.Log($"基准旋转角度: {GetRotationAngle(R_base):F2}°\n");

            // X轴
            Debug.Log("=== X轴 +10° ===");
            TestAxis(Quaternion.Euler(10, 0, 0), R_base_inv);

            // Y轴
            Debug.Log("\n=== Y轴 +10° ===");
            TestAxis(Quaternion.Euler(0, 10, 0), R_base_inv);

            // Z轴
            Debug.Log("\n=== Z轴 +10° ===");
            TestAxis(Quaternion.Euler(0, 0, 10), R_base_inv);

            Debug.Log("\n========== 验证完成 ==========");
        }

        private void TestAxis(Quaternion qInput, Matrix4x4 R_base_inv)
        {
            Matrix4x4 R_out = GetOutputRotationMatrix(qInput);
            Matrix4x4 R_rel = Multiply3x3(R_base_inv, R_out);

            float angle = GetRotationAngle(R_rel);
            Vector3 axis = GetRotationAxis(R_rel);

            Debug.Log($"输入: Euler({qInput.eulerAngles.x:F1}, {qInput.eulerAngles.y:F1}, {qInput.eulerAngles.z:F1})");
            Debug.Log($"相对旋转角度: {angle:F2}°");
            Debug.Log($"相对旋转轴: ({axis.x:F4}, {axis.y:F4}, {axis.z:F4})");
            
            bool ok = Mathf.Abs(angle - 10f) < 0.5f;
            Debug.Log(ok ? "<color=green>✓ 正确</color>" : $"<color=red>✗ 错误(期望10°)</color>");
        }

        private Matrix4x4 GetOutputRotationMatrix(Quaternion trackerQuat)
        {
            // 模拟完整的转换流程
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

        private void PrintMatrix(Matrix4x4 m)
        {
            Debug.Log($"  [{m.m00:F4}, {m.m01:F4}, {m.m02:F4}]");
            Debug.Log($"  [{m.m10:F4}, {m.m11:F4}, {m.m12:F4}]");
            Debug.Log($"  [{m.m20:F4}, {m.m21:F4}, {m.m22:F4}]");
        }
    }
}

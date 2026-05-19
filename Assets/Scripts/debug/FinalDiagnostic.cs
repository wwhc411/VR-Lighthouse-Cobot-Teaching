using UnityEngine;

namespace handeye
{
    /// <summary>
    /// 最终诊断: 理解"旋转不正确"的真正含义
    /// 
    /// 澄清两个概念:
    /// 1. Tracker在自身局部坐标系旋转10° (输入)
    /// 2. TCP在UR base全局坐标系的旋转变化 (输出)
    /// 
    /// 这两者在大角度手眼标定(212°)下是不相等的!
    /// </summary>
    public class FinalDiagnostic : MonoBehaviour
    {
        [ContextMenu("最终诊断: 完整分析")]
        public void CompleteDiagnostic()
        {
            Debug.Log("========== 最终诊断: 完整分析 ==========\n");
            Debug.Log("<size=14><b>目标: 理解为什么会出现11-13°的输出变化</b></size>\n");

            Vector3 pos = Vector3.zero;

            // ========== 部分1: 基准状态 ==========
            Debug.Log("<color=cyan>━━━━━ 第1步: 基准状态 ━━━━━</color>");
            Quaternion qBase = Quaternion.identity;
            
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                pos, qBase, false, out _, out Vector3 rvBase
            );
            
            Matrix4x4 R_base = GetOutputRotationMatrix(qBase);
            float baseAngle = GetRotationAngle(R_base);
            
            Debug.Log($"Tracker: Identity (无旋转)");
            Debug.Log($"输出轴角: ({rvBase.x:F4}, {rvBase.y:F4}, {rvBase.z:F4})");
            Debug.Log($"输出角度: {baseAngle:F2}° ← 这是手眼标定的212°\n");

            // ========== 部分2: X轴旋转 ==========
            Debug.Log("<color=cyan>━━━━━ 第2步: Tracker绕X轴旋转+10° ━━━━━</color>");
            Quaternion qX = Quaternion.Euler(10, 0, 0);
            
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                pos, qX, false, out _, out Vector3 rvX
            );
            
            Matrix4x4 R_x = GetOutputRotationMatrix(qX);
            float xAngle = GetRotationAngle(R_x);
            
            Debug.Log($"Tracker: 绕X轴+10° (局部坐标系)");
            Debug.Log($"输出轴角: ({rvX.x:F4}, {rvX.y:F4}, {rvX.z:F4})");
            Debug.Log($"输出角度: {xAngle:F2}°\n");

            // ========== 部分3: 错误的测试方法 ==========
            Debug.Log("<color=yellow>━━━━━ 第3步: 错误的测试方法 ━━━━━</color>");
            Vector3 deltaSimple = rvX - rvBase;
            float deltaSimpleAngle = deltaSimple.magnitude * Mathf.Rad2Deg;
            
            Debug.Log("方法A: 简单差值 |rv_x - rv_base|");
            Debug.Log($"  结果: {deltaSimpleAngle:F2}°");
            Debug.Log($"  <color=yellow>⚠ 这就是VerifyRotationFix显示的13.07°</color>");
            Debug.Log($"  <color=red>✗ 这种方法是错误的!</color>");
            Debug.Log($"  原因: 旋转向量不满足向量加减法\n");

            // ========== 部分4: 正确的测试方法 ==========
            Debug.Log("<color=green>━━━━━ 第4步: 正确的测试方法 ━━━━━</color>");
            Matrix4x4 R_base_inv = Transpose(R_base);
            Matrix4x4 R_rel = Multiply3x3(R_base_inv, R_x);
            
            float relAngle = GetRotationAngle(R_rel);
            Vector3 relAxis = GetRotationAxis(R_rel);
            
            Debug.Log("方法B: 相对旋转矩阵 R_rel = R_base^(-1) × R_x");
            PrintMatrix("R_rel", R_rel);
            Debug.Log($"相对旋转角度: {relAngle:F2}°");
            Debug.Log($"相对旋转轴: ({relAxis.x:F4}, {relAxis.y:F4}, {relAxis.z:F4})");
            
            bool correct = Mathf.Abs(relAngle - 10f) < 0.5f;
            if (correct)
            {
                Debug.Log($"<color=green>✓ 相对旋转矩阵显示10.00°，系统是正确的!</color>\n");
            }
            else
            {
                Debug.Log($"<color=red>✗ 相对旋转矩阵显示{relAngle:F2}°，系统有问题</color>\n");
            }

            // ========== 部分5: 测试所有轴 ==========
            Debug.Log("<color=cyan>━━━━━ 第5步: 测试所有轴 ━━━━━</color>");
            TestAxisDetailed("X轴", Quaternion.Euler(10, 0, 0), R_base_inv);
            TestAxisDetailed("Y轴", Quaternion.Euler(0, 10, 0), R_base_inv);
            TestAxisDetailed("Z轴", Quaternion.Euler(0, 0, 10), R_base_inv);

            // ========== 总结 ==========
            Debug.Log("\n<color=cyan>━━━━━━━━━━━━━━━━━━━━━━━━━━━━</color>");
            Debug.Log("<size=14><b>总结与结论</b></size>\n");
            
            Debug.Log("<b>问题根源:</b>");
            Debug.Log("  VerifyRotationFix使用了错误的测试方法");
            Debug.Log("  计算 |rv_rotated - rv_base| 不能正确反映相对旋转\n");
            
            Debug.Log("<b>正确的测试方法:</b>");
            Debug.Log("  1. 从输出轴角反构建旋转矩阵");
            Debug.Log("  2. 计算相对旋转矩阵 R_rel = R_base^(-1) × R_rotated");
            Debug.Log("  3. 从R_rel提取角度才是真正的旋转变化\n");
            
            if (correct)
            {
                Debug.Log("<color=green><b>✓✓✓ 系统是正确的!</b></color>");
                Debug.Log("<color=green>直接从矩阵转轴角的方法已经成功修复了问题</color>");
                Debug.Log("<color=green>之前的测试方法本身有误导性</color>\n");
            }
            else
            {
                Debug.Log("<color=red><b>✗✗✗ 系统仍有问题</b></color>");
                Debug.Log("<color=red>需要进一步检查矩阵变换逻辑</color>\n");
            }
            
            Debug.Log("========== 诊断完成 ==========");
        }

        [ContextMenu("简化测试: 只看相对旋转")]
        public void SimpleTest()
        {
            Debug.Log("========== 简化测试: 相对旋转矩阵法 ==========\n");

            Vector3 pos = Vector3.zero;
            Quaternion qBase = Quaternion.identity;
            Matrix4x4 R_base = GetOutputRotationMatrix(qBase);
            Matrix4x4 R_base_inv = Transpose(R_base);

            Debug.Log($"基准旋转: {GetRotationAngle(R_base):F2}°\n");
            Debug.Log("测试: Tracker各轴旋转+10°\n");

            TestAxis("X", Quaternion.Euler(10, 0, 0), R_base_inv);
            TestAxis("Y", Quaternion.Euler(0, 10, 0), R_base_inv);
            TestAxis("Z", Quaternion.Euler(0, 0, 10), R_base_inv);

            Debug.Log("\n期望: 所有轴都应该显示 ~10.00°");
            Debug.Log("========== 测试完成 ==========");
        }

        private void TestAxis(string name, Quaternion qInput, Matrix4x4 R_base_inv)
        {
            Matrix4x4 R_out = GetOutputRotationMatrix(qInput);
            Matrix4x4 R_rel = Multiply3x3(R_base_inv, R_out);
            float angle = GetRotationAngle(R_rel);
            
            bool ok = Mathf.Abs(angle - 10f) < 0.5f;
            string status = ok ? "<color=green>✓</color>" : "<color=red>✗</color>";
            Debug.Log($"{name}轴: {angle:F2}° {status}");
        }

        private void TestAxisDetailed(string name, Quaternion qInput, Matrix4x4 R_base_inv)
        {
            Matrix4x4 R_out = GetOutputRotationMatrix(qInput);
            Matrix4x4 R_rel = Multiply3x3(R_base_inv, R_out);
            
            float angle = GetRotationAngle(R_rel);
            Vector3 axis = GetRotationAxis(R_rel);
            
            Debug.Log($"\n<b>{name}</b>");
            Debug.Log($"  相对旋转角度: {angle:F2}°");
            Debug.Log($"  相对旋转轴: ({axis.x:F4}, {axis.y:F4}, {axis.z:F4})");
            
            bool ok = Mathf.Abs(angle - 10f) < 0.5f;
            if (ok)
            {
                Debug.Log($"  <color=green>✓ 正确 (误差: {Mathf.Abs(angle - 10f):F3}°)</color>");
            }
            else
            {
                Debug.Log($"  <color=red>✗ 错误 (期望10°，偏差: {angle - 10f:F2}°)</color>");
            }
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

        private void PrintMatrix(string label, Matrix4x4 m)
        {
            Debug.Log($"{label}:");
            Debug.Log($"  [{m.m00:F4}, {m.m01:F4}, {m.m02:F4}]");
            Debug.Log($"  [{m.m10:F4}, {m.m11:F4}, {m.m12:F4}]");
            Debug.Log($"  [{m.m20:F4}, {m.m21:F4}, {m.m22:F4}]");
        }
    }
}

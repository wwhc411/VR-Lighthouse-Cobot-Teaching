using UnityEngine;

namespace handeye
{
    /// <summary>
    /// 矩阵索引调试工具
    /// 
    /// 验证Unity Matrix4x4的索引方式，并测试Rodrigues公式
    /// </summary>
    public class MatrixIndexingDebug : MonoBehaviour
    {
        [ContextMenu("测试矩阵索引")]
        public void TestMatrixIndexing()
        {
            Debug.Log("========== Unity Matrix4x4 索引测试 ==========\n");

            // 创建一个简单的矩阵用于测试索引
            Matrix4x4 m = new Matrix4x4();
            m.m00 = 00; m.m01 = 01; m.m02 = 02; m.m03 = 03;
            m.m10 = 10; m.m11 = 11; m.m12 = 12; m.m13 = 13;
            m.m20 = 20; m.m21 = 21; m.m22 = 22; m.m23 = 23;
            m.m30 = 30; m.m31 = 31; m.m32 = 32; m.m33 = 33;

            Debug.Log("测试矩阵(mXY表示第X行第Y列):");
            Debug.Log($"m00={m.m00:F0}  m01={m.m01:F0}  m02={m.m02:F0}  m03={m.m03:F0}");
            Debug.Log($"m10={m.m10:F0}  m11={m.m11:F0}  m12={m.m12:F0}  m13={m.m13:F0}");
            Debug.Log($"m20={m.m20:F0}  m21={m.m21:F0}  m22={m.m22:F0}  m23={m.m23:F0}");
            Debug.Log($"m30={m.m30:F0}  m31={m.m31:F0}  m32={m.m32:F0}  m33={m.m33:F0}\n");

            Debug.Log("验证: mXY中，X是行索引，Y是列索引");
            Debug.Log($"  m21 (第2行第1列) = {m.m21:F0}");
            Debug.Log($"  m12 (第1行第2列) = {m.m12:F0}\n");
        }

        [ContextMenu("测试Rodrigues公式 - 已知旋转")]
        public void TestRodriguesFormula()
        {
            Debug.Log("========== Rodrigues公式测试 ==========\n");

            // 测试1: 绕X轴旋转90度
            float angle = Mathf.PI / 2f; // 90度
            Debug.Log("=== 测试1: 绕X轴旋转90° ===");
            TestAxisRotation(Vector3.right, angle);

            Debug.Log("\n=== 测试2: 绕Y轴旋转90° ===");
            TestAxisRotation(Vector3.up, angle);

            Debug.Log("\n=== 测试3: 绕Z轴旋转90° ===");
            TestAxisRotation(Vector3.forward, angle);

            // 测试4: 绕任意轴旋转
            Vector3 arbitraryAxis = new Vector3(1, 1, 1).normalized;
            Debug.Log("\n=== 测试4: 绕(1,1,1)归一化轴旋转45° ===");
            TestAxisRotation(arbitraryAxis, Mathf.PI / 4f);
        }

        private void TestAxisRotation(Vector3 axis, float angle)
        {
            Debug.Log($"输入: 轴 = ({axis.x:F4}, {axis.y:F4}, {axis.z:F4}), 角度 = {angle * Mathf.Rad2Deg:F2}°");

            // 使用Unity的Quaternion构建旋转
            Quaternion q = Quaternion.AngleAxis(angle * Mathf.Rad2Deg, axis);
            Matrix4x4 R = Matrix4x4.Rotate(q);

            Debug.Log($"旋转矩阵:");
            Debug.Log($"  [{R.m00:F4}, {R.m01:F4}, {R.m02:F4}]");
            Debug.Log($"  [{R.m10:F4}, {R.m11:F4}, {R.m12:F4}]");
            Debug.Log($"  [{R.m20:F4}, {R.m21:F4}, {R.m22:F4}]");

            // 提取角度: θ = arccos((trace - 1) / 2)
            float trace = R.m00 + R.m11 + R.m22;
            float extractedAngle = Mathf.Acos(Mathf.Clamp((trace - 1f) / 2f, -1f, 1f));
            Debug.Log($"提取的角度: {extractedAngle * Mathf.Rad2Deg:F2}° (误差: {Mathf.Abs(extractedAngle - angle) * Mathf.Rad2Deg:F4}°)");

            // 方法1: 当前实现 (m21-m12, m02-m20, m10-m01)
            float s1 = 2f * Mathf.Sin(extractedAngle);
            Vector3 axis1 = new Vector3(
                (R.m21 - R.m12) / s1,
                (R.m02 - R.m20) / s1,
                (R.m10 - R.m01) / s1
            );
            Debug.Log($"方法1(当前): 轴 = ({axis1.x:F4}, {axis1.y:F4}, {axis1.z:F4})");
            Debug.Log($"  与输入轴的点积: {Vector3.Dot(axis, axis1):F4} (期望1.0)");

            // 方法2: 尝试 (m12-m21, m20-m02, m01-m10) - 符号相反
            Vector3 axis2 = new Vector3(
                (R.m12 - R.m21) / s1,
                (R.m20 - R.m02) / s1,
                (R.m01 - R.m10) / s1
            );
            Debug.Log($"方法2(相反): 轴 = ({axis2.x:F4}, {axis2.y:F4}, {axis2.z:F4})");
            Debug.Log($"  与输入轴的点积: {Vector3.Dot(axis, axis2):F4}");
        }

        [ContextMenu("测试完整转换流程")]
        public void TestCompleteConversion()
        {
            Debug.Log("========== 完整转换流程测试 ==========\n");

            // 模拟VerifyRotationFix中的测试场景
            Vector3 pos = Vector3.zero;
            
            // 基准: Identity
            Quaternion qIdentity = Quaternion.identity;
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                pos, qIdentity, false,
                out Vector3 posBase, out Vector3 rotvecBase
            );
            Debug.Log($"基准旋转向量: ({rotvecBase.x:F4}, {rotvecBase.y:F4}, {rotvecBase.z:F4})");
            Debug.Log($"基准角度: {rotvecBase.magnitude * Mathf.Rad2Deg:F2}°\n");

            // 测试: 绕X轴旋转10度
            Quaternion qX = Quaternion.Euler(10, 0, 0);
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                pos, qX, false,
                out Vector3 posX, out Vector3 rotvecX
            );
            Debug.Log($"X轴+10°后: ({rotvecX.x:F4}, {rotvecX.y:F4}, {rotvecX.z:F4})");
            
            Vector3 deltaX = rotvecX - rotvecBase;
            float deltaAngleX = deltaX.magnitude * Mathf.Rad2Deg;
            Debug.Log($"相对变化: Δ({deltaX.x:F4}, {deltaX.y:F4}, {deltaX.z:F4})");
            Debug.Log($"变化角度: {deltaAngleX:F2}° (期望: 10°, 误差: {Mathf.Abs(deltaAngleX - 10f):F2}°)");

            // 检查delta向量的方向
            Vector3 deltaDir = deltaX.normalized;
            Debug.Log($"变化方向(归一化): ({deltaDir.x:F4}, {deltaDir.y:F4}, {deltaDir.z:F4})");
        }
    }
}

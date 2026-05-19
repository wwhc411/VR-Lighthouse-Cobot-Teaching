using UnityEngine;

namespace handeye
{
    /// <summary>
    /// R_tracker2tcp_offset 矩阵验证工具
    /// </summary>
    public class OffsetMatrixTest : MonoBehaviour
    {
        [ContextMenu("测试 R_tracker2tcp_offset 的实际值")]
        public void TestOffsetMatrix()
        {
            Debug.Log("========== R_tracker2tcp_offset 测试 ==========\n");

            // 方法1: 从 SteamVrUrCoordinateConverter 获取
            Matrix4x4 currentOffset = SteamVrUrCoordinateConverter.GetTrackerToTcpOffset();
            Debug.Log("=== 当前 R_tracker2tcp_offset ===");
            PrintMatrix("Current", currentOffset);

            // 方法2: 预期的矩阵（按行定义）
            Matrix4x4 expectedOffset = Matrix4x4.identity;
            expectedOffset.m00 = 1f;  expectedOffset.m01 = 0f;  expectedOffset.m02 = 0f;
            expectedOffset.m10 = 0f;  expectedOffset.m11 = -1f; expectedOffset.m12 = 0f;
            expectedOffset.m20 = 0f;  expectedOffset.m21 = 0f;  expectedOffset.m22 = -1f;
            
            Debug.Log("\n=== 预期的 R_tracker2tcp_offset ===");
            PrintMatrix("Expected", expectedOffset);

            // 方法3: 使用构造函数（列）
            Matrix4x4 constructedOffset = new Matrix4x4(
                new Vector4(1f, 0f, 0f, 0f),
                new Vector4(0f, -1f, 0f, 0f),
                new Vector4(0f, 0f, -1f, 0f),
                new Vector4(0f, 0f, 0f, 1f)
            );
            
            Debug.Log("\n=== 构造函数创建的矩阵 ===");
            PrintMatrix("Constructed", constructedOffset);

            // 对比
            Debug.Log("\n=== 对比结果 ===");
            bool match1 = CompareMatrices(currentOffset, expectedOffset);
            bool match2 = CompareMatrices(currentOffset, constructedOffset);
            bool match3 = CompareMatrices(expectedOffset, constructedOffset);

            Debug.Log($"Current == Expected: {match1}");
            Debug.Log($"Current == Constructed: {match2}");
            Debug.Log($"Expected == Constructed: {match3}");

            // 测试向量变换
            Debug.Log("\n=== 向量变换测试 ===");
            Vector3 testVec = new Vector3(1, 1, 1);
            Debug.Log($"输入向量: {testVec}");

            Vector3 result1 = MultiplyVector(currentOffset, testVec);
            Vector3 result2 = MultiplyVector(expectedOffset, testVec);
            
            Debug.Log($"Current 变换结果: {result1}");
            Debug.Log($"Expected 变换结果: {result2}");
            Debug.Log($"预期结果: (1, -1, -1)");

            if (Mathf.Approximately(result2.x, 1f) && 
                Mathf.Approximately(result2.y, -1f) && 
                Mathf.Approximately(result2.z, -1f))
            {
                Debug.Log("<color=green>✓ 预期矩阵工作正常</color>");
            }

            if (!match1)
            {
                Debug.LogError("<color=red>⚠ 当前矩阵与预期不符！需要修复！</color>");
            }
        }

        [ContextMenu("应用正确的 R_tracker2tcp_offset")]
        public void ApplyCorrectOffset()
        {
            Matrix4x4 correctOffset = Matrix4x4.identity;
            correctOffset.m00 = 1f;  correctOffset.m01 = 0f;  correctOffset.m02 = 0f;
            correctOffset.m10 = 0f;  correctOffset.m11 = -1f; correctOffset.m12 = 0f;
            correctOffset.m20 = 0f;  correctOffset.m21 = 0f;  correctOffset.m22 = -1f;

            SteamVrUrCoordinateConverter.SetTrackerToTcpOffset(correctOffset);
            
            Debug.Log("========== 已应用正确的 R_tracker2tcp_offset ==========");
            PrintMatrix("Applied Offset", correctOffset);
            Debug.Log("\n请重新运行旋转诊断测试验证效果！");
        }

        private void PrintMatrix(string name, Matrix4x4 m)
        {
            Debug.Log($"{name}:");
            Debug.Log($"  [{m.m00:F4}, {m.m01:F4}, {m.m02:F4}]");
            Debug.Log($"  [{m.m10:F4}, {m.m11:F4}, {m.m12:F4}]");
            Debug.Log($"  [{m.m20:F4}, {m.m21:F4}, {m.m22:F4}]");
        }

        private bool CompareMatrices(Matrix4x4 a, Matrix4x4 b, float tolerance = 0.0001f)
        {
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    if (Mathf.Abs(a[i, j] - b[i, j]) > tolerance)
                        return false;
            return true;
        }

        private Vector3 MultiplyVector(Matrix4x4 m, Vector3 v)
        {
            return new Vector3(
                m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
                m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
                m.m20 * v.x + m.m21 * v.y + m.m22 * v.z
            );
        }
    }
}

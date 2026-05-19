using UnityEngine;

namespace handeye
{
    /// <summary>
    /// Unity Matrix4x4 索引约定测试工具
    /// 用于确认 Unity 矩阵是行主序还是列主序
    /// </summary>
    public class MatrixIndexingTest : MonoBehaviour
    {
        [ContextMenu("测试 Unity Matrix4x4 索引约定")]
        public void TestMatrixIndexing()
        {
            Debug.Log("========== Unity Matrix4x4 索引测试 ==========\n");

            // 测试1: 使用 Matrix4x4 构造函数
            Debug.Log("=== 测试1: Matrix4x4 构造函数 ===");
            Matrix4x4 mat1 = new Matrix4x4(
                new Vector4(1, 2, 3, 4),   // 第一个参数
                new Vector4(5, 6, 7, 8),   // 第二个参数
                new Vector4(9, 10, 11, 12), // 第三个参数
                new Vector4(13, 14, 15, 16) // 第四个参数
            );
            Debug.Log("构造时使用:");
            Debug.Log("  参数1: (1, 2, 3, 4)");
            Debug.Log("  参数2: (5, 6, 7, 8)");
            Debug.Log("  参数3: (9, 10, 11, 12)");
            Debug.Log("  参数4: (13, 14, 15, 16)\n");
            
            Debug.Log("使用 [i,j] 索引读取:");
            Debug.Log($"  [0,0]={mat1[0,0]}, [0,1]={mat1[0,1]}, [0,2]={mat1[0,2]}, [0,3]={mat1[0,3]}");
            Debug.Log($"  [1,0]={mat1[1,0]}, [1,1]={mat1[1,1]}, [1,2]={mat1[1,2]}, [1,3]={mat1[1,3]}");
            Debug.Log($"  [2,0]={mat1[2,0]}, [2,1]={mat1[2,1]}, [2,2]={mat1[2,2]}, [2,3]={mat1[2,3]}");
            Debug.Log($"  [3,0]={mat1[3,0]}, [3,1]={mat1[3,1]}, [3,2]={mat1[3,2]}, [3,3]={mat1[3,3]}\n");

            Debug.Log("使用 mXY 属性读取:");
            Debug.Log($"  第0行: m00={mat1.m00}, m01={mat1.m01}, m02={mat1.m02}, m03={mat1.m03}");
            Debug.Log($"  第1行: m10={mat1.m10}, m11={mat1.m11}, m12={mat1.m12}, m13={mat1.m13}");
            Debug.Log($"  第2行: m20={mat1.m20}, m21={mat1.m21}, m22={mat1.m22}, m23={mat1.m23}");
            Debug.Log($"  第3行: m30={mat1.m30}, m31={mat1.m31}, m32={mat1.m32}, m33={mat1.m33}\n");

            // 测试2: 旋转矩阵测试
            Debug.Log("=== 测试2: 绕Z轴旋转90度 ===");
            Quaternion quat = Quaternion.Euler(0, 0, 90);
            Debug.Log($"四元数: (x:{quat.x:F4}, y:{quat.y:F4}, z:{quat.z:F4}, w:{quat.w:F4})\n");

            // Unity 内置转换
            Matrix4x4 unityMatrix = Matrix4x4.Rotate(quat);
            Debug.Log("Unity 内置 Matrix4x4.Rotate() 结果:");
            PrintMatrix3x3("Unity Matrix", unityMatrix);

            // 手动计算（假设行优先）
            Matrix4x4 manualMatrix = QuaternionToRotationMatrix(quat);
            Debug.Log("手动计算结果:");
            PrintMatrix3x3("Manual Matrix", manualMatrix);

            // 比较
            bool match = CompareMatrices(unityMatrix, manualMatrix);
            if (match)
                Debug.Log("<color=green>✓ 手动计算与 Unity 内置结果一致</color>");
            else
                Debug.LogWarning("<color=red>⚠ 手动计算与 Unity 内置结果不一致!</color>");

            // 测试3: 向量变换测试
            Debug.Log("\n=== 测试3: 向量变换验证 ===");
            Vector3 testVec = new Vector3(1, 0, 0); // X轴单位向量
            Debug.Log($"原始向量: {testVec}");

            Vector3 unityTransform = unityMatrix.MultiplyVector(testVec);
            Vector3 manualTransform = MultiplyMatrixVector(manualMatrix, testVec);
            
            Debug.Log($"Unity 变换结果: {unityTransform}");
            Debug.Log($"手动变换结果: {manualTransform}");
            Debug.Log($"预期结果 (Z轴旋转90°): (0, 1, 0)");

            // 测试4: 验证手眼标定矩阵构造
            Debug.Log("\n=== 测试4: 手眼标定矩阵构造 ===");
            Matrix4x4 calibMatrix = new Matrix4x4(
                new Vector4(-0.674113f, -0.738465f, 0.015512f, 0f),
                new Vector4(0.009269f, 0.012541f, 0.999878f, 0f),
                new Vector4(-0.738570f, 0.674175f, -0.001609f, 0f),
                new Vector4(0f, 0f, 0f, 1f)
            );
            Debug.Log("手眼标定矩阵 (使用构造函数):");
            PrintMatrix3x3("T_cam2base", calibMatrix);
            Debug.Log($"位置: ({calibMatrix.m03:F4}, {calibMatrix.m13:F4}, {calibMatrix.m23:F4})");

            // 检查正交性
            Debug.Log("\n=== 正交性检查 ===");
            VerifyOrthogonality("Unity Matrix", unityMatrix);
            VerifyOrthogonality("Manual Matrix", manualMatrix);
            VerifyOrthogonality("Calibration Matrix", calibMatrix);

            Debug.Log("\n========== 测试完成 ==========");
        }

        private Matrix4x4 QuaternionToRotationMatrix(Quaternion q)
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

        private Vector3 MultiplyMatrixVector(Matrix4x4 m, Vector3 v)
        {
            return new Vector3(
                m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
                m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
                m.m20 * v.x + m.m21 * v.y + m.m22 * v.z
            );
        }

        private void PrintMatrix3x3(string name, Matrix4x4 m)
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

        private void VerifyOrthogonality(string name, Matrix4x4 m)
        {
            // 提取3x3旋转部分
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
                Debug.Log($"<color=green>✓ {name} 正交 (误差: {error:F6})</color>");
            else
                Debug.LogWarning($"<color=red>⚠ {name} 非正交! (误差: {error:F6})</color>");
        }

        private Matrix4x4 Transpose(Matrix4x4 m)
        {
            Matrix4x4 result = Matrix4x4.identity;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    result[i, j] = m[j, i];
            return result;
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
    }
}

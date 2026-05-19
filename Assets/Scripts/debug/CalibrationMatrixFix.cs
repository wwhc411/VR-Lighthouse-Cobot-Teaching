using UnityEngine;

namespace handeye
{
    /// <summary>
    /// 手眼标定矩阵诊断与修正工具
    /// 用于检测并修正可能的矩阵转置问题
    /// </summary>
    public class CalibrationMatrixFix : MonoBehaviour
    {
        [Header("=== 手眼标定原始数据 (来自DLL) ===")]
        [Tooltip("DLL输出的旋转矩阵 - 按行输入")]
        public Vector3 row1 = new Vector3(-0.674113f, 0.009269f, -0.738570f);
        public Vector3 row2 = new Vector3(-0.738465f, 0.012541f, 0.674175f);
        public Vector3 row3 = new Vector3(0.015512f, 0.999878f, -0.001609f);
        
        [Tooltip("位置 (mm)")]
        public Vector3 position_mm = new Vector3(-606.508f, 882.720f, 1042.878f);

        [ContextMenu("诊断当前手眼标定矩阵")]
        public void DiagnoseCurrentMatrix()
        {
            Debug.Log("========== 手眼标定矩阵诊断 ==========\n");

            // 获取当前代码中的矩阵（可能是转置的）
            Matrix4x4 currentMatrix = new Matrix4x4(
                new Vector4(-0.674113f, -0.738465f, 0.015512f, 0f),
                new Vector4(0.009269f, 0.012541f, 0.999878f, 0f),
                new Vector4(-0.738570f, 0.674175f, -0.001609f, 0f),
                new Vector4(0f, 0f, 0f, 1f)
            );
            currentMatrix.m03 = -0.606508f;
            currentMatrix.m13 = 0.882720f;
            currentMatrix.m23 = 1.042878f;

            Debug.Log("=== 当前代码中的矩阵 ===");
            PrintMatrix("Current T_cam2base", currentMatrix);
            
            // 正确构造（按行数据构造）
            Matrix4x4 correctMatrix = ConstructMatrixFromRows(row1, row2, row3);
            correctMatrix.m03 = position_mm.x / 1000f;
            correctMatrix.m13 = position_mm.y / 1000f;
            correctMatrix.m23 = position_mm.z / 1000f;

            Debug.Log("\n=== 正确构造的矩阵（按行） ===");
            PrintMatrix("Correct T_cam2base", correctMatrix);

            // 检查是否需要转置
            bool needsTranspose = !CompareMatrices(currentMatrix, correctMatrix);
            if (needsTranspose)
            {
                Debug.LogWarning("<color=red>⚠ 发现问题：当前矩阵需要转置！</color>");
                Debug.Log("\n当前矩阵的旋转部分是正确矩阵的转置");
            }
            else
            {
                Debug.Log("<color=green>✓ 当前矩阵构造正确</color>");
            }

            // 验证正交性
            Debug.Log("\n=== 正交性验证 ===");
            VerifyOrthogonality("Current Matrix", currentMatrix);
            VerifyOrthogonality("Correct Matrix", correctMatrix);

            // 测试变换效果
            Debug.Log("\n=== 变换测试 (使用单位向量) ===");
            Vector3 testVec = new Vector3(1, 0, 0);
            Vector3 result1 = currentMatrix.MultiplyVector(testVec);
            Vector3 result2 = correctMatrix.MultiplyVector(testVec);
            Debug.Log($"输入向量: {testVec}");
            Debug.Log($"当前矩阵变换结果: {result1}");
            Debug.Log($"正确矩阵变换结果: {result2}");

            Debug.Log("\n========== 诊断完成 ==========");
        }

        [ContextMenu("生成正确的矩阵构造代码")]
        public void GenerateCorrectCode()
        {
            Debug.Log("========== 正确的矩阵构造代码 ==========\n");
            
            Debug.Log("// 方法1: 使用行数据直接构造（推荐）");
            Debug.Log("private static Matrix4x4 T_cam2base_m = ConstructMatrixFromRows(");
            Debug.Log($"    new Vector3({row1.x}f, {row1.y}f, {row1.z}f),");
            Debug.Log($"    new Vector3({row2.x}f, {row2.y}f, {row2.z}f),");
            Debug.Log($"    new Vector3({row3.x}f, {row3.y}f, {row3.z}f)");
            Debug.Log(");");
            Debug.Log($"T_cam2base_m.m03 = {position_mm.x / 1000f}f;");
            Debug.Log($"T_cam2base_m.m13 = {position_mm.y / 1000f}f;");
            Debug.Log($"T_cam2base_m.m23 = {position_mm.z / 1000f}f;\n");

            Debug.Log("// 方法2: 使用Matrix4x4构造函数（按列）");
            Debug.Log("// 注意：每个Vector4是一列，需要将行数据转置");
            Debug.Log("private static Matrix4x4 T_cam2base_m = new Matrix4x4(");
            Debug.Log($"    new Vector4({row1.x}f, {row2.x}f, {row3.x}f, 0f),  // 列1");
            Debug.Log($"    new Vector4({row1.y}f, {row2.y}f, {row3.y}f, 0f),  // 列2");
            Debug.Log($"    new Vector4({row1.z}f, {row2.z}f, {row3.z}f, 0f),  // 列3");
            Debug.Log("    new Vector4(0f, 0f, 0f, 1f)");
            Debug.Log(");\n");

            Debug.Log("// 辅助方法");
            Debug.Log("private static Matrix4x4 ConstructMatrixFromRows(Vector3 row1, Vector3 row2, Vector3 row3)");
            Debug.Log("{");
            Debug.Log("    Matrix4x4 mat = Matrix4x4.identity;");
            Debug.Log("    mat.m00 = row1.x; mat.m01 = row1.y; mat.m02 = row1.z;");
            Debug.Log("    mat.m10 = row2.x; mat.m11 = row2.y; mat.m12 = row2.z;");
            Debug.Log("    mat.m20 = row3.x; mat.m21 = row3.y; mat.m22 = row3.z;");
            Debug.Log("    return mat;");
            Debug.Log("}");
        }

        [ContextMenu("对比两种构造方法")]
        public void CompareConstructionMethods()
        {
            Debug.Log("========== 对比两种构造方法 ==========\n");

            // 方法1: 当前代码使用的方法（列构造）
            Matrix4x4 method1 = new Matrix4x4(
                new Vector4(-0.674113f, -0.738465f, 0.015512f, 0f),
                new Vector4(0.009269f, 0.012541f, 0.999878f, 0f),
                new Vector4(-0.738570f, 0.674175f, -0.001609f, 0f),
                new Vector4(0f, 0f, 0f, 1f)
            );

            Debug.Log("=== 方法1: Matrix4x4构造函数（列向量） ===");
            Debug.Log("构造时输入:");
            Debug.Log("  列1: (-0.674113, -0.738465, 0.015512)");
            Debug.Log("  列2: (0.009269, 0.012541, 0.999878)");
            Debug.Log("  列3: (-0.738570, 0.674175, -0.001609)");
            PrintMatrix("Method 1 Result", method1);

            // 方法2: 按行构造
            Matrix4x4 method2 = Matrix4x4.identity;
            method2.m00 = -0.674113f; method2.m01 = 0.009269f; method2.m02 = -0.738570f;
            method2.m10 = -0.738465f; method2.m11 = 0.012541f; method2.m12 = 0.674175f;
            method2.m20 = 0.015512f;  method2.m21 = 0.999878f; method2.m22 = -0.001609f;

            Debug.Log("\n=== 方法2: 逐元素赋值（行优先） ===");
            Debug.Log("构造时输入:");
            Debug.Log("  行1: (-0.674113, 0.009269, -0.738570)");
            Debug.Log("  行2: (-0.738465, 0.012541, 0.674175)");
            Debug.Log("  行3: (0.015512, 0.999878, -0.001609)");
            PrintMatrix("Method 2 Result", method2);

            // 对比
            Debug.Log("\n=== 结论 ===");
            if (CompareMatrices(method1, method2))
            {
                Debug.Log("<color=green>✓ 两种方法结果相同</color>");
            }
            else
            {
                Debug.LogWarning("<color=red>⚠ 两种方法结果不同！</color>");
                Debug.Log("方法1得到的是方法2的转置矩阵");
                
                Matrix4x4 method1Transposed = Transpose(method1);
                if (CompareMatrices(method1Transposed, method2))
                {
                    Debug.Log("<color=yellow>验证：method1的转置 = method2 ✓</color>");
                }
            }
        }

        // ========== 辅助方法 ==========

        private Matrix4x4 ConstructMatrixFromRows(Vector3 row1, Vector3 row2, Vector3 row3)
        {
            Matrix4x4 mat = Matrix4x4.identity;
            mat.m00 = row1.x; mat.m01 = row1.y; mat.m02 = row1.z;
            mat.m10 = row2.x; mat.m11 = row2.y; mat.m12 = row2.z;
            mat.m20 = row3.x; mat.m21 = row3.y; mat.m22 = row3.z;
            return mat;
        }

        private void PrintMatrix(string name, Matrix4x4 m)
        {
            Debug.Log($"{name}:");
            Debug.Log($"  [{m.m00:F6}, {m.m01:F6}, {m.m02:F6}]");
            Debug.Log($"  [{m.m10:F6}, {m.m11:F6}, {m.m12:F6}]");
            Debug.Log($"  [{m.m20:F6}, {m.m21:F6}, {m.m22:F6}]");
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

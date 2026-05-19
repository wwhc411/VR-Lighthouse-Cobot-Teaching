using UnityEngine;

namespace handeye
{
    /// <summary>
    /// 调试旋转矢量转换 - 查看内部四元数转换细节
    /// </summary>
    public class DebugRotationVectorConversion : MonoBehaviour
    {
        [ContextMenu("调试: -90° 转换细节")]
        public void Debug_Minus90()
        {
            Debug.Log("========== 调试 -90° 旋转矢量转换 ==========\n");

            // -90° 绕 X 轴的旋转矢量
            float angle = -90f * Mathf.Deg2Rad;
            Vector3 rv = new Vector3(angle, 0, 0);

            Debug.Log($"输入旋转矢量: ({rv.x:F4}, {rv.y:F4}, {rv.z:F4})");
            Debug.Log($"  角度: -90°\n");

            // 手动执行 RotationVectorToQuaternion 的步骤
            float theta = rv.magnitude;
            Vector3 axis = rv / theta;

            Debug.Log($"步骤1: 提取角度和轴");
            Debug.Log($"  theta = {theta:F4} rad = {theta * Mathf.Rad2Deg:F2}°");
            Debug.Log($"  axis = ({axis.x:F4}, {axis.y:F4}, {axis.z:F4})\n");

            // 规范化
            float theta_normalized = theta;
            Vector3 axis_normalized = axis;

            Debug.Log($"步骤2: 角度规范化");
            Debug.Log($"  初始: theta = {theta_normalized:F4}, axis = ({axis_normalized.x:F4}, {axis_normalized.y:F4}, {axis_normalized.z:F4})");

            while (theta_normalized < 0f)
            {
                theta_normalized = -theta_normalized;
                axis_normalized = -axis_normalized;
                Debug.Log($"  应用规范化: theta = {theta_normalized:F4}, axis = ({axis_normalized.x:F4}, {axis_normalized.y:F4}, {axis_normalized.z:F4})");
            }

            Debug.Log($"  最终: theta = {theta_normalized * Mathf.Rad2Deg:F2}°, axis = ({axis_normalized.x:F4}, {axis_normalized.y:F4}, {axis_normalized.z:F4})\n");

            // 转换为四元数
            float half = theta_normalized * 0.5f;
            float s = Mathf.Sin(half);
            Quaternion q = new Quaternion(
                axis_normalized.x * s,
                axis_normalized.y * s,
                axis_normalized.z * s,
                Mathf.Cos(half)
            );

            Debug.Log($"步骤3: 转换为四元数");
            Debug.Log($"  half_angle = {half * Mathf.Rad2Deg:F2}°");
            Debug.Log($"  sin(half) = {s:F4}");
            Debug.Log($"  cos(half) = {Mathf.Cos(half):F4}");
            Debug.Log($"  q = (w:{q.w:F4}, x:{q.x:F4}, y:{q.y:F4}, z:{q.z:F4})\n");

            // 验证这个四元数代表什么旋转
            Vector3 eulerAngles = q.eulerAngles;
            Debug.Log($"步骤4: 验证四元数");
            Debug.Log($"  Unity.eulerAngles = ({eulerAngles.x:F2}°, {eulerAngles.y:F2}°, {eulerAngles.z:F2}°)");

            // 对比:如果直接用 90° 绕 -X 轴
            Quaternion q_compare1 = Quaternion.Euler(90, 0, 0);
            Debug.Log($"\n对比: Quaternion.Euler(90°, 0, 0) = (w:{q_compare1.w:F4}, x:{q_compare1.x:F4}, y:{q_compare1.y:F4}, z:{q_compare1.z:F4})");

            Quaternion q_compare2 = Quaternion.Euler(-90, 0, 0);
            Debug.Log($"对比: Quaternion.Euler(-90°, 0, 0) = (w:{q_compare2.w:F4}, x:{q_compare2.x:F4}, y:{q_compare2.y:F4}, z:{q_compare2.z:F4})");

            // 转换为旋转矩阵查看
            Matrix4x4 m1 = Matrix4x4.Rotate(q);
            Matrix4x4 m2 = Matrix4x4.Rotate(q_compare2);

            Debug.Log($"\n旋转矩阵对比:");
            Debug.Log($"规范化后的四元数:");
            PrintMatrix(m1);
            Debug.Log($"Euler(-90°):");
            PrintMatrix(m2);

            // 测试旋转一个向量
            Vector3 testVec = Vector3.up; // (0, 1, 0)
            Vector3 rotated1 = q * testVec;
            Vector3 rotated2 = q_compare2 * testVec;

            Debug.Log($"\n旋转测试向量 (0, 1, 0):");
            Debug.Log($"  规范化四元数结果: ({rotated1.x:F4}, {rotated1.y:F4}, {rotated1.z:F4})");
            Debug.Log($"  Euler(-90°)结果: ({rotated2.x:F4}, {rotated2.y:F4}, {rotated2.z:F4})");

            if (Vector3.Distance(rotated1, rotated2) < 0.001f)
            {
                Debug.Log($"  <color=green>✓ 两者等价</color>");
            }
            else
            {
                Debug.LogError($"  <color=red>✗ 两者不等价! 差异 = {Vector3.Distance(rotated1, rotated2):F4}</color>");
            }

            Debug.Log("\n========== 调试完成 ==========");
        }

        void PrintMatrix(Matrix4x4 m)
        {
            Debug.Log($"  [{m.m00:F4}, {m.m01:F4}, {m.m02:F4}]");
            Debug.Log($"  [{m.m10:F4}, {m.m11:F4}, {m.m12:F4}]");
            Debug.Log($"  [{m.m20:F4}, {m.m21:F4}, {m.m22:F4}]");
        }

        [ContextMenu("对比: 不同表示的 -90° 和 90°")]
        public void CompareRepresentations()
        {
            Debug.Log("========== 对比不同的旋转表示 ==========\n");

            // 方法1: -90° 旋转矢量
            Vector3 rv_minus90 = new Vector3(-90f * Mathf.Deg2Rad, 0, 0);
            Debug.Log($"方法1: -90° 旋转矢量 = ({rv_minus90.x:F4}, {rv_minus90.y:F4}, {rv_minus90.z:F4})");

            // 方法2: 90° 旋转矢量（反轴）
            Vector3 rv_90_neg_axis = new Vector3(90f * Mathf.Deg2Rad, 0, 0);
            Debug.Log($"方法2: +90° 旋转矢量 = ({rv_90_neg_axis.x:F4}, {rv_90_neg_axis.y:F4}, {rv_90_neg_axis.z:F4})");

            // 方法3: Unity Euler
            Quaternion q_euler_minus90 = Quaternion.Euler(-90, 0, 0);
            Quaternion q_euler_90 = Quaternion.Euler(90, 0, 0);

            Debug.Log($"\n方法3a: Quaternion.Euler(-90°, 0, 0) = (w:{q_euler_minus90.w:F4}, x:{q_euler_minus90.x:F4}, y:{q_euler_minus90.y:F4}, z:{q_euler_minus90.z:F4})");
            Debug.Log($"方法3b: Quaternion.Euler(90°, 0, 0) = (w:{q_euler_90.w:F4}, x:{q_euler_90.x:F4}, y:{q_euler_90.y:F4}, z:{q_euler_90.z:F4})");

            // 测试它们是否等价
            Vector3 testVec = new Vector3(0, 1, 0);
            Debug.Log($"\n旋转测试向量 (0, 1, 0):");

            Vector3 result_minus90 = q_euler_minus90 * testVec;
            Vector3 result_90 = q_euler_90 * testVec;

            Debug.Log($"  -90° 结果: ({result_minus90.x:F4}, {result_minus90.y:F4}, {result_minus90.z:F4})");
            Debug.Log($"  +90° 结果: ({result_90.x:F4}, {result_90.y:F4}, {result_90.z:F4})");

            Debug.Log($"\n结论: -90° 和 +90° <color={(Vector3.Distance(result_minus90, result_90) > 0.1f ? "red>不等价" : "red>应该不等价但实际相同")}</color>");

            Debug.Log("\n========== 对比完成 ==========");
        }
    }
}

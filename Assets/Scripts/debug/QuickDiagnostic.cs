using UnityEngine;

namespace handeye
{
    /// <summary>
    /// 快速诊断工具 - 定位失败的测试
    /// </summary>
    public class QuickDiagnostic : MonoBehaviour
    {
        [ContextMenu("诊断: 测试1 - 角度规范化")]
        public void DiagnoseTest1()
        {
            Debug.Log("========== 诊断测试1: 角度规范化 ==========\n");

            // 测试关键角度
            float[] angles = { 0f, 180f, 360f, -90f };

            foreach (float angleDeg in angles)
            {
                Debug.Log($"\n--- 测试角度: {angleDeg}° ---");
                
                float angleRad = angleDeg * Mathf.Deg2Rad;
                Vector3 rv_input = new Vector3(angleRad, 0, 0);

                Debug.Log($"1. 输入旋转矢量: ({rv_input.x:F4}, {rv_input.y:F4}, {rv_input.z:F4})");

                // 使用旋转矢量输入
                SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                    Vector3.zero, rv_input, false, out Vector3 pos1, out Vector3 rv_out1
                );

                Debug.Log($"2. 旋转矢量输入 → 输出: ({rv_out1.x:F4}, {rv_out1.y:F4}, {rv_out1.z:F4})");
                Debug.Log($"   输出角度: {rv_out1.magnitude * Mathf.Rad2Deg:F2}°");

                // 正确的做法：使用相同的四元数（从旋转矢量转换得到）
                float theta = Mathf.Abs(angleRad);
                Vector3 axis = angleRad < 0 ? new Vector3(-1, 0, 0) : new Vector3(1, 0, 0);
                
                // 角度规范化
                while (theta > Mathf.PI)
                {
                    theta = 2f * Mathf.PI - theta;
                    axis = -axis;
                }
                
                // 转换为四元数
                float half = theta * 0.5f;
                float s = Mathf.Sin(half);
                Quaternion q_input = new Quaternion(axis.x * s, axis.y * s, axis.z * s, Mathf.Cos(half));
                
                Debug.Log($"3. 规范化后角度: {theta * Mathf.Rad2Deg:F2}°");
                Debug.Log($"   规范化后轴: ({axis.x:F2}, {axis.y:F2}, {axis.z:F2})");
                Debug.Log($"   对应四元数: (w:{q_input.w:F4}, x:{q_input.x:F4}, y:{q_input.y:F4}, z:{q_input.z:F4})");

                SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                    Vector3.zero, q_input, false, out Vector3 pos2, out Vector3 rv_out2
                );

                Debug.Log($"4. 四元数直接输入 → 输出: ({rv_out2.x:F4}, {rv_out2.y:F4}, {rv_out2.z:F4})");
                Debug.Log($"   输出角度: {rv_out2.magnitude * Mathf.Rad2Deg:F2}°");

                // 对比
                Vector3 diff = rv_out1 - rv_out2;
                float error = diff.magnitude * Mathf.Rad2Deg;

                Debug.Log($"5. 两种方法差异: ({diff.x:F4}, {diff.y:F4}, {diff.z:F4})");
                Debug.Log($"   误差大小: {error:F4}°");

                if (error < 0.5f)
                {
                    Debug.Log($"   <color=green>✓ 通过 (误差 < 0.5°)</color>");
                }
                else
                {
                    Debug.LogError($"   <color=red>✗ 失败 (误差 {error:F2}° > 0.5°)</color>");
                }
            }

            Debug.Log("\n========== 诊断完成 ==========");
        }

        [ContextMenu("诊断: 测试2 - 四元数符号")]
        public void DiagnoseTest2()
        {
            Debug.Log("========== 诊断测试2: 四元数符号规范化 ==========\n");

            Quaternion q1 = Quaternion.Euler(120, 45, 30);
            Quaternion q2 = new Quaternion(-q1.x, -q1.y, -q1.z, -q1.w);

            Debug.Log($"q1: (w:{q1.w:F4}, x:{q1.x:F4}, y:{q1.y:F4}, z:{q1.z:F4})");
            Debug.Log($"q2: (w:{q2.w:F4}, x:{q2.x:F4}, y:{q2.y:F4}, z:{q2.z:F4})");
            Debug.Log($"说明: q2 = -q1，两者应该表示相同旋转\n");

            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                Vector3.zero, q1, false, out _, out Vector3 rv1
            );
            Debug.Log($"q1 输出: ({rv1.x:F4}, {rv1.y:F4}, {rv1.z:F4})");

            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                Vector3.zero, q2, false, out _, out Vector3 rv2
            );
            Debug.Log($"q2 输出: ({rv2.x:F4}, {rv2.y:F4}, {rv2.z:F4})");

            float diff = (rv1 - rv2).magnitude * Mathf.Rad2Deg;
            Debug.Log($"\n差异: {diff:F6}°");

            if (diff < 0.01f)
            {
                Debug.Log("<color=green>✓ 测试2通过</color>");
            }
            else
            {
                Debug.LogError($"<color=red>✗ 测试2失败 (差异 {diff:F4}°)</color>");
            }

            Debug.Log("\n========== 诊断完成 ==========");
        }

        [ContextMenu("诊断: 简单往返测试")]
        public void SimpleRoundTripTest()
        {
            Debug.Log("========== 简单往返测试 ==========\n");

            float[] angles = { 10f, 90f, 180f, 360f };

            foreach (float angleDeg in angles)
            {
                Debug.Log($"\n--- 测试: {angleDeg}° ---");

                // 方法1: 使用旋转矢量
                float angleRad = angleDeg * Mathf.Deg2Rad;
                Vector3 rv = new Vector3(angleRad, 0, 0);

                Debug.Log($"输入旋转矢量: {angleDeg}° = {angleRad:F4} rad");

                SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                    Vector3.zero, rv, false, out _, out Vector3 rvOut
                );

                float outputAngle = rvOut.magnitude * Mathf.Rad2Deg;
                Debug.Log($"输出: 角度 = {outputAngle:F2}°");

                // 检查是否合理（考虑手眼标定的影响）
                Debug.Log($"  旋转矢量: ({rvOut.x:F4}, {rvOut.y:F4}, {rvOut.z:F4})");

                // 方法2: 使用四元数
                Quaternion q = Quaternion.Euler(angleDeg, 0, 0);
                SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                    Vector3.zero, q, false, out _, out Vector3 rvOut2
                );

                float diff = (rvOut - rvOut2).magnitude * Mathf.Rad2Deg;
                Debug.Log($"与四元数输入的差异: {diff:F4}°");

                if (diff < 0.5f)
                {
                    Debug.Log("<color=green>✓</color>");
                }
                else
                {
                    Debug.LogError($"<color=red>✗ 差异过大</color>");
                }
            }

            Debug.Log("\n========== 测试完成 ==========");
        }

        [ContextMenu("诊断: 检查基准旋转")]
        public void CheckBaselineRotation()
        {
            Debug.Log("========== 检查基准旋转 ==========\n");

            Debug.Log("测试Identity和0°是否产生相同结果\n");

            // Identity四元数
            Quaternion qIdentity = Quaternion.identity;
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                Vector3.zero, qIdentity, false, out _, out Vector3 rvIdentity
            );
            Debug.Log($"Identity四元数 → 输出: ({rvIdentity.x:F4}, {rvIdentity.y:F4}, {rvIdentity.z:F4})");
            Debug.Log($"  角度: {rvIdentity.magnitude * Mathf.Rad2Deg:F2}°");

            // 0° 旋转矢量
            Vector3 rv0 = Vector3.zero;
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                Vector3.zero, rv0, false, out _, out Vector3 rvOut0
            );
            Debug.Log($"\n0° 旋转矢量 → 输出: ({rvOut0.x:F4}, {rvOut0.y:F4}, {rvOut0.z:F4})");
            Debug.Log($"  角度: {rvOut0.magnitude * Mathf.Rad2Deg:F2}°");

            // 360° 旋转矢量
            float angle360 = 360f * Mathf.Deg2Rad;
            Vector3 rv360 = new Vector3(angle360, 0, 0);
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                Vector3.zero, rv360, false, out _, out Vector3 rvOut360
            );
            Debug.Log($"\n360° 旋转矢量 → 输出: ({rvOut360.x:F4}, {rvOut360.y:F4}, {rvOut360.z:F4})");
            Debug.Log($"  角度: {rvOut360.magnitude * Mathf.Rad2Deg:F2}°");

            // 对比
            float diff1 = (rvIdentity - rvOut0).magnitude * Mathf.Rad2Deg;
            float diff2 = (rvIdentity - rvOut360).magnitude * Mathf.Rad2Deg;

            Debug.Log($"\nIdentity vs 0°: 差异 {diff1:F4}°");
            Debug.Log($"Identity vs 360°: 差异 {diff2:F4}°");

            if (diff1 < 0.01f && diff2 < 0.5f)
            {
                Debug.Log("\n<color=green>✓ 基准旋转正常</color>");
            }
            else
            {
                Debug.LogError("\n<color=red>✗ 基准旋转有问题</color>");
            }

            Debug.Log("\n========== 检查完成 ==========");
        }
    }
}

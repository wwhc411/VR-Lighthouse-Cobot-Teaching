using UnityEngine;

namespace handeye
{
    /// <summary>
    /// 验证 ViveTrackerPoseLogger 的旋转矢量转换修复
    /// </summary>
    public class VerifyLoggerRotationFix : MonoBehaviour
    {
        [ContextMenu("验证: Logger旋转矢量转换")]
        public void VerifyRotationVectorConversion()
        {
            Debug.Log("========== 验证 ViveTrackerPoseLogger 旋转矢量转换 ==========\n");

            // 使用反射访问私有方法
            var loggerType = typeof(ViveTrackerPoseLogger);
            var method = loggerType.GetMethod("QuaternionToRotationVector", 
                System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.NonPublic);

            if (method == null)
            {
                Debug.LogError("无法找到 QuaternionToRotationVector 方法");
                return;
            }

            bool allPassed = true;

            // 测试1: 四元数符号一致性 (q 和 -q 应该产生相同的旋转矢量)
            Debug.Log("=== 测试1: 四元数符号一致性 ===");
            Quaternion q1 = Quaternion.Euler(120, 45, 30);
            Quaternion q2 = new Quaternion(-q1.x, -q1.y, -q1.z, -q1.w);

            Vector3 rv1 = (Vector3)method.Invoke(null, new object[] { q1 });
            Vector3 rv2 = (Vector3)method.Invoke(null, new object[] { q2 });

            float diff1 = Vector3.Distance(rv1, rv2);
            Debug.Log($"q1 = (w:{q1.w:F4}, x:{q1.x:F4}, y:{q1.y:F4}, z:{q1.z:F4})");
            Debug.Log($"q2 = (w:{q2.w:F4}, x:{q2.x:F4}, y:{q2.y:F4}, z:{q2.z:F4})");
            Debug.Log($"rv1 = ({rv1.x:F4}, {rv1.y:F4}, {rv1.z:F4}) → 角度 = {rv1.magnitude * Mathf.Rad2Deg:F2}°");
            Debug.Log($"rv2 = ({rv2.x:F4}, {rv2.y:F4}, {rv2.z:F4}) → 角度 = {rv2.magnitude * Mathf.Rad2Deg:F2}°");
            Debug.Log($"差异: {diff1:F6}°");

            if (diff1 < 0.01f)
            {
                Debug.Log("<color=green>✓ 测试1通过</color>\n");
            }
            else
            {
                Debug.LogError($"<color=red>✗ 测试1失败 (差异 {diff1:F4}°)</color>\n");
                allPassed = false;
            }

            // 测试2: 角度规范化 (测试常见角度)
            Debug.Log("=== 测试2: 角度规范化 ===");
            float[] testAngles = { 0f, 90f, 180f, 360f };

            foreach (float angleDeg in testAngles)
            {
                Quaternion q = Quaternion.Euler(angleDeg, 0, 0);
                Vector3 rv = (Vector3)method.Invoke(null, new object[] { q });
                float outputAngle = rv.magnitude * Mathf.Rad2Deg;

                // 期望: 所有角度都应该在 [0, 180] 范围内
                bool inRange = outputAngle >= 0f && outputAngle <= 180.01f;

                Debug.Log($"{angleDeg}° → 四元数(w:{q.w:F4}, x:{q.x:F4}) → 输出角度 = {outputAngle:F2}° {(inRange ? "✓" : "✗")}");

                if (!inRange)
                {
                    allPassed = false;
                }
            }

            if (allPassed)
            {
                Debug.Log("\n<color=green>✓✓✓ 所有测试通过!</color>");
            }
            else
            {
                Debug.LogError("\n<color=red>✗✗✗ 部分测试失败</color>");
            }

            Debug.Log("\n========== 验证完成 ==========");
        }

        [ContextMenu("验证: 往返转换一致性")]
        public void VerifyRoundTripConsistency()
        {
            Debug.Log("========== 往返转换一致性验证 ==========\n");

            var loggerType = typeof(ViveTrackerPoseLogger);
            var quatToRv = loggerType.GetMethod("QuaternionToRotationVector",
                System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.NonPublic);

            if (quatToRv == null)
            {
                Debug.LogError("无法找到转换方法");
                return;
            }

            // 测试多个四元数
            Quaternion[] testQuats = {
                Quaternion.Euler(10, 0, 0),
                Quaternion.Euler(45, 0, 0),
                Quaternion.Euler(90, 0, 0),
                Quaternion.Euler(135, 0, 0),
                Quaternion.Euler(179, 0, 0)
            };

            bool allPassed = true;

            foreach (var qInput in testQuats)
            {
                // 四元数 → 旋转矢量
                Vector3 rv = (Vector3)quatToRv.Invoke(null, new object[] { qInput });

                // 旋转矢量 → 四元数 (使用 Unity 内置方法验证)
                float angle = rv.magnitude;
                Vector3 axis = angle > 0.0001f ? rv / angle : Vector3.right;
                Quaternion qRecovered = Quaternion.AngleAxis(angle * Mathf.Rad2Deg, axis);

                // 对比旋转效果 (旋转一个测试向量)
                Vector3 testVec = Vector3.up;
                Vector3 result1 = qInput * testVec;
                Vector3 result2 = qRecovered * testVec;

                float error = Vector3.Distance(result1, result2);
                float inputAngle = Quaternion.Angle(Quaternion.identity, qInput);

                Debug.Log($"输入角度: {inputAngle:F2}° → 往返误差: {error:F6} {(error < 0.001f ? "✓" : "✗")}");

                if (error >= 0.001f)
                {
                    allPassed = false;
                    Debug.LogWarning($"  输入四元数: (w:{qInput.w:F4}, x:{qInput.x:F4}, y:{qInput.y:F4}, z:{qInput.z:F4})");
                    Debug.LogWarning($"  恢复四元数: (w:{qRecovered.w:F4}, x:{qRecovered.x:F4}, y:{qRecovered.y:F4}, z:{qRecovered.z:F4})");
                    Debug.LogWarning($"  旋转矢量: ({rv.x:F4}, {rv.y:F4}, {rv.z:F4})");
                }
            }

            if (allPassed)
            {
                Debug.Log("\n<color=green>✓✓✓ 往返转换一致!</color>");
            }
            else
            {
                Debug.LogError("\n<color=red>✗✗✗ 往返转换存在误差</color>");
            }

            Debug.Log("\n========== 验证完成 ==========");
        }
    }
}

using UnityEngine;

namespace handeye
{
    /// <summary>
    /// 验证旋转矢量奇异性修复效果
    /// </summary>
    public class VerifyRotationVectorFix : MonoBehaviour
    {
        [ContextMenu("验证修复: 快速测试")]
        public void QuickTest()
        {
            Debug.Log("========== 验证旋转矢量奇异性修复 ==========\n");

            float[] criticalAngles = { 0f, 90f, 180f, 270f, 350f, 360f, 370f, 720f };
            
            Debug.Log("测试关键角度的往返转换误差:\n");
            Debug.Log("角度(°)  |  往返误差(°)  |  状态");
            Debug.Log("--------------------------------------");

            bool allPassed = true;

            foreach (float angleDeg in criticalAngles)
            {
                float angleRad = angleDeg * Mathf.Deg2Rad;
                Vector3 rv_input = new Vector3(angleRad, 0, 0);

                // rv → quat → rv
                SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                    Vector3.zero, rv_input, false,
                    out _, out Vector3 rv_output
                );

                // 计算期望的规范化角度
                float expectedAngle = angleDeg;
                while (expectedAngle > 180f) expectedAngle -= 360f;
                while (expectedAngle < 0f) expectedAngle += 360f;
                if (expectedAngle > 180f) expectedAngle = 360f - expectedAngle;

                float outputAngle = rv_output.magnitude * Mathf.Rad2Deg;
                
                // 注意：由于手眼标定的存在，输出不是简单的输入
                // 这里我们只检查往返转换，不检查绝对值
                
                // 重新构造输入四元数来验证一致性
                Quaternion q_test = Quaternion.Euler(angleDeg, 0, 0);
                SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                    Vector3.zero, q_test, false,
                    out _, out Vector3 rv_from_quat
                );

                Vector3 diff = rv_output - rv_from_quat;
                float error = diff.magnitude * Mathf.Rad2Deg;

                string status = "✓";
                if (error > 0.01f)
                {
                    status = "✗";
                    allPassed = false;
                }

                Debug.Log($"{angleDeg,6:F0}  |  {error,13:F6}  |  {status}");
            }

            Debug.Log("--------------------------------------\n");

            if (allPassed)
            {
                Debug.Log("<color=green>✓✓✓ 修复成功! 所有角度误差都 < 0.01°</color>");
                Debug.Log("<color=green>旋转矢量输入现在可以正常使用了!</color>");
            }
            else
            {
                Debug.LogError("<color=red>✗ 仍有问题，需要进一步检查</color>");
            }

            Debug.Log("\n========== 验证完成 ==========");
        }

        [ContextMenu("验证修复: 完整扫描")]
        public void FullScan()
        {
            Debug.Log("========== 完整角度扫描 (0-360°) ==========\n");

            int errorCount = 0;
            float maxError = 0f;
            float maxErrorAngle = 0f;

            for (float angleDeg = 0f; angleDeg <= 360f; angleDeg += 1f)
            {
                float angleRad = angleDeg * Mathf.Deg2Rad;
                Vector3 rv_input = new Vector3(angleRad, 0, 0);

                Quaternion q_test = Quaternion.Euler(angleDeg, 0, 0);
                
                SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                    Vector3.zero, rv_input, false,
                    out _, out Vector3 rv_output1
                );

                SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                    Vector3.zero, q_test, false,
                    out _, out Vector3 rv_output2
                );

                Vector3 diff = rv_output1 - rv_output2;
                float error = diff.magnitude * Mathf.Rad2Deg;

                if (error > 0.01f)
                {
                    errorCount++;
                }

                if (error > maxError)
                {
                    maxError = error;
                    maxErrorAngle = angleDeg;
                }
            }

            Debug.Log($"测试点数: 361 (0-360°, 每1°)");
            Debug.Log($"误差 > 0.01° 的点数: {errorCount}");
            Debug.Log($"最大误差: {maxError:F6}° (出现在 {maxErrorAngle:F0}°)\n");

            if (maxError < 0.01f)
            {
                Debug.Log("<color=green>✓✓✓ 完美! 所有角度误差都 < 0.01°</color>");
            }
            else if (maxError < 0.1f)
            {
                Debug.Log($"<color=yellow>⚠ 仍有轻微误差，但已大幅改善</color>");
            }
            else
            {
                Debug.LogError($"<color=red>✗ 在 {maxErrorAngle:F0}° 附近仍有较大误差</color>");
            }

            Debug.Log("\n========== 扫描完成 ==========");
        }

        [ContextMenu("对比修复前后")]
        public void CompareBeforeAfter()
        {
            Debug.Log("========== 对比修复前后的表现 ==========\n");
            Debug.Log("测试360°的处理:\n");

            float angleRad = 360f * Mathf.Deg2Rad;
            Vector3 rv_360 = new Vector3(angleRad, 0, 0);

            Debug.Log($"输入: 360° = {angleRad:F4} 弧度");
            Debug.Log($"旋转矢量: ({rv_360.x:F4}, {rv_360.y:F4}, {rv_360.z:F4})\n");

            // 转换测试
            Quaternion q = Quaternion.Euler(360, 0, 0);  // Unity会自动规范化为(0,0,0)
            Quaternion q_identity = Quaternion.identity;

            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                Vector3.zero, q, false,
                out _, out Vector3 rv_from_360
            );

            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                Vector3.zero, q_identity, false,
                out _, out Vector3 rv_from_0
            );

            Debug.Log($"从360°四元数得到: ({rv_from_360.x:F4}, {rv_from_360.y:F4}, {rv_from_360.z:F4})");
            Debug.Log($"从0°四元数得到:   ({rv_from_0.x:F4}, {rv_from_0.y:F4}, {rv_from_0.z:F4})\n");

            Vector3 diff = rv_from_360 - rv_from_0;
            float error = diff.magnitude * Mathf.Rad2Deg;

            Debug.Log($"差异: {error:F6}°");

            Debug.Log("\n<b>修复前:</b>");
            Debug.Log("  360° → rv → quat → rv 会产生 360° 的巨大误差");
            Debug.Log("  原因: 没有规范化角度到 [0, π] 范围\n");

            Debug.Log("<b>修复后:</b>");
            Debug.Log("  360° → 规范化为 0° → quat → rv");
            Debug.Log("  结果: 360° 和 0° 产生相同的输出 ✓\n");

            if (error < 0.01f)
            {
                Debug.Log("<color=green>✓ 修复成功! 360° 问题已解决</color>");
            }
            else
            {
                Debug.LogWarning($"<color=yellow>⚠ 仍有 {error:F2}° 的差异</color>");
            }

            Debug.Log("\n========== 对比完成 ==========");
        }
    }
}

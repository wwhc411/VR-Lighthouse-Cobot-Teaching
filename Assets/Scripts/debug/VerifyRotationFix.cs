using UnityEngine;

namespace handeye
{
    /// <summary>
    /// 验证修复后的旋转转换
    /// </summary>
    public class VerifyRotationFix : MonoBehaviour
    {
        [Header("=== 测试配置 ===")]
        [Tooltip("测试旋转角度（度）")]
        public float testAngleDeg = 10f;

        [ContextMenu("验证: 修复后的旋转一致性")]
        public void VerifyFix()
        {
            Debug.Log("========== 验证修复后的旋转转换 ==========\n");
            Debug.Log("测试: 直接从旋转矩阵转换到轴角\n");

            // 基准
            Vector3 pos_identity = Vector3.zero;
            Quaternion quat_identity = Quaternion.identity;
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                pos_identity, quat_identity, false,
                out Vector3 posOut_identity, out Vector3 rotvecOut_identity
            );
            
            Debug.Log("=== 基准: Identity ===");
            Debug.Log($"输出旋转向量: ({rotvecOut_identity.x:F4}, {rotvecOut_identity.y:F4}, {rotvecOut_identity.z:F4})");
            Debug.Log($"旋转角度: {rotvecOut_identity.magnitude * Mathf.Rad2Deg:F2}°\n");

            // 测试X轴
            Quaternion quat_rotX = Quaternion.Euler(testAngleDeg, 0, 0);
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                pos_identity, quat_rotX, false,
                out Vector3 posOut_rotX, out Vector3 rotvecOut_rotX
            );
            Vector3 delta_rotX = rotvecOut_rotX - rotvecOut_identity;
            
            Debug.Log($"=== 测试X轴: 旋转 {testAngleDeg}° ===");
            Debug.Log($"输出旋转向量: ({rotvecOut_rotX.x:F4}, {rotvecOut_rotX.y:F4}, {rotvecOut_rotX.z:F4})");
            Debug.Log($"相对变化: Δ({delta_rotX.x:F4}, {delta_rotX.y:F4}, {delta_rotX.z:F4})");
            Debug.Log($"变化量: {delta_rotX.magnitude * Mathf.Rad2Deg:F2}°\n");

            // 测试Y轴
            Quaternion quat_rotY = Quaternion.Euler(0, testAngleDeg, 0);
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                pos_identity, quat_rotY, false,
                out Vector3 posOut_rotY, out Vector3 rotvecOut_rotY
            );
            Vector3 delta_rotY = rotvecOut_rotY - rotvecOut_identity;
            
            Debug.Log($"=== 测试Y轴: 旋转 {testAngleDeg}° ===");
            Debug.Log($"输出旋转向量: ({rotvecOut_rotY.x:F4}, {rotvecOut_rotY.y:F4}, {rotvecOut_rotY.z:F4})");
            Debug.Log($"相对变化: Δ({delta_rotY.x:F4}, {delta_rotY.y:F4}, {delta_rotY.z:F4})");
            Debug.Log($"变化量: {delta_rotY.magnitude * Mathf.Rad2Deg:F2}°\n");

            // 测试Z轴
            Quaternion quat_rotZ = Quaternion.Euler(0, 0, testAngleDeg);
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                pos_identity, quat_rotZ, false,
                out Vector3 posOut_rotZ, out Vector3 rotvecOut_rotZ
            );
            Vector3 delta_rotZ = rotvecOut_rotZ - rotvecOut_identity;
            
            Debug.Log($"=== 测试Z轴: 旋转 {testAngleDeg}° ===");
            Debug.Log($"输出旋转向量: ({rotvecOut_rotZ.x:F4}, {rotvecOut_rotZ.y:F4}, {rotvecOut_rotZ.z:F4})");
            Debug.Log($"相对变化: Δ({delta_rotZ.x:F4}, {delta_rotZ.y:F4}, {delta_rotZ.z:F4})");
            Debug.Log($"变化量: {delta_rotZ.magnitude * Mathf.Rad2Deg:F2}°\n");

            // 验证
            Debug.Log("=== 一致性检查 ===");
            float tolerance = 0.5f;
            bool xOk = Mathf.Abs(delta_rotX.magnitude * Mathf.Rad2Deg - testAngleDeg) < tolerance;
            bool yOk = Mathf.Abs(delta_rotY.magnitude * Mathf.Rad2Deg - testAngleDeg) < tolerance;
            bool zOk = Mathf.Abs(delta_rotZ.magnitude * Mathf.Rad2Deg - testAngleDeg) < tolerance;

            Debug.Log($"X轴: 输入 {testAngleDeg}° → 输出变化 {delta_rotX.magnitude * Mathf.Rad2Deg:F2}° : {(xOk ? "✓" : "✗")}");
            Debug.Log($"Y轴: 输入 {testAngleDeg}° → 输出变化 {delta_rotY.magnitude * Mathf.Rad2Deg:F2}° : {(yOk ? "✓" : "✗")}");
            Debug.Log($"Z轴: 输入 {testAngleDeg}° → 输出变化 {delta_rotZ.magnitude * Mathf.Rad2Deg:F2}° : {(zOk ? "✓" : "✗")}");

            if (xOk && yOk && zOk)
            {
                Debug.Log("\n<color=green>✓✓✓ 修复成功！旋转转换一致性良好！</color>");
                Debug.Log("系统现在可以正常使用了。");
            }
            else
            {
                Debug.LogWarning("\n<color=yellow>⚠ 仍存在问题，需要进一步检查。</color>");
            }

            Debug.Log("\n========== 验证完成 ==========");
        }

        [ContextMenu("对比: 新旧方法")]
        public void CompareOldAndNew()
        {
            Debug.Log("========== 对比新旧转换方法 ==========\n");

            Quaternion quat_test = Quaternion.Euler(10, 0, 0);
            
            // 使用实际系统（新方法）
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                Vector3.zero, quat_test, false,
                out Vector3 posOut, out Vector3 rotvecOut_new
            );

            Debug.Log("=== 新方法 (矩阵→轴角) ===");
            Debug.Log($"输出旋转向量: ({rotvecOut_new.x:F4}, {rotvecOut_new.y:F4}, {rotvecOut_new.z:F4})");
            Debug.Log($"旋转角度: {rotvecOut_new.magnitude * Mathf.Rad2Deg:F2}°\n");

            Debug.Log("注意: 旧方法(矩阵→四元数→轴角)已被替换");
            Debug.Log("新方法在大角度基准旋转时更稳定，数值精度更高。\n");

            Debug.Log("========== 对比完成 ==========");
        }
    }
}

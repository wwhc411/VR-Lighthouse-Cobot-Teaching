using UnityEngine;

namespace handeye
{
    /// <summary>
    /// SteamVR坐标转换器配置组件
    /// 提供Inspector可编辑的Kabsch刚性对齐参数
    /// 
    /// 功能:
    /// - 在Inspector中配置Kabsch点云刚性对齐的R和t参数
    /// - 支持从KabschAlignment组件自动导入变换矩阵
    /// - 可选启用/禁用Kabsch校正
    /// - 缓存变换矩阵以提高性能
    /// 
    /// 使用方法:
    /// 1. 将此组件添加到场景中的GameObject
    /// 2. 运行Kabsch对齐后，右键菜单 → "从KabschAlignment组件导入"
    /// 3. 或手动在Inspector中输入R（3x3）和t（3x1）
    /// 4. 勾选 enableKabschAlignment 启用校正
    /// 
    /// 相关文档: 坐标转换器集成Kabsch刚性对齐功能设计文档.md
    /// </summary>
    [AddComponentMenu("Calibration/SteamVR UR Coordinate Converter Config")]
    public class SteamVrUrCoordinateConverterConfig : MonoBehaviour
    {
        [Header("Kabsch点云刚性对齐")]
        [Tooltip("启用Kabsch刚性对齐校正")]
        public bool enableKabschAlignment = false;

        [Space(10)]
        [Header("Kabsch旋转矩阵 R (3x3)")]
        [Tooltip("第1行: R[0,0] R[0,1] R[0,2]")]
        public Vector3 kabschRotationRow0 = new Vector3(1, 0, 0);
        
        [Tooltip("第2行: R[1,0] R[1,1] R[1,2]")]
        public Vector3 kabschRotationRow1 = new Vector3(0, 1, 0);
        
        [Tooltip("第3行: R[2,0] R[2,1] R[2,2]")]
        public Vector3 kabschRotationRow2 = new Vector3(0, 0, 1);

        [Space(10)]
        [Header("Kabsch平移向量 t (3x1)")]
        [Tooltip("单位：米（m）")]
        public Vector3 kabschTranslation = Vector3.zero;

        [Space(10)]
        [Header("调试信息")]
        [Tooltip("显示Kabsch变换的详细日志")]
        public bool showKabschDebugLog = false;

        private Matrix4x4 cachedKabschTransform;
        private bool transformCacheDirty = true;

        private void OnValidate()
        {
            // Inspector中参数修改时，标记缓存失效
            transformCacheDirty = true;
            
            // 自动同步到静态转换器
            SteamVrUrCoordinateConverter.SetKabschConfig(this);
        }

        private void Awake()
        {
            // 注册到静态转换器
            SteamVrUrCoordinateConverter.SetKabschConfig(this);
            
            if (enableKabschAlignment)
            {
                Debug.Log($"[Kabsch配置] 已启用刚性对齐校正");
            }
        }

        /// <summary>
        /// 获取Kabsch刚性对齐的4x4齐次变换矩阵
        /// </summary>
        public Matrix4x4 GetKabschTransform()
        {
            if (transformCacheDirty)
            {
                BuildKabschTransformMatrix();
                transformCacheDirty = false;
            }
            return cachedKabschTransform;
        }

        /// <summary>
        /// 构建Kabsch变换矩阵
        /// </summary>
        private void BuildKabschTransformMatrix()
        {
            // 按行构建旋转矩阵（Unity Matrix4x4按列存储）
            cachedKabschTransform = new Matrix4x4();
            
            // 列0 (r00, r10, r20)
            cachedKabschTransform.m00 = kabschRotationRow0.x;
            cachedKabschTransform.m10 = kabschRotationRow1.x;
            cachedKabschTransform.m20 = kabschRotationRow2.x;
            
            // 列1 (r01, r11, r21)
            cachedKabschTransform.m01 = kabschRotationRow0.y;
            cachedKabschTransform.m11 = kabschRotationRow1.y;
            cachedKabschTransform.m21 = kabschRotationRow2.y;
            
            // 列2 (r02, r12, r22)
            cachedKabschTransform.m02 = kabschRotationRow0.z;
            cachedKabschTransform.m12 = kabschRotationRow1.z;
            cachedKabschTransform.m22 = kabschRotationRow2.z;
            
            // 平移向量 (列3)
            cachedKabschTransform.m03 = kabschTranslation.x;
            cachedKabschTransform.m13 = kabschTranslation.y;
            cachedKabschTransform.m23 = kabschTranslation.z;
            
            // 齐次坐标
            cachedKabschTransform.m33 = 1f;

            if (showKabschDebugLog)
            {
                Debug.Log($"[Kabsch配置] 变换矩阵已重建:\n{cachedKabschTransform}");
            }
        }

        /// <summary>
        /// 应用Kabsch刚性对齐到输入点
        /// </summary>
        /// <param name="point">输入点（米）</param>
        /// <returns>变换后的点（米）</returns>
        public Vector3 ApplyKabschAlignment(Vector3 point)
        {
            if (!enableKabschAlignment)
                return point;

            Vector3 transformed = GetKabschTransform().MultiplyPoint3x4(point);

            if (showKabschDebugLog)
            {
                Debug.Log($"[Kabsch对齐] 输入: ({point.x:F6}, {point.y:F6}, {point.z:F6}) → 输出: ({transformed.x:F6}, {transformed.y:F6}, {transformed.z:F6})");
            }

            return transformed;
        }

        /// <summary>
        /// 从KabschAlignment组件自动填充参数
        /// </summary>
        [ContextMenu("从KabschAlignment组件导入")]
        public void ImportFromKabschAlignment()
        {
            var kabsch = FindObjectOfType<KabschAlignment>();
            if (kabsch == null)
            {
                Debug.LogError("[Kabsch配置] 场景中未找到KabschAlignment组件");
                return;
            }

            if (!kabsch.IsAlignmentComputed)
            {
                Debug.LogError("[Kabsch配置] KabschAlignment未完成计算，请先执行对齐");
                return;
            }

            // 获取Unity格式的变换矩阵
            Matrix4x4 transform = kabsch.GetTransformMatrix();
            
            // 填充旋转矩阵（按行）
            kabschRotationRow0 = new Vector3(transform.m00, transform.m01, transform.m02);
            kabschRotationRow1 = new Vector3(transform.m10, transform.m11, transform.m12);
            kabschRotationRow2 = new Vector3(transform.m20, transform.m21, transform.m22);
            
            // 填充平移向量
            kabschTranslation = new Vector3(transform.m03, transform.m13, transform.m23);

            transformCacheDirty = true;
            
            Debug.Log($"[Kabsch配置] 已从KabschAlignment导入参数 (RMSE: {kabsch.RMSE:F6}m)");
            Debug.Log($"  旋转矩阵 R:\n" +
                      $"    [{kabschRotationRow0.x:F6}, {kabschRotationRow0.y:F6}, {kabschRotationRow0.z:F6}]\n" +
                      $"    [{kabschRotationRow1.x:F6}, {kabschRotationRow1.y:F6}, {kabschRotationRow1.z:F6}]\n" +
                      $"    [{kabschRotationRow2.x:F6}, {kabschRotationRow2.y:F6}, {kabschRotationRow2.z:F6}]");
            Debug.Log($"  平移向量 t: [{kabschTranslation.x:F6}, {kabschTranslation.y:F6}, {kabschTranslation.z:F6}]");
        }

        /// <summary>
        /// 手动输入矩阵数据（用于复制粘贴）
        /// </summary>
        [ContextMenu("手动输入矩阵（查看Console提示）")]
        public void ShowManualInputInstructions()
        {
            Debug.Log("=== Kabsch矩阵手动输入指南 ===\n" +
                      "1. 在Inspector中展开 'Kabsch旋转矩阵 R'\n" +
                      "2. 按行输入旋转矩阵（3x3）:\n" +
                      "   Row 0: [R00, R01, R02]\n" +
                      "   Row 1: [R10, R11, R12]\n" +
                      "   Row 2: [R20, R21, R22]\n" +
                      "3. 在 'Kabsch平移向量 t' 中输入平移 [tx, ty, tz]\n" +
                      "4. 勾选 'Enable Kabsch Alignment' 启用\n" +
                      "5. 单位必须为米（m）\n\n" +
                      "注意: 矩阵必须是有效的刚性变换（旋转+平移）");
        }

        /// <summary>
        /// 验证旋转矩阵正交性
        /// </summary>
        [ContextMenu("验证旋转矩阵正交性")]
        public void ValidateRotationMatrix()
        {
            Matrix4x4 R = GetKabschTransform();
            
            // 计算 R * R^T 应该接近单位矩阵
            Matrix4x4 RRT = new Matrix4x4();
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    float sum = 0;
                    for (int k = 0; k < 3; k++)
                    {
                        sum += R[i, k] * R[j, k];
                    }
                    RRT[i, j] = sum;
                }
            }
            
            // 检查对角线是否接近1，非对角线是否接近0
            bool isOrthogonal = true;
            float maxError = 0f;
            
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    float expected = (i == j) ? 1f : 0f;
                    float error = Mathf.Abs(RRT[i, j] - expected);
                    maxError = Mathf.Max(maxError, error);
                    
                    if (error > 0.001f)
                    {
                        isOrthogonal = false;
                    }
                }
            }
            
            if (isOrthogonal)
            {
                Debug.Log($"[Kabsch配置] ✓ 旋转矩阵正交性验证通过 (最大误差: {maxError:E4})");
            }
            else
            {
                Debug.LogWarning($"[Kabsch配置] ✗ 旋转矩阵非正交! (最大误差: {maxError:E4})\n" +
                                "请检查输入的矩阵是否为有效的旋转矩阵");
            }
            
            Debug.Log($"R * R^T =\n{RRT}");
        }

        private void OnDrawGizmosSelected()
        {
            if (!enableKabschAlignment)
                return;

            // 可视化Kabsch变换的坐标系
            Vector3 origin = transform.position;
            
            // 绘制变换后的坐标轴
            Matrix4x4 kabschMat = GetKabschTransform();
            Vector3 xAxis = kabschMat.MultiplyVector(Vector3.right) * 0.2f;
            Vector3 yAxis = kabschMat.MultiplyVector(Vector3.up) * 0.2f;
            Vector3 zAxis = kabschMat.MultiplyVector(Vector3.forward) * 0.2f;

            Gizmos.color = Color.red;
            Gizmos.DrawRay(origin, xAxis);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(origin, yAxis);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(origin, zAxis);
            
            // 绘制平移向量
            if (kabschTranslation.magnitude > 0.001f)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawLine(origin, origin + kabschTranslation);
                Gizmos.DrawWireSphere(origin + kabschTranslation, 0.05f);
            }
        }
    }
}

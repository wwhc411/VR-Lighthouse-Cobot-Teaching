using UnityEngine;

namespace handeye
{
    /// <summary>
    /// Kabsch刚性对齐使用示例
    /// 
    /// 演示如何在场景中配置和使用Kabsch点云刚性对齐功能
    /// 
    /// 使用步骤:
    /// 1. 将此脚本添加到场景中的GameObject
    /// 2. 在Inspector中设置必要的引用
    /// 3. 运行Kabsch对齐
    /// 4. 导入参数并启用校正
    /// 5. 验证转换结果
    /// </summary>
    public class KabschAlignmentExample : MonoBehaviour
    {
        [Header("引用")]
        [Tooltip("Kabsch对齐组件")]
        public KabschAlignment kabschAlignment;
        
        [Tooltip("坐标转换器配置组件")]
        public SteamVrUrCoordinateConverterConfig converterConfig;
        
        [Header("测试数据")]
        [Tooltip("测试用的SteamVR位置（米）")]
        public Vector3 testTrackerPosition = new Vector3(0.5f, 1.0f, 0.5f);
        
        [Tooltip("测试用的SteamVR姿态（四元数）")]
        public Quaternion testTrackerRotation = Quaternion.identity;
        
        [Header("结果显示")]
        [Tooltip("未启用Kabsch时的输出位置")]
        public Vector3 resultWithoutKabsch;
        
        [Tooltip("启用Kabsch后的输出位置")]
        public Vector3 resultWithKabsch;
        
        [Tooltip("位置差异（米）")]
        public Vector3 positionDifference;
        
        [Tooltip("位置差异大小（毫米）")]
        public float differenceMagnitude_mm;

        void Start()
        {
            // 自动查找组件（如果未手动设置）
            if (kabschAlignment == null)
            {
                kabschAlignment = FindObjectOfType<KabschAlignment>();
            }
            
            if (converterConfig == null)
            {
                converterConfig = FindObjectOfType<SteamVrUrCoordinateConverterConfig>();
            }
            
            if (converterConfig == null)
            {
                Debug.LogWarning("[示例] 未找到SteamVrUrCoordinateConverterConfig组件，请添加到场景中");
            }
        }

        /// <summary>
        /// 步骤1: 执行Kabsch对齐
        /// </summary>
        [ContextMenu("步骤1: 执行Kabsch对齐")]
        public void Step1_PerformKabschAlignment()
        {
            if (kabschAlignment == null)
            {
                Debug.LogError("[示例] 未设置KabschAlignment组件");
                return;
            }
            
            Debug.Log("[示例] 开始执行Kabsch对齐...");
            kabschAlignment.PerformAlignment();
            
            if (kabschAlignment.IsAlignmentComputed)
            {
                Debug.Log($"[示例] ✓ Kabsch对齐完成! RMSE: {kabschAlignment.RMSE:F6}m ({kabschAlignment.RMSE*1000:F3}mm)");
            }
            else
            {
                Debug.LogError("[示例] ✗ Kabsch对齐失败");
            }
        }

        /// <summary>
        /// 步骤2: 导入Kabsch参数到转换器配置
        /// </summary>
        [ContextMenu("步骤2: 导入Kabsch参数")]
        public void Step2_ImportKabschParameters()
        {
            if (converterConfig == null)
            {
                Debug.LogError("[示例] 未设置SteamVrUrCoordinateConverterConfig组件");
                return;
            }
            
            Debug.Log("[示例] 开始导入Kabsch参数...");
            converterConfig.ImportFromKabschAlignment();
            Debug.Log("[示例] ✓ Kabsch参数已导入到转换器配置");
        }

        /// <summary>
        /// 步骤3: 启用Kabsch校正
        /// </summary>
        [ContextMenu("步骤3: 启用Kabsch校正")]
        public void Step3_EnableKabschAlignment()
        {
            if (converterConfig == null)
            {
                Debug.LogError("[示例] 未设置SteamVrUrCoordinateConverterConfig组件");
                return;
            }
            
            converterConfig.enableKabschAlignment = true;
            Debug.Log("[示例] ✓ Kabsch刚性对齐已启用");
        }

        /// <summary>
        /// 步骤4: 测试转换对比
        /// </summary>
        [ContextMenu("步骤4: 测试转换对比")]
        public void Step4_TestConversion()
        {
            if (converterConfig == null)
            {
                Debug.LogError("[示例] 未设置SteamVrUrCoordinateConverterConfig组件");
                return;
            }
            
            Debug.Log("[示例] 开始测试转换...");
            Debug.Log($"[示例] 测试输入位置: ({testTrackerPosition.x:F4}, {testTrackerPosition.y:F4}, {testTrackerPosition.z:F4})");
            
            // 测试1: 未启用Kabsch
            converterConfig.enableKabschAlignment = false;
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                testTrackerPosition, testTrackerRotation, false,
                out Vector3 posWithout, out Vector3 rotWithout);
            resultWithoutKabsch = posWithout;
            
            // 测试2: 启用Kabsch
            converterConfig.enableKabschAlignment = true;
            SteamVrUrCoordinateConverter.ConvertSteamVrPoseToUrBase(
                testTrackerPosition, testTrackerRotation, false,
                out Vector3 posWith, out Vector3 rotWith);
            resultWithKabsch = posWith;
            
            // 计算差异
            positionDifference = resultWithKabsch - resultWithoutKabsch;
            differenceMagnitude_mm = positionDifference.magnitude * 1000f;
            
            Debug.Log($"[示例] 未启用Kabsch: ({resultWithoutKabsch.x:F6}, {resultWithoutKabsch.y:F6}, {resultWithoutKabsch.z:F6})");
            Debug.Log($"[示例] 启用Kabsch后: ({resultWithKabsch.x:F6}, {resultWithKabsch.y:F6}, {resultWithKabsch.z:F6})");
            Debug.Log($"[示例] 位置差异: ({positionDifference.x:F6}, {positionDifference.y:F6}, {positionDifference.z:F6})");
            Debug.Log($"[示例] 差异大小: {differenceMagnitude_mm:F3}mm");
        }

        /// <summary>
        /// 完整流程: 自动执行所有步骤
        /// </summary>
        [ContextMenu("完整流程: 自动执行")]
        public void FullWorkflow_AutoExecute()
        {
            Debug.Log("========== Kabsch刚性对齐完整流程 ==========");
            
            Step1_PerformKabschAlignment();
            Step2_ImportKabschParameters();
            Step3_EnableKabschAlignment();
            Step4_TestConversion();
            
            Debug.Log("========== 流程完成 ==========");
        }

        /// <summary>
        /// 禁用Kabsch校正
        /// </summary>
        [ContextMenu("禁用Kabsch校正")]
        public void DisableKabschAlignment()
        {
            if (converterConfig == null)
            {
                Debug.LogError("[示例] 未设置SteamVrUrCoordinateConverterConfig组件");
                return;
            }
            
            converterConfig.enableKabschAlignment = false;
            Debug.Log("[示例] Kabsch刚性对齐已禁用");
        }

        /// <summary>
        /// 启用调试日志
        /// </summary>
        [ContextMenu("启用详细调试日志")]
        public void EnableDebugLogging()
        {
            SteamVrUrCoordinateConverter.EnableDebugLog = true;
            if (converterConfig != null)
            {
                converterConfig.showKabschDebugLog = true;
            }
            Debug.Log("[示例] 详细调试日志已启用");
        }

        /// <summary>
        /// 禁用调试日志
        /// </summary>
        [ContextMenu("禁用详细调试日志")]
        public void DisableDebugLogging()
        {
            SteamVrUrCoordinateConverter.EnableDebugLog = false;
            if (converterConfig != null)
            {
                converterConfig.showKabschDebugLog = false;
            }
            Debug.Log("[示例] 详细调试日志已禁用");
        }

        void OnDrawGizmos()
        {
            // 绘制测试位置
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(testTrackerPosition, 0.05f);
            
            // 绘制结果对比
            if (resultWithoutKabsch != Vector3.zero && resultWithKabsch != Vector3.zero)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawWireSphere(resultWithoutKabsch, 0.03f);
                
                Gizmos.color = Color.green;
                Gizmos.DrawWireSphere(resultWithKabsch, 0.03f);
                
                Gizmos.color = Color.white;
                Gizmos.DrawLine(resultWithoutKabsch, resultWithKabsch);
            }
        }
    }
}

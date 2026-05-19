using UnityEngine;
using Valve.VR;

namespace handeye
{
    /// <summary>
    /// Tracker 坐标系偏移包装器
    /// 
    /// 功能：在不修改 SteamVR 插件的情况下，为 Tracker 添加自定义坐标系偏移
    /// 
    /// 使用方法：
    /// 1. 将此脚本添加到与 SteamVR_TrackedObject 相同的 GameObject 上
    /// 2. 在 Inspector 中设置偏移量
    /// 3. 本脚本会在 LateUpdate 中应用偏移，覆盖原始位姿
    /// 
    /// 坐标系说明：
    /// - 偏移量在 Tracker 本地坐标系中表示
    /// - Y轴向上，-Y方向为向下
    /// - 例如：localPositionOffset = (0, -0.15, 0) 表示向下偏移 15cm
    /// </summary>
    [RequireComponent(typeof(SteamVR_TrackedObject))]
    public class TrackerCoordinateOffsetWrapper : MonoBehaviour
    {
        [Header("坐标系偏移设置")]
        [Tooltip("相对于 Tracker 本体的位置偏移（米）\n例如：(0, -0.15, 0) = 向下 15cm")]
        public Vector3 localPositionOffset = new Vector3(0f, -0.15f, 0f);

        [Tooltip("相对于 Tracker 本体的旋转偏移（欧拉角，度）")]
        public Vector3 localRotationOffset = Vector3.zero;

        [Header("调试选项")]
        [Tooltip("是否启用偏移（可用于对比效果）")]
        public bool enableOffset = true;

        [Tooltip("显示调试信息")]
        public bool showDebugInfo = false;

        private SteamVR_TrackedObject trackedObject;
        private Vector3 originalPosition;
        private Quaternion originalRotation;

        void Start()
        {
            trackedObject = GetComponent<SteamVR_TrackedObject>();
            if (trackedObject == null)
            {
                Debug.LogError("[TrackerCoordinateOffsetWrapper] 未找到 SteamVR_TrackedObject 组件！");
                enabled = false;
            }
        }

        void LateUpdate()
        {
            if (!enableOffset || trackedObject == null || !trackedObject.isValid)
                return;

            // 保存原始位姿（用于调试）
            originalPosition = transform.position;
            originalRotation = transform.rotation;

            // 计算偏移后的位姿
            ApplyOffset();

            // 显示调试信息
            if (showDebugInfo)
            {
                Debug.Log($"[TrackerOffset] 原始位置: {originalPosition:F3}, 偏移后: {transform.position:F3}");
            }
        }

        /// <summary>
        /// 应用坐标系偏移
        /// </summary>
        private void ApplyOffset()
        {
            // 步骤1: 计算旋转偏移
            Quaternion rotationOffsetQuat = Quaternion.Euler(localRotationOffset);
            Quaternion newRotation = originalRotation * rotationOffsetQuat;

            // 步骤2: 计算位置偏移（在 Tracker 本地坐标系中）
            // 使用原始旋转来变换偏移向量到世界坐标系
            Vector3 worldOffsetPosition = originalRotation * localPositionOffset;
            Vector3 newPosition = originalPosition + worldOffsetPosition;

            // 步骤3: 应用新的位姿
            transform.position = newPosition;
            transform.rotation = newRotation;
        }

        /// <summary>
        /// 在编辑器中可视化偏移
        /// </summary>
        void OnDrawGizmos()
        {
            if (!enableOffset || !Application.isPlaying)
                return;

            // 绘制原始位置（红色球体）
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(originalPosition, 0.02f);

            // 绘制偏移后位置（绿色球体）
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(transform.position, 0.02f);

            // 绘制连接线
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(originalPosition, transform.position);

            // 绘制偏移向量（蓝色箭头）
            Gizmos.color = Color.blue;
            Vector3 worldOffset = originalRotation * localPositionOffset;
            DrawArrow(originalPosition, worldOffset);
        }

        /// <summary>
        /// 绘制箭头辅助函数
        /// </summary>
        private void DrawArrow(Vector3 start, Vector3 direction)
        {
            Vector3 end = start + direction;
            
            // 绘制箭头主干（多条平行线模拟粗线）
            float thickness = 0.003f; // 粗细
            Vector3 perpendicular1 = Vector3.Cross(direction.normalized, Vector3.up).normalized * thickness;
            Vector3 perpendicular2 = Vector3.Cross(direction.normalized, perpendicular1).normalized * thickness;
            
            if (perpendicular1.magnitude < 0.001f)
            {
                perpendicular1 = Vector3.Cross(direction.normalized, Vector3.forward).normalized * thickness;
                perpendicular2 = Vector3.Cross(direction.normalized, perpendicular1).normalized * thickness;
            }
            
            // 绘制5条线模拟粗箭头主干
            Gizmos.DrawLine(start, end); // 中心线
            Gizmos.DrawLine(start + perpendicular1, end + perpendicular1);
            Gizmos.DrawLine(start - perpendicular1, end - perpendicular1);
            Gizmos.DrawLine(start + perpendicular2, end + perpendicular2);
            Gizmos.DrawLine(start - perpendicular2, end - perpendicular2);

            // 箭头头部（加大尺寸）
            if (direction.magnitude > 0.001f)
            {
                Vector3 right = Vector3.Cross(direction.normalized, Vector3.up);
                if (right.magnitude < 0.1f)
                    right = Vector3.Cross(direction.normalized, Vector3.forward);
                
                right = right.normalized * 0.05f; // 箭头宽度（原来0.01f，现在2.5倍）
                Vector3 arrowBase = end - direction.normalized * 0.02f; // 箭头长度（原来0.02f，现在2倍）
                
                // 绘制箭头的两条边
                Gizmos.DrawLine(end, arrowBase + right);
                Gizmos.DrawLine(end, arrowBase - right);
                
                // 绘制箭头底边，形成三角形
                Gizmos.DrawLine(arrowBase + right, arrowBase - right);
                
                // 再绘制一组平行线让箭头更粗
                Vector3 up = Vector3.Cross(direction.normalized, right).normalized * 0.01f;
                Gizmos.DrawLine(end + up, arrowBase + right + up);
                Gizmos.DrawLine(end + up, arrowBase - right + up);
                Gizmos.DrawLine(end - up, arrowBase + right - up);
                Gizmos.DrawLine(end - up, arrowBase - right - up);
            }
        }

        /// <summary>
        /// 运行时动态设置偏移量
        /// </summary>
        public void SetPositionOffset(Vector3 offset)
        {
            localPositionOffset = offset;
        }

        /// <summary>
        /// 运行时动态设置旋转偏移
        /// </summary>
        public void SetRotationOffset(Vector3 eulerAngles)
        {
            localRotationOffset = eulerAngles;
        }

        /// <summary>
        /// 重置为原始位姿（取消偏移）
        /// </summary>
        public void ResetOffset()
        {
            localPositionOffset = Vector3.zero;
            localRotationOffset = Vector3.zero;
        }
    }
}

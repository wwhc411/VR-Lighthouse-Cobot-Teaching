using System;
using UnityEngine;

namespace handeye
{
    /// <summary>
    /// Tracker 到 TCP 坐标轴映射配置器
    /// 
    /// 功能：
    /// - 在 Unity Inspector 中可视化配置 Tracker 与 TCP 坐标轴的对应关系
    /// - 支持每个轴的方向翻转（正向/反向）
    /// - 实时应用到坐标转换系统
    /// 
    /// 使用方法：
    /// 1. 将此脚本挂载到任意 GameObject 上
    /// 2. 在 Inspector 中配置 TCP 三个轴分别对应 Tracker 的哪个轴
    /// 3. 点击 "应用坐标映射" 按钮或在运行时自动应用
    /// 4. 通过调整映射关系，观察机械臂 TCP 的运动是否符合预期
    /// 
    /// 预设配置：
    /// - Preset 1: X→X, Y→-Y, Z→-Z (当前默认)
    /// - Preset 2: X→-Z, Y→-Y, Z→-X
    /// - Preset 3: X→Y, Y→Z, Z→X
    /// </summary>
    public class TrackerToTcpAxisMapper : MonoBehaviour
    {
        /// <summary>
        /// 坐标轴枚举
        /// </summary>
        public enum Axis
        {
            PositiveX = 0,  // +X
            NegativeX = 1,  // -X
            PositiveY = 2,  // +Y
            NegativeY = 3,  // -Y
            PositiveZ = 4,  // +Z
            NegativeZ = 5   // -Z
        }

        [Header("=== Tracker 到 TCP 坐标轴映射配置 ===")]
        [Tooltip("TCP 的 X 轴对应 Tracker 的哪个轴")]
        public Axis tcpX_from_tracker = Axis.PositiveX;

        [Tooltip("TCP 的 Y 轴对应 Tracker 的哪个轴")]
        public Axis tcpY_from_tracker = Axis.NegativeY;

        [Tooltip("TCP 的 Z 轴对应 Tracker 的哪个轴")]
        public Axis tcpZ_from_tracker = Axis.NegativeZ;

        [Header("=== 状态信息 ===")]
        [SerializeField]
        [Tooltip("当前生成的映射矩阵（只读）")]
        private string currentMatrixInfo = "";

        [Header("=== 快捷操作 ===")]
        [Tooltip("是否在启动时自动应用映射")]
        public bool applyOnStart = true;

        [Tooltip("是否在值改变时自动应用映射")]
        public bool applyOnChange = true;

        // 上一次的配置，用于检测变化
        private Axis lastTcpX;
        private Axis lastTcpY;
        private Axis lastTcpZ;

        void Start()
        {
            if (applyOnStart)
            {
                ApplyAxisMapping();
            }
            
            // 记录初始配置
            lastTcpX = tcpX_from_tracker;
            lastTcpY = tcpY_from_tracker;
            lastTcpZ = tcpZ_from_tracker;
        }

        void Update()
        {
            // 检测配置是否改变
            if (applyOnChange && HasConfigChanged())
            {
                ApplyAxisMapping();
                
                // 更新记录
                lastTcpX = tcpX_from_tracker;
                lastTcpY = tcpY_from_tracker;
                lastTcpZ = tcpZ_from_tracker;
            }
        }

        /// <summary>
        /// 检测配置是否改变
        /// </summary>
        private bool HasConfigChanged()
        {
            return tcpX_from_tracker != lastTcpX ||
                   tcpY_from_tracker != lastTcpY ||
                   tcpZ_from_tracker != lastTcpZ;
        }

        /// <summary>
        /// 应用坐标轴映射到转换器
        /// </summary>
        [ContextMenu("应用坐标映射")]
        public void ApplyAxisMapping()
        {
            // 验证映射有效性
            if (!ValidateMapping())
            {
                Debug.LogError("[坐标轴映射器] 映射配置无效！每个 Tracker 轴只能映射到一个 TCP 轴。");
                return;
            }

            // 构建映射矩阵
            Matrix4x4 mappingMatrix = BuildMappingMatrix();

            // 应用到转换器
            SteamVrUrCoordinateConverter.SetTrackerToTcpOffset(mappingMatrix);

            // 更新状态信息
            UpdateMatrixInfo(mappingMatrix);

            Debug.Log($"[坐标轴映射器] 已应用新的坐标映射:\n{GetMappingDescription()}");
        }

        /// <summary>
        /// 验证映射配置的有效性
        /// 规则：每个 Tracker 轴（X/Y/Z）只能被使用一次
        /// </summary>
        private bool ValidateMapping()
        {
            // 提取轴类型（忽略正负）
            int axisX = (int)tcpX_from_tracker / 2;
            int axisY = (int)tcpY_from_tracker / 2;
            int axisZ = (int)tcpZ_from_tracker / 2;

            // 检查是否有重复
            if (axisX == axisY || axisY == axisZ || axisX == axisZ)
            {
                return false;
            }

            return true;
        }

        /// <summary>
        /// 构建映射矩阵
        /// </summary>
        private Matrix4x4 BuildMappingMatrix()
        {
            Vector3 row1 = GetAxisVector(tcpX_from_tracker); // TCP_X 对应的 Tracker 轴（第一行）
            Vector3 row2 = GetAxisVector(tcpY_from_tracker); // TCP_Y 对应的 Tracker 轴（第二行）
            Vector3 row3 = GetAxisVector(tcpZ_from_tracker); // TCP_Z 对应的 Tracker 轴（第三行）

            // 按行构建矩阵（需要手动设置每个元素）
            Matrix4x4 matrix = new Matrix4x4();
            
            // 第一行：TCP_X = row1.x * Tracker_X + row1.y * Tracker_Y + row1.z * Tracker_Z
            matrix.m00 = row1.x; matrix.m01 = row1.y; matrix.m02 = row1.z; matrix.m03 = 0;
            
            // 第二行：TCP_Y = row2.x * Tracker_X + row2.y * Tracker_Y + row2.z * Tracker_Z
            matrix.m10 = row2.x; matrix.m11 = row2.y; matrix.m12 = row2.z; matrix.m13 = 0;
            
            // 第三行：TCP_Z = row3.x * Tracker_X + row3.y * Tracker_Y + row3.z * Tracker_Z
            matrix.m20 = row3.x; matrix.m21 = row3.y; matrix.m22 = row3.z; matrix.m23 = 0;
            
            // 第四行：齐次坐标
            matrix.m30 = 0; matrix.m31 = 0; matrix.m32 = 0; matrix.m33 = 1;

            return matrix;
        }

        /// <summary>
        /// 根据枚举值获取对应的轴向量
        /// </summary>
        private Vector3 GetAxisVector(Axis axis)
        {
            switch (axis)
            {
                case Axis.PositiveX:
                    return new Vector3(1, 0, 0);
                case Axis.NegativeX:
                    return new Vector3(-1, 0, 0);
                case Axis.PositiveY:
                    return new Vector3(0, 1, 0);
                case Axis.NegativeY:
                    return new Vector3(0, -1, 0);
                case Axis.PositiveZ:
                    return new Vector3(0, 0, 1);
                case Axis.NegativeZ:
                    return new Vector3(0, 0, -1);
                default:
                    return new Vector3(1, 0, 0);
            }
        }

        /// <summary>
        /// 获取映射描述文本
        /// </summary>
        private string GetMappingDescription()
        {
            return $"  TCP_X ← Tracker_{GetAxisName(tcpX_from_tracker)}\n" +
                   $"  TCP_Y ← Tracker_{GetAxisName(tcpY_from_tracker)}\n" +
                   $"  TCP_Z ← Tracker_{GetAxisName(tcpZ_from_tracker)}";
        }

        /// <summary>
        /// 获取轴名称
        /// </summary>
        private string GetAxisName(Axis axis)
        {
            switch (axis)
            {
                case Axis.PositiveX: return "+X";
                case Axis.NegativeX: return "-X";
                case Axis.PositiveY: return "+Y";
                case Axis.NegativeY: return "-Y";
                case Axis.PositiveZ: return "+Z";
                case Axis.NegativeZ: return "-Z";
                default: return "Unknown";
            }
        }

        /// <summary>
        /// 更新矩阵信息显示
        /// </summary>
        private void UpdateMatrixInfo(Matrix4x4 matrix)
        {
            currentMatrixInfo = $"[{matrix.m00:F1}, {matrix.m01:F1}, {matrix.m02:F1}]\n" +
                               $"[{matrix.m10:F1}, {matrix.m11:F1}, {matrix.m12:F1}]\n" +
                               $"[{matrix.m20:F1}, {matrix.m21:F1}, {matrix.m22:F1}]";
        }

        // ========== 预设配置 ==========

        [ContextMenu("预设1: X→X, Y→-Y, Z→-Z (默认)")]
        public void ApplyPreset1()
        {
            tcpX_from_tracker = Axis.PositiveX;
            tcpY_from_tracker = Axis.NegativeY;
            tcpZ_from_tracker = Axis.NegativeZ;
            ApplyAxisMapping();
        }

        [ContextMenu("预设2: X→-Z, Y→-Y, Z→-X")]
        public void ApplyPreset2()
        {
            tcpX_from_tracker = Axis.NegativeZ;
            tcpY_from_tracker = Axis.NegativeY;
            tcpZ_from_tracker = Axis.NegativeX;
            ApplyAxisMapping();
        }

        [ContextMenu("预设3: X→Y, Y→Z, Z→X (循环)")]
        public void ApplyPreset3()
        {
            tcpX_from_tracker = Axis.PositiveY;
            tcpY_from_tracker = Axis.PositiveZ;
            tcpZ_from_tracker = Axis.PositiveX;
            ApplyAxisMapping();
        }

        [ContextMenu("预设4: X→-X, Y→Z, Z→Y")]
        public void ApplyPreset4()
        {
            tcpX_from_tracker = Axis.NegativeX;
            tcpY_from_tracker = Axis.PositiveZ;
            tcpZ_from_tracker = Axis.PositiveY;
            ApplyAxisMapping();
        }

        [ContextMenu("预设5: 单位矩阵 (无变换)")]
        public void ApplyPreset5_Identity()
        {
            tcpX_from_tracker = Axis.PositiveX;
            tcpY_from_tracker = Axis.PositiveY;
            tcpZ_from_tracker = Axis.PositiveZ;
            ApplyAxisMapping();
        }

        [ContextMenu("预设6: X→Z, Y→X, Z→Y")]
        public void ApplyPreset6()
        {
            tcpX_from_tracker = Axis.PositiveZ;
            tcpY_from_tracker = Axis.PositiveX;
            tcpZ_from_tracker = Axis.PositiveY;
            ApplyAxisMapping();
        }

        // ========== Inspector 按钮 ==========

#if UNITY_EDITOR
        [Header("=== 调试信息 ===")]
        [Tooltip("显示当前映射关系")]
        [TextArea(3, 5)]
        public string debugInfo = "点击 '应用坐标映射' 查看当前配置";

        void OnValidate()
        {
            // 在 Inspector 中值改变时更新调试信息
            if (ValidateMapping())
            {
                debugInfo = $"映射关系:\n{GetMappingDescription()}\n\n✓ 配置有效";
            }
            else
            {
                debugInfo = "⚠ 警告: 映射配置无效！\n每个 Tracker 轴只能映射一次。";
            }
        }
#endif
    }
}

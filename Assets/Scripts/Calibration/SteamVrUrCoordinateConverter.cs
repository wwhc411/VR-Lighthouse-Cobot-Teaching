using System;
using UnityEngine;

namespace handeye
{
    /// <summary>
    /// SteamVR 坐标系到 UR 机械臂基座坐标系的转换器
    /// 
    /// 功能:
    /// - 使用手眼标定矩阵 T_cam2base 将 SteamVR 位姿转换到 UR 基座坐标系
    /// - 应用 Tracker 到 TCP 的固定旋转偏移 R_tracker2tcp_offset
    /// - 输出 UR 机械臂可直接使用的目标位姿(米, 弧度)
    /// 
    /// 坐标系定义:
    ///   - SteamVR (Cam): 右手系, X(右) Y(上) Z(后)
    ///   - UR Base: 右手系, 符合 ISO 9787 标准
    ///   - TCP: 工具坐标系, Z轴为工作方向(向前)
    /// 
    /// 核心公式:
    ///   位置: p_base = R_cam2base × p_cam + t_cam2base
    ///   旋转: R_tcp = (R_cam2base × R_tracker) × R_tracker2tcp_offset
    /// 
    /// 数据流: Tracker位姿 → 坐标变换 → UR基座位姿
    /// 
    /// 相关文档: 完整流程说明.md - 阶段2: 坐标系转换
    /// </summary>
    public static class SteamVrUrCoordinateConverter
    {
        /// <summary>
        /// 是否输出调试日志（默认关闭，回放时不需要大量日志）
        /// </summary>
        public static bool EnableDebugLog = false;
        
        // ========== Kabsch点云刚性对齐配置 ==========
        // 
        // 作用: 可选的Kabsch刚性对齐校正步骤
        // 用途: 补偿系统性刚性偏移误差
        // 配置: 通过 SteamVrUrCoordinateConverterConfig 组件在Inspector中设置
        // 变换顺序: Kabsch对齐 → 手眼标定 → Tracker偏移
        private static SteamVrUrCoordinateConverterConfig kabschConfig = null;
        
        // ========== 手眼标定矩阵 T_cam2base = ==========
        //
        // 作用: 将 SteamVR 世界坐标系(Cam)转换到 UR 基座坐标系(Base)
        // 类型: 4×4 齐次变换矩阵
        // 单位: 米(m)
        //
        // 标定结果（更新日期: 2025-10-30）：
        // 标定方法: Custom DLL - Tsai Algorithm
        // 标定点数: 12
        //
        // 齐次变换矩阵 (4x4):
        //   [-0.674113,  0.009269, -0.738570, -0.606508 ]
        //   [-0.738465,  0.012541,  0.674175,  0.882720 ]
        //   [ 0.015512,  0.999878, -0.001609,  1.042878 ]
        //   [ 0.000000,  0.000000,  0.000000,  1.000000 ]
        //
        // 旋转矩阵 R_cam2base (3×3):
        //   [-0.674113,  0.009269, -0.738570 ]
        //   [-0.738465,  0.012541,  0.674175 ]
        //   [ 0.015512,  0.999878, -0.001609 ]
        //
        // 平移向量 t_cam2base (原始单位 mm):
        //   [-606.508, 882.720, 1042.878]
        //
        // Unity 坐标系验证:
        //   Position (m): (-0.61, 0.88, 1.04)
        //   Rotation (deg): (317.61, 269.88, 270.97)
        //   Quaternion: (w:-0.2902, x:-0.2806, y:0.6497, z:0.6442)
        //
        // 注意: Unity Matrix4x4 按列存储, 每个 Vector4 是矩阵的一列
        // 列向量按 (r00, r10, r20) 的顺序填写
        private static Matrix4x4 T_cam2base_m = new Matrix4x4
        (
            // column0 = (r00, r10, r20)
            new Vector4(-0.674113f, -0.738465f, 0.015512f, 0f),   // 第1列
            // column1 = (r01, r11, r21)
            new Vector4(0.009269f, 0.012541f, 0.999878f, 0f),  // 第2列
            // column2 = (r02, r12, r22)
            new Vector4(-0.738570f, 0.674175f, -0.001609f, 0f), // 第3列
            new Vector4(0f, 0f, 0f, 1f)                         // 第4列 (齐次坐标)
        );

        // ========== Tracker 到 TCP 的固定旋转偏移 R_tracker2tcp_offset ==========
        // 
        // 作用: 补偿 Tracker 本体坐标系与 TCP 工具坐标系的方向差异
        // 类型: 3×3 旋转矩阵
        // 单位: 无量纲(正交矩阵)
        // 
        // 物理原因: Tracker 安装方式导致坐标轴重新映射
        // 
        // 坐标系对应关系 (更新日期: 2025-10-30):
        //   TCP_X = Tracker_X   (同向)
        //   TCP_Y = -Tracker_Y  (反向)
        //   TCP_Z = -Tracker_Z  (反向)
        // 
        // 旋转矩阵推导:
        //   TCP坐标系在Tracker坐标系下的基向量表示:
        //   TCP_X = Tracker_X   →  列1: [ 1,  0,  0]
        //   TCP_Y = -Tracker_Y  →  列2: [ 0, -1,  0]
        //   TCP_Z = -Tracker_Z  →  列3: [ 0,  0, -1]
        // 
        // 验证右手系: TCP_X × TCP_Y = TCP_Z
        //   Tracker_X × (-Tracker_Y) = -Tracker_Z = TCP_Z ✓
        // 
        // 注意: 此矩阵根据实际Tracker安装方式确定（2025-10-30）
        private static Matrix4x4 R_tracker2tcp_offset = new Matrix4x4
        (
            // 列向量表示: 每列是TCP坐标系的一个基向量在Tracker坐标系中的表示
            new Vector4(1f, 0f, 0f, 0f),     // 第1列: TCP_X = Tracker_X
            new Vector4(0f, -1f, 0f, 0f),    // 第2列: TCP_Y = -Tracker_Y
            new Vector4(0f, 0f, -1f, 0f),    // 第3列: TCP_Z = -Tracker_Z
            new Vector4(0f, 0f, 0f, 1f)      // 第4列: 齐次坐标
        );

        static SteamVrUrCoordinateConverter()
        {
            // 填入平移分量（m03, m13, m23）
            // 将标定的毫米值转换为米（mm -> m）并写入矩阵的平移分量
            // 原始平移(mm): [-606.508, 882.720, 1042.878]
            T_cam2base_m.m03 = -0.606508f; // X (m)
            T_cam2base_m.m13 = 0.882720f;  // Y (m)
            T_cam2base_m.m23 = 1.042878f;  // Z (m)
        }

        /// <summary>
        /// 动态更新手眼标定矩阵
        /// </summary>
        /// <param name="cam2base_m">新的 T_cam2base 矩阵（单位：米）</param>
        public static void SetCalibration(Matrix4x4 cam2base_m)
        {
            T_cam2base_m = cam2base_m;
        }

        /// <summary>
        /// 动态更新 Tracker 到 TCP 的坐标轴映射矩阵
        /// </summary>
        /// <param name="offset_m">新的 R_tracker2tcp_offset 矩阵（3×3 旋转矩阵）</param>
        public static void SetTrackerToTcpOffset(Matrix4x4 offset_m)
        {
            R_tracker2tcp_offset = offset_m;
            Debug.Log($"[坐标转换器] R_tracker2tcp_offset 已更新");
        }

        /// <summary>
        /// 获取当前的 Tracker 到 TCP 坐标轴映射矩阵
        /// </summary>
        /// <returns>当前的 R_tracker2tcp_offset 矩阵</returns>
        public static Matrix4x4 GetTrackerToTcpOffset()
        {
            return R_tracker2tcp_offset;
        }

        /// <summary>
        /// 设置Kabsch配置组件
        /// </summary>
        /// <param name="config">配置组件实例</param>
        public static void SetKabschConfig(SteamVrUrCoordinateConverterConfig config)
        {
            kabschConfig = config;
            if (config != null && config.enableKabschAlignment)
            {
                Debug.Log($"[坐标转换器] Kabsch刚性对齐已启用");
            }
        }

        /// <summary>
        /// 应用Kabsch刚性对齐（如果启用）
        /// </summary>
        /// <param name="point">输入点（SteamVR坐标系，米）</param>
        /// <returns>校正后的点（米）</returns>
        private static Vector3 ApplyKabschAlignmentIfEnabled(Vector3 point)
        {
            if (kabschConfig != null && kabschConfig.enableKabschAlignment)
            {
                return kabschConfig.ApplyKabschAlignment(point);
            }
            return point;
        }

        /// <summary>
        /// 将 SteamVR Tracker 位姿转换为 UR 基座坐标系下的 TCP 目标位姿
        /// 
        /// 功能: 使机械臂 TCP 移动到 Tracker 当前位置和姿态
        /// 
        /// 转换流程(共3步):
        /// 
        ///   【步骤1】位置坐标变换
        ///     输入: p_cam (SteamVR坐标系, 米)
        ///     公式: p_base = R_cam2base × p_cam + t_cam2base
        ///     输出: p_base (UR基座坐标系, 米)
        /// 
        ///   【步骤2】姿态变换
        ///     2.1 四元数归一化: ||q|| = 1
        ///     2.2 四元数 → 旋转矩阵: q_cam → R_cam (3×3)
        ///     2.3 应用手眼标定: R_intermediate = R_cam2base × R_cam
        ///     2.4 应用 Tracker 偏移: R_tcp = R_intermediate × R_tracker2tcp_offset
        ///     2.5 旋转矩阵 → 四元数: R_tcp → q_base
        /// 
        ///   【步骤3】格式转换
        ///     输入: q_base (四元数)
        ///     转换: 四元数 → 轴角(Rodrigues旋转向量)
        ///     输出: rotvec_base (UR轴角格式, 弧度)
        /// 
        /// 坐标系说明:
        ///   - Cam (SteamVR): 右手系, X(右) Y(上) Z(后)
        ///   - Base (UR): 右手系, 符合 ISO 9787
        ///   - TCP: 工具坐标系, Z轴为工作方向
        /// 
        /// 相关文档: 完整流程说明.md - 阶段2: 坐标系转换
        /// </summary>
        /// <param name="posInput">Tracker 位置（SteamVR，单位：米）</param>
        /// <param name="quatInput">Tracker 姿态（SteamVR 四元数，px,py,pz,pw）</param>
        /// <param name="posInMillimeters">已废弃，保留兼容性</param>
        /// <param name="posOut_m">输出位置（UR 基座，单位：米）</param>
        /// <param name="rotvecOut_rad">输出旋转向量（UR 基座，轴角表示，单位：弧度）</param>
        public static void ConvertSteamVrPoseToUrBase(Vector3 posInput, Quaternion quatInput, bool posInMillimeters, out Vector3 posOut_m, out Vector3 rotvecOut_rad)
        {
            if (EnableDebugLog)
            {
                Debug.Log($"[坐标转换器] ConvertSteamVrPoseToUrBase 开始:");
                Debug.Log($"  输入位置: ({posInput.x:F3}, {posInput.y:F3}, {posInput.z:F3}) {(posInMillimeters ? "mm" : "m")}");
                Debug.Log($"  输入四元数: (w:{quatInput.w:F4}, x:{quatInput.x:F4}, y:{quatInput.y:F4}, z:{quatInput.z:F4})");
            }

            // ========== 单位转换（如果需要）==========
            // 如果输入是毫米，先转换为米
            Vector3 posInput_m = posInMillimeters ? (posInput / 1000f) : posInput;
            
            if (EnableDebugLog && posInMillimeters)
            {
                Debug.Log($"  单位转换: mm → m");
                Debug.Log($"  转换后位置(m): ({posInput_m.x:F4}, {posInput_m.y:F4}, {posInput_m.z:F4})");
            }

            // ========== 步骤0: Kabsch刚性对齐（可选）==========
            // 注意：此步骤在所有其他变换之前执行
            // 公式: p'_cam = R_kabsch × p_cam + t_kabsch
            posInput_m = ApplyKabschAlignmentIfEnabled(posInput_m);
            
            if (EnableDebugLog && kabschConfig != null && kabschConfig.enableKabschAlignment)
            {
                Debug.Log($"  步骤0 - Kabsch对齐后(m): ({posInput_m.x:F4}, {posInput_m.y:F4}, {posInput_m.z:F4})");
            }

            // ========== 步骤1: 位置坐标变换 ==========
            // 公式: p_base = R_cam2base * p_cam + t_cam2base
            posOut_m = TransformPoint(T_cam2base_m, posInput_m);
            
            if (EnableDebugLog)
            {
                Debug.Log($"  步骤1 - 位置变换后(m): ({posOut_m.x:F4}, {posOut_m.y:F4}, {posOut_m.z:F4})");
            }

            // ========== 步骤2: 姿态变换 ==========
            // 2.1 四元数归一化
            Quaternion quatNormalized = Normalize(quatInput);

            // 2.2 SteamVR 四元数 → 旋转矩阵 R_cam
            Matrix4x4 R_cam = QuaternionToRotationMatrix(quatNormalized);

            // 2.3 提取 R_cam2base 旋转部分并相乘: R_base = R_cam2base × R_cam
            Matrix4x4 R_cam2base = ExtractRotationMatrix(T_cam2base_m);
            Matrix4x4 R_base_intermediate = MatrixMultiply3x3(R_cam2base, R_cam);

            // 2.4 应用Tracker到TCP的固定旋转偏移: R_tcp = R_base_intermediate × R_tracker2tcp_offset
            Matrix4x4 R_base = MatrixMultiply3x3(R_base_intermediate, R_tracker2tcp_offset);

            // ========== 步骤3: 格式转换 ==========
            // 直接从旋转矩阵转换为轴角（UR 格式）
            // 注意: 直接转换避免四元数中间步骤的数值误差，在大角度旋转时更稳定
            rotvecOut_rad = RotationMatrixToRotationVector(R_base);
            
            if (EnableDebugLog)
            {
                Debug.Log($"  步骤3 - 格式转换后:");
                Debug.Log($"  输出轴角(rad): ({rotvecOut_rad.x:F4}, {rotvecOut_rad.y:F4}, {rotvecOut_rad.z:F4})");
                Debug.Log($"[坐标转换器] ConvertSteamVrPoseToUrBase 完成");
            }
        }

        /// <summary>
        /// 重载:支持轴角输入（向后兼容）
        /// </summary>
        /// <param name="posInput">Tracker 位置（SteamVR，单位：米）</param>
        /// <param name="rotvecInput">Tracker 姿态（轴角表示，单位：弧度）</param>
        /// <param name="posInMillimeters">已废弃，保留兼容性</param>
        /// <param name="posOut_m">输出位置（UR 基座，单位：米）</param>
        /// <param name="rotvecOut_rad">输出旋转向量（UR 基座，轴角表示，单位：弧度）</param>
        public static void ConvertSteamVrPoseToUrBase(Vector3 posInput, Vector3 rotvecInput, bool posInMillimeters, out Vector3 posOut_m, out Vector3 rotvecOut_rad)
        {
            // 将轴角转换为四元数后调用主方法
            Quaternion quat = RotationVectorToQuaternion(rotvecInput);
            ConvertSteamVrPoseToUrBase(posInput, quat, posInMillimeters, out posOut_m, out rotvecOut_rad);
        }

        /// <summary>
        /// 将相机坐标系的位姿误差向量转换到基座坐标系
        /// 
        /// 功能: 用于视觉伺服误差补偿，将 Tracker1 与 Tracker2 之间的位姿误差转换到 UR 基座坐标系
        /// 
        /// 转换原理:
        ///   【位置误差】只旋转，不平移
        ///     ΔP_base = R_cam2base × ΔP_cam
        ///   
        ///   【旋转误差】需要相似变换
        ///     ΔR_base = R_cam2base × ΔR_cam × R_cam2base^T
        /// 
        /// 关键区别:
        /// - 绝对位姿变换: p_base = R_cam2base × p_cam + t_cam2base (需要平移)
        /// - 误差向量变换: Δp_base = R_cam2base × Δp_cam (只旋转)
        /// 
        /// 适用场景:
        /// - 视觉伺服补偿循环: 先在相机系计算误差，再转换到基座系
        /// - 手眼标定验证: 对比不同坐标系下的误差分量
        /// 
        /// 调用示例:
        /// <code>
        /// // 在相机系计算两个 Tracker 的位姿误差
        /// Vector3 errorCamera = tracker1Pos - tracker2Pos;
        /// Quaternion errorRotCamera = tracker1Rot * Quaternion.Inverse(tracker2Rot);
        /// 
        /// // 转换误差到基座系
        /// ConvertErrorVectorToUrBase(errorCamera, errorRotCamera, 
        ///     out Vector3 errorUR, out Vector3 errorRotUR);
        /// 
        /// // 用基座系误差修正 servoj 目标
        /// newTarget = currentTarget + errorUR;
        /// </code>
        /// 
        /// 相关文档: 视觉伺服误差补偿功能实现方案.md - 阶段2: 手眼坐标变换接口
        /// </summary>
        /// <param name="deltaPosCamera">相机系位置误差（米）</param>
        /// <param name="deltaRotCamera">相机系旋转误差（四元数）</param>
        /// <param name="deltaPosUR">输出：基座系位置误差（米）</param>
        /// <param name="deltaRotUR">输出：基座系旋转误差（旋转矢量 rad）</param>
        public static void ConvertErrorVectorToUrBase(
            Vector3 deltaPosCamera,
            Quaternion deltaRotCamera,
            out Vector3 deltaPosUR,
            out Vector3 deltaRotUR)
        {
            // 日志已禁用（可通过设置 verboseLogging 开启）
            // Debug.Log($"[坐标转换器] ConvertErrorVectorToUrBase 开始:");
            // Debug.Log($"  输入位置误差(m): ({deltaPosCamera.x:F4}, {deltaPosCamera.y:F4}, {deltaPosCamera.z:F4})");
            // Debug.Log($"  输入旋转误差(quat): (w:{deltaRotCamera.w:F4}, x:{deltaRotCamera.x:F4}, y:{deltaRotCamera.y:F4}, z:{deltaRotCamera.z:F4})");

            // ========== 步骤1: 位置误差转换（只旋转，不平移）==========
            // 公式: ΔP_base = R_cam2base × ΔP_cam
            // 
            // 注意: 与绝对位姿不同，误差向量不包含平移分量
            // 因为误差是两个位姿的差值，平移项会被抵消
            
            // 提取旋转矩阵的 3x3 部分
            Matrix4x4 R_cam2base = ExtractRotationMatrix(T_cam2base_m);
            
            // 应用旋转变换
            deltaPosUR = new Vector3(
                R_cam2base.m00 * deltaPosCamera.x + R_cam2base.m01 * deltaPosCamera.y + R_cam2base.m02 * deltaPosCamera.z,
                R_cam2base.m10 * deltaPosCamera.x + R_cam2base.m11 * deltaPosCamera.y + R_cam2base.m12 * deltaPosCamera.z,
                R_cam2base.m20 * deltaPosCamera.x + R_cam2base.m21 * deltaPosCamera.y + R_cam2base.m22 * deltaPosCamera.z
            );
            
            // 日志已禁用
            // Debug.Log($"  步骤1 - 位置误差变换后(m): ({deltaPosUR.x:F4}, {deltaPosUR.y:F4}, {deltaPosUR.z:F4})");

            // ========== 步骤2: 旋转误差转换（相似变换）==========
            // 公式: R_error_base = R_cam2base × R_error_cam × R_cam2base^T
            // 
            // 理论基础:
            //   在相机系: R_target = R_error_cam × R_current
            //   在基座系: R_target_base = R_cam2base × R_target × R_cam2base^T
            //           = R_cam2base × (R_error_cam × R_current) × R_cam2base^T
            //           = (R_cam2base × R_error_cam × R_cam2base^T) × (R_cam2base × R_current × R_cam2base^T)
            //   因此: R_error_base = R_cam2base × R_error_cam × R_cam2base^T
            
            // 2.1 归一化四元数
            Quaternion deltaRotNormalized = Normalize(deltaRotCamera);

            // 2.2 四元数 → 旋转矩阵
            Matrix4x4 R_error_cam = QuaternionToRotationMatrix(deltaRotNormalized);

            // 2.3 计算相似变换: R_error_base = R_cam2base × R_error_cam × R_cam2base^T
            Matrix4x4 R_cam2base_T = Transpose3x3(R_cam2base);
            Matrix4x4 temp = MatrixMultiply3x3(R_cam2base, R_error_cam);
            Matrix4x4 R_error_base = MatrixMultiply3x3(temp, R_cam2base_T);

            // ❌ 移除步骤2.4: 不应该再次应用 Tracker→TCP 偏移
            // 原因: 误差向量是 Tracker 空间的，ConvertSteamVrPoseToUrBase 转换绝对位姿时已包含偏移
            //       这里只需要旋转坐标系，不需要再次变换
            
            // 2.4 旋转矩阵 → 轴角（旋转矢量）
            deltaRotUR = RotationMatrixToRotationVector(R_error_base);
            
            // 日志已禁用
            // Debug.Log($"  步骤2 - 旋转误差变换后(rad): ({deltaRotUR.x:F4}, {deltaRotUR.y:F4}, {deltaRotUR.z:F4})");
            // Debug.Log($"[坐标转换器] ConvertErrorVectorToUrBase 完成");
        }

        // ==================== 辅助方法 ====================

        /// <summary>
        /// 位置点变换（参考文档步骤1.2）
        /// 公式: p_new = R * p + t
        /// </summary>
        /// <param name="transform">齐次变换矩阵（4x4）</param>
        /// <param name="point">输入点（3D）</param>
        /// <returns>变换后的点</returns>
        private static Vector3 TransformPoint(Matrix4x4 transform, Vector3 point)
        {
            // 提取旋转矩阵的3x3部分和平移向量
            // newX = R[0,0]*x + R[0,1]*y + R[0,2]*z + t[0]
            // newY = R[1,0]*x + R[1,1]*y + R[1,2]*z + t[1]
            // newZ = R[2,0]*x + R[2,1]*y + R[2,2]*z + t[2]
            float newX = transform.m00 * point.x + transform.m01 * point.y + transform.m02 * point.z + transform.m03;
            float newY = transform.m10 * point.x + transform.m11 * point.y + transform.m12 * point.z + transform.m13;
            float newZ = transform.m20 * point.x + transform.m21 * point.y + transform.m22 * point.z + transform.m23;

            return new Vector3(newX, newY, newZ);
        }

        /// <summary>
        /// 提取 4x4 齐次变换矩阵的旋转部分（3x3）
        /// </summary>
        private static Matrix4x4 ExtractRotationMatrix(Matrix4x4 transform)
        {
            Matrix4x4 rot = Matrix4x4.identity;
            // 复制旋转部分（左上角 3x3）
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    rot[i, j] = transform[i, j];
                }
            }
            return rot;
        }

        /// <summary>
        /// 四元数转旋转矩阵（参考文档步骤3.2.2）
        /// </summary>
        /// <param name="q">输入四元数</param>
        /// <returns>3x3旋转矩阵（存储为4x4，右下角为单位矩阵）</returns>
        private static Matrix4x4 QuaternionToRotationMatrix(Quaternion q)
        {
            // 公式来自参考文档:
            // R[0,0] = 1 - 2(qy² + qz²)
            // R[0,1] = 2(qx*qy - qw*qz)
            // R[0,2] = 2(qx*qz + qw*qy)
            // R[1,0] = 2(qx*qy + qw*qz)
            // R[1,1] = 1 - 2(qx² + qz²)
            // R[1,2] = 2(qy*qz - qw*qx)
            // R[2,0] = 2(qx*qz - qw*qy)
            // R[2,1] = 2(qy*qz + qw*qx)
            // R[2,2] = 1 - 2(qx² + qy²)

            float xx = q.x * q.x;
            float yy = q.y * q.y;
            float zz = q.z * q.z;
            float xy = q.x * q.y;
            float xz = q.x * q.z;
            float yz = q.y * q.z;
            float wx = q.w * q.x;
            float wy = q.w * q.y;
            float wz = q.w * q.z;

            Matrix4x4 mat = Matrix4x4.identity;
            mat.m00 = 1f - 2f * (yy + zz);
            mat.m01 = 2f * (xy - wz);
            mat.m02 = 2f * (xz + wy);

            mat.m10 = 2f * (xy + wz);
            mat.m11 = 1f - 2f * (xx + zz);
            mat.m12 = 2f * (yz - wx);

            mat.m20 = 2f * (xz - wy);
            mat.m21 = 2f * (yz + wx);
            mat.m22 = 1f - 2f * (xx + yy);

            return mat;
        }

        /// <summary>
        /// 3x3 旋转矩阵相乘（参考文档步骤3.2.3）
        /// 结果: C = A * B
        /// </summary>
        private static Matrix4x4 MatrixMultiply3x3(Matrix4x4 A, Matrix4x4 B)
        {
            // 公式: C[i,j] = Σ(k=0 to 2) A[i,k] * B[k,j]
            Matrix4x4 result = Matrix4x4.identity;

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    float sum = 0f;
                    for (int k = 0; k < 3; k++)
                    {
                        sum += A[i, k] * B[k, j];
                    }
                    result[i, j] = sum;
                }
            }

            return result;
        }

        /// <summary>
        /// 3x3 矩阵转置
        /// </summary>
        private static Matrix4x4 Transpose3x3(Matrix4x4 m)
        {
            Matrix4x4 result = Matrix4x4.identity;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    result[i, j] = m[j, i];
                }
            }
            return result;
        }

        /// <summary>
        /// 旋转矩阵转四元数（参考文档步骤3.2.4）
        /// 使用基于迹（trace）的方法
        /// </summary>
        private static Quaternion RotationMatrixToQuaternion(Matrix4x4 m)
        {
            // trace = R[0,0] + R[1,1] + R[2,2]
            float trace = m.m00 + m.m11 + m.m22;

            Quaternion q = Quaternion.identity;

            if (trace > 0f)
            {
                // 公式（trace > 0 情况）:
                // s = 2 * sqrt(trace + 1)
                // qw = 0.25 * s
                // qx = (R[2,1] - R[1,2]) / s
                // qy = (R[0,2] - R[2,0]) / s
                // qz = (R[1,0] - R[0,1]) / s
                float s = 2f * Mathf.Sqrt(trace + 1f);
                q.w = 0.25f * s;
                q.x = (m.m21 - m.m12) / s;
                q.y = (m.m02 - m.m20) / s;
                q.z = (m.m10 - m.m01) / s;
            }
            else if (m.m00 > m.m11 && m.m00 > m.m22)
            {
                // 最大对角元素是 R[0,0]
                float s = 2f * Mathf.Sqrt(1f + m.m00 - m.m11 - m.m22);
                q.w = (m.m21 - m.m12) / s;
                q.x = 0.25f * s;
                q.y = (m.m01 + m.m10) / s;
                q.z = (m.m02 + m.m20) / s;
            }
            else if (m.m11 > m.m22)
            {
                // 最大对角元素是 R[1,1]
                float s = 2f * Mathf.Sqrt(1f + m.m11 - m.m00 - m.m22);
                q.w = (m.m02 - m.m20) / s;
                q.x = (m.m01 + m.m10) / s;
                q.y = 0.25f * s;
                q.z = (m.m12 + m.m21) / s;
            }
            else
            {
                // 最大对角元素是 R[2,2]
                float s = 2f * Mathf.Sqrt(1f + m.m22 - m.m00 - m.m11);
                q.w = (m.m10 - m.m01) / s;
                q.x = (m.m02 + m.m20) / s;
                q.y = (m.m12 + m.m21) / s;
                q.z = 0.25f * s;
            }

            // 规范化符号: 确保 q.w >= 0，保持四元数唯一性
            // 原理: q 和 -q 表示同一个旋转，选择 w >= 0 的表示
            if (q.w < 0f)
            {
                q.x = -q.x;
                q.y = -q.y;
                q.z = -q.z;
                q.w = -q.w;
            }

            return q;
        }

        /// <summary>
        /// 计算刚体变换的逆矩阵（已废弃，保留兼容性）
        /// 对于 T = [R t; 0 1]，其逆为 T^-1 = [R^T -R^T*t; 0 1]
        /// </summary>
        private static Matrix4x4 InverseRigidBody(Matrix4x4 T)
        {
            Quaternion q = T.rotation;
            Vector3 t = new Vector3(T.m03, T.m13, T.m23);
            Quaternion qInv = Quaternion.Inverse(q);
            Vector3 tInv = -(qInv * t);
            return Matrix4x4.TRS(tInv, qInv, Vector3.one);
        }

        /// <summary>
        /// 轴角表示转换为四元数（参考文档步骤4逆过程）
        /// </summary>
        /// <param name="r">旋转向量（轴*角度，单位：弧度）</param>
        /// <returns>对应的四元数</returns>
        private static Quaternion RotationVectorToQuaternion(Vector3 r)
        {
            float theta = r.magnitude;
            if (theta < 1e-8f) return Quaternion.identity;

            Vector3 axis = r / theta;
            
            // 规范化角度到 [0, π] 范围
            // 旋转 θ 绕轴 n 等价于旋转 (2π-θ) 绕轴 -n
            // 这样可以避免 θ > π 时四元数 w < 0 的情况
            while (theta > Mathf.PI)
            {
                theta = 2f * Mathf.PI - theta;
                axis = -axis;
            }
            while (theta < 0f)
            {
                theta = -theta;
                axis = -axis;
            }
            
            float half = theta * 0.5f;
            float s = Mathf.Sin(half);
            return new Quaternion(axis.x * s, axis.y * s, axis.z * s, Mathf.Cos(half));
        }

        /// <summary>
        /// 旋转矩阵直接转换为轴角表示（推荐方法）
        /// 
        /// 优势：
        /// - 避免四元数中间步骤的数值误差
        /// - 在大角度旋转时更稳定
        /// - 适用于手眼标定等大基准旋转场景
        /// 
        /// 公式:
        ///   θ = arccos((trace(R) - 1) / 2)
        ///   k = (R32-R23, R13-R31, R21-R12) / (2*sin(θ))
        ///   r = θ * k
        /// </summary>
        /// <param name="m">输入旋转矩阵（3x3 正交矩阵）</param>
        /// <returns>旋转向量（轴*角度，单位：弧度）</returns>
        private static Vector3 RotationMatrixToRotationVector(Matrix4x4 m)
        {
            // 计算旋转角度: θ = arccos((trace - 1) / 2)
            float trace = m.m00 + m.m11 + m.m22;
            float cosAngle = Mathf.Clamp((trace - 1f) / 2f, -1f, 1f);
            float angle = Mathf.Acos(cosAngle);
            float sinAngle = Mathf.Sin(angle);

            // 特殊情况1: 角度接近 0（无旋转）
            if (Mathf.Abs(angle) < 1e-6f)
            {
                return Vector3.zero;
            }

            // ========== 关键修复：扩大 π 附近的阈值 ==========
            // 当 sin(angle) 较小时（角度接近 0 或 π），反对称方法不稳定
            // 阈值 0.1 对应角度范围约 [π-0.1, π] 即 [174°, 180°]
            // 以及 [0, 0.1] 即 [0°, 6°]，但 0 附近已被上面处理
            const float SIN_THRESHOLD = 0.1f;
            
            if (sinAngle < SIN_THRESHOLD)
            {
                // 角度接近 π：使用对称矩阵的对角元素方法（更稳定）
                // 原理：R = I + 2*k*k^T - I = 2*k*k^T（当 θ = π 时）
                // 所以 k_i = sqrt((R_ii + 1) / 2)
                
                Vector3 axis180;
                
                // 选择对角元素最大的轴（数值更稳定）
                if (m.m00 >= m.m11 && m.m00 >= m.m22)
                {
                    // X 轴对应的对角元素最大
                    float x = Mathf.Sqrt(Mathf.Max(0f, (m.m00 + 1f) / 2f));
                    if (x < 1e-6f) x = 1e-6f; // 防止除零
                    float y = m.m01 / (2f * x);
                    float z = m.m02 / (2f * x);
                    axis180 = new Vector3(x, y, z).normalized;
                }
                else if (m.m11 >= m.m22)
                {
                    // Y 轴对应的对角元素最大
                    float y = Mathf.Sqrt(Mathf.Max(0f, (m.m11 + 1f) / 2f));
                    if (y < 1e-6f) y = 1e-6f;
                    float x = m.m01 / (2f * y);
                    float z = m.m12 / (2f * y);
                    axis180 = new Vector3(x, y, z).normalized;
                }
                else
                {
                    // Z 轴对应的对角元素最大
                    float z = Mathf.Sqrt(Mathf.Max(0f, (m.m22 + 1f) / 2f));
                    if (z < 1e-6f) z = 1e-6f;
                    float x = m.m02 / (2f * z);
                    float y = m.m12 / (2f * z);
                    axis180 = new Vector3(x, y, z).normalized;
                }
                
                // ========== 轴方向一致性修复 ==========
                // 对称矩阵方法无法确定轴的正负方向
                // 使用反对称部分来确定符号（即使 sin 很小，符号仍然正确）
                Vector3 antiSymAxis = new Vector3(
                    m.m21 - m.m12,
                    m.m02 - m.m20,
                    m.m10 - m.m01
                );
                
                // 如果反对称轴与对称轴方向相反，翻转符号
                if (Vector3.Dot(antiSymAxis, axis180) < 0)
                {
                    axis180 = -axis180;
                }
                
                return axis180 * angle;
            }

            // 一般情况: 从反对称部分提取旋转轴（sin(angle) 足够大时稳定）
            // k = (R32-R23, R13-R31, R21-R12) / (2*sin(θ))
            float s = 2f * sinAngle;
            Vector3 axis = new Vector3(
                (m.m21 - m.m12) / s,
                (m.m02 - m.m20) / s,
                (m.m10 - m.m01) / s
            );

            // 返回旋转向量 r = θ * k
            return axis * angle;
        }

        /// <summary>
        /// 四元数转换为轴角表示（旧方法，保留兼容性）
        /// 
        /// 注意: 在大角度基准旋转时可能产生数值误差
        /// 推荐使用 RotationMatrixToRotationVector() 直接从矩阵转换
        /// 
        /// 公式:
        ///   θ = 2 * arccos(qw)
        ///   k = (qx, qy, qz) / sin(θ/2)
        ///   r = θ * k
        /// 
        /// 修复: 添加四元数符号规范化，避免 θ > π 的情况
        /// </summary>
        /// <param name="q">输入四元数</param>
        /// <returns>旋转向量（轴*角度，单位：弧度）</returns>
        private static Vector3 QuaternionToRotationVector(Quaternion q)
        {
            // 1. 归一化
            q = Normalize(q);

            // 2. 规范化符号: 强制 q.w >= 0，确保角度在 [0, π] 范围
            // 原理: 四元数 q 和 -q 表示同一个旋转
            if (q.w < 0f)
            {
                q.x = -q.x;
                q.y = -q.y;
                q.z = -q.z;
                q.w = -q.w;
            }

            // 3. 限制 qw 在 [0, 1] 范围内（已经保证 >= 0）
            float wClamped = Mathf.Clamp(q.w, 0f, 1f);

            // 4. 计算角度: θ = 2 * arccos(qw)，现在 θ ∈ [0, π]
            float angle = 2f * Mathf.Acos(wClamped);

            // 5. 特殊情况: 角度接近 0（无旋转）
            if (angle < 1e-6f)
            {
                return Vector3.zero;
            }

            // 6. 特殊情况: 角度接近 π（180度）
            // 当 θ ≈ π 时，sin(θ/2) ≈ 1，直接从四元数虚部提取轴
            if (angle > Mathf.PI - 1e-4f)
            {
                Vector3 axis180 = new Vector3(q.x, q.y, q.z);
                float axisLen = axis180.magnitude;
                if (axisLen > 1e-6f)
                {
                    axis180 = axis180 / axisLen;
                    return axis180 * angle;
                }
                else
                {
                    // 极端情况: 无法确定轴方向
                    return Vector3.zero;
                }
            }

            // 7. 一般情况: 0 < θ < π
            // s = sin(θ/2)，由于 θ/2 ∈ (0, π/2)，s > 0 且稳定
            float halfAngle = angle * 0.5f;
            float s = Mathf.Sin(halfAngle);

            // 提取旋转轴: k = (qx, qy, qz) / sin(θ/2)
            Vector3 axis = new Vector3(q.x / s, q.y / s, q.z / s);

            // 返回旋转向量: r = θ * k
            return axis * angle;
        }

        /// <summary>
        /// 归一化四元数（参考文档步骤3.2.1）
        /// </summary>
        private static Quaternion Normalize(Quaternion q)
        {
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (mag < 1e-12f) return Quaternion.identity;
            float inv = 1f / mag;
            q.x *= inv; q.y *= inv; q.z *= inv; q.w *= inv;
            return q;
        }
    }
}


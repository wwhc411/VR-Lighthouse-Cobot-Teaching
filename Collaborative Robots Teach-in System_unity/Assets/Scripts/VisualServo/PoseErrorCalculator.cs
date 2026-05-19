using UnityEngine;

namespace VisualServo
{
    /// <summary>
    /// 位姿误差数据结构
    /// 
    /// 包含位置误差、旋转误差及其范数，用于视觉伺服补偿循环的收敛判断
    /// </summary>
    public class PoseError
    {
        /// <summary>位置误差向量（米）</summary>
        public Vector3 positionError;
        
        /// <summary>旋转误差向量（弧度，旋转矢量表示）</summary>
        public Vector3 rotationError;
        
        /// <summary>位置误差范数（欧几里得距离，米）</summary>
        public float positionMagnitude;
        
        /// <summary>旋转误差角度（弧度）</summary>
        public float rotationMagnitude;

        /// <summary>
        /// 判断位姿误差是否满足收敛条件
        /// </summary>
        /// <param name="posThreshold">位置误差阈值（米），默认 0.001m = 1mm</param>
        /// <param name="rotThreshold">旋转误差阈值（弧度），默认 0.01745rad ≈ 1°</param>
        /// <returns>如果位置和旋转误差都小于阈值则返回 true</returns>
        public bool IsConverged(float posThreshold = 0.001f, float rotThreshold = 0.01745f)
        {
            return positionMagnitude < posThreshold && rotationMagnitude < rotThreshold;
        }

        /// <summary>
        /// 格式化输出误差信息
        /// </summary>
        public override string ToString()
        {
            return string.Format(
                "位置误差: {0:F4}m ({1:F2}mm), 旋转误差: {2:F4}rad ({3:F2}°)",
                positionMagnitude,
                positionMagnitude * 1000f,
                rotationMagnitude,
                rotationMagnitude * Mathf.Rad2Deg
            );
        }
    }

    /// <summary>
    /// 位姿误差计算工具类
    /// 
    /// 功能：
    /// - 计算两个位姿之间的误差（位置差 + 旋转差）
    /// - 四元数与旋转矢量的相互转换
    /// - 旋转误差范数计算
    /// 
    /// 适用场景：
    /// - 视觉伺服误差补偿
    /// - 手眼标定验证
    /// - 位姿跟踪精度评估
    /// 
    /// 坐标系说明：
    /// - 输入位姿可以是任意坐标系（SteamVR 或 UR Base）
    /// - 输出误差与输入位姿在同一坐标系下
    /// - 跨坐标系的误差需要使用手眼变换（SteamVrUrCoordinateConverter）
    /// </summary>
    public static class PoseErrorCalculator
    {
        /// <summary>
        /// 计算两个位姿之间的误差
        /// 
        /// 计算公式：
        ///   位置误差: ΔP = P_target - P_current
        ///   旋转误差: ΔQ = Q_target * Q_current^(-1)
        ///   旋转矢量: ΔR = QuaternionToRotationVector(ΔQ)
        /// 
        /// 注意事项：
        /// - 位置误差是简单的向量差（欧几里得空间）
        /// - 旋转误差是四元数差（李群空间），然后转为旋转矢量
        /// - 输入的两个位姿必须在同一坐标系下
        /// </summary>
        /// <param name="targetPosition">目标位置（米）</param>
        /// <param name="targetRotation">目标旋转（四元数）</param>
        /// <param name="currentPosition">当前位置（米）</param>
        /// <param name="currentRotation">当前旋转（四元数）</param>
        /// <returns>位姿误差对象</returns>
        public static PoseError CalculateError(
            Vector3 targetPosition, Quaternion targetRotation,
            Vector3 currentPosition, Quaternion currentRotation)
        {
            // ========== 步骤1: 计算位置误差 ==========
            // 简单的向量相减
            Vector3 positionError = targetPosition - currentPosition;

            // ========== 步骤2: 计算旋转误差 ==========
            // 四元数差: ΔQ = Q_target * Q_current^(-1)
            // 物理意义: 从当前姿态旋转到目标姿态需要的增量旋转
            Quaternion rotationError = targetRotation * Quaternion.Inverse(currentRotation);

            // ========== 步骤3: 转换为旋转矢量 ==========
            // 将四元数误差转换为轴角表示（旋转矢量）
            // 使用旋转矩阵方法，避免四元数符号歧义导致的方向反转问题
            Vector3 rotationVectorError = QuaternionToRotationVector_Robust(rotationError);

            // ========== 步骤4: 计算误差范数 ==========
            return new PoseError
            {
                positionError = positionError,
                rotationError = rotationVectorError,
                positionMagnitude = positionError.magnitude,        // 位置误差范数（米）
                rotationMagnitude = rotationVectorError.magnitude   // 旋转误差角度（弧度）
            };
        }

        /// <summary>
        /// 四元数转旋转矢量（轴角表示）
        /// 
        /// 公式:
        ///   θ = 2 * arccos(qw)
        ///   k = (qx, qy, qz) / sin(θ/2)
        ///   r = θ * k
        /// 
        /// 特殊情况处理:
        /// - θ ≈ 0: 无旋转，返回零向量
        /// - θ ≈ π: 180度旋转，轴向量从四元数虚部提取
        /// - 一般情况: 使用标准公式
        /// 
        /// 符号规范化:
        /// - 强制 q.w >= 0，确保角度在 [0, π] 范围内
        /// - 避免 q 和 -q 表示同一旋转的歧义
        /// </summary>
        /// <param name="q">输入四元数</param>
        /// <returns>旋转向量（轴*角度，单位：弧度）</returns>
        public static Vector3 QuaternionToRotationVector(Quaternion q)
        {
            // 步骤1: 归一化四元数
            q = NormalizeQuaternion(q);

            // 步骤2: 四元数符号规范化 (强制 q.w >= 0)
            // 因为 q 和 -q 表示相同的旋转，统一使用 w >= 0 的表示
            if (q.w < 0f)
            {
                q.x = -q.x;
                q.y = -q.y;
                q.z = -q.z;
                q.w = -q.w;
            }

            // 步骤3: 计算旋转角度 (现在 q.w 一定在 [0, 1] 范围内)
            float wClamped = Mathf.Clamp(q.w, 0f, 1f);
            float angle = 2f * Mathf.Acos(wClamped);

            // 步骤4: 处理特殊情况

            // 情况A: 接近 180° (q.w ≈ 0)
            // 使用公式: axis = normalize([q.x, q.y, q.z]) * π
            if (angle > Mathf.PI - 1e-4f)
            {
                Vector3 axis180 = new Vector3(q.x, q.y, q.z);
                float axisMag = axis180.magnitude;
                if (axisMag > 1e-8f)
                {
                    axis180 = axis180 / axisMag;
                }
                else
                {
                    // 极端情况: 无法确定轴向，使用默认轴
                    axis180 = new Vector3(1f, 0f, 0f);
                }
                return axis180 * angle;
            }

            // 情况B: 接近 0° (q.w ≈ 1)
            if (angle < 1e-6f)
            {
                return Vector3.zero;
            }

            // 情况C: 一般情况 (0° < angle < 180°)
            // 使用公式: rotationVector = [q.x, q.y, q.z] * (angle / sin(angle/2))
            float sinHalfAngle = Mathf.Sin(angle * 0.5f);
            float scale = angle / sinHalfAngle;
            return new Vector3(q.x * scale, q.y * scale, q.z * scale);
        }

        /// <summary>
        /// 旋转矢量转四元数
        /// 
        /// 公式:
        ///   θ = ||r||
        ///   k = r / θ
        ///   q = (cos(θ/2), sin(θ/2) * k)
        /// 
        /// 角度规范化:
        /// - 将角度规范化到 [0, π] 范围
        /// - θ > π: 转换为 2π-θ 绕反向轴旋转
        /// - θ < 0: 转换为 -θ 绕反向轴旋转
        /// </summary>
        /// <param name="r">旋转向量（轴*角度，单位：弧度）</param>
        /// <returns>对应的四元数</returns>
        public static Quaternion RotationVectorToQuaternion(Vector3 r)
        {
            // 计算旋转角度
            float theta = r.magnitude;
            if (theta < 1e-8f) return Quaternion.identity;

            // 提取旋转轴
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

            // 构造四元数
            float half = theta * 0.5f;
            float s = Mathf.Sin(half);
            return new Quaternion(axis.x * s, axis.y * s, axis.z * s, Mathf.Cos(half));
        }

        /// <summary>
        /// 计算两个位置之间的欧几里得距离
        /// </summary>
        /// <param name="target">目标位置（米）</param>
        /// <param name="current">当前位置（米）</param>
        /// <returns>位置误差范数（米）</returns>
        public static float CalculatePositionError(Vector3 target, Vector3 current)
        {
            return (target - current).magnitude;
        }

        /// <summary>
        /// 计算两个旋转之间的角度差（弧度）
        /// 
        /// 使用四元数点积方法：
        ///   angle = 2 * arccos(|q1 · q2|)
        /// 
        /// 注意：
        /// - 取点积的绝对值，因为 q 和 -q 表示同一旋转
        /// - 结果范围 [0, π]
        /// </summary>
        /// <param name="target">目标旋转（四元数）</param>
        /// <param name="current">当前旋转（四元数）</param>
        /// <returns>旋转误差角度（弧度）</returns>
        public static float CalculateRotationError(Quaternion target, Quaternion current)
        {
            // 归一化输入四元数
            target = NormalizeQuaternion(target);
            current = NormalizeQuaternion(current);

            // 计算点积（四元数内积）
            float dot = Mathf.Abs(
                target.x * current.x +
                target.y * current.y +
                target.z * current.z +
                target.w * current.w
            );

            // 限制在 [0, 1] 范围内（避免数值误差导致 arccos 参数越界）
            dot = Mathf.Clamp(dot, 0f, 1f);

            // 计算角度
            return 2f * Mathf.Acos(dot);
        }

        /// <summary>
        /// 归一化四元数
        /// </summary>
        /// <param name="q">输入四元数</param>
        /// <returns>归一化后的四元数</returns>
        private static Quaternion NormalizeQuaternion(Quaternion q)
        {
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (mag < 1e-12f) return Quaternion.identity;
            float inv = 1f / mag;
            return new Quaternion(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
        }

        /// <summary>
        /// 四元数转旋转矢量（使用旋转矩阵中间转换，避免符号歧义）
        /// 
        /// 优势：
        /// - 避免四元数 q 和 -q 表示同一旋转的符号歧义
        /// - 数值稳定性更好，特别是在大角度旋转时
        /// - 不需要四元数符号规范化，保持误差方向正确性
        /// 
        /// 算法流程：
        /// 1. 四元数 → 旋转矩阵
        /// 2. 旋转矩阵 → 旋转矢量（轴角表示）
        /// 
        /// 应用场景：
        /// - 视觉伺服误差补偿（防止补偿方向反转）
        /// - 大角度旋转误差计算
        /// </summary>
        /// <param name="q">输入四元数</param>
        /// <returns>旋转向量（轴*角度，单位：弧度）</returns>
        private static Vector3 QuaternionToRotationVector_Robust(Quaternion q)
        {
            // 步骤1: 归一化
            q = NormalizeQuaternion(q);
            
            // 步骤2: 四元数 → 旋转矩阵
            Matrix4x4 R = QuaternionToMatrix(q);
            
            // 步骤3: 旋转矩阵 → 旋转矢量
            return RotationMatrixToRotationVector(R);
        }

        /// <summary>
        /// 四元数转旋转矩阵
        /// 
        /// 使用标准的四元数到旋转矩阵转换公式
        /// R = I + 2*[q_vec]_x*[q_vec]_x + 2*q_w*[q_vec]_x
        /// 
        /// 其中 [q_vec]_x 是四元数虚部的反对称矩阵
        /// </summary>
        /// <param name="q">归一化的四元数</param>
        /// <returns>3x3旋转矩阵（作为4x4矩阵存储）</returns>
        private static Matrix4x4 QuaternionToMatrix(Quaternion q)
        {
            float xx = q.x * q.x;
            float yy = q.y * q.y;
            float zz = q.z * q.z;
            float xy = q.x * q.y;
            float xz = q.x * q.z;
            float yz = q.y * q.z;
            float wx = q.w * q.x;
            float wy = q.w * q.y;
            float wz = q.w * q.z;

            Matrix4x4 m = Matrix4x4.identity;
            m.m00 = 1f - 2f * (yy + zz);
            m.m01 = 2f * (xy - wz);
            m.m02 = 2f * (xz + wy);
            
            m.m10 = 2f * (xy + wz);
            m.m11 = 1f - 2f * (xx + zz);
            m.m12 = 2f * (yz - wx);
            
            m.m20 = 2f * (xz - wy);
            m.m21 = 2f * (yz + wx);
            m.m22 = 1f - 2f * (xx + yy);
            
            return m;
        }

        /// <summary>
        /// 旋转矩阵转旋转矢量
        /// 
        /// 算法：
        /// 1. 从矩阵迹计算旋转角度: θ = arccos((trace-1)/2)
        /// 2. 从反对称部分提取旋转轴: k = (R32-R23, R13-R31, R21-R12) / (2*sin(θ))
        /// 3. 旋转矢量: r = θ * k
        /// 
        /// 特殊情况处理：
        /// - θ ≈ 0: 无旋转，返回零向量
        /// - θ ≈ π: 从对角元素提取旋转轴
        /// </summary>
        /// <param name="m">旋转矩阵</param>
        /// <returns>旋转向量（轴*角度，单位：弧度）</returns>
        private static Vector3 RotationMatrixToRotationVector(Matrix4x4 m)
        {
            // 计算旋转角度: θ = arccos((trace - 1) / 2)
            // trace = R00 + R11 + R22
            float trace = m.m00 + m.m11 + m.m22;
            float angle = Mathf.Acos(Mathf.Clamp((trace - 1f) / 2f, -1f, 1f));

            // 特殊情况1: 角度接近 0（无旋转）
            if (Mathf.Abs(angle) < 1e-6f)
            {
                return Vector3.zero;
            }

            // 特殊情况2: 角度接近 π（180度）
            // 此时 sin(θ) ≈ 0，无法从反对称部分提取轴
            // 使用对角元素最大的方法提取旋转轴
            if (Mathf.Abs(angle - Mathf.PI) < 1e-6f)
            {
                Vector3 axis180;
                if (m.m00 >= m.m11 && m.m00 >= m.m22)
                {
                    // X 轴对应的对角元素最大
                    float x = Mathf.Sqrt((m.m00 + 1f) / 2f);
                    float y = m.m01 / (2f * x);
                    float z = m.m02 / (2f * x);
                    axis180 = new Vector3(x, y, z).normalized;
                }
                else if (m.m11 >= m.m22)
                {
                    // Y 轴对应的对角元素最大
                    float y = Mathf.Sqrt((m.m11 + 1f) / 2f);
                    float x = m.m01 / (2f * y);
                    float z = m.m12 / (2f * y);
                    axis180 = new Vector3(x, y, z).normalized;
                }
                else
                {
                    // Z 轴对应的对角元素最大
                    float z = Mathf.Sqrt((m.m22 + 1f) / 2f);
                    float x = m.m02 / (2f * z);
                    float y = m.m12 / (2f * z);
                    axis180 = new Vector3(x, y, z).normalized;
                }
                return axis180 * angle;
            }

            // 一般情况: 从反对称部分提取旋转轴
            // 旋转矩阵的反对称部分: R - R^T = 2*sin(θ)*[k]_x
            // 其中 [k]_x 是旋转轴 k 的反对称矩阵:
            //     [ 0   -kz   ky ]
            //     [ kz   0   -kx ]
            //     [-ky   kx   0  ]
            // 
            // 因此:
            //   R21 - R12 = 2*sin(θ)*kx
            //   R02 - R20 = 2*sin(θ)*ky
            //   R10 - R01 = 2*sin(θ)*kz
            float s = 2f * Mathf.Sin(angle);
            Vector3 axis = new Vector3(
                (m.m21 - m.m12) / s,  // kx
                (m.m02 - m.m20) / s,  // ky
                (m.m10 - m.m01) / s   // kz
            );

            // 返回旋转向量 r = θ * k
            return axis * angle;
        }
    }
}


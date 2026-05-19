using UnityEngine;
using Valve.VR;

namespace Valve.VR
{
    /// <summary>
    /// 相对位姿监控器 - 监控多个 Moving Tracker 在 Reference Tracker 坐标系下的位姿
    /// 
    /// 功能说明：
    /// - Reference Tracker（参考Tracker）：固定放置的参考锚点，定义局部坐标系
    /// - Moving Trackers（移动Tracker列表）：多个移动的靶点
    /// - 输出：每个 Moving Tracker 在 Reference Tracker 坐标系下的位姿
    /// 
    /// 数据来源：
    /// - 直接通过 OpenVR API 获取设备位姿，不依赖场景中的 GameObject
    /// 
    /// 类比关系：
    /// - Reference Tracker 坐标系 ≈ 绝对位姿模式中的相机坐标系
    /// - Moving Trackers ≈ 被跟踪的多个目标物体
    /// </summary>
    public class SteamVR_RelativePoseMonitor : MonoBehaviour
    {
        [Header("Tracker 设备配置")]
        [Tooltip("Reference Tracker（参考锚点）设备索引")]
        [Range(0, 16)]
        public int referenceTrackerIndex = 2;

        [Tooltip("Moving Trackers（移动靶点）设备索引列表")]
        public int[] movingTrackerIndices = new int[] { 1 };

        [Header("向后兼容")]
        [Tooltip("单个 Moving Tracker 设备索引（已弃用，请使用 movingTrackerIndices）")]
        [Range(0, 16)]
        public int movingTrackerIndex = 1;

        [Header("监控设置")]
        [Tooltip("启用相对位姿监控与日志")]
        public bool enableMonitoring = true;

        [Tooltip("触发日志的相对位移阈值（米）")]
        public float positionChangeThreshold = 0.001f;

        [Tooltip("触发日志的相对旋转阈值（度）")]
        public float rotationChangeThreshold = 0.5f;

        [Tooltip("日志最小打印间隔（秒），0 表示不限频")]
        public float logInterval = 0.1f;

        [Tooltip("打印详细的 4x4 变换矩阵")]
        public bool printMatrix = false;

        [Header("日志格式设置")]
        [Tooltip("输出轴角格式（6个值: X Y Z RX RY RZ），默认为四元数格式（7个值: X Y Z QX QY QZ QW）")]
        public bool outputAxisAngle = false;
        
        [Tooltip("输出欧氏距离（Moving Tracker 到 Reference Tracker 的直线距离）")]
        public bool outputEuclideanDistance = true;

        [Header("状态显示 (只读)")]
        [SerializeField] private bool _referenceTrackerConnected = false;
        [SerializeField] private int _movingTrackersCount = 0;
        [SerializeField] private string _movingTrackersStatus = "";

        // ==================== 内部状态 ====================
        
        // 每个 Moving Tracker 的上次记录位姿
        private class TrackerState
        {
            public Vector3 lastPoseInReference;
            public Quaternion lastRotationInReference;
            public Vector3 lastWorldPosition;
            public Quaternion lastWorldRotation;
            public bool hasLastPose = false;
        }
        
        private System.Collections.Generic.Dictionary<int, TrackerState> _trackerStates = 
            new System.Collections.Generic.Dictionary<int, TrackerState>();
        
        private float _lastLogTime = -Mathf.Infinity;

        // 上次记录的 Reference Tracker 世界坐标
        private Vector3 _lastReferenceWorldPosition;
        private Quaternion _lastReferenceWorldRotation;

        // OpenVR 设备位姿数组
        private TrackedDevicePose_t[] _devicePoses = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
        
        // 位姿数据帧号，用于避免同一帧重复获取
        private int _poseFrameCount = -1;
        
        // 缓存的相对位姿（同一帧内复用）
        private System.Collections.Generic.Dictionary<int, (Vector3 position, Quaternion rotation, bool valid)> _cachedRelativePoses = 
            new System.Collections.Generic.Dictionary<int, (Vector3, Quaternion, bool)>();

        // ==================== 生命周期 ====================

        private void Start()
        {
            if (OpenVR.System == null)
            {
                Debug.LogError("<color=red>[相对位姿] OpenVR 未初始化，无法获取设备位姿</color>");
                enabled = false;
                return;
            }

            // 兼容性处理：如果 movingTrackerIndices 为空或默认值，使用 movingTrackerIndex
            if (movingTrackerIndices == null || movingTrackerIndices.Length == 0)
            {
                movingTrackerIndices = new int[] { movingTrackerIndex };
                Debug.LogWarning($"<color=yellow>[相对位姿] movingTrackerIndices 未设置，使用单个 movingTrackerIndex={movingTrackerIndex}</color>");
            }

            // 初始化每个 Moving Tracker 的状态
            foreach (int index in movingTrackerIndices)
            {
                if (!_trackerStates.ContainsKey(index))
                {
                    _trackerStates[index] = new TrackerState();
                }
            }

            Debug.Log("<color=green>[相对位姿] 已初始化（直接使用 OpenVR API）</color>");
            Debug.Log($"<color=cyan>  Reference Tracker（参考锚点）: Device{referenceTrackerIndex}</color>");
            Debug.Log($"<color=cyan>  Moving Trackers（移动靶点）: {string.Join(", ", System.Array.ConvertAll(movingTrackerIndices, i => $"Device{i}"))}</color>");
            Debug.Log("<color=cyan>  输出：每个 Moving Tracker 在 Reference Tracker 坐标系下的位姿</color>");
        }

        private void Update()
        {
            if (!enableMonitoring || OpenVR.System == null)
                return;

            // 使用帧级缓存获取位姿，避免重复调用 OpenVR
            EnsurePoseDataFresh();

            // 获取 Reference Tracker 位姿
            Vector3 refPosition;
            Quaternion refRotation;
            bool refValid = GetDevicePose(referenceTrackerIndex, out refPosition, out refRotation);

            // 更新状态显示
            _referenceTrackerConnected = refValid;

            if (!refValid)
            {
                _movingTrackersStatus = "Reference Tracker 未连接";
                return;
            }

            // 检查 Reference Tracker 是否移动
            bool referenceChanged = false;
            if (_lastReferenceWorldPosition != Vector3.zero) // 已初始化
            {
                float refPosDelta = Vector3.Distance(_lastReferenceWorldPosition, refPosition);
                float refRotDelta = Quaternion.Angle(_lastReferenceWorldRotation, refRotation);
                referenceChanged = refPosDelta > positionChangeThreshold || refRotDelta > rotationChangeThreshold;
            }

            int connectedCount = 0;
            bool anyChanged = false;  // 标记是否有任何 Tracker 移动
            var logData = new System.Collections.Generic.List<string>();  // 收集所有需要输出的日志

            // 遍历所有 Moving Trackers
            foreach (int movingIndex in movingTrackerIndices)
            {
                Vector3 movPosition;
                Quaternion movRotation;
                bool movValid = GetDevicePose(movingIndex, out movPosition, out movRotation);

                if (!movValid)
                {
                    continue;
                }

                connectedCount++;
                TrackerState state = _trackerStates[movingIndex];

                // 计算相对位姿
                Vector3 movingPoseInReference = Quaternion.Inverse(refRotation) * (movPosition - refPosition);
                Quaternion movingRotationInReference = Quaternion.Inverse(refRotation) * movRotation;

                if (!state.hasLastPose)
                {
                    // 首次获取位姿
                    state.lastPoseInReference = movingPoseInReference;
                    state.lastRotationInReference = movingRotationInReference;
                    state.lastWorldPosition = movPosition;
                    state.lastWorldRotation = movRotation;
                    state.hasLastPose = true;

                    Vector3 positionMm = movingPoseInReference * 1000f;
                    float distanceMm = movingPoseInReference.magnitude * 1000f; // 欧氏距离
                    
                    string firstLog;
                    if (outputAxisAngle)
                    {
                        // 输出轴角格式（可选）
                        Vector3 rotationVector = QuaternionToRotationVector(movingRotationInReference);
                        firstLog = $"  Device{movingIndex}: [{positionMm.x:F2}, {positionMm.y:F2}, {positionMm.z:F2}, {rotationVector.x:F4}, {rotationVector.y:F4}, {rotationVector.z:F4}] (mm, rad)";
                    }
                    else
                    {
                        // 输出四元数格式（默认）
                        firstLog = $"  Device{movingIndex}: [{positionMm.x:F2}, {positionMm.y:F2}, {positionMm.z:F2}, {movingRotationInReference.x:F4}, {movingRotationInReference.y:F4}, {movingRotationInReference.z:F4}, {movingRotationInReference.w:F4}] (mm, quat)";
                    }
                    
                    // 添加欧氏距离
                    if (outputEuclideanDistance)
                    {
                        firstLog += $" | 距离: {distanceMm:F2} mm";
                    }
                    
                    logData.Add(firstLog);
                    anyChanged = true;
                    continue;
                }

                // 检查 Moving Tracker 是否移动
                float movingPosDelta = Vector3.Distance(state.lastWorldPosition, movPosition);
                float movingRotDelta = Quaternion.Angle(state.lastWorldRotation, movRotation);
                bool movingChanged = movingPosDelta > positionChangeThreshold || movingRotDelta > rotationChangeThreshold;

                if (referenceChanged || movingChanged)
                {
                    Vector3 positionMm = movingPoseInReference * 1000f;
                    float distanceMm = movingPoseInReference.magnitude * 1000f; // 欧氏距离
                    
                    string trackerLog;
                    if (outputAxisAngle)
                    {
                        // 输出轴角格式（可选）
                        Vector3 rotationVector = QuaternionToRotationVector(movingRotationInReference);
                        trackerLog = $"  Device{movingIndex}: [{positionMm.x:F2}, {positionMm.y:F2}, {positionMm.z:F2}, {rotationVector.x:F4}, {rotationVector.y:F4}, {rotationVector.z:F4}] (mm, rad)";
                    }
                    else
                    {
                        // 输出四元数格式（默认）
                        trackerLog = $"  Device{movingIndex}: [{positionMm.x:F2}, {positionMm.y:F2}, {positionMm.z:F2}, {movingRotationInReference.x:F4}, {movingRotationInReference.y:F4}, {movingRotationInReference.z:F4}, {movingRotationInReference.w:F4}] (mm, quat)";
                    }
                    
                    // 添加欧氏距离
                    if (outputEuclideanDistance)
                    {
                        trackerLog += $" | 距离: {distanceMm:F2} mm";
                    }
                    
                    logData.Add(trackerLog);
                    anyChanged = true;

                    // 如果需要打印变换矩阵
                    if (printMatrix)
                    {
                        Matrix4x4 poseMatrix = Matrix4x4.TRS(movingPoseInReference, movingRotationInReference, Vector3.one);
                        logData.Add($"  Device{movingIndex} 变换矩阵:\n" + MatrixToString(poseMatrix));
                    }

                    // 更新状态（无论是否输出日志都要更新，避免一直触发）
                    state.lastPoseInReference = movingPoseInReference;
                    state.lastRotationInReference = movingRotationInReference;
                    state.lastWorldPosition = movPosition;
                    state.lastWorldRotation = movRotation;
                }
            }

            // 检查是否需要输出日志
            if (anyChanged && logData.Count > 0)
            {
                bool canLog = (logInterval <= 0f || Time.realtimeSinceStartup - _lastLogTime >= logInterval);
                
                if (canLog)
                {
                    // 合并所有日志为一次输出
                    string header = $"<color=yellow>[相对位姿] Reference=Device{referenceTrackerIndex}:</color>";
                    string combinedLog = header + "\n" + string.Join("\n", logData);
                    Debug.Log(combinedLog);
                    
                    // 更新日志时间
                    _lastLogTime = Time.realtimeSinceStartup;
                }
            }

            // 更新 Reference Tracker 的上次位置
            _lastReferenceWorldPosition = refPosition;
            _lastReferenceWorldRotation = refRotation;

            // 更新状态显示
            _movingTrackersCount = connectedCount;
            _movingTrackersStatus = $"{connectedCount}/{movingTrackerIndices.Length} 已连接";
        }

        // ==================== OpenVR 设备位姿获取 ====================

        /// <summary>
        /// 确保当前帧的位姿数据已获取（避免同一帧重复调用 OpenVR）
        /// </summary>
        private void EnsurePoseDataFresh()
        {
            if (_poseFrameCount != Time.frameCount)
            {
                OpenVR.System.GetDeviceToAbsoluteTrackingPose(
                    ETrackingUniverseOrigin.TrackingUniverseStanding,
                    0f,
                    _devicePoses
                );
                _poseFrameCount = Time.frameCount;
                _cachedRelativePoses.Clear();  // 清除上一帧的缓存
            }
        }

        /// <summary>
        /// 从 OpenVR 获取指定设备的位姿
        /// </summary>
        /// <param name="deviceIndex">设备索引</param>
        /// <param name="position">输出：位置（米）</param>
        /// <param name="rotation">输出：旋转（四元数）</param>
        /// <returns>设备是否有效且已连接</returns>
        private bool GetDevicePose(int deviceIndex, out Vector3 position, out Quaternion rotation)
        {
            position = Vector3.zero;
            rotation = Quaternion.identity;

            if (deviceIndex < 0 || deviceIndex >= _devicePoses.Length)
            {
                return false;
            }

            var pose = _devicePoses[deviceIndex];
            
            // 检查设备是否有效且位姿有效
            if (!pose.bDeviceIsConnected || !pose.bPoseIsValid)
            {
                return false;
            }

            // 从 HmdMatrix34_t 提取位置和旋转
            var mat = pose.mDeviceToAbsoluteTracking;
            position = GetPosition(mat);
            rotation = GetRotation(mat);

            return true;
        }

        /// <summary>
        /// 从 HmdMatrix34_t 提取位置
        /// 注意：保持 OpenVR 原始右手坐标系，不做转换
        /// </summary>
        private Vector3 GetPosition(HmdMatrix34_t mat)
        {
            return new Vector3(mat.m3, mat.m7, mat.m11);
        }

        /// <summary>
        /// 从 HmdMatrix34_t 提取旋转（四元数）
        /// 注意：保持 OpenVR 原始右手坐标系，不做转换
        /// </summary>
        private Quaternion GetRotation(HmdMatrix34_t mat)
        {
            float m00 = mat.m0, m01 = mat.m1, m02 = mat.m2;
            float m10 = mat.m4, m11 = mat.m5, m12 = mat.m6;
            float m20 = mat.m8, m21 = mat.m9, m22 = mat.m10;
            
            float trace = m00 + m11 + m22;
            Quaternion q = new Quaternion();
            
            if (trace > 0f)
            {
                float s = Mathf.Sqrt(trace + 1f) * 2f;
                q.w = 0.25f * s;
                q.x = (m21 - m12) / s;
                q.y = (m02 - m20) / s;
                q.z = (m10 - m01) / s;
            }
            else if (m00 > m11 && m00 > m22)
            {
                float s = Mathf.Sqrt(1f + m00 - m11 - m22) * 2f;
                q.w = (m21 - m12) / s;
                q.x = 0.25f * s;
                q.y = (m01 + m10) / s;
                q.z = (m02 + m20) / s;
            }
            else if (m11 > m22)
            {
                float s = Mathf.Sqrt(1f + m11 - m00 - m22) * 2f;
                q.w = (m02 - m20) / s;
                q.x = (m01 + m10) / s;
                q.y = 0.25f * s;
                q.z = (m12 + m21) / s;
            }
            else
            {
                float s = Mathf.Sqrt(1f + m22 - m00 - m11) * 2f;
                q.w = (m10 - m01) / s;
                q.x = (m02 + m20) / s;
                q.y = (m12 + m21) / s;
                q.z = 0.25f * s;
            }
            
            // 归一化
            float mag = Mathf.Sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
            if (mag > Mathf.Epsilon)
            {
                q.x /= mag; q.y /= mag; q.z /= mag; q.w /= mag;
            }
            
            // 符号规范化（确保 w > 0）
            if (q.w < 0f)
            {
                q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w;
            }
            
            return q;
        }

        // ==================== 公开接口 ====================

        /// <summary>
        /// 获取指定 Moving Tracker 在 Reference Tracker 坐标系下的位姿
        /// </summary>
        /// <param name="movingIndex">Moving Tracker 的设备索引</param>
        /// <param name="movingPositionInReference">输出：Moving Tracker 在 Reference 坐标系下的位置（米）</param>
        /// <param name="movingRotationInReference">输出：Moving Tracker 在 Reference 坐标系下的旋转（四元数）</param>
        /// <returns>是否成功获取</returns>
        public bool GetMovingTrackerPoseInReference(int movingIndex, out Vector3 movingPositionInReference, out Quaternion movingRotationInReference)
        {
            movingPositionInReference = Vector3.zero;
            movingRotationInReference = Quaternion.identity;

            if (OpenVR.System == null)
            {
                Debug.LogError("[相对位姿] OpenVR 未初始化");
                return false;
            }

            // 使用帧级缓存，避免同一帧重复调用 OpenVR
            EnsurePoseDataFresh();
            
            // 检查是否有该 Tracker 的缓存
            if (_cachedRelativePoses.TryGetValue(movingIndex, out var cached))
            {
                movingPositionInReference = cached.position;
                movingRotationInReference = cached.rotation;
                return cached.valid;
            }

            // 获取两个设备的位姿
            Vector3 refPos, movPos;
            Quaternion refRot, movRot;

            if (!GetDevicePose(referenceTrackerIndex, out refPos, out refRot))
            {
                Debug.LogError($"[相对位姿] 无法获取 Reference Tracker (Device{referenceTrackerIndex}) 位姿");
                _cachedRelativePoses[movingIndex] = (Vector3.zero, Quaternion.identity, false);
                return false;
            }

            if (!GetDevicePose(movingIndex, out movPos, out movRot))
            {
                Debug.LogError($"[相对位姿] 无法获取 Moving Tracker (Device{movingIndex}) 位姿");
                _cachedRelativePoses[movingIndex] = (Vector3.zero, Quaternion.identity, false);
                return false;
            }

            // 计算相对位姿
            movingPositionInReference = Quaternion.Inverse(refRot) * (movPos - refPos);
            movingRotationInReference = Quaternion.Inverse(refRot) * movRot;
            
            // 缓存结果
            _cachedRelativePoses[movingIndex] = (movingPositionInReference, movingRotationInReference, true);

            return true;
        }

        /// <summary>
        /// 获取第一个 Moving Tracker 在 Reference Tracker 坐标系下的位姿（兼容旧接口）
        /// </summary>
        public bool GetMovingTrackerPoseInReference(out Vector3 movingPositionInReference, out Quaternion movingRotationInReference)
        {
            int targetIndex = (movingTrackerIndices != null && movingTrackerIndices.Length > 0) 
                ? movingTrackerIndices[0] 
                : movingTrackerIndex;
            
            return GetMovingTrackerPoseInReference(targetIndex, out movingPositionInReference, out movingRotationInReference);
        }

        /// <summary>
        /// 获取相对位姿（兼容旧接口）
        /// </summary>
        public bool GetRelativePose(out Vector3 relativePosition, out Quaternion relativeRotation)
        {
            return GetMovingTrackerPoseInReference(out relativePosition, out relativeRotation);
        }

        /// <summary>
        /// 获取指定 Moving Tracker 在 Reference Tracker 坐标系下的位姿矩阵
        /// </summary>
        public bool GetMovingTrackerPoseMatrixInReference(int movingIndex, out Matrix4x4 poseMatrix)
        {
            poseMatrix = Matrix4x4.identity;

            Vector3 position;
            Quaternion rotation;
            if (!GetMovingTrackerPoseInReference(movingIndex, out position, out rotation))
            {
                return false;
            }

            poseMatrix = Matrix4x4.TRS(position, rotation, Vector3.one);
            return true;
        }

        /// <summary>
        /// 获取第一个 Moving Tracker 在 Reference Tracker 坐标系下的位姿矩阵（兼容旧接口）
        /// </summary>
        public bool GetMovingTrackerPoseMatrixInReference(out Matrix4x4 poseMatrix)
        {
            int targetIndex = (movingTrackerIndices != null && movingTrackerIndices.Length > 0) 
                ? movingTrackerIndices[0] 
                : movingTrackerIndex;
            
            return GetMovingTrackerPoseMatrixInReference(targetIndex, out poseMatrix);
        }

        /// <summary>
        /// 获取所有 Moving Trackers 在 Reference Tracker 坐标系下的位姿
        /// </summary>
        public System.Collections.Generic.Dictionary<int, (Vector3 position, Quaternion rotation)> GetAllMovingTrackersPose()
        {
            var result = new System.Collections.Generic.Dictionary<int, (Vector3, Quaternion)>();

            if (movingTrackerIndices == null || movingTrackerIndices.Length == 0)
                return result;

            foreach (int movingIndex in movingTrackerIndices)
            {
                Vector3 position;
                Quaternion rotation;
                if (GetMovingTrackerPoseInReference(movingIndex, out position, out rotation))
                {
                    result[movingIndex] = (position, rotation);
                }
            }

            return result;
        }

        /// <summary>
        /// 获取相对位姿矩阵（兼容旧接口）
        /// </summary>
        public bool GetRelativePoseMatrix(out Matrix4x4 relativeMatrix)
        {
            return GetMovingTrackerPoseMatrixInReference(out relativeMatrix);
        }

        /// <summary>
        /// 检查设备是否已连接
        /// </summary>
        public bool IsDeviceConnected(int deviceIndex)
        {
            if (OpenVR.System == null || deviceIndex < 0 || deviceIndex >= _devicePoses.Length)
                return false;

            // 使用帧级缓存
            EnsurePoseDataFresh();

            return _devicePoses[deviceIndex].bDeviceIsConnected && _devicePoses[deviceIndex].bPoseIsValid;
        }

        // ==================== 辅助方法 ====================

        /// <summary>
        /// 格式化矩阵为可读字符串
        /// </summary>
        private string MatrixToString(Matrix4x4 m)
        {
            return string.Format(
                "[{0:F4}, {1:F4}, {2:F4}, {3:F4}]\n" +
                "[{4:F4}, {5:F4}, {6:F4}, {7:F4}]\n" +
                "[{8:F4}, {9:F4}, {10:F4}, {11:F4}]\n" +
                "[{12:F4}, {13:F4}, {14:F4}, {15:F4}]",
                m.m00, m.m01, m.m02, m.m03,
                m.m10, m.m11, m.m12, m.m13,
                m.m20, m.m21, m.m22, m.m23,
                m.m30, m.m31, m.m32, m.m33
            );
        }

        /// <summary>
        /// 将四元数转换为旋转矢量（轴角表示，单位：弧度）
        /// </summary>
        private Vector3 QuaternionToRotationVector(Quaternion q)
        {
            q.Normalize();
            float angle = 2.0f * Mathf.Acos(Mathf.Clamp(q.w, -1f, 1f));

            if (Mathf.Abs(angle) < 0.0001f)
            {
                return Vector3.zero;
            }

            float sinHalfAngle = Mathf.Sqrt(1.0f - q.w * q.w);
            Vector3 axis;
            
            if (sinHalfAngle < 0.0001f)
            {
                axis = new Vector3(1, 0, 0);
            }
            else
            {
                axis = new Vector3(q.x / sinHalfAngle, q.y / sinHalfAngle, q.z / sinHalfAngle);
            }

            return axis * angle;
        }

        private void OnDestroy()
        {
            if (enableMonitoring && _trackerStates.Count > 0)
            {
                Debug.Log("<color=gray>[相对位姿] 监控已停止</color>");
            }
        }

        /// <summary>
        /// 打印指定 Moving Tracker 的当前相对位姿
        /// </summary>
        public void PrintPose(int movingIndex)
        {
            if (GetMovingTrackerPoseInReference(movingIndex, out Vector3 pos, out Quaternion rot))
            {
                Vector3 posMm = pos * 1000f;
                Vector3 rotVec = QuaternionToRotationVector(rot);
                
                Debug.Log($"[相对位姿] {{Device{movingIndex}在Device{referenceTrackerIndex}坐标系下}}:\n" +
                          $"  位置(m): ({pos.x:F4}, {pos.y:F4}, {pos.z:F4})\n" +
                          $"  位置(mm): ({posMm.x:F2}, {posMm.y:F2}, {posMm.z:F2})\n" +
                          $"  旋转(四元数): (x:{rot.x:F4}, y:{rot.y:F4}, z:{rot.z:F4}, w:{rot.w:F4})\n" +
                          $"  旋转(欧拉角°): ({rot.eulerAngles.x:F2}, {rot.eulerAngles.y:F2}, {rot.eulerAngles.z:F2})\n" +
                          $"  旋转矢量(rad): ({rotVec.x:F4}, {rotVec.y:F4}, {rotVec.z:F4})");
                
                // 根据输出格式设置显示对应的日志格式
                if (outputAxisAngle)
                {
                    Debug.Log($"  <color=cyan>当前日志格式(轴角): [{posMm.x:F2}, {posMm.y:F2}, {posMm.z:F2}, {rotVec.x:F4}, {rotVec.y:F4}, {rotVec.z:F4}] (mm, rad)</color>");
                }
                else
                {
                    Debug.Log($"  <color=cyan>当前日志格式(四元数): [{posMm.x:F2}, {posMm.y:F2}, {posMm.z:F2}, {rot.x:F4}, {rot.y:F4}, {rot.z:F4}, {rot.w:F4}] (mm, quat)</color>");
                }
            }
            else
            {
                Debug.LogWarning($"[相对位姿] 无法获取 Device{movingIndex} 位姿 - 请检查设备连接");
            }
        }

        /// <summary>
        /// 打印所有 Moving Trackers 的当前相对位姿
        /// </summary>
        [ContextMenu("打印所有相对位姿")]
        public void PrintAllPoses()
        {
            if (movingTrackerIndices == null || movingTrackerIndices.Length == 0)
            {
                Debug.LogWarning("[相对位姿] 没有配置 Moving Trackers");
                return;
            }

            Debug.Log($"<color=cyan>[相对位姿] 打印所有 Moving Trackers (共{movingTrackerIndices.Length}个):</color>");
            foreach (int movingIndex in movingTrackerIndices)
            {
                PrintPose(movingIndex);
            }
        }

        /// <summary>
        /// 打印当前相对位姿（兼容旧接口，打印第一个 Moving Tracker）
        /// </summary>
        [ContextMenu("打印当前相对位姿")]
        public void PrintCurrentPose()
        {
            int targetIndex = (movingTrackerIndices != null && movingTrackerIndices.Length > 0) 
                ? movingTrackerIndices[0] 
                : movingTrackerIndex;
            
            PrintPose(targetIndex);
        }

        /// <summary>
        /// 列出所有已连接的设备
        /// </summary>
        [ContextMenu("列出所有设备")]
        public void ListAllDevices()
        {
            if (OpenVR.System == null)
            {
                Debug.LogError("[相对位姿] OpenVR 未初始化");
                return;
            }

            // 使用帧级缓存
            EnsurePoseDataFresh();

            Debug.Log("<color=cyan>[相对位姿] 已连接的设备列表:</color>");
            
            for (int i = 0; i < _devicePoses.Length; i++)
            {
                if (_devicePoses[i].bDeviceIsConnected)
                {
                    var deviceClass = OpenVR.System.GetTrackedDeviceClass((uint)i);
                    string className = deviceClass.ToString();
                    bool poseValid = _devicePoses[i].bPoseIsValid;
                    
                    Debug.Log($"<color=cyan>  Device{i}: {className} | 位姿有效: {poseValid}</color>");
                }
            }
        }
    }
}

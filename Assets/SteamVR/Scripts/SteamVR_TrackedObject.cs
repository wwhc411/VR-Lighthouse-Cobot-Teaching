//======= Copyright (c) Valve Corporation, All rights reserved. ===============
//
// Purpose: For controlling in-game objects with tracked devices.
//
//=============================================================================

using UnityEngine;
using Valve.VR;

namespace Valve.VR
{
    public class SteamVR_TrackedObject : MonoBehaviour
    {
        public enum EIndex
        {
            None = -1,
            Hmd = (int)OpenVR.k_unTrackedDeviceIndex_Hmd,
            Device1,
            Device2,
            Device3,
            Device4,
            Device5,
            Device6,
            Device7,
            Device8,
            Device9,
            Device10,
            Device11,
            Device12,
            Device13,
            Device14,
            Device15,
            Device16
        }

        public EIndex index;

        [Tooltip("If not set, relative to parent")]
        public Transform origin;

        [Header("设备类型")]
        [Tooltip("是否为基站设备（TrackingReference），基站的位姿变化将立即输出，不受限频和阈值限制")]
        public bool isTrackingReference = false;

    [Header("位姿监控（已添加）")]
    [Tooltip("启用位姿变化监控与日志")]
    public bool enablePoseMonitoring = true;

    [Tooltip("触发位姿变化日志的位移阈值（米）")]
    public float positionChangeThreshold = 0.001f;

    [Tooltip("触发位姿变化日志的旋转阈值（度）")]
    public float rotationChangeThreshold = 1.0f;

    [Tooltip("为 true 时打印更详细的位姿事件日志")]
    public bool verbosePoseLogs = false;

    [Tooltip("日志最小打印间隔（秒），小于该间隔的重复日志会被抑制，0 表示不抑制")]
    public float logInterval = 1.0f;

        public bool isValid { get; private set; }

    // 上一次已知的世界空间位姿（用于变化检测）
    private Vector3 _lastWorldPosition;
    private Quaternion _lastWorldRotation;
    private bool _hasLastPose = false;
    // 上一次日志输出时间（秒，使用 realtimeSinceStartup）
    private float _lastLogTime = -Mathf.Infinity;

        private void OnNewPoses(TrackedDevicePose_t[] poses)
        {
            if (index == EIndex.None)
                return;

            var i = (int)index;
            // record previous validity to detect lost/regained tracking
            bool prevIsValid = isValid;

            isValid = false;
            if (poses.Length <= i)
            {
                if (prevIsValid && enablePoseMonitoring)
                    ThrottledLog("<color=red>[SteamVR 跟踪对象] 位姿丢失 - 设备: " + index + " | 时间: " + System.DateTime.Now.ToString("HH:mm:ss.fff") + "</color>");
                return;
            }

            if (!poses[i].bDeviceIsConnected)
            {
                if (prevIsValid && enablePoseMonitoring)
                    ThrottledLog("<color=red>[SteamVR 跟踪对象] 设备断开连接 - 设备: " + index + " | 时间: " + System.DateTime.Now.ToString("HH:mm:ss.fff") + "</color>");
                return;
            }

            if (!poses[i].bPoseIsValid)
            {
                if (prevIsValid && enablePoseMonitoring)
                    ThrottledLog("<color=red>[SteamVR 跟踪对象] 位姿无效 - 设备: " + index + " | 时间: " + System.DateTime.Now.ToString("HH:mm:ss.fff") + "</color>");
                return;
            }

            isValid = true;

            var pose = new SteamVR_Utils.RigidTransform(poses[i].mDeviceToAbsoluteTracking);

            // apply transform
            if (origin != null)
            {
                transform.position = origin.transform.TransformPoint(pose.pos);
                transform.rotation = origin.rotation * pose.rot;
            }
            else
            {
                transform.localPosition = pose.pos;
                transform.localRotation = pose.rot;
            }

            // Pose monitoring: detect regained tracking and significant pose changes
            if (enablePoseMonitoring)
            {
                var currentPos = transform.position;
                var currentRot = transform.rotation;

                // regained tracking
                if (!prevIsValid && isValid)
                {
                    ThrottledLog("<color=green>[SteamVR 跟踪对象] 位姿恢复 - 设备: " + index + " | 时间: " + System.DateTime.Now.ToString("HH:mm:ss.fff") + "</color>");
                    _lastWorldPosition = currentPos;
                    _lastWorldRotation = currentRot;
                    _hasLastPose = true;
                }
                else if (_hasLastPose)
                {
                    float posDelta = Vector3.Distance(_lastWorldPosition, currentPos);
                    float angleDelta = Quaternion.Angle(_lastWorldRotation, currentRot);

                    // 基站设备：立即输出所有位姿变化，不受阈值和限频限制
                    if (isTrackingReference)
                    {
                        if (posDelta > 0.0f || angleDelta > 0.0f)
                        {
                            Debug.Log("<color=cyan>[SteamVR 跟踪对象] 位姿发生变化 - 设备: " + index +
                                      " | Δ位移: " + posDelta.ToString("F4") + " m" +
                                      " | Δ旋转: " + angleDelta.ToString("F2") + "°" +
                                      " | 时间: " + System.DateTime.Now.ToString("HH:mm:ss.fff") + "</color>");

                            _lastWorldPosition = currentPos;
                            _lastWorldRotation = currentRot;
                        }
                    }
                    // 非基站设备：受阈值和限频限制
                    else if (posDelta > positionChangeThreshold || angleDelta > rotationChangeThreshold)
                    {
                        ThrottledLog("<color=yellow>[SteamVR 跟踪对象] 位姿发生变化 - 设备: " + index +
                                  " | Δ位移: " + posDelta.ToString("F4") + " m" +
                                  " | Δ旋转: " + angleDelta.ToString("F2") + "°" +
                                  " | 时间: " + System.DateTime.Now.ToString("HH:mm:ss.fff") + "</color>");

                        _lastWorldPosition = currentPos;
                        _lastWorldRotation = currentRot;
                    }
                    // 如果位姿变化未超过阈值，则不输出任何日志（即使 verbosePoseLogs 为 true）
                }
                else
                {
                    // first time obtaining a pose
                    _lastWorldPosition = currentPos;
                    _lastWorldRotation = currentRot;
                    _hasLastPose = true;
                }
            }
        }

        SteamVR_Events.Action newPosesAction;

        SteamVR_TrackedObject()
        {
            newPosesAction = SteamVR_Events.NewPosesAction(OnNewPoses);
        }

        private void Awake()
        {
            OnEnable();
        }

        void OnEnable()
        {
            var render = SteamVR_Render.instance;
            if (render == null)
            {
                enabled = false;
                return;
            }

            newPosesAction.enabled = true;
        }

        void OnDisable()
        {
            newPosesAction.enabled = false;
            isValid = false;
        }

        public void SetDeviceIndex(int index)
        {
            if (System.Enum.IsDefined(typeof(EIndex), index))
                this.index = (EIndex)index;
        }

        // 限频日志输出：仅当距离上次输出超过 logInterval 秒时才打印
        private void ThrottledLog(string message)
        {
            if (logInterval <= 0f)
            {
                Debug.Log(message);
                return;
            }

            if (Time.realtimeSinceStartup - _lastLogTime >= logInterval)
            {
                Debug.Log(message);
                _lastLogTime = Time.realtimeSinceStartup;
            }
        }
    }
}
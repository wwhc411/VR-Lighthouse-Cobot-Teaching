using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Valve.VR
{
    public class SteamVR_TrackingReferenceManager : MonoBehaviour
    {
        private Dictionary<uint, TrackingReferenceObject> trackingReferences = new Dictionary<uint, TrackingReferenceObject>();

        private void OnEnable()
        {
            SteamVR_Events.NewPoses.AddListener(OnNewPoses);
        }

        private void OnDisable()
        {
            SteamVR_Events.NewPoses.RemoveListener(OnNewPoses);
        }

        private void OnNewPoses(TrackedDevicePose_t[] poses)
        {
            if (poses == null)
                return;

            for (uint deviceIndex = 0; deviceIndex < poses.Length; deviceIndex++)
            {
                if (trackingReferences.ContainsKey(deviceIndex) == false)
                {
                    ETrackedDeviceClass deviceClass = OpenVR.System.GetTrackedDeviceClass(deviceIndex);

                    if (deviceClass == ETrackedDeviceClass.TrackingReference)
                    {
                        TrackingReferenceObject trackingReference = new TrackingReferenceObject();
                        trackingReference.trackedDeviceClass = deviceClass;
                        trackingReference.gameObject = new GameObject("Tracking Reference " + deviceIndex.ToString());
                        trackingReference.gameObject.transform.parent = this.transform;
                        trackingReference.trackedObject = trackingReference.gameObject.AddComponent<SteamVR_TrackedObject>();
                        trackingReference.renderModel = trackingReference.gameObject.AddComponent<SteamVR_RenderModel>();
                        trackingReference.renderModel.createComponents = false;
                        trackingReference.renderModel.updateDynamically = false;

                        trackingReferences.Add(deviceIndex, trackingReference);

                        // 打印绿色日志提示基站位姿标定更新
                        Debug.Log("<color=green>[SteamVR 基站] 基站位姿标定更新 - 设备: " + deviceIndex + 
                                  " | 时间: " + System.DateTime.Now.ToString("HH:mm:ss.fff") + "</color>");

                        trackingReference.gameObject.SendMessage("SetDeviceIndex", (int)deviceIndex, SendMessageOptions.DontRequireReceiver);
                        
                        // 标记为基站设备，使其位姿变化立即输出
                        trackingReference.trackedObject.isTrackingReference = true;
                    }
                    else
                    {
                        trackingReferences.Add(deviceIndex, new TrackingReferenceObject() { trackedDeviceClass = deviceClass });
                    }
                }
            }
        }

        private class TrackingReferenceObject
        {
            public ETrackedDeviceClass trackedDeviceClass;
            public GameObject gameObject;
            public SteamVR_RenderModel renderModel;
            public SteamVR_TrackedObject trackedObject;
        }
    }
}
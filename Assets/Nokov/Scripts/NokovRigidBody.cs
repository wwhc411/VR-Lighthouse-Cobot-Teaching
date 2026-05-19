// ***********************************************************************
// Assembly         : Assembly-CSharp
// Author           : duguguang
// Created          : 05-10-2020
//
// Last Modified By : duguguang
// Last Modified On : 07-25-2020
// ***********************************************************************
// <copyright file="NokovRigidBody.cs" company="Nokov">
//     Copyright (c) Nokov. All rights reserved.
// </copyright>
// <summary>Rigidbody Script</summary>
// ***********************************************************************

using System;
using UnityEngine;
using System.IO;

public static class RotationSaver 
{
    public static void SaveRotation(Quaternion rotation)
    {
        var filePath = Path.Combine(Application.persistentDataPath, "rotation.txt");
        string rotationString = $"{rotation.x},{rotation.y},{rotation.z},{rotation.w}";
        File.WriteAllText(filePath, rotationString);
    }
}

public class NokovRigidBody : MonoBehaviour
    {
        public StreamingClient StreamingClient;
        // public Int32 RigidBodyId;
        public String RigidBodyName;
        public TrackingType trackingType;
        public Quaternion baseQuat;
        public bool DataMix;
        private bool isFirstUpdate = true;

        public enum TrackingType
        {
            /// <summary>
            /// With this setting, both the pose's rotation and position will be applied to the parent transform
            /// </summary>
            RotationAndPosition,
            /// <summary>
            /// With this setting, only the pose's rotation will be applied to the parent transform
            /// </summary>
            RotationOnly,
            /// <summary>
            /// With this setting, only the pose's position will be applied to the parent transform
            /// </summary>
            PositionOnly
        }

        public bool TryGetBaseQuat(ref Quaternion q)
        {
            if (!isFirstUpdate)
            {
                q = baseQuat;
                return true;
            }

            return false;
        }

    void Start()
    {
        if ( this.StreamingClient == null )
        {
            this.StreamingClient = StreamingClient.FindDefaultClient();

            if ( this.StreamingClient == null )
            {
                Debug.LogError( GetType().FullName + ": Streaming client not set, and no " + typeof( StreamingClient ).FullName + " components found in scene; disabling this component.", this );
                this.enabled = false;
                return;
            }
        }
    }


#if UNITY_2017_1_OR_NEWER
    void OnEnable()
    {
        Application.onBeforeRender += OnBeforeRender;
    }


    void OnDisable()
    {
        Application.onBeforeRender -= OnBeforeRender;
    }


    void OnBeforeRender()
    {
        UpdatePose();
    }
#endif


    void Update()
    {
        UpdatePose();
    }

    void UpdatePose()
    {
        NokovRigidBodyState rbState = StreamingClient.GetLatestRigidBodyState(RigidBodyName);
        // NokovRigidBodyState rbState = StreamingClient.GetLatestRigidBodyState(RigidBodyId);
        if (rbState != null)
        {
            if ((trackingType == TrackingType.RotationAndPosition ||
                trackingType == TrackingType.RotationOnly))
            {
                this.transform.localRotation = rbState.Pose.Orientation;
            }

                if ((trackingType == TrackingType.RotationAndPosition ||
                    trackingType == TrackingType.PositionOnly))
                {
                    this.transform.localPosition = rbState.Pose.Position;
                }

            if (isFirstUpdate && DataMix)
            {
                baseQuat = rbState.Pose.Orientation; // 假设rbState已经被定义和初始化

                //// 计算从当前旋转到目标旋转的差值
                //Quaternion currentQuat = gameObject.transform.localRotation;
                //Quaternion differenceQuat = baseQuat * Quaternion.Inverse(currentQuat);

                //// 将差值旋转应用到游戏对象上
                //gameObject.transform.Rotate(differenceQuat.eulerAngles, Space.Self);

                RotationSaver.SaveRotation(baseQuat);

                isFirstUpdate = false;
                trackingType = TrackingType.PositionOnly; // 假设TrackingType已经被定义
                gameObject.GetComponent<AudioListener>().enabled = false;
            }

        }
    }
}

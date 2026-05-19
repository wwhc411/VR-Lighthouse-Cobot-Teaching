// ***********************************************************************
// Assembly         : Assembly-CSharp
// Author           : duguguang
// Created          : 05-10-2020
//
// Last Modified By : duguguang
// Last Modified On : 07-23-2020
// ***********************************************************************
// <copyright file="NokovSkeletonAnimator.cs" company="Nokov">
//     Copyright (c) Nokov. All rights reserved.
// </copyright>
// <summary></summary>
// ***********************************************************************

using System;
using System.Collections.Generic;
using UnityEngine;

//[System.Serializable]
//public struct Bones
//{
//    public String Bone1;
//    public String Bone2;
//    public String Bone3;
//}
public class NokovSkeletonAnimator : MonoBehaviour
{
    public StreamingClient StreamingClient;

    public string SkeletonAssetName = "Skeleton1";

    //public Bones Bone;

    public Avatar DestinationAvatar;

    private string[] NeedBoneNames = { "SLTight", "SLShank", "SLFoot",
                                    "SRTight", "SRShank", "SRFoot"};
    #region Private fields
    private HumanPose m_humanPose = new HumanPose();

    private NokovSkeletonDefinition m_skeletonDef;

    private GameObject m_rootObject;

    private Dictionary<Int32, GameObject> m_boneObjectMap;

    private Dictionary<string, string> m_cachedMecanimBoneNameMap = new Dictionary<string, string>();

    private Avatar m_srcAvatar;

    private HumanPoseHandler m_srcPoseHandler;

    private HumanPoseHandler m_destPoseHandler;
    #endregion Private fields

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

        CacheBoneNameMap( this.StreamingClient.BoneNamingConvention, /*this.SkeletonAssetName*/ "" );

        m_skeletonDef = this.StreamingClient.GetSkeletonDefinitionByName( this.SkeletonAssetName );

        if ( m_skeletonDef == null )
        {
            Debug.LogError( GetType().FullName + ": Could not find skeleton definition with the name \"" + this.SkeletonAssetName + "\"", this );
            this.enabled = false;
            return;
        }

        string rootObjectName = "Nokov Skeleton - " + this.SkeletonAssetName;
        m_rootObject = new GameObject( rootObjectName );
        m_rootObject.transform.position = Vector3.zero;
        m_rootObject.transform.rotation = Quaternion.identity;

        m_boneObjectMap = new Dictionary<Int32, GameObject>( m_skeletonDef.Bones.Count );

        for ( int boneDefIdx = 0; boneDefIdx < m_skeletonDef.Bones.Count; ++boneDefIdx )
        {
            NokovSkeletonDefinition.BoneDefinition boneDef = m_skeletonDef.Bones[boneDefIdx];

            GameObject boneObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
            boneObject.name = boneDef.Name;
            boneObject.SetActive(false);

            // Root Bone
            if (boneDef.ParentId == 0)
            {
                boneObject.transform.parent = m_rootObject.transform;
            }
            else
            {
                boneObject.transform.parent = m_boneObjectMap[boneDef.ParentId].transform;
            }

            boneObject.transform.localPosition = boneDef.Offset;
            //boneObject.transform.localRotation = Quaternion.identity;
            boneObject.transform.localRotation = boneDef.orientation;

            m_boneObjectMap[boneDef.Id] = boneObject;
        }

        MecanimSetup( rootObjectName );
    }


    void Update()
    {
        if(StreamingClient.GetNeedRefreshSkeleton())
        {
            m_skeletonDef = this.StreamingClient.GetSkeletonDefinitionByName(this.SkeletonAssetName);
            StreamingClient.SetNeedRefreshSkeleton();
        }

        if (m_skeletonDef == null)
            return;

        NokovSkeletonState skelState = StreamingClient.GetLatestSkeletonState( m_skeletonDef.Id );
        if ( skelState != null )
        {
            for ( int i = 0; i < m_skeletonDef.Bones.Count; ++i )
            {
                Int32 boneId = m_skeletonDef.Bones[i].Id;

                NokovPose bonePose = new NokovPose();
                GameObject boneObject;

                bool foundPose = false;

                if (StreamingClient.SkeletonCoordinate.Local == this.StreamingClient.Csys)
                {
                    foundPose = skelState.LocalBonePoses.TryGetValue(boneId, out bonePose);                          
                }
                else
                {
                    foundPose = skelState.BonePoses.TryGetValue(boneId, out bonePose);
                }
                
                bool foundObject = m_boneObjectMap.TryGetValue(boneId, out boneObject);
                if (foundPose && foundObject)
                {
                    if (StreamingClient.SkeletonCoordinate.World == this.StreamingClient.Csys)
                    {
                        boneObject.transform.position = bonePose.Position;
                        boneObject.transform.rotation = bonePose.Orientation;
                    }
                    else
                    {

                        boneObject.transform.localPosition = bonePose.Position;
                        boneObject.transform.localRotation = bonePose.Orientation;
                    }
                }
            }


            if (m_srcPoseHandler != null && m_destPoseHandler != null)
            {
                m_srcPoseHandler.GetHumanPose(ref m_humanPose);
                m_destPoseHandler.SetHumanPose(ref m_humanPose);
            }
        }
    }

    #region Private methods
    private void MecanimSetup( string rootObjectName)
    {
        string[] humanTraitBoneNames = HumanTrait.BoneName;

        List<HumanBone> humanBones = new List<HumanBone>(m_skeletonDef.Bones.Count);
        for (int humanBoneNameIdx = 0; humanBoneNameIdx < humanTraitBoneNames.Length; ++humanBoneNameIdx)
        {
            string humanBoneName = humanTraitBoneNames[humanBoneNameIdx];
            if (m_cachedMecanimBoneNameMap.ContainsKey(humanBoneName))
            {
                HumanBone humanBone = new HumanBone();
                humanBone.humanName = humanBoneName;
                humanBone.boneName = m_cachedMecanimBoneNameMap[humanBoneName];
                humanBone.limit.useDefaultValues = true;

                humanBones.Add(humanBone);
            }
        }

        List<SkeletonBone> skeletonBones = new List<SkeletonBone>(m_skeletonDef.Bones.Count + 1);

        {
            SkeletonBone rootBone = new SkeletonBone();
            rootBone.name = rootObjectName;
            rootBone.position = Vector3.zero;
            rootBone.rotation = Quaternion.identity;
            rootBone.scale = Vector3.one;

            skeletonBones.Add(rootBone);
        }

        for (int boneDefIdx = 0; boneDefIdx < m_skeletonDef.Bones.Count; ++boneDefIdx)
        {
            NokovSkeletonDefinition.BoneDefinition boneDef = m_skeletonDef.Bones[boneDefIdx];

            SkeletonBone skelBone = new SkeletonBone();
            skelBone.name = boneDef.Name;
            skelBone.position = boneDef.Offset;
            skelBone.rotation = boneDef.orientation;
            skelBone.scale = Vector3.one;

            skeletonBones.Add(skelBone);
        }

        HumanDescription humanDesc = new HumanDescription();
        humanDesc.human = humanBones.ToArray();
        humanDesc.skeleton = skeletonBones.ToArray();

        humanDesc.upperArmTwist = 0.5f;
        humanDesc.lowerArmTwist = 0.5f;
        humanDesc.upperLegTwist = 0.5f;
        humanDesc.lowerLegTwist = 0.5f;
        humanDesc.armStretch = 0.05f;
        humanDesc.legStretch = 0.05f;
        humanDesc.feetSpacing = 0.0f;
        humanDesc.hasTranslationDoF = true;

        m_srcAvatar = AvatarBuilder.BuildHumanAvatar(m_rootObject, humanDesc);

        if (m_srcAvatar.isValid == false || m_srcAvatar.isHuman == false)
        {
            Debug.LogError(GetType().FullName + ": Unable to create source Avatar for retargeting. Check that your Skeleton Asset Name and Bone Naming Convention are configured correctly.", this);
            this.enabled = false;
            return;
        }

        m_srcPoseHandler = new HumanPoseHandler(m_srcAvatar, m_rootObject.transform);
        m_destPoseHandler = new HumanPoseHandler(DestinationAvatar, this.transform);
    }


    private void CacheBoneNameMap( NokovBoneNameConvention convention, string assetName )
    {
        m_cachedMecanimBoneNameMap.Clear();

        switch ( convention )
        {
            case NokovBoneNameConvention.MARKERLESS:
                m_cachedMecanimBoneNameMap.Add("Hips", "Hips");
                m_cachedMecanimBoneNameMap.Add("Spine", "Spine");
                m_cachedMecanimBoneNameMap.Add("Head", "Head");

                m_cachedMecanimBoneNameMap.Add("LeftShoulder", "LeftShoulder");
                m_cachedMecanimBoneNameMap.Add("LeftUpperArm", "LeftArm");
                m_cachedMecanimBoneNameMap.Add("LeftLowerArm", "LeftForeArm");
                m_cachedMecanimBoneNameMap.Add("LeftHand", "LeftHand");

                m_cachedMecanimBoneNameMap.Add("RightShoulder", "RightShoulder");
                m_cachedMecanimBoneNameMap.Add("RightUpperArm", "RightArm");
                m_cachedMecanimBoneNameMap.Add("RightLowerArm", "RightForeArm");
                m_cachedMecanimBoneNameMap.Add("RightHand", "RightHand");

                m_cachedMecanimBoneNameMap.Add("LeftUpperLeg", "LeftUpLeg");
                m_cachedMecanimBoneNameMap.Add("LeftLowerLeg", "LeftLeg");
                m_cachedMecanimBoneNameMap.Add("LeftFoot", "LeftFoot");

                m_cachedMecanimBoneNameMap.Add("RightUpperLeg", "RightUpLeg");
                m_cachedMecanimBoneNameMap.Add("RightLowerLeg", "RightLeg");
                m_cachedMecanimBoneNameMap.Add("RightFoot", "RightFoot");
                break;

            case NokovBoneNameConvention.VRBody:
                m_cachedMecanimBoneNameMap.Add("Hips", "Hips");
                m_cachedMecanimBoneNameMap.Add("Spine", "Spine");
                m_cachedMecanimBoneNameMap.Add("Spine2", "Spine2");
                m_cachedMecanimBoneNameMap.Add("Neck", "Neck");
                m_cachedMecanimBoneNameMap.Add("Head", "Head");

                m_cachedMecanimBoneNameMap.Add("LeftShoulder", "LeftShoulder");
                m_cachedMecanimBoneNameMap.Add("LeftUpperArm", "LeftArm");
                m_cachedMecanimBoneNameMap.Add("LeftLowerArm", "LeftForeArm");
                m_cachedMecanimBoneNameMap.Add("LeftHand", "LeftHand");

                m_cachedMecanimBoneNameMap.Add("RightShoulder", "RightShoulder");
                m_cachedMecanimBoneNameMap.Add("RightUpperArm", "RightArm");
                m_cachedMecanimBoneNameMap.Add("RightLowerArm", "RightForeArm");
                m_cachedMecanimBoneNameMap.Add("RightHand", "RightHand");

                m_cachedMecanimBoneNameMap.Add("LeftUpperLeg", "LeftUpLeg");
                m_cachedMecanimBoneNameMap.Add("LeftLowerLeg", "LeftLeg");
                m_cachedMecanimBoneNameMap.Add("LeftFoot", "LeftFoot");
                m_cachedMecanimBoneNameMap.Add("LeftToeBase", "LeftToeBase");

                m_cachedMecanimBoneNameMap.Add("RightUpperLeg", "RightUpLeg");
                m_cachedMecanimBoneNameMap.Add("RightLowerLeg", "RightLeg");
                m_cachedMecanimBoneNameMap.Add("RightFoot", "RightFoot");
                m_cachedMecanimBoneNameMap.Add("RightToeBase", "RightToeBase");
                break;

            case NokovBoneNameConvention.XingYing:
                m_cachedMecanimBoneNameMap.Add("Hips", "Hips");
                m_cachedMecanimBoneNameMap.Add("Spine", "Spine");
                m_cachedMecanimBoneNameMap.Add("Chest", "Spine1");
                m_cachedMecanimBoneNameMap.Add("UpperChest", "Spine2");
                m_cachedMecanimBoneNameMap.Add("Neck", "Neck");
                m_cachedMecanimBoneNameMap.Add("Head", "Head");

                m_cachedMecanimBoneNameMap.Add("LeftShoulder", "LeftShoulder");
                m_cachedMecanimBoneNameMap.Add("LeftUpperArm", "LeftArm");
                m_cachedMecanimBoneNameMap.Add("LeftLowerArm", "LeftForeArm");
                m_cachedMecanimBoneNameMap.Add("LeftHand", "LeftHand");

                m_cachedMecanimBoneNameMap.Add("RightShoulder", "RightShoulder");
                m_cachedMecanimBoneNameMap.Add("RightUpperArm", "RightArm");
                m_cachedMecanimBoneNameMap.Add("RightLowerArm", "RightForeArm");
                m_cachedMecanimBoneNameMap.Add("RightHand", "RightHand");

                m_cachedMecanimBoneNameMap.Add("LeftUpperLeg", "LeftUpLeg");
                m_cachedMecanimBoneNameMap.Add("LeftLowerLeg", "LeftLeg");
                m_cachedMecanimBoneNameMap.Add("LeftFoot", "LeftFoot");
                m_cachedMecanimBoneNameMap.Add("LeftToeBase", "LeftToeBase");

                m_cachedMecanimBoneNameMap.Add("RightUpperLeg", "RightUpLeg");
                m_cachedMecanimBoneNameMap.Add("RightLowerLeg", "RightLeg");
                m_cachedMecanimBoneNameMap.Add("RightFoot", "RightFoot");
                m_cachedMecanimBoneNameMap.Add("RightToeBase", "RightToeBase");
                break;

            case NokovBoneNameConvention.XingYingWithHand:
                m_cachedMecanimBoneNameMap.Add("Hips", "Hips");
                m_cachedMecanimBoneNameMap.Add("Spine", "Spine");
                m_cachedMecanimBoneNameMap.Add("Chest", "Spine1");
                m_cachedMecanimBoneNameMap.Add("UpperChest", "Spine2");
                m_cachedMecanimBoneNameMap.Add("Neck", "Neck");
                m_cachedMecanimBoneNameMap.Add("Head", "Head");

                m_cachedMecanimBoneNameMap.Add("LeftShoulder", "LeftShoulder");
                m_cachedMecanimBoneNameMap.Add("LeftUpperArm", "LeftArm");
                m_cachedMecanimBoneNameMap.Add("LeftLowerArm", "LeftForeArm");
                m_cachedMecanimBoneNameMap.Add("LeftHand", "LeftHand");

                m_cachedMecanimBoneNameMap.Add("RightShoulder", "RightShoulder");
                m_cachedMecanimBoneNameMap.Add("RightUpperArm", "RightArm");
                m_cachedMecanimBoneNameMap.Add("RightLowerArm", "RightForeArm");
                m_cachedMecanimBoneNameMap.Add("RightHand", "RightHand");

                m_cachedMecanimBoneNameMap.Add("LeftUpperLeg", "LeftUpLeg");
                m_cachedMecanimBoneNameMap.Add("LeftLowerLeg", "LeftLeg");
                m_cachedMecanimBoneNameMap.Add("LeftFoot", "LeftFoot");
                m_cachedMecanimBoneNameMap.Add("LeftToeBase", "LeftToeBase");

                m_cachedMecanimBoneNameMap.Add("RightUpperLeg", "RightUpLeg");
                m_cachedMecanimBoneNameMap.Add("RightLowerLeg", "RightLeg");
                m_cachedMecanimBoneNameMap.Add("RightFoot", "RightFoot");
                m_cachedMecanimBoneNameMap.Add("RightToeBase", "RightToeBase");

                m_cachedMecanimBoneNameMap.Add("Left Thumb Proximal", "LeftHandThumb1");
                m_cachedMecanimBoneNameMap.Add("Left Thumb Intermediate", "LeftHandThumb2");
                m_cachedMecanimBoneNameMap.Add("Left Thumb Distal", "LeftHandThumb3");
                m_cachedMecanimBoneNameMap.Add("Right Thumb Proximal", "RightHandThumb1");
                m_cachedMecanimBoneNameMap.Add("Right Thumb Intermediate", "RightHandThumb2");
                m_cachedMecanimBoneNameMap.Add("Right Thumb Distal", "RightHandThumb3");

                m_cachedMecanimBoneNameMap.Add("Left Index Proximal", "LeftHandIndex1");
                m_cachedMecanimBoneNameMap.Add("Left Index Intermediate", "LeftHandIndex2");
                m_cachedMecanimBoneNameMap.Add("Left Index Distal", "LeftHandIndex3");
                m_cachedMecanimBoneNameMap.Add("Right Index Proximal", "RightHandIndex1");
                m_cachedMecanimBoneNameMap.Add("Right Index Intermediate", "RightHandIndex2");
                m_cachedMecanimBoneNameMap.Add("Right Index Distal", "RightHandIndex3");

                m_cachedMecanimBoneNameMap.Add("Left Middle Proximal", "LeftHandMiddle1");
                m_cachedMecanimBoneNameMap.Add("Left Middle Intermediate", "LeftHandMiddle2");
                m_cachedMecanimBoneNameMap.Add("Left Middle Distal", "LeftHandMiddle3");
                m_cachedMecanimBoneNameMap.Add("Right Middle Proximal", "RightHandMiddle1");
                m_cachedMecanimBoneNameMap.Add("Right Middle Intermediate", "RightHandMiddle2");
                m_cachedMecanimBoneNameMap.Add("Right Middle Distal", "RightHandMiddle3");

                m_cachedMecanimBoneNameMap.Add("Left Ring Proximal", "LeftHandRing1");
                m_cachedMecanimBoneNameMap.Add("Left Ring Intermediate", "LeftHandRing2");
                m_cachedMecanimBoneNameMap.Add("Left Ring Distal", "LeftHandRing3");
                m_cachedMecanimBoneNameMap.Add("Right Ring Proximal", "RightHandRing1");
                m_cachedMecanimBoneNameMap.Add("Right Ring Intermediate", "RightHandRing2");
                m_cachedMecanimBoneNameMap.Add("Right Ring Distal", "RightHandRing3");

                m_cachedMecanimBoneNameMap.Add("Left Little Proximal", "LeftHandPinky1");
                m_cachedMecanimBoneNameMap.Add("Left Little Intermediate", "LeftHandPinky2");
                m_cachedMecanimBoneNameMap.Add("Left Little Distal", "LeftHandPinky3");
                m_cachedMecanimBoneNameMap.Add("Right Little Proximal", "RightHandPinky1");
                m_cachedMecanimBoneNameMap.Add("Right Little Intermediate", "RightHandPinky2");
                m_cachedMecanimBoneNameMap.Add("Right Little Distal", "RightHandPinky3");
                break;

            case NokovBoneNameConvention.NoConvention:
                m_cachedMecanimBoneNameMap.Add("Hips", "Hips");
                m_cachedMecanimBoneNameMap.Add("Spine", "Spine");
                m_cachedMecanimBoneNameMap.Add("Head", "Head");
                m_cachedMecanimBoneNameMap.Add("Chest", "Chest");
                m_cachedMecanimBoneNameMap.Add("UpperChest", "UpperChest");

                m_cachedMecanimBoneNameMap.Add("LeftShoulder", "LeftShoulder");
                m_cachedMecanimBoneNameMap.Add("LeftUpperArm", "LeftUpperArm");
                m_cachedMecanimBoneNameMap.Add("LeftLowerArm", "LeftLowerArm");
                m_cachedMecanimBoneNameMap.Add("LeftHand", "LeftHand");

                m_cachedMecanimBoneNameMap.Add("RightShoulder", "RightShoulder");
                m_cachedMecanimBoneNameMap.Add("RightUpperArm", "RightUpperArm");
                m_cachedMecanimBoneNameMap.Add("RightLowerArm", "RightLowerArm");
                m_cachedMecanimBoneNameMap.Add("RightHand", "RightHand");

                m_cachedMecanimBoneNameMap.Add("LeftUpperLeg", "LeftUpperLeg");
                m_cachedMecanimBoneNameMap.Add("LeftLowerLeg", "LeftLowerLeg");
                m_cachedMecanimBoneNameMap.Add("LeftFoot", "LeftFoot");
                m_cachedMecanimBoneNameMap.Add("LeftToeBase", "LeftToeBase");

                m_cachedMecanimBoneNameMap.Add("RightUpperLeg", "RightUpperLeg");
                m_cachedMecanimBoneNameMap.Add("RightLowerLeg", "RightLowerLeg");
                m_cachedMecanimBoneNameMap.Add("RightFoot", "RightFoot");
                m_cachedMecanimBoneNameMap.Add("RightToeBase", "RightToeBase");

                m_cachedMecanimBoneNameMap.Add("Left Thumb Proximal", "Left Thumb Proximal");
                m_cachedMecanimBoneNameMap.Add("Left Thumb Intermediate", "Left Thumb Intermediate");
                m_cachedMecanimBoneNameMap.Add("Left Thumb Distal", "Left Thumb Distal");
                m_cachedMecanimBoneNameMap.Add("Right Thumb Proximal", "Right Thumb Proximal");
                m_cachedMecanimBoneNameMap.Add("Right Thumb Intermediate", "Right Thumb Intermediate");
                m_cachedMecanimBoneNameMap.Add("Right Thumb Distal", "Right Thumb Distal");

                m_cachedMecanimBoneNameMap.Add("Left Index Proximal", "Left Index Proximal");
                m_cachedMecanimBoneNameMap.Add("Left Index Intermediate", "Left Index Intermediate");
                m_cachedMecanimBoneNameMap.Add("Left Index Distal", "Left Index Distal");
                m_cachedMecanimBoneNameMap.Add("Right Index Proximal", "Right Index Proximal");
                m_cachedMecanimBoneNameMap.Add("Right Index Intermediate", "Right Index Intermediate");
                m_cachedMecanimBoneNameMap.Add("Right Index Distal", "Right Index Distal");

                m_cachedMecanimBoneNameMap.Add("Left Middle Proximal", "Left Middle Proximal");
                m_cachedMecanimBoneNameMap.Add("Left Middle Intermediate", "Left Middle Intermediate");
                m_cachedMecanimBoneNameMap.Add("Left Middle Distal", "Left Middle Distal");
                m_cachedMecanimBoneNameMap.Add("Right Middle Proximal", "Right Middle Proximal");
                m_cachedMecanimBoneNameMap.Add("Right Middle Intermediate", "Right Middle Intermediate");
                m_cachedMecanimBoneNameMap.Add("Right Middle Distal", "Right Middle Distal");

                m_cachedMecanimBoneNameMap.Add("Left Ring Proximal", "Left Ring Proximal");
                m_cachedMecanimBoneNameMap.Add("Left Ring Intermediate", "Left Ring Intermediate");
                m_cachedMecanimBoneNameMap.Add("Left Ring Distal", "Left Ring Distal");
                m_cachedMecanimBoneNameMap.Add("Right Ring Proximal", "Right Ring Proximal");
                m_cachedMecanimBoneNameMap.Add("Right Ring Intermediate", "Right Ring Intermediate");
                m_cachedMecanimBoneNameMap.Add("Right Ring Distal", "Right Ring Distal");

                m_cachedMecanimBoneNameMap.Add("Left Little Proximal", "Left Little Proximal");
                m_cachedMecanimBoneNameMap.Add("Left Little Intermediate", "Left Little Intermediate");
                m_cachedMecanimBoneNameMap.Add("Left Little Distal", "Left Little Distal");
                m_cachedMecanimBoneNameMap.Add("Right Little Proximal", "Right Little Proximal");
                m_cachedMecanimBoneNameMap.Add("Right Little Intermediate", "Right Little Intermediate");
                m_cachedMecanimBoneNameMap.Add("Right Little Distal", "Right Little Distal");
                break;
        }
    }
    #endregion Private methods
}

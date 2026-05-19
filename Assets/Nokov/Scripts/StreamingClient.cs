// ***********************************************************************
// Assembly         : Assembly-CSharp
// Author           : duguguang
// Created          : 05-10-2020
//
// Last Modified By : duguguang
// Last Modified On : 07-24-2020
// ***********************************************************************
// <copyright file="StreamingClient.cs" company="Nokov">
//     Copyright (c) Nokov. All rights reserved.
// </copyright>
// <summary>StreamingClient</summary>
// ***********************************************************************

using System;
using System.Collections.Generic;
using System.Net;
using System.Runtime.InteropServices;
using System.Threading;
using UnityEngine;
using Nokov.SDK;
using CSNokovSDK;
using UnityEngine.Assertions;
using Assert = UnityEngine.Assertions.Assert;
using Utility;
using UnityEngine.Animations;
using System.Net.Sockets;
using System.Net.NetworkInformation;
using System.Text.RegularExpressions;
using UnityEngine.UI;
using System.Collections.Concurrent;
// using Unity.VisualScripting; // 已注释：未使用且在旧版Unity中不存在

public enum NokovBoneNameConvention
{
    XingYing,
    XingYingWithHand,
    MARKERLESS,
    VRBody,
    NoConvention,
}

public class NokovPose
{
    public Vector3 Position;
    public Quaternion Orientation;
}

public class NokovMarkerState
{
    public Vector3 Position;
    public float Size;
    public bool Labeled;
    public Int32 Id;
}


public class NokovRigidBodyState
{
    public NokovHiResTimer.Timestamp DeliveryTimestamp;
    public NokovPose Pose;
}

public class NokovSkeletonState
{
    public Dictionary<Int32, NokovPose> BonePoses;
    public Dictionary<Int32, NokovPose> LocalBonePoses;
}

public class NokovRigidBodyDefinition
{
    public class MarkerDefinition
    {
        public Vector3 Position;
        public Int32 RequiredLabel;
    }

    public Int32 Id;
    public string Name;
   // public List<MarkerDefinition> Markers;
}

public class NokovSkeletonDefinition
{
    public class BoneDefinition
    {
        public Int32 Id;

        public Int32 ParentId;

        public string Name;

        public Vector3 Offset;

        public Quaternion orientation;

        public double length;

    }

    public Int32 Id;

    public string Name;

    public List<BoneDefinition> Bones;

    public Dictionary<Int32, Int32> BoneIdToParentIdMap;
}

public static class NokovHiResTimer
{
    public struct Timestamp
    {
        internal Int64 m_ticks;

        public float AgeSeconds
        {
            get
            {
                return Now().SecondsSince( this );
            }
        }

        public float SecondsSince( Timestamp reference )
        {
            Int64 deltaTicks = m_ticks - reference.m_ticks;
            return deltaTicks / (float)System.Diagnostics.Stopwatch.Frequency;
        }
    }

    public static Timestamp Now()
    {
        return new Timestamp {
            m_ticks = System.Diagnostics.Stopwatch.GetTimestamp()
        };
    }
}


public class LocalNicIpHelper
{
    public enum ADDRESSFAM
    {
        IPv4, IPv6
    }

    public static string GetIP(ADDRESSFAM Addfam = ADDRESSFAM.IPv4)
     {
         if (Addfam == ADDRESSFAM.IPv6 && !Socket.OSSupportsIPv6)
         {
             return null;
         }
         string output = "";
         foreach (NetworkInterface item in NetworkInterface.GetAllNetworkInterfaces())
         {
 #if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
             NetworkInterfaceType _type1 = NetworkInterfaceType.Wireless80211;
             NetworkInterfaceType _type2 = NetworkInterfaceType.Ethernet;
 
             if ((item.NetworkInterfaceType == _type1 || item.NetworkInterfaceType == _type2) && item.OperationalStatus == OperationalStatus.Up)
 #endif 
             {
                 foreach (UnicastIPAddressInformation ip in item.GetIPProperties().UnicastAddresses)
                 {
                     //IPv4
                     if (Addfam == ADDRESSFAM.IPv4)
                     {
                         if (ip.Address.AddressFamily == AddressFamily.InterNetwork)
                         {
                             output = ip.Address.ToString();
                             //Debug.Log("IP:" + output);
                         }
                     }
 
                     //IPv6
                     else if (Addfam == ADDRESSFAM.IPv6)
                     {
                         if (ip.Address.AddressFamily == AddressFamily.InterNetworkV6)                        {
                             output = ip.Address.ToString();
                         }
                     }
                 }
             }
         }
         return output;
     }
}

public class StreamingClient : MonoBehaviour
{
    public enum SkeletonCoordinate
    {
        World,
        Local,
    }

    public enum E_LengthUnit
    {
        M,
        CM,
        MM
    }

    public enum E_UpAxis
    {
        Y,
        Z
    }

    public string ServerIp = "10.1.1.198"; //NokovHostServerIpAddress
    public SkeletonCoordinate Csys = SkeletonCoordinate.World;
    public E_LengthUnit LenthUnit = E_LengthUnit.M;
    public E_UpAxis UpAxis = E_UpAxis.Y;
    //public UInt16 ServerCommandPort = NokovSDKConstants.DefaultCommandPort;
    //public UInt16 ServerDataPort = NokovSDKConstants.DefaultDataPort;
    public bool DrawMarkers = false;
    public bool DrawUnlabeledMarkers = false;
    public NokovBoneNameConvention BoneNamingConvention = NokovBoneNameConvention.XingYing;
    public Text fpsDisplay;
    private ConcurrentQueue<double> fpsValues = new ConcurrentQueue<double>();

    #region Private fields
    private bool m_receivedFrameSinceConnect = false;
    private NokovHiResTimer.Timestamp m_lastFrameDeliveryTimestamp;
    private Coroutine m_connectionHealthCoroutine = null;
    
    // 线程安全标志：用于native回调中判断组件是否仍然活跃（替代this.enabled的检查）
    private volatile bool m_isActiveAndReceiving = false;

    private SDKClient m_client;
    private NokovSDKClient.DataDescriptions m_dataDescs;

    private bool RefreshSkeleton = false;
    private bool RefreshRigid = false;

    private List<NokovRigidBodyDefinition> m_rigidBodyDefinitions = new List<NokovRigidBodyDefinition>();
    private List<NokovSkeletonDefinition> m_skeletonDefinitions = new List<NokovSkeletonDefinition>();

    private Dictionary<Int32, NokovRigidBodyState> m_latestRigidBodyStates = new Dictionary<Int32, NokovRigidBodyState>();

    private Dictionary<Int32, NokovSkeletonState> m_latestSkeletonStates = new Dictionary<Int32, NokovSkeletonState>();

    private Dictionary<Int32, NokovMarkerState> m_latestMarkerStates = new Dictionary<Int32, NokovMarkerState>();

    private Dictionary<Int32, NokovMarkerState> m_latestUnlabeledMarkerStates = new Dictionary<Int32, NokovMarkerState>();

    private Dictionary<Int32, GameObject> m_latestMarkerSpheres = new Dictionary<Int32, GameObject>();

    private Dictionary<Int32, GameObject> m_latestUnlabeledMarkerCubus = new Dictionary<Int32, GameObject>();

    private List<SmoothFramePointArray> m_smoothFramePointArray = new List<SmoothFramePointArray>();
    private List<SmoothFramePointArray> m_smoothFrameRigidPosArray = new List<SmoothFramePointArray>();
    private List<SmoothFramePointArray> m_smoothFrameRigidRotArray = new List<SmoothFramePointArray>();

    private object m_frameDataUpdateLock = new object();
    private int counts = 0;
    private int lastFrameNo = 0;
    private DateTime lastReport;
    private Text versionText = null;
    #endregion Private fields

    public Vector3 NokovToUnityTranslation(float xPos, float yPos, float zPos)
    {
        // Flipping x axis instead. Unity use meter by default but Nokov millimeter
        Vector3 Location = new Vector3(0,0,0);

        if (UpAxis == E_UpAxis.Y)
            Location = new Vector3(-xPos, yPos, zPos);
        else
            Location = new Vector3(-yPos, zPos, xPos);

        switch (LenthUnit)
        {
            case E_LengthUnit.M:
               return Location / 1000;

            case E_LengthUnit.CM:
                return Location / 100;

            case E_LengthUnit.MM:
                return Location;

            default: return Location / 1000;
        }
    }

    // Check Ip 
    public static bool CheckIPFormat(string strJudgeString)
    {
        bool blnTest = false;
        bool _Result = true;

        Regex regex = new Regex("^[0-9]{1,3}.[0-9]{1,3}.[0-9]{1,3}.[0-9]{1,3}$");
        blnTest = regex.IsMatch(strJudgeString);
        if (blnTest == true)
        {
            string[] strTemp = strJudgeString.Split(new char[] { '.' }); 
            int nDotCount = strTemp.Length - 1;
            if (3 == nDotCount)
            {
                for (int i = 0; i < strTemp.Length; i++)
                {
                    if (Convert.ToInt32(strTemp[i]) > 255)
                    {                    
                        Debug.LogError("invalid ip");
                        _Result = false;
                    }
                }
            }
            else
            {
                Debug.LogError("invalid ip");
                _Result = false;
            }
        }
        else
        {
            _Result = false;
        }
        return _Result;
    }


    public static Quaternion NokovToUnityQuaternion(float x, float y, float z, float w ,E_UpAxis upAxis)
    {
        if (upAxis == E_UpAxis.Y)
            return new Quaternion(-x, y, z, -w);
        else
            return new Quaternion(-y, z, x, -w);
    }

    private void Update()
        {
            if (DrawMarkers)
            {
                List<Int32> markerIds = new List<Int32>();
                lock (m_frameDataUpdateLock)
                {
                    // Move existing spheres and create new ones if necessary
                    foreach (KeyValuePair<Int32, NokovMarkerState> markerEntry in m_latestMarkerStates)
                    { 
                        if (m_latestMarkerSpheres.ContainsKey( markerEntry.Key ))
                        {
                            m_latestMarkerSpheres[markerEntry.Key].transform.position = markerEntry.Value.Position;
                        }
                        else
                        {
                            var sphere = GameObject.CreatePrimitive( PrimitiveType.Sphere );
                            sphere.name = "Marker[" + markerEntry.Key + "]";
                            sphere.transform.parent = this.transform;
                            sphere.transform.localScale = new Vector3( markerEntry.Value.Size, markerEntry.Value.Size, markerEntry.Value.Size );
                            sphere.transform.position = markerEntry.Value.Position;
                            
                            m_latestMarkerSpheres[markerEntry.Key] = sphere;
                        }
                        markerIds.Add( markerEntry.Key );
                    }
                    // find spheres to remove that weren't in the previous frame
                    List<Int32> markerSphereIdsToDelete = new List<Int32>();
                    foreach (KeyValuePair<Int32, GameObject> markerSphereEntry in m_latestMarkerSpheres)
                    {
                        if (!markerIds.Contains( markerSphereEntry.Key ))
                        {
                            // stale marker, tag for removal
                            markerSphereIdsToDelete.Add( markerSphereEntry.Key );
                        }
                    }
                    // remove stale spheres
                    foreach(Int32 markerId in markerSphereIdsToDelete)
                    {
                        if(m_latestMarkerSpheres.ContainsKey(markerId))
                        {
                            Destroy( m_latestMarkerSpheres[markerId] );
                            m_latestMarkerSpheres.Remove( markerId );
                        }
                    }
                }
            }
            else
            {
                // not drawing markers, remove all marker spheres
                foreach (KeyValuePair<Int32, GameObject> markerSphereEntry in m_latestMarkerSpheres)
                {
                    Destroy( m_latestMarkerSpheres[markerSphereEntry.Key] );
                }
                m_latestMarkerSpheres.Clear();
            }


            if(DrawUnlabeledMarkers)
            {
            
                List<Int32> unlabeledMarkerIds = new List<Int32>();
                lock (m_frameDataUpdateLock)
                {
                    // Move existing cubes and create new ones if necessary
                    foreach (KeyValuePair<Int32, NokovMarkerState> markerEntry in m_latestUnlabeledMarkerStates)
                    {
                        if (m_latestUnlabeledMarkerCubus.ContainsKey(markerEntry.Key))
                        {
                            m_latestUnlabeledMarkerCubus[markerEntry.Key].transform.position = markerEntry.Value.Position;
                        }
                        else
                        {
                            var cubu = GameObject.CreatePrimitive(PrimitiveType.Cube);
                            cubu.name = "UnlabeledMarker[" + markerEntry.Key + "]";
                            cubu.transform.parent = this.transform;
                            cubu.transform.localScale = new Vector3(markerEntry.Value.Size, markerEntry.Value.Size, markerEntry.Value.Size);
                            cubu.transform.position = markerEntry.Value.Position;

                            Material mat = new Material(Shader.Find("Hidden/OptiTrack/Editor/MarkerShader"));
                            mat.color = new Color(1, 0.43f, 0, 1);
                            cubu.GetComponent<MeshRenderer>().material = mat;

                            m_latestUnlabeledMarkerCubus[markerEntry.Key] = cubu;
                        }
                    unlabeledMarkerIds.Add(markerEntry.Key);
                    }
                    // find cubes to remove that weren't in the previous frame
                    List<Int32> UnlabeledmarkerCubeIdsToDelete = new List<Int32>();
                    foreach (KeyValuePair<Int32, GameObject> UnlabeledmarkerCubeEntry in m_latestUnlabeledMarkerCubus)
                    {
                        if (!unlabeledMarkerIds.Contains(UnlabeledmarkerCubeEntry.Key))
                        {
                            // stale marker, tag for removal
                            UnlabeledmarkerCubeIdsToDelete.Add(UnlabeledmarkerCubeEntry.Key);
                        }
                    }
                    // remove stale cubes
                    foreach (Int32 markerId in UnlabeledmarkerCubeIdsToDelete)
                    {
                        if (m_latestUnlabeledMarkerCubus.ContainsKey(markerId))
                        {
                            Destroy(m_latestUnlabeledMarkerCubus[markerId]);
                            m_latestUnlabeledMarkerCubus.Remove(markerId);
                        }
                    }
                }
            }
            else
            {
                foreach (KeyValuePair<Int32, GameObject> markerSphereEntry in m_latestUnlabeledMarkerCubus)
                {
                    Destroy(m_latestUnlabeledMarkerCubus[markerSphereEntry.Key]);
                }
                m_latestUnlabeledMarkerCubus.Clear();
            }

            CheckNeedRefresh();


        if (fpsValues.TryDequeue(out double fps))
        {
            if (versionText != null)
            {
                versionText.text = fps.ToString("0.00") + " fps";
            }
        }

    }

    public static StreamingClient FindDefaultClient()
    {
        StreamingClient[] allClients = FindObjectsOfType<StreamingClient>();

        if ( allClients.Length == 0 )
        {
            Debug.LogError( "Unable to locate any " + typeof( StreamingClient ).FullName + " components." );
            return null;
        }
        else if ( allClients.Length > 1 )
        {
            Debug.LogWarning( "Multiple " + typeof( StreamingClient ).FullName + " components found in scene; defaulting to first available." );
        }

        return allClients[0];
    }


    public NokovRigidBodyState GetLatestRigidBodyState( String rigidBodyName )
    {
        NokovRigidBodyState rbState = null;
        
        lock ( m_frameDataUpdateLock )
		{
            for (int i = 0; i < m_rigidBodyDefinitions.Count; i++)
            {
                if (m_rigidBodyDefinitions[i].Name == rigidBodyName)
                {
                    m_latestRigidBodyStates.TryGetValue( m_rigidBodyDefinitions[i].Id, out rbState);
                }
            }
		}
        
        return rbState;
    }


    public NokovSkeletonState GetLatestSkeletonState( Int32 skeletonId )
    {
        NokovSkeletonState skelState;

        lock ( m_frameDataUpdateLock )
        {
            m_latestSkeletonStates.TryGetValue( skeletonId, out skelState );
        }

        return skelState;
    }

    public List<NokovMarkerState> GetLatestMarkerStates()
    {
        List<NokovMarkerState> markerStates = new List<NokovMarkerState>();

        lock (m_frameDataUpdateLock)
        {
            foreach (KeyValuePair<Int32, NokovMarkerState> markerEntry in m_latestMarkerStates)
            {
                NokovMarkerState newMarkerState = new NokovMarkerState
                {
                    Position = markerEntry.Value.Position,
                    Labeled = markerEntry.Value.Labeled,
                    Size = markerEntry.Value.Size,
                    Id = markerEntry.Value.Id
                };
                markerStates.Add( newMarkerState );
            }
        }

        return markerStates;
    }

    public List<NokovMarkerState> GetLatestUnlabeledMarkerStates()
    {
        List<NokovMarkerState> UnlabeledMarkerStates = new List<NokovMarkerState>();

        lock (m_frameDataUpdateLock)
        {
            foreach (KeyValuePair<Int32, NokovMarkerState> markerEntry in m_latestUnlabeledMarkerStates)
            {
                NokovMarkerState newMarkerState = new NokovMarkerState
                {
                    Position = markerEntry.Value.Position,
                    Labeled = markerEntry.Value.Labeled,
                    Size = markerEntry.Value.Size,
                    Id = markerEntry.Value.Id
                };
                UnlabeledMarkerStates.Add(newMarkerState);
            }
        }

        return UnlabeledMarkerStates;
    }


    public NokovRigidBodyDefinition GetRigidBodyDefinitionById( Int32 rigidBodyId )
    {
        for ( int i = 0; i < m_rigidBodyDefinitions.Count; ++i )
        {
            NokovRigidBodyDefinition rbDef = m_rigidBodyDefinitions[i];

            if ( rbDef.Id == rigidBodyId )
            {
                return rbDef;
            }
        }

        return null;
    }
    
    public NokovRigidBodyDefinition GetRigidBodyDefinitionByName( String rigidBodyName )
    {
        for ( int i = 0; i < m_rigidBodyDefinitions.Count; ++i )
        {
            NokovRigidBodyDefinition rbDef = m_rigidBodyDefinitions[i];

            if ( rbDef.Name == rigidBodyName )
            {
                return rbDef;
            }
        }

        return null;
    }


    public List<NokovRigidBodyDefinition> GetAllRigidBodyDefinitions()
    {
        return new List<NokovRigidBodyDefinition>(m_rigidBodyDefinitions);
    }
    

    public void UpdateDefinitions()
    {
        NokovSDKClient c = m_client as NokovSDKClient;

        int result = c != null ? 0 : 1;
        NokovSDKException.ThrowIfNotOK(result, "GetClient Failed");

        m_dataDescs = c.GetDataDescriptions();

        m_rigidBodyDefinitions.Clear();
        m_skeletonDefinitions.Clear();

        // Translate rigid body definitions.
        for (int nativeRbDescIdx = 0; nativeRbDescIdx < m_dataDescs.RigidBodyDescriptions.Count; ++nativeRbDescIdx)
        {
            sRigidBodyDescription nativeRb = m_dataDescs.RigidBodyDescriptions[nativeRbDescIdx];

            int actualLength = Array.IndexOf(nativeRb.Name, '\0');
            if (actualLength == -1) actualLength = NokovSDKConstants.MaxNameLength; 

            NokovRigidBodyDefinition rbDef = new NokovRigidBodyDefinition
            {
                Id = nativeRb.Id,
                Name = new string(nativeRb.Name, 0, actualLength),
            };

            m_rigidBodyDefinitions.Add(rbDef);
        }

        // Translate skeleton definitions.
        for (int nativeSkelDescIdx = 0; nativeSkelDescIdx < m_dataDescs.SkeletonDescriptions.Count; ++nativeSkelDescIdx)
        {
            sSkeletonDescription nativeSkel = m_dataDescs.SkeletonDescriptions[nativeSkelDescIdx];

            int actualLength = Array.IndexOf(nativeSkel.Name, '\0');
            if (actualLength == -1) actualLength = NokovSDKConstants.MaxNameLength;

            NokovSkeletonDefinition skelDef = new NokovSkeletonDefinition
            {
                Id = nativeSkel.Id,
                Name = new string(nativeSkel.Name, 0, actualLength),
                Bones = new List<NokovSkeletonDefinition.BoneDefinition>(nativeSkel.RigidBodyCount),
                BoneIdToParentIdMap = new Dictionary<int, int>(),
            };

            for (int nativeBoneIdx = 0; nativeBoneIdx < nativeSkel.RigidBodyCount; ++nativeBoneIdx)
            {
                sRigidBodyDescription nativeBone = nativeSkel.RigidBodies[nativeBoneIdx];

                int actualBoneLength = Array.IndexOf(nativeBone.Name, '\0');
                if (actualBoneLength == -1) actualBoneLength = NokovSDKConstants.MaxNameLength;

                NokovSkeletonDefinition.BoneDefinition boneDef =
                    new NokovSkeletonDefinition.BoneDefinition
                    {
                        Id = nativeBone.Id,
                        ParentId = nativeBone.ParentId,
                        Name = new string(nativeBone.Name, 0, actualBoneLength),
                        Offset = NokovToUnityTranslation(nativeBone.OffsetX, nativeBone.OffsetY, nativeBone.OffsetZ),
                        orientation = NokovToUnityQuaternion(nativeBone.Qx, nativeBone.Qy, nativeBone.Qz, nativeBone.Qw, UpAxis),
                        length = 1.0f,
                    };

                skelDef.Bones.Add(boneDef);
                skelDef.BoneIdToParentIdMap[boneDef.Id] = boneDef.ParentId;
            }

            m_skeletonDefinitions.Add(skelDef);
        }
    }

    public NokovSkeletonDefinition GetSkeletonDefinitionById(Int32 skeletonId)
    {
        for (int i = 0; i < m_skeletonDefinitions.Count; ++i)
        {
            NokovSkeletonDefinition skelDef = m_skeletonDefinitions[i];

            if (skelDef.Id == skeletonId)
            {
                return skelDef;
            }
        }

        return null;
    }


    public NokovSkeletonDefinition GetSkeletonDefinitionByName( string skeletonAssetName )
    {
        for ( int i = 0; i < m_skeletonDefinitions.Count; ++i )
        {
            NokovSkeletonDefinition skelDef = m_skeletonDefinitions[i];
            if (skelDef.Name.Equals(skeletonAssetName, StringComparison.InvariantCultureIgnoreCase)) 
            {
                return skelDef;
            }
        }

        return null;
    }


    void OnEnable()
    {
        try
        {
            if (!CheckIPFormat(ServerIp))
                return;

            if(m_client != null)
            {
                this.enabled = false;
                Debug.LogError("More than one client in scene.", this);
                return;
            }

            m_client = new NokovSDKClient();
            m_client.Connect(ServerIp);
            UpdateDefinitions();

        }
        catch ( Exception ex )
        {
            Debug.LogException( ex, this );
            Debug.LogError( GetType().FullName + ": Error connecting to server; check your configuration, and make sure the server is currently streaming.", this );
            this.enabled = false;
            return;
        }

        NokovSDKClient c = m_client as NokovSDKClient;
        if (null != c)
        {
            NokovSDKClient.NativeFrameReceived += OnNokovSDKFrameReceived;
        }

        // 设置线程安全标志，表示组件已准备好接收数据
        m_isActiveAndReceiving = true;

        m_connectionHealthCoroutine = StartCoroutine(CheckConnectionHealth());

    }


    void OnDisable()
    {
        // 0. 立即设置标志为false，阻止native回调继续处理数据
        m_isActiveAndReceiving = false;

        // 1. 首先停止协程
        if (m_connectionHealthCoroutine != null)
        {
            StopCoroutine(m_connectionHealthCoroutine);
            m_connectionHealthCoroutine = null;
        }

        // 2. 然后注销事件（必须在断开连接之前，防止回调访问已释放资源）
        NokovSDKClient c = m_client as NokovSDKClient;
        if (null != c)
        {
            NokovSDKClient.NativeFrameReceived -= OnNokovSDKFrameReceived;
        }

        // 3. 最后断开连接和释放资源（添加空检查）
        if (m_client != null)
        {
            try
            {
                m_client.DisConnect();
                m_client.Dispose();
            }
            catch (System.Exception ex)
            {
                Debug.LogWarning($"[StreamingClient] OnDisable时断开连接出错: {ex.Message}", this);
            }
            finally
            {
                m_client = null;
            }
        }
    }


    /// <summary>
    /// 应用程序退出时确保清理资源
    /// </summary>
    void OnApplicationQuit()
    {
        // 0. 立即设置标志为false，阻止native回调继续处理数据
        m_isActiveAndReceiving = false;

        // 确保在应用退出前清理所有资源
        if (m_connectionHealthCoroutine != null)
        {
            StopCoroutine(m_connectionHealthCoroutine);
            m_connectionHealthCoroutine = null;
        }

        // 注销事件
        NokovSDKClient c = m_client as NokovSDKClient;
        if (null != c)
        {
            NokovSDKClient.NativeFrameReceived -= OnNokovSDKFrameReceived;
        }

        // 释放客户端资源
        if (m_client != null)
        {
            try
            {
                m_client.DisConnect();
                m_client.Dispose();
            }
            catch
            {
                // 忽略退出时的异常
            }
            m_client = null;
        }
    }


    System.Collections.IEnumerator CheckConnectionHealth()
    {
        const float kHealthCheckIntervalSeconds = 1.0f;
        const float kRecentFrameThresholdSeconds = 5.0f;

        YieldInstruction checkIntervalYield = new WaitForSeconds( kHealthCheckIntervalSeconds );
        NokovHiResTimer.Timestamp connectionInitiatedTimestamp = NokovHiResTimer.Now();
        NokovHiResTimer.Timestamp lastFrameReceivedTimestamp;
        bool wasReceivingFrames = false;
        bool warnedPendingFirstFrame = false;

        while ( true )
        {
            yield return checkIntervalYield;

            if ( m_receivedFrameSinceConnect == false )
            {
                if ( connectionInitiatedTimestamp.AgeSeconds > kRecentFrameThresholdSeconds )
                {
                    if ( warnedPendingFirstFrame == false )
                    {
                        Debug.LogWarning( GetType().FullName + ": No frames received from the server yet. Verify your connection settings are correct and that the server is streaming.", this );
                        warnedPendingFirstFrame = true;
                    }

                    continue;
                }
            }
            else
            {
                lastFrameReceivedTimestamp.m_ticks = Interlocked.Read( ref m_lastFrameDeliveryTimestamp.m_ticks );
                bool receivedRecentFrame = lastFrameReceivedTimestamp.AgeSeconds < kRecentFrameThresholdSeconds;

                if ( wasReceivingFrames == false && receivedRecentFrame == true )
                {
                    wasReceivingFrames = true;
                    Debug.Log( GetType().FullName + ": Receiving streaming data from the server.", this );
                    continue;
                }
                else if ( wasReceivingFrames == true && receivedRecentFrame == false )
                {
                    wasReceivingFrames = false;
                    Debug.LogWarning( GetType().FullName + ": No streaming frames received from the server recently.", this );
                    continue;
                }
            }
        }
    }


    #region Private methods
    private void OnNokovSDKFrameReceived( object sender, NokovSDKClient.NativeFrameReceivedEventArgs eventArgs )
    {
        // 安全检查：使用线程安全的标志变量（不能在非主线程访问this.enabled）
        if (!m_isActiveAndReceiving)
        {
            return;
        }

        if (!Monitor.TryEnter(m_frameDataUpdateLock))
        {
            return;
        }

        try
        {
            ++counts;
            TimeSpan timeSpan = DateTime.Now - lastReport;
            if (timeSpan.TotalSeconds > 1)
            {
                double frequency = counts / timeSpan.TotalSeconds;
                counts = 0;
                lastReport = DateTime.Now;

                fpsValues.Enqueue(frequency);
            }
            // Update health markers.
            m_receivedFrameSinceConnect = true;
            Interlocked.Exchange( ref m_lastFrameDeliveryTimestamp.m_ticks, NokovHiResTimer.Now().m_ticks );

            // Process received frame.
            sFrameOfMocapData pFrame = eventArgs.MarshaledFrame;
            // Frame Of Mocap Data params
            bool bIsRecording = (pFrame.Params & 0x01) != 0;
            bool bTrackedModelsChanged = (pFrame.Params & 0x02) != 0;
            if (bIsRecording)
                print("RECORDING\n");
            if (bTrackedModelsChanged)
                print("Models Changed.\n");

            if (pFrame.FrameNumber == lastFrameNo)
            {
                Debug.LogError(GetType().FullName + ": OnNokovSDKFrameReceivedV2, Get Repeated frame. FrameNumber is " + lastFrameNo, this);
                return;
            }

            //Debug.Log("FrameNo:" + pFrame.FrameNumber);

            lastFrameNo = pFrame.FrameNumber;
            //print("RigibodyCount: " + pFrame.RigidBodyCount);
            // Update rigid bodies.
            for (int rbIdx = 0; rbIdx < pFrame.RigidBodyCount; ++rbIdx)
            {
                sRigidBodyData rbData = pFrame.RigidBodies[rbIdx];
                bool bTrackedThisFrame = (rbData.Params & 0x01) != 0;
                //if (bTrackedThisFrame == false)
                //{
                //    print("InvalidTracedFrame");
                //    continue;
                //}

                // Ensure we have a state corresponding to this rigid body ID.
                NokovRigidBodyState rbState = GetOrCreateRigidBodyState(rbData.Id );

                rbState.DeliveryTimestamp = NokovHiResTimer.Now();
                
                rbState.Pose.Position = NokovToUnityTranslation(rbData.X, rbData.Y, rbData.Z);
                //rbState.Pose.Orientation = new Quaternion(-rbData.QX, rbData.QY, rbData.QZ, -rbData.QW );
                rbState.Pose.Orientation = NokovToUnityQuaternion(rbData.QX, rbData.QY, rbData.QZ, rbData.QW, UpAxis);
            }

            // Update skeletons.
            for (int skelIdx = 0; skelIdx < pFrame.SkeletonCount; ++skelIdx)
            {
                sSkeletonData skelData = pFrame.Skeletons[skelIdx];

                NokovSkeletonState skelState = GetOrCreateSkeletonState(skelData.Id);

                for (int boneIdx = 0; boneIdx < skelData.RigidBodyCount; ++boneIdx)
                {
                    IntPtr ptr = IntPtr.Add(skelData.RigidBodies, Marshal.SizeOf<sRigidBodyData>() * boneIdx);
                    sRigidBodyData boneData = Marshal.PtrToStructure<sRigidBodyData>(ptr);

                    Int32 boneId = boneData.Id;

                    if (skelState.BonePoses.ContainsKey(boneId) == false)
                    {
                        skelState.BonePoses[boneId] = new NokovPose();
                    }
                    if (skelState.LocalBonePoses.ContainsKey(boneId) == false)
                    {
                        skelState.LocalBonePoses[boneId] = new NokovPose();
                    }

                    Vector3 bonePos = NokovToUnityTranslation(boneData.X, boneData.Y, boneData.Z);
                    Quaternion boneOri = NokovToUnityQuaternion(boneData.QX, boneData.QY, boneData.QZ, boneData.QW, UpAxis);

                    if (StreamingClient.SkeletonCoordinate.World == this.Csys)
                    {
                        skelState.BonePoses[boneId].Position = bonePos;
                        skelState.BonePoses[boneId].Orientation = boneOri;
                    }
                    else
                    {
                        skelState.LocalBonePoses[boneId].Position = bonePos;
                        skelState.LocalBonePoses[boneId].Orientation = boneOri;
                    }
                }
            }

            //// Update markers
            m_latestMarkerStates.Clear();
            m_latestUnlabeledMarkerStates.Clear();

           // print("LabeledMarkerCount: " + pFrame.LabeledMarkerCount);

            for (int markerIdx = 0; markerIdx < pFrame.LabeledMarkerCount; ++markerIdx)
            {
                sMarker marker = pFrame.LabeledMarkers[markerIdx];
                // Flip coordinate handedness

                // default UpAxis Y
                NokovMarkerState markerState = GetOrCreateMarkerState(marker.Id);
                markerState.Position = NokovToUnityTranslation(marker.X, marker.Y, marker.Z);
                //markerState.Size = marker.Size;
                markerState.Size = 0.01f;
                markerState.Labeled = (marker.Params & 0x10) == 0;
                markerState.Id = marker.Id;
            }
           
            for (int UnLabeledMarkerIdx = 0; UnLabeledMarkerIdx < pFrame.OtherMarkerCount; ++UnLabeledMarkerIdx)
            {
                IntPtr unlabeledMarkers = IntPtr.Add(pFrame.OtherMarkers, 3 * sizeof(float) * UnLabeledMarkerIdx);

                float[] unlabeledMarker = new float[3];
                Marshal.Copy(unlabeledMarkers, unlabeledMarker, 0, 3);
                NokovMarkerState unlabeledMarkerState = GetOrCreateUnlabeledMarkerState(UnLabeledMarkerIdx);

                unlabeledMarkerState.Position = NokovToUnityTranslation(unlabeledMarker[0], unlabeledMarker[1], unlabeledMarker[2]);
                unlabeledMarkerState.Size = 0.01f;
                unlabeledMarkerState.Id = UnLabeledMarkerIdx;
            }
        }
        catch (Exception ex)
        {
            Debug.LogError( GetType().FullName + ": OnNokovSDKFrameReceived encountered an exception.", this );
            Debug.LogException( ex, this );
        }
        finally
        {
            Monitor.Exit( m_frameDataUpdateLock );
        }
    }

    private NokovRigidBodyState GetOrCreateRigidBodyState( Int32 rigidBodyId )
    {
        NokovRigidBodyState returnedState = null;

        if ( m_latestRigidBodyStates.TryGetValue(rigidBodyId, out var rigidBodyState) )
        {
            returnedState = rigidBodyState;
        }
        else
        {
            NokovRigidBodyState newRbState = new NokovRigidBodyState {
                Pose = new NokovPose(),
            };

            m_latestRigidBodyStates[rigidBodyId] = newRbState;

            returnedState = newRbState;
        }

        return returnedState;
    }


    private NokovSkeletonState GetOrCreateSkeletonState( Int32 skeletonId )
    {
        NokovSkeletonState returnedState = null;

        if ( m_latestSkeletonStates.TryGetValue(skeletonId, out var skeletonState) )
        {
            returnedState = skeletonState;
        }
        else
        {
            NokovSkeletonState newSkeletonState = new NokovSkeletonState {
                BonePoses = new Dictionary<Int32, NokovPose>(),
                LocalBonePoses = new Dictionary<Int32, NokovPose>(),
            };

            m_latestSkeletonStates[skeletonId] = newSkeletonState;

            returnedState = newSkeletonState;
        }

        return returnedState;
    }

    private NokovMarkerState GetOrCreateMarkerState(Int32 markerId)
    {
        NokovMarkerState returnedState = null;

        if (m_latestMarkerStates.TryGetValue(markerId, out var markerState))
        {
            returnedState = markerState;
        }
        else
        {
            NokovMarkerState newMarkerState = new NokovMarkerState
            {
                Position = new Vector3(),
            };

            m_latestMarkerStates[markerId] = newMarkerState;

            returnedState = newMarkerState;
        }

        return returnedState;
    }

    private NokovMarkerState GetOrCreateUnlabeledMarkerState(Int32 markerId)
    {
        NokovMarkerState returnedState = null;

        if (m_latestUnlabeledMarkerStates.TryGetValue(markerId, out var unlabeledMarkerState))
        {
            returnedState = unlabeledMarkerState;
        }
        else
        {
            NokovMarkerState newMarkerState = new NokovMarkerState
            {
                Position = new Vector3(),
            };

            m_latestUnlabeledMarkerStates[markerId] = newMarkerState;

            returnedState = newMarkerState;
        }

        return returnedState;
    }

    public void CheckNeedRefresh()
    {
        
        if(Nokov.SDK.NokovSDKClient.Refresh)
        {
            RefreshRigid = true;
            RefreshSkeleton = true;
            UpdateDefinitions();

            foreach (KeyValuePair<Int32, GameObject> markerSphereEntry in m_latestMarkerSpheres)
            {
                Destroy(m_latestMarkerSpheres[markerSphereEntry.Key]);
            }
            m_latestMarkerSpheres.Clear();

            foreach (KeyValuePair<Int32, GameObject> markerSphereEntry in m_latestUnlabeledMarkerCubus)
            {
                Destroy(m_latestUnlabeledMarkerCubus[markerSphereEntry.Key]);
            }
            m_latestUnlabeledMarkerCubus.Clear();
            Nokov.SDK.NokovSDKClient.Refresh = false;
        }
    }

    public bool GetNeedRefreshRigid()
    {
        return RefreshRigid;
    }

    public void SetNeedRefreshRigid()
    {
        RefreshRigid = false;
    }

    public bool GetNeedRefreshSkeleton()
    {
        return RefreshSkeleton;
    }

    public void SetNeedRefreshSkeleton()
    {
        RefreshSkeleton = false;
    }

    public void _EnterFrameDataUpdateLock()
    {
        Monitor.Enter( m_frameDataUpdateLock );
    }

    public void _ExitFrameDataUpdateLock()
    {
        Monitor.Exit( m_frameDataUpdateLock );
    }
#endregion Private methods
}

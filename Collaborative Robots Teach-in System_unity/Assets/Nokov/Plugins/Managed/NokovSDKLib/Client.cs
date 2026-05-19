// ***********************************************************************
// Assembly         : Assembly-CSharp
// Author           : duguguang
// Created          : 05-10-2020
//
// Last Modified By : duguguang
// Last Modified On : 07-24-2020
// ***********************************************************************
// <copyright file="Client.cs" company="Nokov">
//     Copyright (c) Nokov. All rights reserved.
// </copyright>
// <summary>Managed Client</summary>
// ***********************************************************************

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using AOT;
using CSNokovSDK;

[Serializable]
public class NokovSDKException : System.Exception
{
    public NokovSDKException()
    {
    }


    public NokovSDKException(string message)
        : base(message)
    {
    }


    public NokovSDKException(string message, Exception inner)
        : base(message, inner)
    {
    }


    internal static void ThrowIfNotOK(int result, string message)
    {
        if (result != 0)
        {
            throw new NokovSDKException(message + " (" + result.ToString() + ")");
        }
    }
}

public abstract class SDKClient : IDisposable
{
    public bool Connected { get; protected set; }

    ~SDKClient()
    {
        Dispose(false);
    }

    public abstract void Connect(string serverAddress);
    public abstract void DisConnect();
    public abstract void DestroyClient();

    public void Dispose()
    {
        Dispose(true);
        GC.SuppressFinalize(this);
    }

    protected void Dispose(bool disposing)
    {
        if (m_disposed)
            return;

        try
        {
            if (Connected)
            {
                DisConnect();
                Connected = false;
            }

            DestroyClient();
        }
        catch (Exception ex)
        {
            // 在Dispose期间捕获异常，防止崩溃
            try
            {
                UnityEngine.Debug.LogWarning($"[SDKClient] Dispose时发生异常: {ex.Message}");
            }
            catch
            {
                // Unity已卸载，忽略
            }
        }
        finally
        {
            m_disposed = true;
        }
    }

    protected void ThrowIfDisposed()
    {
        if (m_disposed)
        {
            throw new ObjectDisposedException(GetType().FullName);
        }
    }

    private bool m_disposed = false;
}


namespace Nokov.SDK
{
    // The Class to handle UnManaged Assets
    internal class NokovSDKClient : SDKClient
    {
        public static bool Refresh = false;
        public class DataDescriptions
        {
            public List<sMarkerSetDescription> MarkerSetDescriptions;
            public List<sRigidBodyDescription> RigidBodyDescriptions;
            public List<sSkeletonDescription> SkeletonDescriptions;
        }

        public Version NokovSDKLibVersion
        {
            get
            {
                Byte[] NokovSDKLibVersion = new Byte[4];
                CNokovSDK.NokovVersion(m_clientHandle, NokovSDKLibVersion);
                return new Version( NokovSDKLibVersion[0], NokovSDKLibVersion[1], NokovSDKLibVersion[2], NokovSDKLibVersion[3] );
            }
        }

        public static event EventHandler<NativeFrameReceivedEventArgs> NativeFrameReceived;

        public class NativeFrameReceivedEventArgs : EventArgs
        {
            private sFrameOfMocapData? m_marshaledFrame;
            private IntPtr m_nativeFrame;
            private IntPtr m_clientHandle;

            public IntPtr ClientHandle
            {
                get
                {
                    return m_clientHandle;
                }

                set
                {
                    m_clientHandle = value;
                }
            }

            public IntPtr NativeFramePointer {
                get
                {
                    return m_nativeFrame;
                }

                set
                {
                    // Invalidate lazily-evaluated cached marshaled frame.
                    m_marshaledFrame = null;

                    m_nativeFrame = value;
                }
            }

            public sFrameOfMocapData MarshaledFrame {
                get {
                    if ( m_marshaledFrame.HasValue == false )
                    {
                        m_marshaledFrame = (sFrameOfMocapData)Marshal.PtrToStructure( NativeFramePointer, typeof( sFrameOfMocapData ) );
                    }

                    return m_marshaledFrame.Value;
                }
            }
        }


        #region Private fields
        private IntPtr m_clientHandle = IntPtr.Zero;
        private static NokovSDKFrameReceivedCallback m_nativeFrameReceivedHandler = FrameReceivedNativeThunk;
        private static NokovNotifyMsgCallback m_NotifyMsgHandler = NotifyMsgEvent;
        private static NativeFrameReceivedEventArgs m_nativeFrameReceivedEventArgs = new NativeFrameReceivedEventArgs();
        #endregion Private fields

        public NokovSDKClient()
        {
            int retval = (int)CNokovSDK.CreateClient(out m_clientHandle);
            NokovSDKException.ThrowIfNotOK( retval, "NokovSDK_Client_Create failed." );

            if ( m_clientHandle == IntPtr.Zero )
            {
                throw new NokovSDKException( "NokovSDK_Client_Create returned null handle." );
            }

            m_nativeFrameReceivedEventArgs.ClientHandle = m_clientHandle;
            NokovSDKException.ThrowIfNotOK(retval, "NokovSDK_Client_SetDataDescriptionReceivedCallback failed.");

            retval = (int)CNokovSDK.SetDataCallback(m_clientHandle, m_nativeFrameReceivedHandler, m_clientHandle);
            NokovSDKException.ThrowIfNotOK( retval, "NokovSDK_Client_SetFrameReceivedCallback failed." );

            CNokovSDK.SetNotifyMsgCallback(m_clientHandle, m_NotifyMsgHandler, m_clientHandle);
        }

        public override void Connect(string serverAddress)
        {
            int retval = (int)CNokovSDK.Initialize(m_clientHandle, serverAddress);
            NokovSDKException.ThrowIfNotOK(retval, "NokovSDK_Client_Connect failed.");

            Connected = true;
        }

        public void Disconnect()
        {
            ThrowIfDisposed();

            if ( Connected )
            {
                int retval = (int)CNokovSDK.Uninitialize( m_clientHandle );
                NokovSDKException.ThrowIfNotOK(retval, "NokovSDK_Client_Disconnect failed." );

                Connected = false;
            }
        }


        public DataDescriptions GetDataDescriptions()
        {
            ThrowIfDisposed();

            IntPtr pDataDescriptions;

            int ret = (int)CNokovSDK.GetDataDescriptions(m_clientHandle, out pDataDescriptions);
            if (null  == pDataDescriptions)
            {
                NokovSDKException.ThrowIfNotOK(ret, "NokovSDK_Client_GetDataDescriptions failed.");
            }

            sDataDescriptions dataDescriptions = (sDataDescriptions)Marshal.PtrToStructure(pDataDescriptions, typeof( sDataDescriptions ) );

            // Do a quick first pass to determine the required capacity for the returned lists.
            Int32 numMarkerSetDescs = 0;
            Int32 numRigidBodyDescs = 0;
            Int32 numSkeletonDescs = 0;

            for ( Int32 i = 0; i < dataDescriptions.DataDescriptionCount; ++i )
            {
                sDataDescription desc = dataDescriptions.DataDescriptions[i];

                switch ( desc.DescriptionType )
                {
                    case (Int32)NokovSDKDataDescriptionType.NokovSDKDataDescriptionType_MarkerSet:
                        ++numMarkerSetDescs;
                        break;
                    case (Int32)NokovSDKDataDescriptionType.NokovSDKDataDescriptionType_RigidBody:
                        ++numRigidBodyDescs;
                        break;
                    case (Int32)NokovSDKDataDescriptionType.NokovSDKDataDescriptionType_Skeleton:
                        ++numSkeletonDescs;
                        break;
                }
            }

            // Allocate the lists to be returned based on our counts.
            DataDescriptions retDescriptions = new DataDescriptions {
                MarkerSetDescriptions = new List<sMarkerSetDescription>( numMarkerSetDescs ),
                RigidBodyDescriptions = new List<sRigidBodyDescription>( numRigidBodyDescs ),
                SkeletonDescriptions = new List<sSkeletonDescription>( numSkeletonDescs ),
            };

            try
            {
                // Now populate the lists.
                for (Int32 i = 0; i < dataDescriptions.DataDescriptionCount; ++i)
                {
                    sDataDescription desc = dataDescriptions.DataDescriptions[i];

                    switch (desc.DescriptionType)
                    {
                        case (Int32)NokovSDKDataDescriptionType.NokovSDKDataDescriptionType_MarkerSet:
                            sMarkerSetDescription markerSetDesc = (sMarkerSetDescription)Marshal.PtrToStructure((IntPtr)desc.Description, typeof(sMarkerSetDescription));
                            retDescriptions.MarkerSetDescriptions.Add(markerSetDesc);
                            break;
                        case (Int32)NokovSDKDataDescriptionType.NokovSDKDataDescriptionType_RigidBody:
                            sRigidBodyDescription rigidBodyDesc = (sRigidBodyDescription)Marshal.PtrToStructure((IntPtr)desc.Description, typeof(sRigidBodyDescription));
                            retDescriptions.RigidBodyDescriptions.Add(rigidBodyDesc);
                            break;
                        case (Int32)NokovSDKDataDescriptionType.NokovSDKDataDescriptionType_Skeleton:
                            sSkeletonDescription skeletonDesc = (sSkeletonDescription)Marshal.PtrToStructure((IntPtr)desc.Description, typeof(sSkeletonDescription));
                            retDescriptions.SkeletonDescriptions.Add(skeletonDesc);
                            break;
                    }
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("ERROR - Exception occurred in GetDataDescriptions: " + ex.ToString());
            }

            return retDescriptions;
        }

        [MonoPInvokeCallback(typeof(NokovSDKFrameReceivedCallback))]
        public static void FrameReceivedNativeThunk( IntPtr pFrameOfMocapData, IntPtr pUserData )
        {
            // 安全检查：确保指针有效
            if (pFrameOfMocapData == IntPtr.Zero)
            {
                return;
            }

            try
            {
                // 使用局部变量避免多线程问题
                var handler = NativeFrameReceived;
                if (handler != null)
                {
                    m_nativeFrameReceivedEventArgs.NativeFramePointer = pFrameOfMocapData;
                    handler(null, m_nativeFrameReceivedEventArgs);
                }
            }
            catch (Exception)
            {
                // 注意：这是native回调，不能调用任何Unity主线程API（包括Debug.Log）
                // 如果需要调试，可以写入文件或使用System.Diagnostics.Debug
                // 此处静默忽略异常，因为可能是组件正在销毁
            }
        }

        [MonoPInvokeCallback(typeof(NokovNotifyMsgCallback))]
        public static void NotifyMsgEvent(IntPtr pNotify, IntPtr pUserData)
        {
            try
            {
                Refresh = true;
            }
            catch
            {
                // 忽略Unity卸载后的异常
            }
        }

        public override void DisConnect()
        {
            // 安全检查：确保句柄有效
            if (m_clientHandle == IntPtr.Zero)
            {
                Connected = false;
                return;
            }

            try
            {
                NokovSDKError disconnectResult = (NokovSDKError)CNokovSDK.Uninitialize(m_clientHandle);

                if (disconnectResult != NokovSDKError.NokovSDKError_OK)
                {
                    System.Diagnostics.Debug.WriteLine("NokovSDK_Client_Disconnect returned " + disconnectResult.ToString() + ".");
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("NokovSDK_Client_Disconnect exception: " + ex.Message);
            }

            Connected = false;
        }

        public override void DestroyClient()
        {
            // 安全检查：确保句柄有效
            if (m_clientHandle == IntPtr.Zero)
            {
                return;
            }

            try
            {
                // 先清除回调，防止在销毁后还有回调进来
                CNokovSDK.SetDataCallback(m_clientHandle, null, IntPtr.Zero);
                CNokovSDK.SetNotifyMsgCallback(m_clientHandle, null, IntPtr.Zero);

                // 销毁原生客户端
                NokovSDKError destroyResult = (NokovSDKError)CNokovSDK.DestroyClient(m_clientHandle);

                if (destroyResult != NokovSDKError.NokovSDKError_OK)
                {
                    System.Diagnostics.Debug.WriteLine("NokovSDK_Client_Destroy returned " + destroyResult.ToString() + ".");
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("NokovSDK_Client_Destroy exception: " + ex.Message);
            }
            finally
            {
                m_clientHandle = IntPtr.Zero;
            }
        }

        // for test now 
        public sFrameOfMocapData GetLastFrame()
        {
            IntPtr framePtr;
            sFrameOfMocapData data = new sFrameOfMocapData();

            try
            {
                framePtr = CNokovSDK.GetLastFrameOfMocapData(m_clientHandle);
                data = Marshal.PtrToStructure<sFrameOfMocapData>(framePtr);
            }
            catch
            {

            }

            return data;
        }
    }
}



using System;
using System.Runtime.InteropServices;
using System.Linq;
using CSNokovSDK;
using System.Collections.Generic;

namespace Samples
{
    class Program
    {
        static private IntPtr clientHandler = IntPtr.Zero;
        static private NokovSDKFrameReceivedCallback DATA_CB = MyDataHandler;
        static private NokovNotifyMsgCallback NOTIFY_CB = NotifyHandler;
        static private bool bExit = false;
        static void Main(string[] args)
        {
            Console.WriteLine("Please input the server IP\n");
            try
            {
                string ip = Console.ReadLine();
                int iResult = CreateClient(ip);
                if (iResult != 0)
                {
                    Console.WriteLine("Error initializing client.  Exiting\n");
                    while (Console.ReadKey().Key.ToString() != "Q")
                    {

                    }
                    return;
                }
                ProcessDataDescriptions();

                Console.WriteLine("Successfully connected to server\n");
                Console.WriteLine("\n1: Callback passive receiving data\n2: Host reading data\nEnter 1 or 2 and select: ");
                string s = Console.ReadLine();
                if (s == "1")
                {
                    StartDataStreaming();
                    Console.WriteLine("callback");
                }
                else
                {
                    Action a = readDataStreaming;
                    a.BeginInvoke(null, null);
                    Console.WriteLine("reading");
                }
                CNokovSDK.SetNotifyMsgCallback(clientHandler, NOTIFY_CB, IntPtr.Zero);
                while (Console.ReadKey().Key.ToString() != "Q")
                {

                }
                bExit = true;
            }
            catch
            {
                Console.WriteLine("Error.  Exiting\n");
                while (Console.ReadKey().Key.ToString() != "Q")
                {

                }
            }
            
        }

        static void MyDataHandler(IntPtr pFrameOfData, IntPtr pUserData)
        {
            var frame = (sFrameOfMocapData)Marshal.PtrToStructure(pFrameOfData, typeof(sFrameOfMocapData));
            Console.WriteLine("\nFrameNO:{0}\tTimeStamp:{1}", frame.FrameNumber, frame.Timestamp);
            Console.WriteLine("MarkerSet [Count={0}]", frame.MarkerSetCount);
            for (int i = 0; i < frame.MarkerSetCount; i++)
            {
                sMarkerSetData markerset = frame.MarkerSets[i];
                Console.WriteLine("Markerset{0}: {1} [nMarkers Count={2}]",
                    i + 1, string.Concat(markerset.Name.TakeWhile(c => c != '\0')),
                    markerset.MarkerCount);
                Console.WriteLine("{");
                for (int i_Marker = 0; i_Marker < markerset.MarkerCount; i_Marker++)
                {
                    IntPtr ptr = IntPtr.Add(markerset.Markers, Marshal.SizeOf(typeof(tMarkerData)) * i_Marker);
                    var Markers = (tMarkerData)Marshal.PtrToStructure(ptr, typeof(tMarkerData));

                    Console.WriteLine("\tMarker{0}(mm) \tx:{1}\ty:{2}\tz:{3}",
                        i_Marker,Markers.Values[0].ToString("000.00"),
                        Markers.Values[1].ToString("000.00"),Markers.Values[2].ToString("000.00"));
                }
                Console.WriteLine( "}");
             }

            //Print RigidBodies
            Console.WriteLine("Markerset.RigidBodies [Count={0}]", frame.RigidBodyCount);
            for (int i = 0; i < frame.RigidBodyCount; i++)
            {
                Console.WriteLine("{");
                Console.WriteLine("\tid:{0}", frame.RigidBodies[i].Id);
                Console.WriteLine("\t    (mm)\tx:{0}\ty:{1}\tz:{2}\t",
                    frame.RigidBodies[i].X.ToString("000.00"),
                    frame.RigidBodies[i].Y.ToString("000.00"),
                    frame.RigidBodies[i].Z.ToString("000.00"));
                Console.WriteLine("\t\t\tqx:{0}\tqy:{1}\tqz:{2}\tqw:{3}",
                    frame.RigidBodies[i].QX.ToString("000.00"),
                    frame.RigidBodies[i].QY.ToString("000.00"),
                    frame.RigidBodies[i].QZ.ToString("000.00"),
                    frame.RigidBodies[i].QW.ToString("000.00"));

                Console.WriteLine("\tRigidBody markers [Count={0}]", frame.RigidBodies[i].NMarkers);
                for (int iMarker = 0; iMarker < frame.RigidBodies[i].NMarkers; iMarker++)
                {
                    if (frame.RigidBodies[i].MarkerIDs != IntPtr.Zero)
                    {
                        IntPtr ptr = IntPtr.Add(frame.RigidBodies[i].MarkerIDs, Marshal.SizeOf(typeof(int)) * iMarker);
                        var id = (int)Marshal.PtrToStructure(ptr, typeof(int));
                        Console.Write("\tMarker{0}(mm)", id);
                    }
                        
                    if (frame.RigidBodies[i].Markers != IntPtr.Zero)
                    {
                        IntPtr ptr = IntPtr.Add(frame.RigidBodies[i].Markers, Marshal.SizeOf(typeof(tMarkerData)) * iMarker);
                        var Markers = (tMarkerData)Marshal.PtrToStructure(ptr, typeof(tMarkerData));
                        Console.WriteLine("\tx:{0}\ty:{1}\tz:{2}",
                            Markers.Values[0].ToString("000.00"),
                            Markers.Values[1].ToString("000.00"),
                            Markers.Values[2].ToString("000.00"));
                    }
                }
                Console.WriteLine("}");
            }
            
            
            //Print Skeletons
            Console.WriteLine("Markerset.Skeletons [Count={0}]", frame.SkeletonCount);
            for (int i = 0; i < frame.SkeletonCount; i++)
            {
                Console.WriteLine("Markerset{0}.Skeleton [nRigidBodies Count={0}]",
                    frame.Skeletons[i].Id, frame.Skeletons[i].RigidBodyCount);
                Console.WriteLine("{");
                // print the rigidbody of skeleton
                for (int j = 0; j < frame.Skeletons[i].RigidBodyCount; j++)
                {
                    IntPtr ptr = IntPtr.Add(frame.Skeletons[i].RigidBodies, Marshal.SizeOf(typeof(sRigidBodyData)) * j);
                    var rigid = (sRigidBodyData)Marshal.PtrToStructure(ptr, typeof(sRigidBodyData));
                    // print the position and the orientation/rotation

                    Console.WriteLine("\tid:{0}", rigid.Id);
                    Console.WriteLine("\t    (mm)\tx:{0}\ty:{1}\tz:{2}",
                        rigid.X.ToString("000.00"),
                        rigid.Y.ToString("000.00"),
                        rigid.Z.ToString("000.00"));

                    Console.WriteLine("\t\t\tqx:{0}\tqy:{1}\tqz:{2}\tqw:{3}",
                        rigid.QX.ToString("000.00"),
                        rigid.QY.ToString("000.00"),
                        rigid.QZ.ToString("000.00"),
                        rigid.QW.ToString("000.00"));

                    Console.WriteLine("\tRigidBody markers [Count={0}]", rigid.NMarkers);
                    for (int iMarker = 0; iMarker < rigid.NMarkers; iMarker++)
                    {
                        if (rigid.MarkerIDs != IntPtr.Zero)
                        {
                            ptr = IntPtr.Add(rigid.MarkerIDs, Marshal.SizeOf(typeof(int)) * iMarker);
                            var id = (int)Marshal.PtrToStructure(ptr, typeof(int));
                            Console.Write("\tMarker{0}(mm)", id);
                        }

                        if (rigid.Markers != IntPtr.Zero)
                        {
                            ptr = IntPtr.Add(rigid.Markers, Marshal.SizeOf(typeof(tMarkerData)) * iMarker);
                            var Markers = (tMarkerData)Marshal.PtrToStructure(ptr, typeof(tMarkerData));
                            Console.WriteLine("\tx:{0}\ty:{1}\tz:{2}",
                                Markers.Values[0].ToString("000.00"),
                                Markers.Values[1].ToString("000.00"),
                                Markers.Values[2].ToString("000.00"));
                        }
                    }
                }
                Console.WriteLine("}");//The data output of the j-th RigidBody (RigidBody) is completed
            }

            Console.WriteLine("Other Markers [Count={0}]", frame.OtherMarkerCount);
            Console.WriteLine("{");
            for (int i = 0; i < frame.OtherMarkerCount; i++)//Output the id and information (x, y, z) of the unnamed marker included
            {
                IntPtr ptr = IntPtr.Add(frame.OtherMarkers, Marshal.SizeOf(typeof(tMarkerData)) * i);
                var Markers = (tMarkerData)Marshal.PtrToStructure(ptr, typeof(tMarkerData));
                Console.WriteLine("\tOther Marker{0}(mm)\tx:{1}\ty:{2}\tz:{3}",
                    i,
                    Markers.Values[0].ToString("000.00"),
                    Markers.Values[1].ToString("000.00"),
                    Markers.Values[2].ToString("000.00"));
            }
            Console.WriteLine("}");//The data output of unnamed marker is completed, and the data output of one frame is completed
            Console.WriteLine("Analog [Count={0}]", frame.AnalogdataCount);//Output the total number of analog data contained
            Console.WriteLine("{");
            for (int i = 0; i < frame.AnalogdataCount; ++i)
            {
                Console.WriteLine("\tAnalogData {0}: {1}", 
                    i, frame.Analogdata[i].ToString("000.0000"));
            }
            Console.WriteLine("}\n");
        }

        static void NotifyHandler(IntPtr pNotify, IntPtr pUserData)
        {
            var notify = (sNotifyMsg)Marshal.PtrToStructure(pNotify, typeof(sNotifyMsg));
            Console.WriteLine("Notify Type:{0}, Value:{1}, timestamp:{2}, param1:{3}",
                notify.Type, notify.Value, notify.TimeStamp, notify.Param1);
        }

        static int CreateClient(string ip)
        {
            CNokovSDK.CreateClient(out clientHandler);
            if (clientHandler == IntPtr.Zero)
                return 1;
            byte[] ver = new byte[4];
            CNokovSDK.NokovVersion(clientHandler, ver);
            Console.WriteLine("NokovSDK Sample Client 2.4.0.5430(NokovSDK ver.{0}.{1}.{2}.{3})\n", ver[0], ver[1], ver[2], ver[3]);
            int retval = (int)CNokovSDK.Initialize(clientHandler, ip);
            if (retval != 0)
            {
                Console.WriteLine("Unable to connect to server.Error code: {0}. Exiting\n", retval);
                return 1;
            }
            return 0;
        }

        static int ProcessDataDescriptions()
        {
            IntPtr pDataDescriptions = IntPtr.Zero;
            int ret = (int)CNokovSDK.GetDataDescriptions(clientHandler, out pDataDescriptions);
            if (null == pDataDescriptions)
            {
                Console.WriteLine("Error get data def. Exiting\n");
                return 1;
            }

            sDataDescriptions dataDescriptions = (sDataDescriptions)Marshal.PtrToStructure(pDataDescriptions, typeof(sDataDescriptions));
            for (Int32 i = 0; i < dataDescriptions.DataDescriptionCount; ++i)
            {
                sDataDescription desc = dataDescriptions.DataDescriptions[i];

                switch (desc.DescriptionType)
                {
                    case (Int32)NokovSDKDataDescriptionType.NokovSDKDataDescriptionType_MarkerSet:
                        sMarkerSetDescription markerSetDesc = (sMarkerSetDescription)Marshal.PtrToStructure((IntPtr)desc.Description, typeof(sMarkerSetDescription));
                        Console.WriteLine("MarkerSetName : {0}", string.Concat(markerSetDesc.Name.TakeWhile(c => c != '\0')));
                        for (int markerIndex = 0; markerIndex < markerSetDesc.MarkerCount; ++markerIndex)
                        {
                            Console.WriteLine("Marker[{0}] : {1}", markerIndex, Marshal.PtrToStringAnsi(Marshal.ReadIntPtr(markerSetDesc.MarkerNames, IntPtr.Size * markerIndex)));
                        }
                        break;
                    case (Int32)NokovSDKDataDescriptionType.NokovSDKDataDescriptionType_RigidBody:
                        sRigidBodyDescription rigidBodyDesc = (sRigidBodyDescription)Marshal.PtrToStructure((IntPtr)desc.Description, typeof(sRigidBodyDescription));
                        Console.WriteLine("RigidBody:{0}", string.Concat(rigidBodyDesc.Name.TakeWhile(c => c != '\0')));
                        break;
                    case (Int32)NokovSDKDataDescriptionType.NokovSDKDataDescriptionType_Skeleton:
                        sSkeletonDescription skeletonDesc = (sSkeletonDescription)Marshal.PtrToStructure((IntPtr)desc.Description, typeof(sSkeletonDescription));
                        Console.WriteLine("Skeleton Name:{0}, id:{1}, rigids:{2}",
                            string.Concat(skeletonDesc.Name.TakeWhile(c => c != '\0')), skeletonDesc.Id, skeletonDesc.RigidBodyCount);
                        for (int boneIndex = 0; boneIndex < skeletonDesc.RigidBodyCount; ++boneIndex)
                        {
                            var boneDescription = skeletonDesc.RigidBodies[boneIndex];
                            Console.WriteLine("[{0}] {1} {2} {3}mm {4}mm {5}mm {6} {7} {8} {9}", 
                                boneDescription.Id, string.Concat(boneDescription.Name.TakeWhile(c => c != '\0')),
                                boneDescription.ParentId, boneDescription.OffsetX.ToString("F2"),
                                boneDescription.OffsetY.ToString("F2"), boneDescription.OffsetZ.ToString("F2"),
                                boneDescription.Qx.ToString("F2"), boneDescription.Qy.ToString("F2"),
                                boneDescription.Qz.ToString("F2"), boneDescription.Qw.ToString("F2"));
                        }
                        break;
                    case (Int32)NokovSDKDataDescriptionType.NokovSDKDataDescriptionType_Param:
                        sDataParam dataParam = (sDataParam)Marshal.PtrToStructure((IntPtr)desc.Description, typeof(sDataParam));
                        Console.WriteLine("FrameRate:{0}", dataParam.FrameRate);
                        break;
                    default:
                        break;
                }
            }

            return 0;
        }

        static int StartDataStreaming()
        {
            CNokovSDK.SetDataCallback(clientHandler, DATA_CB, IntPtr.Zero);
            return 0;
        }

        static void readDataStreaming()
        {
            IntPtr data = IntPtr.Zero;
            while (!bExit)
            {
                data = CNokovSDK.GetLastFrameOfMocapData(clientHandler);
                if (data != IntPtr.Zero)
                {
                    MyDataHandler(data, IntPtr.Zero);
                }
                CNokovSDK.NokovFreeFrame(clientHandler,data);
            }
        }
    }
}
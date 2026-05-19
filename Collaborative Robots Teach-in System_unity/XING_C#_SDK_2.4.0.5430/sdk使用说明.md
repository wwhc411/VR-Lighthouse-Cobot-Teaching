内容概述
NOKOV SDK 使用手册用于帮助开发者了解SDK 的使用规则，该手册包含所有的接 
口及详细说明，为用户最终实现从NOKOV 程序接收动捕数据。
适用的 NOKOV 程序及版本：
XINGYING 1.0.0.4321 及以上版本； 
XING 1.4.0.4537 及以上版本；
一、类型定义
1、sRigidBodyData
刚体信息
public struct sRigidBodyData {
///  <summary>  /// 
The identifier /// 
</summary> 
public Int32 Id; 
///  <summary>  /// 
The x 
/// </summary> 
public  float  X;  /// 
<summary> /// 
The y 
/// </summary> 
public  float  Y;  /// 
<summary> /// 
The z 
/// </summary> 
public  float  Z;  /// 
<summary> /// 
The qx 
/// </summary> 
public float QX; 
///  <summary>  /// 
The qy 
/// </summary> 
public float QY; 
/// <summary> /// The qz 
/// </summary> public 
float QZ; /// <summary> /// 
The qw /// </summary> 
public float QW; /// 
<summary> /// The n 
markers /// </summary> 
public int NMarkers; /// 
<summary> /// The 
markers /// </summary> 
public IntPtr Markers; /// 
<summary> /// The marker 
ids /// </summary> public 
IntPtr MarkerIDs; /// 
<summary> /// The marker 
sizes /// </summary> 
public IntPtr MarkerSizes; 
/// <summary> /// The 
mean error /// </summary> 
public float MeanError; /// 
<summary> /// The 
parameters /// </summary> 
public Int16 Params;
} 
Members
ID 刚体ID 
x, y, z 刚体位
置
qx, qy, qz, qw 
刚体方向
NMarkers 
组成刚体的Marker 点个数 
Markers 
Marker 点数据 
MarkerIDs 
Marker ID 
MarkerSizes 
Marker 大
小
MeanError 
偏差 
params 
保留
2、sSkeletonData
人体信息。
public struct sSkeletonData {
/// <summary> 
/// The identifier 
/// </summary> 
public Int32 Id; 
/// <summary> 
/// The rigid body count 
/// </summary> 
public Int32 RigidBodyCount; 
/// <summary> 
/// The rigid bodies 
/// </summary> 
public IntPtr RigidBodies; // Pointer to sRigidBodyData[RigidBodyCount]
}
Members
ID 
人体ID 
RigidBodies 
刚体/骨骼个数 
sRigidBodyData 
刚体/骨骼信息
3、sMarker
Marker 信息。
public struct sMarker {
/// <summary> /// 
The identifier /// 
</summary> 
public Int32 Id; /// 
<summary> /// 
The position x /// 
</summary> 
public float X; /// 
<summary> /// 
The position y /// 
</summary> 
public float Y; /// 
<summary> /// 
The position z /// 
</summary> 
public float Z; /// 
<summary> /// 
The size 
/// </summary> public float 
Size; /// <summary> /// The 
parameters(reserved) /// 
</summary> public Int16 
Params;
}
Members
ID Marker 点ID 
x, y, z Marker 点位
置 
size Marker 大小 
params 
保留
4、sFrameOfMocapData
动捕数据。
public struct sFrameOfMocapData { 
/// <summary> /// The frame 
number /// </summary> 
public Int32 FrameNumber;
/// <summary> 
/// The marker set count 
/// </summary> 
public Int32 MarkerSetCount; 
/// <summary> 
/// The marker sets 
/// </summary> 
[MarshalAs(UnmanagedType.ByValArray, SizeConst =
NokovSDKConstants.MaxModels)] public 
sMarkerSetData[] MarkerSets;
/// <summary> 
/// The undefined marker count, 
/// </summary> 
public Int32 OtherMarkerCount; 
/// <summary> 
/// The undefined markers, Pointer to float[OtherMarkerCount][3] /// 
</summary> 
public IntPtr OtherMarkers;
/// <summary> 
/// The rigid body count 
/// </summary> 
public Int32 RigidBodyCount; 
/// <summary> 
/// The rigid bodies 
/// </summary> 
[MarshalAs(UnmanagedType.ByValArray, SizeConst =
NokovSDKConstants.MaxRigidBodies)] public 
sRigidBodyData[] RigidBodies;
/// <summary> 
/// The skeleton count 
/// </summary> 
public Int32 SkeletonCount; 
/// <summary> 
/// The skeletons 
/// </summary> 
[MarshalAs(UnmanagedType.ByValArray, SizeConst =
NokovSDKConstants.MaxSkeletons)] 
public sSkeletonData[] Skeletons;
/// <summary>
/// The labeled marker count 
/// </summary> 
public Int32 LabeledMarkerCount; 
/// <summary> 
/// The labeled markers 
/// </summary> 
[MarshalAs(UnmanagedType.ByValArray, SizeConst =
NokovSDKConstants.MaxLabeledMarkers)] 
public sMarker[] LabeledMarkers;
/// <summary> 
/// The analogdata count 
/// </summary> 
public Int32 AnalogdataCount; 
/// <summary> 
/// The analogdata 
/// </summary> 
[MarshalAs(UnmanagedType.ByValArray, SizeConst =
NokovSDKConstants.MaxAnalogChannels)] 
public float[] Analogdata;
/// <summary> 
/// The host defined time delta between capture and send 
/// </summary> 
public float FLatency; 
/// <summary> 
/// The SMPTE timecode (if available) 
/// </summary> 
public UInt32 Timecode; 
/// <summary> 
/// The timecode subframe 
/// </summary> 
public UInt32 TimecodeSubframe; 
/// <summary> 
/// The FrameGroup timestamp, the number of milliseconds since the Epoch
1970-01-01 00:00:00 +0000 (UTC). 
/// </summary> public 
UInt64 Timestamp; /// 
<summary> 
/// The host defined parameters 
/// </summary> 
public Int16 Params; }
Members
FrameNumber 
帧号
MarkerSetsCount 
MarkerSet 个数
MocapData 
MarkerSet 信息
OtherMarkersCount 
未命名 Marker 点个
数 OtherMarkers 
未命名 Marker 点数据 
RigidBodiesCount 
刚体个数
RigidBodies
刚体信息
SkeletonsCount 人
体/骨骼个数 
Skeletons 人体/骨
骼信息 
LabeledMarkersCount 
命名Marker 点个
数
LabeledMarkers 命名
Marker 点数据 
Analogdatas 模拟数据
通道个数 
Analogdata 
模拟数据 
FLatency 
时间差 
Timecode SMPTE
时间码 
TimecodeSubframe 
帧数偏差 
TimeStamp 时间戳
（毫秒）
params 
保留
6、Verbosity
日志等级
public enum NokovVerbosity
{
/// <summary> 
///  The  verbosity  none 
/// </summary> 
Verbosity_None  =  0, 
/// <summary> 
/// The verbosity information 
/// </summary> 
Verbosity_Info,  ///  <summary> 
/// The verbosity warning /// 
</summary> 
Verbosity_Warning, /// 
<summary> /// The verbosity 
error /// </summary>
Verbosity_Error,
/// <summary>
/// The verbosity debug
/// </summary>
Verbosity_Debug,
}
7、sMarkerSetDescription
MarkerSet 描述信息。
public struct sMarkerSetDescription { 
/// <summary> /// The 
MarkerSet name /// 
</summary>
[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 
NokovSDKConstants.MaxNameLength)] 
public string Name; 
/// <summary> 
/// The number of markers in MarkerSet 
/// </summary> 
public Int32 MarkerCount; 
/// <summary> 
/// The marker names, char**, "array of marker names" /// 
</summary> 
public IntPtr MarkerNames; 
}
Members
Name MarkerSet 名
称
MarkersCount 
Marker 点个数

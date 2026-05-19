using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Tracker位姿数据 - 数据结构定义（简化版）
/// 仅包含位姿控制必需的参数
/// 更新日期: 2025-12-02
/// </summary>
[Serializable]
public class RigidBodyCaptureData
{
    public Metadata Metadata;           // 元数据
    public List<FrameData> FrameData;   // 帧数据列表
}

/// <summary>
/// 元数据：采集信息摘要
/// </summary>
[Serializable]
public class Metadata
{
    public string CollectionTime;       // 采集时间 (格式: 2025-11-14 20:55:15)
    public int TotalFrames;             // 总帧数
    public string RigidBodyName;        // 数据名称
}

/// <summary>
/// 单帧数据：位置 + 姿态（四元数）+ 可选的TCP数据
/// </summary>
[Serializable]
public class FrameData
{
    public int FrameNumber;             // 帧序号 (从1开始)
    public long UnixTimeStamp;          // Unix时间戳 (毫秒)
    public PositionData Position;       // Tracker位置数据 (单位: mm)
    public QuaternionData Quaternion;   // Tracker四元数姿态
    
    // 可选：录制时同步记录的TCP数据（用于直接回放模式）
    public bool HasTcpData;             // 是否包含TCP数据
    public TcpPoseData TcpPose;         // TCP位姿数据（UR基座坐标系）

    /// <summary>
    /// 获取位置向量（Unity Vector3格式，自动转换mm→m）
    /// </summary>
    public Vector3 GetPosition()
    {
        return new Vector3(
            (float)(Position.X / 1000.0),
            (float)(Position.Y / 1000.0),
            (float)(Position.Z / 1000.0)
        );
    }

    /// <summary>
    /// 获取旋转矢量（从四元数转换，单位：弧度）
    /// 用于URScript的p[x,y,z,rx,ry,rz]格式
    /// </summary>
    public Vector3 GetRotationVector()
    {
        Quaternion q = GetQuaternion();
        float angle = 2.0f * Mathf.Acos(Mathf.Clamp(q.w, -1f, 1f));
        float s = Mathf.Sqrt(1.0f - q.w * q.w);
        
        if (s < 0.001f)
            return Vector3.zero;
        
        return new Vector3(
            q.x / s * angle,
            q.y / s * angle,
            q.z / s * angle
        );
    }

    /// <summary>
    /// 获取四元数（Unity Quaternion格式）
    /// </summary>
    public Quaternion GetQuaternion()
    {
        return new Quaternion(
            (float)Quaternion.X,
            (float)Quaternion.Y,
            (float)Quaternion.Z,
            (float)Quaternion.W
        );
    }

    /// <summary>
    /// 检查位置数据是否有效（非无效标记值）
    /// </summary>
    public bool IsPositionValid()
    {
        return Position.X < 9999998.0 && Position.Y < 9999998.0 && Position.Z < 9999998.0;
    }
}

/// <summary>
/// 位置数据 (单位: mm)
/// </summary>
[Serializable]
public class PositionData
{
    public double X;
    public double Y;
    public double Z;
}

/// <summary>
/// 四元数数据
/// </summary>
[Serializable]
public class QuaternionData
{
    public double X;
    public double Y;
    public double Z;
    public double W;
}

/// <summary>
/// TCP位姿数据（UR基座坐标系）
/// 用于直接回放模式，跳过手眼标定转换
/// </summary>
[Serializable]
public class TcpPoseData
{
    public double X;        // 位置X (米)
    public double Y;        // 位置Y (米)
    public double Z;        // 位置Z (米)
    public double RX;       // 旋转矢量RX (弧度)
    public double RY;       // 旋转矢量RY (弧度)
    public double RZ;       // 旋转矢量RZ (弧度)
}

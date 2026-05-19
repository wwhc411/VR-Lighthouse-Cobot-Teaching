using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace HandEyeCalibration.DLL
{
    /// <summary>
    /// DLL中定义的三维向量结构
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct Vector3D_Unity
    {
        public double x;
        public double y;
        public double z;

        public Vector3D_Unity(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        /// <summary>
        /// 从Unity Vector3转换（注意单位：Unity使用米，DLL可能使用毫米）
        /// </summary>
        public Vector3D_Unity(Vector3 vector, float scale = 1000f)
        {
            this.x = vector.x * scale;
            this.y = vector.y * scale;
            this.z = vector.z * scale;
        }

        /// <summary>
        /// 转换为Unity Vector3（注意单位转换）
        /// </summary>
        public Vector3 ToUnityVector3(float scale = 0.001f)
        {
            return new Vector3((float)(x * scale), (float)(y * scale), (float)(z * scale));
        }

        public override string ToString()
        {
            return $"({x:F3}, {y:F3}, {z:F3})";
        }
    }

    /// <summary>
    /// DLL中定义的四元数结构
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct Quaternion_Unity
    {
        public double w;
        public double x;
        public double y;
        public double z;

        public Quaternion_Unity(double w, double x, double y, double z)
        {
            this.w = w;
            this.x = x;
            this.y = y;
            this.z = z;
        }

        /// <summary>
        /// 从Unity Quaternion转换
        /// </summary>
        public Quaternion_Unity(Quaternion quaternion)
        {
            this.w = quaternion.w;
            this.x = quaternion.x;
            this.y = quaternion.y;
            this.z = quaternion.z;
        }

        /// <summary>
        /// 转换为Unity Quaternion
        /// </summary>
        public Quaternion ToUnityQuaternion()
        {
            return new Quaternion((float)x, (float)y, (float)z, (float)w);
        }

        /// <summary>
        /// 归一化四元数
        /// </summary>
        public void Normalize()
        {
            double norm = Math.Sqrt(w * w + x * x + y * y + z * z);
            if (norm > 1e-12)
            {
                w /= norm;
                x /= norm;
                y /= norm;
                z /= norm;
            }
        }

        public override string ToString()
        {
            return $"(w:{w:F4}, x:{x:F4}, y:{y:F4}, z:{z:F4})";
        }
    }

    /// <summary>
    /// DLL中定义的位姿结构（位置+旋转）
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct Pose_Unity
    {
        public Vector3D_Unity Position;
        public Quaternion_Unity Quaternion;

        public Pose_Unity(Vector3D_Unity position, Quaternion_Unity quaternion)
        {
            Position = position;
            Quaternion = quaternion;
        }

        /// <summary>
        /// 从Unity Transform创建（机器人位姿）
        /// 注意：机器人坐标系使用旋转向量(rx,ry,rz)，需要转换为四元数
        /// </summary>
        public static Pose_Unity FromRobotPose(double x, double y, double z, double rx, double ry, double rz)
        {
            Vector3D_Unity position = new Vector3D_Unity(x, y, z);
            Quaternion_Unity rotation = RotationVectorToQuaternion(rx, ry, rz);
            return new Pose_Unity(position, rotation);
        }

        /// <summary>
        /// 从相机位姿创建（位置+四元数）
        /// </summary>
        public static Pose_Unity FromCameraPose(double x, double y, double z, double qx, double qy, double qz, double qw)
        {
            Vector3D_Unity position = new Vector3D_Unity(x, y, z);
            Quaternion_Unity rotation = new Quaternion_Unity(qw, qx, qy, qz);
            rotation.Normalize();
            return new Pose_Unity(position, rotation);
        }

        /// <summary>
        /// 旋转向量转四元数（Rodrigues公式）
        /// </summary>
        private static Quaternion_Unity RotationVectorToQuaternion(double rx, double ry, double rz)
        {
            double theta = Math.Sqrt(rx * rx + ry * ry + rz * rz);

            if (theta < 1e-10)
            {
                return new Quaternion_Unity(1, 0, 0, 0);
            }

            double halfTheta = theta * 0.5;
            double sinHalfTheta = Math.Sin(halfTheta);
            double cosHalfTheta = Math.Cos(halfTheta);

            double w = cosHalfTheta;
            double x = (rx / theta) * sinHalfTheta;
            double y = (ry / theta) * sinHalfTheta;
            double z = (rz / theta) * sinHalfTheta;

            return new Quaternion_Unity(w, x, y, z);
        }

        public override string ToString()
        {
            return $"Pos:{Position}, Rot:{Quaternion}";
        }
    }

    /// <summary>
    /// DLL中定义的点云数据结构（用于批量位姿输入）
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct Point_Unity
    {
        public int MarkNum;      // Mark标记数量
        public int PointNum;     // 每个Mark的测量次数

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 1024)]
        public Pose_Unity[] Points;  // 位姿数组（最多1024个）

        public Point_Unity(int markNum, int pointNum)
        {
            MarkNum = markNum;
            PointNum = pointNum;
            Points = new Pose_Unity[1024];
        }

        /// <summary>
        /// 获取指定索引的位姿
        /// 索引规则：Points[j + PointNum * i] 表示第i个Mark的第j次测量
        /// </summary>
        public Pose_Unity GetPose(int markIndex, int pointIndex)
        {
            int index = pointIndex + PointNum * markIndex;
            if (index >= 1024)
                throw new IndexOutOfRangeException($"索引超出范围: {index} >= 1024");
            return Points[index];
        }

        /// <summary>
        /// 设置指定索引的位姿
        /// </summary>
        public void SetPose(int markIndex, int pointIndex, Pose_Unity pose)
        {
            int index = pointIndex + PointNum * markIndex;
            if (index >= 1024)
                throw new IndexOutOfRangeException($"索引超出范围: {index} >= 1024");
            Points[index] = pose;
        }

        /// <summary>
        /// 创建手眼标定用的Point_Unity（单Mark，多次测量）
        /// </summary>
        public static Point_Unity CreateForHandEyeCalibration(int poseCount)
        {
            if (poseCount > 1024)
                throw new ArgumentException($"位姿数量不能超过1024，当前: {poseCount}");

            Point_Unity point = new Point_Unity
            {
                MarkNum = 1,
                PointNum = poseCount,
                Points = new Pose_Unity[1024]
            };
            return point;
        }
    }
}

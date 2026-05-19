using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace HandEyeCalibration.DLL
{
    /// <summary>
    /// myDll.dll接口封装类
    /// 提供探针标定和手眼标定的DLL调用功能
    /// </summary>
    public static class DllInterface
    {
        // DLL文件名（需放置在Plugins文件夹下）
        private const string DLL_NAME = "myDll";

        #region DLL函数导入

        /// <summary>
        /// 探针尖端位置计算
        /// </summary>
        /// <param name="input">输入：包含多组Mark位姿的测量数据</param>
        /// <param name="output">输出：探针尖端在探针坐标系中的位置</param>
        /// <returns>返回0表示成功，非0表示失败</returns>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.StdCall)]
        private static extern int calculateNeedleTip(ref Point_Unity input, ref Vector3D_Unity output);

        /// <summary>
        /// 手眼标定 - 眼在手外场景
        /// </summary>
        /// <param name="gripper2base">输入：机械臂末端到基座的位姿序列（gripper→base）</param>
        /// <param name="target2cam">输入：标定板到相机的位姿序列（target→cam）</param>
        /// <param name="cam2gripper">输出：相机到基座的位姿变换（实际为cam→base）</param>
        /// <returns>返回0表示成功，1表示输入数据长度不匹配</returns>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.StdCall)]
        private static extern int calculateHandAndEye(
            ref Point_Unity gripper2base,
            ref Point_Unity target2cam,
            ref Pose_Unity cam2gripper);

        /// <summary>
        /// 测试函数：加法
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.StdCall)]
        private static extern int Sum(int a, int b);

        /// <summary>
        /// 测试函数：乘法
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.StdCall)]
        private static extern int Multiplication(int a, int b);

        #endregion

        #region 公共接口封装

        /// <summary>
        /// 测试DLL是否正常加载
        /// </summary>
        public static bool TestDllConnection()
        {
            try
            {
                int result = Sum(3, 5);
                if (result == 8)
                {
                    Debug.Log($"[DLL测试] DLL加载成功！Sum(3, 5) = {result}");
                    return true;
                }
                else
                {
                    Debug.LogError($"[DLL测试] DLL功能异常！Sum(3, 5) = {result} (期望8)");
                    return false;
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"[DLL测试] DLL加载失败: {ex.Message}\n{ex.StackTrace}");
                return false;
            }
        }

        /// <summary>
        /// 执行探针尖端位置计算
        /// </summary>
        /// <param name="markPoses">多组Mark位姿数据（每组包含多次测量）</param>
        /// <param name="needleTip">输出：探针尖端位置</param>
        /// <returns>计算是否成功</returns>
        public static bool CalculateNeedleTip(Point_Unity markPoses, out Vector3D_Unity needleTip)
        {
            needleTip = new Vector3D_Unity();

            try
            {
                int result = calculateNeedleTip(ref markPoses, ref needleTip);

                if (result == 0)
                {
                    Debug.Log($"[探针标定] 计算成功！尖端位置: {needleTip}");
                    return true;
                }
                else
                {
                    Debug.LogError($"[探针标定] 计算失败，错误码: {result}");
                    return false;
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"[探针标定] 调用异常: {ex.Message}\n{ex.StackTrace}");
                return false;
            }
        }

        /// <summary>
        /// 执行手眼标定（眼在手外场景）
        /// </summary>
        /// <param name="gripperToBasePoses">机械臂末端到基座的位姿序列</param>
        /// <param name="targetToCamPoses">标定板到相机的位姿序列</param>
        /// <param name="camToBase">输出：相机到基座的变换关系</param>
        /// <returns>标定是否成功</returns>
        public static bool CalculateHandAndEye(
            Point_Unity gripperToBasePoses,
            Point_Unity targetToCamPoses,
            out Pose_Unity camToBase)
        {
            camToBase = new Pose_Unity();

            try
            {
                // 验证输入数据
                if (gripperToBasePoses.PointNum != targetToCamPoses.PointNum)
                {
                    Debug.LogError($"[手眼标定] 输入数据长度不匹配！" +
                        $"gripper2base: {gripperToBasePoses.PointNum}, " +
                        $"target2cam: {targetToCamPoses.PointNum}");
                    return false;
                }

                if (gripperToBasePoses.PointNum < 3)
                {
                    Debug.LogError($"[手眼标定] 数据点数量不足！至少需要3组，当前: {gripperToBasePoses.PointNum}");
                    return false;
                }

                Debug.Log($"[手眼标定] 开始标定，使用 {gripperToBasePoses.PointNum} 组位姿数据...");

                int result = calculateHandAndEye(
                    ref gripperToBasePoses,
                    ref targetToCamPoses,
                    ref camToBase);

                if (result == 0)
                {
                    Debug.Log($"[手眼标定] 标定成功！\n" +
                        $"相机到基座的变换:\n" +
                        $"  位置: {camToBase.Position}\n" +
                        $"  旋转: {camToBase.Quaternion}");
                    return true;
                }
                else if (result == 1)
                {
                    Debug.LogError($"[手眼标定] 输入数据长度不匹配（DLL返回）");
                    return false;
                }
                else
                {
                    Debug.LogError($"[手眼标定] 标定失败，错误码: {result}");
                    return false;
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"[手眼标定] 调用异常: {ex.Message}\n{ex.StackTrace}");
                return false;
            }
        }

        #endregion

        #region 辅助方法

        /// <summary>
        /// 创建机械臂位姿数据（从6DOF数据：X Y Z RX RY RZ）
        /// </summary>
        public static Point_Unity CreateGripperToBaseData(double[,] robotData)
        {
            if (robotData.GetLength(1) != 6)
                throw new ArgumentException("机械臂数据应为6列 (X Y Z RX RY RZ)");

            int poseCount = robotData.GetLength(0);
            Point_Unity point = Point_Unity.CreateForHandEyeCalibration(poseCount);

            for (int i = 0; i < poseCount; i++)
            {
                Pose_Unity pose = Pose_Unity.FromRobotPose(
                    robotData[i, 0], robotData[i, 1], robotData[i, 2],  // 位置
                    robotData[i, 3], robotData[i, 4], robotData[i, 5]); // 旋转向量
                point.SetPose(0, i, pose);
            }

            return point;
        }

        /// <summary>
        /// 创建相机位姿数据（从7DOF数据：X Y Z QX QY QZ QW）
        /// </summary>
        public static Point_Unity CreateTargetToCamData(double[,] cameraData)
        {
            if (cameraData.GetLength(1) != 7)
                throw new ArgumentException("相机数据应为7列 (X Y Z QX QY QZ QW)");

            int poseCount = cameraData.GetLength(0);
            Point_Unity point = Point_Unity.CreateForHandEyeCalibration(poseCount);

            for (int i = 0; i < poseCount; i++)
            {
                Pose_Unity pose = Pose_Unity.FromCameraPose(
                    cameraData[i, 0], cameraData[i, 1], cameraData[i, 2],  // 位置
                    cameraData[i, 3], cameraData[i, 4], cameraData[i, 5], cameraData[i, 6]); // 四元数
                point.SetPose(0, i, pose);
            }

            return point;
        }

        #endregion
    }
}

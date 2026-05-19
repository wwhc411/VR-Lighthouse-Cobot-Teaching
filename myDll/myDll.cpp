/**
 * @file myDll.cpp
 * @brief DLL核心功能实现文件
 * @details 实现探针标定和手眼标定算法，使用Eigen进行矩阵运算，OpenCV进行标定计算
 */

#include "pch.h"
#include "myDll.h"  
#include "framework.h"  
#include <iostream>
#include <vector>
#include <Eigen>
#include "opencv2/opencv.hpp"

using Eigen::MatrixXd;  // 动态大小的双精度矩阵
using Eigen::VectorXd;  // 动态大小的双精度向量

/**
 * @brief 计算探针尖端位置
 * @details 算法原理：
 *          1. 探针尖端在空间中是固定点，无论探针如何旋转移动，尖端位置不变
 *          2. 对于每次测量，探针Mark在相机坐标系中的位置会变化
 *          3. 通过多次测量，建立超定方程组 Ax = b
 *          4. 使用最小二乘法（QR分解）求解尖端在相机坐标系中的坐标
 *          5. 将相机坐标系下的尖端位置转换到探针坐标系
 *          6. 对所有测量结果取平均，得到最终的探针尖端坐标
 * 
 * @note 参考文献：
 *       opencv+Eigen 手眼标定探针尖端标注（概述）
 *       https://blog.csdn.net/2301_76925998/article/details/145118871
 */
int calculateNeedleTip(Point_Unity* input, Vector3D_Unity* output)
{
    // 提取输入数据的维度信息
    int numMarks = input->MarkNum;   // 探针上Mark标记的数量, 索引用i表示
    int numPoints = input->PointNum; // 每个Mark采集了多少个测量点, 索引用j表示

    // 构建最小二乘法的超定方程组 Ax = b
    // A矩阵：(j-1)*i 行，3 列，系数矩阵
    // b向量：(j-1)*i 个元素，常数项
    // x向量：3个元素（待求的尖端坐标）
    MatrixXd A((numPoints - 1) * numMarks, 3);
    VectorXd B((numPoints - 1) * numMarks);

    // 填充方程组的系数矩阵A和常数向量B
    int row = 0;
    for (int i = 0; i < numMarks; ++i) {      // 遍历每个Mark标记
        for (int j = 1; j < numPoints; ++j) { // 遍历每个测量位姿，跳过第一个作为参考
            // 获取第j次测量的Mark位置（相机坐标系）
            double x_j = input->Points[j + numPoints * i].Position.x;
            double y_j = input->Points[j + numPoints * i].Position.y;
            double z_j = input->Points[j + numPoints * i].Position.z;

            // 获取第1次测量的Mark位置作为参考（相机坐标系）
            double x_1 = input->Points[numPoints * i].Position.x;
            double y_1 = input->Points[numPoints * i].Position.y;
            double z_1 = input->Points[numPoints * i].Position.z;

            // 构建方程：设探针尖端坐标为(px, py, pz)
            // 利用距离约束：||Mark_j - Tip|| = ||Mark_1 - Tip||
            // 展开得到线性方程：2(x_j-x_1)*px + 2(y_j-y_1)*py + 2(z_j-z_1)*pz = ||Mark_j||^2 - ||Mark_1||^2
            A(row, 0) = 2 * (x_j - x_1);
            A(row, 1) = 2 * (y_j - y_1);
            A(row, 2) = 2 * (z_j - z_1);
            B(row++) = x_j * x_j + y_j * y_j + z_j * z_j - (x_1 * x_1 + y_1 * y_1 + z_1 * z_1);
        }
    }

    // 使用列主元QR分解求解最小二乘问题，得到探针尖端在相机坐标系中的坐标
    Eigen::Vector3d result = A.colPivHouseholderQr().solve(B);

    // 将探针尖端坐标从相机坐标系转换到探针坐标系，并对所有测量求平均
    Eigen::Vector3d pointSum_in_probe = Eigen::Vector3d::Zero();
    for (int i = 0; i < (numMarks * numPoints); ++i) {  // 遍历每个Mark的所有测量数据点
        // 通过四元数直接赋值初始化（探针到相机的旋转）
        Eigen::Quaterniond q(
            input->Points[i].Quaternion.w,
            input->Points[i].Quaternion.x,
            input->Points[i].Quaternion.y,
            input->Points[i].Quaternion.z);

        // 将四元数转换为旋转矩阵
        // r_Probe2Camera: 探针坐标系到相机坐标系的旋转矩阵，用于将探针坐标系的点转到相机坐标系
        Eigen::Matrix3d r_Probe2Camera = q.toRotationMatrix();
        
        // T_Probe2Camera: 探针坐标系原点在相机坐标系中的位置（平移向量）
        Eigen::Vector3d T_Probe2Camera(
            input->Points[i].Position.x,
            input->Points[i].Position.y,
            input->Points[i].Position.z);

        // 计算逆变换：相机坐标系到探针坐标系
        // 旋转矩阵的逆等于其转置（正交矩阵性质）
        Eigen::Matrix3d R_inv = r_Probe2Camera.transpose();
       
        // 坐标系转换公式：P_probe = R_inv * (P_camera - T)
        // 将相机坐标系下的尖端位置转换到探针坐标系
        Eigen::Vector3d point_in_camera(result[0], result[1], result[2]);
        Eigen::Vector3d point_in_probe = R_inv * (point_in_camera - T_Probe2Camera);

        // 在probe坐标系下累加所有测量的尖端坐标，用于计算平均值
        pointSum_in_probe = pointSum_in_probe + point_in_probe;
    }

    // 计算探针坐标系下尖端的平均坐标，作为最终结果输出
    output->x = pointSum_in_probe[0] / ((double)numMarks * numPoints);
    output->y = pointSum_in_probe[1] / ((double)numMarks * numPoints);
    output->z = pointSum_in_probe[2] / ((double)numMarks * numPoints);
   
    return 0;  // 返回0表示计算成功
}

/**
 * @brief 手眼标定算法实现（眼在手外场景）
 * @details 【眼在手外标定】：相机固定在机器人外部，观察机械臂末端抓持的标定板
 * 
 *          本函数求解 AX=XB 方程，其中：
 *          - A: gripper相对于base的变换序列（机械臂运动）
 *          - B: target相对于cam的变换序列（相机观测标定板）
 *          - X: cam相对于base的变换（待求解，相机在机器人基座标系中的位姿）
 * 
 *          ⚠️ 重要说明：
 *          OpenCV的calibrateHandEye函数最初设计用于"眼在手上"场景，
 *          但通过适当的输入输出映射，可以用于"眼在手外"场景。
 * 
 *          输入输出映射关系（眼在手外场景）：
 *          ┌─────────────────────┬──────────────────────┬─────────────────────┐
 *          │  本函数参数名称      │  Tsai算法中的符号     │  实际物理含义        │
 *          ├─────────────────────┼──────────────────────┼─────────────────────┤
 *          │  R_gripper2base     │  R_base2gripper^-1   │  末端到基座的旋转    │
 *          │  t_gripper2base     │  T_base2gripper^-1   │  末端到基座的平移    │
 *          │  R_target2cam       │  R_target2cam        │  标定板到相机的旋转  │
 *          │  t_target2cam       │  T_target2cam        │  标定板到相机的平移  │
 *          │  R_cam2gripper(输出) │  R_cam2base          │  相机到基座的旋转    │
 *          │  t_cam2gripper(输出) │  T_cam2base          │  相机到基座的平移    │
 *          └─────────────────────┴──────────────────────┴─────────────────────┘
 * 
 *          算法流程：
 *          1. 验证输入数据长度一致性
 *          2. 将Eigen四元数和位置转换为OpenCV Mat格式
 *          3. 调用OpenCV的calibrateHandEye函数（Tsai算法）
 *          4. 将结果从OpenCV Mat转换回Eigen格式
 *          5. 将旋转矩阵转换为四元数输出
 * 
 * @note 参考文献：
 *       - https://blog.csdn.net/qq_19319481/article/details/150462358
 *       - https://docs.opencv.org/4.5.3/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b
 *       - https://www.bilibili.com/video/BV1By4y1b7Q7
 */
int calculateHandAndEye(Point_Unity* gripper2base, Point_Unity* target2cam, Pose_Unity* cam2gripper)
{
    // 输入数据验证：两组数据必须长度相同，一一对应
    if (gripper2base->PointNum != target2cam->PointNum) return 1;

    // 获取采集的位姿对数量，每对数据对应机械臂的一个不同抓取位置
    int num_poses = gripper2base->PointNum;

    // 【眼在手外场景说明】
    // 相机固定在机器人外部，机械臂末端抓持标定板
    // 采集过程：机械臂移动到多个不同位姿，相机观察标定板，记录两组数据
    
    // 存储 gripper 相对于 base 的旋转矩阵和位移向量序列
    // 这些数据来自机械臂控制器，描述末端执行器在基座标系中的位姿
    // 在眼在手外场景中：标定板固定在末端，随机械臂一起运动
    // 修复：使用reserve()而不是构造函数，避免预分配空元素
    std::vector< cv::Mat > R_gripper2base;
    R_gripper2base.reserve(num_poses);
    std::vector< cv::Mat > t_gripper2base;
    t_gripper2base.reserve(num_poses);

    // 存储 target(标定板) 相对于 cam(相机) 的旋转矩阵和位移向量序列
    // 这些数据通过视觉算法（如solvePnP）计算得到，描述标定板在相机坐标系中的位姿
    // 在眼在手外场景中：相机固定不动，观察随机械臂移动的标定板
    std::vector< cv::Mat > R_target2cam;
    R_target2cam.reserve(num_poses);
    std::vector< cv::Mat > t_target2cam;
    t_target2cam.reserve(num_poses);

    // 遍历所有采集的位姿对，进行数据格式转换
    for (int i = 0; i < num_poses; ++i) {  // 修复：从i=0开始遍历所有数据
        // ========== 处理 gripper 到 base 的变换 ==========
        // 通过直接赋值方式初始化四元数（Eigen格式）
        Eigen::Quaterniond q_gripper2base(
            gripper2base->Points[i].Quaternion.w,
            gripper2base->Points[i].Quaternion.x,
            gripper2base->Points[i].Quaternion.y,
            gripper2base->Points[i].Quaternion.z);

        // 将四元数转换为3x3旋转矩阵
        Eigen::Matrix3d r_gripper2base = q_gripper2base.toRotationMatrix();
        
        // 提取位移向量
        Eigen::Vector3d T_gripper2base(
            gripper2base->Points[i].Position.x,
            gripper2base->Points[i].Position.y,
            gripper2base->Points[i].Position.z);

        // 将Eigen旋转矩阵转换为OpenCV Mat格式（列优先存储）
        cv::Mat R_gripper2base_(3, 3, CV_64FC1);
        R_gripper2base_.at<double>(0, 0) = r_gripper2base(0, 0);
        R_gripper2base_.at<double>(1, 0) = r_gripper2base(1, 0);
        R_gripper2base_.at<double>(2, 0) = r_gripper2base(2, 0);
        R_gripper2base_.at<double>(0, 1) = r_gripper2base(0, 1);
        R_gripper2base_.at<double>(1, 1) = r_gripper2base(1, 1);
        R_gripper2base_.at<double>(2, 1) = r_gripper2base(2, 1);
        R_gripper2base_.at<double>(0, 2) = r_gripper2base(0, 2);
        R_gripper2base_.at<double>(1, 2) = r_gripper2base(1, 2);
        R_gripper2base_.at<double>(2, 2) = r_gripper2base(2, 2);

        // 将Eigen位移向量转换为OpenCV Mat格式（3x1列向量）
        cv::Mat t_gripper2base_(3, 1, CV_64FC1);
        t_gripper2base_.at<double>(0, 0) = T_gripper2base[0];
        t_gripper2base_.at<double>(1, 0) = T_gripper2base[1];
        t_gripper2base_.at<double>(2, 0) = T_gripper2base[2];

        // 添加到向量序列中
        R_gripper2base.push_back(R_gripper2base_);
        t_gripper2base.push_back(t_gripper2base_);

        // ========== 处理 target 到 cam 的变换 ==========
        // 通过直接赋值方式初始化四元数（Eigen格式）
        Eigen::Quaterniond q_target2cam(
            target2cam->Points[i].Quaternion.w,
            target2cam->Points[i].Quaternion.x,
            target2cam->Points[i].Quaternion.y,
            target2cam->Points[i].Quaternion.z);

        // 将四元数转换为3x3旋转矩阵
        Eigen::Matrix3d r_target2cam = q_target2cam.toRotationMatrix();
        
        // 提取位移向量
        Eigen::Vector3d T_target2cam(
            target2cam->Points[i].Position.x,
            target2cam->Points[i].Position.y,
            target2cam->Points[i].Position.z);

        // 将Eigen旋转矩阵转换为OpenCV Mat格式（列优先存储）
        cv::Mat R_target2cam_(3, 3, CV_64FC1);
        R_target2cam_.at<double>(0, 0) = r_target2cam(0, 0);
        R_target2cam_.at<double>(1, 0) = r_target2cam(1, 0);
        R_target2cam_.at<double>(2, 0) = r_target2cam(2, 0);
        R_target2cam_.at<double>(0, 1) = r_target2cam(0, 1);
        R_target2cam_.at<double>(1, 1) = r_target2cam(1, 1);
        R_target2cam_.at<double>(2, 1) = r_target2cam(2, 1);
        R_target2cam_.at<double>(0, 2) = r_target2cam(0, 2);
        R_target2cam_.at<double>(1, 2) = r_target2cam(1, 2);
        R_target2cam_.at<double>(2, 2) = r_target2cam(2, 2);

        // 将Eigen位移向量转换为OpenCV Mat格式（3x1列向量）
        cv::Mat t_target2cam_(3, 1, CV_64FC1);
        t_target2cam_.at<double>(0, 0) = T_target2cam[0];
        t_target2cam_.at<double>(1, 0) = T_target2cam[1];
        t_target2cam_.at<double>(2, 0) = T_target2cam[2];

        // 添加到向量序列中
        R_target2cam.push_back(R_target2cam_);
        t_target2cam.push_back(t_target2cam_);
    }

    // ========== 执行手眼标定计算 ==========
    // 存储标定结果：相机相对于机器人基座的旋转和位移（眼在手外场景）
    // 注意：虽然变量名为cam2gripper，但在眼在手外场景下实际表示cam2base
    cv::Mat R_cam2gripper, t_cam2gripper;

    // 调用OpenCV手眼标定函数，使用Tsai算法
    // 在眼在手外场景下的AX=XB方程求解：
    // - A = gripper2base（机械臂运动）
    // - B = target2cam（相机观测）
    // - X = cam2base（求解目标：相机相对于机器人基座的位姿）
    // 
    // 变量名映射（眼在手外场景）：
    // - R_cam2gripper（输出）实际代表 R_cam2base（相机到基座的旋转）
    // - t_cam2gripper（输出）实际代表 T_cam2base（相机到基座的平移）
    cv::calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper, cv::CALIB_HAND_EYE_TSAI);

    // ========== 将标定结果转换回Eigen格式 ==========
    // 将OpenCV Mat格式的旋转矩阵转换为Eigen Matrix3d
    // 在眼在手外场景下：这是相机相对于机器人基座的旋转矩阵（R_cam2base）
    Eigen::Matrix3d r_cam2gripper;
    r_cam2gripper(0, 0) = R_cam2gripper.at<double>(0, 0);
    r_cam2gripper(1, 0) = R_cam2gripper.at<double>(1, 0);
    r_cam2gripper(2, 0) = R_cam2gripper.at<double>(2, 0);
    r_cam2gripper(0, 1) = R_cam2gripper.at<double>(0, 1);
    r_cam2gripper(1, 1) = R_cam2gripper.at<double>(1, 1);
    r_cam2gripper(2, 1) = R_cam2gripper.at<double>(2, 1);
    r_cam2gripper(0, 2) = R_cam2gripper.at<double>(0, 2);
    r_cam2gripper(1, 2) = R_cam2gripper.at<double>(1, 2);
    r_cam2gripper(2, 2) = R_cam2gripper.at<double>(2, 2);
    
    // 将旋转矩阵转换为四元数（Eigen会自动归一化）
    // 在眼在手外场景下：这是相机相对于机器人基座的旋转四元数（Q_cam2base）
    Eigen::Quaterniond q_cam2gripper(r_cam2gripper);

    // 将四元数赋值给输出结构体
    // 注意：虽然参数名为cam2gripper，但在眼在手外场景下，实际输出的是cam2base
    cam2gripper->Quaternion.w = q_cam2gripper.w();
    cam2gripper->Quaternion.x = q_cam2gripper.x();
    cam2gripper->Quaternion.y = q_cam2gripper.y();
    cam2gripper->Quaternion.z = q_cam2gripper.z();

    // 将位移向量赋值给输出结构体
    // 在眼在手外场景下：这是相机原点在机器人基座标系中的位置（T_cam2base）
    cam2gripper->Position.x = t_cam2gripper.at<double>(0, 0);
    cam2gripper->Position.y = t_cam2gripper.at<double>(1, 0);
    cam2gripper->Position.z = t_cam2gripper.at<double>(2, 0);

    return 0;  // 返回0表示标定成功
}

/**
 * @brief 整数加法测试函数
 * @details 用于测试DLL导出和调用功能是否正常工作
 * @param[in] value1 第一个加数
 * @param[in] value2 第二个加数
 * @return 两数之和
 */
int Sum(int value1, int value2)
{
    int sumValue = value1 + value2;
    return sumValue;
}

/**
 * @brief 整数乘法测试函数
 * @details 用于测试DLL导出和调用功能是否正常工作
 * @param[in] value1 第一个乘数
 * @param[in] value2 第二个乘数
 * @return 两数之积
 */
int Multiplication(int value1, int value2)
{
    return value1 * value2;
}

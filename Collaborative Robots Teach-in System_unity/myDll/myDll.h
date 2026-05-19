/**
 * @file myDll.h
 * @brief 机器视觉和机器人标定动态链接库头文件
 * @details 本DLL提供探针尖端位置计算和手眼标定功能，用于机器人视觉系统
 * @author 
 * @date 2025
 * @version 1.0
 */

#ifndef _ENCRYPTBASE_H
#define _ENCRYPTBASE_H

#include <Eigen>

/**
 * @brief 使用C语言方式导出函数
 * @details extern "C" 告诉编译器使用C语言的函数调用约定和名称修饰规则
 *          这样可以确保生成的DLL可以被C、C++、C#等多种语言调用
 *          __cplusplus 是C++编译器预定义的宏，用于判断是否在C++环境中编译
 */
extern "C" 
{
	/**
	 * @struct Vector3D_Unity
	 * @brief 三维向量结构体，用于表示空间中的位置或方向
	 * @details 使用双精度浮点数存储，适用于高精度计算场景
	 */
	typedef struct {
		double x;  ///< X轴坐标分量
		double y;  ///< Y轴坐标分量
		double z;  ///< Z轴坐标分量
	} Vector3D_Unity;

	/**
	 * @struct Quaternion_Unity
	 * @brief 四元数结构体，用于表示三维空间中的旋转
	 * @details 四元数避免了欧拉角的万向锁问题，适合进行旋转插值和累积
	 *          满足归一化条件：w^2 + x^2 + y^2 + z^2 = 1
	 */
	typedef struct {
		double w;  ///< 四元数实部（标量部分）
		double x;  ///< 四元数虚部i分量
		double y;  ///< 四元数虚部j分量
		double z;  ///< 四元数虚部k分量
	} Quaternion_Unity;

	/**
	 * @struct Pose_Unity
	 * @brief 位姿结构体，完整描述刚体在三维空间中的位置和姿态
	 * @details 位姿 = 位置（平移） + 姿态（旋转）
	 *          常用于机器人学中描述末端执行器、相机等的空间状态
	 */
	typedef struct {
		Vector3D_Unity Position;      ///< 位置信息（平移向量）
		Quaternion_Unity Quaternion;  ///< 姿态信息（旋转四元数）
	}Pose_Unity;

	/**
	 * @struct Point_Unity
	 * @brief 点云数据结构体，存储多个标记点（Mark）的多次测量位姿
	 * @details 用于存储探针标定或手眼标定过程中采集的多组位姿数据
	 *          数据组织方式：Points[j + PointNum * i] 表示第i个Mark的第j次测量
	 */
	typedef struct {
		int        MarkNum;      ///< 探针上Mark标记的数量（标记点个数）
		int        PointNum;     ///< 每个Mark采集了多少个测量点（采样次数）
		Pose_Unity Points[1024]; ///< 位姿数组，存储所有测量数据（最多1024个）
	}Point_Unity;

	/**
	 * @brief 计算探针尖端在探针坐标系中的位置
	 * @details 使用最小二乘法，通过多次测量不同姿态下的探针Mark位置，
	 *          计算出固定不动的探针尖端位置。基于Eigen库实现SVD求解。
	 * 
	 * @param[in]  input  输入的探针Mark位姿测量数据
	 *                    - MarkNum: 探针上的Mark数量
	 *                    - PointNum: 每个Mark采集的测量次数
	 *                    - Points: 所有测量的位姿数据（相机坐标系）
	 * @param[out] output 计算得到的探针尖端位置（探针坐标系）
	 * 
	 * @return 返回状态码
	 *         - 0: 计算成功
	 * 
	 * @note 算法原理参考：https://blog.csdn.net/2301_76925998/article/details/145118871
	 * @note 需要至少2个不同姿态的测量数据才能求解
	 */
	_declspec(dllexport) int _stdcall calculateNeedleTip(Point_Unity* input, Vector3D_Unity* output);

	/**
	 * @brief 手眼标定函数（眼在手外场景），计算相机相对于机器人基座的变换关系
	 * @details 眼在手外标定（Eye-to-Hand Calibration）求解方程 AX=XB，其中：
	 *          - A: 机械臂末端在基座标系中的变换（gripper2base）
	 *          - B: 标定板在相机坐标系中的变换（target2cam）
	 *          - X: 相机相对于基座的变换（cam2base，求解目标）
	 * 
	 *          注意：OpenCV的calibrateHandEye函数使用Tsai算法，其输入输出含义
	 *          在眼在手外场景下的对应关系为：
	 *          - R_gripper2base (输入) <=> R_base2gripper (Tsai算法中的机械臂运动)
	 *          - T_gripper2base (输入) <=> T_base2gripper (Tsai算法中的机械臂运动)
	 *          - R_target2cam   (输入) <=> R_target2cam   (Tsai算法中的相机观测)
	 *          - T_target2cam   (输入) <=> T_target2cam   (Tsai算法中的相机观测)
	 *          - R_cam2gripper  (输出) <=> R_cam2base     (实际求解的相机相对基座的旋转)
	 *          - T_cam2gripper  (输出) <=> T_cam2base     (实际求解的相机相对基座的平移)
	 * 
	 * @param[in]  gripper2base 机械臂末端相对于基座的位姿序列
	 *                          每次移动机械臂到不同位置时记录一组数据
	 * @param[in]  target2cam   标定板相对于相机的位姿序列
	 *                          与gripper2base一一对应，同一时刻采集
	 * @param[out] cam2gripper  计算得到的相机相对于基座的位姿关系（实际为cam2base）
	 *                          注意：虽然参数名为cam2gripper，但在眼在手外场景下，
	 *                          实际输出的是相机相对于机器人基座的变换关系
	 * 
	 * @return 返回状态码
	 *         - 0: 标定成功
	 *         - 1: 输入数据长度不匹配（两组数据PointNum不相等）
	 * 
	 * @note 算法参考：
	 *       - https://blog.csdn.net/qq_19319481/article/details/150462358
	 *       - https://docs.opencv.org/4.5.3/d9/d0c/group__calib3d.html
	 * @note 建议至少采集5-10组不同姿态的数据以获得稳定结果
	 * @note 眼在手外场景：相机固定在机器人外部，观察机械臂末端抓持的标定板
	 */
	_declspec(dllexport) int _stdcall calculateHandAndEye(Point_Unity* gripper2base, Point_Unity* target2cam, Pose_Unity* cam2gripper);

	/**
	 * @brief 整数加法测试函数
	 * @param[in] value1 加数1
	 * @param[in] value2 加数2
	 * @return 两数之和
	 * @note 用于测试DLL导出功能是否正常
	 */
	_declspec(dllexport) int _stdcall Sum(int value1, int value2);

	/**
	 * @brief 整数乘法测试函数
	 * @param[in] value1 乘数1
	 * @param[in] value2 乘数2
	 * @return 两数之积
	 * @note 用于测试DLL导出功能是否正常
	 */
	_declspec(dllexport) int _stdcall Multiplication(int value1, int value2);
}

#endif
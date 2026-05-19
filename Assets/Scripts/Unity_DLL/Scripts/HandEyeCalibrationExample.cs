using UnityEngine;
using HandEyeCalibration.DLL;

namespace HandEyeCalibration
{
    /// <summary>
    /// 手眼标定示例和可视化
    /// 提供使用示例和Scene视图中的Gizmos可视化
    /// </summary>
    public class HandEyeCalibrationExample : MonoBehaviour
    {
        [Header("可视化设置")]
        [SerializeField] private bool showGizmos = true;
        [SerializeField] private float gizmoSize = 0.1f;
        [SerializeField] private Color robotColor = Color.red;
        [SerializeField] private Color cameraColor = Color.blue;

        [Header("示例数据")]
        [SerializeField] private bool runExampleOnStart = false;
        
        private HandEyeCalibrationManager manager;
        private CalibrationResult lastResult;

        private void Start()
        {
            manager = GetComponent<HandEyeCalibrationManager>();
            
            if (runExampleOnStart && manager != null)
            {
                RunExample();
            }
        }

        /// <summary>
        /// 运行完整的标定示例
        /// </summary>
        [ContextMenu("运行标定示例")]
        public void RunExample()
        {
            if (manager == null)
            {
                Debug.LogError("[示例] 未找到HandEyeCalibrationManager组件！");
                return;
            }

            Debug.Log("=== 开始手眼标定示例 ===");

            // 1. 测试DLL连接
            Debug.Log("步骤1: 测试DLL连接...");
            if (!DllInterface.TestDllConnection())
            {
                Debug.LogError("[示例] DLL连接失败，请检查Plugins文件夹中的myDll.dll");
                return;
            }

            // 2. 加载示例数据
            Debug.Log("步骤2: 加载示例数据...");
            manager.LoadSampleData();
            Debug.Log($"已加载 {manager.GetDataCount()} 组标定数据");

            // 3. 执行标定
            Debug.Log("步骤3: 执行标定...");
            lastResult = manager.PerformCalibration();

            // 4. 显示结果
            if (lastResult.IsSuccess)
            {
                Debug.Log("=== 标定成功 ===");
                Debug.Log($"相机位置: {lastResult.Position}");
                Debug.Log($"相机旋转: {lastResult.Rotation.eulerAngles}");
                Debug.Log("\n完整结果:\n" + lastResult.ResultText);

                // 5. 测试坐标变换
                Debug.Log("\n步骤4: 测试坐标变换...");
                TestCoordinateTransform();
            }
            else
            {
                Debug.LogError($"=== 标定失败 ===\n{lastResult.ErrorMessage}");
            }
        }

        /// <summary>
        /// 测试坐标变换功能
        /// </summary>
        private void TestCoordinateTransform()
        {
            if (manager == null) return;

            // 测试几个示例点
            Vector3[] testPoints = new Vector3[]
            {
                new Vector3(0, 0, 0),      // 原点
                new Vector3(1, 0, 0),      // X轴
                new Vector3(0, 1, 0),      // Y轴
                new Vector3(0, 0, 1),      // Z轴
                new Vector3(0.5f, 0.5f, 0.5f)  // 对角线
            };

            Debug.Log("坐标变换测试（相机→基座）:");
            foreach (var point in testPoints)
            {
                Vector3 transformed = manager.TransformPointCameraToBase(point);
                Debug.Log($"  {point} → {transformed}");
            }
        }

        /// <summary>
        /// 在Scene视图中绘制Gizmos
        /// </summary>
        private void OnDrawGizmos()
        {
            if (!showGizmos || lastResult == null || !lastResult.IsSuccess) return;

            // 绘制相机坐标系（在机器人基座坐标系中的位置）
            Matrix4x4 camMatrix = lastResult.TransformMatrix;
            
            Gizmos.color = Color.white;
            Gizmos.DrawWireCube(lastResult.Position, Vector3.one * gizmoSize);

            // 绘制坐标轴
            DrawAxis(lastResult.Position, lastResult.Rotation, gizmoSize);

            // 绘制标签
            #if UNITY_EDITOR
            UnityEditor.Handles.Label(lastResult.Position + Vector3.up * gizmoSize * 2, 
                $"相机位置\n{lastResult.Position:F3}");
            #endif
        }

        /// <summary>
        /// 绘制坐标轴
        /// </summary>
        private void DrawAxis(Vector3 position, Quaternion rotation, float size)
        {
            Vector3 right = rotation * Vector3.right * size;
            Vector3 up = rotation * Vector3.up * size;
            Vector3 forward = rotation * Vector3.forward * size;

            // X轴 - 红色
            Gizmos.color = Color.red;
            Gizmos.DrawRay(position, right);

            // Y轴 - 绿色
            Gizmos.color = Color.green;
            Gizmos.DrawRay(position, up);

            // Z轴 - 蓝色
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(position, forward);
        }

        #region 代码示例

        /// <summary>
        /// 示例：如何在代码中使用手眼标定
        /// </summary>
        public void CodeExample()
        {
            // 获取管理器
            var calibrationManager = GetComponent<HandEyeCalibrationManager>();

            // 方式1：添加单组数据
            var robotPose = new RobotPoseData(100, 200, 300, 0.1, 0.2, 0.3);
            var cameraPose = new CameraPoseData(50, 60, 70, 0, 0, 0, 1);
            calibrationManager.AddCalibrationData(robotPose, cameraPose);

            // 方式2：批量添加数据（从文本）
            string robotData = "100 200 300 0.1 0.2 0.3\n150 250 350 0.15 0.25 0.35";
            string cameraData = "50 60 70 0.7 0 0 0.7\n55 65 75 0.6 0.1 0.1 0.8";
            calibrationManager.AddCalibrationDataFromText(robotData, cameraData);

            // 方式3：加载示例数据
            calibrationManager.LoadSampleData();

            // 执行标定
            var result = calibrationManager.PerformCalibration();

            if (result.IsSuccess)
            {
                // 使用标定结果进行坐标变换
                Vector3 pointInCamera = new Vector3(1, 2, 3);
                Vector3 pointInBase = calibrationManager.TransformPointCameraToBase(pointInCamera);
                Debug.Log($"变换结果: {pointInCamera} → {pointInBase}");

                // 访问变换矩阵
                Matrix4x4 transformMatrix = result.TransformMatrix;
                Vector3 cameraPosition = result.Position;
                Quaternion cameraRotation = result.Rotation;
            }
        }

        /// <summary>
        /// 示例：直接调用DLL接口（高级用法）
        /// </summary>
        public void DirectDllExample()
        {
            // 准备数据
            double[,] robotData = new double[5, 6];
            double[,] cameraData = new double[5, 7];

            // 填充数据...
            for (int i = 0; i < 5; i++)
            {
                robotData[i, 0] = i * 100;  // X
                robotData[i, 1] = i * 100;  // Y
                robotData[i, 2] = 500;      // Z
                robotData[i, 3] = 0.1;      // RX
                robotData[i, 4] = 0.2;      // RY
                robotData[i, 5] = 0.3;      // RZ

                cameraData[i, 0] = i * 50;  // X
                cameraData[i, 1] = i * 50;  // Y
                cameraData[i, 2] = 300;     // Z
                cameraData[i, 3] = 0;       // QX
                cameraData[i, 4] = 0;       // QY
                cameraData[i, 5] = 0.707;   // QZ
                cameraData[i, 6] = 0.707;   // QW
            }

            // 创建DLL输入
            Point_Unity gripperToBase = DllInterface.CreateGripperToBaseData(robotData);
            Point_Unity targetToCam = DllInterface.CreateTargetToCamData(cameraData);

            // 调用DLL
            Pose_Unity camToBase;
            bool success = DllInterface.CalculateHandAndEye(
                gripperToBase,
                targetToCam,
                out camToBase);

            if (success)
            {
                Debug.Log($"DLL标定成功！位置: {camToBase.Position}");
                Debug.Log($"旋转: {camToBase.Quaternion}");
            }
        }

        #endregion
    }
}

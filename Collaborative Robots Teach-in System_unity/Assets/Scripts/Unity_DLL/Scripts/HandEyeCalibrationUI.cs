using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System;

namespace HandEyeCalibration
{
    /// <summary>
    /// 手眼标定UI控制器
    /// 负责UI交互和数据显示
    /// </summary>
    [RequireComponent(typeof(HandEyeCalibrationManager))]
    public class HandEyeCalibrationUI : MonoBehaviour
    {
        [Header("核心组件")]
        [SerializeField] private HandEyeCalibrationManager calibrationManager;
        [SerializeField] private ViveTrackerPoseLogger trackerPoseLogger;

        [Header("输入区域")]
        [SerializeField] private TMP_InputField robotDataInput;
        [SerializeField] private TMP_InputField cameraDataInput;
        
        [Header("按钮")]
        [SerializeField] private Button captureDataButton;
        [SerializeField] private Button loadSampleButton;
        [SerializeField] private Button calibrateButton;
        [SerializeField] private Button clearDataButton;
        [SerializeField] private Button saveResultButton;
        [SerializeField] private Button testTransformButton;
        
        [Header("数据采集设置")]
        [Tooltip("Tracker设备ID(默认为1)")]
        [SerializeField] private uint trackerDeviceId = 1;

        [Header("结果显示")]
        [SerializeField] private TMP_Text resultText;
        [SerializeField] private TMP_Text statusText;
        [SerializeField] private TMP_Text dataCountText;

        [Header("变换测试面板")]
        [SerializeField] private GameObject transformTestPanel;
        [SerializeField] private TMP_InputField testInputX;
        [SerializeField] private TMP_InputField testInputY;
        [SerializeField] private TMP_InputField testInputZ;
        [SerializeField] private TMP_Text testOutputText;
        [SerializeField] private Button executeTransformButton;

        private void Awake()
        {
            // 自动获取管理器组件
            if (calibrationManager == null)
            {
                calibrationManager = GetComponent<HandEyeCalibrationManager>();
            }
        }

        private void Start()
        {
            // 绑定按钮事件
            if (captureDataButton != null)
                captureDataButton.onClick.AddListener(OnCaptureData);
            
            if (loadSampleButton != null)
                loadSampleButton.onClick.AddListener(OnLoadSampleData);
            
            if (calibrateButton != null)
                calibrateButton.onClick.AddListener(OnCalibrate);
            
            if (clearDataButton != null)
                clearDataButton.onClick.AddListener(OnClearData);
            
            if (saveResultButton != null)
            {
                saveResultButton.onClick.AddListener(OnSaveResult);
                saveResultButton.interactable = false;
            }
            
            if (testTransformButton != null)
            {
                testTransformButton.onClick.AddListener(OnTestTransform);
                testTransformButton.interactable = false;
            }

            if (executeTransformButton != null)
                executeTransformButton.onClick.AddListener(OnExecuteTransform);
            
            // 检查ViveTrackerPoseLogger组件
            if (trackerPoseLogger == null)
            {
                trackerPoseLogger = FindObjectOfType<ViveTrackerPoseLogger>();
                if (trackerPoseLogger == null)
                {
                    Debug.LogWarning("[HandEyeCalibrationUI] 未找到ViveTrackerPoseLogger组件,数据采集功能将不可用");
                }
            }

            // 订阅管理器事件
            if (calibrationManager != null)
            {
                calibrationManager.OnCalibrationCompleted += OnCalibrationCompleted;
                calibrationManager.OnStatusUpdated += OnStatusUpdated;
            }

            // 初始化变换测试面板
            if (transformTestPanel != null)
                transformTestPanel.SetActive(false);

            UpdateDataCountDisplay();
        }

        private void OnDestroy()
        {
            // 取消订阅
            if (calibrationManager != null)
            {
                calibrationManager.OnCalibrationCompleted -= OnCalibrationCompleted;
                calibrationManager.OnStatusUpdated -= OnStatusUpdated;
            }
        }

        #region 按钮事件处理

        /// <summary>
        /// 捕获一帧数据(TCP + Tracker)
        /// </summary>
        private void OnCaptureData()
        {
            if (trackerPoseLogger == null)
            {
                Debug.LogError("[HandEyeCalibrationUI] ViveTrackerPoseLogger未设置,无法捕获数据");
                if (statusText != null)
                {
                    statusText.text = "[" + DateTime.Now.ToString("HH:mm:ss") + "] 错误:未找到ViveTrackerPoseLogger组件";
                }
                return;
            }

            // 获取TCP位姿(机器人数据)
            Vector3 tcpPositionMm;
            Vector3 tcpRotationRad;
            if (!trackerPoseLogger.GetRobotTcpPoseForCalibration(out tcpPositionMm, out tcpRotationRad))
            {
                Debug.LogError("[HandEyeCalibrationUI] 无法获取TCP数据");
                if (statusText != null)
                {
                    statusText.text = "[" + DateTime.Now.ToString("HH:mm:ss") + "] 错误:无法获取TCP数据,请检查机器人连接";
                }
                return;
            }

            // 获取Tracker位姿(相机数据)
            Vector3 trackerPositionMm;
            Quaternion trackerRotation;
            if (!trackerPoseLogger.GetTrackerPoseForCalibration(trackerDeviceId, out trackerPositionMm, out trackerRotation))
            {
                Debug.LogError($"[HandEyeCalibrationUI] 无法获取Tracker[ID:{trackerDeviceId}]数据");
                if (statusText != null)
                {
                    statusText.text = $"[{DateTime.Now.ToString("HH:mm:ss")}] 错误:无法获取Tracker[ID:{trackerDeviceId}]数据,请检查设备连接";
                }
                return;
            }

            // 格式化机器人数据行: X Y Z RX RY RZ (mm, mm, mm, rad, rad, rad)
            string robotDataLine = string.Format(System.Globalization.CultureInfo.InvariantCulture,
                "{0:F2} {1:F2} {2:F2} {3:F4} {4:F4} {5:F4}",
                tcpPositionMm.x, tcpPositionMm.y, tcpPositionMm.z,
                tcpRotationRad.x, tcpRotationRad.y, tcpRotationRad.z);

            // 格式化相机数据行: X Y Z QX QY QZ QW (mm, mm, mm, qx, qy, qz, qw)
            string cameraDataLine = string.Format(System.Globalization.CultureInfo.InvariantCulture,
                "{0:F2} {1:F2} {2:F2} {3:F4} {4:F4} {5:F4} {6:F4}",
                trackerPositionMm.x, trackerPositionMm.y, trackerPositionMm.z,
                trackerRotation.x, trackerRotation.y, trackerRotation.z, trackerRotation.w);

            // 追加到输入框
            if (robotDataInput != null)
            {
                if (!string.IsNullOrEmpty(robotDataInput.text))
                {
                    robotDataInput.text += "\n";
                }
                robotDataInput.text += robotDataLine;
            }

            if (cameraDataInput != null)
            {
                if (!string.IsNullOrEmpty(cameraDataInput.text))
                {
                    cameraDataInput.text += "\n";
                }
                cameraDataInput.text += cameraDataLine;
            }

            // 更新状态显示
            int lineCount = string.IsNullOrEmpty(robotDataInput.text) ? 0 : robotDataInput.text.Split('\n').Length;
            if (statusText != null)
            {
                statusText.text = $"[{DateTime.Now.ToString("HH:mm:ss")}] 数据已捕获 (共{lineCount}组)";
            }
            
            Debug.Log($"[HandEyeCalibrationUI] 数据已捕获:\n机器人: {robotDataLine}\n相机: {cameraDataLine}");
        }

        /// <summary>
        /// 公共方法：供外部脚本(如GripButtonUI)调用，触发数据采集
        /// 用于通过Power物理按钮或UI按钮触发手眼标定数据采集
        /// </summary>
        public void CaptureCalibrationData()
        {
            OnCaptureData();
        }

        /// <summary>
        /// 公共方法：供外部脚本调用，使用指定的设备ID触发数据采集
        /// </summary>
        /// <param name="deviceId">要使用的Tracker设备ID</param>
        public void CaptureCalibrationData(uint deviceId)
        {
            uint originalId = trackerDeviceId;
            trackerDeviceId = deviceId;
            OnCaptureData();
            trackerDeviceId = originalId;
        }

        /// <summary>
        /// 公共方法：供外部脚本调用，使用相对位姿数据进行标定
        /// 用于捕获相对于参考设备的相对位姿作为Tracker数据输入
        /// </summary>
        /// <param name="relativePosition">相对位置（米）</param>
        /// <param name="relativeRotation">相对旋转（四元数）</param>
        public void CaptureCalibrationDataWithRelativePose(Vector3 relativePosition, Quaternion relativeRotation)
        {
            if (trackerPoseLogger == null)
            {
                Debug.LogError("[HandEyeCalibrationUI] ViveTrackerPoseLogger未设置,无法捕获数据");
                if (statusText != null)
                {
                    statusText.text = "[" + DateTime.Now.ToString("HH:mm:ss") + "] 错误:未找到ViveTrackerPoseLogger组件";
                }
                return;
            }

            // 获取TCP位姿(机器人数据) - 仍然从原逻辑获取
            Vector3 tcpPositionMm;
            Vector3 tcpRotationRad;
            if (!trackerPoseLogger.GetRobotTcpPoseForCalibration(out tcpPositionMm, out tcpRotationRad))
            {
                Debug.LogError("[HandEyeCalibrationUI] 无法获取TCP数据");
                if (statusText != null)
                {
                    statusText.text = "[" + DateTime.Now.ToString("HH:mm:ss") + "] 错误:无法获取TCP数据,请检查机器人连接";
                }
                return;
            }

            // 使用传入的相对位姿数据
            // 将位置从米转换为毫米
            Vector3 trackerPositionMm = relativePosition * 1000f;
            Quaternion trackerRotation = relativeRotation;

            // 格式化机器人数据行: X Y Z RX RY RZ (mm, mm, mm, rad, rad, rad)
            string robotDataLine = string.Format(System.Globalization.CultureInfo.InvariantCulture,
                "{0:F2} {1:F2} {2:F2} {3:F4} {4:F4} {5:F4}",
                tcpPositionMm.x, tcpPositionMm.y, tcpPositionMm.z,
                tcpRotationRad.x, tcpRotationRad.y, tcpRotationRad.z);

            // 格式化相机数据行(使用相对位姿): X Y Z QX QY QZ QW (mm, mm, mm, qx, qy, qz, qw)
            string cameraDataLine = string.Format(System.Globalization.CultureInfo.InvariantCulture,
                "{0:F2} {1:F2} {2:F2} {3:F4} {4:F4} {5:F4} {6:F4}",
                trackerPositionMm.x, trackerPositionMm.y, trackerPositionMm.z,
                trackerRotation.x, trackerRotation.y, trackerRotation.z, trackerRotation.w);

            // 追加到输入框
            if (robotDataInput != null)
            {
                if (!string.IsNullOrEmpty(robotDataInput.text))
                {
                    robotDataInput.text += "\n";
                }
                robotDataInput.text += robotDataLine;
            }

            if (cameraDataInput != null)
            {
                if (!string.IsNullOrEmpty(cameraDataInput.text))
                {
                    cameraDataInput.text += "\n";
                }
                cameraDataInput.text += cameraDataLine;
            }

            // 更新状态显示
            int lineCount = string.IsNullOrEmpty(robotDataInput.text) ? 0 : robotDataInput.text.Split('\n').Length;
            if (statusText != null)
            {
                statusText.text = $"[{DateTime.Now.ToString("HH:mm:ss")}] 相对位姿数据已捕获 (共{lineCount}组)";
            }
            
            Debug.Log($"[HandEyeCalibrationUI] 相对位姿数据已捕获:\n机器人: {robotDataLine}\n相机(相对位姿): {cameraDataLine}");
        }

        /// <summary>
        /// 加载示例数据
        /// </summary>
        private void OnLoadSampleData()
        {
            if (calibrationManager == null) return;

            calibrationManager.LoadSampleData();
            
            // 更新UI显示（从管理器获取示例数据文本）
            if (robotDataInput != null)
            {
                robotDataInput.text = @"-0.73 -195.12 694.16 -0.0048 -2.2207 2.2175
-484.30 -24.23 379.82 -0.2826 0.3906 -1.7357
172.31 476.92 309.08 -1.2908 0.0939 -0.1228
492.97 -139.06 383.11 -1.0147 1.0941 -1.3658
-77.73 -388.16 373.88 0.0100 0.4014 -3.0293
369.80 -122.78 558.52 -0.7425 0.7920 -1.4699
68.39 357.99 582.73 -0.9592 -0.0417 0.1033
441.07 -80.73 494.38 0.5365 0.3739 1.0960
81.26 107.16 388.05 -0.0061 0.0363 1.4637
43.60 111.29 564.01 0.0309 0.0666 1.4648
-129.59 455.10 297.79 -2.7623 -0.7320 0.0640
428.44 206.59 353.13 -2.4653 1.1676 -0.3048
313.34 -413.11 297.24 -1.1224 2.4419 -0.9746
-416.49 -209.73 399.00 -0.7172 -1.5094 1.8190
-182.40 399.66 318.75 -2.5694 -0.9180 0.1611
362.56 225.02 403.58 -1.6788 0.6756 -0.5618
151.06 -103.27 492.29 -0.8552 0.8272 -1.3422
363.91 159.28 421.49 -0.6647 0.2848 -0.8368
335.70 266.32 447.32 -1.0819 0.4038 -0.5925
100.36 -87.29 656.74 -0.0420 0.0682 -0.2428
398.26 291.95 280.56 1.1359 -0.3419 2.3478
-254.34 -315.34 509.33 -1.0886 -0.9040 -0.0821
-163.79 239.60 382.04 -0.6424 -0.5328 0.9553";
            }

            if (cameraDataInput != null)
            {
                cameraDataInput.text = @"341.73 -361.63 -875.93 0.7025 -0.0044 -0.7116 -0.0018
152.22 -662.37 -388.77 0.8396 0.5328 -0.0950 0.0471
-332.64 -745.92 -1060.41 0.7564 0.0912 0.6386 -0.1080
296.46 -687.91 -1362.44 0.9938 0.0998 -0.0477 -0.0081
531.14 -682.24 -782.28 0.5834 0.4461 -0.5431 0.4071
277.10 -506.26 -1244.69 0.9598 0.2787 -0.0321 -0.0084
-216.17 -469.29 -957.33 0.6459 0.1865 0.6992 -0.2433
236.46 -569.48 -1313.03 -0.1012 -0.1938 -0.4385 0.8717
36.46 -667.04 -952.59 -0.0484 -0.0379 -0.6896 0.7215
31.78 -489.72 -916.76 -0.0443 -0.0381 -0.6738 0.7366
-315.78 -752.21 -757.01 0.4224 -0.3134 0.6833 0.5063
-54.49 -712.38 -1307.68 0.8051 -0.5038 0.2638 0.1686
566.80 -777.94 -1170.02 0.8546 -0.3735 -0.3283 -0.1498
343.18 -646.25 -456.48 -0.3282 -0.0329 0.9397 -0.0902
-263.82 -732.94 -697.84 0.3744 -0.2412 0.7586 0.4756
-74.82 -657.99 -1245.12 0.9192 -0.1065 0.3782 0.0255
249.74 -565.00 -1025.99 0.9745 0.2224 0.0190 -0.0217
-10.35 -642.01 -1240.86 0.8580 0.3853 0.3052 -0.1492
-116.93 -612.36 -1220.24 0.8869 0.1768 0.4109 -0.1155
232.48 -398.99 -972.75 0.5882 0.5263 0.4210 -0.4469
-139.00 -780.51 -1279.12 0.5830 0.0138 -0.4092 0.7017
450.04 -543.26 -608.78 0.4529 0.3460 0.8165 0.0918
-103.84 -667.32 -710.59 0.2133 0.1503 0.8944 -0.3632";
            }

            UpdateDataCountDisplay();
        }

        /// <summary>
        /// 执行标定
        /// </summary>
        private void OnCalibrate()
        {
            if (calibrationManager == null) return;

            // 如果输入框中有新数据，先添加到管理器
            if (!string.IsNullOrWhiteSpace(robotDataInput.text) && 
                !string.IsNullOrWhiteSpace(cameraDataInput.text))
            {
                calibrationManager.ClearCalibrationData();
                calibrationManager.AddCalibrationDataFromText(
                    robotDataInput.text,
                    cameraDataInput.text);
            }

            // 执行标定
            var result = calibrationManager.PerformCalibration();

            // 显示结果
            if (resultText != null)
            {
                resultText.text = result.ResultText;
            }

            UpdateDataCountDisplay();
        }

        /// <summary>
        /// 清空数据
        /// </summary>
        private void OnClearData()
        {
            if (calibrationManager == null) return;

            calibrationManager.ClearCalibrationData();
            
            if (robotDataInput != null)
                robotDataInput.text = "";
            
            if (cameraDataInput != null)
                cameraDataInput.text = "";
            
            if (resultText != null)
                resultText.text = "数据已清空";

            if (saveResultButton != null)
                saveResultButton.interactable = false;

            if (testTransformButton != null)
                testTransformButton.interactable = false;

            UpdateDataCountDisplay();
        }

        /// <summary>
        /// 保存结果
        /// </summary>
        private void OnSaveResult()
        {
            // Unity WebGL/移动平台可能需要特殊处理
            // 这里简单复制到剪贴板
            if (resultText != null && !string.IsNullOrEmpty(resultText.text))
            {
                GUIUtility.systemCopyBuffer = resultText.text;
                if (statusText != null)
                {
                    statusText.text = "[" + DateTime.Now.ToString("HH:mm:ss") + "] 结果已复制到剪贴板";
                }
                Debug.Log("[UI] 标定结果已复制到剪贴板");
            }
        }

        /// <summary>
        /// 打开变换测试面板
        /// </summary>
        private void OnTestTransform()
        {
            if (transformTestPanel != null)
            {
                transformTestPanel.SetActive(!transformTestPanel.activeSelf);
            }
        }

        /// <summary>
        /// 执行坐标变换
        /// </summary>
        private void OnExecuteTransform()
        {
            if (calibrationManager == null || testOutputText == null) return;

            try
            {
                float x = string.IsNullOrEmpty(testInputX.text) ? 0 : float.Parse(testInputX.text);
                float y = string.IsNullOrEmpty(testInputY.text) ? 0 : float.Parse(testInputY.text);
                float z = string.IsNullOrEmpty(testInputZ.text) ? 0 : float.Parse(testInputZ.text);

                Vector3 inputPoint = new Vector3(x, y, z);
                Vector3 outputPoint = calibrationManager.TransformPointCameraToBase(inputPoint);

                testOutputText.text = $"输入 (相机坐标): ({x:F3}, {y:F3}, {z:F3})\n" +
                                     $"输出 (基座坐标): ({outputPoint.x:F3}, {outputPoint.y:F3}, {outputPoint.z:F3})";
            }
            catch (Exception ex)
            {
                testOutputText.text = $"错误: {ex.Message}";
            }
        }

        #endregion

        #region 事件回调

        /// <summary>
        /// 标定完成回调
        /// </summary>
        private void OnCalibrationCompleted(CalibrationResult result)
        {
            if (saveResultButton != null)
                saveResultButton.interactable = result.IsSuccess;

            if (testTransformButton != null)
                testTransformButton.interactable = result.IsSuccess;
        }

        /// <summary>
        /// 状态更新回调
        /// </summary>
        private void OnStatusUpdated(string status)
        {
            if (statusText != null)
            {
                statusText.text = status;
            }
        }

        #endregion

        #region UI更新

        /// <summary>
        /// 更新数据计数显示
        /// </summary>
        private void UpdateDataCountDisplay()
        {
            if (dataCountText != null && calibrationManager != null)
            {
                int count = calibrationManager.GetDataCount();
                dataCountText.text = $"当前数据组数: {count}";
            }
        }

        #endregion

        #region 编辑器辅助

#if UNITY_EDITOR
        /// <summary>
        /// 编辑器中自动连接组件
        /// </summary>
        private void OnValidate()
        {
            if (calibrationManager == null)
            {
                calibrationManager = GetComponent<HandEyeCalibrationManager>();
            }
        }
#endif

        #endregion
    }
}

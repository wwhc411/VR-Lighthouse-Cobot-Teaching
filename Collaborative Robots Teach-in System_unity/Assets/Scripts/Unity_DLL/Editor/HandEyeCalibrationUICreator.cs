using UnityEngine;
using UnityEngine.UI;
using UnityEditor;
using TMPro;

namespace HandEyeCalibration.Editor
{
    /// <summary>
    /// 手眼标定UI自动创建工具
    /// 在Unity编辑器中通过菜单 Tools → Hand-Eye Calibration (DLL) → Create Complete UI 调用
    /// </summary>
    public class HandEyeCalibrationUICreator : EditorWindow
    {
        [MenuItem("Tools/Hand-Eye Calibration (DLL)/Create Complete UI")]
        public static void CreateCompleteUI()
        {
            // 创建Canvas
            GameObject canvasObj = CreateCanvas();

            // 创建主面板
            GameObject mainPanel = CreateMainPanel(canvasObj.transform);

            // 创建标题
            CreateTitle(mainPanel.transform);

            // 创建输入区域
            GameObject inputPanel = CreateInputPanel(mainPanel.transform);

            // 创建按钮区域
            GameObject buttonPanel = CreateButtonPanel(mainPanel.transform);

            // 创建结果显示区域
            GameObject resultPanel = CreateResultPanel(mainPanel.transform);

            // 创建状态栏
            GameObject statusPanel = CreateStatusPanel(mainPanel.transform);

            // 创建变换测试面板
            GameObject transformTestPanel = CreateTransformTestPanel(mainPanel.transform);

            // 创建管理器GameObject
            GameObject managerObj = new GameObject("HandEyeCalibrationManager");
            var manager = managerObj.AddComponent<HandEyeCalibrationManager>();
            var uiController = managerObj.AddComponent<HandEyeCalibrationUI>();

            // 自动连接所有UI组件引用
            ConnectUIReferences(uiController, inputPanel, buttonPanel, resultPanel, statusPanel, transformTestPanel);

            // 选中管理器对象
            Selection.activeGameObject = managerObj;

            Debug.Log("[UI创建器] 手眼标定UI创建完成！请检查Inspector中的组件引用是否正确。");
            EditorUtility.DisplayDialog("创建成功", 
                "手眼标定UI已创建完成！\n\n" +
                "- Canvas和UI面板已就绪\n" +
                "- HandEyeCalibrationManager已添加\n" +
                "- 所有组件引用已自动连接\n\n" +
                "请确保:\n" +
                "1. 已安装TextMeshPro包\n" +
                "2. 将myDll.dll放入Assets/Plugins文件夹", 
                "OK");
        }

        [MenuItem("Tools/Hand-Eye Calibration (DLL)/Create Manager Only")]
        public static void CreateManagerOnly()
        {
            GameObject managerObj = new GameObject("HandEyeCalibrationManager");
            managerObj.AddComponent<HandEyeCalibrationManager>();
            Selection.activeGameObject = managerObj;
            Debug.Log("[UI创建器] 手眼标定管理器已创建（仅脚本，无UI）");
        }

        #region UI创建方法

        private static GameObject CreateCanvas()
        {
            GameObject canvasObj = new GameObject("HandEyeCalibrationCanvas");
            Canvas canvas = canvasObj.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            canvasObj.AddComponent<CanvasScaler>();
            canvasObj.AddComponent<GraphicRaycaster>();

            CanvasScaler scaler = canvasObj.GetComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920, 1080);

            return canvasObj;
        }

        private static GameObject CreateMainPanel(Transform parent)
        {
            GameObject panel = new GameObject("MainPanel");
            panel.transform.SetParent(parent, false);

            RectTransform rect = panel.AddComponent<RectTransform>();
            rect.anchorMin = new Vector2(0, 0);
            rect.anchorMax = new Vector2(1, 1);
            rect.offsetMin = new Vector2(50, 50);
            rect.offsetMax = new Vector2(-50, -50);

            Image image = panel.AddComponent<Image>();
            image.color = new Color(0.2f, 0.2f, 0.2f, 0.95f);

            return panel;
        }

        private static void CreateTitle(Transform parent)
        {
            GameObject titleObj = new GameObject("Title");
            titleObj.transform.SetParent(parent, false);

            RectTransform rect = titleObj.AddComponent<RectTransform>();
            rect.anchorMin = new Vector2(0, 1);
            rect.anchorMax = new Vector2(1, 1);
            rect.pivot = new Vector2(0.5f, 1);
            rect.anchoredPosition = new Vector2(0, -20);
            rect.sizeDelta = new Vector2(-40, 60);

            TextMeshProUGUI text = titleObj.AddComponent<TextMeshProUGUI>();
            text.text = "手眼标定系统 (DLL版本)";
            text.fontSize = 36;
            text.alignment = TextAlignmentOptions.Center;
            text.color = Color.white;
            text.fontStyle = FontStyles.Bold;
        }

        private static GameObject CreateInputPanel(Transform parent)
        {
            GameObject inputPanel = new GameObject("InputPanel");
            inputPanel.transform.SetParent(parent, false);

            RectTransform rect = inputPanel.AddComponent<RectTransform>();
            rect.anchorMin = new Vector2(0, 0.5f);
            rect.anchorMax = new Vector2(1, 1);
            rect.offsetMin = new Vector2(20, 20);
            rect.offsetMax = new Vector2(-20, -100);

            // 左侧：机器人数据
            GameObject robotPanel = CreateInputField(inputPanel.transform, "RobotDataPanel", 
                "机器人数据 (X Y Z RX RY RZ)", new Vector2(0, 0), new Vector2(0.48f, 1));

            // 右侧：相机数据
            GameObject cameraPanel = CreateInputField(inputPanel.transform, "CameraDataPanel",
                "相机数据 (X Y Z QW QX QY QZ)", new Vector2(0.52f, 0), new Vector2(1, 1));

            return inputPanel;
        }

        private static GameObject CreateInputField(Transform parent, string name, string label, 
            Vector2 anchorMin, Vector2 anchorMax)
        {
            GameObject panel = new GameObject(name);
            panel.transform.SetParent(parent, false);

            RectTransform rect = panel.AddComponent<RectTransform>();
            rect.anchorMin = anchorMin;
            rect.anchorMax = anchorMax;
            rect.offsetMin = Vector2.zero;
            rect.offsetMax = Vector2.zero;

            // 标签
            GameObject labelObj = new GameObject("Label");
            labelObj.transform.SetParent(panel.transform, false);
            RectTransform labelRect = labelObj.AddComponent<RectTransform>();
            labelRect.anchorMin = new Vector2(0, 1);
            labelRect.anchorMax = new Vector2(1, 1);
            labelRect.pivot = new Vector2(0.5f, 1);
            labelRect.anchoredPosition = Vector2.zero;
            labelRect.sizeDelta = new Vector2(0, 30);

            TextMeshProUGUI labelText = labelObj.AddComponent<TextMeshProUGUI>();
            labelText.text = label;
            labelText.fontSize = 18;
            labelText.alignment = TextAlignmentOptions.Left;
            labelText.color = Color.white;

            // 输入框
            GameObject inputObj = new GameObject("InputField");
            inputObj.transform.SetParent(panel.transform, false);
            RectTransform inputRect = inputObj.AddComponent<RectTransform>();
            inputRect.anchorMin = new Vector2(0, 0);
            inputRect.anchorMax = new Vector2(1, 1);
            inputRect.offsetMin = new Vector2(0, 0);
            inputRect.offsetMax = new Vector2(0, -35);

            Image inputBg = inputObj.AddComponent<Image>();
            inputBg.color = new Color(0.15f, 0.15f, 0.15f, 1f);

            TMP_InputField inputField = inputObj.AddComponent<TMP_InputField>();
            inputField.lineType = TMP_InputField.LineType.MultiLineNewline;

            // 输入框文本
            GameObject textArea = new GameObject("TextArea");
            textArea.transform.SetParent(inputObj.transform, false);
            RectTransform textAreaRect = textArea.AddComponent<RectTransform>();
            textAreaRect.anchorMin = Vector2.zero;
            textAreaRect.anchorMax = Vector2.one;
            textAreaRect.offsetMin = new Vector2(5, 5);
            textAreaRect.offsetMax = new Vector2(-5, -5);

            GameObject textObj = new GameObject("Text");
            textObj.transform.SetParent(textArea.transform, false);
            RectTransform textRect = textObj.AddComponent<RectTransform>();
            textRect.anchorMin = Vector2.zero;
            textRect.anchorMax = Vector2.one;
            textRect.offsetMin = Vector2.zero;
            textRect.offsetMax = Vector2.zero;

            TextMeshProUGUI inputText = textObj.AddComponent<TextMeshProUGUI>();
            inputText.fontSize = 14;
            inputText.color = Color.white;

            inputField.textViewport = textAreaRect;
            inputField.textComponent = inputText;

            return inputObj;
        }

        private static GameObject CreateButtonPanel(Transform parent)
        {
            GameObject buttonPanel = new GameObject("ButtonPanel");
            buttonPanel.transform.SetParent(parent, false);

            RectTransform rect = buttonPanel.AddComponent<RectTransform>();
            rect.anchorMin = new Vector2(0, 0.45f);
            rect.anchorMax = new Vector2(1, 0.5f);
            rect.offsetMin = new Vector2(20, 0);
            rect.offsetMax = new Vector2(-20, 0);

            // 创建按钮
            CreateButton(buttonPanel.transform, "LoadSampleButton", "加载示例数据", 0);
            CreateButton(buttonPanel.transform, "CalibrateButton", "执行标定", 1);
            CreateButton(buttonPanel.transform, "ClearDataButton", "清空数据", 2);
            CreateButton(buttonPanel.transform, "SaveResultButton", "保存结果", 3);
            CreateButton(buttonPanel.transform, "TestTransformButton", "测试变换", 4);

            return buttonPanel;
        }

        private static GameObject CreateButton(Transform parent, string name, string text, int index)
        {
            GameObject buttonObj = new GameObject(name);
            buttonObj.transform.SetParent(parent, false);

            RectTransform rect = buttonObj.AddComponent<RectTransform>();
            float buttonWidth = 180;
            float buttonHeight = 50;
            float spacing = 20;
            float startX = -(buttonWidth * 2.5f + spacing * 2);

            rect.anchorMin = new Vector2(0.5f, 0.5f);
            rect.anchorMax = new Vector2(0.5f, 0.5f);
            rect.pivot = new Vector2(0.5f, 0.5f);
            rect.anchoredPosition = new Vector2(startX + index * (buttonWidth + spacing), 0);
            rect.sizeDelta = new Vector2(buttonWidth, buttonHeight);

            Image image = buttonObj.AddComponent<Image>();
            image.color = new Color(0.3f, 0.5f, 0.8f, 1f);

            Button button = buttonObj.AddComponent<Button>();
            button.targetGraphic = image;

            // 按钮文本
            GameObject textObj = new GameObject("Text");
            textObj.transform.SetParent(buttonObj.transform, false);
            RectTransform textRect = textObj.AddComponent<RectTransform>();
            textRect.anchorMin = Vector2.zero;
            textRect.anchorMax = Vector2.one;
            textRect.offsetMin = Vector2.zero;
            textRect.offsetMax = Vector2.zero;

            TextMeshProUGUI buttonText = textObj.AddComponent<TextMeshProUGUI>();
            buttonText.text = text;
            buttonText.fontSize = 18;
            buttonText.alignment = TextAlignmentOptions.Center;
            buttonText.color = Color.white;

            return buttonObj;
        }

        private static GameObject CreateResultPanel(Transform parent)
        {
            GameObject resultPanel = new GameObject("ResultPanel");
            resultPanel.transform.SetParent(parent, false);

            RectTransform rect = resultPanel.AddComponent<RectTransform>();
            rect.anchorMin = new Vector2(0, 0.1f);
            rect.anchorMax = new Vector2(1, 0.43f);
            rect.offsetMin = new Vector2(20, 0);
            rect.offsetMax = new Vector2(-20, -10);

            // 标签
            GameObject labelObj = new GameObject("Label");
            labelObj.transform.SetParent(resultPanel.transform, false);
            RectTransform labelRect = labelObj.AddComponent<RectTransform>();
            labelRect.anchorMin = new Vector2(0, 1);
            labelRect.anchorMax = new Vector2(1, 1);
            labelRect.pivot = new Vector2(0.5f, 1);
            labelRect.anchoredPosition = Vector2.zero;
            labelRect.sizeDelta = new Vector2(0, 30);

            TextMeshProUGUI labelText = labelObj.AddComponent<TextMeshProUGUI>();
            labelText.text = "标定结果";
            labelText.fontSize = 20;
            labelText.alignment = TextAlignmentOptions.Left;
            labelText.color = Color.white;
            labelText.fontStyle = FontStyles.Bold;

            // 结果文本区域
            GameObject textAreaObj = new GameObject("ResultText");
            textAreaObj.transform.SetParent(resultPanel.transform, false);
            RectTransform textAreaRect = textAreaObj.AddComponent<RectTransform>();
            textAreaRect.anchorMin = new Vector2(0, 0);
            textAreaRect.anchorMax = new Vector2(1, 1);
            textAreaRect.offsetMin = new Vector2(0, 0);
            textAreaRect.offsetMax = new Vector2(0, -35);

            Image textBg = textAreaObj.AddComponent<Image>();
            textBg.color = new Color(0.1f, 0.1f, 0.1f, 1f);

            TextMeshProUGUI resultText = textAreaObj.AddComponent<TextMeshProUGUI>();
            resultText.fontSize = 14;
            resultText.alignment = TextAlignmentOptions.TopLeft;
            resultText.color = Color.white;
            resultText.margin = new Vector4(10, 10, 10, 10);
            resultText.text = "就绪，请输入标定数据或加载示例数据";

            return resultPanel;
        }

        private static GameObject CreateStatusPanel(Transform parent)
        {
            GameObject statusPanel = new GameObject("StatusPanel");
            statusPanel.transform.SetParent(parent, false);

            RectTransform rect = statusPanel.AddComponent<RectTransform>();
            rect.anchorMin = new Vector2(0, 0);
            rect.anchorMax = new Vector2(1, 0.08f);
            rect.offsetMin = new Vector2(20, 10);
            rect.offsetMax = new Vector2(-20, -10);

            Image bg = statusPanel.AddComponent<Image>();
            bg.color = new Color(0.15f, 0.15f, 0.15f, 1f);

            // 状态文本
            GameObject statusTextObj = new GameObject("StatusText");
            statusTextObj.transform.SetParent(statusPanel.transform, false);
            RectTransform statusRect = statusTextObj.AddComponent<RectTransform>();
            statusRect.anchorMin = new Vector2(0, 0);
            statusRect.anchorMax = new Vector2(0.7f, 1);
            statusRect.offsetMin = new Vector2(10, 0);
            statusRect.offsetMax = new Vector2(-10, 0);

            TextMeshProUGUI statusText = statusTextObj.AddComponent<TextMeshProUGUI>();
            statusText.text = "[系统就绪]";
            statusText.fontSize = 16;
            statusText.alignment = TextAlignmentOptions.Left;
            statusText.color = Color.green;

            // 数据计数文本
            GameObject dataCountObj = new GameObject("DataCountText");
            dataCountObj.transform.SetParent(statusPanel.transform, false);
            RectTransform dataCountRect = dataCountObj.AddComponent<RectTransform>();
            dataCountRect.anchorMin = new Vector2(0.7f, 0);
            dataCountRect.anchorMax = new Vector2(1, 1);
            dataCountRect.offsetMin = new Vector2(10, 0);
            dataCountRect.offsetMax = new Vector2(-10, 0);

            TextMeshProUGUI dataCountText = dataCountObj.AddComponent<TextMeshProUGUI>();
            dataCountText.text = "当前数据组数: 0";
            dataCountText.fontSize = 16;
            dataCountText.alignment = TextAlignmentOptions.Right;
            dataCountText.color = Color.yellow;

            return statusPanel;
        }

        private static GameObject CreateTransformTestPanel(Transform parent)
        {
            GameObject testPanel = new GameObject("TransformTestPanel");
            testPanel.transform.SetParent(parent, false);

            RectTransform rect = testPanel.AddComponent<RectTransform>();
            rect.anchorMin = new Vector2(0.6f, 0.25f);
            rect.anchorMax = new Vector2(0.95f, 0.42f);
            rect.offsetMin = Vector2.zero;
            rect.offsetMax = Vector2.zero;

            Image bg = testPanel.AddComponent<Image>();
            bg.color = new Color(0.25f, 0.35f, 0.45f, 0.95f);

            // 默认隐藏
            testPanel.SetActive(false);

            // 标题
            GameObject titleObj = new GameObject("Title");
            titleObj.transform.SetParent(testPanel.transform, false);
            RectTransform titleRect = titleObj.AddComponent<RectTransform>();
            titleRect.anchorMin = new Vector2(0, 1);
            titleRect.anchorMax = new Vector2(1, 1);
            titleRect.pivot = new Vector2(0.5f, 1);
            titleRect.anchoredPosition = new Vector2(0, -5);
            titleRect.sizeDelta = new Vector2(0, 30);

            TextMeshProUGUI titleText = titleObj.AddComponent<TextMeshProUGUI>();
            titleText.text = "坐标变换测试";
            titleText.fontSize = 20;
            titleText.alignment = TextAlignmentOptions.Center;
            titleText.color = Color.white;
            titleText.fontStyle = FontStyles.Bold;

            // 输入字段
            CreateSmallInputField(testPanel.transform, "TestInputX", "X:", 0);
            CreateSmallInputField(testPanel.transform, "TestInputY", "Y:", 1);
            CreateSmallInputField(testPanel.transform, "TestInputZ", "Z:", 2);

            // 执行按钮
            GameObject executeBtn = new GameObject("ExecuteTransformButton");
            executeBtn.transform.SetParent(testPanel.transform, false);
            RectTransform executeBtnRect = executeBtn.AddComponent<RectTransform>();
            executeBtnRect.anchorMin = new Vector2(0.5f, 0);
            executeBtnRect.anchorMax = new Vector2(0.5f, 0);
            executeBtnRect.pivot = new Vector2(0.5f, 0);
            executeBtnRect.anchoredPosition = new Vector2(0, 60);
            executeBtnRect.sizeDelta = new Vector2(150, 40);

            Image btnImage = executeBtn.AddComponent<Image>();
            btnImage.color = new Color(0.2f, 0.7f, 0.3f, 1f);

            Button executeButton = executeBtn.AddComponent<Button>();

            GameObject btnTextObj = new GameObject("Text");
            btnTextObj.transform.SetParent(executeBtn.transform, false);
            RectTransform btnTextRect = btnTextObj.AddComponent<RectTransform>();
            btnTextRect.anchorMin = Vector2.zero;
            btnTextRect.anchorMax = Vector2.one;
            btnTextRect.offsetMin = Vector2.zero;
            btnTextRect.offsetMax = Vector2.zero;

            TextMeshProUGUI btnText = btnTextObj.AddComponent<TextMeshProUGUI>();
            btnText.text = "执行变换";
            btnText.fontSize = 16;
            btnText.alignment = TextAlignmentOptions.Center;
            btnText.color = Color.white;

            // 输出文本
            GameObject outputObj = new GameObject("TestOutputText");
            outputObj.transform.SetParent(testPanel.transform, false);
            RectTransform outputRect = outputObj.AddComponent<RectTransform>();
            outputRect.anchorMin = new Vector2(0.05f, 0);
            outputRect.anchorMax = new Vector2(0.95f, 0);
            outputRect.pivot = new Vector2(0.5f, 0);
            outputRect.anchoredPosition = new Vector2(0, 5);
            outputRect.sizeDelta = new Vector2(0, 50);

            TextMeshProUGUI outputText = outputObj.AddComponent<TextMeshProUGUI>();
            outputText.text = "输入坐标后点击执行";
            outputText.fontSize = 12;
            outputText.alignment = TextAlignmentOptions.TopLeft;
            outputText.color = Color.cyan;

            return testPanel;
        }

        private static void CreateSmallInputField(Transform parent, string name, string label, int index)
        {
            GameObject fieldObj = new GameObject(name);
            fieldObj.transform.SetParent(parent, false);

            RectTransform rect = fieldObj.AddComponent<RectTransform>();
            rect.anchorMin = new Vector2(0.05f, 1);
            rect.anchorMax = new Vector2(0.95f, 1);
            rect.pivot = new Vector2(0.5f, 1);
            rect.anchoredPosition = new Vector2(0, -40 - index * 35);
            rect.sizeDelta = new Vector2(0, 30);

            // 标签
            GameObject labelObj = new GameObject("Label");
            labelObj.transform.SetParent(fieldObj.transform, false);
            RectTransform labelRect = labelObj.AddComponent<RectTransform>();
            labelRect.anchorMin = new Vector2(0, 0);
            labelRect.anchorMax = new Vector2(0.2f, 1);
            labelRect.offsetMin = Vector2.zero;
            labelRect.offsetMax = Vector2.zero;

            TextMeshProUGUI labelText = labelObj.AddComponent<TextMeshProUGUI>();
            labelText.text = label;
            labelText.fontSize = 16;
            labelText.alignment = TextAlignmentOptions.Left;
            labelText.color = Color.white;

            // 输入框
            GameObject inputObj = new GameObject("Input");
            inputObj.transform.SetParent(fieldObj.transform, false);
            RectTransform inputRect = inputObj.AddComponent<RectTransform>();
            inputRect.anchorMin = new Vector2(0.22f, 0);
            inputRect.anchorMax = new Vector2(1, 1);
            inputRect.offsetMin = Vector2.zero;
            inputRect.offsetMax = Vector2.zero;

            Image inputBg = inputObj.AddComponent<Image>();
            inputBg.color = new Color(0.2f, 0.2f, 0.2f, 1f);

            TMP_InputField inputField = inputObj.AddComponent<TMP_InputField>();
            inputField.text = "0";

            GameObject textObj = new GameObject("Text");
            textObj.transform.SetParent(inputObj.transform, false);
            RectTransform textRect = textObj.AddComponent<RectTransform>();
            textRect.anchorMin = Vector2.zero;
            textRect.anchorMax = Vector2.one;
            textRect.offsetMin = new Vector2(5, 0);
            textRect.offsetMax = new Vector2(-5, 0);

            TextMeshProUGUI inputText = textObj.AddComponent<TextMeshProUGUI>();
            inputText.fontSize = 14;
            inputText.color = Color.white;
            inputText.alignment = TextAlignmentOptions.Left;

            inputField.textComponent = inputText;
        }

        private static void ConnectUIReferences(HandEyeCalibrationUI uiController, 
            GameObject inputPanel, GameObject buttonPanel, GameObject resultPanel, 
            GameObject statusPanel, GameObject transformTestPanel)
        {
            // 使用反射自动连接所有引用
            var so = new SerializedObject(uiController);

            // 输入字段
            SetSerializedReference(so, "robotDataInput", 
                inputPanel.transform.Find("RobotDataPanel").GetComponent<TMP_InputField>());
            SetSerializedReference(so, "cameraDataInput",
                inputPanel.transform.Find("CameraDataPanel").GetComponent<TMP_InputField>());

            // 按钮
            SetSerializedReference(so, "loadSampleButton",
                buttonPanel.transform.Find("LoadSampleButton").GetComponent<Button>());
            SetSerializedReference(so, "calibrateButton",
                buttonPanel.transform.Find("CalibrateButton").GetComponent<Button>());
            SetSerializedReference(so, "clearDataButton",
                buttonPanel.transform.Find("ClearDataButton").GetComponent<Button>());
            SetSerializedReference(so, "saveResultButton",
                buttonPanel.transform.Find("SaveResultButton").GetComponent<Button>());
            SetSerializedReference(so, "testTransformButton",
                buttonPanel.transform.Find("TestTransformButton").GetComponent<Button>());

            // 结果显示
            SetSerializedReference(so, "resultText",
                resultPanel.transform.Find("ResultText").GetComponent<TextMeshProUGUI>());
            SetSerializedReference(so, "statusText",
                statusPanel.transform.Find("StatusText").GetComponent<TextMeshProUGUI>());
            SetSerializedReference(so, "dataCountText",
                statusPanel.transform.Find("DataCountText").GetComponent<TextMeshProUGUI>());

            // 变换测试面板
            SetSerializedReference(so, "transformTestPanel", transformTestPanel);
            SetSerializedReference(so, "testInputX",
                transformTestPanel.transform.Find("TestInputX/Input").GetComponent<TMP_InputField>());
            SetSerializedReference(so, "testInputY",
                transformTestPanel.transform.Find("TestInputY/Input").GetComponent<TMP_InputField>());
            SetSerializedReference(so, "testInputZ",
                transformTestPanel.transform.Find("TestInputZ/Input").GetComponent<TMP_InputField>());
            SetSerializedReference(so, "testOutputText",
                transformTestPanel.transform.Find("TestOutputText").GetComponent<TextMeshProUGUI>());
            SetSerializedReference(so, "executeTransformButton",
                transformTestPanel.transform.Find("ExecuteTransformButton").GetComponent<Button>());

            so.ApplyModifiedProperties();
            Debug.Log("[UI创建器] 所有UI组件引用已自动连接");
        }

        private static void SetSerializedReference(SerializedObject so, string propertyName, Object value)
        {
            SerializedProperty prop = so.FindProperty(propertyName);
            if (prop != null)
            {
                prop.objectReferenceValue = value;
            }
            else
            {
                Debug.LogWarning($"[UI创建器] 未找到属性: {propertyName}");
            }
        }

        #endregion
    }
}

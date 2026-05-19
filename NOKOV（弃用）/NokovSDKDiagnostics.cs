using UnityEngine;
using System;

/// <summary>
/// Nokov SDK诊断工具
/// 用于检查SDK集成状态和连接问题
/// </summary>
public class NokovSDKDiagnostics : MonoBehaviour
{
    [Header("诊断配置")]
    [Tooltip("自动运行诊断")]
    public bool runOnStart = true;

    void Start()
    {
        if (runOnStart)
        {
            RunDiagnostics();
        }
    }

    [ContextMenu("运行完整诊断")]
    public void RunDiagnostics()
    {
        Debug.Log("╔═══════════════════════════════════════════════════════════╗");
        Debug.Log("║          Nokov SDK 集成诊断工具                          ║");
        Debug.Log("╚═══════════════════════════════════════════════════════════╝");

        // 1. 检查NokovSDKManager
        CheckSDKManager();

        // 2. 检查NokovDataLogger
        CheckDataLogger();

        // 3. 检查DLL文件
        CheckDLLFiles();

        // 4. 检查命名空间
        CheckNamespaces();

        // 5. 测试连接
        TestConnection();

        Debug.Log("╔═══════════════════════════════════════════════════════════╗");
        Debug.Log("║          诊断完成                                         ║");
        Debug.Log("╚═══════════════════════════════════════════════════════════╝\n");
    }

    private void CheckSDKManager()
    {
        Debug.Log("\n【1/5】检查 NokovSDKManager...");

        if (NokovSDKManager.Instance == null)
        {
            Debug.LogError("  ✗ NokovSDKManager.Instance 为 null!");
            Debug.LogError("  → 解决方案: 在场景中创建GameObject并添加NokovSDKManager组件");
            return;
        }

        Debug.Log("  ✓ NokovSDKManager 存在");
        Debug.Log($"  - 服务器IP: {NokovSDKManager.Instance.serverIP}");
        Debug.Log($"  - 自动连接: {NokovSDKManager.Instance.autoConnect}");
        Debug.Log($"  - 接收模式: {NokovSDKManager.Instance.receiveMode}");
        Debug.Log($"  - 连接状态: {(NokovSDKManager.Instance.IsConnected ? "已连接" : "未连接")}");
        Debug.Log($"  - 调试日志: {NokovSDKManager.Instance.enableDebugLog}");
    }

    private void CheckDataLogger()
    {
        Debug.Log("\n【2/5】检查 NokovDataLogger...");

        NokovDataLogger logger = FindObjectOfType<NokovDataLogger>();
        if (logger == null)
        {
            Debug.LogWarning("  ✗ 场景中未找到 NokovDataLogger 组件");
            Debug.LogWarning("  → 解决方案: 在GameObject上添加NokovDataLogger组件");
            return;
        }

        Debug.Log("  ✓ NokovDataLogger 存在");
        Debug.Log($"  - 启用日志: {logger.enableLogging}");
        Debug.Log($"  - 日志模式: {logger.logMode}");
        Debug.Log($"  - 日志间隔: {logger.logInterval}s");
        Debug.Log($"  - 位置单位: {logger.positionUnit}");
        Debug.Log($"  - 旋转格式: {logger.rotationFormat}");

        if (!logger.enableLogging)
        {
            Debug.LogError("  ✗ enableLogging 为 false, 日志已禁用!");
            Debug.LogError("  → 解决方案: 在Inspector中启用 'Enable Logging'");
        }
    }

    private void CheckDLLFiles()
    {
        Debug.Log("\n【3/5】检查 DLL 文件...");

        string[] requiredDLLs = new string[]
        {
            "Assets/Plugins/x64/CSNokovSDK.dll",
            "Assets/Plugins/x64/nokov_sdk.dll"
        };

        bool allDLLsFound = true;
        foreach (string dllPath in requiredDLLs)
        {
            if (System.IO.File.Exists(dllPath))
            {
                Debug.Log($"  ✓ 找到: {dllPath}");
            }
            else
            {
                Debug.LogError($"  ✗ 缺失: {dllPath}");
                allDLLsFound = false;
            }
        }

        if (!allDLLsFound)
        {
            Debug.LogError("  → 解决方案:");
            Debug.LogError("     1. 从 XING_C#_SDK_2.4.0.5430/bin/x64/ 复制DLL文件");
            Debug.LogError("     2. 粘贴到 Assets/Plugins/x64/ 目录");
            Debug.LogError("     3. 在Unity中选中DLL, Inspector设置为 Windows x64");
        }
    }

    private void CheckNamespaces()
    {
        Debug.Log("\n【4/5】检查命名空间引用...");

        try
        {
            // 尝试访问CSNokovSDK命名空间
            Type sdkType = Type.GetType("CSNokovSDK.CNokovSDK");
            if (sdkType != null)
            {
                Debug.Log("  ✓ CSNokovSDK 命名空间可访问");
            }
            else
            {
                Debug.LogError("  ✗ 无法找到 CSNokovSDK.CNokovSDK 类型");
                Debug.LogError("  → 可能原因:");
                Debug.LogError("     1. CSNokovSDK.dll 未正确导入");
                Debug.LogError("     2. DLL平台设置错误");
                Debug.LogError("     3. 需要重启Unity编辑器");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"  ✗ 命名空间检查异常: {e.Message}");
        }
    }

    private void TestConnection()
    {
        Debug.Log("\n【5/5】测试连接状态...");

        if (NokovSDKManager.Instance == null)
        {
            Debug.LogError("  ✗ NokovSDKManager不存在,跳过连接测试");
            return;
        }

        if (NokovSDKManager.Instance.IsConnected)
        {
            Debug.Log("  ✓ SDK已连接到服务器");
            Debug.Log($"  - 当前帧率: {NokovSDKManager.Instance.CurrentFPS:F1} FPS");
            
            if (NokovSDKManager.Instance.CurrentFPS < 1)
            {
                Debug.LogWarning("  ⚠ 帧率为0, 可能未接收到数据");
                Debug.LogWarning("  → 检查事项:");
                Debug.LogWarning("     1. Nokov软件是否启用了数据流 (Data Streaming)");
                Debug.LogWarning("     2. 是否有刚体在动捕区域内");
                Debug.LogWarning("     3. 刚体是否已标定");
            }
        }
        else
        {
            Debug.LogWarning("  ✗ SDK未连接");
            Debug.LogWarning("  → 检查事项:");
            Debug.LogWarning($"     1. Nokov服务器是否运行 (IP: {NokovSDKManager.Instance.serverIP})");
            Debug.LogWarning("     2. 网络是否连通 (尝试ping)");
            Debug.LogWarning("     3. 防火墙是否允许连接");
            Debug.LogWarning("     4. autoConnect 是否启用");

            if (Application.isPlaying)
            {
                Debug.Log("  → 尝试手动连接...");
                bool success = NokovSDKManager.Instance.Connect();
                if (success)
                {
                    Debug.Log("  ✓ 手动连接成功!");
                }
                else
                {
                    Debug.LogError("  ✗ 手动连接失败, 请检查上述事项");
                }
            }
        }
    }

    [ContextMenu("测试事件订阅")]
    public void TestEventSubscription()
    {
        Debug.Log("\n【事件订阅测试】");

        if (NokovSDKManager.Instance == null)
        {
            Debug.LogError("NokovSDKManager 不存在");
            return;
        }

        // 测试订阅
        NokovSDKManager.Instance.OnRigidBodyDataReceived += TestRigidBodyHandler;
        NokovSDKManager.Instance.OnFrameDataReceived += TestFrameHandler;
        NokovSDKManager.Instance.OnConnectionStateChanged += TestConnectionHandler;

        Debug.Log("✓ 已订阅测试事件 (5秒后自动取消订阅)");
        Invoke("UnsubscribeTestEvents", 5f);
    }

    private void TestRigidBodyHandler(CSNokovSDK.sRigidBodyData[] rigids)
    {
        Debug.Log($"<color=green>【事件测试】接收到 {rigids.Length} 个刚体数据</color>");
        if (rigids.Length > 0)
        {
            var first = rigids[0];
            Debug.Log($"  刚体[{first.Id}]: 位置=({first.X:F2}, {first.Y:F2}, {first.Z:F2})mm");
        }
    }

    private void TestFrameHandler(CSNokovSDK.sFrameOfMocapData frame)
    {
        Debug.Log($"<color=cyan>【事件测试】接收到帧数据: 帧号={frame.FrameNumber}, 刚体数={frame.RigidBodyCount}</color>");
    }

    private void TestConnectionHandler(bool connected)
    {
        Debug.Log($"<color=yellow>【事件测试】连接状态变化: {connected}</color>");
    }

    private void UnsubscribeTestEvents()
    {
        if (NokovSDKManager.Instance != null)
        {
            NokovSDKManager.Instance.OnRigidBodyDataReceived -= TestRigidBodyHandler;
            NokovSDKManager.Instance.OnFrameDataReceived -= TestFrameHandler;
            NokovSDKManager.Instance.OnConnectionStateChanged -= TestConnectionHandler;
            Debug.Log("✓ 已取消测试事件订阅");
        }
    }

    void OnDestroy()
    {
        UnsubscribeTestEvents();
    }

    [ContextMenu("打印快速排查清单")]
    public void PrintQuickChecklist()
    {
        Debug.Log(@"
╔═══════════════════════════════════════════════════════════╗
║          Nokov SDK 无日志输出 - 快速排查清单            ║
╚═══════════════════════════════════════════════════════════╝

【步骤1】确认组件存在
  □ 场景中有GameObject挂载 NokovSDKManager
  □ 同一个或另一个GameObject挂载 NokovDataLogger
  □ 两个组件都在 Inspector 中可见

【步骤2】检查NokovSDKManager设置
  □ Server IP = 10.1.1.198
  □ Auto Connect = ✓
  □ Enable Debug Log = ✓
  □ Receive Mode = Callback (或 Polling)

【步骤3】检查NokovDataLogger设置
  □ Enable Logging = ✓
  □ Log Mode = RigidBodyBasic (推荐)
  □ Log Interval = 1.0 秒

【步骤4】检查DLL文件
  □ Assets/Plugins/x64/CSNokovSDK.dll 存在
  □ Assets/Plugins/x64/nokov_sdk.dll 存在
  □ DLL Inspector 设置:
     - Platform = Windows
     - CPU = x86_64

【步骤5】检查Nokov软件端
  □ XING/XINGYING 软件正在运行
  □ 启用了数据流 (Data Streaming)
  □ 有刚体在动捕区域内
  □ 刚体已标定并显示正常

【步骤6】检查网络连接
  □ 在Windows命令行执行: ping 10.1.1.198
  □ 防火墙允许Unity访问网络
  □ 电脑与Nokov系统在同一网段

【步骤7】查看Unity Console
  □ 是否有错误信息 (红色)
  □ 是否有 '[Nokov] 成功连接到服务器' 消息
  □ 是否有 'DllNotFoundException' 错误

═══════════════════════════════════════════════════════════

如果以上步骤都正常,但仍无输出:
1. 尝试切换接收模式 (Callback ↔ Polling)
2. 重启Unity编辑器
3. 在代码中添加断点调试
4. 使用本脚本的 Context Menu → '测试事件订阅'
        ");
    }
}

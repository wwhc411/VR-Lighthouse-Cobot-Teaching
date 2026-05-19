using UnityEngine;

/// <summary>
/// 最简单的Nokov连接测试
/// 直接输出关键信息,无需复杂配置
/// </summary>
public class SimpleNokovTest : MonoBehaviour
{
    void Start()
    {
        Debug.Log("╔═══════════════════════════════════════════════════════════╗");
        Debug.Log("║          SimpleNokovTest - 开始测试                       ║");
        Debug.Log("╚═══════════════════════════════════════════════════════════╝");

        // 测试1: 检查NokovSDKManager是否存在
        Debug.Log("\n【测试1】检查NokovSDKManager...");
        if (NokovSDKManager.Instance == null)
        {
            Debug.LogError("✗ NokovSDKManager.Instance 为 null!");
            Debug.LogError("→ 请在场景中创建GameObject并添加NokovSDKManager组件");
            return;
        }
        Debug.Log("✓ NokovSDKManager 存在");

        // 测试2: 检查连接状态
        Debug.Log("\n【测试2】检查连接状态...");
        Debug.Log($"服务器IP: {NokovSDKManager.Instance.serverIP}");
        Debug.Log($"自动连接: {NokovSDKManager.Instance.autoConnect}");
        Debug.Log($"调试日志: {NokovSDKManager.Instance.enableDebugLog}");
        Debug.Log($"接收模式: {NokovSDKManager.Instance.receiveMode}");
        Debug.Log($"连接状态: {(NokovSDKManager.Instance.IsConnected ? "✓ 已连接" : "✗ 未连接")}");

        // 测试3: 订阅事件
        Debug.Log("\n【测试3】订阅数据事件...");
        NokovSDKManager.Instance.OnRigidBodyDataReceived += OnRigidBodyData;
        NokovSDKManager.Instance.OnFrameDataReceived += OnFrameData;
        NokovSDKManager.Instance.OnConnectionStateChanged += OnConnectionChanged;
        Debug.Log("✓ 已订阅所有事件");

        // 测试4: 如果未连接,尝试手动连接
        if (!NokovSDKManager.Instance.IsConnected)
        {
            Debug.Log("\n【测试4】尝试手动连接...");
            bool success = NokovSDKManager.Instance.Connect();
            if (success)
            {
                Debug.Log("<color=green>✓ 连接成功!</color>");
            }
            else
            {
                Debug.LogError("<color=red>✗ 连接失败!</color>");
                Debug.LogError("请检查:");
                Debug.LogError("  1. Nokov软件(XING/XINGYING)是否运行");
                Debug.LogError("  2. IP地址是否正确: 10.1.1.198");
                Debug.LogError("  3. 网络是否连通(在CMD中执行: ping 10.1.1.198)");
                Debug.LogError("  4. 防火墙是否阻止连接");
            }
        }

        Debug.Log("\n═══════════════════════════════════════════════════════════");
        Debug.Log("测试完成! 等待数据接收...");
        Debug.Log("如果10秒内无数据,请检查Nokov软件的Data Streaming设置");
        Debug.Log("═══════════════════════════════════════════════════════════\n");
    }

    private void OnRigidBodyData(CSNokovSDK.sRigidBodyData[] rigids)
    {
        Debug.Log($"<color=green>【数据接收】收到 {rigids.Length} 个刚体数据!</color>");
        
        for (int i = 0; i < rigids.Length; i++)
        {
            var rigid = rigids[i];
            
            // 检查数据有效性
            bool isValid = NokovSDKManager.IsValidPosition(rigid.X, rigid.Y, rigid.Z);
            string validTag = isValid ? "[有效]" : "[无效]";
            
            Debug.Log($"  刚体[{rigid.Id}] {validTag}:");
            Debug.Log($"    位置(mm): X={rigid.X,9:F2}  Y={rigid.Y,9:F2}  Z={rigid.Z,9:F2}");
            Debug.Log($"    四元数:   QX={rigid.QX,7:F4} QY={rigid.QY,7:F4} QZ={rigid.QZ,7:F4} QW={rigid.QW,7:F4}");
            Debug.Log($"    Marker数: {rigid.NMarkers} | 误差: {rigid.MeanError:F4}");
        }
    }

    private void OnFrameData(CSNokovSDK.sFrameOfMocapData frame)
    {
        Debug.Log($"<color=cyan>【帧数据】帧号: {frame.FrameNumber} | 时间戳: {frame.Timestamp} | 刚体数: {frame.RigidBodyCount}</color>");
    }

    private void OnConnectionChanged(bool connected)
    {
        if (connected)
        {
            Debug.Log("<color=green>【连接状态】已连接到Nokov服务器</color>");
        }
        else
        {
            Debug.LogWarning("<color=yellow>【连接状态】与Nokov服务器断开连接</color>");
        }
    }

    void OnDestroy()
    {
        if (NokovSDKManager.Instance != null)
        {
            NokovSDKManager.Instance.OnRigidBodyDataReceived -= OnRigidBodyData;
            NokovSDKManager.Instance.OnFrameDataReceived -= OnFrameData;
            NokovSDKManager.Instance.OnConnectionStateChanged -= OnConnectionChanged;
        }
    }
}

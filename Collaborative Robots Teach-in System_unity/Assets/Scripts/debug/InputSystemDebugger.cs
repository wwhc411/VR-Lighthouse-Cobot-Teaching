using UnityEngine;
using UnityEngine.EventSystems;

/// <summary>
/// 输入系统诊断工具
/// 用于检测 UI 按钮失效问题的根本原因
/// 使用方法: 将此脚本附加到场景中任意 GameObject 上
/// </summary>
public class InputSystemDebugger : MonoBehaviour
{
    [Header("诊断配置")]
    [Tooltip("每秒检测频率")]
    public float checkInterval = 1f;

    [Tooltip("是否在每次鼠标点击时输出日志")]
    public bool logMouseClicks = true;

    [Tooltip("是否检测 EventSystem 状态")]
    public bool checkEventSystem = true;

    private float nextCheckTime = 0f;
    private EventSystem currentEventSystem;

    void Start()
    {
        Debug.Log("<color=cyan>========== 输入系统诊断器已启动 ==========</color>");
        Debug.Log($"  检测间隔: {checkInterval} 秒");
        Debug.Log($"  鼠标点击日志: {(logMouseClicks ? "启用" : "禁用")}");
        Debug.Log($"  EventSystem 检测: {(checkEventSystem ? "启用" : "禁用")}");
        
        // 初始化时检查一次
        PerformDiagnostics();
    }

    void Update()
    {
        // 鼠标点击检测
        if (logMouseClicks)
        {
            if (Input.GetMouseButtonDown(0))
            {
                Debug.Log($"<color=green>[InputDebug] 检测到鼠标左键点击</color> at {Input.mousePosition}");
                
                // 检查 EventSystem 是否识别到点击
                if (EventSystem.current != null)
                {
                    var currentObject = EventSystem.current.currentSelectedGameObject;
                    Debug.Log($"  EventSystem.current.currentSelectedGameObject: {(currentObject != null ? currentObject.name : "null")}");
                    Debug.Log($"  IsPointerOverGameObject: {EventSystem.current.IsPointerOverGameObject()}");
                }
            }

            if (Input.GetMouseButtonDown(1))
                Debug.Log($"<color=green>[InputDebug] 检测到鼠标右键点击</color> at {Input.mousePosition}");
        }

        // 定期诊断
        if (Time.time >= nextCheckTime)
        {
            nextCheckTime = Time.time + checkInterval;
            if (checkEventSystem)
            {
                PerformDiagnostics();
            }
        }
    }

    /// <summary>
    /// 执行完整诊断
    /// </summary>
    void PerformDiagnostics()
    {
        Debug.Log("<color=cyan>---------- 输入系统诊断报告 ----------</color>");

        // 1. EventSystem 检查
        EventSystem[] eventSystems = FindObjectsOfType<EventSystem>();
        Debug.Log($"<color=yellow>[1] EventSystem 数量:</color> {eventSystems.Length}");
        
        if (eventSystems.Length == 0)
        {
            Debug.LogError("  <color=red>✗ 没有找到 EventSystem！UI 按钮无法工作！</color>");
            Debug.LogError("  <color=yellow>解决方案: 在场景中添加 EventSystem（GameObject → UI → Event System）</color>");
        }
        else if (eventSystems.Length > 1)
        {
            Debug.LogWarning($"  <color=yellow>⚠ 发现多个 EventSystem（{eventSystems.Length} 个），可能导致冲突</color>");
            for (int i = 0; i < eventSystems.Length; i++)
            {
                Debug.LogWarning($"    EventSystem {i + 1}: {eventSystems[i].gameObject.name} (active: {eventSystems[i].enabled})");
            }
        }
        else
        {
            EventSystem es = eventSystems[0];
            currentEventSystem = es;
            Debug.Log($"  <color=green>✓ EventSystem: {es.gameObject.name}</color>");
            Debug.Log($"    • enabled: {es.enabled}");
            Debug.Log($"    • gameObject.activeInHierarchy: {es.gameObject.activeInHierarchy}");
            Debug.Log($"    • currentInputModule: {(es.currentInputModule != null ? es.currentInputModule.GetType().Name : "null")}");
            
            if (es.currentInputModule != null)
            {
                Debug.Log($"    • inputModule.enabled: {es.currentInputModule.enabled}");
            }
            else
            {
                Debug.LogError("  <color=red>✗ EventSystem.currentInputModule 为 null！</color>");
                Debug.LogError("  <color=yellow>解决方案: 添加 StandaloneInputModule 组件到 EventSystem</color>");
            }
        }

        // 2. Input Manager 检查
        Debug.Log($"<color=yellow>[2] Unity Input 状态:</color>");
        Debug.Log($"  • Input.mousePresent: {Input.mousePresent}");
        Debug.Log($"  • Input.touchSupported: {Input.touchSupported}");
        Debug.Log($"  • Input.simulateMouseWithTouches: {Input.simulateMouseWithTouches}");
        Debug.Log($"  • Time.timeScale: {Time.timeScale}");

        // 3. SteamVR 检查
        Debug.Log($"<color=yellow>[3] SteamVR 状态:</color>");
        try
        {
            if (Valve.VR.SteamVR.initializedState == Valve.VR.SteamVR.InitializedStates.InitializeSuccess)
            {
                Debug.Log("  <color=green>✓ SteamVR 已初始化</color>");
            }
            else
            {
                Debug.LogWarning($"  <color=yellow>⚠ SteamVR 状态: {Valve.VR.SteamVR.initializedState}</color>");
            }
        }
        catch (System.Exception ex)
        {
            Debug.LogWarning($"  <color=yellow>⚠ 无法检测 SteamVR 状态: {ex.Message}</color>");
        }

        // 4. Canvas 检查
        Canvas[] canvases = FindObjectsOfType<Canvas>();
        Debug.Log($"<color=yellow>[4] Canvas 数量:</color> {canvases.Length}");
        int screenSpaceOverlayCount = 0;
        foreach (var canvas in canvases)
        {
            if (canvas.renderMode == RenderMode.ScreenSpaceOverlay)
            {
                screenSpaceOverlayCount++;
                Debug.Log($"  ✓ {canvas.gameObject.name} (ScreenSpaceOverlay, enabled: {canvas.enabled})");
            }
        }
        if (screenSpaceOverlayCount == 0)
        {
            Debug.LogWarning("  <color=yellow>⚠ 没有找到 ScreenSpaceOverlay 类型的 Canvas</color>");
        }

        Debug.Log("<color=cyan>---------- 诊断完成 ----------</color>");
    }

    /// <summary>
    /// 手动触发诊断（可通过代码或 Inspector 按钮调用）
    /// </summary>
    [ContextMenu("立即执行诊断")]
    public void ManualDiagnose()
    {
        PerformDiagnostics();
    }
}

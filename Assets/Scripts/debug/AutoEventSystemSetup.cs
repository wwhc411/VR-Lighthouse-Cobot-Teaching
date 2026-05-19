using UnityEngine;
using UnityEngine.EventSystems;

/// <summary>
/// 自动确保场景中存在 EventSystem
/// 如果缺失，会自动创建一个
/// </summary>
[DefaultExecutionOrder(-1000)] // 确保在其他脚本之前执行
public class AutoEventSystemSetup : MonoBehaviour
{
    void Awake()
    {
        // 检查场景中是否存在 EventSystem
        EventSystem existingEventSystem = FindObjectOfType<EventSystem>();
        
        if (existingEventSystem == null)
        {
            Debug.LogWarning("<color=yellow>[AutoEventSystemSetup] 场景中缺少 EventSystem，正在自动创建...</color>");
            
            // 创建 EventSystem GameObject
            GameObject eventSystemObj = new GameObject("EventSystem");
            eventSystemObj.AddComponent<EventSystem>();
            eventSystemObj.AddComponent<StandaloneInputModule>();
            
            Debug.Log("<color=green>[AutoEventSystemSetup] ✓ EventSystem 已自动创建并配置</color>");
            Debug.Log("  • EventSystem: 已添加");
            Debug.Log("  • StandaloneInputModule: 已添加");
            Debug.Log("  <color=cyan>建议: 通过 GameObject → UI → Event System 手动添加以获得更好的控制</color>");
        }
        else
        {
            Debug.Log($"<color=green>[AutoEventSystemSetup] ✓ EventSystem 已存在: {existingEventSystem.gameObject.name}</color>");
            
            // 检查是否有 InputModule
            if (existingEventSystem.currentInputModule == null)
            {
                Debug.LogWarning("<color=yellow>[AutoEventSystemSetup] EventSystem 缺少 InputModule，正在添加...</color>");
                existingEventSystem.gameObject.AddComponent<StandaloneInputModule>();
                Debug.Log("<color=green>[AutoEventSystemSetup] ✓ StandaloneInputModule 已自动添加</color>");
            }
        }
    }
}

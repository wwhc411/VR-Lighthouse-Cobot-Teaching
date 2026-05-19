using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using Valve.VR;
using Valve.VR.Extras;

/// <summary>
/// 激光指针 UI 交互扩展
/// 支持点击 Unity UI 按钮、滑动条等控件
/// 
/// 使用方法：
/// 1. 将此脚本添加到带有 SteamVR_LaserPointer 的控制器上
/// 2. 将 Canvas 设置为 World Space 模式
/// 3. 在 Canvas 上添加 GraphicRaycaster 组件
/// 4. 场景中需要有 EventSystem
/// </summary>
[RequireComponent(typeof(SteamVR_LaserPointer))]
public class LaserPointerUI : MonoBehaviour
{
    [Header("组件引用")]
    [Tooltip("激光指针组件（自动获取）")]
    public SteamVR_LaserPointer laserPointer;
    
    [Tooltip("控制器位姿组件")]
    public SteamVR_Behaviour_Pose pose;

    [Header("交互设置")]
    [Tooltip("UI 交互的最大距离")]
    public float maxDistance = 10f;
    
    [Tooltip("点击触发的 Action")]
    public SteamVR_Action_Boolean clickAction = SteamVR_Input.GetBooleanAction("InteractUI");

    [Header("调试")]
    public bool showDebugLog = false;

    // 内部状态
    private GameObject _currentHoveredObject;
    private GameObject _currentPressedObject;
    private PointerEventData _pointerEventData;
    private Camera _eventCamera;

    void Start()
    {
        if (laserPointer == null)
            laserPointer = GetComponent<SteamVR_LaserPointer>();
        
        if (pose == null)
            pose = GetComponent<SteamVR_Behaviour_Pose>();

        // 创建用于 UI 事件的 PointerEventData
        _pointerEventData = new PointerEventData(EventSystem.current);
        
        // 获取或创建事件相机
        _eventCamera = Camera.main;
        if (_eventCamera == null)
        {
            Debug.LogWarning("[LaserPointerUI] 未找到主相机，UI 交互可能不正常");
        }
    }

    void Update()
    {
        if (EventSystem.current == null)
            return;

        // 执行 UI 射线检测
        Ray ray = new Ray(transform.position, transform.forward);
        PerformUIRaycast(ray);

        // 处理点击输入
        HandleClickInput();
    }

    /// <summary>
    /// 执行 UI 射线检测
    /// </summary>
    void PerformUIRaycast(Ray ray)
    {
        // 更新 PointerEventData
        _pointerEventData.position = GetScreenPosition(ray);
        _pointerEventData.delta = Vector2.zero;

        // 使用 EventSystem 进行射线检测
        var raycastResults = new System.Collections.Generic.List<RaycastResult>();
        EventSystem.current.RaycastAll(_pointerEventData, raycastResults);

        // 过滤结果，只保留距离内的
        RaycastResult? closestResult = null;
        float closestDistance = maxDistance;

        foreach (var result in raycastResults)
        {
            // 计算实际 3D 距离
            float distance = Vector3.Distance(transform.position, result.worldPosition);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestResult = result;
            }
        }

        // 处理 Hover 状态
        GameObject newHoveredObject = closestResult?.gameObject;
        
        if (newHoveredObject != _currentHoveredObject)
        {
            // 离开旧对象
            if (_currentHoveredObject != null)
            {
                ExecuteEvents.Execute(_currentHoveredObject, _pointerEventData, ExecuteEvents.pointerExitHandler);
                
                if (showDebugLog)
                    Debug.Log($"[LaserPointerUI] 离开: {_currentHoveredObject.name}");
            }

            // 进入新对象
            if (newHoveredObject != null)
            {
                ExecuteEvents.Execute(newHoveredObject, _pointerEventData, ExecuteEvents.pointerEnterHandler);
                
                if (showDebugLog)
                    Debug.Log($"[LaserPointerUI] 悬停: {newHoveredObject.name}");
            }

            _currentHoveredObject = newHoveredObject;
        }

        // 更新 PointerEventData 的 pointerCurrentRaycast
        if (closestResult.HasValue)
        {
            _pointerEventData.pointerCurrentRaycast = closestResult.Value;
        }
    }

    /// <summary>
    /// 处理点击输入
    /// </summary>
    void HandleClickInput()
    {
        if (clickAction == null || pose == null)
            return;

        SteamVR_Input_Sources inputSource = pose.inputSource;

        // 按下
        if (clickAction.GetStateDown(inputSource))
        {
            if (_currentHoveredObject != null)
            {
                _currentPressedObject = _currentHoveredObject;
                
                // 触发 PointerDown
                ExecuteEvents.Execute(_currentPressedObject, _pointerEventData, ExecuteEvents.pointerDownHandler);
                
                // 设置选中状态（用于 InputField 等）
                _pointerEventData.selectedObject = _currentPressedObject;
                EventSystem.current.SetSelectedGameObject(_currentPressedObject);

                if (showDebugLog)
                    Debug.Log($"[LaserPointerUI] 按下: {_currentPressedObject.name}");
            }
        }

        // 松开
        if (clickAction.GetStateUp(inputSource))
        {
            if (_currentPressedObject != null)
            {
                // 触发 PointerUp
                ExecuteEvents.Execute(_currentPressedObject, _pointerEventData, ExecuteEvents.pointerUpHandler);

                // 如果松开时仍在同一个对象上，触发 Click
                if (_currentPressedObject == _currentHoveredObject)
                {
                    ExecuteEvents.Execute(_currentPressedObject, _pointerEventData, ExecuteEvents.pointerClickHandler);
                    
                    if (showDebugLog)
                        Debug.Log($"[LaserPointerUI] 点击: {_currentPressedObject.name}");
                }

                _currentPressedObject = null;
            }
        }

        // 拖拽（持续按住时）
        if (clickAction.GetState(inputSource) && _currentPressedObject != null)
        {
            ExecuteEvents.Execute(_currentPressedObject, _pointerEventData, ExecuteEvents.dragHandler);
        }
    }

    /// <summary>
    /// 将 3D 射线转换为屏幕坐标（用于 UI 事件系统）
    /// </summary>
    Vector2 GetScreenPosition(Ray ray)
    {
        if (_eventCamera == null)
            return Vector2.zero;

        // 将射线方向投影到屏幕空间
        Vector3 worldPoint = ray.origin + ray.direction * 1f;
        Vector3 screenPoint = _eventCamera.WorldToScreenPoint(worldPoint);
        return new Vector2(screenPoint.x, screenPoint.y);
    }

    void OnDisable()
    {
        // 清理状态
        if (_currentHoveredObject != null && _pointerEventData != null)
        {
            ExecuteEvents.Execute(_currentHoveredObject, _pointerEventData, ExecuteEvents.pointerExitHandler);
            _currentHoveredObject = null;
        }
        _currentPressedObject = null;
    }
}

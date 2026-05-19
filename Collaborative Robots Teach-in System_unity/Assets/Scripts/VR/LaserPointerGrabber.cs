using UnityEngine;
using Valve.VR;
using Valve.VR.Extras;

/// <summary>
/// 激光指针远程抓取功能
/// 用激光指向物体，按下按键后将物体"吸附"到手中或保持在原位置跟随
/// 
/// 使用方法：
/// 1. 将此脚本添加到带有 SteamVR_LaserPointer 的控制器上
/// 2. 确保可抓取物体上有 Collider 和 Rigidbody
/// 3. 可选：给物体添加 "Grabbable" 标签来限制可抓取范围
/// 
/// 输入模式：
/// - SteamVR Action 模式：需要配置 SteamVR Input 绑定
/// - OpenVR 直接模式：直接读取控制器按键，无需配置绑定
/// </summary>
[RequireComponent(typeof(SteamVR_LaserPointer))]
public class LaserPointerGrabber : MonoBehaviour
{
    [Header("组件引用")]
    public SteamVR_LaserPointer laserPointer;
    public SteamVR_Behaviour_Pose pose;

    [Header("输入模式")]
    [Tooltip("使用 OpenVR 直接读取按键（推荐，无需配置绑定）")]
    public bool useOpenVRDirectInput = true;
    
    [Tooltip("OpenVR 直接模式下使用的按键")]
    public OpenVRButton openVRButton = OpenVRButton.Grip;
    
    [Tooltip("控制器设备索引（0=自动检测）")]
    public uint controllerDeviceIndex = 0;

    [Header("SteamVR Action 模式（需要配置绑定）")]
    [Tooltip("抓取触发的 Action（仅在非直接模式下使用）")]
    public SteamVR_Action_Boolean grabAction;

    public enum OpenVRButton
    {
        Trigger,        // 扳机键
        Grip,           // 侧握键
        Touchpad,       // 触摸板按下
        Menu,           // 菜单键
        A,              // A 按钮
        B               // B 按钮
    }
    
    [Tooltip("抓取模式")]
    public GrabMode grabMode = GrabMode.AttachToHand;
    
    [Tooltip("抓取距离限制")]
    public float maxGrabDistance = 10f;
    
    [Tooltip("仅抓取带有此标签的物体（留空则抓取所有）")]
    public string grabbableTag = "";
    
    [Tooltip("抓取时物体到手的距离（AttachToHand 模式）")]
    public float attachDistance = 0.3f;

    [Header("物理设置")]
    [Tooltip("抓取时的跟随速度")]
    public float followSpeed = 15f;
    
    [Tooltip("抓取时的旋转跟随速度")]
    public float rotationSpeed = 10f;
    
    [Tooltip("松开时是否保留速度（用于抛出）")]
    public bool throwOnRelease = true;
    
    [Tooltip("抛出速度倍数")]
    public float throwVelocityMultiplier = 1.5f;

    [Header("状态显示（只读）")]
    [SerializeField] private bool _isGrabbing = false;
    [SerializeField] private string _currentTargetName = "";

    [Header("调试")]
    [Tooltip("显示详细调试日志")]
    public bool showDebugLog = true;
    
    [Tooltip("显示射线 Gizmo")]
    public bool showRayGizmo = true;

    // 抓取模式枚举
    public enum GrabMode
    {
        [Tooltip("将物体吸附到手附近")]
        AttachToHand,
        
        [Tooltip("物体保持原距离跟随")]
        KeepDistance,
        
        [Tooltip("物体保持在世界空间位置，跟随手的旋转")]
        FollowRotationOnly
    }

    // 内部状态
    private GameObject _grabbedObject;
    private Rigidbody _grabbedRigidbody;
    private float _grabDistance;
    private Vector3 _grabOffset;
    private Quaternion _grabRotationOffset;
    private bool _wasKinematic;
    private bool _usedGravity;
    
    // 速度追踪（用于抛出）
    private Vector3 _lastPosition;
    private Vector3 _velocity;

    // OpenVR 直接输入
    private VRControllerState_t _controllerState;
    private VRControllerState_t _lastControllerState;
    private uint _cachedDeviceIndex = OpenVR.k_unTrackedDeviceIndexInvalid;

    void Start()
    {
        if (laserPointer == null)
            laserPointer = GetComponent<SteamVR_LaserPointer>();
        
        if (pose == null)
            pose = GetComponent<SteamVR_Behaviour_Pose>();

        // 订阅激光指针事件
        if (laserPointer != null)
        {
            laserPointer.PointerIn += OnPointerIn;
            laserPointer.PointerOut += OnPointerOut;
        }
        
        // 初始化 OpenVR 直接输入
        if (useOpenVRDirectInput)
        {
            Debug.Log("[LaserGrabber] 使用 OpenVR 直接输入模式（无需配置 SteamVR Input 绑定）");
        }
    }

    void OnDestroy()
    {
        if (laserPointer != null)
        {
            laserPointer.PointerIn -= OnPointerIn;
            laserPointer.PointerOut -= OnPointerOut;
        }
    }

    void OnPointerIn(object sender, PointerEventArgs e)
    {
        if (!_isGrabbing)
        {
            _currentTargetName = e.target.name;
        }
    }

    void OnPointerOut(object sender, PointerEventArgs e)
    {
        if (!_isGrabbing)
        {
            _currentTargetName = "";
        }
    }

    void Update()
    {
        HandleGrabInput();
        
        if (_isGrabbing && _grabbedObject != null)
        {
            UpdateGrabbedObject();
        }
    }

    /// <summary>
    /// 处理抓取输入
    /// </summary>
    void HandleGrabInput()
    {
        bool buttonDown = false;
        bool buttonUp = false;
        bool buttonHeld = false;

        if (useOpenVRDirectInput)
        {
            // 使用 OpenVR 直接读取按键
            HandleOpenVRDirectInput(out buttonDown, out buttonUp, out buttonHeld);
        }
        else
        {
            // 使用 SteamVR Action 系统
            HandleSteamVRActionInput(out buttonDown, out buttonUp, out buttonHeld);
        }

        // 按下抓取键
        if (buttonDown)
        {
            if (showDebugLog)
                Debug.Log($"[LaserGrabber] 检测到抓取键按下！");
            TryGrab();
        }

        // 松开抓取键
        if (buttonUp)
        {
            Release();
        }
    }

    /// <summary>
    /// OpenVR 直接输入处理
    /// </summary>
    void HandleOpenVRDirectInput(out bool buttonDown, out bool buttonUp, out bool buttonHeld)
    {
        buttonDown = false;
        buttonUp = false;
        buttonHeld = false;

        if (OpenVR.System == null)
            return;

        // 获取控制器设备索引
        uint deviceIndex = GetControllerDeviceIndex();
        if (deviceIndex == OpenVR.k_unTrackedDeviceIndexInvalid)
        {
            if (showDebugLog && Time.frameCount % 300 == 0)
                Debug.LogWarning("[LaserGrabber] 未找到控制器设备");
            return;
        }

        // 保存上一帧状态
        _lastControllerState = _controllerState;

        // 获取当前控制器状态
        if (!OpenVR.System.GetControllerState(deviceIndex, ref _controllerState, (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(VRControllerState_t))))
        {
            return;
        }

        // 获取按钮掩码
        ulong buttonMask = GetButtonMask(openVRButton);
        
        bool wasPressed = (_lastControllerState.ulButtonPressed & buttonMask) != 0;
        bool isPressed = (_controllerState.ulButtonPressed & buttonMask) != 0;

        buttonDown = isPressed && !wasPressed;
        buttonUp = !isPressed && wasPressed;
        buttonHeld = isPressed;

        // 调试输出
        if (showDebugLog && Time.frameCount % 60 == 0)
        {
            Debug.Log($"[LaserGrabber] OpenVR直接输入: 设备:{deviceIndex}, 按钮:{openVRButton}, 按下:{isPressed}");
        }
    }

    /// <summary>
    /// SteamVR Action 输入处理
    /// </summary>
    void HandleSteamVRActionInput(out bool buttonDown, out bool buttonUp, out bool buttonHeld)
    {
        buttonDown = false;
        buttonUp = false;
        buttonHeld = false;

        if (grabAction == null)
        {
            if (showDebugLog && Time.frameCount % 300 == 0)
                Debug.LogWarning("[LaserGrabber] grabAction 为空！请在 Inspector 中配置或检查 SteamVR Input");
            return;
        }
        
        if (pose == null)
        {
            if (showDebugLog && Time.frameCount % 300 == 0)
                Debug.LogWarning("[LaserGrabber] pose 为空！请确保控制器上有 SteamVR_Behaviour_Pose 组件");
            return;
        }

        SteamVR_Input_Sources inputSource = pose.inputSource;
        
        // 调试：每秒检测一次按键状态
        if (showDebugLog && Time.frameCount % 60 == 0)
        {
            bool isActive = grabAction.GetActive(inputSource);
            bool state = grabAction.GetState(inputSource);
            Debug.Log($"[LaserGrabber] Action活跃:{isActive}, 按下:{state}, 输入源:{inputSource}");
        }

        buttonDown = grabAction.GetStateDown(inputSource);
        buttonUp = grabAction.GetStateUp(inputSource);
        buttonHeld = grabAction.GetState(inputSource);
    }

    /// <summary>
    /// 获取控制器设备索引
    /// </summary>
    uint GetControllerDeviceIndex()
    {
        // 如果手动指定了设备索引
        if (controllerDeviceIndex > 0 && controllerDeviceIndex < OpenVR.k_unMaxTrackedDeviceCount)
        {
            return controllerDeviceIndex;
        }

        // 根据 pose 的 inputSource 自动检测
        if (pose != null)
        {
            ETrackedControllerRole role = pose.inputSource == SteamVR_Input_Sources.LeftHand 
                ? ETrackedControllerRole.LeftHand 
                : ETrackedControllerRole.RightHand;
            
            uint index = OpenVR.System.GetTrackedDeviceIndexForControllerRole(role);
            if (index != OpenVR.k_unTrackedDeviceIndexInvalid)
            {
                _cachedDeviceIndex = index;
                return index;
            }
        }

        // 使用缓存的索引
        if (_cachedDeviceIndex != OpenVR.k_unTrackedDeviceIndexInvalid)
        {
            return _cachedDeviceIndex;
        }

        // 遍历查找第一个控制器
        for (uint i = 0; i < OpenVR.k_unMaxTrackedDeviceCount; i++)
        {
            if (OpenVR.System.GetTrackedDeviceClass(i) == ETrackedDeviceClass.Controller)
            {
                _cachedDeviceIndex = i;
                return i;
            }
        }

        return OpenVR.k_unTrackedDeviceIndexInvalid;
    }

    /// <summary>
    /// 获取 OpenVR 按钮掩码
    /// <summary>
    /// 获取 OpenVR 按钮掩码
    /// </summary>
    ulong GetButtonMask(OpenVRButton button)
    {
        switch (button)
        {
            case OpenVRButton.Trigger:
                return 1ul << (int)EVRButtonId.k_EButton_SteamVR_Trigger;
            case OpenVRButton.Grip:
                return 1ul << (int)EVRButtonId.k_EButton_Grip;
            case OpenVRButton.Touchpad:
                return 1ul << (int)EVRButtonId.k_EButton_SteamVR_Touchpad;
            case OpenVRButton.Menu:
                return 1ul << (int)EVRButtonId.k_EButton_ApplicationMenu;
            case OpenVRButton.A:
                return 1ul << (int)EVRButtonId.k_EButton_A;
            case OpenVRButton.B:
                return 1ul << (int)EVRButtonId.k_EButton_ApplicationMenu; // B 通常映射到 Menu
            default:
                return 1ul << (int)EVRButtonId.k_EButton_Grip;
        }
    }

    /// <summary>
    /// 尝试抓取激光指向的物体
    /// </summary>
    void TryGrab()
    {
        Ray ray = new Ray(transform.position, transform.forward);
        RaycastHit hit;

        if (showDebugLog)
            Debug.Log($"[LaserGrabber] 尝试抓取，射线起点:{transform.position}, 方向:{transform.forward}");

        if (Physics.Raycast(ray, out hit, maxGrabDistance))
        {
            if (showDebugLog)
                Debug.Log($"[LaserGrabber] 射线击中: {hit.collider.gameObject.name}, 距离: {hit.distance:F2}m");
            
            GameObject target = hit.collider.gameObject;
            
            // 检查标签
            if (!string.IsNullOrEmpty(grabbableTag) && !target.CompareTag(grabbableTag))
            {
                Debug.Log($"[LaserGrabber] {target.name} 不可抓取（标签不匹配）");
                return;
            }

            // 获取 Rigidbody
            Rigidbody rb = target.GetComponent<Rigidbody>();
            if (rb == null)
            {
                rb = target.GetComponentInParent<Rigidbody>();
            }

            if (rb == null)
            {
                Debug.Log($"[LaserGrabber] {target.name} 没有 Rigidbody，无法抓取");
                return;
            }

            // 开始抓取
            _grabbedObject = rb.gameObject;
            _grabbedRigidbody = rb;
            _grabDistance = hit.distance;
            _isGrabbing = true;
            _currentTargetName = _grabbedObject.name;

            // 保存原始物理状态
            _wasKinematic = rb.isKinematic;
            _usedGravity = rb.useGravity;

            // 计算抓取偏移
            _grabOffset = Quaternion.Inverse(transform.rotation) * (_grabbedObject.transform.position - transform.position);
            _grabRotationOffset = Quaternion.Inverse(transform.rotation) * _grabbedObject.transform.rotation;

            // 根据模式设置物理
            if (grabMode == GrabMode.AttachToHand)
            {
                rb.useGravity = false;
                _grabDistance = attachDistance;
            }
            else if (grabMode == GrabMode.KeepDistance)
            {
                rb.useGravity = false;
            }

            _lastPosition = _grabbedObject.transform.position;

            Debug.Log($"[LaserGrabber] 抓取: {_grabbedObject.name}，距离: {_grabDistance:F2}m");
        }
    }

    /// <summary>
    /// 更新被抓取物体的位置
    /// </summary>
    void UpdateGrabbedObject()
    {
        if (_grabbedRigidbody == null)
            return;

        Vector3 targetPosition;
        Quaternion targetRotation;

        switch (grabMode)
        {
            case GrabMode.AttachToHand:
                // 物体跟随到手前方固定距离
                targetPosition = transform.position + transform.forward * attachDistance;
                targetRotation = transform.rotation * _grabRotationOffset;
                break;

            case GrabMode.KeepDistance:
                // 物体保持原来的抓取距离
                targetPosition = transform.position + transform.forward * _grabDistance;
                targetRotation = transform.rotation * _grabRotationOffset;
                break;

            case GrabMode.FollowRotationOnly:
            default:
                // 物体位置不变，只跟随旋转
                targetPosition = _grabbedObject.transform.position;
                targetRotation = transform.rotation * _grabRotationOffset;
                break;
        }

        // 平滑移动（使用物理）
        if (!_grabbedRigidbody.isKinematic)
        {
            Vector3 velocity = (targetPosition - _grabbedRigidbody.position) * followSpeed;
            _grabbedRigidbody.velocity = velocity;

            // 旋转
            Quaternion rotationDiff = targetRotation * Quaternion.Inverse(_grabbedRigidbody.rotation);
            rotationDiff.ToAngleAxis(out float angle, out Vector3 axis);
            if (angle > 180f) angle -= 360f;
            _grabbedRigidbody.angularVelocity = axis * (angle * Mathf.Deg2Rad * rotationSpeed);
        }
        else
        {
            // Kinematic 物体直接设置位置
            _grabbedRigidbody.MovePosition(Vector3.Lerp(_grabbedRigidbody.position, targetPosition, Time.deltaTime * followSpeed));
            _grabbedRigidbody.MoveRotation(Quaternion.Slerp(_grabbedRigidbody.rotation, targetRotation, Time.deltaTime * rotationSpeed));
        }

        // 追踪速度（用于抛出）
        _velocity = (_grabbedObject.transform.position - _lastPosition) / Time.deltaTime;
        _lastPosition = _grabbedObject.transform.position;
    }

    /// <summary>
    /// 释放抓取的物体
    /// </summary>
    void Release()
    {
        if (!_isGrabbing || _grabbedRigidbody == null)
            return;

        // 恢复物理状态
        _grabbedRigidbody.isKinematic = _wasKinematic;
        _grabbedRigidbody.useGravity = _usedGravity;

        // 抛出
        if (throwOnRelease && !_wasKinematic)
        {
            _grabbedRigidbody.velocity = _velocity * throwVelocityMultiplier;
            
            // 可选：添加控制器的速度
            if (pose != null)
            {
                Vector3 controllerVelocity = pose.GetVelocity();
                Vector3 angularVelocity = pose.GetAngularVelocity();
                
                _grabbedRigidbody.velocity += controllerVelocity * throwVelocityMultiplier;
                _grabbedRigidbody.angularVelocity = angularVelocity;
            }
        }

        Debug.Log($"[LaserGrabber] 释放: {_grabbedObject.name}");

        // 清理状态
        _grabbedObject = null;
        _grabbedRigidbody = null;
        _isGrabbing = false;
        _currentTargetName = "";
    }

    /// <summary>
    /// 强制释放当前抓取的物体
    /// </summary>
    public void ForceRelease()
    {
        Release();
    }

    /// <summary>
    /// 是否正在抓取物体
    /// </summary>
    public bool IsGrabbing => _isGrabbing;

    /// <summary>
    /// 当前抓取的物体
    /// </summary>
    public GameObject GrabbedObject => _grabbedObject;

    /// <summary>
    /// 诊断当前配置状态
    /// </summary>
    [ContextMenu("诊断配置")]
    public void DiagnoseSetup()
    {
        Debug.Log("========== LaserPointerGrabber 诊断 ==========");
        
        // 检查组件
        Debug.Log($"1. SteamVR_LaserPointer: {(laserPointer != null ? "✓ 已配置" : "✗ 缺失")}");
        Debug.Log($"2. SteamVR_Behaviour_Pose: {(pose != null ? "✓ 已配置" : "✗ 缺失")}");
        
        if (pose != null)
        {
            Debug.Log($"   - Input Source: {pose.inputSource}");
        }
        
        // 检查 Action
        if (grabAction != null)
        {
            Debug.Log($"3. Grab Action: ✓ 已配置");
            Debug.Log($"   - Action 名称: {grabAction.fullPath}");
            
            if (pose != null)
            {
                bool isActive = grabAction.GetActive(pose.inputSource);
                Debug.Log($"   - Action 活跃: {(isActive ? "✓ 是" : "✗ 否 - 可能未绑定按键")}");
            }
        }
        else
        {
            Debug.LogError("3. Grab Action: ✗ 缺失！请配置 grabAction");
        }
        
        // 检查标签设置
        if (!string.IsNullOrEmpty(grabbableTag))
        {
            Debug.Log($"4. 抓取标签筛选: 仅抓取 '{grabbableTag}' 标签的物体");
        }
        else
        {
            Debug.Log($"4. 抓取标签筛选: 已禁用（可抓取所有物体）");
        }
        
        // 射线测试
        Ray ray = new Ray(transform.position, transform.forward);
        RaycastHit hit;
        if (Physics.Raycast(ray, out hit, maxGrabDistance))
        {
            Debug.Log($"5. 射线检测: ✓ 击中 '{hit.collider.gameObject.name}'，距离 {hit.distance:F2}m");
            
            var rb = hit.collider.GetComponent<Rigidbody>() ?? hit.collider.GetComponentInParent<Rigidbody>();
            if (rb != null)
            {
                Debug.Log($"   - Rigidbody: ✓ 存在");
            }
            else
            {
                Debug.LogWarning($"   - Rigidbody: ✗ 缺失！目标物体需要 Rigidbody 才能被抓取");
            }
        }
        else
        {
            Debug.LogWarning($"5. 射线检测: ✗ 未击中任何物体（最大距离 {maxGrabDistance}m）");
        }
        
        Debug.Log("==============================================");
    }

    void OnDrawGizmos()
    {
        if (!showRayGizmo)
            return;
            
        // 绘制射线
        Gizmos.color = _isGrabbing ? Color.green : Color.cyan;
        Gizmos.DrawRay(transform.position, transform.forward * maxGrabDistance);
        
        // 绘制射线击中点
        Ray ray = new Ray(transform.position, transform.forward);
        RaycastHit hit;
        if (Physics.Raycast(ray, out hit, maxGrabDistance))
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(hit.point, 0.05f);
        }
    }
}

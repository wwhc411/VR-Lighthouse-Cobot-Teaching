using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR;
using Valve.VR.InteractionSystem;

/// <summary>
/// 简单抓取脚本 - 实现 VR 控制器抓取和释放物体的基本功能
/// 使用方法：
/// 1. 将此脚本添加到可抓取的物体上
/// 2. 确保物体上有 Interactable 组件
/// 3. 确保物体上有 Collider 组件
/// </summary>
public class SimpleAttach : MonoBehaviour
{
    private Interactable interactable;

    void Start()
    {
        // 获取物体上的 Interactable 组件
        interactable = GetComponent<Interactable>();
    }

    /// <summary>
    /// 当控制器开始悬停在物体上时调用
    /// </summary>
    private void OnHandHoverBegin(Hand hand)
    {
        // 显示抓取提示（控制器上的文字/图标）
        hand.ShowGrabHint();
    }

    /// <summary>
    /// 当控制器离开物体悬停范围时调用
    /// </summary>
    private void OnHandHoverEnd(Hand hand)
    {
        // 隐藏抓取提示
        hand.HideGrabHint();
    }

    /// <summary>
    /// 控制器悬停期间每帧调用
    /// </summary>
    private void HandHoverUpdate(Hand hand)
    {
        // 检测用户是否开始抓取动作（按下扳机/握把）
        GrabTypes grabType = hand.GetGrabStarting();
        
        // 检测用户是否正在释放此物体
        bool isGrabEnding = hand.IsGrabEnding(gameObject);

        // ========== 抓取物体 ==========
        // 条件：物体未被任何手附着 且 用户按下了抓取按钮
        if (interactable.attachedToHand == null && grabType != GrabTypes.None)
        {
            // 将物体附着到控制器上
            hand.AttachObject(gameObject, grabType);
            
            // 锁定悬停状态（防止切换到其他物体）
            hand.HoverLock(interactable);
        }
        
        // ========== 释放物体 ==========
        // 条件：用户松开了抓取按钮
        else if (isGrabEnding)
        {
            // 从控制器上分离物体
            hand.DetachObject(gameObject);
            
            // 解除悬停锁定
            hand.HoverUnlock(interactable);
        }
    }
}

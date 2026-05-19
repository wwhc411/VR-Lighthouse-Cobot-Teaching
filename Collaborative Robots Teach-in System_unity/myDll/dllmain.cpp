/**
 * @file dllmain.cpp
 * @brief DLL入口点文件
 * @details 定义DLL的主入口函数DllMain，处理DLL的加载、卸载以及线程附加/分离事件
 */

// dllmain.cpp : 定义 DLL 应用程序的入口点。
#include "pch.h"

/**
 * @brief DLL主入口函数
 * @details Windows系统在加载/卸载DLL或创建/销毁线程时会自动调用此函数
 *          这是DLL的生命周期管理函数，可以在此处执行初始化和清理工作
 * 
 * @param[in] hModule DLL模块句柄，用于标识当前DLL实例
 * @param[in] ul_reason_for_call 调用原因，指示DLL被调用的具体事件
 *            - DLL_PROCESS_ATTACH: 进程首次加载DLL时调用
 *            - DLL_THREAD_ATTACH:  新线程被创建时调用（除首次加载进程的主线程外）
 *            - DLL_THREAD_DETACH:  线程正常退出时调用
 *            - DLL_PROCESS_DETACH: 进程卸载DLL时调用（通过FreeLibrary或进程终止）
 * @param[in] lpReserved 保留参数
 *            - 动态加载时（LoadLibrary）：为NULL
 *            - 静态加载时（链接时）：为非NULL值
 * 
 * @return BOOL 
 *         - TRUE: 初始化成功，允许DLL继续加载
 *         - FALSE: 初始化失败，DLL加载将被中止
 * 
 * @note 当前实现中未进行特殊处理，所有事件直接返回TRUE
 * @note 避免在DllMain中执行复杂操作，可能导致死锁或其他问题
 */
BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
        // DLL被进程首次加载时执行
        // 可以在此处进行一次性初始化工作，如分配资源、初始化全局变量等
        break;
        
    case DLL_THREAD_ATTACH:
        // 进程创建新线程时执行（主线程除外）
        // 可以在此处进行线程相关的初始化
        break;
        
    case DLL_THREAD_DETACH:
        // 线程正常退出时执行
        // 可以在此处进行线程相关的清理工作
        break;
        
    case DLL_PROCESS_DETACH:
        // DLL从进程地址空间卸载时执行
        // 可以在此处进行清理工作，如释放资源、关闭文件句柄等
        break;
    }
    return TRUE;
}


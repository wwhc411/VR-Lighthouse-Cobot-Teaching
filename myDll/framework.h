/**
 * @file framework.h
 * @brief Windows DLL框架头文件
 * @details 包含Windows API的标准头文件，用于DLL开发
 */

#pragma once

/**
 * @def WIN32_LEAN_AND_MEAN
 * @brief 减少Windows头文件的内容，加快编译速度
 * @details 定义此宏后，会从windows.h中排除以下内容：
 *          - Cryptography API
 *          - DDE (Dynamic Data Exchange)
 *          - RPC (Remote Procedure Call)
 *          - Shell API
 *          - Windows Sockets 1.1
 *          - COM (Component Object Model)
 *          等极少使用的API，可以显著减少编译时间和生成文件大小
 */
#define WIN32_LEAN_AND_MEAN

// 包含Windows API头文件，提供DLL开发所需的基础定义
// 如：BOOL, DWORD, HMODULE, APIENTRY等类型和宏
#include <windows.h>

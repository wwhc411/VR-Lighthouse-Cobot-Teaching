/****************************************************************************
MIT许可证
版权所有 (c) 2021 Roman Parak
特此免费授予任何获得本软件及相关文档文件（"软件"）副本的人不受限制地处理
软件的权利，包括但不限于使用、复制、修改、合并、发布、分发、再许可和/或销售
软件副本的权利，并允许获得软件的人员这样做，但须符合以下条件：
上述版权声明和本许可声明应包含在软件的所有副本或主要部分中。
本软件按"原样"提供，不提供任何形式的明示或暗示保证，包括但不限于对适销性、
特定用途适用性和非侵权性的保证。在任何情况下，作者或版权持有人均不对任何索赔、
损害或其他责任负责，无论是在合同、侵权行为或其他方面的诉讼中，由软件或软件的
使用或其他交易引起、由此产生或与之相关。
*****************************************************************************
作者     : Roman Parak
邮箱     : Roman.Parak @outlook.com
Github   : https://github.com/rparak
文件名   : ur_data_processing.cs
功能描述 : UR机器人TCP/IP通讯核心处理模块，负责与UR机器人进行双向数据通讯
****************************************************************************/

// 系统库
using System;
using System.Diagnostics;           // 计时器功能
using System.Net.Sockets;          // TCP/IP网络通讯
using System.Threading;            // 多线程处理
using System.Collections.Generic;  // 集合类型
// Unity引擎库
using UnityEngine;
using Debug = UnityEngine.Debug;   // Unity调试输出


/// <summary>
/// UR机器人数据处理主类
/// 负责管理与UR机器人的TCP/IP通讯，包括数据读取和控制命令发送
/// </summary>
public class ur_data_processing : MonoBehaviour
{
    /// <summary>
    /// 全局主控制变量
    /// 用于控制机器人连接和断开状态
    /// </summary>
    public static class GlobalVariables_Main_Control
    {
        public static bool connect, disconnect;     // 连接和断开控制标志
    }

    /// <summary>
    /// UR机器人数据流类
    /// 负责从机器人读取实时数据（只读模式）
    /// </summary>
    public static class UR_Stream_Data
    {
        // IP地址和端口号配置
        public static string ip_address;              // 机器人IP地址
        public const ushort port_number = 30013;     // 实时数据端口（只读）
        
        // 通讯参数
        public static int time_step;                 // 通讯周期（毫秒）
        
        // 关节空间数据：
        public static double[] J_Orientation = new double[6];  // 关节角度 {J1..J6} (弧度)
        
        // 笛卡尔空间数据：
        public static double[] C_Position = new double[3];     // 位置 {X, Y, Z} (米)
        public static double[] C_Orientation = new double[3];  // 姿态 {轴角} (弧度)
        
        // 线程状态信息
        public static bool is_alive = false;         // 线程是否存活
    }
    /// <summary>
    /// UR机器人控制数据类
    /// 负责向机器人发送控制命令（读写模式）
    /// </summary>
    public static class UR_Control_Data
    {
        // IP地址和端口号配置
        public static string ip_address;              // 机器人IP地址
        public const ushort port_number = 30003;     // 实时控制端口（读写）
        
        // 通讯参数
        public static int time_step;                 // 通讯周期（毫秒）
        
        // UR3/UR3e控制参数：
        public static string aux_command_str;        // 辅助命令字符串
        public static byte[] command;                // 字节格式的命令
        public static bool[] button_pressed = new bool[12];  // 按钮按下状态（12个按钮）
        public static bool joystick_button_pressed;  // 摇杆按钮按下标志（由按钮阵列统计得出）
        public static bool manual_send_active;       // 手动发送（例如笛卡尔面板“发送一次”脉冲）
        
        // 线程状态信息
        public static bool is_alive = false;         // 线程是否存活
    }

    // UR机器人TCP/IP通讯类实例
    private UR_Stream ur_stream_robot;      // 数据流处理实例
    private UR_Control ur_ctrl_robot;       // 控制命令处理实例
    
    // 其他变量
    private int main_ur3_state = 0;         // 主状态机状态（0=断开，1=连接）
    private int aux_counter_pressed_btn = 0;// 按下按钮计数器

    /// <summary>
    /// Unity启动时调用，初始化UR机器人通讯参数
    /// </summary>
    void Start()
    {
        // 初始化UR机器人TCP/IP通讯参数
        
        // 数据读取配置：
        UR_Stream_Data.ip_address = "192.168.1.103";    // 默认本地IP（仿真器）
        // 通讯频率：CB系列125Hz(8ms)，E系列500Hz(2ms)
        UR_Stream_Data.time_step = 8;
        
        // 控制写入配置：
        UR_Control_Data.ip_address = "192.168.1.103";   // 默认本地IP（仿真器）
        // 通讯频率：CB系列125Hz(8ms)，E系列500Hz(2ms)
        UR_Control_Data.time_step = 8;

        // 初始化UR机器人TCP/IP通讯实例
        ur_stream_robot = new UR_Stream();    // 创建数据流处理实例
        ur_ctrl_robot = new UR_Control();     // 创建控制处理实例
    }

    /// <summary>
    /// Unity固定更新函数，主状态机循环处理
    /// </summary>
     void FixedUpdate()
    {
        switch (main_ur3_state)
        {
            case 0:
                {
                    // ------------------------ 等待状态 {断开连接状态} ------------------------//

                    if (GlobalVariables_Main_Control.connect == true)
                    {
                        // 启动数据流处理线程
                        ur_stream_robot.Start();
                        // 启动控制命令处理线程
                        ur_ctrl_robot.Start();

                        // 切换到连接状态
                        main_ur3_state = 1;
                    }
                }
                break;
            case 1:
                {
                    // ------------------------ 数据处理状态 {已连接状态} ------------------------//

                    // 检查摇杆控制模式下的按钮状态
                    for (int i = 0; i < UR_Control_Data.button_pressed.Length; i++)
                    {
                        // 统计按下的按钮数量
                        if (UR_Control_Data.button_pressed[i] == true)
                        {
                            aux_counter_pressed_btn++;
                        }
                    }

                    // 如果至少有一个按钮被按下
                    if (aux_counter_pressed_btn > 0)
                    {
                        // 开始移动 -> 进入速度控制模式
                        UR_Control_Data.joystick_button_pressed = true;
                    }
                    else
                    {
                        // 停止移动 -> 退出速度控制模式
                        UR_Control_Data.joystick_button_pressed = false;
                    }

                    // 重置辅助计数变量
                    aux_counter_pressed_btn = 0;

                    if (GlobalVariables_Main_Control.disconnect == true)
                    {
                        // 停止数据读取线程
                        if (UR_Stream_Data.is_alive == true)
                        {
                            ur_stream_robot.Stop();
                        }
                        // 停止控制写入线程
                        if (UR_Control_Data.is_alive == true)
                        {
                            ur_ctrl_robot.Stop();
                        }
                        // 当两个线程都停止后，返回初始状态
                        if (UR_Stream_Data.is_alive == false && UR_Control_Data.is_alive == false)
                        {
                            // 返回到等待状态（断开连接状态）
                            main_ur3_state = 0;
                        }
                    }
                }
                break;
        }
    }

    /// <summary>
    /// 应用程序退出时的清理工作
    /// </summary>
    void OnApplicationQuit()
    {
        try
        {
            // 销毁数据流处理实例，释放TCP/IP连接
            ur_stream_robot.Destroy();
            // 销毁控制处理实例，释放TCP/IP连接
            ur_ctrl_robot.Destroy();

            // 销毁当前对象
            Destroy(this);
        }
        catch (Exception e)
        {
           Debug.LogException(e);    // 输出异常信息
        }
    }

    /// <summary>
    /// UR机器人数据流处理类
    /// 负责从机器人实时读取关节角度和位置数据
    /// </summary>
    class UR_Stream
    {
        // 类变量初始化
        
        // 线程相关
        private Thread robot_thread = null;         // 机器人通讯线程
        private bool exit_thread = false;           // 线程退出标志
        
        // TCP/IP通讯相关
        private TcpClient tcp_client = new TcpClient();     // TCP客户端
        private NetworkStream network_stream = null;        // 网络数据流
        
        // 数据包缓冲区（读取）
        private byte[] packet = new byte[1220];     // 1220字节的数据包缓冲区（兼容UR3e）
        
        // 主状态机
        private int state_id = 0;                   // 状态ID（0=初始化，1=数据读取）

        // 数据偏移量定义：
        private const byte first_packet_size = 4;  // 第一个数据包大小（整数，4字节）
        private const byte offset = 8;              // 其他数据包大小（双精度，8字节）

        // 消息总长度（字节）
        private static List<UInt32> msg_length_list = new List<UInt32>();   // 消息长度列表 在初始化阶段（state 0）用来“采样”前几帧收到的数据包长度，收集一批样本。
        private static UInt32 total_msg_length = 0;                         // 总消息长度  确定并保存“标准/目标”的数据包长度，后续用于校验每一帧是否是期望格式

// 代码里的流程对照：
// 初始化阶段（state 0）
// 连续读取数据包长度 → 加入 msg_length_list
// 收够 10 个 → msg_length_list.Sort() → 取最大值赋给 total_msg_length → 切到 state 1
// 解析阶段（state 1）
// 每帧读取后先判断 BitConverter.ToUInt32(packet, 0) == total_msg_length
// 相等 → 才进行 Array.Reverse(packet) 和后续的关节/位姿解包
// 不等 → 丢弃该帧（跳过解析）

        /// <summary>
        /// UR机器人数据流处理线程
        /// 持续从机器人读取实时数据
        /// </summary>
        public void UR_Stream_Thread()
        {
            try
            {
                if (tcp_client.Connected == false)
                {
                    // 如果控制器未连接，则连接到控制器
                    tcp_client.Connect(UR_Stream_Data.ip_address, UR_Stream_Data.port_number);
                }

                // 初始化TCP/IP通讯数据流
                network_stream = tcp_client.GetStream();
                // 避免命令侧的 Nagle 延迟
                tcp_client.NoDelay = true;
                // 避免 Nagle 算法引入的缓冲延迟，提升实时性
                tcp_client.NoDelay = true;

                while (exit_thread == false)
                {
                    switch (state_id)
                    {
                        case 0:
                            {
                                // 通过多次读取数据来获取消息总长度
                                // Read(byte[] buffer, int offset, int size)。三个参数的作用如下：
                                // buffer（这里是 packet）
                                // 用来承接读取到的数据的字节数组，读出的数据会被写进这个数组中。
                                // offset（这里是 0）
                                // 在 buffer 中开始写入的位置索引。0 表示从数组开头写入。如果设成 10，表示从 buffer[10] 开始写。
                                // size（这里是 packet.Length）
                                // 本次最多读取的字节数。这里传入 packet.Length，表示尝试读满整个缓冲区（1220 字节），但注意“最多”并不等于“保证”，网络读取可能返回更少的字节。
                                if (network_stream.Read(packet, 0, packet.Length) != 0)
                                {
                                    if (msg_length_list.Count == 10)
                                    {
                                        // 收集10个样本后，排序并取最大值作为标准消息长度
                                        msg_length_list.Sort();
                                        total_msg_length = msg_length_list[msg_length_list.Count - 1];
                                        state_id = 1;  // 切换到数据读取状态
                                    }
                                    else
                                    {
                                        // 添加消息长度到列表中
                                        msg_length_list.Add(BitConverter.ToUInt32(packet, first_packet_size - 4));
                                    }
                                }

                            }
                            break;

                        case 1:
                            {
                                // ==================== 数据包解包操作 ====================
                                // 从机器人TCP端口30013读取实时数据包
                                if (network_stream.Read(packet, 0, packet.Length) != 0)
                                {
                                    // 若有积压的数据，尽快读到最新一帧（丢弃旧帧，仅保留最后一帧来可视化）
                                    while (network_stream.DataAvailable)
                                    {
                                        int _ = network_stream.Read(packet, 0, packet.Length);
                                        if (_ == 0) break;
                                    }

                                    // 验证数据包完整性：检查消息长度是否匹配
                                    if (BitConverter.ToUInt32(packet, first_packet_size - 4) == total_msg_length)
                                    {
                                        // ==================== 字节序转换 ====================
                                        // UR机器人使用大端序(Big-Endian)，而Intel x86使用小端序(Little-Endian)
                                        // 需要反转整个数据包的字节顺序进行转换
                                        Array.Reverse(packet);

                                        // ==================== 数据解包：关节空间数据 ====================
                                        // 注意：偏移量32-37对应关节角度数据位置
                                        // 详细信息参考UR客户端接口文档(Client Interface)

                                        // 计算公式：packet.Length - first_packet_size - (索引 * offset)
                                        // 1116 - 4 - (32 * 8) = 856字节位置开始读取J1数据
                                        UR_Stream_Data.J_Orientation[0] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (32 * offset));  // J1关节角度
                                        UR_Stream_Data.J_Orientation[1] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (33 * offset));  // J2关节角度
                                        UR_Stream_Data.J_Orientation[2] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (34 * offset));  // J3关节角度
                                        UR_Stream_Data.J_Orientation[3] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (35 * offset));  // J4关节角度
                                        UR_Stream_Data.J_Orientation[4] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (36 * offset));  // J5关节角度
                                        UR_Stream_Data.J_Orientation[5] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (37 * offset));  // J6关节角度
                                        // ==================== 数据解包：笛卡尔空间位置数据 ====================
                                        // 偏移量56-58对应工具中心点(TCP)的笛卡尔坐标位置
                                        // 1116 - 4 - (56 * 8) = 664字节位置开始读取X坐标数据
                                        UR_Stream_Data.C_Position[0] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (56 * offset));    // X轴位置（米）
                                        UR_Stream_Data.C_Position[1] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (57 * offset));    // Y轴位置（米）
                                        UR_Stream_Data.C_Position[2] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (58 * offset));    // Z轴位置（米）

                                        // ==================== 数据解包：笛卡尔空间姿态数据 ====================
                                        // 偏移量59-61对应工具中心点(TCP)的姿态（旋转向量表示）
                                        UR_Stream_Data.C_Orientation[0] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (59 * offset)); // RX绕X轴旋转（弧度）
                                        UR_Stream_Data.C_Orientation[1] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (60 * offset)); // RY绕Y轴旋转（弧度）
                                        UR_Stream_Data.C_Orientation[2] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (61 * offset)); // RZ绕Z轴旋转（弧度）
                                    }
                                }
                            }
                            break;
                    }
                }
            }
            catch (SocketException e)
            {
                Debug.LogException(e);  // 输出Socket异常信息
            }
        }

        /// <summary>
        /// 启动数据流处理线程
        /// </summary>
        public void Start()
        {
            // 启动线程（双重否定表肯定）
            exit_thread = false;
            // 创建并启动监听数据的线程
            robot_thread = new Thread(new ThreadStart(UR_Stream_Thread));
            robot_thread.IsBackground = true;  // 设置为后台线程
            robot_thread.Start();
            // 标记线程为激活状态
            UR_Stream_Data.is_alive = true;
        }
        
        /// <summary>
        /// 停止数据流处理线程
        /// </summary>
        public void Stop()
        {
            exit_thread = true;         // 设置退出标志
            Thread.Sleep(100);          // 等待线程结束
            UR_Stream_Data.is_alive = robot_thread.IsAlive;  // 更新线程状态
            robot_thread.Abort();      // 强制终止线程
        }
        
        /// <summary>
        /// 销毁数据流处理实例，释放网络连接
        /// </summary>
        public void Destroy()
        {
            if (tcp_client.Connected == true)
            {
                // 断开通讯连接
                network_stream.Dispose();  // 释放网络流
                tcp_client.Close();        // 关闭TCP客户端
            }
            Thread.Sleep(100);             // 等待资源释放
        }
    }

    /// <summary>
    /// UR机器人控制类
    /// 负责向机器人发送控制命令
    /// </summary>
    class UR_Control
    {
        // 类变量初始化
        
        // 线程相关
        private Thread robot_thread = null;         // 机器人控制线程
        private bool exit_thread = false;           // 线程退出标志
        
        // TCP/IP通讯相关
        private TcpClient tcp_client = new TcpClient();     // TCP客户端
        private NetworkStream network_stream = null;        // 网络数据流

        /// <summary>
        /// UR机器人控制线程
        /// 持续向机器人发送控制命令
        /// </summary>
        public void UR_Control_Thread()
        {
            try
            {
                if (tcp_client.Connected != true)
                {
                    // 如果控制器未连接，则连接到控制器
                    tcp_client.Connect(UR_Control_Data.ip_address, UR_Control_Data.port_number);

                }

                // 初始化TCP/IP通讯数据流
                network_stream = tcp_client.GetStream();

                // 初始化计时器
                var t = new Stopwatch();

                while (exit_thread == false)
                {
                    // 开始计时
                    t.Start();

                    // 注意：
                    // 关于命令的详细信息，请参考URScript编程语言文档

                    // 当任一控制源请求发送（摇杆/按钮 或 手动脉冲）时，周期性下发当前命令
                    if (UR_Control_Data.joystick_button_pressed == true || UR_Control_Data.manual_send_active == true)
                    {
                        // 发送命令（字节格式）-> 机器人速度控制（X,Y,Z和欧拉角{RX, RY, RZ}）
                        network_stream.Write(UR_Control_Data.command, 0, UR_Control_Data.command.Length);
                    }

                    // 停止计时
                    t.Stop();

                    // 重新计算时间：t = t1 - t0 -> 经过的时间（毫秒）
                    if (t.ElapsedMilliseconds < UR_Stream_Data.time_step)
                    {
                        // 如果处理时间小于设定周期，则等待剩余时间
                        Thread.Sleep(UR_Stream_Data.time_step - (int)t.ElapsedMilliseconds);
                    }

                    // 重置（重启）计时器
                    t.Restart();
                }
            }
            catch (SocketException e)
            {
                Debug.LogException(e);  // 输出Socket异常信息
            }
        }

        /// <summary>
        /// 启动控制处理线程
        /// </summary>
        public void Start()
        {
            // 启动线程
            exit_thread = false;
            // 创建并启动控制命令发送线程
            robot_thread = new Thread(new ThreadStart(UR_Control_Thread));
            robot_thread.IsBackground = true;  // 设置为后台线程
            robot_thread.Start();
            // 标记线程为激活状态
            UR_Control_Data.is_alive = true;
        }
        
        /// <summary>
        /// 停止控制处理线程
        /// </summary>
        public void Stop()
        {
            exit_thread = true;         // 设置退出标志
            Thread.Sleep(100);          // 等待线程结束
            UR_Control_Data.is_alive = robot_thread.IsAlive;  // 更新线程状态
            robot_thread.Abort();      // 强制终止线程
        }
        
        /// <summary>
        /// 销毁控制处理实例，释放网络连接
        /// </summary>
        public void Destroy()
        {
            if (tcp_client.Connected == true)
            {
                // 断开通讯连接
                network_stream.Dispose();  // 释放网络流
                tcp_client.Close();        // 关闭TCP客户端
            }
            Thread.Sleep(100);             // 等待资源释放
        }
    }
}

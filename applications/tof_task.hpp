#pragma once

#include <stdint.h>

// GY-56 红外测距对外接口 (4.4.2.1)
//
// 实现: applications/tof_task.cpp
//       tof_handle_rx() / tof_handle_error() 由 uart_task.cpp 的串口回调(huart6 分支)调用,
//       因为 HAL_UARTEx_RxEventCallback / HAL_UART_ErrorCallback 全工程只能有一份定义。
//
// 使用: 其它任务只读下面的数据, 不要直接碰串口。

// ===== 总开关: 红外测距暂缓(2026/10/13 决定) =====
// false = 只编译、不接传感器、不挂 DMA: 悬空的 USART6_RX 不会产生噪声中断, 也不会有假数据。
// true  = 接线完成后改这里, 立即恢复测距(代码已就绪)。
// ⚠️ 注意: 规则书 4.4.2.1 与 7.3 把"测距读取"列为底盘动力调试前的前置项,
//    这里只是"暂时不做", 最终验收前必须改回 true 并接上 GY-56。
constexpr bool TOF_ENABLE = false;

// uart_task.cpp 的回调转发入口
void tof_handle_rx(uint16_t size);
void tof_handle_error();

// 测距数据(volatile: 中断里写, 任务里读)
extern volatile float tof_distance_cm;  // 最近一次有效距离, 单位 cm, <0 表示无效
extern volatile bool tof_valid;         // 当前数据是否有效(校验错/超量程/超时都会置 false)
extern volatile uint32_t tof_last_ms;   // 最近一次有效帧的时刻(ms)
extern volatile uint32_t tof_bad_sum;   // 校验和/帧头失败计数
extern volatile uint32_t tof_timeouts;  // 通信超时计数

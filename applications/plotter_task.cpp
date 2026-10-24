#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "referee/pm02/pm02.hpp"

// 波形走 USART1（丝印 UART2）921600，上位机用 SerialPlot。
// 一帧 = AA BB 08 + 2个float32 little-endian，共11字节，无附加校验。
// huart1 上只能挂一个 Plotter，两个任务同时发帧会交错成乱码。
//
// 通道1：PM02 0x0201 robot_status.chassis_power_limit，原始功率上限W，不减本地余量。
// 通道2：按用户指定，使用PM02 0x0202 power_heat.reserved_3作为实际功率。
// 当前协议库将第二项命名为保留字段，其物理含义仍需实机确认；不混用CAN功率计或预测值。
// 未收到PM02报文时字段保留初始化值0，不能据此判断通信有效；内部13通道RAM诊断保留。
extern sp::PM02 pm02;  // 定义在applications/uart_task.cpp

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (true) {
    //  必须确认上一帧已经发完(UART 回到 READY)再填下一帧:
    // Plotter 内部只有一份 plot_frame_ 缓冲, 而发送走 DMA —— 在 DMA 还在读这块内存时改写,
    // SerialPlot 上就会出现错位帧/花帧(库里的 send() 只是把 BUSY 记进 hal_status_, 那时缓冲已经被改坏了)。
    // huart1.gState 由 HAL 在 DMA 发送完成中断里恢复成 HAL_UART_STATE_READY。
    // 一帧11字节@921600约0.12ms；UART忙时跳过，不改DMA正在读取的帧。
    if (huart1.gState == HAL_UART_STATE_READY) {
      float power_limit_w;
      float actual_power_w;
      taskENTER_CRITICAL();
      power_limit_w = static_cast<float>(pm02.robot_status.chassis_power_limit);
      actual_power_w = pm02.power_heat.reserved_3;
      taskEXIT_CRITICAL();
      plotter.plot(power_limit_w, actual_power_w);
    }
    osDelay(1);
  }
}

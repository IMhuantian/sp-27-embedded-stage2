#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

// DT7 接收机接在 USART3 上(PC10=TX / PC11=RX)，100000bps / 偶校验 / 8 数据位 / 1 停止位，
// 这些都在 cboard.ioc 里配好了，接收走 DMA1_Stream1
// 注意: 本板(底盘 C 板)只接 DT7。GY-56 用的 USART6 在**云台 C 板**工程(..\files\stage_2_gimbal)里，
//       两块板各自读自己的接收机与传感器，不互相转发。
sp::DBus remote(&huart3);

extern "C" void uart_task()
{
  remote.request();

  while (true) {
    // remote 上的数据：ch_lh / ch_lv / ch_rh / ch_rv / ch_lu 是摇杆和拨轮，范围 [-1, 1]；
    // sw_l / sw_r 是左右三位开关；is_alive() 看接收机在不在线
    osDelay(10);
  }
}

// 串口空闲中断 + DMA 收满
//  全工程只有这一个定义（HAL 的回调是弱符号覆盖式的，不能有两个 .cpp 都实现）
//  DMA 半满(HT)事件也会进这个回调(HAL 没有关掉 HT 中断)，但半满不是"一批数据收完了"：
//      那时 DMA 还在往缓冲里写，解析到的是半帧；DBus 那边还会以错误长度提前返回。
//      所以 HT 事件直接忽略，只处理空闲(IDLE)与收满(TC) —— 两者都表示 DMA 已经停下来了。
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart->RxEventType == HAL_UART_RXEVENT_HT) return;  // 半满：不是一批数据收完

  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

// 收出错的时候重新挂 DMA
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) remote.request();
}

#include "cmsis_os.h"
#include "usart.h"
#include "dma.h"
#include "tof_task.hpp"  // 对外接口声明(与 uart_task.cpp 共用, 保证签名一致)

// GY-56 红外测距任务 (4.4.2.1)
// 接线: 自制降压板 5V → GY-56 VCC, GND 三端共地; GY-56 TX → C板 PG9(USART6_RX), RX → PG14(USART6_TX)
// CubeMX: USART6 波特率 9600 8N1(原 115200), RX DMA(DMA2_Stream2)已配好, 空闲中断收 9 字节帧
// 帧: 5A 5A 15 04 距离H 距离L mode 温度 校验和(前 8 字节求和取低 8 位), 距离单位 cm
// 注意: 串口空闲中断回调 HAL_UARTEx_RxEventCallback 只有一个(在 uart_task.cpp 里),
//       本文件只提供 tof_handle_rx()/tof_handle_error(), 由 uart_task.cpp 的 huart6 分支调用。

namespace
{
uint8_t tof_buf[9];

// 量程判据(超量程按无效处理): 高精度/快速/一般模式都是 1.2m;
// 若上电命令切到"长距离模式(mode 0, 量程 2m)", 把这里改成 200
constexpr uint16_t TOF_RANGE_CM = 120;
}

// 给其它任务看的全局量(以后折叠逻辑读 tof_distance_cm / tof_valid)
volatile float tof_distance_cm = -1.0f;  // 最近一次有效距离, <0 表示无效
volatile bool tof_valid = false;         // 数据当前是否有效
volatile uint32_t tof_last_ms = 0;       // 最近一次有效帧的时刻
volatile uint32_t tof_bad_sum = 0;       // 校验/帧头失败计数(验收演示"基本处理")
volatile uint32_t tof_timeouts = 0;      // 通信超时次数

static bool tof_parse(const uint8_t* b, float* cm, uint8_t* mode)
{
  if (b[0] != 0x5A || b[1] != 0x5A || b[2] != 0x15) return false;  // 帧头/帧类型错

  uint8_t sum = 0;
  for (int i = 0; i < 8; ++i) sum += b[i];
  if (sum != b[8]) return false;                                   // 校验和错

  const uint16_t d = (uint16_t)(b[4] << 8) | b[5];
  if (d == 0 || d > TOF_RANGE_CM) return false;   // 无效值/超量程(默认 1.2m = 120cm)

  *cm = (float)d;
  *mode = b[6];
  return true;
}

// uart_task.cpp 的空闲中断回调里调用(huart6 分支)
void tof_handle_rx(uint16_t size)
{
  (void)size;
  if (!TOF_ENABLE) return;  // 测距暂缓: 不解析也不重挂 DMA

  float cm;
  uint8_t mode;
  if (tof_parse(tof_buf, &cm, &mode)) {
    tof_distance_cm = cm;
    tof_valid = true;
    tof_last_ms = osKernelSysTick();
  } else {
    tof_bad_sum++;
  }

  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, tof_buf, sizeof(tof_buf));  // 重新挂 DMA
}

// uart_task.cpp 的错误回调里调用(huart6 分支)
void tof_handle_error()
{
  if (!TOF_ENABLE) return;
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, tof_buf, sizeof(tof_buf));
}

void tof_init()
{
  if (!TOF_ENABLE) return;  // 测距暂缓: 不挂 DMA, 悬空 RX 不产生噪声中断
  // 可选: 上电把测距模式切到"一般(35ms, ±2cm)"并保存, 注释掉则用出厂默认(高精度 200ms)
  // const uint8_t cmd_mode[] = {0xA5, 0x54, 0x03, 0xFC};  // 测量模式=3(一般)
  // const uint8_t cmd_save[] = {0xA5, 0x87, 0x01, 0x2D};  // 保存设置
  // HAL_UART_Transmit(&huart6, (uint8_t*)cmd_mode, 4, 10);
  // HAL_UART_Transmit(&huart6, (uint8_t*)cmd_save, 4, 10);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, tof_buf, sizeof(tof_buf));
}

void tof_update(uint32_t now_ms)
{
  // 通信异常: 超过 500ms 没有有效帧 → 数据判无效(规则书要求的基本处理)
  if (tof_valid && now_ms - tof_last_ms > 500) {
    tof_valid = false;
    tof_timeouts++;
  }
}

extern "C" void tof_task()
{
  osDelay(100);  // 等传感器上电稳定

  if (!TOF_ENABLE) {
    // 红外测距暂缓: 任务保留(接线即用), 但不碰串口、不占 CPU
    for (;;) osDelay(1000);
  }

  tof_init();

  while (true) {
    tof_update(osKernelSysTick());
    osDelay(10);
  }
}

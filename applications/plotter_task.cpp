#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

// 波形走 USART1（丝印 UART2）921600，上位机用 SerialPlot。
// 一帧 = AA BB + size + N 个 float，size = 通道数 x 4 = 0x30，共 3 + 48 = 51 字节。
// huart1 上只能挂一个 Plotter，两个任务同时发帧会交错成乱码。
//
// 通道(规则书 4.4.2.2 要求调 PID 时能看到 目标值/反馈值/控制量):
//   0 目标轮速(左前) rad/s
//   1 实际轮速(左前) rad/s
//   2 控制量(左前, 轮侧转矩) N·m
//   3/4/5 右前/左后/右后 实际轮速 rad/s   ← 直线行驶时看四轮是否一致
//   6 偏航 yaw rad                        ← 直线偏航锁定开关的前后对比
//   7 模型预测功率 W                       ← 功率控制(4.4.2.3)
//   8 功率计实测功率 W                     ← 验收要求"预测 vs 实际"
//   9/10/11 车体实测 vx/vy/wz              ← 验收要求"车体速度"; 直线行驶看 wz 是否≈0
// (红外测距/云台姿态等通道在**云台 C 板**工程 ..\files\stage_2_gimbal 的 plotter 里，本板不混用)
extern float plot_ch[12];  // 定义在 applications/can_task.cpp

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (true) {
    //  必须确认上一帧已经发完(UART 回到 READY)再填下一帧:
    // Plotter 内部只有一份 plot_frame_ 缓冲, 而发送走 DMA —— 在 DMA 还在读这块内存时改写,
    // SerialPlot 上就会出现错位帧/花帧(库里的 send() 只是把 BUSY 记进 hal_status_, 那时缓冲已经被改坏了)。
    // huart1.gState 由 HAL 在 DMA 发送完成中断里恢复成 HAL_UART_STATE_READY。
    // 一帧 = 51 B @921600 ≈ 0.55 ms, 正常 1 ms 周期够用; 万一上一帧还没发完, 本帧直接丢弃(不补发)。
    if (huart1.gState == HAL_UART_STATE_READY) {
      plotter.plot(
        plot_ch[0], plot_ch[1], plot_ch[2], plot_ch[3], plot_ch[4], plot_ch[5], plot_ch[6],
        plot_ch[7], plot_ch[8], plot_ch[9], plot_ch[10], plot_ch[11]);
    }
    osDelay(1);
  }
}

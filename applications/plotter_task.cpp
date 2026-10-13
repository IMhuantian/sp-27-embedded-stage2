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
extern float plot_ch[12];  // 定义在 applications/can_task.cpp

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (true) {
    plotter.plot(
      plot_ch[0], plot_ch[1], plot_ch[2], plot_ch[3], plot_ch[4], plot_ch[5], plot_ch[6], plot_ch[7],
      plot_ch[8], plot_ch[9], plot_ch[10], plot_ch[11]);
    osDelay(1);
  }
}

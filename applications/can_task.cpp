/*
 * 第二阶段: 底盘控制 + CAN 收发 (规则书 4.4.2.2 遥控器与底盘控制 / 4.4.2.3 功率控制)
 *
 * 硬件: 4×RM3508 + C620 电调, 接 CAN1(PD0/PD1), ID = 1/2/3/4(电调指示灯设定)
 *       功率计接 CAN2(0x301 两条总线都会收), 标准帧 8 字节
 *
 * 4.4.2.2 遥控映射(与本文件实现一一对应):
 *   右拨杆下档 → 全部电机失能(最高优先级, 每周期发 0 转矩)
 *   右拨杆中档 → 底盘可动
 *   左摇杆前后 → 前后移动                        (ch_lv)
 *   左摇杆左右 → 横向移动                        (ch_lh)
 *   左拨轮     → 原地旋转, 满量程目标角速度 1rad/s (ch_lu)
 *   右拨杆上档 → 本实现按失能处理(规则书未定义该档功能, 保守处理)
 *
 * 控制链: 摇杆死区 → 目标速度斜坡 → 麦克纳姆解算 → 每轮速度环 PID
 *         → 功率模型等比限流 → cmd(轮侧转矩 N·m)
 *         同时用四轮实测转速反解车体速度(vx/vy/wz), 与目标/反馈/控制量一起送 SerialPlot。
 *
 * ⚠️ 用法: 先按 CHASSIS_MOTION_ENABLE = false 编译烧录做 M2(纯失能, 交指定人员检查),
 *          检查通过后改成 true 再做 M3(运动, 车轮离地 + 低限幅)。
 */

#include <cmath>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"

namespace
{
// ===== M2/M3 总开关: false = 只保留失能逻辑(给安全检查烧录的版本), true = 允许运动 =====
constexpr bool CHASSIS_MOTION_ENABLE = true;

// ===== 电调 ID 自检模式: true = 只报 4 个 CAN ID 有没有反馈, 禁止一切运动 =====
// 用法: 改成 true 烧录 → SerialPlot 看通道 0/1/2/3 = 左前/右前/左后/右后是否在线(1=在线, 0=不在线)。
// 用途: 确认 C620 的 ID 到底设成了几号(见 指导文档\第二阶段-电调ID设置与确认.md)。
// 自检做完必须改回 false, 否则底盘永远不会动。
constexpr bool ID_CHECK_MODE = false;

// ---- 机械参数(规则书 4.4.2.2) ----
constexpr float WHEEL_R = 0.077f;  // 轮半径 m (轮径 154mm)
constexpr float HALF_L = 0.165f;   // 纵向半间距 m (纵向间距 330mm)
constexpr float HALF_W = 0.185f;   // 横向半间距 m (横向间距 370mm)

// ---- 减速比: 按导师确认的口径取 14.9:1 ----
// 作用: RM_Motor 用它把电机侧原始数据换算成输出轴量纲 ——
//   speed = rpm/60×2π/ratio (轮侧 rad/s)、torque = current×Kt×ratio (轮侧 N·m)、cmd() 也是轮侧 N·m。
// 后果: 20A 对应的轮侧转矩 = (0.3/19.203)×20×14.9 ≈ 4.66 N·m (用 19.203 时是 6 N·m)。
// 若哪天实测发现是 19.203:1, 只改这一行即可(读数会整体差 1.29 倍)。
// ⚠️ 库里的 sp::M3508_P19(≈19.203) 不要用于这里。
constexpr float GEAR_RATIO = 14.9f;

// ---- 速度限幅 ----
constexpr float V_MAX = 1.0f;   // 车体线速度上限 m/s (首测 1.0, 稳定后逐步放)
constexpr float WZ_MAX = 1.0f;  // 旋转角速度上限 rad/s (规则书: 拨轮满量程 = 1)

// ---- 摇杆死区(4.4.2.2 "方向应与操作者输入一致": 没死区时摇杆回中抖动会让底盘自己爬) ----
// DT7 回中抖动实测在 ±0.01 量级; 死区内输出 0, 死区外线性重映射(过零连续, 满量程仍是 ±1)
constexpr float STICK_DEADZONE = 0.02f;

// ---- 目标速度斜坡(4.4.2.2 "直线行驶时应尽量减小偏航、振荡"; 同时充当软启动) ----
constexpr float V_SLEW = 3.0f;  // 车体线速度变化率上限 m/s² (0 → 1m/s 约 0.33s)
constexpr float W_SLEW = 4.0f;  // 角速度变化率上限 rad/s²

// ---- 方向标定(试车发现反了就把对应项改 -1, 不要动机械) ----
// 这三项按"遥控器通道极性 × sp::Mecanum 约定"推出来, 不是猜的:
//   dbus.hpp:70-72  ch_lv 上正下负 | ch_lh 右正左负 | ch_lu 左正右负
//   mecanum.hpp:32-34  vx 前进为正 | vy 左移为正 | wz 逆时针(右手螺旋)为正
// 所以"推杆方向 = 车动方向"要求: vx=+ch_lv, vy=-ch_lh, wz=+ch_lu
constexpr float FWD_DIR = +1.0f;   // ch_lv 上推(+) = 前进
constexpr float SIDE_DIR = -1.0f;  // ch_lh 右推(+) 要得到"右移", 而 vy 正方向是左移 → 取负
constexpr float ROT_DIR = +1.0f;   // ch_lu 左推(+) = 左转, 而 wz 正方向就是逆时针(左转) → 取正

// ---- 每轮速度环, 输出 = 轮侧转矩 N·m ----
constexpr float SPD_DT = 1e-3f;      // 与控制周期 osDelay(1) 一致
constexpr float SPD_KP = 1.0f;       // 整定起点, 见指南 M4
constexpr float SPD_KI = 0.2f;
constexpr float SPD_KD = 0.0f;
constexpr float SPD_MAX_OUT = 1.0f;  // 单轮转矩限幅 N·m (总输出限幅), 首测 1.0, 稳定后放 3.0
constexpr float SPD_MAX_IOUT = 0.5f; // 积分分项限幅(防积分饱和)
constexpr float SPD_ALPHA = 0.8f;    // 反馈低通

// ---- M4: 直线行驶的偏航锁定(可选). 建议先用 false 把基本运动跑通, 再开来对比效果 ----
constexpr bool STRAIGHT_LOCK_ENABLE = false;
constexpr float YAW_KP = 0.6f;         // 偏航修正 P: 输出 rad/s / 偏差 rad
constexpr float YAW_KD = 0.05f;        // 用 imu.vyaw 做阻尼
constexpr float YAW_MAX = 0.25f;       // 偏航修正的上限 rad/s, 不允许抢主控制
constexpr float STRAIGHT_EPS = 0.10f;  // |ch_lh|、|ch_lu| 都小于它才算"在走直线"
// ⚠️ 符号标定(开启偏航锁定前必做): 失能状态下把车放平、俯视逆时针转 ~90°。
//    若 imu.yaw 变大 → 与麦克纳姆 wz(逆时针为正)同号 → 保持 +1;
//    若 imu.yaw 变小(例如 C 板倒装/绕 z 轴反向) → 改成 -1, 否则闭环变正反馈, 车会自己打转。
constexpr float YAW_SIGN = +1.0f;

// ---- 功率模型(4.4.2.3, 整定见指南 M5) ----
// K1/K2 必须与喂进来的 τ、ω 同口径: 本文件用的是轮侧 τ(N·m)、轮侧 ω(rad/s),
// 与参考资料《底盘功率控制》的口径一致(主项 Στ·ω 与取哪一侧无关; 若哪天改用电机侧量纲,
// 修正项系数要按 i² 重算: K1/i²、K2×i²)。下面只是参考初值, 必须按 M5 流程自己整定。
constexpr float POWER_LIMIT_W = 60.0f;  // ⚠️ 占位! 实际上限值必须问导师
constexpr float K1 = 2.0f;              // Στ² 系数(轮侧口径初值)
constexpr float K2 = 0.01f;             // Σω² 系数(轮侧口径初值)
constexpr float K3 = 2.0f;              // 静态功耗 W(静止实测整定)
}  // namespace

// ---- 底盘 4×C620 接在哪条 CAN 上（**接线必须与这里一致**）----
// CAN1 = C 板上 2-pin 口（黑 CANL / 红 CANH）；CAN2 = 4-pin 口（灰 CANL / 灰 CANH / 灰 GND / 红 5V）
// 已确认(2026/10/13): 底盘电机接 **CAN1**, 功率计接 CAN2。
// 注意: 代码对两条总线都 config()+start() 且都解析 0x301, 所以功率计插错口也不会瞎。
// 另一条总线同时留给以后云台的 GM6020。
constexpr bool MOTORS_ON_CAN1 = true;

sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);
sp::CAN & mcan = MOTORS_ON_CAN1 ? can1 : can2;  // 底盘电机所在总线: 发 0x200 / 收 0x201~0x204
sp::CAN & ocan = MOTORS_ON_CAN1 ? can2 : can1;  // 另一条: 只收 0x301(功率计)

// 第三个参数 = 减速比, 必须显式传 GEAR_RATIO:
// 默认值 1.0f 会让 speed/torque 变成"电机侧"量纲, 且 cmd() 的量程会差一个减速比。
// 2026-10-14: 用户按 C620 绿灯闪烁次数确认实物位置。
sp::RM_Motor wheel_lf(4, sp::RM_Motors::M3508, GEAR_RATIO);
sp::RM_Motor wheel_rf(3, sp::RM_Motors::M3508, GEAR_RATIO);
sp::RM_Motor wheel_lr(1, sp::RM_Motors::M3508, GEAR_RATIO);
sp::RM_Motor wheel_rr(2, sp::RM_Motors::M3508, GEAR_RATIO);

// 默认右轮按"反装"解算(reverse_rf/rr = true), 试车发现某轮转向不对就把对应 bool 翻过来
sp::Mecanum mec(WHEEL_R, HALF_L, HALF_W);

sp::PID spd_lf(SPD_DT, SPD_KP, SPD_KI, SPD_KD, SPD_MAX_OUT, SPD_MAX_IOUT, SPD_ALPHA);
sp::PID spd_rf(SPD_DT, SPD_KP, SPD_KI, SPD_KD, SPD_MAX_OUT, SPD_MAX_IOUT, SPD_ALPHA);
sp::PID spd_lr(SPD_DT, SPD_KP, SPD_KI, SPD_KD, SPD_MAX_OUT, SPD_MAX_IOUT, SPD_ALPHA);
sp::PID spd_rr(SPD_DT, SPD_KP, SPD_KI, SPD_KD, SPD_MAX_OUT, SPD_MAX_IOUT, SPD_ALPHA);

extern sp::DBus remote;  // applications/uart_task.cpp
extern sp::Mahony imu;   // applications/imu_task.cpp

// 功率计(0x301): data[0..1]=电流×100, [2..3]=电压×100, [4..5]=功率×100, [6..7]=0
float pm_current = 0.0f;  // A
float pm_voltage = 0.0f;  // V
float pm_power = 0.0f;    // W
float p_pred = 0.0f;      // 模型预测功率 W
float k_power = 1.0f;     // 当前功率限流系数

// 目标速度斜坡状态(失能时清零 → 每次恢复运动都从 0 平滑起步, 不会突然窜出去)
float vx_ramp = 0.0f;
float vy_ramp = 0.0f;
float wz_ramp = 0.0f;

// 给 plotter_task 的 12 通道(SerialPlot 一帧 = 3 + 12×4 = 51 字节, 921600 下 1ms 周期够用):
// 0 目标轮速(左前) 1 实际轮速(左前) 2 控制量(左前) 3/4/5 右前/左后/右后实际轮速
// 6 偏航 yaw 7 预测功率 8 功率计实测功率
// 9/10/11 车体实测 vx/vy/wz(由四轮反馈反解) ← 规则书"车体速度"与直线行驶证据
float plot_ch[12] = {0};

// 直线偏航锁定状态
bool yaw_lock_active = false;
float yaw_set = 0.0f;

namespace
{
// 失能: 每周期向 4 台电机发 0 转矩指令, 清 PID 积分, 复位限流系数。
// 失能状态具有最高优先级, 任何其它代码路径不得恢复输出(规则书 4.4.2.2)。
void disable_all()
{
  wheel_lf.cmd(0.0f);
  wheel_rf.cmd(0.0f);
  wheel_lr.cmd(0.0f);
  wheel_rr.cmd(0.0f);
  spd_lf.clear();
  spd_rf.clear();
  spd_lr.clear();
  spd_rr.clear();
  k_power = 1.0f;
  yaw_lock_active = false;  // 下次恢复运动时重新锁存偏航零点
  vx_ramp = 0.0f;           // 斜坡状态清零: 下次恢复运动从 0 起步(软启动)
  vy_ramp = 0.0f;
  wz_ramp = 0.0f;
  // 让 SerialPlot 上直接看出"已失能": 目标/控制量归零(实际轮速仍显示真实值)
  plot_ch[0] = 0.0f;
  plot_ch[2] = 0.0f;
  p_pred = 0.0f;
}

// 摇杆死区 + 线性重映射: 死区内归零, 死区外保证过零连续且推满时仍输出 ±1(不损失满量程)
float stick(float v, float dz)
{
  const float a = fabsf(v);
  if (a <= dz) return 0.0f;
  return (v > 0.0f ? 1.0f : -1.0f) * (a - dz) / (1.0f - dz);
}

// 一阶斜坡限幅: 每个控制周期最多变化 rate×dt (防阶跃冲击与振荡, 兼作软启动)
float slew(float now, float target, float rate, float dt)
{
  const float lim = rate * dt;
  const float d = target - now;
  if (d > lim) return now + lim;
  if (d < -lim) return now - lim;
  return target;
}

void parse_power(const uint8_t * d)
{
  pm_current = (float)(int16_t)((d[0] << 8) | d[1]) / 100.0f;   // A
  pm_voltage = (float)(uint16_t)((d[2] << 8) | d[3]) / 100.0f;  // V
  pm_power = (float)(uint16_t)((d[4] << 8) | d[5]) / 100.0f;    // W
}

// 预测功率超限时把 4 轮转矩等比缩小 K'。
// P = Σ(τ·ω) + K1·Στ² + K2·Σω² + K3, 令 τ' = K'·τ 代入 P = Pmax:
//   K1·Στ²·K'² + Σ(τ·ω)·K' + (K2·Σω² + K3 − Pmax) = 0, 取正根。
float power_scale(float t1, float w1, float t2, float w2, float t3, float w3, float t4, float w4)
{
  const float sum_tw = t1 * w1 + t2 * w2 + t3 * w3 + t4 * w4;
  const float sum_t2 = t1 * t1 + t2 * t2 + t3 * t3 + t4 * t4;
  const float sum_w2 = w1 * w1 + w2 * w2 + w3 * w3 + w4 * w4;
  p_pred = sum_tw + K1 * sum_t2 + K2 * sum_w2 + K3;
  if (p_pred <= POWER_LIMIT_W) return 1.0f;

  const float a = K1 * sum_t2;
  const float b = sum_tw;
  const float c = K2 * sum_w2 + K3 - POWER_LIMIT_W;
  const float disc = b * b - 4.0f * a * c;
  // 兜底: 判别式<0 只可能发生在 c>0(即"零转矩下预测功率都已超限")——此时 K'∈[0,1] 内
  // 功率随 K' 单调上升, 最小值在 K'=0, 所以直接给 0 转矩最安全(而不是给一个固定系数)
  if (a <= 0.0f || disc < 0.0f) return 0.0f;

  float k = (-b + sqrtf(disc)) / (2.0f * a);
  if (k > 1.0f) k = 1.0f;
  if (k < 0.0f) k = 0.0f;
  return k;
}

void chassis_step()
{
  const uint32_t now = osKernelSysTick();

  // ---- 电调 ID 自检(诊断模式, 优先于一切): 只报告 4 个 CAN ID 是否在线, 绝不运动 ----
  // 必须放在"三层失能"之前: ID 设错时那台电机本来就没有反馈, 若先判失能就永远看不到结果。
  if (ID_CHECK_MODE) {
    disable_all();
    const bool t = (now >= 2000u);                         // 上电 2s 后再判定(之前一律 0)
    plot_ch[0] = (t && wheel_lf.is_open()) ? 1.0f : 0.0f;  // CAN ID 4 (左前)
    plot_ch[1] = (t && wheel_rf.is_open()) ? 1.0f : 0.0f;  // CAN ID 3 (右前)
    plot_ch[2] = (t && wheel_lr.is_open()) ? 1.0f : 0.0f;  // CAN ID 1 (左后)
    plot_ch[3] = (t && wheel_rr.is_open()) ? 1.0f : 0.0f;  // CAN ID 2 (右后)
    return;
  }

  // ---- 三层失能(最高优先级): 电机反馈中断 / 遥控器失联 / 右拨杆下档 ----
  // is_alive() = 距上次收到该电机反馈 < 100ms(见 rm_motor.cpp:103), 比 is_open() 更严:
  // 运行中拔掉/压断某台 C620 的通信线, 会立刻整机进入失能(单轮速度环失去反馈会失控)。
  if (
    !wheel_lf.is_alive(now) || !wheel_rf.is_alive(now) || !wheel_lr.is_alive(now) ||
    !wheel_rr.is_alive(now) || !remote.is_alive(now) || remote.sw_r == sp::DBusSwitchMode::DOWN) {
    disable_all();
    return;
  }

  // ---- M2 阶段(CHASSIS_MOTION_ENABLE = false)到此为止: 永远失能, 不做任何运动 ----
  // ---- M3 阶段改成 true 后: 中档可动, 上档暂未定义(只发 0) ----
  if (!CHASSIS_MOTION_ENABLE || remote.sw_r != sp::DBusSwitchMode::MID) {
    disable_all();
    return;
  }

  // ---- 摇杆 → 车体目标速度(死区 → 方向标定 → 限幅) ----
  const float vx_in = FWD_DIR * stick(remote.ch_lv, STICK_DEADZONE) * V_MAX;
  const float vy_in = SIDE_DIR * stick(remote.ch_lh, STICK_DEADZONE) * V_MAX;
  const float wz_in = ROT_DIR * stick(remote.ch_lu, STICK_DEADZONE) * WZ_MAX;

  // 目标速度斜坡: 抑制阶跃冲击、减小振荡, 并让每次恢复运动都是平滑起步
  vx_ramp = slew(vx_ramp, vx_in, V_SLEW, SPD_DT);
  vy_ramp = slew(vy_ramp, vy_in, V_SLEW, SPD_DT);
  wz_ramp = slew(wz_ramp, wz_in, W_SLEW, SPD_DT);

  float wz = wz_ramp;

  // 直线偏航锁定(可选): 只推前后、没有横移和旋转时, 用偏航闭环把方向拽回锁存零点。
  // 偏差用 limit_angle 处理 ±π 跳变; 修正量限幅, 不允许盖过手动指令。
  if (
    STRAIGHT_LOCK_ENABLE && fabsf(remote.ch_lh) < STRAIGHT_EPS &&
    fabsf(remote.ch_lu) < STRAIGHT_EPS) {
    if (!yaw_lock_active) {
      yaw_set = imu.yaw;  // 进入直线状态, 锁存当前偏航为零点
      yaw_lock_active = true;
    }
    const float yaw_err = sp::limit_angle(yaw_set - imu.yaw);
    const float corr = YAW_SIGN * (YAW_KP * yaw_err - YAW_KD * imu.vyaw);
    wz += sp::limit_max(corr, YAW_MAX);
  }
  else {
    yaw_lock_active = false;
  }

  mec.calc(vx_ramp, vy_ramp, wz);  // → 四轮目标转速 rad/s

  spd_lf.calc(mec.speed_lf, wheel_lf.speed);
  spd_rf.calc(mec.speed_rf, wheel_rf.speed);
  spd_lr.calc(mec.speed_lr, wheel_lr.speed);
  spd_rr.calc(mec.speed_rr, wheel_rr.speed);

  // 功率限流(4.4.2.3): 超限时四轮转矩等比缩小
  k_power = power_scale(
    spd_lf.out, wheel_lf.speed, spd_rf.out, wheel_rf.speed, spd_lr.out, wheel_lr.speed, spd_rr.out,
    wheel_rr.speed);
  wheel_lf.cmd(k_power * spd_lf.out);
  wheel_rf.cmd(k_power * spd_rf.out);
  wheel_lr.cmd(k_power * spd_lr.out);
  wheel_rr.cmd(k_power * spd_rr.out);

  // 车体速度反解(验收要展示"车体速度"; 也是直线行驶偏航漂移/速度不一致的客观证据)
  // ⚠️ 形参顺序是 (lf, lr, rf, rr) —— lr 在 rf 前面, 别按 lf/rf/lr/rr 写
  mec.update(wheel_lf.speed, wheel_lr.speed, wheel_rf.speed, wheel_rr.speed);

  plot_ch[0] = mec.speed_lf;          // 目标轮速(左前) rad/s
  plot_ch[1] = wheel_lf.speed;        // 实际轮速(左前) rad/s
  plot_ch[2] = k_power * spd_lf.out;  // 控制量(轮侧转矩) N·m
  plot_ch[3] = wheel_rf.speed;        // 实际轮速(右前) rad/s
  plot_ch[4] = wheel_lr.speed;        // 实际轮速(左后) rad/s
  plot_ch[5] = wheel_rr.speed;        // 实际轮速(右后) rad/s
  plot_ch[6] = imu.yaw;               // 偏航 rad
  plot_ch[7] = p_pred;                // 预测功率 W
  plot_ch[8] = pm_power;              // 功率计实测 W
  plot_ch[9] = mec.vx;                // 车体实测 vx m/s (前进为正)
  plot_ch[10] = mec.vy;               // 车体实测 vy m/s (左移为正)
  plot_ch[11] = mec.wz;               // 车体实测 wz rad/s (逆时针为正)
}
}  // namespace

extern "C" void can_task()
{
  osDelay(500);  // 等电调上电稳定

  can1.config();
  can1.start();
  can2.config();
  can2.start();

  while (true) {
    chassis_step();

    // 四台电机控制帧 ID 都是 0x200, 合成一帧只发一次
    wheel_lf.write(mcan.tx_data);
    wheel_rf.write(mcan.tx_data);
    wheel_lr.write(mcan.tx_data);
    wheel_rr.write(mcan.tx_data);
    mcan.send(wheel_lf.tx_id);

    osDelay(1);  // 1ms 控制周期
  }
}

// 四个电机反馈帧(0x201~0x204)分发; d = 刚 recv() 出来的 mcan.rx_data
static void dispatch_motors(uint8_t * d, uint32_t stamp_ms)
{
  const uint32_t id = mcan.rx_id;
  if (id == wheel_lf.rx_id) {
    wheel_lf.read(d, stamp_ms);
  }
  else if (id == wheel_rf.rx_id) {
    wheel_rf.read(d, stamp_ms);
  }
  else if (id == wheel_lr.rx_id) {
    wheel_lr.read(d, stamp_ms);
  }
  else if (id == wheel_rr.rx_id) {
    wheel_rr.read(d, stamp_ms);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  const uint32_t stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();
      if (can1.rx_id == 0x301) {
        parse_power(can1.rx_data);  // 功率计(标准帧), 接哪条都能收
      }
      else if (MOTORS_ON_CAN1) {
        dispatch_motors(can1.rx_data, stamp_ms);
      }
    }
    else if (hcan == &hcan2) {
      can2.recv();
      if (can2.rx_id == 0x301) {
        parse_power(can2.rx_data);
      }
      else if (!MOTORS_ON_CAN1) {
        dispatch_motors(can2.rx_data, stamp_ms);
      }
    }
  }
}

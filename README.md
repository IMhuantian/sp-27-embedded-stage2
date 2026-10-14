# cboard

RoboMaster 开发板 C 型（STM32F407IGHx）上的嵌入式工程。
当前进度：**第二阶段 · 底盘**（云台尚未到位，相关功能未实现）。

## 功能

- 蜂鸣器提示音
- RGB 流水灯（也能用来判断程序有没有堵塞）
- IMU(BMI088) 采集 + Mahony 姿态解算，三轴姿态角（roll / pitch / yaw）经 SerialPlot 输出
  （USART1 / 921600 / 帧头 `AA BB`）
- DT7 遥控器通讯
- GY-56 红外测距读取（USART6 / 9600 / DMA + 空闲中断），对无效值、超量程、校验错和通信超时做了处理
  ⏸️ **当前暂缓**（2026/10/13）：`tof_task.hpp` 里 `TOF_ENABLE = false`，代码保留但不接传感器、不挂 DMA
- 麦克纳姆底盘 4×RM3508 + C620：遥控输入 → 死区 → 目标速度斜坡 → 麦轮解算 → 每轮速度环 PID → 功率限流 → 转矩指令
- 车体实测速度：每周期用 `mec.update(四轮反馈)` 反解出 vx / vy / wz（轨迹跟踪阶段要用）
- 功率计（CAN 标准帧 `0x301`）解析 + 底盘预测功率模型限流，预测功率/实测功率一起送 SerialPlot

## 任务

| 任务 | 优先级 | 栈(字) | 入口 | 干什么 |
|---|---|---|---|---|
| defaultTask | Normal | 128 | StartDefaultTask | USB 初始化 |
| ImuTask | AboveNormal | 512 | imu_task | 1kHz 采 IMU 并解算 |
| UartTask | Normal | 256 | uart_task | DBus 接收 + 串口回调转发（USART3 / USART6） |
| TofTask | Normal | 256 | tof_task | 10ms 巡检 GY-56、超时判无效（`TOF_ENABLE=false` 时只睡眠，不碰串口） |
| PlotterTask | Low | 128 | plotter_task | 1kHz 串口打印 12 通道（轮速/控制量/yaw/功率/车体速度） |
| CanTask | Low | 256 | can_task | 底盘控制 + CAN 收发 |
| LedTask | Low | 128 | led_task | 流水灯 |
| BuzzerTask | Low | 128 | buzzer_task | 提示音 |

在 CubeMX 里加任务的时候，Code Generation Option 要选 `As external`，这样函数体留在 `applications/`
下面，CubeMX 只生成 extern 声明和创建语句（`Src/freertos.c` 里对应 `osThreadDef` / `osThreadCreate`）。

## 硬件接口

C 板外壳丝印和 CubeMX 里的外设名**是错位的**，接线前先看这张表：

| 板上接口 | 实际外设 | 本工程用途 | 备注 |
|---|---|---|---|
| 3-pin UART，丝印 `UART1` | USART6 | **GY-56 测距（当前暂缓）** | Pin1 GND / Pin2 TXD(PG14) / Pin3 RXD(PG9)；**这个口不引出 5V**，测距模块的 5V 要从降压板取。接线后：CubeMX 把 USART6 波特率改 9600，并把 `tof_task.hpp` 的 `TOF_ENABLE` 改 `true` |
| 4-pin UART，丝印 `UART2` | USART1 | SerialPlot | Pin1 RXD(PB7) / Pin2 TXD(PA9) / Pin3 GND / Pin4 5V |
| DBUS 口（与 PWM 共用） | USART3 | DT7 接收机 | 100000bps / 偶校验，经反相电路 |
| CAN1（2-pin，黑 CANL / 红 CANH） | CAN1 | 4×C620 电调 | 电调自带动力电；ID 用指示灯设成 1/2/3/4 |
| CAN2（4-pin，灰 CANL/CANH/GND + 红 5V） | CAN2 | 功率计 | 功率计插哪条 CAN 都能收（代码两条总线都监听 `0x301`） |

## 底盘控制（规则书 4.4.2.2）

遥控映射：

| 输入 | 作用 |
|---|---|
| 右拨杆下档 | **全部电机失能**（最高优先级） |
| 右拨杆中档 | 底盘可动 |
| 右拨杆上档 | 未定义，按失能处理（只发 0） |
| 左摇杆前后 `ch_lv` | 前后移动 |
| 左摇杆左右 `ch_lh` | 横向移动 |
| 左拨轮 `ch_lu` | 旋转，满量程目标角速度 1 rad/s |

摇杆先过**死区** `STICK_DEADZONE = 0.02`（DT7 回中抖动在 ±0.01 量级，没有死区松手后底盘会自己慢慢爬），死区外做线性重映射（过零连续、满量程仍是 ±1）；再过**目标速度斜坡** `V_SLEW = 3.0 m/s²`、`W_SLEW = 4.0 rad/s²`，既作软启动（0→1 m/s 约 0.33 s），也直接对应规则书"减小振荡"。

失能有**三层**，任何一层成立都在本周期向 4 台电机发 0 转矩、清 PID 积分、复位限流系数，
并且只有这一条路径能决定输出：

1. 右拨杆下档
2. 遥控器失联（`remote.is_alive()`）
3. 任一台电机反馈中断（`is_alive()` = 距上次反馈 < 100ms）

控制链路：摇杆 → `stick()` 死区映射 → `slew()` 速率限幅（`vx_ramp / vy_ramp / wz_ramp`）→ `mec.calc(vx_ramp, vy_ramp, wz_ramp)` → 四轮目标转速 → 每轮速度环 PID（输出为**轮侧转矩 N·m**，总输出和各分项都有限幅）→ 功率限流系数 `K'` → `wheel.cmd(K' * pid.out)`。

反解：每周期还调用 `mec.update(wheel_lf.speed, wheel_lr.speed, wheel_rf.speed, wheel_rr.speed)` 得到车体实测 `mec.vx / mec.vy / mec.wz`（送 SerialPlot 通道 9/10/11）。⚠️ `update()` 的形参顺序是 `(lf, lr, rf, rr)`——`lr` 在 `rf` 前面。

车体与轮速的换算（`sp::Mecanum`，`WHEEL_R=0.077m`、`HALF_L=0.165m`、`HALF_W=0.185m`）：

```
lf = (vx - vy - (HALF_L+HALF_W)·wz) / R      lr = (vx + vy - (HALF_L+HALF_W)·wz) / R
rf = (vx + vy + (HALF_L+HALF_W)·wz) / R      rr = (vx - vy + (HALF_L+HALF_W)·wz) / R
```

`sp::Mecanum` 默认按"右轮反装"（`reverse_rf/rr = true`）解算。**试车方向标定**不对时：

- 单个轮子转向反了 → 改 `Mecanum` 构造里对应的 `reverse_xx`
- 四个轮子一起反（前后反）→ `FWD_DIR` 取反；横移反 → `SIDE_DIR` 取反；旋转反 → `ROT_DIR` 取反
  （当前 `FWD_DIR=+1`、`SIDE_DIR=-1`、`ROT_DIR=+1` 是按"遥控通道极性 × `sp::Mecanum` 的 `vx` 前进/`vy` 左移/`wz` 逆时针为正"推导出来的，正常不需要动）

`RM_Motor` 的**第三个构造参数必须显式传减速比 `GEAR_RATIO`**（`can_task.cpp:56`，本车 = **14.9**，
导师确认 2026/10/13）。传默认的 `1.0f` 会让 `angle/speed/torque` 全变成电机侧量纲，
而且 `cmd()` 的量程会差一个减速比。14.9:1 下满量程轮侧转矩 ≈ **4.66 N·m**（20 A），
`SPD_MAX_OUT` 就是在这个量纲上做限幅。若实测发现真实总减速比是 19.203，把 `GEAR_RATIO`
改成 `sp::M3508_P19` 即可（一行）。

### 编译开关（`applications/can_task.cpp` 顶部）

| 开关 | 当前值 | 说明 |
|---|---|---|
| `CHASSIS_MOTION_ENABLE` | `true` | 运动总开关。**做 M2 失能检查时先临时改回 `false`**（那才是纯失能版），检查通过后再改 `true` 测运动 |
| `ID_CHECK_MODE` | `false` | 电调 ID 自检：置 `true` 时 `plot_ch[0..3]` = 左前4/右前3/左后1/右后2 是否在线（1=在线），只发 0 转矩 |
| `MOTORS_ON_CAN1` | `true` | 4 台 C620 接哪条 CAN；另一条自动留给功率计和以后的云台 |
| `STRAIGHT_LOCK_ENABLE` | `false` | 直线行驶偏航锁定（可选）。开启前必须先标定 yaw 轴与符号，见下方注释与 `YAW_SIGN` |

### 直线行驶偏航锁定（可选）

只推前后（`|ch_lh|`、`|ch_lu|` 都很小）时，把当前 `imu.yaw` 锁存为零点，用
`corr = YAW_SIGN · (YAW_KP·yaw_err − YAW_KD·imu.vyaw)` 修正 `wz`，修正量限幅 `YAW_MAX`，
不允许盖过手动指令。偏差用 `sp::limit_angle()` 处理 ±π 跳变；失能时清除锁存，
恢复运动后重新锁零点。

开启前两件事必须做：① 确认 `imu.yaw` 是**竖直轴**（转动时如果只有 roll/pitch 在变，说明
`imu_task.cpp` 里的 `r_ab` 与实际装法不匹配）；② 俯视逆时针转车时 `imu.yaw` 变大才保持
`YAW_SIGN = +1`，否则闭环变正反馈，车会自己打转。

## 功率控制（规则书 4.4.2.3）

预测功率模型（四项：轴功 / 转矩与加减速附加损耗 / 角速度损耗 / 静态待机）：

```
P = Σ(τ·ω) + K1·Στ² + K2·Σω² + K3
```

这里 `τ` 是**轮侧**转矩（速度环输出）、`ω` 是**轮侧**转速（`motor.speed`），与参考资料
《底盘功率控制》同口径（主项 `Στ·ω` 与取哪一侧无关）。但参考值只是起点：**K1/K2 必须自己整定**
（若改用电机侧量纲，修正项要按 i² 重算：`K1/i²`、`K2×i²`；本车 i = 14.9 → i² ≈ 222）。

超限时令 `τ' = K'·τ` 代入 `P = Pmax`，解一元二次方程取正根：

```
K1·Στ²·K'² + Σ(τ·ω)·K' + (K2·Σω² + K3 − Pmax) = 0
```

判别式 < 0 只可能发生在"零转矩下预测功率都已超限"（`c > 0`），此时 `K' = 0` 最安全。
整定顺序：静止先定 `K3` → `K1 = 2.0`、`K2 = 0.01` 起调 → 加减速调 `K1` → 原地旋转调 `K2`；
实际限幅取"上限 − 2~5W"，保证不会持续（>0.5s）超过功率上限。

功率计帧（CAN 标准帧，反馈 ID `0x301`，8 字节）：

| 字节 | 含义 |
|---|---|
| `data[0..1]` | 电流 × 100（有符号，高字节在前） |
| `data[2..3]` | 电压 × 100 |
| `data[4..5]` | 功率 × 100 |
| `data[6..7]` | 0 |

裁判系统下发时，底盘功率上限在 `pm02.robot_status.chassis_power_limit`（`0x0201` 机器人状态帧，
单位 W）；本工程目前用编译期常数 `POWER_LIMIT_W`（占位 60W），**上限值需要向导师确认**。

## 红外测距（规则书 4.4.2.1）

GY-56（VL53L0X）出厂为 UART 9600 8N1、高精度模式（200ms）连续输出，帧 9 字节：

```
5A 5A 15 04 距离高 距离低 mode 温度 校验和     校验和 = 前 8 字节之和的低 8 位，距离单位 cm
```

`applications/tof_task.cpp` 的处理：帧头错 / 校验和错 / 距离为 0 / 超量程（默认 >120cm，
若用命令切到长距离模式则改 `TOF_RANGE_CM`）都判无效并计数；超过 500ms 没有有效帧判通信超时。
对外只暴露 `tof_distance_cm` / `tof_valid` / `tof_last_ms` 和三个计数。

接收用 DMA2_Stream2 + 空闲中断（`HAL_UARTEx_ReceiveToIdle_DMA`）。注意
`HAL_UARTEx_RxEventCallback` / `HAL_UART_ErrorCallback` **全工程只能有一份定义**，
所以 huart6 的分支写在 `applications/uart_task.cpp` 里，`tof_task.cpp` 只提供
`tof_handle_rx()` / `tof_handle_error()`。

## 注意事项

安全方面：

1. 动电机之前，先确认 DT7 右拨杆下档能让所有电机可靠失能（失能逻辑要经指定人员检查通过才测运动）
2. 烧录、接线、拆装、人工转机构之前，把动力电源断掉
3. 改完代码第一次测试，把机构固定住或者让轮子离地，输出限幅先给低一点
4. 出现失控、异响、异味、过热、冒烟、线束被拉扯、结构松动，立刻失能断电

## 已确认 / 待确认

已确认（导师口径，2026/10/13）：

- **减速比 = 14.9:1** → 代码 `GEAR_RATIO = 14.9f`（`applications/can_task.cpp:56`），不使用 `sp::M3508_P19`
- **底盘 4×C620 接 CAN1**（`MOTORS_ON_CAN1 = true`），功率计接 CAN2
- **红外测距暂缓** → `TOF_ENABLE = false`；接好传感器后改 `true` 即可。⚠️ 规则书 4.4.2.1 / 7.3 把测距读取列为底盘动力调试的前置项，**最终验收前必须补上**

待确认：

1. 底盘功率上限值（以及验收现场是否下发裁判系统 `0x0201`；代码占位 `POWER_LIMIT_W = 60.0f`）
2. C620 电调 ID **实际设定**（2026-10-14 用户按绿灯闪烁次数确认：`4/3/1/2` → `wheel_lf/rf/lr/rr`，即左前4、右前3、左后1、右后2）。收发均按电机对象的 ID 分发；轮向和横移/旋转仍需实机验证。

## 参考

- RoboMaster 开发板 C 型用户手册、原理图
- RoboMaster GM6020 直流无刷电机使用说明、M3508 + C620 说明
- RoboMaster 机器人专用遥控器（接收机）用户手册
- GY-56 红外测距模块使用说明
- 队内 STM32 开发环境安装与使用教程
- `sp_middleware` 各模块的 readme
- `指导文档\第二阶段-调试参数速查表.md`（18 个可调参数的含义与调整方向）

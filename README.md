# 第二阶段 · 底盘 C 板工程（cboard）

第二阶段工程：**底盘 C 板**。只实现两块内容 —— 遥控器与底盘控制（规则书 **4.4.2.2**）、底盘功率控制（**4.4.2.3**）。

## 两块 C 板的分工（2026-10-14 决定）

| 功能 | 底盘 C 板（本工程） | 云台 C 板（`..\files\stage_2_gimbal`） |
| --- | --- | --- |
| 遥控器 + 底盘运动（4.4.2.2） | ✅ 本工程 | — |
| 底盘功率控制（4.4.2.3） | ✅ 本工程 | — |
| 红外测距（4.4.2.1，GY-56 @USART6） | — | ✅ `tof_task.*` |
| 云台自动折叠（4.4.7） | — | ✅ `fold_task.*` 骨架 |
| 云台电机控制（GM6020_V / C610，CAN1） | — | ✅ `gimbal_task.*` 骨架 |
| C 板基本功能调试（4.4.8 蜂鸣器/LED） | 有驱动 | ✅ 做验收 |
| DT7 接收机（DBus @USART3） | ✅ 底盘自己读 | ✅ 云台自己读 |
| IMU（BMI088 SPI1） | ✅ 底盘偏航自锁用 | ✅ 云台姿态用 |

两块板**各自读自己的接收机**，不互相转发；两个工程是独立的 CubeMX + CMake 工程，各自编译烧录。

## 功能

- **失能优先**：右拨杆下档 → 四轮电机停止输出。**三层失能**（拨杆下档 / 遥控器失联 / 任一电机反馈中断 >100 ms）都走同一个 `disable_all()`，进入失能时目标速度与斜坡状态一起清零。
- **遥控映射**（规则书 4.4.2.2）：右拨杆中档才可动（左拨杆上档时由正式测试序列接管）；左摇杆前后 = 前进后退；左摇杆左右 = 左右横移；拨轮 = 自转（满量程 1 rad/s）；摇杆带 2% 死区与线性重映射。
- **正式测试触发（左拨杆上档）**：左拨杆**上档** → 车沿前进方向自动走 `AUTO_DISTANCE_M = 3.3 m` 后停住并保持不动（距离用实测车体速度积分闭环，速度走 `v = √(2as)` 梯形曲线，加减速复用 `V_SLEW` 斜坡）；上档期间摇杆无效，中途失能则放弃本次且**不会自动重新起步**（要重跑必须把拨杆拨离上档再切回）。
- **速度规划**：目标速度经斜坡限幅（`V_SLEW = 0.8 m/s²`、`W_SLEW = 4 rad/s²`），抑制起步冲击与松手后的爬行。
- **麦轮运动学**：`sp::Mecanum` 逆解到四轮线速度 → 每轮速度环 PID（总输出与积分分项分别限幅）→ 轮侧转矩 → C620 电流指令。
- **车体速度反解**：四轮反馈反解出实测 `vx / vy / wz`，供 SerialPlot 与验收展示（规则书要求能算车体速度）。
- **直线偏航锁定**（可选，默认关）：`STRAIGHT_LOCK_ENABLE` 打开后用 IMU yaw 的 P + D 修正叠加到自转项。
- **功率控制**（规则书 4.4.2.3）：按每轮转速/转矩估计机械功率，再按电机与电调损耗模型（K1/K2/K3）估计总功率，超 `POWER_TARGET_W`（= 上限 80 W − 3 W 余量）时等比缩减四轮转矩指令；功率计（CAN2）实测值既做对照，也参与第二轮"只减不增"的兜底限流（上限档位与断电行为的依据见下面「功率控制」一节）。

> 测距读取（4.4.2.1）**不在本工程**：GY-56 的 `tof_task.*` 与 SerialPlot 的测距通道都在云台板工程 `..\files\stage_2_gimbal` 里。

## 任务

| 任务 | 优先级 | 栈（word） | 入口 | 说明 |
| --- | --- | --- | --- | --- |
| defaultTask | Normal | 128 | `defaultTask` | CubeMX 默认任务 |
| PlotterTask | Low | 128 | `plotter_task` | 1 ms 往 SerialPlot 发一帧 12 通道波形 |
| CanTask | Low | 256 | `can_task` | **底盘主任务**：1 ms 控制周期，遥控解析 → 麦轮逆解 → PID → 功率限制 → CAN 发送 |
| ImuTask | AboveNormal | 512 | `imu_task` | BMI088 + Mahony，1 kHz |
| UartTask | Normal | 256 | `uart_task` | 接收机中断/DMA，只负责挂 DMA 与解析 |
| LedTask | Low | 128 | `led_task` | 呼吸灯 |
| BuzzerTask | Low | 128 | `buzzer_task` | 上电提示音 |

> 云台工程另有 `GimbalTask`（云台电机）与 `FoldTask`（自动折叠）；本工程**没有** TofTask。

## 硬件接口

| 板上丝印 | 外设 | 接什么 | 备注 |
| --- | --- | --- | --- |
| 2-pin，黑 CANL / 红 CANH | CAN1 | **4 台 C620**（M3508） | `MOTORS_ON_CAN1 = true`；ID 4/3/1/2 = 左前/右前/左后/右后 |
| 4-pin | CAN2 | **功率计** | 标准帧 `0x301`，只收不发 |
| 4-pin，丝印 `UART2` | USART1 | **SerialPlot** 921600 | TX PA9 / RX PB7，TX 走 DMA |
| 3-pin，丝印 `UART1` | USART6 | **预留** | 云台板的 GY-56 用这个口（9600 8N1 + RX DMA2_Stream2）。本板不接测距模块 |
| 与 PWM 共用口 | USART3 | **DT7 接收机（DBus）** | 100000 bps / 偶校验 / 8 数据位 / 1 停止位，RX DMA1_Stream1 |
| SPI1 | BMI088 | 板载 IMU | CS1 = PA4，CS2 = PB0 |

## 底盘控制（规则书 4.4.2.2）

**`applications/can_task.cpp` 是唯一动手写的地方**，常量都在文件开头：

| 常量 | 现值 | 含义 |
| --- | --- | --- |
| `CHASSIS_MOTION_ENABLE` | `true` | 底盘运动总开关（`false` = 只留失能逻辑，不动电机） |
| `GEAR_RATIO` | `13.7f` | 麦轮总减速比（**不要**用 `sp::M3508_P19`） |
| `WHEEL_R / HALF_L / HALF_W` | 0.077 / 0.165 / 0.185 | 轮半径、前后轮距一半、左右轮距一半（154 / 330 / 370 mm） |
| `V_MAX / WZ_MAX` | 1.0 m/s / 1.0 rad/s | 摇杆满量程对应的车体速度（自转 1 rad/s 是规则书要求） |
| `STICK_DEADZONE` | 0.02f | 摇杆死区 |
| `V_SLEW / W_SLEW` | 0.8 m/s² / 4 rad/s² | 目标速度斜坡 |
| `AUTO_DRIVE_ENABLE` | `true` | 正式测试自动行驶总开关（`false` = 回到纯手动遥控） |
| `AUTO_TRIGGER_ON_LEFT` | `true` | 触发源：`true` = 左拨杆上档（正式）；`false` = 右拨杆上档 |
| `referee_power_limit_w` | `0` | 裁判系统实时功率上限（0 = 用规则书 80 W）；接上 PM02 用户串口后由 `pm02.robot_status.chassis_power_limit` 写入 |
| `AUTO_DISTANCE_M` | 3.3 m | 上档自动行驶的距离（换距离只改这一行） |
| `AUTO_SPEED_MPS` | 0.6 m/s | 自动行驶巡航速度（≤ `V_MAX`） |
| `AUTO_STOP_TOL_M` / `AUTO_TIMEOUT_MS` | 0.03 m / 15 s | 到位容差 / 超时保护 |
| `FWD_DIR / SIDE_DIR / ROT_DIR` | +1 / -1 / +1 | 整车方向极性（推导依据见文件内注释） |
| `SPD_KP / SPD_KI / SPD_KD` | 0.9 / 0.02 / 0.0 | 轮速环 PID（2026-10-14 队友整定） |
| `SPD_MAX_OUT / SPD_MAX_IOUT` | 1.0 / 0.5 | 总输出与积分分项限幅 |
| `SPD_ALPHA` | 0.0f | 反馈滤波（0 = 不滤） |
| `STRAIGHT_LOCK_ENABLE` | `false` | 直线偏航锁定开关 |
| `POWER_LIMIT_W / POWER_MARGIN_W / K1 / K2 / K3` | 80 / 3 / 2 / 0.01 / 2 | 功率模型（**K1/K2/K3 与余量仍未经实测标定**，不得宣称合规；上限 80 W 是新手册的保守档） |
| `MOTORS_ON_CAN1` | `true` | 四台 C620 挂 CAN1 |

**每轮极性的唯一正确改法**：`sp::Mecanum mec(WHEEL_R, HALF_L, HALF_W, reverse_lf, reverse_lr, reverse_rf, reverse_rr)` 的后四个标志（默认 false / false / true / true）。库在逆解时乘 `sign`、在反解时除 `sign`，指令与反馈同时自洽；**不要**用 `cmd(-τ)` 或事后取反（会把 PID 反馈和 SerialPlot 一起弄错）。整车方向只能用 `FWD_DIR / SIDE_DIR / ROT_DIR` 整项取反。

## 功率控制（规则书 4.4.2.3）

- 预测模型：`P = Σ(τ·ω) + K1·Στ² + K2·Σω² + K3`（轮侧 τ / N·m、ω / rad/s；K1/K2/K3 是电机 + 电调损耗系数，仍待实测标定）。
- **上限与余量（2026-10-15 按新到手的手册更新）**：《RM 2025 裁判系统 用户手册（中）V1.4》P19「规则参数设置」只有 **80 / 120 / Unlimited** 三档；P11「机器人属性说明」写明"超过该限制及其能量时，会对底盘进行断电"。因此取保守档 `POWER_LIMIT_W = 80 W`，再按参考资料《底盘功率控制》"三、调试"的建议留 `POWER_MARGIN_W = 3 W` 余量 → 实际限幅 `POWER_TARGET_W = 77 W`。规则参数设成 Unlimited 时把 `POWER_LIMIT_UNLIMITED` 置 `true`。比赛时上限由服务器下发，而本工程只解析 CAN `0x301` 的电压/电流/功率，**读不到上限 / 缓冲能量 / 增益**（那些在裁判系统用户串口协议里，见该手册 P29 与《裁判系统串口协议说明》）。
- 超限时四轮转矩**等比**缩减（不改变车体运动方向），限制后的指令写入 SerialPlot 供验收对照。
- **第二轮实测兜底（只减不增）**：实测功率（一阶低通约 5 ms）超 `POWER_TARGET_W` → 按 `目标/实测` 再乘一次；实测电流超 `PM_CURRENT_CONT_LIMIT_A = 10 A`（PM02 底盘通道最大持续负载，《RM 电源模块 PM02 使用说明书》产品参数：10 A 持续 / 30 A（<500 ms）/ 三通道合计 20 A）→ 按 `10/实测` 压；最低压到 `POWER_FEEDBACK_MIN_K = 0.2`。`0x301` 超 `PM_DATA_TIMEOUT_MS = 100 ms` 未更新就当没有功率计，该层自动失效（行为与改动前一致）。
- 实测功率计帧：CAN2 上标准帧 `0x301`，Data[0:1] = 电流×100（int16，可负）、[2:3] = 电压×100（uint16）、[4:5] = 功率×100（uint16）、[6:7] = 0（解析见 `can_task.cpp` 的 `parse_power()`）。

## 注意事项

- `applications/can_task.cpp` 里 `plot_ch[12]`、`applications/plotter_task.cpp` 里 12 通道波形与 `sp::Mecanum::update()` 的形参顺序（`lf, lr, rf, rr`，**lr 在 rf 前**）三处要一起改。
- SerialPlot 的 DMA 帧缓冲：`plotter_task.cpp` 里用 `huart1.gState == HAL_UART_STATE_READY` 做门控，避免在 DMA 还没读完上一帧时改写缓冲（否则波形错位）。
- DT7 的 DMA 半满（HT）事件不是"一批数据收完"，`uart_task.cpp` 里已显式忽略 `HAL_UART_RXEVENT_HT`。
- `sp_middleware` 是 submodule（上游 `TongjiSuperPower/sp_middleware`，指向 `eddf9a35`，本工程未修改其中任何文件）。SerialPlot 的 DMA 帧缓冲门控是**应用层**改动，在 `applications/plotter_task.cpp`。克隆后需 `git submodule update --init --recursive`。
- 上电前确认：电机 ID、正负极性、机械装配；**先把车架空**再做首次运动测试。

## 已确认 / 待确认

**已确认（2026-10-14）**
1. 减速比 `13.7:1`。
2. CAN 分工：CAN1 = 4×C620，CAN2 = 功率计。
3. 电调 ID：用户按 C620 绿灯闪烁次数确认 `4/3/1/2` → `wheel_lf / rf / lr / rr`（左前 4、右前 3、左后 1、右后 2）。收发都按电机对象的 ID 分发。
4. 轮速环整定：`SPD_KP = 0.9`、`SPD_KI = 0.02`、`SPD_ALPHA = 0`、`V_SLEW = 0.8 m/s²`（队友 2026-10-14 现场调试结果）。

**待确认**
1. 底盘功率上限：按新手册的保守档取 80 W（"规则参数设置"只有 80/120/Unlimited 三档；比赛时由服务器下发，我们读不到；缓冲能量只在裁判客户端实时信息页可见，不在我们读得到的 CAN 帧里）。**要问导师的是：验收现场是否下发 0x0201？不发的话上限用 80 还是 120 W。** K1/K2/K3 与 `POWER_MARGIN_W` 仍需实测标定。
2. 遗留现象：较快横移时前后偏移、右横移时车头右偏（yaw 0.50 → 0.44 rad）—— 现场问题，待排查。
3. 测距（4.4.2.1）与云台折叠（4.4.7）的实物验收在**云台工程**侧完成（本工程已移除测距代码）。

## 参考

- `..\files\stage_2_gimbal\README.md` —— 云台 C 板工程（测距 / 云台 / 折叠）

# 工程功能与接口总结

## 1. 工程目标

本工程基于 MSPM0G3507，实现步进电机云台控制。当前核心功能：

- 接收 K230 摄像头矩形识别误差数据。
- 接收 IMU 陀螺仪姿态数据，解析 `roll / pitch / yaw`。
- X 轴使用安装在云台上的陀螺仪做自稳定：进入追踪后第一次收到的 `yaw` 作为目标角。
- 在 X 轴自稳基础上叠加摄像头 X 轴 PID 补偿。
- Y 轴只使用摄像头 Y 轴 PID 追踪，不做陀螺仪自稳。
- UART1 同总线控制 X/Y 两个闭环步进电机驱动器。
- Y 轴支持电机实时位置读取限位，并保留软件估算限位兜底。
- 支持激光继电器 IO 控制和心电图绘制模式。

当前追踪模式最终输出：

```c
final_speed_x = gyro_speed_x + vision_speed_x;
final_speed_y = vision_speed_y;
```

其中：

- `gyro_speed_x`：X 轴陀螺仪自稳输出。
- `vision_speed_x`：摄像头矩形中心 X 误差 PID 输出。
- `vision_speed_y`：摄像头矩形中心 Y 误差 PID 输出。

## 2. UART 分工

### UART0：K230 视觉输入

MSPM0 使用：

```text
UART0_RX = PA11
UART0_TX = PA10
baud     = 115200, 8N1
```

连接建议：

```text
K230 TX -> PA11
K230 RX -> PA10
K230 GND -> MSPM0 GND
```

视觉数据帧：

```text
AA 55 err_x_H err_x_L err_y_H err_y_L valid checksum
```

字段说明：

```text
err_x    int16_t，矩形中心 X - 画面中心 X
err_y    int16_t，矩形中心 Y - 画面中心 Y
valid    1 表示识别到矩形，0 表示未识别
checksum 前 7 字节累加和低 8 位
```

### UART1：步进电机驱动器总线

MSPM0 使用：

```text
UART1_TX = PA8
UART1_RX = PA18
baud     = 115200, 8N1
```

电机地址：

```text
X 轴电机地址 = 0x01
Y 轴电机地址 = 0x02
```

连接建议：

```text
PA8  -> X/Y 电机驱动器 RX，可共用
PA18 <- 电机驱动器 TX
GND  -> 电机驱动器 GND
```

注意：如果 X/Y 两个驱动器 TX 都直接并到 PA18，回包时可能冲突。当前代码只主动读取 Y 轴位置，建议调试阶段只接 Y 电机 TX 到 PA18。

### UART2：IMU 陀螺仪输入

MSPM0 使用：

```text
UART2_RX = PA22
UART2_TX = PA21
baud     = 38400, 8N1
```

连接建议：

```text
IMU TX  -> PA22
IMU RX  -> PA21
IMU GND -> MSPM0 GND
```

IMU 数据：

```text
帧头：55 53
帧长：11 字节
```

解析结果：

```c
roll_deg
pitch_deg
yaw_deg
```

当前只使用 `yaw_deg` 做 X 轴自稳定。

## 3. 电机通信协议

文件：

```text
motor.c
motor.h
```

速度模式命令：

```text
addr F6 dir speedH speedL accel sync 6B
```

位置模式命令：

```text
addr FD dir speedH speedL accel pulse[4] rel_abs sync 6B
```

停止命令：

```text
addr FE 98 sync 6B
```

读取实时位置：

```text
发送：addr 36 6B
返回：addr 36 sign position[4] 6B
```

主要接口：

```c
void Motor_setspeed_X(int16_t speed);
void Motor_setspeed_Y(int16_t speed);
void Motor_setspeed_XY(int16_t speed_x, int16_t speed_y);

void Motor_move_relative_X(int32_t pulses, uint16_t speed, uint8_t accel);
void Motor_move_relative_Y(int32_t pulses, uint16_t speed, uint8_t accel);
void Motor_move_relative_XY(int32_t pulses_x, int32_t pulses_y,
    uint16_t speed, uint8_t accel);

void Motor_stop_X(void);
void Motor_stop_Y(void);
void Motor_stop_XY(void);

void Motor_read_target_position_X(void);
void Motor_read_target_position_Y(void);
void Motor_read_realtime_position_X(void);
void Motor_read_realtime_position_Y(void);
```

调试变量：

```c
motor_last_x_speed
motor_last_y_speed
motor_last_tx_data[13]
motor_last_tx_len
motor_tx_count
motor_tx_timeout_count
motor_last_x_position_pulses
motor_last_y_position_pulses
```

## 4. 当前控制参数

### X 轴陀螺仪自稳

当前陀螺仪安装在云台 X 轴上，会跟随云台运动。进入追踪模式后，第一次收到的 `yaw_deg` 被记录为目标角：

```c
gimbal_x_target_yaw_deg = yaw_deg;
```

后续控制误差：

```c
gimbal_x_yaw_error_deg = target_yaw - current_yaw;
```

参数：

```c
#define GIMBAL_X_YAW_STABILIZE_ENABLE (1)
#define GIMBAL_X_PID_KP               (35.0f)
#define GIMBAL_X_PID_KI               (0.0f)
#define GIMBAL_X_PID_KD               (0.0f)
#define GIMBAL_X_YAW_DEADBAND_DEG     (0.5f)
#define GIMBAL_X_SPEED_DEADBAND       (8)
#define GIMBAL_X_MOTOR_REVERSE        (0)
```

如果自稳方向反了，修改：

```c
#define GIMBAL_X_MOTOR_REVERSE        (1)
```

### 摄像头 PID

```c
#define VISION_X_PID_KP               (0.35f)
#define VISION_X_PID_KI               (0.0f)
#define VISION_X_PID_KD               (0.05f)
#define VISION_Y_PID_KP               (0.35f)
#define VISION_Y_PID_KI               (0.0f)
#define VISION_Y_PID_KD               (0.05f)
#define VISION_PIXEL_DEADBAND         12
#define VISION_MIN_TRACK_SPEED        30
#define VISION_MOTOR_UPDATE_DIV       4
#define VISION_LOST_TIMEOUT_MS        300
```

方向配置：

```c
#define VISION_X_MOTOR_REVERSE        (1)
#define VISION_Y_MOTOR_REVERSE        (0)
```

如果视觉追踪方向反了，只改对应轴的 `VISION_*_MOTOR_REVERSE`。

### 齿轮比补偿

电机与云台轴齿轮比：

```text
X 轴：1/5
Y 轴：1/10
```

代码补偿：

```c
#define GIMBAL_X_GEAR_COMPENSATION    (5)
#define GIMBAL_Y_GEAR_COMPENSATION    (10)
```

## 5. Y 轴限位

当前有两层 Y 轴限位。

### 电机实时位置限位

优先使用 Y 电机 `0x36` 实时位置回包做限位：

```c
#define Y_POSITION_LIMIT_ENABLE       (1)
#define Y_POSITION_QUERY_INTERVAL_MS  (100U)
#define Y_POSITION_TIMEOUT_MS         (300U)
#define Y_GIMBAL_LIMIT_MIN_DEG_X10    (-350)
#define Y_GIMBAL_LIMIT_MAX_DEG_X10    (350)
```

角度单位是 `度 * 10`，当前范围为：

```text
-35.0 度 到 +35.0 度
```

换算关系：

```text
电机原始位置一圈 = 65536
云台角度 = 电机角度 / 10
```

### 软件估算限位兜底

如果暂时没有有效位置回包，会使用速度积分估算位置：

```c
#define Y_SOFT_LIMIT_ENABLE           (1)
#define Y_SOFT_LIMIT_MIN_POS          (-5000)
#define Y_SOFT_LIMIT_MAX_POS          (5000)
#define Y_SOFT_LIMIT_UP_DIR           (1)
```

这是兜底保护，不如真实电机位置可靠。上电时建议把 Y 轴放在安全中位。

## 6. 按键、激光和模式

按键：

```text
Button1 = PB21，按下接 GND，进入追踪模式
Button2 = PB14，按下接 GND，进入心电图绘制模式
Button3 = PB15，按下接 GND，进入电机测试/停止相关逻辑
```

激光继电器：

```text
laser = PB13
```

当前宏：

```c
#define LASER_RELAY_ACTIVE_HIGH       (1)
```

如果继电器模块低电平触发，改为：

```c
#define LASER_RELAY_ACTIVE_HIGH       (0)
```

模式：

```c
APP_MODE_TRACKING   = 1
APP_MODE_ECG        = 2
APP_MODE_MOTOR_TEST = 3
APP_MODE_STOP       = 4
```

## 7. 心电图绘制模式

心电图模式使用内置波形数组 `ecg_wave_points[]`，通过相对位置模式控制 X/Y 轴移动，并控制激光开关。

参数：

```c
#define ECG_DRAW_INTERVAL_MS          (120U)
#define ECG_X_PULSES_PER_POINT        (60)
#define ECG_Y_PULSE_SCALE             (8)
#define ECG_POSITION_SPEED            (600U)
#define ECG_POSITION_ACCEL            (0U)
#define ECG_RETURN_WAIT_MS            (500U)
```

## 8. 重要 Watch 变量

视觉通信：

```c
uart0_rx_count
uart0_poll_count
uart0_raw_frame_count
uart0_drop_count
vision_frame_count
vision_frame_ok
vision_checksum_error_count
vision_header_error_count
```

视觉数据：

```c
err_x
err_y
valid
sum_calc
```

IMU：

```c
uart2_rx_count
uart2_poll_count
imu_frame_count
imu_checksum_error_count
roll_deg
pitch_deg
yaw_deg
```

X 轴自稳：

```c
gimbal_x_target_yaw_deg
gimbal_x_yaw_error_deg
gimbal_x_yaw_target_locked
gyro_speed_x
```

控制输出：

```c
vision_speed_x
vision_speed_y
final_speed_x
final_speed_y
tracking_motor_update_count
vision_motor_update_count
vision_timeout_stop
```

Y 轴位置限位：

```c
motor_uart1_rx_count
y_position_query_count
y_position_frame_count
y_position_error_count
y_position_valid
y_position_age_ms
y_motor_position_raw
y_gimbal_position_deg_x10
y_position_limit_state
y_position_limit_hit_count
```

Y 轴软件限位：

```c
y_soft_position
y_soft_limited_speed
y_soft_limit_state
y_soft_limit_hit_count
```

电机发送：

```c
motor_last_x_speed
motor_last_y_speed
motor_last_tx_data[0..12]
motor_last_tx_len
motor_tx_count
motor_tx_timeout_count
```

## 9. K230 脚本说明

工程目录下存在：

```text
det_video.py
```

该脚本用于 K230 摄像头矩形识别，并通过串口发送矩形中心相对屏幕中心的误差。

发送格式必须与 MSPM0 一致：

```text
AA 55 err_x_H err_x_L err_y_H err_y_L valid checksum
```

建议确认 K230 端：

```text
UART 波特率：115200
K230 TX 接 MSPM0 PA11
K230 GND 与 MSPM0 共地
```

## 10. 当前追踪控制流程

1. `UART0` 接收 K230 视觉误差帧。
2. `err_x / err_y` 进入视觉 PID，生成 `vision_speed_x / vision_speed_y`。
3. `UART2` 接收 IMU 数据，解析 `yaw_deg`。
4. 第一次有效 `yaw_deg` 锁定为 `gimbal_x_target_yaw_deg`。
5. 后续 yaw 误差进入 X 轴自稳 PID，生成 `gyro_speed_x`。
6. `UART1` 每 100ms 查询 Y 轴电机实时位置，更新 Y 轴限位状态。
7. 合成速度：

```c
final_speed_x = gyro_speed_x + vision_speed_x;
final_speed_y = vision_speed_y;
```

8. 对 Y 轴速度做位置限位/软件限位。
9. 通过 UART1 发送 X/Y 电机速度命令。

## 11. 构建方式

使用 CCS Theia 或在 `Debug` 目录下执行：

```text
D:\Ti\ccs\utils\bin\gmake.exe -B all
```

输出文件：

```text
Debug/gpio_toggle_output_LP_MSPM0G3507_nortos_ticlang.out
```

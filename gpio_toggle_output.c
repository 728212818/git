/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 */


#include "ti_msp_dl_config.h"
#include <stdio.h>
#include "motor.h"

#define DELAY (16000000)
#define DELAY_us (32)

#define CONTROL_TIMER_PERIOD_MS       (10U)
#define CONTROL_TIMER_PERIOD_S        (0.01f)

#define LASER_RELAY_ACTIVE_HIGH       (0)

#define ECG_DRAW_INTERVAL_MS          (120U)
#define ECG_X_SIZE_PULSES_PER_POINT   (40)
#define ECG_Y_SIZE_PULSES_PER_LEVEL   (8)
#define ECG_POSITION_SPEED            (600U)
#define ECG_POSITION_ACCEL            (0U)
#define ECG_RETURN_WAIT_MS            (500U)
#define ECG_REPEAT_COUNT              (4U)

#define GIMBAL_X_PID_DT_S             CONTROL_TIMER_PERIOD_S
#define GIMBAL_X_PID_KP               (29.0f)
#define GIMBAL_X_PID_KI               (0.7f)
#define GIMBAL_X_PID_KD               (10.0f)
#define GIMBAL_X_PID_INTEGRAL_LIMIT   (300.0f)
#define GIMBAL_X_YAW_DEADBAND_DEG     (0.5f)
#define GIMBAL_X_SPEED_DEADBAND       (8)
#define GIMBAL_X_SPEED_LIMIT          MOTOR_SPEED_MAX
#define GIMBAL_X_MOTOR_REVERSE        (0)
#define GIMBAL_X_GEAR_COMPENSATION    (5)
#define GIMBAL_Y_GEAR_COMPENSATION    (10)
#define GIMBAL_X_YAW_STABILIZE_ENABLE (1)
#define GIMBAL_X_VISION_TARGET_ENABLE (1)
#define GIMBAL_X_VISION_DEG_PER_PIXEL_S (0.60f)
#define GIMBAL_X_VISION_TARGET_LIMIT_DEG (60.0f)

#define GIMBAL_USE_VISION_TRACKING    (1)
#define VISION_RX_LEN                 8
#define VISION_FRAME_HEAD1            0xAA
#define VISION_FRAME_HEAD2            0x55
#define VISION_VALID_TARGET           1

#define VISION_PID_DT_S               CONTROL_TIMER_PERIOD_S
#define VISION_X_PID_KP               (0.22f)
#define VISION_X_PID_KI               (0.0f)
#define VISION_X_PID_KD               (0.90f)
#define VISION_Y_PID_KP               (0.20f)
#define VISION_Y_PID_KI               (0.01f)
#define VISION_Y_PID_KD               (0.05f)
#define VISION_PID_INTEGRAL_LIMIT     (300.0f)
#define VISION_PIXEL_DEADBAND         12
#define VISION_MIN_TRACK_SPEED        30
#define VISION_SPEED_LIMIT            MOTOR_SPEED_MAX
#define VISION_X_MOTOR_REVERSE        (1)
#define VISION_Y_MOTOR_REVERSE        (0)
#define VISION_TRACK_Y_ENABLE         (1)
#define VISION_MOTOR_UPDATE_DIV       4
#define VISION_LOST_TIMEOUT_MS        300

#define Y_SOFT_LIMIT_ENABLE           (1)
#define Y_SOFT_LIMIT_MIN_POS          (-5000)
#define Y_SOFT_LIMIT_MAX_POS          (5000)
#define Y_SOFT_LIMIT_UP_DIR           (1)
#define Y_POSITION_LIMIT_ENABLE       (1)
#define Y_POSITION_QUERY_INTERVAL_MS  (100U)
#define Y_POSITION_TIMEOUT_MS         (300U)
#define Y_GIMBAL_LIMIT_MIN_DEG_X10    (-200)
#define Y_GIMBAL_LIMIT_MAX_DEG_X10    (200)
#define Y_MOTOR_RAW_PER_REV           (65536.0f)
#define Y_MOTOR_ADDR                  (0x02)
#define MOTOR_POSITION_FRAME_LEN      (8)

#define X_MOTOR_STANDALONE_TEST       (0)
#define X_MOTOR_TEST_SPEED            (300)
#define X_MOTOR_TEST_HOLD_MS          (1000)
#define MOTOR_TEST_SPEED              (300)
#define MOTOR_TEST_STEP_MS            (1000U)
#define BUTTON1_LONG_PRESS_MS         (1500U)
#define TRACKING_NO_INPUT_TEST_ENABLE (1)
#define TRACKING_NO_INPUT_TIMEOUT_MS  (1000U)
#define TRACKING_NO_INPUT_TEST_SPEED  (180)
#define TRACKING_NO_INPUT_STEP_MS     (800U)

#define RX_LEN                        VISION_RX_LEN

typedef enum {
    APP_MODE_NONE = 0,
    APP_MODE_TRACKING = 1,
    APP_MODE_ECG = 2,
    APP_MODE_MOTOR_TEST = 3,
    APP_MODE_STOP = 4
} AppMode;

void parse_imu_data(void);

#define IMU_RX_LEN    11      // 整包长度：0x55+0x53 + 9字节数据
#define FRAME_HEAD1   0x55    // 帧头1
#define FRAME_HEAD2   0x53    // 帧头2


//======================摄像头数据接受
#define RX_LEN    VISION_RX_LEN
volatile uint8_t rx_buf[RX_LEN];
volatile uint8_t vision_frame_buf[RX_LEN];
volatile uint8_t rx_idx = 0;
volatile bool rx_finish = false;
//======================

//========================
volatile uint8_t imu_rx_buf[IMU_RX_LEN];
volatile uint8_t imu_rx_idx = 0;
bool imu_rx_finish = false;

volatile int16_t roll, pitch, yaw;
float roll_deg;
float pitch_deg;
float yaw_deg;
//==============================

volatile int16_t err_x = 0;
volatile int16_t err_y = 0;
volatile uint8_t valid = 0;
volatile uint8_t sum_calc = 0;
volatile uint8_t vision_frame_ok = 0;
volatile uint32_t vision_frame_count = 0;
volatile uint32_t vision_checksum_error_count = 0;
volatile uint32_t vision_header_error_count = 0;
volatile int16_t vision_speed_x = 0;
volatile int16_t vision_speed_y = 0;
volatile int16_t gyro_speed_x = 0;
volatile int16_t final_speed_x = 0;
volatile int16_t final_speed_y = 0;
volatile float gimbal_x_target_yaw_deg = 0.0f;
volatile float gimbal_x_base_yaw_deg = 0.0f;
volatile float gimbal_x_vision_target_offset_deg = 0.0f;
volatile float gimbal_x_yaw_error_deg = 0.0f;
volatile uint8_t gimbal_x_yaw_target_locked = 0;
volatile uint8_t uart0_last_byte = 0;
volatile uint32_t uart0_rx_count = 0;
volatile uint32_t uart0_raw_frame_count = 0;
volatile uint32_t uart0_drop_count = 0;
volatile uint8_t uart2_last_byte = 0;
volatile uint32_t uart2_rx_count = 0;
volatile uint32_t uart0_poll_count = 0;
volatile uint32_t uart2_poll_count = 0;
volatile uint32_t imu_frame_count = 0;
volatile uint32_t imu_checksum_error_count = 0;
volatile uint32_t vision_motor_update_count = 0;
volatile uint8_t vision_timeout_stop = 0;
volatile uint32_t control_timer_tick_count = 0;
volatile uint8_t laser_state = 0;
volatile uint8_t app_mode = APP_MODE_NONE;
volatile uint32_t ecg_step_count = 0;
volatile uint16_t ecg_point_index = 0;
volatile uint8_t ecg_cycle_count = 0;
volatile uint8_t ecg_draw_done = 0;
volatile uint16_t ecg_interval_ms = 0;
volatile uint32_t tracking_motor_update_count = 0;
volatile uint32_t motor_test_step_count = 0;
volatile uint32_t tracking_no_input_ms = 0;
volatile uint8_t tracking_no_input_test_active = 0;
volatile int32_t y_soft_position = 0;
volatile int16_t y_soft_limited_speed = 0;
volatile uint8_t y_soft_limit_state = 0;
volatile uint32_t y_soft_limit_hit_count = 0;
static float y_soft_position_f = 0.0f;
volatile uint8_t motor_uart1_last_byte = 0;
volatile uint32_t motor_uart1_rx_count = 0;
volatile uint32_t y_position_query_count = 0;
volatile uint32_t y_position_frame_count = 0;
volatile uint32_t y_position_error_count = 0;
volatile uint8_t y_position_valid = 0;
volatile int32_t y_motor_position_raw = 0;
volatile int16_t y_gimbal_position_deg_x10 = 0;
volatile uint8_t y_position_limit_state = 0;
volatile uint32_t y_position_limit_hit_count = 0;
volatile uint16_t y_position_age_ms = Y_POSITION_TIMEOUT_MS;
static volatile uint8_t motor_position_rx_buf[MOTOR_POSITION_FRAME_LEN];
static volatile uint8_t motor_position_rx_idx = 0;

static const int8_t ecg_wave_points[] = {
    0, 0, 0, 1, 3, 5, 3, 1,
    0, -2, 8, 24, 6, -5, 0, 0,
    2, 5, 9, 10, 8, 4, 1, 0
};

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float integral_limit;
    int16_t output_limit;
} PID_Controller;

static PID_Controller gimbal_x_pid = {
    GIMBAL_X_PID_KP,
    GIMBAL_X_PID_KI,
    GIMBAL_X_PID_KD,
    0.0f,
    0.0f,
    GIMBAL_X_PID_INTEGRAL_LIMIT,
    GIMBAL_X_SPEED_LIMIT
};

static bool gimbal_x_yaw_inited = false;

static PID_Controller vision_x_pid = {
    VISION_X_PID_KP,
    VISION_X_PID_KI,
    VISION_X_PID_KD,
    0.0f,
    0.0f,
    VISION_PID_INTEGRAL_LIMIT,
    VISION_SPEED_LIMIT
};

static PID_Controller vision_y_pid = {
    VISION_Y_PID_KP,
    VISION_Y_PID_KI,
    VISION_Y_PID_KD,
    0.0f,
    0.0f,
    VISION_PID_INTEGRAL_LIMIT,
    VISION_SPEED_LIMIT
};

static float clamp_float(float value, float min_value, float max_value)
{
    if(value > max_value)
    {
        return max_value;
    }
    if(value < min_value)
    {
        return min_value;
    }
    return value;
}

static float angle_error_deg(float target_deg, float current_deg)
{
    float error = target_deg - current_deg;

    while(error > 180.0f)
    {
        error -= 360.0f;
    }
    while(error < -180.0f)
    {
        error += 360.0f;
    }

    return error;
}

static void pid_reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

static int16_t pid_update(PID_Controller *pid, float error, float dt_s)
{
    float derivative;
    float output;

    pid->integral += error * dt_s;
    pid->integral = clamp_float(pid->integral,
        -pid->integral_limit, pid->integral_limit);

    derivative = (error - pid->last_error) / dt_s;
    pid->last_error = error;

    output = (pid->kp * error) + (pid->ki * pid->integral) +
        (pid->kd * derivative);
    output = clamp_float(output, -(float)pid->output_limit,
        (float)pid->output_limit);

    return (int16_t)output;
}

static int16_t scale_speed_with_gear(int16_t speed, int16_t gear_ratio)
{
    int32_t scaled_speed = (int32_t)speed * gear_ratio;

    if(scaled_speed > MOTOR_SPEED_MAX)
    {
        return MOTOR_SPEED_MAX;
    }
    if(scaled_speed < -MOTOR_SPEED_MAX)
    {
        return -MOTOR_SPEED_MAX;
    }

    return (int16_t)scaled_speed;
}

static int16_t add_speed_limit(int16_t base_speed, int16_t add_speed)
{
    int32_t speed = (int32_t)base_speed + add_speed;

    if(speed > MOTOR_SPEED_MAX)
    {
        return MOTOR_SPEED_MAX;
    }
    if(speed < -MOTOR_SPEED_MAX)
    {
        return -MOTOR_SPEED_MAX;
    }

    return (int16_t)speed;
}

static float add_angle_wrap_deg(float base_deg, float add_deg)
{
    float angle = base_deg + add_deg;

    while(angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while(angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

static float angle_limit_around_base_deg(float target_deg, float base_deg,
    float limit_deg)
{
    float delta = angle_error_deg(target_deg, base_deg);

    delta = clamp_float(delta, -limit_deg, limit_deg);
    return add_angle_wrap_deg(base_deg, delta);
}

static void Laser_Set(uint8_t enable)
{
    laser_state = (enable != 0) ? 1 : 0;

#if LASER_RELAY_ACTIVE_HIGH
    if(laser_state != 0)
    {
        DL_GPIO_setPins(laser_PORT, laser_PIN_0_PIN);
    }
    else
    {
        DL_GPIO_clearPins(laser_PORT, laser_PIN_0_PIN);
    }
#else
    if(laser_state != 0)
    {
        DL_GPIO_clearPins(laser_PORT, laser_PIN_0_PIN);
    }
    else
    {
        DL_GPIO_setPins(laser_PORT, laser_PIN_0_PIN);
    }
#endif
}

static void Laser_On(void)
{
    Laser_Set(1);
}

static void Laser_Off(void)
{
    Laser_Set(0);
}

static bool button_pressed(uint32_t pin)
{
    return ((DL_GPIO_readPins(Button_PORT, pin) & pin) == 0);
}

static void stop_all_motion(void)
{
    gyro_speed_x = 0;
    vision_speed_x = 0;
    vision_speed_y = 0;
    final_speed_x = 0;
    final_speed_y = 0;
    pid_reset(&gimbal_x_pid);
    pid_reset(&vision_x_pid);
    pid_reset(&vision_y_pid);
    gimbal_x_yaw_inited = false;
    gimbal_x_yaw_target_locked = 0;
    gimbal_x_yaw_error_deg = 0.0f;
    Motor_stop_XY();
}

static int16_t y_axis_apply_soft_limit(int16_t speed)
{
#if Y_SOFT_LIMIT_ENABLE
    int16_t limited_speed = speed;
    bool moving_up;
    bool moving_down;

    y_soft_limit_state = 0;
    moving_up = ((Y_SOFT_LIMIT_UP_DIR != 0) && (speed > 0)) ||
        ((Y_SOFT_LIMIT_UP_DIR == 0) && (speed < 0));
    moving_down = ((Y_SOFT_LIMIT_UP_DIR != 0) && (speed < 0)) ||
        ((Y_SOFT_LIMIT_UP_DIR == 0) && (speed > 0));

#if Y_POSITION_LIMIT_ENABLE
    if((y_position_valid != 0U) && (y_position_age_ms < Y_POSITION_TIMEOUT_MS))
    {
        y_position_limit_state = 0;

        if((y_gimbal_position_deg_x10 >= Y_GIMBAL_LIMIT_MAX_DEG_X10) &&
            moving_up)
        {
            limited_speed = 0;
            y_position_limit_state = 1;
        }
        else if((y_gimbal_position_deg_x10 <= Y_GIMBAL_LIMIT_MIN_DEG_X10) &&
            moving_down)
        {
            limited_speed = 0;
            y_position_limit_state = 2;
        }

        if(limited_speed != speed)
        {
            y_position_limit_hit_count++;
        }

        y_soft_limited_speed = limited_speed;
        return limited_speed;
    }
#endif

    if((y_soft_position >= Y_SOFT_LIMIT_MAX_POS) && moving_up)
    {
        limited_speed = 0;
        y_soft_limit_state = 1;
    }
    else if((y_soft_position <= Y_SOFT_LIMIT_MIN_POS) && moving_down)
    {
        limited_speed = 0;
        y_soft_limit_state = 2;
    }

    if(limited_speed != speed)
    {
        y_soft_limit_hit_count++;
    }

    y_soft_limited_speed = limited_speed;
    return limited_speed;
#else
    y_soft_limited_speed = speed;
    y_soft_limit_state = 0;
    return speed;
#endif
}

static void y_axis_soft_limit_update_position(int16_t applied_speed)
{
#if Y_SOFT_LIMIT_ENABLE
    y_soft_position_f += (float)applied_speed * CONTROL_TIMER_PERIOD_S;

    if(y_soft_position_f > (float)Y_SOFT_LIMIT_MAX_POS)
    {
        y_soft_position_f = (float)Y_SOFT_LIMIT_MAX_POS;
    }
    else if(y_soft_position_f < (float)Y_SOFT_LIMIT_MIN_POS)
    {
        y_soft_position_f = (float)Y_SOFT_LIMIT_MIN_POS;
    }

    y_soft_position = (int32_t)y_soft_position_f;
#else
    (void)applied_speed;
#endif
}

static bool y_axis_soft_limit_guard_tick(void)
{
#if Y_SOFT_LIMIT_ENABLE && VISION_TRACK_Y_ENABLE
    int16_t limited_speed = y_axis_apply_soft_limit(final_speed_y);

    if(limited_speed != final_speed_y)
    {
        final_speed_y = limited_speed;
        Motor_setspeed_XY(final_speed_x, final_speed_y);
        return true;
    }

    y_axis_soft_limit_update_position(final_speed_y);
#endif
    return false;
}

static void send_combined_gimbal_speed(void)
{
    final_speed_x = gyro_speed_x;
    final_speed_y = y_axis_apply_soft_limit(vision_speed_y);
    y_axis_soft_limit_update_position(final_speed_y);

#if VISION_TRACK_Y_ENABLE
    Motor_setspeed_XY(final_speed_x, final_speed_y);
#else
    Motor_setspeed_X(final_speed_x);
#endif
}

static void send_combined_gimbal_speed_if_changed(void)
{
    int16_t next_speed_x = gyro_speed_x;
    int16_t next_speed_y = y_axis_apply_soft_limit(vision_speed_y);

    if((next_speed_x == final_speed_x) && (next_speed_y == final_speed_y))
    {
        y_axis_soft_limit_update_position(final_speed_y);
        return;
    }

    final_speed_x = next_speed_x;
    final_speed_y = next_speed_y;
    y_axis_soft_limit_update_position(final_speed_y);
    tracking_motor_update_count++;

#if VISION_TRACK_Y_ENABLE
    Motor_setspeed_XY(final_speed_x, final_speed_y);
#else
    Motor_setspeed_X(final_speed_x);
#endif
}

static void gimbal_x_update_yaw_compensation(float current_yaw_deg)
{
    int16_t speed_cmd;

    if(gimbal_x_yaw_inited == false)
    {
        gimbal_x_base_yaw_deg = current_yaw_deg;
        gimbal_x_target_yaw_deg = current_yaw_deg;
        gimbal_x_yaw_error_deg = 0.0f;
        gimbal_x_yaw_target_locked = 1;
        gimbal_x_yaw_inited = true;
        pid_reset(&gimbal_x_pid);
        gyro_speed_x = 0;
        return;
    }

    gimbal_x_vision_target_offset_deg = angle_error_deg(
        gimbal_x_target_yaw_deg, gimbal_x_base_yaw_deg);
    gimbal_x_yaw_error_deg = angle_error_deg(gimbal_x_target_yaw_deg,
        current_yaw_deg);

    if((gimbal_x_yaw_error_deg < GIMBAL_X_YAW_DEADBAND_DEG) &&
        (gimbal_x_yaw_error_deg > -GIMBAL_X_YAW_DEADBAND_DEG))
    {
        pid_reset(&gimbal_x_pid);
        gyro_speed_x = 0;
        return;
    }

    speed_cmd = pid_update(&gimbal_x_pid, gimbal_x_yaw_error_deg,
        GIMBAL_X_PID_DT_S);

#if GIMBAL_X_MOTOR_REVERSE
    speed_cmd = -speed_cmd;
#endif

    if((speed_cmd < GIMBAL_X_SPEED_DEADBAND) &&
        (speed_cmd > -GIMBAL_X_SPEED_DEADBAND))
    {
        speed_cmd = 0;
    }

    speed_cmd = scale_speed_with_gear(speed_cmd, GIMBAL_X_GEAR_COMPENSATION);
    gyro_speed_x = speed_cmd;
}

static uint8_t vision_checksum(const volatile uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;

    for(uint8_t i = 0; i < len; i++)
    {
        sum += data[i];
    }

    return sum;
}

static void vision_stop_tracking(void)
{
    pid_reset(&vision_x_pid);
    pid_reset(&vision_y_pid);
    vision_speed_x = 0;
    vision_speed_y = 0;
}

static bool vision_timeout_check(void)
{
    static uint32_t last_frame_count = 0;
    static uint16_t lost_ms = 0;

    if(vision_frame_count != last_frame_count)
    {
        last_frame_count = vision_frame_count;
        lost_ms = 0;
        vision_timeout_stop = 0;
        return false;
    }

    if(lost_ms < VISION_LOST_TIMEOUT_MS)
    {
        lost_ms += CONTROL_TIMER_PERIOD_MS;
        return false;
    }

    if(vision_timeout_stop == 0)
    {
        vision_stop_tracking();
        valid = 0;
        vision_timeout_stop = 1;
        return true;
    }

    return false;
}

static int16_t vision_pid_to_speed(PID_Controller *pid, int16_t pixel_error,
    float dt_s)
{
    int16_t speed_cmd;

    if((pixel_error < VISION_PIXEL_DEADBAND) &&
        (pixel_error > -VISION_PIXEL_DEADBAND))
    {
        pid_reset(pid);
        return 0;
    }

    speed_cmd = pid_update(pid, (float)pixel_error, dt_s);
    if((speed_cmd < GIMBAL_X_SPEED_DEADBAND) &&
        (speed_cmd > -GIMBAL_X_SPEED_DEADBAND))
    {
        speed_cmd = 0;
    }
    else if((speed_cmd > 0) && (speed_cmd < VISION_MIN_TRACK_SPEED))
    {
        speed_cmd = VISION_MIN_TRACK_SPEED;
    }
    else if((speed_cmd < 0) && (speed_cmd > -VISION_MIN_TRACK_SPEED))
    {
        speed_cmd = -VISION_MIN_TRACK_SPEED;
    }

    return speed_cmd;
}

static void vision_track_rect_error(int16_t pixel_err_x, int16_t pixel_err_y,
    uint8_t target_valid)
{
    int16_t speed_x;
    int16_t speed_y;
    float target_offset_deg;
    static uint8_t update_div = 0;
    static int16_t last_speed_x = 0;
    static int16_t last_speed_y = 0;

    if(target_valid != VISION_VALID_TARGET)
    {
        vision_stop_tracking();
        return;
    }

#if GIMBAL_X_VISION_TARGET_ENABLE
    speed_x = 0;
    pid_reset(&vision_x_pid);
    vision_speed_x = 0;
    if((pixel_err_x < VISION_PIXEL_DEADBAND) &&
        (pixel_err_x > -VISION_PIXEL_DEADBAND))
    {
        target_offset_deg = 0.0f;
    }
    else
    {
        target_offset_deg = (float)pixel_err_x *
            GIMBAL_X_VISION_DEG_PER_PIXEL_S * VISION_PID_DT_S;
    }

#if VISION_X_MOTOR_REVERSE
    target_offset_deg = -target_offset_deg;
#endif

    gimbal_x_target_yaw_deg = add_angle_wrap_deg(gimbal_x_target_yaw_deg,
        target_offset_deg);
    gimbal_x_target_yaw_deg = angle_limit_around_base_deg(
        gimbal_x_target_yaw_deg, gimbal_x_base_yaw_deg,
        GIMBAL_X_VISION_TARGET_LIMIT_DEG);
    gimbal_x_vision_target_offset_deg = angle_error_deg(
        gimbal_x_target_yaw_deg, gimbal_x_base_yaw_deg);
#else
    speed_x = vision_pid_to_speed(&vision_x_pid, pixel_err_x, VISION_PID_DT_S);
#endif
    speed_y = vision_pid_to_speed(&vision_y_pid, pixel_err_y, VISION_PID_DT_S);

#if !GIMBAL_X_VISION_TARGET_ENABLE
#if VISION_X_MOTOR_REVERSE
    speed_x = -speed_x;
#endif
#endif

#if VISION_Y_MOTOR_REVERSE
    speed_y = -speed_y;
#endif

#if !GIMBAL_X_VISION_TARGET_ENABLE
    speed_x = scale_speed_with_gear(speed_x, GIMBAL_X_GEAR_COMPENSATION);
    vision_speed_x = speed_x;
#endif
    speed_y = scale_speed_with_gear(speed_y, GIMBAL_Y_GEAR_COMPENSATION);

    vision_speed_y = speed_y;

#if VISION_TRACK_Y_ENABLE
    update_div++;
    if((update_div < VISION_MOTOR_UPDATE_DIV) &&
        (speed_x == last_speed_x) && (speed_y == last_speed_y))
    {
        return;
    }
    update_div = 0;
    last_speed_x = speed_x;
    last_speed_y = speed_y;
    vision_motor_update_count++;
#else
    vision_motor_update_count++;
#endif
}

static void parse_vision_data(void)
{
    if((vision_frame_buf[0] != VISION_FRAME_HEAD1) ||
        (vision_frame_buf[1] != VISION_FRAME_HEAD2))
    {
        vision_header_error_count++;
        vision_frame_ok = 0;
        return;
    }

    sum_calc = vision_checksum(vision_frame_buf, VISION_RX_LEN - 1);
    if(sum_calc != vision_frame_buf[VISION_RX_LEN - 1])
    {
        vision_checksum_error_count++;
        vision_frame_ok = 0;
        return;
    }

    err_x = (int16_t)(((uint16_t)vision_frame_buf[2] << 8) |
        vision_frame_buf[3]);
    err_y = (int16_t)(((uint16_t)vision_frame_buf[4] << 8) |
        vision_frame_buf[5]);
    valid = vision_frame_buf[6];
    vision_frame_ok = 1;
    vision_frame_count++;

    vision_track_rect_error(err_x, err_y, valid);
}





void delay_cyc_us(uint16_t us)
{
    for(uint16_t i=0;i<us;i++)
    {
        delay_cycles(DELAY_us);
    }
}

static void delay_ms(uint16_t ms)
{
    for(uint16_t i = 0; i < ms; i++)
    {
        delay_cyc_us(1000);
    }
}

static void wait_buttons_released(void)
{
    while(button_pressed(Button_Button1_PIN) ||
        button_pressed(Button_Button2_PIN) ||
        button_pressed(Button_Button3_PIN))
    {
        delay_ms(10);
    }
}

static AppMode select_startup_mode(void)
{
    while(1)
    {
        if(button_pressed(Button_Button1_PIN))
        {
            delay_ms(20);
            if(button_pressed(Button_Button1_PIN))
            {
                return APP_MODE_TRACKING;
            }
        }

        if(button_pressed(Button_Button2_PIN))
        {
            delay_ms(20);
            if(button_pressed(Button_Button2_PIN))
            {
                return APP_MODE_ECG;
            }
        }

        if(button_pressed(Button_Button3_PIN))
        {
            delay_ms(20);
            if(button_pressed(Button_Button3_PIN))
            {
                return APP_MODE_MOTOR_TEST;
            }
        }
    }
}

static void x_motor_standalone_test(void)
{
    Motor_setspeed_X(X_MOTOR_TEST_SPEED);
    delay_ms(X_MOTOR_TEST_HOLD_MS);

    Motor_setspeed_X(0);
    delay_ms(300);

    Motor_setspeed_X(-X_MOTOR_TEST_SPEED);
    delay_ms(X_MOTOR_TEST_HOLD_MS);

    Motor_setspeed_X(0);
    delay_ms(300);
}

static void ecg_draw_reset(void)
{
    ecg_point_index = 0;
    ecg_step_count = 0;
    ecg_cycle_count = 0;
    ecg_draw_done = 0;
    ecg_interval_ms = 0;
    Laser_Off();
    Motor_stop_XY();
}

static void ecg_draw_task(void)
{
    int32_t pulses_x;
    int32_t pulses_y;
    uint16_t point_count = (uint16_t)(sizeof(ecg_wave_points) /
        sizeof(ecg_wave_points[0]));

    if(ecg_draw_done != 0)
    {
        return;
    }

    if(ecg_interval_ms < ECG_DRAW_INTERVAL_MS)
    {
        ecg_interval_ms += CONTROL_TIMER_PERIOD_MS;
        return;
    }
    ecg_interval_ms = 0;

    if(ecg_point_index == 0)
    {
        Laser_On();
    }

    if(ecg_point_index >= (point_count - 1U))
    {
        ecg_cycle_count++;
        ecg_point_index = 0;

        if(ecg_cycle_count >= ECG_REPEAT_COUNT)
        {
            Laser_Off();
            Motor_stop_XY();
            ecg_draw_done = 1;
        }
        return;
    }

    pulses_x = ECG_X_SIZE_PULSES_PER_POINT;
    pulses_y = ((int32_t)ecg_wave_points[ecg_point_index + 1U] -
        (int32_t)ecg_wave_points[ecg_point_index]) *
        ECG_Y_SIZE_PULSES_PER_LEVEL;

    Motor_move_relative_XY(pulses_x, pulses_y, ECG_POSITION_SPEED,
        ECG_POSITION_ACCEL);
    ecg_point_index++;
    ecg_step_count++;
}

static void uart0_process_byte(uint8_t ch)
{
    uart0_last_byte = ch;
    uart0_rx_count++;

    if(rx_idx == 0)
    {
        if(ch == VISION_FRAME_HEAD1)
        {
            rx_buf[rx_idx++] = ch;
        }
        else
        {
            uart0_drop_count++;
        }
    }
    else if(rx_idx == 1)
    {
        if(ch == VISION_FRAME_HEAD2)
        {
            rx_buf[rx_idx++] = ch;
        }
        else
        {
            rx_idx = 0;
            uart0_drop_count++;
        }
    }
    else
    {
        rx_buf[rx_idx++] = ch;
        if(rx_idx >= RX_LEN)
        {
            for(uint8_t i = 0; i < RX_LEN; i++)
            {
                vision_frame_buf[i] = rx_buf[i];
            }
            uart0_raw_frame_count++;
            rx_finish = true;
            rx_idx = 0;
        }
    }
}

static void uart2_process_byte(uint8_t ch)
{
    uart2_last_byte = ch;
    uart2_rx_count++;

    if(imu_rx_idx == 0)
    {
        if(ch == FRAME_HEAD1)
        {
            imu_rx_buf[imu_rx_idx++] = ch;
        }
    }
    else if(imu_rx_idx == 1)
    {
        if(ch == FRAME_HEAD2)
        {
            imu_rx_buf[imu_rx_idx++] = ch;
        }
        else
        {
            imu_rx_idx = 0;
        }
    }
    else
    {
        imu_rx_buf[imu_rx_idx++] = ch;
        if(imu_rx_idx == IMU_RX_LEN)
        {
            imu_rx_finish = true;
            imu_rx_idx = 0;
        }
    }
}

static void parse_motor_position_frame(void)
{
    uint8_t sign;
    uint32_t abs_position;
    int32_t signed_position;
    float motor_deg_x10;

    sign = motor_position_rx_buf[2];
    abs_position = ((uint32_t)motor_position_rx_buf[3] << 24) |
        ((uint32_t)motor_position_rx_buf[4] << 16) |
        ((uint32_t)motor_position_rx_buf[5] << 8) |
        (uint32_t)motor_position_rx_buf[6];

    signed_position = (int32_t)abs_position;
    if(sign != 0U)
    {
        signed_position = -signed_position;
    }

    y_motor_position_raw = signed_position;
    motor_deg_x10 = ((float)signed_position * 3600.0f) /
        (Y_MOTOR_RAW_PER_REV * (float)GIMBAL_Y_GEAR_COMPENSATION);
    y_gimbal_position_deg_x10 = (int16_t)motor_deg_x10;
    y_position_valid = 1;
    y_position_age_ms = 0;
    y_position_frame_count++;
}

static void motor_uart1_process_byte(uint8_t ch)
{
    motor_uart1_last_byte = ch;
    motor_uart1_rx_count++;

    if(motor_position_rx_idx == 0U)
    {
        if(ch == Y_MOTOR_ADDR)
        {
            motor_position_rx_buf[motor_position_rx_idx++] = ch;
        }
        return;
    }

    if(motor_position_rx_idx == 1U)
    {
        if(ch == 0x36U)
        {
            motor_position_rx_buf[motor_position_rx_idx++] = ch;
        }
        else if(ch == 0x00U)
        {
            motor_position_rx_buf[motor_position_rx_idx++] = ch;
        }
        else
        {
            motor_position_rx_idx = 0;
            y_position_error_count++;
        }
        return;
    }

    if((motor_position_rx_idx == 2U) && (ch == 0x6BU))
    {
        motor_position_rx_idx = 0;
        return;
    }

    motor_position_rx_buf[motor_position_rx_idx++] = ch;

    if((motor_position_rx_idx == 4U) &&
        (motor_position_rx_buf[1] == 0x00U))
    {
        if((motor_position_rx_buf[2] == 0xEEU) &&
            (motor_position_rx_buf[3] == 0x6BU))
        {
            y_position_valid = 0;
        }
        y_position_error_count++;
        motor_position_rx_idx = 0;
        return;
    }

    if(motor_position_rx_idx >= MOTOR_POSITION_FRAME_LEN)
    {
        if((motor_position_rx_buf[1] == 0x36U) &&
            (motor_position_rx_buf[7] == 0x6BU))
        {
            parse_motor_position_frame();
        }
        else
        {
            y_position_error_count++;
        }
        motor_position_rx_idx = 0;
    }
}

static void uart0_poll_receive(void)
{
    uint8_t ch;
    uint8_t limit = 32;

    while((DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false) && (limit > 0U))
    {
        ch = DL_UART_Main_receiveData(UART_0_INST);
        uart0_process_byte(ch);
        uart0_poll_count++;
        limit--;
    }
}

static void motor_uart1_poll_receive(void)
{
    uint8_t ch;
    uint8_t limit = 32;

    while((DL_UART_Main_isRXFIFOEmpty(UART_1_INST) == false) && (limit > 0U))
    {
        ch = DL_UART_Main_receiveData(UART_1_INST);
        motor_uart1_process_byte(ch);
        limit--;
    }
}

static void y_position_query_task(void)
{
    static uint16_t query_interval_ms = 0;

    if(y_position_age_ms < Y_POSITION_TIMEOUT_MS)
    {
        y_position_age_ms += CONTROL_TIMER_PERIOD_MS;
    }
    else
    {
        y_position_valid = 0;
    }

    if(query_interval_ms < Y_POSITION_QUERY_INTERVAL_MS)
    {
        query_interval_ms += CONTROL_TIMER_PERIOD_MS;
        return;
    }

    query_interval_ms = 0;
    Motor_read_realtime_position_Y();
    y_position_query_count++;
}

static void uart2_poll_receive(void)
{
    uint8_t ch;
    uint8_t limit = 32;

    while((DL_UART_Main_isRXFIFOEmpty(UART_2_INST) == false) && (limit > 0U))
    {
        ch = DL_UART_Main_receiveData(UART_2_INST);
        uart2_process_byte(ch);
        uart2_poll_count++;
        limit--;
    }
}

static void tracking_control_task(void)
{
    bool control_updated = false;
    static uint16_t no_input_step_ms = 0;
    static uint8_t no_input_state = 0;

    uart0_poll_receive();
    motor_uart1_poll_receive();
    uart2_poll_receive();
    y_position_query_task();

    if(rx_finish)
    {
        parse_vision_data();
        rx_finish = false;
        control_updated = true;
        tracking_no_input_ms = 0;
        tracking_no_input_test_active = 0;
    }

    if(imu_rx_finish)
    {
        parse_imu_data();
        imu_rx_finish = false;
        control_updated = true;
        tracking_no_input_ms = 0;
        tracking_no_input_test_active = 0;
    }

    if(vision_timeout_check())
    {
        control_updated = true;
    }

    if(control_updated)
    {
        send_combined_gimbal_speed_if_changed();
    }
    else
    {
        (void)y_axis_soft_limit_guard_tick();
    }

#if TRACKING_NO_INPUT_TEST_ENABLE
    if((uart0_rx_count == 0U) && (uart2_rx_count == 0U))
    {
        if(tracking_no_input_ms < TRACKING_NO_INPUT_TIMEOUT_MS)
        {
            tracking_no_input_ms += CONTROL_TIMER_PERIOD_MS;
            return;
        }

        tracking_no_input_test_active = 1;
        if(no_input_step_ms < TRACKING_NO_INPUT_STEP_MS)
        {
            no_input_step_ms += CONTROL_TIMER_PERIOD_MS;
            return;
        }
        no_input_step_ms = 0;

        if(no_input_state == 0U)
        {
            Motor_setspeed_X(TRACKING_NO_INPUT_TEST_SPEED);
            no_input_state = 1U;
        }
        else if(no_input_state == 1U)
        {
            Motor_setspeed_X(-TRACKING_NO_INPUT_TEST_SPEED);
            no_input_state = 2U;
        }
        else
        {
            Motor_setspeed_X(0);
            no_input_state = 0U;
        }
    }
    else
    {
        tracking_no_input_ms = 0;
        tracking_no_input_test_active = 0;
    }
#endif
}

static void motor_test_task(void)
{
    static uint16_t step_ms = 0;
    static uint8_t state = 0;

    if(step_ms < MOTOR_TEST_STEP_MS)
    {
        step_ms += CONTROL_TIMER_PERIOD_MS;
        return;
    }
    step_ms = 0;

    switch(state)
    {
        case 0:
            Motor_setspeed_XY(MOTOR_TEST_SPEED, 0);
            break;

        case 1:
            Motor_setspeed_XY(0, 0);
            break;

        case 2:
            Motor_setspeed_XY(-MOTOR_TEST_SPEED, 0);
            break;

        case 3:
            Motor_setspeed_XY(0, MOTOR_TEST_SPEED);
            break;

        case 4:
            Motor_setspeed_XY(0, 0);
            break;

        case 5:
            Motor_setspeed_XY(0, -MOTOR_TEST_SPEED);
            break;

        default:
            Motor_setspeed_XY(0, 0);
            state = 0;
            return;
    }

    state++;
    if(state > 5U)
    {
        state = 0;
    }
    motor_test_step_count++;
}



void parse_imu_data(void)
{
    uint8_t sum_calc = 0;
    uint8_t sum_recv = imu_rx_buf[10];

    // 计算校验和（按协议：所有字节相加）
    for(int i=0; i<10; i++)
    {
        sum_calc += imu_rx_buf[i];
    }

    // 校验失败直接丢弃
    if(sum_calc != sum_recv)
    {
        imu_checksum_error_count++;
        return;
    }

    // 解析角度（按公式转换）
    roll  = (int16_t)((imu_rx_buf[3] << 8) | imu_rx_buf[2]);
    pitch = (int16_t)((imu_rx_buf[5] << 8) | imu_rx_buf[4]);
    yaw   = (int16_t)((imu_rx_buf[7] << 8) | imu_rx_buf[6]);

    // 转换为角度（公式：/32768 * 180°）
    roll_deg  = (float)roll  / 32768.0f * 180.0f;
    pitch_deg = (float)pitch / 32768.0f * 180.0f;
    yaw_deg   = (float)yaw   / 32768.0f * 180.0f;
    imu_frame_count++;

    // 这里可以直接用角度值控制电机
    // 例：根据pitch控制X轴速度
#if GIMBAL_X_YAW_STABILIZE_ENABLE
    gimbal_x_update_yaw_compensation(yaw_deg);
#else
    gyro_speed_x = 0;
    gimbal_x_yaw_error_deg = 0.0f;
#endif
}


void UART_0_INST_IRQHandler(void)
{
    uint8_t ch;
    while(DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false)
    {
        ch = DL_UART_Main_receiveData(UART_0_INST);
        uart0_process_byte(ch);
    }
}

void UART_0_INST_IRQHandler_unused(void)
{
    uint8_t ch;
    while(DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false)
    {
        ch = DL_UART_Main_receiveData(UART_0_INST);

        // 存入数组
        if(rx_idx < RX_LEN)
        {
            rx_buf[rx_idx++] = ch;
        }

        // 收满4字节 → 完成
        if(rx_idx >= RX_LEN)
        {
            rx_finish = true;
            rx_idx = 0;
        }
    }
}

void UART_1_INST_IRQHandler(void)
{
    uint8_t ch;
    uint16_t timeout = 1000;

    while((DL_UART_Main_isRXFIFOEmpty(UART_1_INST) == false) && timeout--)
    {
        ch = DL_UART_Main_receiveData(UART_1_INST);
        motor_uart1_process_byte(ch);
    }
}

// void UART_2_INST_IRQHandler(void)
// {
//     uint8_t ch;
//     uint8_t test= 0x01;
//     while(DL_UART_Main_isRXFIFOEmpty(UART_2_INST) == false)
//     {
//         ch = DL_UART_Main_receiveData(UART_2_INST);
//         DL_UART_Main_transmitData(UART_2_INST,test);
//     }
// }

void UART_2_INST_IRQHandler(void)
{
    uint8_t ch;
    uint16_t timeout = 1000; // 防止卡死的超时计数器

    while(DL_UART_Main_isRXFIFOEmpty(UART_2_INST) == false && timeout--)
    {
        ch = DL_UART_Main_receiveData(UART_2_INST);
        uart2_process_byte(ch);
    }
}

void CONTROL_TIMER_INST_IRQHandler(void)
{
    switch(DL_Timer_getPendingInterrupt(CONTROL_TIMER_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            control_timer_tick_count++;

#if !X_MOTOR_STANDALONE_TEST
            if(button_pressed(Button_Button3_PIN))
            {
                app_mode = APP_MODE_STOP;
                Laser_Off();
                stop_all_motion();
            }

            switch(app_mode)
            {
                case APP_MODE_TRACKING:
                    tracking_control_task();
                    break;

                case APP_MODE_ECG:
                    ecg_draw_task();
                    break;

                case APP_MODE_MOTOR_TEST:
                    motor_test_task();
                    break;

                case APP_MODE_STOP:
                case APP_MODE_NONE:
                    break;
            }
#endif
            break;

        default:
            break;
    }
}

int main(void)
{
    SYSCFG_DL_init();
    Laser_On();
    app_mode = select_startup_mode();
    wait_buttons_released();

    if(app_mode == APP_MODE_TRACKING)
    {
        Laser_On();
    }
    else if(app_mode == APP_MODE_ECG)
    {
        ecg_draw_reset();
        Laser_On();
    }
    else if(app_mode == APP_MODE_STOP)
    {
        stop_all_motion();
    }

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    NVIC_ClearPendingIRQ(CONTROL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);
    DL_Timer_startCounter(CONTROL_TIMER_INST);

    while (1) 
    {
#if X_MOTOR_STANDALONE_TEST
        x_motor_standalone_test();
#else
        __WFI();
#endif
    }
}

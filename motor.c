#include "motor.h"

#define MOTOR_X_ADDR    0x01
#define MOTOR_Y_ADDR    0x02
#define MOTOR_FRAME_END 0x6B
#define MOTOR_CMD_GAP_CYCLES 32000
#define MOTOR_POS_CMD_LEN 13
#define MOTOR_UART_BUSY_TIMEOUT 30000U

static volatile uint8_t tx_data_forspeed[8];
static volatile uint8_t tx_data_forposition[MOTOR_POS_CMD_LEN];
volatile int16_t motor_last_x_speed = 0;
volatile int16_t motor_last_y_speed = 0;
volatile uint8_t motor_last_tx_data[MOTOR_POS_CMD_LEN] = {0};
volatile uint8_t motor_last_tx_len = 0;
volatile uint32_t motor_tx_count = 0;
volatile uint32_t motor_tx_timeout_count = 0;
volatile int32_t motor_last_x_position_pulses = 0;
volatile int32_t motor_last_y_position_pulses = 0;

static void Motor_build_speed_cmd(uint8_t addr, int16_t speed,
    volatile uint8_t *data);
static void Motor_build_position_cmd(uint8_t addr, int32_t pulses,
    uint16_t speed, uint8_t accel, uint8_t mode, uint8_t sync,
    volatile uint8_t *data);

static void Motor_send_bytes(volatile uint8_t *data, uint8_t len)
{
    uint32_t timeout;

    motor_last_tx_len = len;
    for(uint8_t i = 0; i < len; i++)
    {
        if(i < MOTOR_POS_CMD_LEN)
        {
            motor_last_tx_data[i] = data[i];
        }
        DL_UART_Main_transmitData(UART_1_INST, data[i]);
        timeout = MOTOR_UART_BUSY_TIMEOUT;
        while((DL_UART_isBusy(UART_1_INST) == true) && (timeout > 0U))
        {
            timeout--;
        }
        if(timeout == 0U)
        {
            motor_tx_timeout_count++;
            break;
        }
    }
    motor_tx_count++;
}

static void Motor_send_speed_cmd(uint8_t addr, int16_t speed)
{
    Motor_build_speed_cmd(addr, speed, tx_data_forspeed);
    Motor_send_bytes(tx_data_forspeed, 8);
}

static void Motor_build_speed_cmd(uint8_t addr, int16_t speed,
    volatile uint8_t *data)
{
    uint8_t dir;
    uint16_t abs_speed;

    if(speed >= 0)
    {
        dir = 0x01;
        abs_speed = (uint16_t)speed;
    }
    else
    {
        dir = 0x00;
        abs_speed = (uint16_t)(-speed);
    }

    if(abs_speed > MOTOR_SPEED_MAX)
    {
        abs_speed = MOTOR_SPEED_MAX;
    }

    data[0] = addr;
    data[1] = 0xF6;
    data[2] = dir;
    data[3] = (uint8_t)((abs_speed >> 8) & 0xFF);
    data[4] = (uint8_t)(abs_speed & 0xFF);
    data[5] = MOTOR_ACCEL_DEFAULT;
    data[6] = MOTOR_SYNC_DISABLE;
    data[7] = MOTOR_FRAME_END;
}

static void Motor_build_position_cmd(uint8_t addr, int32_t pulses,
    uint16_t speed, uint8_t accel, uint8_t mode, uint8_t sync,
    volatile uint8_t *data)
{
    uint8_t dir;
    uint32_t abs_pulses;

    if(pulses >= 0)
    {
        dir = 0x01;
        abs_pulses = (uint32_t)pulses;
    }
    else
    {
        dir = 0x00;
        abs_pulses = (uint32_t)(-pulses);
    }

    if(speed > MOTOR_SPEED_MAX)
    {
        speed = MOTOR_SPEED_MAX;
    }

    data[0] = addr;
    data[1] = 0xFD;
    data[2] = dir;
    data[3] = (uint8_t)((speed >> 8) & 0xFF);
    data[4] = (uint8_t)(speed & 0xFF);
    data[5] = accel;
    data[6] = (uint8_t)((abs_pulses >> 24) & 0xFF);
    data[7] = (uint8_t)((abs_pulses >> 16) & 0xFF);
    data[8] = (uint8_t)((abs_pulses >> 8) & 0xFF);
    data[9] = (uint8_t)(abs_pulses & 0xFF);
    data[10] = mode;
    data[11] = sync;
    data[12] = MOTOR_FRAME_END;
}

static void Motor_read_target_position_cmd(uint8_t addr)
{
    volatile uint8_t tx_data[3];

    tx_data[0] = addr;
    tx_data[1] = 0x33;
    tx_data[2] = MOTOR_FRAME_END;

    Motor_send_bytes(tx_data, 3);
}

static void Motor_read_realtime_position_cmd(uint8_t addr)
{
    volatile uint8_t tx_data[3];

    tx_data[0] = addr;
    tx_data[1] = 0x36;
    tx_data[2] = MOTOR_FRAME_END;

    Motor_send_bytes(tx_data, 3);
}

void Motor_setspeed_X(int16_t speed)
{
    motor_last_x_speed = speed;
    Motor_send_speed_cmd(MOTOR_X_ADDR, speed);
}

void Motor_setspeed_Y(int16_t speed)
{
    motor_last_y_speed = speed;
    Motor_send_speed_cmd(MOTOR_Y_ADDR, speed);
}

void Motor_setspeed_XY(int16_t speed_x, int16_t speed_y)
{
    motor_last_x_speed = speed_x;
    Motor_build_speed_cmd(MOTOR_X_ADDR, speed_x, tx_data_forspeed);
    Motor_send_bytes(tx_data_forspeed, 8);

    delay_cycles(MOTOR_CMD_GAP_CYCLES);

    motor_last_y_speed = speed_y;
    Motor_build_speed_cmd(MOTOR_Y_ADDR, speed_y, tx_data_forspeed);
    Motor_send_bytes(tx_data_forspeed, 8);
}

static void Motor_move_relative_cmd(uint8_t addr, int32_t pulses,
    uint16_t speed, uint8_t accel)
{
    Motor_build_position_cmd(addr, pulses, speed, accel,
        MOTOR_POSITION_RELATIVE, MOTOR_SYNC_DISABLE, tx_data_forposition);
    Motor_send_bytes(tx_data_forposition, MOTOR_POS_CMD_LEN);
}

void Motor_read_target_position_X(void)
{
    Motor_read_target_position_cmd(MOTOR_X_ADDR);
}

void Motor_read_target_position_Y(void)
{
    Motor_read_target_position_cmd(MOTOR_Y_ADDR);
}

void Motor_read_realtime_position_X(void)
{
    Motor_read_realtime_position_cmd(MOTOR_X_ADDR);
}

void Motor_read_realtime_position_Y(void)
{
    Motor_read_realtime_position_cmd(MOTOR_Y_ADDR);
}

void Motor_move_relative_X(int32_t pulses, uint16_t speed, uint8_t accel)
{
    motor_last_x_position_pulses = pulses;
    Motor_move_relative_cmd(MOTOR_X_ADDR, pulses, speed, accel);
}

void Motor_move_relative_Y(int32_t pulses, uint16_t speed, uint8_t accel)
{
    motor_last_y_position_pulses = pulses;
    Motor_move_relative_cmd(MOTOR_Y_ADDR, pulses, speed, accel);
}

void Motor_move_relative_XY(int32_t pulses_x, int32_t pulses_y, uint16_t speed,
    uint8_t accel)
{
    motor_last_x_position_pulses = pulses_x;
    Motor_move_relative_cmd(MOTOR_X_ADDR, pulses_x, speed, accel);

    delay_cycles(MOTOR_CMD_GAP_CYCLES);

    motor_last_y_position_pulses = pulses_y;
    Motor_move_relative_cmd(MOTOR_Y_ADDR, pulses_y, speed, accel);
}

static void Motor_stop_cmd(uint8_t addr)
{
    volatile uint8_t tx_data[5];

    tx_data[0] = addr;
    tx_data[1] = 0xFE;
    tx_data[2] = 0x98;
    tx_data[3] = MOTOR_SYNC_DISABLE;
    tx_data[4] = MOTOR_FRAME_END;

    Motor_send_bytes(tx_data, 5);
}

void Motor_stop_X(void)
{
    motor_last_x_speed = 0;
    Motor_stop_cmd(MOTOR_X_ADDR);
}

void Motor_stop_Y(void)
{
    motor_last_y_speed = 0;
    Motor_stop_cmd(MOTOR_Y_ADDR);
}

void Motor_stop_XY(void)
{
    Motor_stop_X();
    delay_cycles(MOTOR_CMD_GAP_CYCLES);
    Motor_stop_Y();
}

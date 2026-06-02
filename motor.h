#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "ti_msp_dl_config.h"

#define MOTOR_SPEED_MAX         1500
#define MOTOR_ACCEL_DEFAULT     10
#define MOTOR_SYNC_DISABLE      0
#define MOTOR_POSITION_RELATIVE 0
#define MOTOR_POSITION_ABSOLUTE 1

extern volatile int16_t motor_last_x_speed;
extern volatile int16_t motor_last_y_speed;
extern volatile uint8_t motor_last_tx_data[13];
extern volatile uint8_t motor_last_tx_len;
extern volatile uint32_t motor_tx_count;
extern volatile uint32_t motor_tx_timeout_count;
extern volatile int32_t motor_last_x_position_pulses;
extern volatile int32_t motor_last_y_position_pulses;

void Motor_setspeed_X(int16_t speed);
void Motor_setspeed_Y(int16_t speed);
void Motor_setspeed_XY(int16_t speed_x, int16_t speed_y);
void Motor_move_relative_X(int32_t pulses, uint16_t speed, uint8_t accel);
void Motor_move_relative_Y(int32_t pulses, uint16_t speed, uint8_t accel);
void Motor_move_relative_XY(int32_t pulses_x, int32_t pulses_y, uint16_t speed,
    uint8_t accel);
void Motor_stop_X(void);
void Motor_stop_Y(void);
void Motor_stop_XY(void);
void Motor_read_target_position_X(void);
void Motor_read_target_position_Y(void);
void Motor_read_realtime_position_X(void);
void Motor_read_realtime_position_Y(void);

#endif

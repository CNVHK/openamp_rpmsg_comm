#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdint.h>

typedef enum {
    BALANCE_STATE_DISABLED = 0,
    BALANCE_STATE_ARMING = 1,
    BALANCE_STATE_ACTIVE = 2,
    BALANCE_STATE_FAULT = 3
} BalanceState;

enum {
    BALANCE_FAULT_NONE = 0,
    BALANCE_FAULT_IMU = 1U << 0,
    BALANCE_FAULT_LEFT_MOTOR = 1U << 1,
    BALANCE_FAULT_RIGHT_MOTOR = 1U << 2,
    BALANCE_FAULT_CAN = 1U << 3,
    BALANCE_FAULT_FALL = 1U << 4,
    BALANCE_FAULT_OVERRUN = 1U << 5,
    BALANCE_FAULT_ARM_TIMEOUT = 1U << 6,
    BALANCE_FAULT_SPEED = 1U << 7,
    BALANCE_FAULT_ARM_CONDITION = BALANCE_FAULT_SPEED,
    BALANCE_FAULT_CONFIG = BALANCE_FAULT_SPEED
};

typedef struct {
    uint8_t state;
    uint8_t fault;
    uint16_t control_hz;
    float pitch_rad;
    float pitch_rate_rad_s;
    float wheel_position_m;
    float wheel_velocity_m_s;
    float left_torque_nm;
    float right_torque_nm;
    uint32_t loop_count;
} BalanceTelemetry;

int balance_control_init(void);
int balance_control_enable(void);
void balance_control_disable(void);
void balance_control_poll(void);
const BalanceTelemetry *balance_control_get_telemetry(void);

#endif

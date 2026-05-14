#ifndef JC4010_CAN_H
#define JC4010_CAN_H

#include <stdint.h>
#include <stddef.h>

#define JC4010_CAN_DLC 8

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[JC4010_CAN_DLC];
} Jc4010CanFrame;

typedef struct {
    float position_deg;
    float speed_rpm;
    float current_a;
} Jc4010MotorState;

typedef enum {
    JC4010_CMD_HEARTBEAT = 1,
    JC4010_CMD_ENABLE = 2,
    JC4010_CMD_ZERO_POSITION = 3,
    JC4010_CMD_SET_MODE = 4,
    JC4010_CMD_PVT = 5,
    JC4010_CMD_SAFE_STOP = 6
} Jc4010Command;

void jc4010_build_cmd(uint8_t motor_id, uint8_t cmd_word, uint16_t reg_addr,
                      int32_t value, Jc4010CanFrame *frame);
void jc4010_build_pvt(uint8_t motor_id, int32_t position_x100_deg,
                      uint16_t speed_rpm, uint8_t torque_percent,
                      Jc4010CanFrame *frame);
void jc4010_build_enable(uint8_t motor_id, Jc4010CanFrame *frame);
void jc4010_build_set_mode(uint8_t motor_id, uint16_t mode, Jc4010CanFrame *frame);
void jc4010_build_zero_position(uint8_t motor_id, Jc4010CanFrame *frame);
void jc4010_build_safe_stop(uint8_t motor_id, Jc4010CanFrame *frame);
int jc4010_parse_status(uint32_t can_id, const uint8_t data[8],
                        Jc4010MotorState *motor1, Jc4010MotorState *motor2);

#endif


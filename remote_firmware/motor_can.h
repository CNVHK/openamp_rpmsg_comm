#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include <stdint.h>

#define MOTOR_CAN_DLC 8

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[MOTOR_CAN_DLC];
} MotorCanFrame;

void motor_build_pvt(uint8_t motor_id, int32_t position_x100_deg,
                     uint16_t speed_rpm, uint8_t torque_percent,
                     MotorCanFrame *frame);
void motor_build_enable(uint8_t motor_id, MotorCanFrame *frame);
void motor_build_set_mode(uint8_t motor_id, uint16_t mode, MotorCanFrame *frame);
void motor_build_set_origin(uint8_t motor_id, MotorCanFrame *frame);
void motor_build_set_temporary_origin(uint8_t motor_id, MotorCanFrame *frame);
void motor_build_safe_stop(uint8_t motor_id, MotorCanFrame *frame);

#endif

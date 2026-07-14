#include "motor_can.h"

static void motor_clear_frame(uint8_t motor_id, MotorCanFrame *frame)
{
    frame->id = 0x600u + motor_id;
    frame->dlc = MOTOR_CAN_DLC;
    for (int i = 0; i < MOTOR_CAN_DLC; ++i) {
        frame->data[i] = 0;
    }
}

static void motor_build_register_write(uint8_t motor_id, uint16_t reg_addr,
                                       uint16_t value, MotorCanFrame *frame)
{
    motor_clear_frame(motor_id, frame);

    frame->data[0] = 0x2B;
    frame->data[1] = (uint8_t)(reg_addr >> 8);
    frame->data[2] = (uint8_t)(reg_addr & 0xff);
    frame->data[3] = 0x00;

    frame->data[4] = (uint8_t)(value >> 8);
    frame->data[5] = (uint8_t)(value & 0xff);
}

void motor_build_pvt(uint8_t motor_id, int32_t position_x100_deg,
                     uint16_t speed_rpm, uint8_t torque_percent,
                     MotorCanFrame *frame)
{
    motor_clear_frame(motor_id, frame);

    frame->data[0] = 0x25;
    frame->data[1] = (uint8_t)(position_x100_deg >> 24);
    frame->data[2] = (uint8_t)(position_x100_deg >> 16);
    frame->data[3] = (uint8_t)(position_x100_deg >> 8);
    frame->data[4] = (uint8_t)(position_x100_deg & 0xff);
    frame->data[5] = (uint8_t)(speed_rpm >> 8);
    frame->data[6] = (uint8_t)(speed_rpm & 0xff);
    frame->data[7] = torque_percent;
}

void motor_build_enable(uint8_t motor_id, MotorCanFrame *frame)
{
    motor_build_register_write(motor_id, 0x00A2, 1, frame);
}

void motor_build_set_mode(uint8_t motor_id, uint16_t mode, MotorCanFrame *frame)
{
    motor_build_register_write(motor_id, 0x0060, mode, frame);
}

void motor_build_set_origin(uint8_t motor_id, MotorCanFrame *frame)
{
    motor_build_register_write(motor_id, 0x00A6, 1, frame);
}

void motor_build_set_temporary_origin(uint8_t motor_id, MotorCanFrame *frame)
{
    motor_build_register_write(motor_id, 0x00A7, 1, frame);
}

void motor_build_safe_stop(uint8_t motor_id, MotorCanFrame *frame)
{
    motor_build_pvt(motor_id, 0, 0, 0, frame);
}
